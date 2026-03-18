#include <yaml-cpp/yaml.h>//YAML 配置解析器

#include <cmath>//数学库
#include <memory>//动态内存管理头文件
#include <mutex>//互斥锁
#include <shared_mutex>//共享互斥锁

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>//用于将控制指令序列化后发布至指定的 Topic
#include <unitree/robot/channel/channel_subscriber.hpp>//用于订阅机器人反馈的状态并触发回调函数

// IDL 接口定义语言
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
using namespace unitree::robot::b2; // 访问运动切换客户端相关的类

static const std::string HG_CMD_TOPIC = "rt/lowcmd";// 底层指令发布的 DDS 话题名称
static const std::string HG_STATE_TOPIC = "rt/lowstate";// 底层状态订阅的 DDS 话题名称

using namespace unitree::common; // 访问 SDK 通用工具类 
using namespace unitree::robot; // 访问机器人底层通信相关的类

const int G1_NUM_MOTOR = 29; //29个电机数

template <typename T>
class DataBuffer {//这个类的设计保证了：你拿到的每一包数据，要么是完整的上一时刻状态，要么是完整的当前时刻状态，绝不会是中间态。
 public:
  void SetData(const T &newData) {//写入操作 - 排他锁
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = std::make_shared<T>(newData);
  }

  std::shared_ptr<const T> GetData() {//读取操作 - 共享锁
    std::shared_lock<std::shared_mutex> lock(mutex);
    return data ? data : nullptr;
  }

  void Clear() {//清理操作
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = nullptr;
  }

 private:
  std::shared_ptr<T> data;
  std::shared_mutex mutex;
};

struct ImuState {//惯性测量单元状态
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
};

struct MotorCommand {//电机控制指令。这是程序发送给硬件的“任务单”。
  std::array<float, G1_NUM_MOTOR> q_target = {};
  std::array<float, G1_NUM_MOTOR> dq_target = {};
  std::array<float, G1_NUM_MOTOR> kp = {};
  std::array<float, G1_NUM_MOTOR> kd = {};
  std::array<float, G1_NUM_MOTOR> tau_ff = {};
};

struct MotorState {//电机实时状态。这是机器人硬件回传给程序的“反馈单”。
  std::array<float, G1_NUM_MOTOR> q = {};
  std::array<float, G1_NUM_MOTOR> dq = {};
};

enum MotorType { GearboxS = 0, GearboxM = 1, GearboxL = 2 };//三种减速器。小中大

std::array<MotorType, G1_NUM_MOTOR> G1MotorType{//按照索引顺序记录了机器人全身每一个关节对应的电机型号。
    // clang-format off
    // legs
    GearboxM, GearboxM, GearboxM, GearboxL, GearboxS, GearboxS,
    GearboxM, GearboxM, GearboxM, GearboxL, GearboxS, GearboxS,
    // waist
    GearboxM, GearboxS, GearboxS,
    // arms
    GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS,
    GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS, GearboxS
    // clang-format on
};

enum PRorAB { PR = 0, AB = 1 };//运动学解算模式

enum G1JointValidIndex {//关节索引地图
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20,
  LeftWristYaw = 21,
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27,
  RightWristYaw = 28
};

inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else
        CRC32 <<= 1;
      if (data & xbit) CRC32 ^= dwPolynomial;

      xbit >>= 1;
    }
  }
  return CRC32;
};

float GetMotorKp(MotorType type) {//刚度分配器
  switch (type) {
    case GearboxS:
      return 40;
    case GearboxM:
      return 40;
    case GearboxL:
      return 100;
    default:
      return 0;
  }
}

float GetMotorKd(MotorType type) {//阻尼分配器
  switch (type) {
    case GearboxS:
      return 1;
    case GearboxM:
      return 1;
    case GearboxL:
      return 1;
    default:
      return 0;
  }
}

class G1Example {
 private:
  double time_;// 运行时间计数器，记录程序启动了多久
  double control_dt_;  // [2ms]控制周期
  double duration_;    // [3 s]动作时长基准
  PRorAB mode_;// 控制模式，决定是 PR 还是 AB 解算
  uint8_t mode_machine_;// 状态机标志，记录机器人当前的运行模式
  std::vector<std::vector<double>> frames_data_;//二维动态数组。离线轨迹存储

  DataBuffer<MotorState> motor_state_buffer_;// 存放电机回传的状态
  DataBuffer<MotorCommand> motor_command_buffer_;// 存放待发送的电机指令
  DataBuffer<ImuState> imu_state_buffer_;// 存放惯导反馈的姿态

  ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;// DDS 发布者：负责把指令“广播”给机器人
  ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;// DDS 订阅者：负责从机器人那里“收听”状态
  ThreadPtr command_writer_ptr_, control_thread_ptr_;// 线程句柄：管理程序的后台运行任务

  std::shared_ptr<MotionSwitcherClient> msc;// 运动模式切换客户端：用于控制机器人的高/低层模式切换

 public:
  G1Example(std::string networkInterface)//构造函数
      : time_(0.0),//初始化列表
        control_dt_(0.002),
        duration_(3.0),
        mode_(PR),
        mode_machine_(0) {
    ChannelFactory::Instance()->Init(0, networkInterface);//打开网口，激活通讯引擎

    msc.reset(new MotionSwitcherClient());//创建模式切换遥控器。实例化一个专门用来和机器人“系统管家”对话的工具
    msc->SetTimeout(5.0F);//设置耐心极限
    msc->Init();//连接管理中心

    /*Shut down  motion control-related service*/ //强制停用官方运动服务
    while(queryMotionStatus())
    {
        std::cout << "Try to deactivate the motion control-related service." << std::endl;
        int32_t ret = msc->ReleaseMode(); 
        if (ret == 0) {//说明发送成功
            std::cout << "ReleaseMode succeeded." << std::endl;
        } else {
            std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
        }
        sleep(5);
    }

    loadBehaviorLibrary("motion");//加载动作剧本。会根据宏定义的路径寻找 motion.seq 文件，并将其反序列化到内存变量 frames_data_ 中

    // create publisher发布者
    lowcmd_publisher_.reset(//初始化智能指针
        new ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();//正式在网络协议栈中注册这个发布频道

    // create subscriber订阅者
    lowstate_subscriber_.reset(
        new ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
            HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(
        std::bind(&G1Example::LowStateHandler, this, std::placeholders::_1), 1);//回调函数

    // create threads
    command_writer_ptr_ =
        CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000,//创建一个定时循环线程,线程的名字是command_writer，
                                &G1Example::LowCommandWriter, this);//成员函数指针，入口函数，告诉线程：“你每次醒来时，就去跑这个函数”。
    control_thread_ptr_ = CreateRecurrentThreadEx(
        "control", UT_CPU_ID_NONE, 2000, &G1Example::Control, this);
  }

  void loadBehaviorLibrary(std::string behavior_name) {//从磁盘读取离线轨迹文件（.seq），并将其解析为程序可以高效访问的内存矩阵
    std::string resource_dir = BLIB_DIR;// 获取文件夹路径
    YAML::Node motion = YAML::LoadFile(resource_dir + behavior_name + ".seq");//将该文本文件加载进内存，形成一个类似字典的树状结构 motion

    std::string content = motion["components"][1]["content"].as<std::string>();//访问特定节点，类型转换成string。获取这段轨迹的描述和涉及的部件数量。
    int num_parts = motion["components"][1]["num_parts"].as<int>();
    std::cout << "BehaviorName: " << behavior_name + ".seq\n";// 读取这段动作的名字
    std::cout << content << " with " << num_parts << "\n";// 读取它包含多少个关节

    auto frames = motion["components"][1]["frames"];// 找到文件中存放所有动作帧的部分

    for (const auto &frame : frames) {// 遍历每一时刻（每一行）
      std::vector<double> frame_data;// 准备一个小盒子装这一时刻的关节角度
      for (const auto &element : frame) {// 遍历该时刻下的所有关节角度（每一列）
        frame_data.push_back(element.as<double>());// 把文本“0.5”变成数字 0.5 存进去
      }
      frames_data_.push_back(frame_data);// 存入类的成员变量矩阵。frames_data_ 现在就像一张表格。每一行代表一个时间点的全身（或双臂）姿态，每一列代表某个具体电机的目标角度。
    }

    std::cout << frames_data_.size() << " knots with " << frames_data_[0].size()
              << " DOF\n";
  }

  void ReportRPY() {//从内存中提取并打印机器人当前的RPY角度
    const std::shared_ptr<const ImuState> imu_tmp_ptr =//指向常量的智能指针，
        imu_state_buffer_.GetData();
    if (imu_tmp_ptr) {//空指针检查
      std::cout << "rpy: [" << imu_tmp_ptr->rpy.at(0) << ", "
                << imu_tmp_ptr->rpy.at(1) << ", " << imu_tmp_ptr->rpy.at(2)
                << "]" << std::endl;//访问结构体ImuState里的rpy数组。
    }
  }

  void LowStateHandler(const void *message) {//底层通讯传过来的原始内存地址
    unitree_hg::msg::dds_::LowState_ low_state =
        *(const unitree_hg::msg::dds_::LowState_ *)message;

    if (low_state.crc() !=//CRC 循环冗余校验
        Crc32Core((uint32_t *)&low_state,
                  (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
      std::cout << "low_state CRC Error" << std::endl;
      return;
    }

    // get motor state
    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      ms_tmp.q.at(i) = low_state.motor_state()[i].q();// 获取实际角度
      ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();// 获取实际速度
    }
    motor_state_buffer_.SetData(ms_tmp);// 存入带锁的缓冲区

    // get imu state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();
    imu_state_buffer_.SetData(imu_tmp);

    // update mode machine 记录机器人当前的运行模式
    if (mode_machine_ != low_state.mode_machine()) {
      if (mode_machine_ == 0)
        std::cout << "G1 type: " << unsigned(low_state.mode_machine())
                  << std::endl;
      mode_machine_ = low_state.mode_machine();
    }
  }

  void LowCommandWriter() {//把计算出来的指令打包，贴上防伪标签（CRC），然后通过网线准确地传达给每一个电机
    unitree_hg::msg::dds_::LowCmd_ dds_low_command;//在内存中开辟空间，创建一个指令结构体对象
    dds_low_command.mode_pr() = mode_;
    dds_low_command.mode_machine() = mode_machine_;

    const std::shared_ptr<const MotorCommand> mc =//拿出计算好的最新指令mc
        motor_command_buffer_.GetData();
    if (mc) {//指令转换与填充
      for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
        dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable使能电机
        dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);//前馈力矩
        dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);// 目标角度
        dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);// 目标速度
        dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);// 比例增益（刚度）
        dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);// 微分增益（阻尼）
      }

      dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command,//贴上安全标签
                                        (sizeof(dds_low_command) >> 2) - 1);
      lowcmd_publisher_->Write(dds_low_command);//发出
    }
  }

  void Control() {
    MotorCommand motor_command_tmp;// 准备存计算结果的临时盒子
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();// 拿到最新的关节角度

    if (ms) {
      time_ += control_dt_;//每次进来，时间指针就往前推 2ms
      if (time_ < duration_) {
        // [Stage 1]: set robot to zero posture 平滑归零 (Soft Start / Zero Posture)
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {//在 3 秒内，让关节从当前角度平滑地移动到 0 度。ratio 越接近 1，就离 0 度越近。
          double ratio = std::clamp(time_ / duration_, 0.0, 1.0);

          double q_des = 0;
          motor_command_tmp.tau_ff.at(i) = 0.0;
          motor_command_tmp.q_target.at(i) =
              (q_des - ms->q.at(i)) * ratio + ms->q.at(i);
          motor_command_tmp.dq_target.at(i) = 0.0;
          motor_command_tmp.kp.at(i) = GetMotorKp(G1MotorType[i]);
          motor_command_tmp.kd.at(i) = GetMotorKd(G1MotorType[i]);
        }
      } else {
        // [Stage 2]: tracking the offline trajectory 执行剧本
        size_t frame_index = (size_t)((time_ - duration_) / control_dt_);//计算当前时间对应 frames_data_ 矩阵里的哪一行。
        if (frame_index >= frames_data_.size()) {//如果播放完毕
          frame_index = frames_data_.size() - 1;//强行停在最后一行
          time_ = 0.0;  // RESET
        }

        if (frame_index % 100 == 0)
          std::cout << "Frame Index: " << frame_index << std::endl;

        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
          size_t index_in_frame = i - LeftShoulderPitch;
          motor_command_tmp.q_target.at(i) =
              (i >= LeftShoulderPitch)//如果关节是肩膀及以下，就去查剧本里的角度。如果是下半身关节，保持 0 度
                  ? frames_data_[frame_index][index_in_frame]
                  : 0.0;
          motor_command_tmp.dq_target.at(i) = 0.0;
          motor_command_tmp.tau_ff.at(i) = 0.0;
          motor_command_tmp.kp.at(i) = GetMotorKp(G1MotorType[i]);
          motor_command_tmp.kd.at(i) = GetMotorKd(G1MotorType[i]);
        }
      }

      motor_command_buffer_.SetData(motor_command_tmp);
    }
  }

  std::string queryServiceName(std::string form,std::string name)//逻辑映射表，根据输入的机器人“形态”和“版本名称”，返回对应的服务名称
  {
      if(form == "0")
      {
          if(name == "normal" ) return "sport_mode"; 
          if(name == "ai" ) return "ai_sport"; 
          if(name == "advanced" ) return "advanced_sport"; 
      }
      else
      {
          if(name == "ai-w" ) return "wheeled_sport(go2W)"; 
          if(name == "normal-w" ) return "wheeled_sport(b2W)";
      }
      return "";
  }

  int queryMotionStatus()//运动状态问讯
  {
      std::string robotForm,motionName;
      int motionStatus;
      int32_t ret = msc->CheckMode(robotForm,motionName);//向机器人发送一个请求，询问当前的硬件状态
      if (ret == 0) {
          std::cout << "CheckMode succeeded." << std::endl;
      } else {
          std::cout << "CheckMode failed. Error code: " << ret << std::endl;
      }
      if(motionName.empty())
      {
          std::cout << "The motion control-related service is deactivated." << std::endl;//没有运行
          motionStatus = 0;
      }
      else
      {
          std::string serviceName = queryServiceName(robotForm,motionName);
          std::cout << "Service: "<< serviceName<< " is activate" << std::endl;
          motionStatus = 1;
      }
      return motionStatus;
  }
};

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_dual_arm_example network_interface_name"
              << std::endl;
    exit(0);
  }
  std::string networkInterface = argv[1];
  G1Example custom(networkInterface);

  while (true) sleep(10);

  return 0;
}

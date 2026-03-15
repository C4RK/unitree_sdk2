#include <cmath>          // 数学库：计算正弦波（sin/cos）让脚踝平滑摆动
#include <memory>         // 智能指针：管理内存，防止内存泄漏
#include <mutex>          // 互斥锁：防止多个线程同时改写电机指令
#include <shared_mutex>   // 读写锁：提高效率，允许多个线程同时读取状态，但只允许一个写入

#include "gamepad.hpp"    // 游戏手柄控制：允许你用手柄遥控脚踝的动作

// DDS 这是宇树机器人通信的核心。可以把它想象成机器人的“神经网络”。
#include <unitree/robot/channel/channel_publisher.hpp>  // 发布者：负责把你的“控制指令”发给机器人
#include <unitree/robot/channel/channel_subscriber.hpp> // 订阅者：负责接收机器人传回的“实时状态”

// IDL 数据定义 (低层接口)  这些是由 IDL（接口定义语言）生成的结构体，定义了机器人能听懂的“语言格式”
#include <unitree/idl/hg/IMUState_.hpp> //机器人的姿态数据（加速度、角速度、倾斜角度）
#include <unitree/idl/hg/LowCmd_.hpp> //底层控制指令。包括每个电机的位置，比例增益，微分增益等。
#include <unitree/idl/hg/LowState_.hpp> //底层状态反馈。告诉你电机现在的实际角度、转速和温度
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp> //模式切换器

//控制指令频道。Low level Command。程序计算出的电机角度、力矩等数据，都会打包发往这个频道。机器人内部的电机驱动器会监听这个频道并执行动作。
static const std::string HG_CMD_TOPIC = "rt/lowcmd";
//躯干惯性测量单元。专门用来获取机器人身体在空间中的平衡状态（倾斜角、加速度）。这对保持机器人不倒地非常关键。
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
//状态反馈频道。Low level State。机器人每时每刻都在往这个频道发送它“自己”的数据（比如当前电机转到了多少度、是否有报错）。程序需要订阅它来获取反馈。
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

//控制机器人时，通常会有两个程序在同时跑。如果接收线程正在写数据，控制线程正好在读数据，程序就会“打架”（内存冲突）。这个类就是为了解决这个冲突，确保数据读写安全的。
template <typename T>
class DataBuffer {
 public:
  //存入数据 - 写入
  void SetData(const T &newData) {
    std::unique_lock<std::shared_mutex> lock(mutex); //锁门（独占）
    data = std::make_shared<T>(newData);             //放入新数据
  }

  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex); //锁门（共享）
    return data ? data : nullptr;                    //返回数据
  }

  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex); //重置缓冲区。比如机器人断开连接了，旧的数据就不能再用了，必须清空防止误操作
    data = nullptr;
  }

 private:
  std::shared_ptr<T> data; //智能指针
  std::shared_mutex mutex; //读写锁
};

const int G1_NUM_MOTOR = 29; //29个电机
struct ImuState { //平衡感
  std::array<float, 3> rpy = {}; //长度为 3 的数组，翻滚角、俯仰角、偏航角
  std::array<float, 3> omega = {}; //长度为 3 的数组。角速度。描述机器人转动得有多快
};
struct MotorCommand {//控制指令 - 我下达的命令
  std::array<float, G1_NUM_MOTOR> q_target = {}; //目标角度
  std::array<float, G1_NUM_MOTOR> dq_target = {};//目标速度
  std::array<float, G1_NUM_MOTOR> kp = {};       //比例增益，越大，关节越僵硬；越小，越像弹簧
  std::array<float, G1_NUM_MOTOR> kd = {};       //微分增益。防止关节像弹簧一样来回乱晃
  std::array<float, G1_NUM_MOTOR> tau_ff = {};   //前馈力矩。给电机的额外“推力”，通常用来抵消重力。
};
struct MotorState {//状态反馈 - 机器人告诉我的
  std::array<float, G1_NUM_MOTOR> q = {};//实际角度
  std::array<float, G1_NUM_MOTOR> dq = {};//实际速度
};

// Stiffness for all G1 Joints 刚度
std::array<float, G1_NUM_MOTOR> Kp{
    60, 60, 60, 100, 40, 40,      // legs
    60, 60, 60, 100, 40, 40,      // legs
    60, 40, 40,                   // waist
    40, 40, 40, 40,  40, 40, 40,  // arms
    40, 40, 40, 40,  40, 40, 40   // arms
};

// Damping for all G1 Joints 阻尼
std::array<float, G1_NUM_MOTOR> Kd{
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1,              // waist
    1, 1, 1, 1, 1, 1, 1,  // arms
    1, 1, 1, 1, 1, 1, 1   // arms
};

enum class Mode { //枚举类，多选菜单，模式
  PR = 0,  // Series Control for Ptich/Roll Joints。 Pitch (俯仰)：脚尖向上勾或向下踩，Roll (翻滚)：脚板向左翻或向右翻
  AB = 1   // Parallel Control for A/B Joints。脚踝后方那两个并排的推杆电机。
};

enum G1JointIndex {//关节编号
  LeftHipPitch = 0,       //左腿髋俯仰
  LeftHipRoll = 1,        //左腿髋翻滚
  LeftHipYaw = 2,         //左腿髋偏航
  LeftKnee = 3,           //左腿膝
  LeftAnklePitch = 4,     //左踝俯仰
  LeftAnkleB = 4,         //左踝俯仰
  LeftAnkleRoll = 5,      //左踝翻滚
  LeftAnkleA = 5,         //左踝翻滚
  RightHipPitch = 6,      //右腿髋俯仰
  RightHipRoll = 7,       //右腿髋翻滚
  RightHipYaw = 8,        //右腿髋偏航
  RightKnee = 9,          //右腿膝
  RightAnklePitch = 10,   //右踝俯仰
  RightAnkleB = 10,       //右踝俯仰
  RightAnkleRoll = 11,    //右踝翻滚
  RightAnkleA = 11,       //右踝翻滚
  WaistYaw = 12,          //腰偏航
  WaistRoll = 13,         //腰翻滚       // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistA = 13,            //腰翻滚       // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistPitch = 14,        //腰俯仰       // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistB = 14,            //腰俯仰       // NOTE INVALID for g1 23dof/29dof with waist locked
  LeftShoulderPitch = 15, //左肩俯仰
  LeftShoulderRoll = 16,  //左肩翻滚
  LeftShoulderYaw = 17,   //左肩偏航
  LeftElbow = 18,         //左肘
  LeftWristRoll = 19,     //左腕翻滚
  LeftWristPitch = 20,    //左腕俯仰     // NOTE INVALID for g1 23dof
  LeftWristYaw = 21,      //左腕偏航     // NOTE INVALID for g1 23dof
  RightShoulderPitch = 22,//右肩俯仰
  RightShoulderRoll = 23, //右肩翻滚
  RightShoulderYaw = 24,  //右肩偏航
  RightElbow = 25,        //右肘
  RightWristRoll = 26,    //右腕翻滚
  RightWristPitch = 27,   //右腕俯仰     // NOTE INVALID for g1 23dof
  RightWristYaw = 28      //右腕偏航     // NOTE INVALID for g1 23dof
};
//安检，计算校验码，确保数据包正确
inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {//参数含义：uint32_t *ptr指向你要发送的数据包的指针；uint32_t len数据包的长度；返回一个 32 位的数字作为“指纹”
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;//初始值，把所有位填满
  const uint32_t dwPolynomial = 0x04c11db7;//标尺
  for (uint32_t i = 0; i < len; i++) {//外层循环，遍历数据包里的每一个 32 位数据
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {//内层循环，对每一个数据位（bit）进行复杂的位运算
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial; //如果检测到某些特定条件，就用“标尺”去异或一下
      } else
        CRC32 <<= 1;
      if (data & xbit) CRC32 ^= dwPolynomial;

      xbit >>= 1;
    }
  }
  return CRC32;
};

class G1Example {
 private:
  double time_;
  double control_dt_;  // [2ms]
  double duration_;    // [3 s]
  int counter_;
  Mode mode_pr_;
  uint8_t mode_machine_;

  Gamepad gamepad_;
  REMOTE_DATA_RX rx_;

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;

  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
  ChannelSubscriberPtr<IMUState_> imutorso_subscriber_;
  ThreadPtr command_writer_ptr_, control_thread_ptr_;

  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;

 public:
  G1Example(std::string networkInterface)
      : time_(0.0),
        control_dt_(0.002),
        duration_(3.0),
        counter_(0),
        mode_pr_(Mode::PR),
        mode_machine_(0) {
    ChannelFactory::Instance()->Init(0, networkInterface);

    // try to shutdown motion control-related service
    msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    msc_->SetTimeout(5.0f);
    msc_->Init();
    std::string form, name;
    while (msc_->CheckMode(form, name), !name.empty()) {
      if (msc_->ReleaseMode())
        std::cout << "Failed to switch to Release Mode\n";
      sleep(5);
    }

    // create publisher
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();
    // create subscriber
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(std::bind(&G1Example::LowStateHandler, this, std::placeholders::_1), 1);
    imutorso_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    imutorso_subscriber_->InitChannel(std::bind(&G1Example::imuTorsoHandler, this, std::placeholders::_1), 1);
    // create threads
    command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &G1Example::LowCommandWriter, this);
    control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, &G1Example::Control, this);
  }

  //接收躯干姿态数据的回调函数
  void imuTorsoHandler(const void *message) { //参数是一个无类型指针
    IMUState_ imu_torso = *(const IMUState_ *)message;//告知编译器，数据包内容的格式是 IMUState_，按照这个格式解析
    auto &rpy = imu_torso.rpy();//从解析好的数据里提取出 RPY：Roll (翻滚)：机器人左右晃不晃；Pitch (俯仰)：机器人前后倒不倒；Yaw (偏航)：机器人有没有在水平打转
    if (counter_ % 500 == 0)//IMU数据刷新很快，每 500 次才打印一次，大约一秒钟一次，降频显示。
      printf("IMU.torso.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]); //把三个轴的角度打印出来，保留两位小数
  }
//情报中心。回调函数。每当机器人发送一个LowState数据包，这个函数就会被调用。
  void LowStateHandler(const void *message) {
    LowState_ low_state = *(const LowState_ *)message;
    //CRC检验
    if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
      std::cout << "[ERROR] CRC Error" << std::endl;
      return;//如果校验码不对，直接丢弃
    }

    // get motor state
    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {//遍历 29 个电机，把每个电机的角度（q）和速度（dq）存好。如果电机报错，打印错误日志。
      ms_tmp.q.at(i) = low_state.motor_state()[i].q();
      ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
      if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll)
        std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
    }
    motor_state_buffer_.SetData(ms_tmp);

    // get imu state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();
    imu_state_buffer_.SetData(imu_tmp);

    // update gamepad
    memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
    gamepad_.update(rx_.RF_RX);

    // update mode machine
    if (mode_machine_ != low_state.mode_machine()) {
      if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
      mode_machine_ = low_state.mode_machine();
    }

    // report robot status every second
    if (++counter_ % 500 == 0) {
      counter_ = 0;
      // IMU
      auto &rpy = low_state.imu_state().rpy();
      printf("IMU.pelvis.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);

      // RC
      printf("gamepad_.A.pressed: %d\n", static_cast<int>(gamepad_.A.pressed));
      printf("gamepad_.B.pressed: %d\n", static_cast<int>(gamepad_.B.pressed));
      printf("gamepad_.X.pressed: %d\n", static_cast<int>(gamepad_.X.pressed));
      printf("gamepad_.Y.pressed: %d\n", static_cast<int>(gamepad_.Y.pressed));

      // Motor
      auto &ms = low_state.motor_state();
      printf("All %d Motors:", G1_NUM_MOTOR);
      printf("\nmode: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,", ms[i].mode());
      printf("\npos: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].q());
      printf("\nvel: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].dq());
      printf("\ntau_est: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].tau_est());
      printf("\ntemperature: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%d,%d;", ms[i].temperature()[0], ms[i].temperature()[1]);
      printf("\nvol: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].vol());
      printf("\nsensor: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,%u;", ms[i].sensor()[0], ms[i].sensor()[1]);
      printf("\nmotorstate: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,", ms[i].motorstate());
      printf("\nreserve: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,%u,%u,%u;", ms[i].reserve()[0], ms[i].reserve()[1], ms[i].reserve()[2], ms[i].reserve()[3]);
      printf("\n");
    }
  }

  void LowCommandWriter() {
    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
    dds_low_command.mode_machine() = mode_machine_;

    const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
    if (mc) {
      for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
        dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
        dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
        dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
        dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
        dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);
        dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);
      }

      dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
      lowcmd_publisher_->Write(dds_low_command);
    }
  }

  void Control() {
    MotorCommand motor_command_tmp;
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();

    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      motor_command_tmp.tau_ff.at(i) = 0.0;
      motor_command_tmp.q_target.at(i) = 0.0;
      motor_command_tmp.dq_target.at(i) = 0.0;
      motor_command_tmp.kp.at(i) = Kp[i];
      motor_command_tmp.kd.at(i) = Kd[i];
    }

    if (ms) {
      time_ += control_dt_;
      if (time_ < duration_) {
        // [Stage 1]: set robot to zero posture
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
          double ratio = std::clamp(time_ / duration_, 0.0, 1.0);
          motor_command_tmp.q_target.at(i) = (1.0 - ratio) * ms->q.at(i);
        }
      } else if (time_ < duration_ * 2) {
        // [Stage 2]: swing ankle using PR mode
        mode_pr_ = Mode::PR;
        double max_P = M_PI * 30.0 / 180.0;
        double max_R = M_PI * 10.0 / 180.0;
        double t = time_ - duration_;
        double L_P_des = max_P * std::sin(2.0 * M_PI * t);
        double L_R_des = max_R * std::sin(2.0 * M_PI * t);
        double R_P_des = max_P * std::sin(2.0 * M_PI * t);
        double R_R_des = -max_R * std::sin(2.0 * M_PI * t);

        motor_command_tmp.q_target.at(LeftAnklePitch) = L_P_des;
        motor_command_tmp.q_target.at(LeftAnkleRoll) = L_R_des;
        motor_command_tmp.q_target.at(RightAnklePitch) = R_P_des;
        motor_command_tmp.q_target.at(RightAnkleRoll) = R_R_des;
      } else {
        // [Stage 3]: swing ankle using AB mode
        mode_pr_ = Mode::AB;
        double max_A = M_PI * 30.0 / 180.0;
        double max_B = M_PI * 10.0 / 180.0;
        double t = time_ - duration_ * 2;
        double L_A_des = +max_A * std::sin(M_PI * t);
        double L_B_des = +max_B * std::sin(M_PI * t + M_PI);
        double R_A_des = -max_A * std::sin(M_PI * t);
        double R_B_des = -max_B * std::sin(M_PI * t + M_PI);

        motor_command_tmp.q_target.at(LeftAnkleA) = L_A_des;
        motor_command_tmp.q_target.at(LeftAnkleB) = L_B_des;
        motor_command_tmp.q_target.at(RightAnkleA) = R_A_des;
        motor_command_tmp.q_target.at(RightAnkleB) = R_B_des;
      }

      motor_command_buffer_.SetData(motor_command_tmp);
    }
  }
};

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_ankle_swing_example network_interface" << std::endl;
    exit(0);
  }
  std::string networkInterface = argv[1];
  G1Example custom(networkInterface);
  while (true) sleep(10);
  return 0;
}

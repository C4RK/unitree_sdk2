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
  double time_;//运行总时间
  double control_dt_;  // [2ms]步长（2ms），决定了动作的细腻程度
  double duration_;    // [3 s]动作切换周期
  int counter_;//计数器，专门用来控制打印日志的频率
  Mode mode_pr_;//记录当前是 PR 模式还是 AB 模式
  uint8_t mode_machine_;// 记录机器人的型号状态

  Gamepad gamepad_;// 游戏手柄：把原始信号转换成“A键按下”、“左摇杆推”等语义
  REMOTE_DATA_RX rx_;// 接收缓存：专门存放手柄发来的原始字节数据

  DataBuffer<MotorState> motor_state_buffer_;// 存：机器人现在转到哪了
  DataBuffer<MotorCommand> motor_command_buffer_;// 存：我下令让机器人转到哪
  DataBuffer<ImuState> imu_state_buffer_;// 存：机器人的身体歪没歪

  ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;// 发射塔：把指令发给机器人
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;// 接收塔：收电机的状态
  ChannelSubscriberPtr<IMUState_> imutorso_subscriber_;// 接收塔：收身体的姿态
  ThreadPtr command_writer_ptr_, control_thread_ptr_;// 两个并行的引擎

  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;// 钥匙：切换机器人控制权的客户端

 public:
  G1Example(std::string networkInterface)//构造函数
      : time_(0.0),
        control_dt_(0.002),
        duration_(3.0),
        counter_(0),
        mode_pr_(Mode::PR),
        mode_machine_(0) {
    ChannelFactory::Instance()->Init(0, networkInterface);//建立通讯网

    // try to shutdown motion control-related service
    msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();//创建一个叫msc的切换员。是和机器人原厂服务之间沟通的桥梁
    msc_->SetTimeout(5.0f);// 设置 5 秒超时，防止网络卡死
    msc_->Init();// 建立连接
    std::string form, name;
    //// 检查当前机器人正在运行什么模式 (CheckMode)；如果 name 不为空 (!name.empty())，说明机器人还被原厂模式控制着
    while (msc_->CheckMode(form, name), !name.empty()) {
      // 尝试释放当前模式 (ReleaseMode)
      if (msc_->ReleaseMode())
        std::cout << "Failed to switch to Release Mode\n"; // 如果失败了报错
      // 休息 5 秒再检查一次，直到机器人彻底进入“自由模式（Release Mode）”
        sleep(5);
    }

    // create publisher 创建发布者：负责把 LowCmd_ 指令发送到 HG_CMD_TOPIC 频道
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();
    // create subscriber 创建订阅者：负责监听 HG_STATE_TOPIC 和 HG_IMU_TORSO频道里的机器人状态
    // 这里使用了 bind 绑定：一旦收到新状态，立即自动调用 LowStateHandler 函数
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(std::bind(&G1Example::LowStateHandler, this, std::placeholders::_1), 1);
    imutorso_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    imutorso_subscriber_->InitChannel(std::bind(&G1Example::imuTorsoHandler, this, std::placeholders::_1), 1);
    // create threads
    // 创建“指令发送”线程：每 2000 微秒（2ms）执行一次 LowCommandWriter
    command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &G1Example::LowCommandWriter, this);
    // 创建“运动计算”线程：每 2000 微秒（2ms）执行一次 Control
    control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, &G1Example::Control, this);
  }

  //接收躯干姿态数据的回调函数
  void imuTorsoHandler(const void *message) { //参数是一个无类型指针
    IMUState_ imu_torso = *(const IMUState_ *)message;//告知编译器，数据包内容的格式是 IMUState_，按照这个格式解析
    auto &rpy = imu_torso.rpy();//从解析好的数据里提取出 RPY：Roll (翻滚)：机器人左右晃不晃；Pitch (俯仰)：机器人前后倒不倒；Yaw (偏航)：机器人有没有在水平打转
    if (counter_ % 500 == 0)//IMU数据刷新很快，每 500 次才打印一次，大约一秒钟一次，降频显示。
      printf("IMU.torso.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]); //把三个轴的角度打印出来，保留两位小数
  }
//情报中心。回调函数。每当机器人发送一个LowState数据包，这个函数就会被调用。这个函数是运行在接收线程里的。
  void LowStateHandler(const void *message) {
    LowState_ low_state = *(const LowState_ *)message;
    //CRC检验
    if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
      std::cout << "[ERROR] CRC Error" << std::endl;
      return;//如果校验码不对，直接丢弃
    }

    // get motor state
    MotorState ms_tmp;//局部变量，存储电机的数据
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {//遍历 29 个电机，把每个电机的角度（q）和速度（dq）存好。如果电机报错，打印错误日志。
      ms_tmp.q.at(i) = low_state.motor_state()[i].q(); //low_state.motor_state()[i] -- 从网络收到的原始数据包，里面包含第 i 个电机的各种原始信息。
      ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
      if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll)//正常运行下 为0；程序筛选出了下半身所有核心电机
        std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
    }
    motor_state_buffer_.SetData(ms_tmp);//将局部装满数据的ms_tmp 拷贝进全局共享的 motor_state_buffer_ 中。跨线程搬运

    // get imu state
    ImuState imu_tmp;//在栈空间里临时创建一个 ImuState 结构体实例。
    imu_tmp.omega = low_state.imu_state().gyroscope();//获取陀螺仪数据，角速度
    imu_tmp.rpy = low_state.imu_state().rpy();//获取姿态角，获取 Roll（翻滚）、Pitch（俯仰）、Yaw（偏航）
    imu_state_buffer_.SetData(imu_tmp);//存入安全中转站

    // update gamepad
    memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);//暴力搬运函数。从存储无线遥控信号的起始地址&low_state.wireless_remote()[0]搬到临时缓存区rx_.buff。
    gamepad_.update(rx_.RF_RX);//翻译成“人话”

    // update mode machine
    if (mode_machine_ != low_state.mode_machine()) {//检查当前记录的状态 mode_machine_ 是否与机器人刚刚发回来的 low_state.mode_machine() 不一致
      if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;//打印机器人型号
      mode_machine_ = low_state.mode_machine();//把最新的模式值保存到本地变量 mode_machine_ 中。
    }

    // report robot status every second 打印报告机器人的状态
    if (++counter_ % 500 == 0) {//每次函数被调用（即收到新数据），计数器就加 1；只有当计数器是 500 的倍数时才进入大括号
      counter_ = 0;
      // IMU
      auto &rpy = low_state.imu_state().rpy();
      printf("IMU.pelvis.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);//骨盆（Pelvis）的倾斜角度。这是判断机器人是否由于脚踝摆动导致身体不稳的最直接数据。

      // RC 遥控器 手柄上的 A、B、X、Y 键是否被按下。
      printf("gamepad_.A.pressed: %d\n", static_cast<int>(gamepad_.A.pressed));
      printf("gamepad_.B.pressed: %d\n", static_cast<int>(gamepad_.B.pressed));
      printf("gamepad_.X.pressed: %d\n", static_cast<int>(gamepad_.X.pressed));
      printf("gamepad_.Y.pressed: %d\n", static_cast<int>(gamepad_.Y.pressed));

      // Motor 电机全状态
      auto &ms = low_state.motor_state();
      printf("All %d Motors:", G1_NUM_MOTOR);
      printf("\nmode: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,", ms[i].mode());//控制模式
      printf("\npos: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].q());//实时角度
      printf("\nvel: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].dq());//转动速度
      printf("\ntau_est: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].tau_est());//估算力矩
      printf("\ntemperature: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%d,%d;", ms[i].temperature()[0], ms[i].temperature()[1]);//温度
      printf("\nvol: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%.2f,", ms[i].vol());//电压
      printf("\nsensor: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,%u;", ms[i].sensor()[0], ms[i].sensor()[1]);//传感器
      printf("\nmotorstate: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,", ms[i].motorstate());//报错码
      printf("\nreserve: ");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) printf("%u,%u,%u,%u;", ms[i].reserve()[0], ms[i].reserve()[1], ms[i].reserve()[2], ms[i].reserve()[3]);//预留位
      printf("\n");
    }
  }

  void LowCommandWriter() {//把你在程序里算好的“理想动作”，打包成机器人硬件能听懂的二进制格式，然后通过网络“发射”出去。
    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
    dds_low_command.mode_machine() = mode_machine_;

    const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
    if (mc) {
      for (size_t i = 0; i < G1_NUM_MOTOR; i++) {//把自己定义的 MotorCommand 里的数据 填进宇树标准的 dds_low_command 盒子里
        dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable 1代表电机启用
        dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
        dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
        dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
        dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);
        dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);
      }

      dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);//CRC校验
      lowcmd_publisher_->Write(dds_low_command);//发送指令
    }
  }

  void Control() {
    MotorCommand motor_command_tmp;//创建临时的动作清单,指令清单。
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();// 从Buffer里获取机器人当前的实际位置

    for (int i = 0; i < G1_NUM_MOTOR; ++i) {//先把29个电机设置好默认的kp和kd。电机的目标位置默认为0
      motor_command_tmp.tau_ff.at(i) = 0.0;
      motor_command_tmp.q_target.at(i) = 0.0;
      motor_command_tmp.dq_target.at(i) = 0.0;
      motor_command_tmp.kp.at(i) = Kp[i];
      motor_command_tmp.kd.at(i) = Kd[i];
    }

    if (ms) {//如果成功拿到了机器人的当前状态，才开始指挥
      time_ += control_dt_;//计时器累加
      if (time_ < duration_) {
        // [Stage 1]: set robot to zero posture 平滑立正（0 ~ duration_ 秒）
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
          double ratio = std::clamp(time_ / duration_, 0.0, 1.0);//从 0% 慢慢变成 100%
          motor_command_tmp.q_target.at(i) = (1.0 - ratio) * ms->q.at(i);//ms->q是机器人此刻真实的关节角度。采用线形插值将q慢慢变到0.
        }
      } else if (time_ < duration_ * 2) {
        // [Stage 2]: swing ankle using PR mode PR 模式（前后/左右摇摆）
        mode_pr_ = Mode::PR;//告诉机器人：现在用“俯仰/翻滚”模式来理解我的指令
        double max_P = M_PI * 30.0 / 180.0; //定义摆动幅度：俯仰最大 30度（转化为弧度）
        double max_R = M_PI * 10.0 / 180.0; //定义摆动幅度：翻滚最大 10度（转化为弧度）
        double t = time_ - duration_; //这一阶段走过的时间 t
        //利用正弦波 sin 计算当前时间对应的角度（从 -max 到 +max 循环）
        double L_P_des = max_P * std::sin(2.0 * M_PI * t);// 左脚俯仰
        double L_R_des = max_R * std::sin(2.0 * M_PI * t);// 左脚翻滚
        double R_P_des = max_P * std::sin(2.0 * M_PI * t);// 右脚俯仰
        double R_R_des = -max_R * std::sin(2.0 * M_PI * t);// 右脚翻滚
        //把算好的角度填进对应的“穴位”里
        motor_command_tmp.q_target.at(LeftAnklePitch) = L_P_des;
        motor_command_tmp.q_target.at(LeftAnkleRoll) = L_R_des;
        motor_command_tmp.q_target.at(RightAnklePitch) = R_P_des;
        motor_command_tmp.q_target.at(RightAnkleRoll) = R_R_des;
      } else {
        // [Stage 3]: swing ankle using AB mode
        mode_pr_ = Mode::AB;//切换到AB模式，指挥推杆。
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
      //这一毫秒辛苦算好的“指令单”塞进保险柜；另一个线程会立刻把它取走发给电机
      motor_command_buffer_.SetData(motor_command_tmp);
    }
  }
};

int main(int argc, char const *argv[]) {//argc 整数类型，表示命令行输入参数的个数; argv: 字符指针数组（字符串数组），存储具体的参数内容;
  //argv[0] 永远是程序本身的名称,argv[1] 是你输入的第一个参数（在这里是网卡名称）
  if (argc < 2) {
    std::cout << "Usage: g1_ankle_swing_example network_interface" << std::endl;
    exit(0); //立即终止整个进程
  }
  std::string networkInterface = argv[1];
  G1Example custom(networkInterface);//在栈 (Stack) 上创建了一个名为 custom 的实例
  while (true) sleep(10);
  return 0;
}

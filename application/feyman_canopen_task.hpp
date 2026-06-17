#pragma once

#include <array>
#include <cstdint>

#include "can.hpp"
#include "libxr_def.hpp"
#include "managers/data_types.hpp"
#include "message.hpp"
#include "semaphore.hpp"
#include "thread.hpp"

namespace Application {

/// FEYMAN IMU 的 CANopen 任务配置参数。
struct FeymanCanopenConfig {
  /// CANopen 节点 ID；需要与传感器当前节点号一致。
  uint8_t node_id = 0x7F;
  /// 传感器 CAN 波特率，单位 bps；通过对象字典 0x3003 写入。
  uint32_t baudrate = 250000;
  /// 传感器 TPDO 数据上报频率，单位 Hz。
  uint32_t data_rate_hz = 20;
  /// CANopen producer heartbeat 周期，单位 ms；写入对象 0x1017。
  uint32_t heartbeat_ms = 1000;
  /// 后台任务优先级，取值使用 LibXR::Thread::Priority。
  uint32_t priority =
      static_cast<uint32_t>(LibXR::Thread::Priority::HIGH);
  /// 后台任务栈大小，单位 byte。
  uint32_t stack_size = 2048;
  /// 单次 SDO 请求等待响应的超时时间，单位 ms。
  uint32_t sdo_timeout_ms = 200;
  /// 连续 SDO 请求之间的保护延时，单位 ms。
  uint32_t sdo_inter_request_delay_ms = 5;
  /// 切换 CANopen 工作模式后的等待时间，单位 ms。
  uint32_t work_mode_settle_ms = 50;
  /// 任务启动后、配置设备前的等待时间，单位 ms。
  uint32_t startup_delay_ms = 50;
  /// 是否输出详细配置过程日志。
  bool verbose_config_log = true;
  /// 发布 FEYMAN 姿态/惯性数据的 LibXR topic 名称。
  const char* topic_name = "feyman_imu_pose";
  /// 可选日志输出函数；传入一整行文本，由调用方决定输出到 UART 或其它终端。
  void (*log_writer)(const char* text) = nullptr;
};

/// 负责通过 CANopen 配置 FEYMAN IMU，并把 TPDO 数据发布到 topic。
class FeymanCanopenTask {
 public:
  /// 构造任务对象。
  /// @param can 已初始化的 CAN 驱动实例，生命周期由调用方管理。
  /// @param config CANopen 节点、频率、日志和任务资源配置。
  explicit FeymanCanopenTask(
      LibXR::CAN* can,
      const FeymanCanopenConfig& config = FeymanCanopenConfig{});
  ~FeymanCanopenTask() = default;

  /// 创建 topic、注册 CAN 回调并启动后台线程。
  LibXR::ErrorCode Start();
  /// 请求后台线程停止，并唤醒可能正在等待的 SDO 流程。
  void Stop();

 private:
  /// SDO 响应缓存；由 CAN 回调写入，任务线程读取。
  struct SdoResponse {
    /// 对象字典索引。
    uint16_t index = 0;
    /// 对象字典子索引。
    uint8_t subindex = 0;
    /// SDO 响应命令字。
    uint8_t command = 0;
    /// expedited SDO 的 0~4 字节数据区，小端序保存。
    std::array<uint8_t, 4> data = {0, 0, 0, 0};
    /// 是否收到 SDO abort 响应。
    bool abort = false;
    /// SDO abort 码，仅 abort 为 true 时有效。
    uint32_t abort_code = 0;
  };

  /// 当前等待中的 SDO 事务，用于匹配异步 CAN 响应。
  struct SdoTransaction {
    /// true 表示已有请求发出并正在等待响应。
    bool active = false;
    /// 期望响应的对象字典索引。
    uint16_t index = 0;
    /// 期望响应的对象字典子索引。
    uint8_t subindex = 0;
    /// 匹配到的响应内容。
    SdoResponse response{};
  };

  /// 本任务关心的 TPDO 类型。
  enum class PdoKind : uint8_t {
    /// TPDO1 映射三轴加速度原始值。
    TPDO1_ACCEL = 0,
    /// TPDO2 映射三轴角速度原始值。
    TPDO2_GYRO = 1,
  };

  /// 最近一次收到的 PDO/heartbeat 状态快照。
  struct PdoState {
    /// 三轴加速度原始 int16 数据。
    std::array<int16_t, 3> acc_raw = {0, 0, 0};
    /// 三轴角速度原始 int16 数据。
    std::array<int16_t, 3> gyro_raw = {0, 0, 0};
    /// 设备状态标志，当前预留给后续对象映射扩展。
    uint16_t status_flags = 0;
    /// 每收到一个有效 PDO 后递增的本地序号。
    uint16_t sequence = 0;
    /// 标记加速度/角速度 PDO 是否已经收到。
    uint8_t valid_mask = 0;
    /// 最近一次 heartbeat 帧中的 CANopen 节点状态。
    uint8_t heartbeat_state = 0;
    /// 最近一次有效 PDO 的 MCU 时间戳，单位 us。
    uint64_t latest_pdo_tick_us = 0;
    /// 最近一次 heartbeat 的 MCU 时间戳，单位 us。
    uint64_t latest_heartbeat_tick_us = 0;
  };

  /// CAN 错误回调中的轻量缓存；实际日志在任务线程中输出。
  struct PendingCanError {
    /// 是否有待输出的错误记录。
    bool pending = false;
    /// 两次刷新之间累计的错误帧数量。
    uint32_t count = 0;
    /// 最近一次错误帧 ID。
    uint32_t last_error_id = 0;
    /// CAN 控制器错误状态快照。
    LibXR::CAN::ErrorState state{};
    /// state 是否读取成功。
    bool state_valid = false;
  };

  /// LibXR 线程入口适配函数。
  static void TaskEntry(FeymanCanopenTask* task);
  /// CAN 回调入口适配函数，把 C 风格回调转发到对象方法。
  static void OnCanFrame(bool in_isr, FeymanCanopenTask* task,
                         const LibXR::CAN::ClassicPack& pack);

  /// 任务主循环：初始化设备，然后持续发布 PDO 数据和错误日志。
  void Run();
  /// 分发收到的 CAN 帧到 SDO、heartbeat、TPDO 或错误处理路径。
  void HandleCanFrame(bool in_isr, const LibXR::CAN::ClassicPack& pack);
  /// 记录 CAN 错误状态，避免在中断上下文中直接打印日志。
  void HandleCanError(const LibXR::CAN::ClassicPack& pack);
  /// 匹配并保存当前 SDO 事务的响应。
  void HandleSdoResponse(const LibXR::CAN::ClassicPack& pack);
  /// 更新 heartbeat 状态和接收时间。
  void HandleHeartbeat(const LibXR::CAN::ClassicPack& pack);
  /// 解析 TPDO1/TPDO2 中的三轴原始值。
  void HandlePdo(PdoKind kind, const LibXR::CAN::ClassicPack& pack);
  /// 在加速度和角速度都更新后发布 FeymanPoseMsg。
  void PublishPoseIfReady();
  /// 将 CAN 错误缓存转换为普通任务上下文日志。
  void FlushPendingCanErrorLog();

  /// 执行完整 CANopen 初始化流程：预操作、参数配置、TPDO 映射、启动节点。
  LibXR::ErrorCode ConfigureDevice();
  /// 配置波特率、节点号、数据频率、心跳、工作模式等基础参数。
  LibXR::ErrorCode ConfigureBasicParameters();
  /// 配置一个 TPDO 的 COB-ID、映射表和事件定时器。
  LibXR::ErrorCode ConfigureTpdo(uint8_t pdo_index, uint16_t cob_id,
                                 uint8_t map_count,
                                 const uint32_t* mappings);
  /// 发送 CANopen NMT 命令。
  LibXR::ErrorCode SendNmt(uint8_t command);
  /// 读取 32 位对象字典值。
  LibXR::ErrorCode SdoReadU32(uint16_t index, uint8_t subindex,
                              uint32_t* value_out);
  /// 写入 8 位对象字典值。
  LibXR::ErrorCode SdoWriteU8(uint16_t index, uint8_t subindex, uint8_t value);
  /// 写入 16 位对象字典值。
  LibXR::ErrorCode SdoWriteU16(uint16_t index, uint8_t subindex,
                               uint16_t value);
  /// 写入 32 位对象字典值。
  LibXR::ErrorCode SdoWriteU32(uint16_t index, uint8_t subindex,
                               uint32_t value);
  /// 组包并发送 expedited SDO 请求。
  LibXR::ErrorCode SendSdoRequest(uint8_t command, uint16_t index,
                                  uint8_t subindex,
                                  const std::array<uint8_t, 4>& data);
  /// 等待并校验指定索引/子索引的 SDO 响应。
  LibXR::ErrorCode WaitSdoResponse(uint16_t index, uint8_t subindex,
                                   SdoResponse& response);
  /// 按配置插入 SDO 请求间隔。
  void DelayBetweenSdoRequests();
  /// 构造并发送 Classic CAN 帧。
  LibXR::ErrorCode SendCanFrame(uint32_t id, const uint8_t* data, uint8_t dlc,
                                LibXR::CAN::Type type);

  /// 输出普通日志。
  void Log(const char* text);
  /// verbose_config_log 开启时输出配置日志。
  void LogConfig(const char* text);
  /// 格式化输出普通日志。
  void Logf(const char* fmt, ...);
  /// verbose_config_log 开启时格式化输出配置日志。
  void LogConfigf(const char* fmt, ...);
  /// 读取并输出 CAN 控制器错误计数和状态。
  void LogCanErrorState(const char* context);

  /// 将 FEYMAN 加速度原始值换算为 m/s^2。
  static float DecodeAccelMps2(int16_t raw);
  /// 将 FEYMAN 角速度原始值换算为 deg/s。
  static float DecodeGyroDps(int16_t raw);

  /// 外部 CAN 驱动指针，生命周期由调用方管理。
  LibXR::CAN* can_;
  /// 任务运行参数的本地副本。
  FeymanCanopenConfig config_;
  /// 后台线程对象。
  LibXR::Thread thread_{};
  /// 输出 topic 指针，由 Start 创建；重复启动前会释放旧实例。
  LibXR::Topic* topic_ = nullptr;
  /// 任务运行标志，Stop 和任务循环共享。
  volatile bool running_ = false;
  /// 注册到 CAN 驱动的接收/错误回调。
  LibXR::CAN::Callback can_callback_{};
  /// SDO 响应同步信号量。
  LibXR::Semaphore sdo_sem_{0};
  /// 当前 SDO 事务状态。
  SdoTransaction sdo_transaction_{};
  /// PDO 与 heartbeat 的最新状态。
  PdoState pdo_state_{};
  /// 等待配对发布的 PDO 位图，用于跟踪本轮已收到哪些 TPDO。
  uint8_t pending_pdo_mask_ = 0;
  /// 待输出的 CAN 错误摘要。
  PendingCanError pending_can_error_{};
  /// 上一次已经发布的 PDO 时间戳，用于避免重复发布。
  uint64_t last_publish_tick_us_ = 0;
  /// 设备是否已经完成 CANopen 配置。
  bool device_configured_ = false;
};

}  // namespace Application

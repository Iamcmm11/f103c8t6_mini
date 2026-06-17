# FEYMAN 架构整理计划

## Summary
- 现状：FEYMAN 现在已经是单独的 LibXR/FreeRTOS 应用线程。启动链是 [freertos.c](C:/Users/18457/Desktop/robot/f105rct6/Core/Src/freertos.c:132) 调 `app_main()`，然后 [app_main.cpp](C:/Users/18457/Desktop/robot/f105rct6/User/app_main.cpp:187) 配置 FEYMAN、[app_main.cpp](C:/Users/18457/Desktop/robot/f105rct6/User/app_main.cpp:200) 创建 `FeymanCanopenTask`、[app_main.cpp](C:/Users/18457/Desktop/robot/f105rct6/User/app_main.cpp:367) 调 `Start()`。
- `FeymanCanopenTask::Start()` 内部创建 `feyman_imu_pose` topic、注册 CAN 回调，并创建名为 `FeymanCAN` 的线程。
- 当前问题：FEYMAN 的 CANopen SDO/TPDO、对象字典、解码、任务线程、topic 发布都揉在 `application/feyman_canopen_task.*` 里；相比 WIT/YIS 的 `modules + managers/application` 分层，FEYMAN 还停留在联调型结构。
- 采用你选择的“轻量分层”：先不新增统一 CAN manager，把 FEYMAN 底层驱动移到 `modules`，保留现有 app_main 启动方式和 topic/bridge 行为。

## Key Changes
- 新增 `modules/feyman_mcs10/`，放 FEYMAN MCS10 的底层驱动/BSP：
  - `Module::FeymanMCS10` 负责 CANopen 对象字典、NMT、SDO read/write、TPDO 映射、CAN 回调分发、acc/gyro 原始值解码。
  - module 不创建线程、不创建 topic、不依赖 `managers`，只依赖 `LibXR::CAN` 和必要同步原语。
- 保留 `Application::FeymanCanopenTask` 作为应用任务层：
  - 继续创建 `FeymanCAN` 线程。
  - 调用 `Module::FeymanMCS10::Configure()` 和读取最新 sample。
  - 将 module sample 转换成 `Manager::FeymanPoseMsg`，继续发布到 `feyman_imu_pose`。
  - 保留 `SyncSignalManager` 时间戳补偿和 UART5 日志。
- 暂不新增统一 `CANDeviceManager`：
  - 当前只有一个 CAN 设备接入，manager 会增加抽象成本。
  - 等后续出现 2 个以上 CAN 设备、共享总线生命周期/健康状态/统一注册表时，再加 `CANDeviceManager` 或 `CanSensorManager`。
- 清理 `app_main` 的 FEYMAN 启动区：
  - 保留 `FeymanCanopenConfig` 和 `feyman_task.Start()` 在 `app_main`，因为 `app_main` 是当前 composition root。
  - 修正 boot log 中 FEYMAN rate 使用硬编码 `20UL` 的问题，改为打印 `feyman_config.data_rate_hz`，避免当前配置 `100Hz` 但日志显示 `20Hz` 的不一致。
- 更新 CMake：
  - `modules/CMakeLists.txt` 增加 FEYMAN module 源文件。
  - `application` 继续链接现有 `managers`/`module` 依赖链，不引入反向依赖。

## Public Interfaces
- 新增 module 层接口，建议最小形态：
  - `Module::FeymanMCS10Config`：`node_id`、`baudrate`、`data_rate_hz`、`heartbeat_ms`、`sdo_timeout_ms`、`sdo_inter_request_delay_ms`、`work_mode_settle_ms`。
  - `Module::FeymanMCS10Sample`：decoded `acc[3]`、`gyro[3]`、`sequence`、`heartbeat_state`、`status_flags`、`latest_pdo_tick_us`。
  - `Module::FeymanMCS10::Init(LibXR::CAN*, config)`、`Configure()`、`PollSample(sample_out)`、`Stop()`。
- 保持现有外部行为不变：
  - topic 名仍是 `feyman_imu_pose`。
  - `Manager::FeymanPoseMsg` 继续作为 bridge 消费的数据结构。
  - `BridgePoseSource::FEYMAN` 和 UART bridge 协议不改。

## Test Plan
- 构建验证：运行 `cmake --build build\Debug`，确认新增 module 和 application 依赖无循环。
- 启动日志验证：确认仍出现 `[feyman] configure ok`，并且 boot log 的 rate 与 `feyman_config.data_rate_hz` 一致。
- CANopen 验证：确认 SDO read/write、TPDO1 accel、TPDO2 gyro、heartbeat 接收行为不变。
- Bridge 验证：`BridgePoseSource::FEYMAN` 下，上位机仍能收到 FEYMAN acc/gyro，Euler/quaternion 仍按当前策略填 `NaN`。
- 回归风险点：确认 CAN 回调 ISR 路径仍使用 callback-safe semaphore/API，不恢复到普通 `Post()`。

## Assumptions
- 这次只做轻量分层，不引入统一 CAN manager。
- FEYMAN 仍使用 `CAN2`、默认 node `0x7F`、topic `feyman_imu_pose`。
- 当前 FEYMAN 只稳定发布 acc/gyro；Euler/quaternion TPDO 暂不恢复。
- 后续接多个 CAN 设备时，再基于实际设备数量和协议相似度决定是否新增统一 manager。

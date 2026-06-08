# IMU UART GPIO Button Debug 2026-06

## 背景

这轮问题出现在 `USART1` 桥协议已经接通、YIS 姿态持续推流也正常之后。

我们新增了 GPIO 按钮事件桥接，希望 MCU 在按键触发时，通过 `USART1` 桥协议主动推送：

- `CMD_GPIO_BUTTON_PUSH = 0x33`

当前按钮映射是：

- `PC10 -> 'A'`
- `PA15 -> 'B'`
- `PB3  -> 'C'`
- `PB4  -> 'D'`

目标是让 NV 侧在 `console` / `capture` 模式下都能实时收到按钮事件。

---

## 初始现象

问题不是一次性暴露出来的，而是分阶段出现：

1. 最早版本中：
   - 只看到 `B` 有效；
   - 按下 `B` 会触发 `HardFault`。

2. 修完第一轮后：
   - `A/B/C` 已经有效；
   - `D` 仍然无效，怀疑板级连接问题；
   - 但在 **开流状态**（`start` 后持续 YIS 推流）下再按键：
     - `NV` 侧收不到按钮事件；
     - 连 `UART5` 调试口也没有按钮相关日志；
   - 一旦 `stop` 关流，按钮事件又恢复正常。

这个现象很关键，因为它说明问题不在对端脚本解析，而更可能在 MCU 内部调度或发送链路。

---

## 先前的错误实现

最开始 GPIO 按钮事件的发送路径是：

```text
GPIOManager 线程
-> PublishGPIOButtonCommandCallback()
-> IMUUartBridgeTask::PublishGPIOButtonCommand()
-> 直接 SendResponse()
-> 直接走 USART1 发送
```

这个做法的问题是：

1. GPIO 线程直接参与桥协议发包，职责耦合过深。
2. GPIO 线程和桥线程可能并发写同一个 `USART1`。
3. 按键触发时，GPIO 线程会走一串串口发送路径，出现了 `HardFault` 风险。

也就是说，GPIO 层本来应该只报告“按钮事件”，不应该自己成为桥协议发送线程。

---

## 根因分析

这次最终定位出来的其实是两个问题，前后叠加。

### 1. GPIO 输入配置不稳定

原始 `gpio.c` 里这几个按钮脚：

- `PA15`
- `PC10`
- `PB3`
- `PB4`

都是 `GPIO_MODE_INPUT + GPIO_NOPULL`。

这会带来两个问题：

1. 如果外部没有稳定上拉/下拉，输入会漂浮；
2. 在不同按键和不同板级连接上，现象会很不一致。

这和早期“只有 B 稳定有效”是对得上的。

### 2. 开流后桥线程占满 CPU，低优先级 GPIO 线程被饿住

当前任务优先级上：

- `IMUUartBridgeTask`：`MEDIUM`
- `GPIOManager`：`LOW`

开流后桥线程主循环会不断做这些事：

- 解析串口命令
- 处理按钮待发队列
- 推 `0x30` 姿态包
- 推 `0x32` 同步事件包

问题在于旧实现中，桥线程在 streaming 状态下即使本轮没有实际完成有效工作，也会继续空转，不主动 `Sleep()`。

结果就是：

- 高优先级 bridge 线程长期占用 CPU；
- 低优先级 GPIO 线程拿不到调度片；
- 开流状态下按键扫描线程甚至跑不到“检测边沿 -> 入队”这一步；
- 所以：
  - `UART5` 没有按钮调试日志；
  - NV 侧当然也收不到按钮帧。

这个判断最终是被诊断日志证实的：

- 关流时有 `queue_push_ok`；
- 开流后连 `queue_push_ok` 都没有；
- 说明根本不是“发出去了没收到”，而是“事件没进入桥队列”。

---

## 修复内容

### 修复 1：GPIO 线程不再直接发 UART

把按钮发包从“GPIO 线程直接发桥协议”改成“GPIO 线程只入队，桥线程统一发送”。

新路径变成：

```text
GPIOManager 线程
-> PublishGPIOButtonCommandCallback()
-> IMUUartBridgeTask::PublishGPIOButtonCommand()
-> 入 gpio_button_queue_

IMUUartBridgeTask 线程
-> PublishPendingGPIOButtonCommand()
-> SendResponse(0x33)
-> USART1 发出
```

这样做的好处：

1. GPIO 层只负责产生事件，不再负责桥协议；
2. `USART1` 发送统一收口到 bridge 线程；
3. 避免 GPIO 线程和 bridge 线程并发写串口；
4. 消除了最早那种按键触发即 `HardFault` 的路径。

涉及文件：

- `application/imu_uart_bridge_task.hpp`
- `application/imu_uart_bridge_task.cpp`
- `User/app_main.cpp`
- `managers/gpio_manager.hpp`
- `managers/gpio_manager.cpp`

---

### 修复 2：按钮输入统一改成上拉输入

把按钮引脚改成：

```text
Direction = INPUT
Pull      = UP
```

既在 Cube 生成的 `gpio.c` 中改，也在 `app_main.cpp` 中运行时再次显式 `SetConfig()`，避免后续生成代码或板级差异导致配置飘掉。

涉及文件：

- `Core/Src/gpio.c`
- `User/app_main.cpp`

这个修改解决的是“输入不稳定 / 漂浮 / 某些键表现异常”的问题。

---

### 修复 3：GPIO 线程栈提高

虽然 `HardFault` 的主因后来证明不只是“栈太小”，但为了减少局部风险，还是把 GPIO 线程栈提高到了更稳妥的值。

在 `app_main.cpp` 里：

- `gpio_config.stack_size = 1024`

这不是核心根因修复，但它降低了调试和后续扩展时的脆弱性。

---

### 修复 4：桥线程在“没做成事 / 遇到回压”时主动让出 CPU

这是最终解决“开流后按键无响应”的关键点。

Bridge 主循环改成：

1. 每轮统计是否真正做成了工作：
   - 是否处理到命令；
   - 是否成功发出按钮；
   - 是否成功发出姿态；
   - 是否成功发出同步事件。
2. 如果本轮：
   - 没做成任何有效工作；
   - 或者按钮发送回压；
   - 或者姿态发送回压；
3. 就主动：

```cpp
Thread::Sleep(1);
```

这一步不会改协议语义，也不会提升按钮业务优先级，只是避免 bridge 线程在开流时空转霸占 CPU。

这是这次问题真正的闭环修复。

---

## 为什么“提高按钮优先级抢占 IMU”不是最终方案

中间我们尝试过一种更激进的做法：

- 如果按钮帧待发且串口发生回压，就先不继续推 IMU，把按钮事件优先送出去。

这个做法最后被回撤了，原因很明确：

1. 它对按钮是友好的；
2. 但它会增加 YIS 队列堆积概率；
3. 而当前 YIS -> bridge 这条链路并不是 lossless：
   - `yis_queue_` 深度只有 32；
   - topic 到队列的 `Push()` 失败没有显式处理；
   - bridge 线程还会把一批 YIS 样本 `Pop` 掉后只保留 latest。

所以这类“按钮抢占 IMU”的方案，不适合作为最终修复。

最终保留的方案是：

- 不抢占 IMU 语义；
- 只通过 bridge 线程主动 `Sleep(1)` 来恢复任务公平调度。

---

## 诊断方案

为了分清问题到底停在哪一层，这次还加了一组按钮链路诊断。

桥侧诊断字段：

- `queue_push_ok`
- `queue_push_full`
- `queue_pop_ok`
- `send_ok`
- `send_fail`

通过 `UART5` 打印成：

```text
[gpio-diag] stage=queue_push_ok cmd=A q=1 push_ok=3 push_full=0 pop_ok=2 send_ok=2 send_fail=0 pending=0
```

这套日志的用途是：

- `queue_push_ok`
  - GPIO 线程已经扫描到并成功入桥队列
- `queue_push_full`
  - GPIO 线程扫描到了，但桥按钮队列满了
- `queue_pop_ok`
  - bridge 线程已经取到按钮事件
- `send_ok`
  - bridge 线程已成功把按钮桥帧交给串口发送链路
- `send_fail`
  - bridge 线程取到了按钮事件，但发串口失败

这次最终定位“开流后按键无响应”就是靠这个诊断得到的。

---

## 验证结果

最终验证表现是：

1. 关流时：
   - `UART5` 可以看到 `queue_push_ok`
   - NV 侧可以看到：
     - `gpio_button,command,A`
     - `gpio_button,command,B`
     - `gpio_button,command,C`

2. 开流前版本：
   - 开流后 `UART5` 完全没有按钮诊断
   - 说明事件没进队列，不是对端问题

3. 开流后修复版本：
   - 对端可以正常收到按钮事件
   - 说明 bridge 线程主动让出 CPU 的修复生效

目前剩余观察点：

- `D = PB4` 仍然疑似硬件连接问题
- 软件侧已确认桥协议和调度链路基本闭合

---

## 涉及文件

本轮和按钮桥接问题直接相关的文件：

- `Core/Src/gpio.c`
- `User/app_main.cpp`
- `application/imu_uart_bridge_task.hpp`
- `application/imu_uart_bridge_task.cpp`
- `managers/gpio_manager.hpp`
- `managers/gpio_manager.cpp`

---

## 经验总结

这次问题有两个很值得记住的教训。

### 1. 事件源线程不要直接做串口桥协议发包

像 GPIO 这种低层事件线程，应该只报告事件，不应该直接持有或驱动高层通信链路。

正确的做法是：

- 事件源线程 -> 入队
- 协议线程 -> 统一发包

### 2. “没收到”不一定是串口问题，也可能是任务调度问题

这次最容易误判的点就是：

- NV 没收到按钮事件

表面看像串口没发出，实际上是：

- GPIO 线程在开流时根本没机会运行到入队步骤

所以在 RTOS 系统里，排查“事件丢失”时一定要把链路拆开看：

```text
输入采样
-> 入队
-> 协议线程取队
-> 串口发送
-> 对端接收
-> 对端解析
```

只看最后一段，很容易把根因看错。


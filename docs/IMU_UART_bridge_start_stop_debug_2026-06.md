# IMU UART Bridge start/stop Debug 记录

## 背景

本轮修改引入了 `CMD_STREAM_CONTROL=0x03`，用于控制 MCU 侧 YIS pose 推送和 TIM2/TIM5 sync event 推送的启动与停止。

测试过程中出现两个表象：

- 有时卡在 FreeRTOS `heap_4.c` 的如下断言：

```c
configASSERT( ( ( ( size_t ) pxNewBlockLink ) & portBYTE_ALIGNMENT_MASK ) == 0 );
```

- 少数情况下直接进入 `HardFault`。

表面上看像是 `pvPortMalloc()` 自己异常，但这个断言更常见的含义是：

- heap 空闲链表元数据被破坏；
- 或者更早之前已经发生了越界/悬挂访问，这里只是下一次分配时才暴露。

## 初步排查

先排除了几类误判：

- `CMD_STREAM_CONTROL start` 路径本身没有新增明显的大块动态分配；
- `SendResponse()` 发送帧会拷贝到 UART 写队列，不是直接保留栈上 buffer 指针；
- `StartAll()`/`StopAll()` 会改变同步 PWM 和事件队列状态，但不是直接调用 `malloc` 的热点。

因此排查方向转向：

- start/stop 边界上的 ISR/任务并发；
- I2C / UART DMA 异步回调是否会访问已经失效的对象；
- 任务栈是否过紧，导致踩堆后表现在 heap_4。

## 根因

最终定位到主因在 `STM32I2C` 的 DMA 阻塞等待实现。

### 问题链路

上层很多读寄存器路径是这样的：

1. 在函数栈上创建 `Semaphore sem(0)`；
2. 用这个 `sem` 构造 `ReadOperation op(sem, timeout)`；
3. 调用 `i2c_->MemRead(..., op)`；
4. `STM32I2C` 在 DMA 分支里把 `op` 拷贝到成员 `read_op_`；
5. 如果 DMA 正常在超时前完成，回调里 `read_op_.UpdateStatus(...)`，事情结束。

问题在于 DMA **超时** 的场景：

1. `MemRead(..., op)` 阻塞等待超时返回；
2. 上层函数退出，栈上的 `Semaphore sem` 被析构；
3. 但 `STM32I2C::read_op_` 里仍然保留着指向这个已析构 semaphore 的指针；
4. 稍后如果 DMA 完成回调或错误回调迟到到来，`read_op_.UpdateStatus(...)` 会去 `Post` 这个失效 semaphore；
5. 结果是访问已经失效的 FreeRTOS 内核对象，进而破坏 heap 或触发异常。

### 为什么会表现成 heap_4 对齐断言

因为被破坏的是 FreeRTOS 内部对象/堆结构，真正出错位置未必在 I2C 回调当下。

常见表现是：

- 当场 HardFault；
- 若破坏较轻，后续某次 `new` / `pvPortMalloc()` 才在 `heap_4.c` 中发现空闲块链表异常，于是卡在对齐断言。

这也是为什么现象看起来像 “start 命令导致 malloc 崩”，但实际根因在更早的异步回调。

## 修复

### 1. 修复 I2C DMA 超时后的悬挂 operation

修改文件：

- `Middlewares/Third_Party/LibXR/driver/st/stm32_i2c.cpp`
- `Middlewares/Third_Party/LibXR/driver/st/stm32_i2c.hpp`

修复内容：

- 为 DMA 阻塞等待增加统一的 `WaitBlockingDma()`；
- 一旦等待超时，先清掉驱动内保存的 `read_op_ / write_op_ / read_buff_`；
- 然后调用 `HAL_I2C_Master_Abort_IT()` 请求终止当前传输；
- 在 `HAL_I2C_*CpltCallback`、`HAL_I2C_ErrorCallback`、`HAL_I2C_AbortCpltCallback` 里都清理 pending operation；
- DMA 启动失败时立即返回，不再把一个根本没成功启动的传输标记为 pending。

这样即使回调迟到到来，也只会看到空 operation，不会再去访问已析构 semaphore。

### 2. 收紧 sync start/stop 边界

修改文件：

- `managers/sync_signal_manager.cpp`
- `managers/sync_signal_manager.hpp`
- `User/app_main.cpp`

修复内容：

- `StartAll()` 前先停旧 PWM，再重置 session 状态；
- `StopAll()` 先把队列标记为 inactive 并清空，再停 PWM，减少 stop 边界上的中断残留写入；
- `EnableTimUpdateInterrupt()` 里在启用 update interrupt 前先 disable/clear flag/reset counter，减少启动瞬间吃到旧 pending update 的机会；
- `ToSessionTickUs()` 在 inactive 状态下直接返回 `0`，避免 stop 后旧时间轴继续被当作有效相对时间。

### 3. 适度增加任务栈

修改文件：

- `User/app_main.cpp`

将以下任务栈从 `1024` 提高到 `1536`：

- `YISIMUAcquisitionTask`
- `IMUUartBridgeTask`

这不是主因修复，但能降低局部数组、打包帧、日志格式化带来的栈风险。

## 结果

修复后连续执行 start/stop 测试未再复现：

- `heap_4.c` 对齐断言；
- 随机 `HardFault`；
- start/stop 边界上的明显异常。

## 经验结论

这次问题说明一条很重要的原则：

**在 MCU 上做“阻塞等待 DMA 完成”的封装时，如果 operation 内部引用了调用栈对象，就必须保证超时返回后，任何迟到回调都不能再访问这份 operation。**

否则这类 bug 很容易伪装成：

- `malloc` 崩；
- FreeRTOS 队列/信号量异常；
- 偶发 HardFault；
- 只有高频 start/stop 或串口压力测试下才复现。

## 后续建议

- 后续若继续扩展 DMA 异步接口，优先审查所有 “成员里缓存 operation，operation 又指向栈对象” 的路径；
- 若再出现 heap_4 断言，不要先怀疑 `malloc`，先回查最近一次异步回调是否可能访问失效对象；
- start/stop 类控制接口一旦涉及 ISR 事件源，默认就要按“边界竞态”去设计，而不是只看功能逻辑是否通顺。

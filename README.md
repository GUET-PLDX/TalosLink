# TalosLink

Talos 视觉通信的 MCU 端 C++20 协议组件，依据 `/home/wanqiq/Reference` 中的
`packet.hpp`、`stm32.hpp`、`serial.hpp` 和 `serial_protocol_test.cpp` 实现。
收发端口、回调、数据视图和紧凑布局统一使用 LibXR，可用于 STM32F407。
LibXR 回调在初始化时创建并分配内存；编解码和 Poll/Send 不额外分配内存，
不使用异常或 RTTI。

协议核心采用单头文件布局，独立使用方包含 `TalosLink.hpp`：

```text
TalosLink.hpp              # 协议结构、编码、流式解析和 LibXR 端口适配
CMakeLists.txt             # xrobot 头文件路径 / 独立测试构建
README.md                 # 使用说明及线协议
tests/talos_link_test.cpp  # 主机端协议测试
```

`Pldx::Talos::TalosLink` 接收完整视觉指令和简化视觉指令；回调只报告有效帧。
UART 和 USB CDC 均通过 LibXR ReadPort/WritePort 收发。
调用方负责在线超时、控制模式及开火安全判断。

```cpp
using Link = Pldx::Talos::TalosLink;
auto vision = Link::VisionCallback::Create(
    [](bool in_isr, State* state, const Pldx::Talos::SendVisionData& FRAME) {
      state->OnVision(FRAME);
    }, &state);
Link link(*uart.read_port_, *uart.write_port_, vision);
auto rx_result = link.Poll();
LibXR::WriteOperation operation;
auto tx_result = link.Send(imu_frame, operation);
// 检查 rx_result / tx_result；USB CDC 使用其对应的 LibXR 端口。
```

`Poll()` 消费调用开始时已排队的接收字节，不等待新数据、不创建挂起读操作；
半帧保留到下一次调用。`Send()` 返回 LibXR 原始错误码（例如 BUSY、FULL、
NOT_SUPPORT），由调用方决定重试；异步发送返回成功表示已提交，完成状态由
传入的 WriteOperation 管理。端口发送队列必须能容纳完整反馈帧，最大 37 字节。
端口及异步操作引用的回调/轮询状态需存活至操作完成。

也可用 `Link link(vision, simple_vision)` 仅解析数据，调用
`link.Feed(LibXR::ConstRawData(bytes.data(), bytes.size()), in_isr)`。
请显式传递地址和字节数，尤其不能直接将 vector/span 转换为 ConstRawData。
`Encode()` 仍返回拥有存储的字节数组，避免返回悬空视图。
回调同步执行，Feed 的 ISR 标志会透传；Poll 在所属线程中调用，回调标志为 false。
每个实例及其接收端口由单一线程拥有，回调中不要重入 Feed/Poll；
帧引用仅在回调期间有效。回调应在初始化时创建并长期持有。

## 线协议

小端、IEEE754 32 位 float、紧凑布局。帧头 `5A len id`，帧尾 `A5`。
`len` 是业务载荷长度，排除帧头、时间戳和帧尾；视觉指令没有时间戳。

| 方向 | ID | len | 总长度 | 内容 |
|---|---|---|---|---|
| MCU → host | 01 | 29 | 37 | 时间戳、颜色、弹速、姿态、角速度 |
| MCU → host | 03 | 3 | 11 | 时间戳、following / power_rune / quanta |
| MCU → host | 03 | 13 | 21 | 时间戳、颜色、简化姿态 |
| host → MCU | 02 | 29 | 33 | 开火建议、目标角、速度/加速度前馈、距离 |
| host → MCU | 04 | 4 | 8 | 目标 yaw |

角度单位 rad，角速度 rad/s，距离 m，时间戳 ms。颜色 RED=0 / BLUE=1。
接收支持半帧、粘包、噪声恢复、错误帧尾恢复，拒绝非法 bool、非有限浮点数和负距离。
协议没有 CRC；帧头帧尾不能保证检测所有传输错误。

参考中的 Quanta 结构也保留在 `TalosLink.hpp`，但其载荷为 300 字节，
超出 uint8_t len 的表示范围，不能按参考串口长度公式传输。当前不解析该帧；
启用前需要与上位机明确分包或升级长度字段。

## 构建与验证

```sh
cmake -S . -B build -DLIBXR_SOURCE_DIR=/path/to/LibXR -DLIBXR_NO_EIGEN=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

也可作为 xrobot 模块目录使用，CMake 会向已有 `xr` 添加头文件路径。
独立构建需要提供真实 LibXR 源码路径，测试覆盖协议解析和端口队列收发。
协议核心提供显式调用接口；固件 Application 和 YAML 接线见下文。

## xrobot 固件接入

`TalosLink.hpp` 同时提供协议核心 `Pldx::Talos::TalosLink` 和
固件模块 `TalosLink : LibXR::Application`。实例独占 `usb_talos_cdc`，线程每 2 ms
轮询接收和发送完整 IMU 帧，每 100 ms 发送能力帧。反馈时间戳为开机毫秒数，
姿态来自 `gimbal_euler`，角速度来自 `gimbal_gyro`（rad/s），弹速来自
`launcher_ref`，队伍颜色来自 `sentry_ref.robot_status.robot_id`。未收到裁判数据时
弹速为 0、颜色为 RED；power_rune 和 quanta 为 0。

完整视觉指令映射 yaw/pitch、速度与加速度前馈、开火建议到 CMD::Data。
简化指令保持最近 AHRS pitch、仅控制 yaw，并清除开火建议和前馈。
150 ms 无有效视觉帧后，模块将 AI 状态置为离线并清除开火建议；
CMD 继续执行遥控器在线和控制模式门控。USB 队列满时丢弃本次 IMU
采样，下一轮发送最新状态；能力帧成功提交后才更新发送周期。

云台配置见 `User/RobotConfig/sentry_gimbal.yaml`，该 USB 使用 Talos 线协议。
主机须对应发送 `02/len=29` 或 `04/len=4` 视觉帧、接收 `01/len=29`
IMU 和 `03/len=3` 能力帧。独立协议测试定义 `TALOS_LINK_PROTOCOL_ONLY`，仅编译同一头文件中的协议核心；
固件默认包含完整 Application。

# TalosLink

Talos 视觉通信的 MCU 端 C++20 协议组件，依据 `/home/wanqiq/Reference` 中的
`packet.hpp`、`stm32.hpp`、`serial.hpp` 和 `serial_protocol_test.cpp` 实现。
无堆分配、异常、RTTI 或操作系统依赖，可用于 STM32F407 / LibXR。

`Pldx::Talos::TalosLink` 接收完整视觉指令和简化视觉指令；回调只报告有效帧。
调用方负责 UART/USB 读写、在线超时、控制模式及开火安全判断。

```cpp
Pldx::Talos::TalosLink link(context, on_vision, on_simple_vision);
link.Feed(received_bytes);  // std::span<const uint8_t>
auto tx = Pldx::Talos::TalosLink::Encode(imu_frame);
// 由调用方发送 tx；每个实例由单一线程拥有，回调中不要重入 Feed。
```

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

参考中的 Quanta 结构也保留在 `TalosPacket.hpp`，但其载荷为 300 字节，
超出 uint8_t len 的表示范围，不能按参考串口长度公式传输。当前不解析该帧；
启用前需要与上位机明确分包或升级长度字段。

## 构建与验证

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

也可作为 xrobot 模块目录使用，CMake 会向 `xr` 添加头文件路径。
目前提供显式调用的协议接口，尚未注册 Application、线程、topic 或 YAML 运行实例。

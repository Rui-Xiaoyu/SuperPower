# SuperPower

`SuperPower` 是主控侧的超级电容 CAN 通信模块，负责接收 `0x052` 反馈帧、同步裁判系统功率信息，并把控制帧发回超电控制板。

这个模块只做通信和状态缓存，不负责电机限幅计算，也不负责 UI 绘制。功率控制由 `PowerControl` 使用这里提供的实测功率、输出能力和在线状态完成。

## 职责

- 接收超电反馈帧 `0x052`
- 解析底盘功率、裁判系统总功率、底盘功率限制、输出能力和状态码
- 订阅 `chassis_ref` 话题，缓存裁判系统底盘功率上限和缓冲能量
- 收到有效反馈帧后，在 CAN 接收回调里按 `5 ms` 最小间隔发送控制帧 `0x061`
- 使用最后接收时间判断离线，掉线后清空对外状态

## 协议

| 项目 | 约定 |
|---|---|
| CAN 类型 | Classic CAN |
| 帧格式 | 标准帧 |
| 数据长度 | 8 字节 |
| 字节序 | STM32 本地小端 |
| 反馈帧 ID | `0x052`，超电到主控 |
| 控制帧 ID | `0x061`，主控到超电 |

代码使用 packed 结构体描述 8 字节数据区，并通过 `memcpy` 在 CAN 数据区和结构体之间转换。

## 反馈帧

反馈帧由超电控制板发送给主控，标准帧 ID 为 `0x052`。

```cpp
struct __attribute__((packed)) StatusData {
  uint8_t status_code;
  uint16_t chassis_power;
  uint16_t referee_power;
  uint16_t chassis_power_limit;
  uint8_t cap_energy;
};
```

| 偏移 | 字段 | 类型 | 含义 |
|---:|---|---|---|
| 0 | `status_code` | `uint8_t` | 超电状态码 |
| 1 | `chassis_power` | `uint16_t` | 底盘实际功率编码值 |
| 3 | `referee_power` | `uint16_t` | 裁判系统总输出功率编码值 |
| 5 | `chassis_power_limit` | `uint16_t` | 超电反馈的底盘功率限制 |
| 7 | `cap_energy` | `uint8_t` | 输出能力原始值，范围 `0~255` |

`chassis_power` 和 `referee_power` 按下面公式解码后对外提供：

```cpp
power_w = (static_cast<float>(encoded) - 16384.0f) / 64.0f;
```

`cap_energy` 通过 `GetCapEnergy()` 以归一化比例对外提供，返回 `cap_energy / 255.0f`。

## 控制帧

控制帧由主控发送给超电控制板，标准帧 ID 为 `0x061`。

```cpp
struct __attribute__((packed)) CommandData {
  uint8_t enable_dcdc : 1;
  uint8_t system_restart : 1;
  uint8_t reserved0 : 3;
  uint8_t clear_error : 1;
  uint8_t enable_active_charging_limit : 1;
  uint8_t use_new_feedback_message : 1;
  uint16_t referee_power_limit;
  uint16_t referee_energy_buffer;
  uint8_t active_charging_limit_ratio;
  int16_t reserved2;
};
```

| 字段 | 当前写入 |
|---|---|
| `enable_dcdc` | 固定置 `1` |
| `use_new_feedback_message` | 固定置 `1`，选择 `0x052` 反馈帧 |
| `referee_power_limit` | `chassis_ref.rs.chassis_power_limit` |
| `referee_energy_buffer` | `chassis_ref.power_buffer` |
| 其他字段 | 默认 `0` |

模块不单独开线程发送控制帧。收到有效反馈帧后，CAN 接收回调会检查距离上一次发送是否已经超过 `5 ms`，满足条件才发送一帧。构造完成时会强制发送一次控制帧，请求超电使用 `0x052` 反馈帧。

## 在线判定

模块启动后，在收到第一帧有效反馈帧之前认为离线。最后一次有效反馈超过 `1000 ms` 后认为离线，并清空对外状态。

## 对外接口

| 接口 | 在线返回 | 离线返回 |
|---|---|---|
| `GetChassisPower()` | 解码后的底盘实际功率，单位 W | `0` |
| `GetRefereePower()` | 解码后的裁判系统总输出功率，单位 W | `0` |
| `GetChassisPowerLimit()` | 超电反馈的底盘功率限制 | `0` |
| `GetSuperPowerOutputMax()` | 兼容接口，等同于 `GetChassisPowerLimit()` | `0` |
| `GetPowerLimit()` | 底盘功率限制钳位到 `uint8_t` | `0` |
| `GetCapEnergy()` | `cap_energy / 255.0f` | `0` |
| `GetCapEnergyRaw()` | `cap_energy` 原始值 | `0` |
| `GetStatusCode()` | 超电状态码 | `0` |
| `IsPowerStageOn()` | `status_code` bit7 | `false` |
| `IsNewFeedbackFormat()` | `status_code` bit6 | `false` |
| `GetErrorLevel()` | `status_code` 低 2 位 | `NO_ERROR` |
| `IsOnline()` | `true` | `false` |

## YAML 配置

最小配置如下：

```yaml
- id: superpower
  name: SuperPower
  constructor_args:
    can_bus_name: can1
```

`can_bus_name` 必须对应 `User/app_main.cpp` 中已经注册的 CAN 设备。系统里还需要有 `Referee` 模块持续发布 `chassis_ref` 话题，否则控制帧里的裁判系统功率信息只会保持默认值或上一次缓存值。

## 模块声明

Required Hardware:

- can

Depends:

- qdu-future/Referee

代码入口：

- `Modules/SuperPower/SuperPower.hpp`

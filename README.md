# SuperPower

`SuperPower` 是主控侧的安合超级电容 CAN 通信模块，负责同步裁判系统功率信息、发送超电控制帧和查询帧，并缓存超电反馈状态。

这个模块只做通信和状态缓存，不负责电机限幅计算，也不负责 UI 绘制。功率控制由 `PowerControl` 使用这里提供的实测功率、剩余能量比例和在线状态完成。

## 职责

- 发送超电使能指令帧 `0x600`
- 发送输入功率上限设定帧 `0x601`
- 发送输出电压设定帧 `0x602`
- 发送输出侧信息读取远程帧 `0x612`
- 接收 `0x600` 到 `0x613` 范围内的标准反馈帧
- 订阅 `chassis_ref` 话题，根据裁判系统功率上限和缓冲能量更新输入功率上限
- 收到有效反馈帧后，在 CAN 接收回调里按 `10 ms` 最小间隔下发控制帧和查询帧
- 使用最后接收时间判断在线状态，掉线后清空对外状态

## 协议

| 项目 | 约定 |
|---|---|
| CAN 类型 | Classic CAN |
| 帧格式 | 标准帧 |
| 数据长度 | 8 字节 |
| 数值分辨率 | 功率、电压、电流字段按 `/100` 还原 |
| 反馈过滤范围 | `0x600` 到 `0x613` |

多字节字段按高字节在前写入和读取。

## 发送帧

### `0x600` 指令帧

`0x600` 为标准数据帧。数据区前 2 字节写入指令字：

| 状态 | 写入值 |
|---|---:|
| 在线 | `2` |
| 离线 | `0` |

### `0x601` 输入功率上限设定帧

`0x601` 为标准数据帧。数据区前 2 字节写入输入功率上限，单位按 `W * 100` 编码。

功率上限来自 `chassis_ref`：

```cpp
power_limit =
    chassis_power_limit + 5.0f * ((40.0f - power_buffer) / 40.0f);
```

### `0x602` 输出电压设定帧

`0x602` 为标准数据帧。数据区前 2 字节固定写入 `2300`，表示 `23.00 V`。

### `0x612` 输出侧信息读取帧

`0x612` 为标准远程帧，用于请求输出侧功率、电容电压和输出电流。

## 接收帧

### `0x612` 输出侧信息

| 偏移 | 字段 | 类型 | 对外状态 |
|---:|---|---|---|
| 0 | `output_power` | `uint16_t` | `GetChassisPower()` |
| 2 | `cap_volt` | `uint16_t` | `GetCapVolt()` |
| 4 | `output_curr` | `uint16_t` | `GetOutputCurr()` |

这三个字段都按 `/100` 还原为实际值。

### `0x600` 指令反馈

| 偏移 | 字段 | 类型 | 对外状态 |
|---:|---|---|---|
| 0 | `cap_instruct` | `uint16_t` | `GetInstruct()` |

### `0x601` 目标功率反馈

| 偏移 | 字段 | 类型 | 对外状态 |
|---:|---|---|---|
| 0 | `target_power` | `uint16_t` | `GetTargetPower()` |

字段按 `/100` 还原为 W。

### `0x602` 输出电压上限反馈

| 偏移 | 字段 | 类型 | 对外状态 |
|---:|---|---|---|
| 0 | `cap_volt_max` | `uint16_t` | `GetInfo().cap_volt_max` |

字段按 `/100` 还原为 V。

## 在线判定

模块启动后，在收到第一帧有效反馈之前认为离线。最后一次有效反馈超过 `500 ms` 后认为离线，并清空对外状态。

## 对外接口

| 接口 | 在线返回 | 离线返回 |
|---|---|---|
| `GetChassisPower()` | 输出功率，单位 W | `0` |
| `GetCapEnergy()` | 电容剩余能量比例 | `0` |
| `GetPercentage()` | 电容剩余能量比例 | `0` |
| `GetCapVolt()` | 电容电压，单位 V | `0` |
| `GetOutputCurr()` | 输出电流，单位 A | `0` |
| `GetTargetPower()` | 目标功率，单位 W | `0` |
| `GetRefereePower()` | 保留接口，等同于 `GetTargetPower()` | `0` |
| `GetSuperPowerOutputMax()` | 保留接口，等同于 `GetTargetPower()` | `0` |
| `GetPowerLimit()` | 当前下发功率上限钳位到 `uint8_t` | `0` |
| `GetInstruct()` | 当前指令字 | `0` |
| `IsOnline()` | `true` | `false` |

`GetCapEnergy()` 根据电容电压使用平方关系计算，电压范围按 `16.0 V` 到 `23.0 V` 映射到 `0.0` 到 `1.0`。

## YAML 配置

最小配置如下：

```yaml
- id: superpower
  name: SuperPower
  constructor_args:
    can_bus_name: can1
```

`can_bus_name` 必须对应 `User/app_main.cpp` 中已经注册的 CAN 设备。系统里还需要有 `Referee` 模块持续发布 `chassis_ref` 话题，否则输入功率上限只会保持默认值或上一次缓存值。

## 模块声明

Required Hardware:

- can

Depends:

- qdu-future/Referee

代码入口：

- `Modules/SuperPower/SuperPower.hpp`

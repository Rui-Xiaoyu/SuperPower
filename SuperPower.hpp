#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 超级电容电源模块
constructor_args:
  - can_bus_name: "can1"
template_args: []
required_hardware:
  - can
depends:
  - qdu-future/Referee
=== END MANIFEST === */
// clang-format on
#include <algorithm>
#include <cstdint>

#include "Referee.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"

static constexpr uint32_t CAP_INSTRUCT_ID = 0x600;    /* 指令帧 */
static constexpr uint32_t CAP_POWER_LIMIT_ID = 0x601; /* 输入功率上限设定帧 */
static constexpr uint32_t CAP_OUTPUT_VOLT_ID = 0x602; /* 输出电压设定帧 */
static constexpr uint32_t CAP_OUTPUT_CUR_ID = 0x603;  /* 输出电流设定帧 */

static constexpr uint32_t CAP_OUTPUT_ID = 0x612;  /* 输出侧信息读取帧 */
static constexpr uint32_t CAP_TP_TIME_ID = 0x613; /* 温度/运行时间读取帧 */

static constexpr uint8_t CAP_INSTRUCT_DLC = 2;
static constexpr uint8_t CAP_POWER_LIMIT_DLC = 2;
static constexpr uint8_t CAP_OUTPUT_VOLT_DLC = 2;
static constexpr uint8_t CAP_TX_DLC = 8;

static constexpr uint16_t CAP_ENABLE_COMMAND = 2;
static constexpr uint16_t CAP_VOLT_MAX = 2300; /* 23.00V */

static constexpr float CAP_V_MAX = 23.0f;
static constexpr float CAP_V_MIN = 16.0f;

static constexpr uint32_t CAP_CONTROL_PERIOD_MS = 10;
static constexpr uint32_t CAP_OFFLINE_TIMEOUT_MS = 500;

class SuperPower : public LibXR::Application {
 public:
  /* 超电运行状态缓存 */
  struct Info {
    float cap_volt = 0.0f;     /* 电容当前电压 V */
    float output_curr = 0.0f;  /* 输出电流 A */
    float output_power = 0.0f; /* 输出功率 W */
    float percentage = 0.0f;   /* 电容剩余能量比例 */
    float target_power = 0.0f; /* 超电反馈的目标功率 W */
    uint16_t cap_instruct = 0; /* 超电反馈的当前指令字 */
    float cap_volt_max = 0.0f; /* 超电设定的输出电压上限 V */
    bool online = false;       /* 超电在线状态 */
  };

  typedef struct {
    uint16_t ready : 1;
    uint16_t operate : 1;
    uint16_t alarm : 1;
    uint16_t powerswitch : 1;
    uint16_t loadswitch : 1;
    uint16_t const_vlot : 1;
    uint16_t const_cur : 1;
    uint16_t const_power : 1;
    uint16_t retain : 7;
    uint16_t err : 1;
  } ModuleState;

  typedef enum {
    NORMAL,                /* 正常 */
    INPUT_UNDERVOLT,       /* 输入欠压 */
    INPUT_OVERVOLT,        /* 输入过压 */
    INPUT_OVERCUR,         /* 输入过流 */
    INPUT_OVERPOWER,       /* 输入过功率 */
    PROTECT_OVERTP,        /* 过温保护 */
    PROTECT_LOWTP,         /* 低温保护 */
    OUTPUT_OVERVOLT,       /* 输出过压 */
    OUTPUT_OVERCUR,        /* 输出过流 */
    OUTPUT_OVERPOWER,      /* 输出过功率 */
    ZERO_OVERDRIFT,        /* 零点漂移过大 */
    REVERSE_ERRO,          /* 反接错误 */
    FAILURE_CONTROL,       /* 控制故障 */
    FAILURE_COMMUNICATION, /* 通信故障 */
    FAILURE_ERR,           /* 未知故障 */
  } CapState;

  SuperPower(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             const char* can_bus_name)
      : can_(hw.template FindOrExit<LibXR::CAN>({can_bus_name})) {
    UNUSED(app);

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, SuperPower* self, const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        },
        this);

    /* 注册 CAN 接收回调，过滤超电协议标准帧 */
    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, CAP_INSTRUCT_ID,
                   CAP_TP_TIME_ID);

    RegisterRefereeCallback();

    const uint32_t NOW_MS =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    SendControlFrames(NOW_MS, true);
  }

  void RegisterRefereeCallback() {
    auto topic_handle = LibXR::Topic::Find("chassis_ref", nullptr);
    ASSERT(topic_handle != nullptr);

    auto referee_callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, SuperPower* self,
           const Referee::ChassisPack& chassis_pack) {
          UNUSED(in_isr);
          self->power_limit_ =
              static_cast<float>(chassis_pack.rs.chassis_power_limit) +
              5.0f * ((40.0f - static_cast<float>(chassis_pack.power_buffer)) /
                      40.0f);
        },
        this);

    LibXR::Topic chassis_ref_topic(topic_handle);
    chassis_ref_topic.RegisterCallback(referee_callback);
  }

  void OnCanFrame(const LibXR::CAN::ClassicPack& pack) {
    if (!Decode(pack)) {
      return;
    }

    const uint32_t NOW_MS =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    last_rx_time_ms_ = LibXR::MillisecondTimestamp(NOW_MS);
    status_received_ = true;
    info_.percentage = CalculatePercentage();
    info_.online = true;
    SendControlFrames(NOW_MS);
  }

  /* 根据在线状态生成使能指令 */
  void UpdateInstruct() { instruct_ = info_.online ? CAP_ENABLE_COMMAND : 0; }

  /* 按协议类型组装控制帧和查询帧 */
  void Control(uint32_t can_id) {
    LibXR::CAN::ClassicPack tx{};
    tx.id = can_id;

    switch (can_id) {
      case CAP_INSTRUCT_ID: {
        tx.type = LibXR::CAN::Type::STANDARD;
        tx.dlc = CAP_TX_DLC;
        WriteUint16(tx.data, instruct_);
        break;
      }
      case CAP_POWER_LIMIT_ID: {
        tx.type = LibXR::CAN::Type::STANDARD;
        tx.dlc = CAP_TX_DLC;
        WriteUint16(tx.data, EncodeHundredths(power_limit_));
        break;
      }
      case CAP_OUTPUT_VOLT_ID: {
        tx.type = LibXR::CAN::Type::STANDARD;
        tx.dlc = CAP_TX_DLC;
        WriteUint16(tx.data, CAP_VOLT_MAX);
        break;
      }
      case CAP_OUTPUT_CUR_ID: {
        tx.type = LibXR::CAN::Type::STANDARD;
        tx.dlc = CAP_TX_DLC;
        WriteUint16(tx.data, 0);
        break;
      }
      case CAP_OUTPUT_ID:
        tx.type = LibXR::CAN::Type::REMOTE_STANDARD;
        tx.dlc = CAP_TX_DLC;
        break;
      default:
        return;
    }

    (void)can_->AddMessage(tx);
  }

  void SetPowerLimit(float power_limit) { power_limit_ = power_limit; }

  float GetPercentage() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return info_.percentage;
  }

  float GetCapEnergy() { return GetPercentage(); }

  float GetCapVolt() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return info_.cap_volt;
  }

  float GetOutputCurr() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return info_.output_curr;
  }

  float GetChassisPower() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return info_.output_power;
  }

  float GetTargetPower() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return info_.target_power;
  }

  float GetRefereePower() { return GetTargetPower(); }

  float GetSuperPowerOutputMax() { return GetTargetPower(); }

  uint8_t GetPowerLimit() {
    if (!RefreshOnlineState()) {
      return 0;
    }

    return static_cast<uint8_t>(std::clamp(power_limit_, 0.0f, 255.0f));
  }

  uint16_t GetInstruct() {
    if (!RefreshOnlineState()) {
      return 0;
    }

    return info_.cap_instruct;
  }

  bool IsOnline() { return RefreshOnlineState(); }

  const Info& GetInfo() {
    RefreshOnlineState();
    return info_;
  }

  void OnMonitor() override {}

 private:
  /* CAN 接收回调负责解析反馈并触发控制帧下发 */
  static void RxCallback(bool in_isr, SuperPower* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
    self->OnCanFrame(pack);
  }

  /* 按 CAN ID 解析反馈数据，分辨率 /100 还原为实际值 */
  bool Decode(const LibXR::CAN::ClassicPack& pack) {
    const uint8_t* raw = pack.data;
    switch (pack.id) {
      case CAP_OUTPUT_ID: {
        if (pack.dlc < 6) {
          return false;
        }
        info_.output_power = static_cast<float>(ReadUint16(raw)) / 100.0f;
        info_.cap_volt = static_cast<float>(ReadUint16(&raw[2])) / 100.0f;
        info_.output_curr = static_cast<float>(ReadUint16(&raw[4])) / 100.0f;
        break;
      }
      case CAP_INSTRUCT_ID: {
        if (pack.dlc < CAP_INSTRUCT_DLC) {
          return false;
        }
        info_.cap_instruct = ReadUint16(raw);
        break;
      }
      case CAP_POWER_LIMIT_ID: {
        if (pack.dlc < CAP_POWER_LIMIT_DLC) {
          return false;
        }
        info_.target_power = static_cast<float>(ReadUint16(raw)) / 100.0f;
        break;
      }
      case CAP_OUTPUT_VOLT_ID: {
        if (pack.dlc < CAP_OUTPUT_VOLT_DLC) {
          return false;
        }
        info_.cap_volt_max = static_cast<float>(ReadUint16(raw)) / 100.0f;
        break;
      }
      default:
        return false;
    }

    return true;
  }

  bool RefreshOnlineState() {
    if (!status_received_) {
      Offline();
      return false;
    }

    const auto NOW = LibXR::Timebase::GetMilliseconds();
    if ((NOW - last_rx_time_ms_).ToMillisecond() > CAP_OFFLINE_TIMEOUT_MS) {
      Offline();
      return false;
    }

    info_.online = true;
    return true;
  }

  void SendControlFrames(uint32_t now_ms, bool force = false) {
    const auto NOW_TIMESTAMP = LibXR::MillisecondTimestamp(now_ms);
    const auto LAST_TX_TIMESTAMP =
        LibXR::MillisecondTimestamp(last_control_tx_time_ms_);

    if (!force && (NOW_TIMESTAMP - LAST_TX_TIMESTAMP).ToMillisecond() <
                      CAP_CONTROL_PERIOD_MS) {
      return;
    }

    UpdateInstruct();
    Control(CAP_INSTRUCT_ID);
    Control(CAP_OUTPUT_VOLT_ID);
    Control(CAP_OUTPUT_ID);
    Control(CAP_POWER_LIMIT_ID);
    last_control_tx_time_ms_ = now_ms;
  }

  /* 掉线时清零所有状态 */
  void Offline() {
    info_.cap_volt = 0.0f;
    info_.output_curr = 0.0f;
    info_.output_power = 0.0f;
    info_.target_power = 0.0f;
    info_.cap_instruct = 0;
    info_.cap_volt_max = 0.0f;
    info_.percentage = 0.0f;
    info_.online = false;
  }

  /* 基于电容电压计算剩余能量百分比 */
  float CalculatePercentage() const {
    const float C_MAX = CAP_V_MAX * CAP_V_MAX;
    const float C_CAP = info_.cap_volt * info_.cap_volt;
    const float C_MIN = CAP_V_MIN * CAP_V_MIN;
    float pct = (C_CAP - C_MIN) / (C_MAX - C_MIN);
    pct = std::max(0.0f, std::min(1.0f, pct));
    return pct;
  }

  static uint16_t ReadUint16(const uint8_t* raw) {
    return static_cast<uint16_t>((static_cast<uint16_t>(raw[0]) << 8) |
                                 static_cast<uint16_t>(raw[1]));
  }

  static void WriteUint16(uint8_t* raw, uint16_t value) {
    raw[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    raw[1] = static_cast<uint8_t>(value & 0xFF);
  }

  static uint16_t EncodeHundredths(float value) {
    const float CLAMPED =
        std::clamp(value, 0.0f, static_cast<float>(UINT16_MAX) / 100.0f);
    return static_cast<uint16_t>(CLAMPED * 100.0f);
  }

  LibXR::CAN* can_;

  Info info_{};
  float power_limit_ = 0.0f;
  uint16_t instruct_ = 0;

  LibXR::MillisecondTimestamp last_rx_time_ms_ = 0.0f;
  uint32_t last_control_tx_time_ms_ = 0;
  bool status_received_ = false;
};

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
#include <cstring>

#include "Referee.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"

/* 超电反馈帧 CAN 标准 ID, 超电发给主控 */
#define FEEDBACK_ID 0x52
/* 超电控制帧 CAN 标准 ID, 主控发给超电 */
#define COMMAND_ID 0x61

class SuperPower : public LibXR::Application {
 public:
  /* 0x52 反馈数据 */
  struct __attribute__((packed)) StatusData {
    uint8_t status_code;
    uint16_t chassis_power;
    uint16_t referee_power;
    uint16_t chassis_power_limit;
    uint8_t cap_energy;
  };

  /* 0x61 控制数据 */
  struct __attribute__((packed)) CommandData {
    uint8_t enable_dcdc : 1;                  /* 允许启动 DCDC */
    uint8_t system_restart : 1;               /* 系统重启 */
    uint8_t reserved0 : 3;                    /* 协议保留 */
    uint8_t clear_error : 1;                  /* 清除错误 */
    uint8_t enable_active_charging_limit : 1; /* 启用主动充电限制 */
    uint8_t use_new_feedback_message : 1;     /* 选择 0x52 反馈帧 */

    uint16_t referee_power_limit;        /* 裁判系统功率限制 W */
    uint16_t referee_energy_buffer;      /* 裁判系统缓冲能量 J */
    uint8_t active_charging_limit_ratio; /* 允许启动 DCDC 的能量比例 */
    int16_t reserved2;                   /* 协议保留 */
  };

  enum class ErrorLevel : uint8_t {
    NO_ERROR = 0,
    ERROR_RECOVER_AUTO = 1,   /* 可自动恢复 */
    ERROR_RECOVER_MANUAL = 2, /* 需要手动恢复 */
    ERROR_UNRECOVERABLE = 3,  /* 不可恢复 */
  };

  SuperPower(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             const char* can_bus_name)
      : can_(hw.template FindOrExit<LibXR::CAN>({can_bus_name})) {
    UNUSED(app);

    std::memset(&command_data_, 0, sizeof(command_data_));
    command_data_.enable_dcdc = 1;
    command_data_.use_new_feedback_message = 1;

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, SuperPower* self, const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        },
        this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, FEEDBACK_ID, FEEDBACK_ID);

    RegisterRefereeCallback();

    const uint32_t NOW_MS =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    SendCommandFrame(NOW_MS, true);
  }

  void RegisterRefereeCallback() {
    auto topic_handle = LibXR::Topic::Find("chassis_ref", nullptr);
    ASSERT(topic_handle != nullptr);

    auto referee_callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, SuperPower* self,
           const Referee::ChassisPack& chassis_pack) {
          UNUSED(in_isr);
          self->command_data_.referee_power_limit =
              chassis_pack.rs.chassis_power_limit;
          self->command_data_.referee_energy_buffer = chassis_pack.power_buffer;
        },
        this);

    LibXR::Topic chassis_ref_topic(topic_handle);
    chassis_ref_topic.RegisterCallback(referee_callback);
  }

  void OnFeedbackFrame(const LibXR::CAN::ClassicPack& pack) {
    if (pack.dlc < sizeof(StatusData)) {
      return;
    }

    StatusData data{};
    std::memcpy(&data, pack.data, sizeof(StatusData));
    DecodeStatusData(data);
    status_received_ = true;
    last_rx_time_ms_ = LibXR::Timebase::GetMilliseconds();

    const uint32_t NOW_MS = static_cast<uint32_t>(last_rx_time_ms_);
    SendCommandFrame(NOW_MS);
  }

  void DecodeStatusData(const StatusData& data) {
    status_code_ = data.status_code;
    chassis_power_ = DecodePower(data.chassis_power);
    referee_power_ = DecodePower(data.referee_power);
    chassis_power_limit_ = static_cast<float>(data.chassis_power_limit);
    cap_energy_ = data.cap_energy;
  }

  float GetChassisPower() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return chassis_power_;
  }

  float GetRefereePower() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return referee_power_;
  }

  float GetChassisPowerLimit() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return chassis_power_limit_;
  }

  float GetSuperPowerOutputMax() { return GetChassisPowerLimit(); }

  float GetCapEnergy() {
    RefreshOnlineState();
    return static_cast<float>(cap_energy_) / CAP_ENERGY_MAX;
  }

  uint8_t GetCapEnergyRaw() {
    RefreshOnlineState();
    return cap_energy_;
  }

  uint8_t GetStatusCode() {
    RefreshOnlineState();
    return status_code_;
  }

  uint8_t GetPowerLimit() {
    if (!RefreshOnlineState()) {
      return 0;
    }

    if (chassis_power_limit_ >= CAP_ENERGY_MAX) {
      return static_cast<uint8_t>(CAP_ENERGY_MAX);
    }

    return static_cast<uint8_t>(chassis_power_limit_);
  }

  bool IsPowerStageOn() {
    RefreshOnlineState();
    return (status_code_ >> STATUS_POWER_STAGE_BIT) & 0x01;
  }

  bool IsNewFeedbackFormat() {
    RefreshOnlineState();
    return (status_code_ >> STATUS_FEEDBACK_FORMAT_BIT) & 0x01;
  }

  ErrorLevel GetErrorLevel() {
    RefreshOnlineState();
    return static_cast<ErrorLevel>(status_code_ & STATUS_ERROR_LEVEL_MASK);
  }

  bool IsOnline() { return RefreshOnlineState(); }

  void OnMonitor() override {}

 private:
  static constexpr uint32_t COMMAND_PERIOD_MS = 5;
  static constexpr uint32_t OFFLINE_TIMEOUT_MS = 1000;
  static constexpr float POWER_ENCODE_OFFSET = 16384.0f;
  static constexpr float POWER_ENCODE_SCALE = 64.0f;
  static constexpr float CAP_ENERGY_MAX = 255.0f;
  static constexpr uint8_t STATUS_POWER_STAGE_BIT = 7;
  static constexpr uint8_t STATUS_FEEDBACK_FORMAT_BIT = 6;
  static constexpr uint8_t STATUS_ERROR_LEVEL_MASK = 0x03;

  static float DecodePower(uint16_t encoded) {
    return (static_cast<float>(encoded) - POWER_ENCODE_OFFSET) /
           POWER_ENCODE_SCALE;
  }

  static void RxCallback(bool in_isr, SuperPower* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
    self->OnFeedbackFrame(pack);
  }

  void ClearStatus() {
    chassis_power_ = 0.0f;
    referee_power_ = 0.0f;
    chassis_power_limit_ = 0.0f;
    cap_energy_ = 0;
    status_code_ = 0;
  }

  bool RefreshOnlineState() {
    if (!status_received_) {
      ClearStatus();
      return false;
    }

    const auto NOW = LibXR::Timebase::GetMilliseconds();
    if ((NOW - last_rx_time_ms_).ToMillisecond() > OFFLINE_TIMEOUT_MS) {
      ClearStatus();
      return false;
    }

    return true;
  }

  void SendCommandFrame(uint32_t now_ms, bool force = false) {
    const auto NOW_TIMESTAMP = LibXR::MillisecondTimestamp(now_ms);
    const auto LAST_TX_TIMESTAMP =
        LibXR::MillisecondTimestamp(last_command_tx_time_ms_);

    if (!force && (NOW_TIMESTAMP - LAST_TX_TIMESTAMP).ToMillisecond() <
                      COMMAND_PERIOD_MS) {
      return;
    }

    SendCommandFrame();
    last_command_tx_time_ms_ = now_ms;
  }

  void SendCommandFrame() {
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = COMMAND_ID;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = sizeof(CommandData);
    static_assert(sizeof(CommandData) == 8,
                  "CommandData must be 8 bytes for CAN");
    std::memcpy(tx_pack.data, &command_data_, sizeof(CommandData));
    can_->AddMessage(tx_pack);
  }

  LibXR::CAN* can_;

  CommandData command_data_{};
  float chassis_power_ = 0.0f;
  float referee_power_ = 0.0f;
  float chassis_power_limit_ = 0.0f;
  LibXR::MillisecondTimestamp last_rx_time_ms_ = 0.0f;
  uint32_t last_command_tx_time_ms_ = 0;
  uint8_t cap_energy_ = 0;
  uint8_t status_code_ = 0;
  bool status_received_ = false;
};

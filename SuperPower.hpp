#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 超级电容电源模块
constructor_args:
  - can_bus_name: "can1"
  - task_stack_depth: 800
  - thread_priority: LibXR::Thread::Priority::HIGH
  - referee: "@nullptr"
template_args: []
required_hardware:
  - can
depends:
  - qdu-future/Referee
  - qdu-future/CMD
=== END MANIFEST === */
// clang-format on
#include <algorithm>
#include <cstdio>
#include <cstring>

#include "Referee.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

/* 超电反馈CAN ID (新通讯格式) */
#define SUPERPOWER_FEEDBACK_ID 0x52
/* 发送给超电的命令CAN ID */
#define SUPERPOWER_CMD_ID 0x61

class SuperPower : public LibXR::Application {
 public:
  /* 新通讯格式 (0x52) 反馈数据结构 */
  struct __attribute__((packed)) TxDataNew {
    uint8_t status_code;
    uint16_t chassis_power;
    uint16_t referee_power;
    uint16_t chassis_power_limit;
    uint8_t cap_energy;
  };

  /* 发送给超电的命令数据结构 */
  struct __attribute__((packed)) CmdData {
    uint8_t enable_dcdc : 1;    /* 允许启动 DCDC */
    uint8_t system_restart : 1; /* 系统重启 */
    uint8_t resv0 : 3;
    uint8_t clear_error : 1;                  /* 清除错误 */
    uint8_t enable_active_charging_limit : 1; /* 启用主动充电限制 */
    uint8_t use_new_feedback_message : 1;     /* 是否使用新的反馈信息格式 */

    uint16_t referee_power_limit;   /* 裁判系统功率限制 W */
    uint16_t referee_energy_buffer; /* 裁判系统缓冲能量 J */
    uint8_t
        active_charging_limit_ratio; /* 允许启动 DCDC 的电容能量比例 (0-255) */
    int16_t resv2;
  };

  /* statusCode 位域定义 */
  enum class ErrorLevel : uint8_t {
    NO_ERROR = 0,
    ERROR_RECOVER_AUTO = 1,   /* 可自动恢复的错误 */
    ERROR_RECOVER_MANUAL = 2, /* 需要手动恢复的错误 */
    ERROR_UNRECOVERABLE = 3,  /* 不可恢复的错误，如过流保护触发 */
  };

  SuperPower(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
      const char* can_bus_name, uint32_t task_stack_depth,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH,
      Referee* referee = nullptr)
      : can_(hw.template FindOrExit<LibXR::CAN>({can_bus_name})) {
    UNUSED(app);

    LibXR::Memory::FastSet(&cmd_data_, 0, sizeof(cmd_data_));
    cmd_data_.use_new_feedback_message = 1; /* 默认使用新通讯格式 */
    cmd_data_.enable_dcdc = 1;              /* 默认启动DCDC */

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, SuperPower* self, const LibXR::CAN::ClassicPack& pack) {
          UNUSED(in_isr);
          if (pack.id == SUPERPOWER_FEEDBACK_ID) {
            self->last_rx_time_ms_ = LibXR::Timebase::GetMilliseconds();
            self->recv_.Push(pack);
          }
        },
        this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, SUPERPOWER_FEEDBACK_ID,
                   SUPERPOWER_FEEDBACK_ID);

    thread_.Create(this, ThreadFunction, "SuperPowerThread", task_stack_depth,
                   thread_priority);
  }

  static void ThreadFunction(SuperPower* super_power) {
    LibXR::Topic::ASyncSubscriber<Referee::ChassisPack> referee_suber(
        "chassis_ref");
    referee_suber.StartWaiting();

    auto last_time = LibXR::Timebase::GetMilliseconds();

    while (true) {
      if (referee_suber.Available()) {
        const auto chassis_pack = referee_suber.GetData();
        super_power->cmd_data_.referee_power_limit =
            chassis_pack.rs.chassis_power_limit;
        super_power->cmd_data_.referee_energy_buffer =
            chassis_pack.power_buffer;
        super_power->referee_chassis_pack_ = chassis_pack;
        referee_suber.StartWaiting();
      }
      super_power->Update();

      super_power->thread_.SleepUntil(last_time, 2);
    }
  }

  void Update() {
    constexpr float OFFLINE_TIMEOUT_S = 1.0f;

    LibXR::CAN::ClassicPack pack;
    if (recv_.Pop(pack) == LibXR::ErrorCode::OK) {
      DecodePowerData(pack);
    }

    auto now = LibXR::Timebase::GetMilliseconds();
    if (last_rx_time_ms_ > now) {
      now = last_rx_time_ms_;
    }
    const float TIME_SINCE_LAST_RX = (now - last_rx_time_ms_).ToSecondf();
    dt_ = TIME_SINCE_LAST_RX;

    /* 超过阈值未收到反馈，认为超电掉线并清空关键状态。 */
    if (TIME_SINCE_LAST_RX > OFFLINE_TIMEOUT_S) {
      online_ = false;
      chassis_power_ = 0.0f;
      referee_power_ = 0.0f;
      chassis_power_limit_ = 0.0f;
      cap_energy_ = 0;
      status_code_ = 0;
    } else {
      online_ = true;
      SendCommand();
    }
  }

  /**
   * @brief 偏移二进制解码: encoded = raw * 64 + 16384
   *        解码: raw = (encoded - 16384) / 64.0f
   *        量程 -256W ~ +768W, 分辨率 0.015625W
   */
  static float DecodePower(uint16_t encoded) {
    return (static_cast<float>(encoded) - 16384.0f) / 64.0f;
  }

  void DecodePowerData(const LibXR::CAN::ClassicPack& pack) {
    TxDataNew data;
    std::memcpy(&data, pack.data, sizeof(TxDataNew));
    status_code_ = data.status_code;
    chassis_power_ = DecodePower(data.chassis_power);
    referee_power_ = DecodePower(data.referee_power);
    chassis_power_limit_ = static_cast<float>(data.chassis_power_limit);
    cap_energy_ = data.cap_energy;
  }

  float GetChassisPower() { return chassis_power_; }

  float GetCapEnergy() { return static_cast<float>(cap_energy_) / 255.0f; }

  uint8_t GetStatusCode() { return status_code_; }

  bool IsOnline() { return online_; }

  /* statusCode 位域解析 */

  /* 功率级状态：1 启动，0 未启动（触发保护或主控板禁用）。 */
  bool IsPowerStageOn() { return (status_code_ >> 7) & 0x01; }

  /* 反馈信息格式：1 新通讯格式，0 旧通讯格式（RM2024）。 */
  bool IsNewFeedbackFormat() { return (status_code_ >> 6) & 0x01; }

  /* 错误等级（低 2 位）。 */
  ErrorLevel GetErrorLevel() {
    return static_cast<ErrorLevel>(status_code_ & 0x03);
  }

  void OnMonitor() override {}

 private:
  /**
   * @brief 通过CAN发送命令给超电
   */
  void SendCommand() {
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = SUPERPOWER_CMD_ID;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = sizeof(CmdData);
    static_assert(sizeof(CmdData) == 8, "CmdData must be 8 bytes for CAN");
    std::memcpy(tx_pack.data, &cmd_data_, sizeof(CmdData));
    can_->AddMessage(tx_pack);
  }
  LibXR::Thread thread_;

  float chassis_power_ = 0.0f;
  float referee_power_ = 0.0f;
  float chassis_power_limit_ = 0.0f;
  uint8_t cap_energy_ = 0;
  uint8_t status_code_ = 0;

  LibXR::CAN* can_;
  CmdData cmd_data_{};
  Referee::ChassisPack referee_chassis_pack_{};

  LibXR::MillisecondTimestamp last_rx_time_ms_ = 0.0f;
  LibXR::MillisecondTimestamp last_ui_tx_time_ms_ = 0.0f;
  float dt_ = 0.0f;
  bool online_ = false;

  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_{1};
};

#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 超级电容电源模块
constructor_args:
  - can_bus_name: "can1"
  - task_stack_depth: 2048
template_args: []
required_hardware:
  - can
depends: []
=== END MANIFEST === */
// clang-format on
#include <algorithm>
#include <cstring>

#include "app_framework.hpp"
#include "can.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

/*新超电ID 0X52*/
#define SUPERPOWERID 0x52
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

  /* statusCode 位域定义 */
  enum class ErrorLevel : uint8_t {
    NO_ERROR = 0,
    ERROR_RECOVER_AUTO = 1,   /* 可自动恢复的错误 */
    ERROR_RECOVER_MANUAL = 2, /* 需要手动恢复的错误 */
    ERROR_UNRECOVERABLE = 3,  /* 不可恢复的错误，如过流保护触发 */
  };

  SuperPower(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             const char* can_bus_name, uint32_t task_stack_depth)
      : can_(hw.template FindOrExit<LibXR::CAN>({can_bus_name})) {
    UNUSED(app);

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, SuperPower* self, const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        },
        this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, SUPERPOWERID,
                   SUPERPOWERID);
    thread_.Create(this, ThreadFunction, "SuperPowerThread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
  }

  static void ThreadFunction(SuperPower* super_power) {
    while (true) {
      super_power->Update();
      super_power->thread_.Sleep(2);
    }
  }

  void Update() {
    LibXR::CAN::ClassicPack pack;
    if (recv_.Pop(pack) == LibXR::ErrorCode::OK) {
      this->DecodePowerData(pack);
    }

    auto now = LibXR::Timebase::GetMilliseconds();
    if (last_rx_time_ms_ > now) {
      now = last_rx_time_ms_;
    }
    float time_since_last_rx = (now - last_rx_time_ms_).ToSecondf();

    dt_ = time_since_last_rx;

    /*如果0.1s没有收到新的数据 认为超电掉线*/
    if (time_since_last_rx > 0.1f) {
      online_ = false;
      chassis_power_ = 0.0f;
      referee_power_ = 0.0f;
      chassis_power_limit_ = 0.0f;
      cap_energy_ = 0;
      status_code_ = 0;
    } else {
      online_ = true;
    }
  }

  static void RxCallback(bool in_isr, SuperPower* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
    if (pack.id == SUPERPOWERID) {
      self->last_rx_time_ms_ = LibXR::Timebase::GetMilliseconds();
      self->PushToQueue(pack);
    }
  }

  void PushToQueue(const LibXR::CAN::ClassicPack& pack) { recv_.Push(pack); }

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

  float GetChassisPower() { return this->chassis_power_; }

  float GetRefereePower() { return this->referee_power_; }

  float GetChassisPowerLimit() { return this->chassis_power_limit_; }

  uint8_t GetCapEnergy() { return this->cap_energy_; }

  uint8_t GetStatusCode() { return this->status_code_; }

  /* statusCode 位域解析 */

  /*功率级状态 1为启动 0为未启动（触发保护或主控板禁用）*/
  bool IsPowerStageOn() { return (status_code_ >> 7) & 0x01; }

  /* 反馈信息格式 | 1为新通讯格式，0为旧通讯格式（RM2024）*/
  bool IsNewFeedbackFormat() { return (status_code_ >> 6) & 0x01; }

  /*错误等级*/
  ErrorLevel GetErrorLevel() {
    return static_cast<ErrorLevel>(status_code_ & 0x03);
  }

  bool IsOnline() { return online_; }

  void OnMonitor() override {}

 private:
  LibXR::Thread thread_;
  float chassis_power_ = 0.0f;
  float referee_power_ = 0.0f;
  float chassis_power_limit_ = 0.0f;
  uint8_t cap_energy_ = 0;
  uint8_t status_code_ = 0;
  LibXR::CAN* can_;

  LibXR::MillisecondTimestamp last_rx_time_ms_ = 0.0f;
  float dt_ = 0.0f;
  bool online_ = false;

  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_{1};
};

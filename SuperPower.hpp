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

#define SUPERPOWERID 0X51

class SuperPower : public LibXR::Application {
 public:
  /* 旧通讯格式 (0x051) 反馈数据结构 */
  struct __attribute__((packed)) TxData {
    uint8_t status_code;
    float chassis_power;
    uint16_t chassis_power_limit;
    uint8_t cap_energy;
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
      chassis_power_limit_ = 0;
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

  void DecodePowerData(const LibXR::CAN::ClassicPack& pack) {
    TxData data;
    std::memcpy(&data, pack.data, sizeof(TxData));
    status_code_ = data.status_code;
    chassis_power_ = data.chassis_power;
    chassis_power_limit_ = data.chassis_power_limit;
    cap_energy_ = data.cap_energy;
  }

  float GetChassisPower() { return this->chassis_power_; }

  uint16_t GetChassisPowerLimit() { return this->chassis_power_limit_; }

  uint8_t GetCapEnergy() { return this->cap_energy_; }

  uint8_t GetStatusCode() { return this->status_code_; }

  bool IsOnline() { return online_; }

  void OnMonitor() override {}

 private:
  LibXR::Thread thread_;
  float chassis_power_ = 0.0f;
  uint16_t chassis_power_limit_ = 0;
  uint8_t cap_energy_ = 0;
  uint8_t status_code_ = 0;
  LibXR::CAN* can_;

  LibXR::MillisecondTimestamp last_rx_time_ms_ = 0.0f;
  float dt_ = 0.0f;
  bool online_ = false;

  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_{1};
};

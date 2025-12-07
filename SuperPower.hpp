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

#include "app_framework.hpp"
#include "can.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

class SuperPower : public LibXR::Application {
 public:
  SuperPower(LibXR::HardwareContainer &hw,
             LibXR::ApplicationManager &app,
             const char *can_bus_name,
             uint32_t task_stack_depth)
      : can_(hw.template FindOrExit<LibXR::CAN>({can_bus_name})) {
    UNUSED(app);

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, SuperPower *self, const LibXR::CAN::ClassicPack &pack) {
          RxCallback(in_isr, self, pack);
        },
        this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, 0x51, 0x51);
    thread_.Create(this, ThreadFunction, "SuperPowerThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  static void ThreadFunction(SuperPower *super_power) {
    auto last_time = LibXR::Timebase::GetMilliseconds();

    while (true) {
      super_power->Update();
      super_power->thread_.SleepUntil(last_time, 2.0f);
    }
  }

  void Update() {
    LibXR::CAN::ClassicPack pack;
    if (recv_.Pop(pack) == LibXR::ErrorCode::OK) {
      this->DecodePowerData(pack);
    }
    corrected_chassis_power_ = static_cast<float>(chassis_power_ * 0.8179 + 0.4);
  }

  static void RxCallback(bool in_isr, SuperPower *self,
                         const LibXR::CAN::ClassicPack &pack) {
    UNUSED(in_isr);
    if (pack.id == 0x51) {
      self->PushToQueue(pack);
    }
  }

  void PushToQueue(const LibXR::CAN::ClassicPack &pack) { recv_.Push(pack); }

  void DecodePowerData(const LibXR::CAN::ClassicPack &pack) {
    memcpy(&chassis_power_, &pack.data[1], sizeof(float));
  }

  float GetChassisPower() { return this->corrected_chassis_power_; }

  void OnMonitor() override {}

 private:
  LibXR::Thread thread_;
  float chassis_power_ = 0.0f;
  float corrected_chassis_power_;
  LibXR::CAN *can_;

  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_{1};
};

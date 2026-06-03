#include "sync_signal_manager.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "timebase.hpp"

namespace Manager {

namespace {

constexpr uint8_t kSyncEventFlagOverflow = 0x01;
constexpr size_t kSyncEventQueueCapacity = 64;
constexpr size_t kSyncEventSourceCount = 2;

struct SyncEventQueue {
  std::array<SyncEventRecord, kSyncEventQueueCapacity> records{};
  volatile uint8_t head = 0;
  volatile uint8_t tail = 0;
  volatile uint32_t dropped_count = 0;
  volatile bool active = false;
  volatile uint32_t session_id = 0;
  volatile uint64_t session_start_mcu_tick_us = 0;
  std::array<SyncEventRecord, kSyncEventSourceCount> latest{};
  std::array<uint32_t, kSyncEventSourceCount> sequence{};
  std::array<uint32_t, kSyncEventSourceCount> nominal_period_us{};
};

SyncEventQueue sync_event_queue;
SyncSignalManager* active_manager = nullptr;

size_t SourceIndex(SyncEventSource source) {
  return (source == SyncEventSource::TIM5_CAMERA_TRIGGER_30HZ) ? 1U : 0U;
}

uint32_t NominalPeriodUs(SyncEventSource source) {
  return sync_event_queue.nominal_period_us[SourceIndex(source)];
}

uint64_t RelativeTickUs(uint64_t session_start_mcu_tick_us,
                        uint64_t absolute_mcu_tick_us) {
  return (absolute_mcu_tick_us > session_start_mcu_tick_us)
             ? (absolute_mcu_tick_us - session_start_mcu_tick_us)
             : 0U;
}

void ResetSessionStateFromCritical(uint64_t start_mcu_tick_us) {
  sync_event_queue.head = 0;
  sync_event_queue.tail = 0;
  sync_event_queue.dropped_count = 0;
  sync_event_queue.session_start_mcu_tick_us = start_mcu_tick_us;
  ++sync_event_queue.session_id;
  for (size_t i = 0; i < kSyncEventSourceCount; ++i) {
    sync_event_queue.latest[i] = SyncEventRecord{};
    sync_event_queue.sequence[i] = 0;
  }
}

}  // namespace

LibXR::ErrorCode SyncSignalManager::RegisterPwmOutput(
    const SyncPwmOutputConfig& config) {
  if (config.pwm == nullptr || config.nominal_period_us == 0U) {
    return LibXR::ErrorCode::ARG_ERR;
  }
  if (output_count_ >= outputs_.size()) {
    return LibXR::ErrorCode::NO_MEM;
  }

  const size_t source_index = SourceIndex(config.source);
  sync_event_queue.nominal_period_us[source_index] = config.nominal_period_us;
  outputs_[output_count_] = config;
  last_start_results_[output_count_] = LibXR::ErrorCode::INIT_ERR;
  ++output_count_;
  active_manager = this;
  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode SyncSignalManager::StartAll() {
  LibXR::ErrorCode final_result = LibXR::ErrorCode::OK;
  const uint64_t session_start_mcu_tick_us = LibXR::Timebase::GetMicroseconds();

  for (size_t i = 0; i < output_count_; ++i) {
    if (outputs_[i].pwm != nullptr) {
      (void)outputs_[i].pwm->Disable();
    }
  }

  taskENTER_CRITICAL();
  ResetSessionStateFromCritical(session_start_mcu_tick_us);
  sync_event_queue.active = true;
  taskEXIT_CRITICAL();

  for (size_t i = 0; i < output_count_; ++i) {
    const SyncPwmOutputConfig& output = outputs_[i];
    if (output.before_enable != nullptr) {
      output.before_enable(output.context);
    }

    const LibXR::ErrorCode ec = output.pwm->Enable();
    last_start_results_[i] = ec;
    if (ec == LibXR::ErrorCode::OK) {
      RecordEvent(output.source, session_start_mcu_tick_us);
    } else if (final_result == LibXR::ErrorCode::OK) {
      final_result = ec;
    }
  }

  if (final_result != LibXR::ErrorCode::OK) {
    (void)StopAll();
  }
  return final_result;
}

LibXR::ErrorCode SyncSignalManager::StopAll() {
  LibXR::ErrorCode final_result = LibXR::ErrorCode::OK;

  taskENTER_CRITICAL();
  sync_event_queue.active = false;
  sync_event_queue.head = 0;
  sync_event_queue.tail = 0;
  sync_event_queue.dropped_count = 0;
  for (size_t i = 0; i < kSyncEventSourceCount; ++i) {
    sync_event_queue.latest[i] = SyncEventRecord{};
  }
  taskEXIT_CRITICAL();

  for (size_t i = 0; i < output_count_; ++i) {
    if (outputs_[i].pwm == nullptr) {
      continue;
    }
    const LibXR::ErrorCode ec = outputs_[i].pwm->Disable();
    if (ec != LibXR::ErrorCode::OK && final_result == LibXR::ErrorCode::OK) {
      final_result = ec;
    }
  }

  return final_result;
}

LibXR::ErrorCode SyncSignalManager::GetLastStartResult(
    SyncEventSource source) const {
  for (size_t i = 0; i < output_count_; ++i) {
    if (outputs_[i].source == source) {
      return last_start_results_[i];
    }
  }
  return LibXR::ErrorCode::NOT_FOUND;
}

LibXR::ErrorCode SyncSignalManager::StartRegisteredOutputs() {
  if (active_manager == nullptr) {
    return LibXR::ErrorCode::INIT_ERR;
  }
  return active_manager->StartAll();
}

LibXR::ErrorCode SyncSignalManager::StopRegisteredOutputs() {
  if (active_manager == nullptr) {
    return LibXR::ErrorCode::INIT_ERR;
  }
  return active_manager->StopAll();
}

void SyncSignalManager::RecordEventFromISR(SyncEventSource source,
                                           uint64_t mcu_tick_us) {
  if (!sync_event_queue.active) {
    return;
  }

  const size_t source_index = SourceIndex(source);
  SyncEventRecord event;
  event.source = source;
  event.sequence = sync_event_queue.sequence[source_index]++;
  event.mcu_tick_us =
      RelativeTickUs(sync_event_queue.session_start_mcu_tick_us, mcu_tick_us);
  event.nominal_period_us = NominalPeriodUs(source);

  const uint8_t next_head = static_cast<uint8_t>(
      (sync_event_queue.head + 1U) % kSyncEventQueueCapacity);
  if (next_head == sync_event_queue.tail) {
    ++sync_event_queue.dropped_count;
    event.flags = kSyncEventFlagOverflow;
  }
  event.dropped_count = sync_event_queue.dropped_count;

  sync_event_queue.latest[source_index] = event;
  if (next_head == sync_event_queue.tail) {
    return;
  }

  sync_event_queue.records[sync_event_queue.head] = event;
  sync_event_queue.head = next_head;
}

void SyncSignalManager::RecordEvent(SyncEventSource source,
                                    uint64_t mcu_tick_us) {
  taskENTER_CRITICAL();
  RecordEventFromISR(source, mcu_tick_us);
  taskEXIT_CRITICAL();
}

bool SyncSignalManager::GetLatestEvent(SyncEventSource source,
                                       SyncEventRecord& event) {
  const size_t source_index = SourceIndex(source);
  taskENTER_CRITICAL();
  event = sync_event_queue.latest[source_index];
  const bool valid =
      sync_event_queue.active && sync_event_queue.sequence[source_index] != 0U;
  taskEXIT_CRITICAL();
  return valid;
}

bool SyncSignalManager::PopEvent(SyncEventRecord& event) {
  taskENTER_CRITICAL();
  if (sync_event_queue.tail == sync_event_queue.head) {
    taskEXIT_CRITICAL();
    return false;
  }

  event = sync_event_queue.records[sync_event_queue.tail];
  sync_event_queue.tail =
      static_cast<uint8_t>((sync_event_queue.tail + 1U) %
                           kSyncEventQueueCapacity);
  taskEXIT_CRITICAL();
  return true;
}

bool SyncSignalManager::IsActive() {
  taskENTER_CRITICAL();
  const bool active = sync_event_queue.active;
  taskEXIT_CRITICAL();
  return active;
}

uint32_t SyncSignalManager::GetSessionId() {
  taskENTER_CRITICAL();
  const uint32_t session_id = sync_event_queue.session_id;
  taskEXIT_CRITICAL();
  return session_id;
}

uint64_t SyncSignalManager::GetSessionStartMcuTickUs() {
  taskENTER_CRITICAL();
  const uint64_t start = sync_event_queue.session_start_mcu_tick_us;
  taskEXIT_CRITICAL();
  return start;
}

uint64_t SyncSignalManager::ToSessionTickUs(uint64_t absolute_mcu_tick_us) {
  taskENTER_CRITICAL();
  const bool active = sync_event_queue.active;
  const uint64_t start = sync_event_queue.session_start_mcu_tick_us;
  taskEXIT_CRITICAL();
  if (!active) {
    return 0U;
  }
  return RelativeTickUs(start, absolute_mcu_tick_us);
}

}  // namespace Manager

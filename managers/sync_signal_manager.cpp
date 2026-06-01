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
  std::array<SyncEventRecord, kSyncEventSourceCount> latest{};
  std::array<uint32_t, kSyncEventSourceCount> sequence{};
  std::array<uint32_t, kSyncEventSourceCount> nominal_period_us{};
};

SyncEventQueue sync_event_queue;

size_t SourceIndex(SyncEventSource source) {
  return (source == SyncEventSource::TIM5_CAMERA_TRIGGER_30HZ) ? 1U : 0U;
}

uint32_t NominalPeriodUs(SyncEventSource source) {
  return sync_event_queue.nominal_period_us[SourceIndex(source)];
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
  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode SyncSignalManager::StartAll() {
  LibXR::ErrorCode final_result = LibXR::ErrorCode::OK;
  for (size_t i = 0; i < output_count_; ++i) {
    const SyncPwmOutputConfig& output = outputs_[i];
    if (output.before_enable != nullptr) {
      output.before_enable(output.context);
    }

    const LibXR::ErrorCode ec = output.pwm->Enable();
    last_start_results_[i] = ec;
    if (ec == LibXR::ErrorCode::OK) {
      RecordEvent(output.source, LibXR::Timebase::GetMicroseconds());
    } else if (final_result == LibXR::ErrorCode::OK) {
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

void SyncSignalManager::RecordEventFromISR(SyncEventSource source,
                                           uint64_t mcu_tick_us) {
  const size_t source_index = SourceIndex(source);
  SyncEventRecord event;
  event.source = source;
  event.sequence = sync_event_queue.sequence[source_index]++;
  event.mcu_tick_us = mcu_tick_us;
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
  const bool valid = event.mcu_tick_us != 0U;
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

}  // namespace Manager

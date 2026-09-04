#include "prism_ros_adapter/driver.hpp"
#include "prism_ros_adapter/device_time_resolver.hpp"
#include "prism_ros_adapter/lidar_frame_accumulator.hpp"

#include "prism/usb_sdk.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <exception>
#include <future>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace prism_ros_adapter {
namespace {

constexpr double kStandardGravity = 9.80665;
constexpr double kRadiansPerDegree = 0.017453292519943295769;
constexpr uint16_t kVideoChunkLast = 0x0002u;

uint64_t steadyNowNs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

bool isTimeout(const std::exception& error) {
  std::string text = error.what();
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text.find("timeout") != std::string::npos ||
         text.find("timed out") != std::string::npos;
}

std::string narrow(const std::wstring& value) {
  std::string result;
  result.reserve(value.size());
  for (const wchar_t c : value) {
    if (c >= 0 && c <= 0x7f) result.push_back(static_cast<char>(c));
  }
  return result;
}

prism::LidarModel toSdkModel(LidarModel model) {
  return model == LidarModel::Mid360S ? prism::LidarModel::Mid360S
                                      : prism::LidarModel::Mid360;
}

std::string lidarModelName(prism::LidarModel model) {
  switch (model) {
    case prism::LidarModel::Mid360:
      return "mid360";
    case prism::LidarModel::Mid360S:
      return "mid360s";
    case prism::LidarModel::None:
      return "none";
  }
  return "unknown";
}

std::string lidarModelName(LidarModel model) {
  return model == LidarModel::Mid360S ? "mid360s" : "mid360";
}

DeviceState fromSdk(const prism::HelloInfo& hello,
                    const prism::DeviceVersions& versions,
                    const prism::DeviceInfo& info) {
  DeviceState output;
  output.host_sdk_version = prism::hostSdkVersion();
  output.agent_version = versions.agent;
  output.sensor_board_version = versions.sensor_board;
  output.combined_version = versions.combined;
  output.agent_protocol_version = hello.protocol_version;
  output.product_serial = info.product_serial;
  output.usb_serial = narrow(info.serial_number);
  output.vendor_id = info.vendor_id;
  output.product_id = info.product_id;
  output.info_version = info.info_version;
  output.usb_speed = prism::usbLinkSpeedName(info.usb_speed);
  output.usb3_connected = info.usb3_connected;
  output.sensor_board_online = info.sensor_board_online;
  output.sensor_board_time_synced = info.sensor_board_time_synced;
  output.detected_camera_count = info.detected_camera_count;
  output.detected_imu_count = info.detected_imu_count;
  output.camera_present_mask = info.camera_present_mask;
  output.camera_streaming_mask = info.camera_streaming_mask;
  output.imu_present_mask = info.imu_present_mask;
  output.imu_receiving_mask = info.imu_receiving_mask;
  output.imu_time_synced_mask = info.imu_time_synced_mask;
  output.imu_init_error_mask = info.imu_init_error_mask;
  for (size_t index = 0; index < output.imu_init_error_reason.size(); ++index) {
    output.imu_init_error_reason[index] =
        prism::imuInitErrorReasonName(info.imu_init_error_reason[index]);
  }
  output.camera_fps = info.camera_fps;
  output.imu_fps = info.imu_fps;
  output.sensor_board_error_code =
      static_cast<uint8_t>(info.sensor_board_error_code);
  output.sensor_board_error_flags = info.sensor_board_error_flags;
  output.sensor_board_error = info.sensor_board_error;
  return output;
}

DeviceConfigurationState fromSdk(const prism::DeviceConfiguration& value) {
  DeviceConfigurationState output;
  output.camera_fps = value.camera_fps;
  output.imu_rate_hz = value.imu_rate_hz;
  output.mjpeg_quality = value.mjpeg_quality;
  output.generation = value.generation;
  output.persisted = value.persisted;
  return output;
}

LidarStatusState fromSdk(const prism::LidarStatus& value) {
  LidarStatusState output;
  output.available = value.available;
  output.enabled = value.enabled;
  output.connected = value.connected;
  output.receiving = value.receiving;
  output.model = lidarModelName(value.model);
  output.device_type = value.device_type;
  output.handle = value.handle;
  output.packet_count = value.packet_count;
  output.point_count = value.point_count;
  output.dropped_point_count = value.dropped_point_count;
  output.serial = value.serial;
  output.lidar_ip = value.lidar_ip;
  output.error = value.error;
  return output;
}

LidarNetworkState fromSdk(const prism::LidarNetworkStatus& value) {
  LidarNetworkState output;
  output.enabled = value.configuration.enabled;
  output.host_ip = value.configuration.host_ip;
  output.netmask = value.configuration.netmask;
  output.lidar_ip = value.configuration.lidar_ip;
  output.interface_present = value.interface_present;
  output.link_up = value.link_up;
  output.address_applied = value.address_applied;
  output.same_subnet = value.same_subnet;
  output.target_reachable = value.target_reachable;
  output.persisted = value.persisted;
  output.error_code = value.error_code;
  output.generation = value.generation;
  output.interface_name = value.interface_name;
  output.error = value.error;
  return output;
}

WifiHotspotState fromSdk(const prism::WifiHotspotStatus& value) {
  WifiHotspotState output;
  output.present = value.present;
  output.enabled = value.enabled;
  output.running = value.running;
  output.access_point_running = value.ap_running;
  output.dhcp_running = value.dhcp_running;
  output.persisted = value.persisted;
  output.error_code = value.error_code;
  output.interface_name = value.interface_name;
  output.ssid = value.ssid;
  output.address = value.address;
  output.error = value.error;
  return output;
}

ExposureState fromSdk(const prism::ExposureConfiguration& value) {
  ExposureState output;
  output.automatic_camera_mask = value.automatic_camera_mask;
  output.target_brightness = value.target_brightness;
  output.manual_exposure_time_us = value.manual_exposure_time_us;
  output.gain_x1024 = value.gain_x1024;
  return output;
}

ExposureLimitsState fromSdk(const prism::ExposureLimits& value) {
  ExposureLimitsState output;
  output.min_exposure_time_us = value.min_exposure_time_us;
  output.max_exposure_time_us = value.max_exposure_time_us;
  output.effective_max_exposure_time_us =
      value.effective_max_exposure_time_us;
  output.min_gain_x1024 = value.min_gain_x1024;
  output.max_gain_x1024 = value.max_gain_x1024;
  return output;
}

SystemTimeSyncState fromSdk(const prism::SystemTimeSyncResult& value) {
  SystemTimeSyncState output;
  output.before_offset_us = value.before.offset_us;
  output.applied_correction_us = value.applied_correction_us;
  output.after_offset_us = value.after.offset_us;
  output.round_trip_us = value.after.round_trip_us;
  output.jitter_us = value.after.jitter_us;
  output.correction_passes = value.correction_passes;
  output.system_time_set = value.system_time_set;
  output.ptp_hardware_clock_set = value.ptp_hardware_clock_set;
  output.hardware_clock_set = value.hardware_clock_set;
  output.verified = value.verified;
  output.rtc_device = value.rtc_device;
  return output;
}

std::string exceptionText(std::exception_ptr error) {
  try {
    if (error) std::rethrow_exception(error);
  } catch (const std::exception& exception) {
    return exception.what();
  } catch (...) {
    return "unknown error";
  }
  return {};
}

template <typename T>
class DispatchQueue {
 public:
  using Handler = std::function<void(const T&)>;

  DispatchQueue(size_t capacity, Handler handler)
      : capacity_(capacity), handler_(std::move(handler)) {}

  ~DispatchQueue() { stop(); }

  void start() {
    if (!handler_ || running_.exchange(true)) return;
    worker_ = std::thread([this]() { workerMain(); });
  }

  void stop() {
    if (!running_.exchange(false)) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.clear();
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

  void push(T value) {
    if (!handler_ || !running_.load()) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() >= capacity_) {
        queue_.pop_front();
        dropped_.fetch_add(1);
      }
      queue_.push_back(std::move(value));
    }
    cv_.notify_one();
  }

  uint64_t dropped() const noexcept { return dropped_.load(); }

 private:
  void workerMain() {
    while (running_.load()) {
      T value;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() {
          return !running_.load() || !queue_.empty();
        });
        if (!running_.load()) break;
        value = std::move(queue_.front());
        queue_.pop_front();
      }
      try {
        handler_(value);
      } catch (...) {
        // A ROS subscriber or middleware failure must never stall USB reads.
      }
    }
  }

  size_t capacity_;
  Handler handler_;
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> dropped_{0};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<T> queue_;
  std::thread worker_;
};

struct CameraAssembly {
  bool metadata_seen = false;
  prism::VideoMeta metadata;
  uint8_t last_mask = 0;
  uint8_t complete_mask = 0;
  uint8_t invalid_mask = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  std::array<uint32_t, 4> received{};
  std::array<std::vector<uint8_t>, 4> jpeg;
};

}  // namespace

struct Driver::Impl {
  struct ControlContext {
    prism::Client& client;
    std::unique_ptr<prism::ImuStream>& imu_stream;
    std::unique_ptr<prism::LidarStream>& lidar_stream;
    uint32_t imu_sensor_count;
    bool& video_started;
    bool& imu_started;
    bool& lidar_started;
  };

  struct ControlCommand {
    std::function<void(ControlContext&)> execute;
    std::function<void(std::exception_ptr)> cancel;
  };

  Impl(DriverConfig config_in, DriverCallbacks callbacks_in)
      : config(std::move(config_in)),
        callbacks(std::move(callbacks_in)),
        camera_dispatch(2, callbacks.camera),
        board_imu_dispatch(8192, callbacks.board_imu),
        lidar_dispatch(16, callbacks.lidar_points),
        lidar_imu_dispatch(2048, callbacks.lidar_imu) {
    for (auto& counter : board_imu_samples) counter.store(0);
  }

  template <typename Result, typename Function>
  Result invokeControl(Function function) {
    auto promise = std::make_shared<std::promise<Result>>();
    auto future = promise->get_future();
    ControlCommand command;
    command.execute = [promise, function = std::move(function)](
                          ControlContext& context) mutable {
          try {
            promise->set_value(function(context));
          } catch (...) {
            promise->set_exception(std::current_exception());
          }
        };
    command.cancel = [promise](std::exception_ptr error) {
      try {
        promise->set_exception(error);
      } catch (...) {
      }
    };
    {
      std::lock_guard<std::mutex> lock(control_mutex);
      if (!accepting_controls) {
        throw std::runtime_error(
            "Prism driver is not currently accepting service commands");
      }
      control_commands.push_back(std::move(command));
    }
    return future.get();
  }

  void enableControls() {
    std::lock_guard<std::mutex> lock(control_mutex);
    accepting_controls = true;
  }

  void cancelControls(const std::string& reason) noexcept {
    std::deque<ControlCommand> pending;
    {
      std::lock_guard<std::mutex> lock(control_mutex);
      accepting_controls = false;
      pending.swap(control_commands);
    }
    const auto error =
        std::make_exception_ptr(std::runtime_error(reason));
    for (auto& command : pending) command.cancel(error);
  }

  void processControls(ControlContext& context) {
    std::deque<ControlCommand> pending;
    {
      std::lock_guard<std::mutex> lock(control_mutex);
      pending.swap(control_commands);
    }
    for (auto& command : pending) command.execute(context);
  }

  void ensureStreamObjects(ControlContext& context) {
    if (config.enable_board_imu && !context.imu_stream) {
      context.imu_stream = std::make_unique<prism::ImuStream>(
          context.client, [this](const prism::ImuSample& sample) {
            dispatchBoardImu(sample);
          });
    }
    if (config.enable_lidar && !context.lidar_stream) {
      context.lidar_stream = std::make_unique<prism::LidarStream>(
          context.client,
          [this](const prism::LidarPointBatch& batch) {
            dispatchLidarPoints(batch);
          },
          [this](const prism::LidarImuSample& sample) {
            dispatchLidarImu(sample);
          });
    }
  }

  StreamState streamState(const ControlContext& context) const {
    StreamState output;
    output.camera_enabled = config.enable_camera;
    output.board_imu_enabled = config.enable_board_imu;
    output.lidar_enabled = config.enable_lidar;
    output.camera_active = context.video_started;
    output.board_imu_active = context.imu_started;
    output.lidar_active = context.lidar_started;
    output.lidar_model = lidarModelName(config.lidar_model);
    return output;
  }

  std::string streamStateText(const ControlContext& context) const {
    return context.video_started || context.imu_started ||
                   context.lidar_started
               ? "streaming"
               : "idle";
  }

  void startConfiguredStreams(ControlContext& context) {
    ensureStreamObjects(context);
    if (config.enable_camera && !context.video_started) {
      const auto video =
          context.client.startVideo1280x1024(config.camera_fps);
      context.video_started = video.enabled;
      if (!video.enabled) throw std::runtime_error("camera start was rejected");
    }

    if (config.enable_board_imu && !context.imu_started) {
      if (!context.imu_stream) {
        throw std::runtime_error("board IMU stream is unavailable");
      }
      context.imu_stream->start(context.imu_sensor_count, config.imu_rate_hz);
      context.imu_started = true;
    }

    establishDeviceTimeReference(context.client, context.imu_stream.get());

    if (config.enable_lidar && !context.lidar_started) {
      if (!context.lidar_stream) {
        throw std::runtime_error("LiDAR stream is unavailable");
      }
      lidar_frame_accumulator.reset();
      context.lidar_stream->start(toSdkModel(config.lidar_model));
      context.lidar_started = true;
    }
  }

  template <typename Result, typename Function>
  Result runIdleOperation(ControlContext& context,
                          const std::string& state_text,
                          Function function) {
    publishStatus(state_text);
    try {
      stopConfiguredStreams(context);
    } catch (...) {
      const auto stop_error = std::current_exception();
      try {
        startConfiguredStreams(context);
        publishStatus(streamStateText(context));
      } catch (...) {
        fatal_control_error = "failed to pause streams for " + state_text +
                              ": " + exceptionText(stop_error) +
                              "; stream recovery failed: " +
                              exceptionText(std::current_exception());
        throw std::runtime_error(fatal_control_error);
      }
      std::rethrow_exception(stop_error);
    }

    camera_assemblies.clear();
    device_time_resolver.reset();
    try {
      Result result = function(context.client);
      try {
        startConfiguredStreams(context);
        publishStatus(streamStateText(context));
      } catch (...) {
        fatal_control_error = state_text +
                              " completed, but stream restart failed: " +
                              exceptionText(std::current_exception());
        throw std::runtime_error(fatal_control_error);
      }
      return result;
    } catch (...) {
      const auto operation_error = std::current_exception();
      if (!fatal_control_error.empty()) std::rethrow_exception(operation_error);
      try {
        startConfiguredStreams(context);
        publishStatus(streamStateText(context));
      } catch (...) {
        fatal_control_error = state_text + " failed: " +
                              exceptionText(operation_error) +
                              "; stream recovery failed: " +
                              exceptionText(std::current_exception());
        throw std::runtime_error(fatal_control_error);
      }
      std::rethrow_exception(operation_error);
    }
  }

  void stopConfiguredStreams(ControlContext& context) {
    if (context.lidar_started && context.lidar_stream) {
      context.lidar_stream->stop();
      context.lidar_started = false;
      lidar_frame_accumulator.reset();
    }

    if (context.imu_started && context.imu_stream) {
      // Camera and board IMU share one aggregate capture session. Either SDK
      // stop call stops both paths.
      context.imu_stream->stop();
      context.imu_started = false;
      context.video_started = false;
    } else if (context.video_started) {
      context.client.stopVideo();
      context.video_started = false;
    }
  }

  void reopenControlClient(ControlContext& context) {
    // A large CLOCK_REALTIME/PHC step can leave the Agent's aggregate capture
    // session and timestamp synchronizers tied to the previous epoch even
    // after an SDK stop/start. Reopening the USB session matches a fresh node
    // start and lets camera, board IMU, and LiDAR establish the new epoch.
    context.imu_stream.reset();
    context.lidar_stream.reset();
    context.client.close();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    context.client = openClient();
    context.client.setKeepaliveEnabled(true);

    const auto info = context.client.deviceInfo();
    usb3_connected = info.usb3_connected;
    sensor_board_online = info.sensor_board_online;
    sensor_board_time_synced = info.sensor_board_time_synced;
    product_serial = info.product_serial;
    if ((config.enable_camera || config.enable_board_imu) &&
        !info.sensor_board_online) {
      throw std::runtime_error("sensor-board is offline after time sync");
    }
    // Cameras can occupy non-contiguous slots (e.g. present_mask 0b0101 =
    // slots 0 and 2). Deriving the mask from the count would request empty
    // slots and make sensor-board capture enable time out (-110).
    camera_mask = static_cast<uint8_t>(info.camera_present_mask & 0x0fu);
    if (camera_mask == 0u) camera_mask = 0x0fu;

    ensureStreamObjects(context);
  }

  SystemTimeSyncState synchronizeSystemTime(ControlContext& context) {
    publishStatus("synchronizing_time");
    try {
      stopConfiguredStreams(context);
    } catch (...) {
      const auto stop_error = std::current_exception();
      try {
        startConfiguredStreams(context);
        publishStatus("streaming");
      } catch (...) {
        fatal_control_error =
            "failed to pause streams for time synchronization: " +
            exceptionText(stop_error) + "; stream recovery failed: " +
            exceptionText(std::current_exception());
        throw std::runtime_error(fatal_control_error);
      }
      throw;
    }

    camera_assemblies.clear();
    device_time_resolver.reset();
    prism::SystemTimeSyncResult result;
    try {
      result = context.client.synchronizeSystemTime();
    } catch (...) {
      const auto sync_error = std::current_exception();
      try {
        reopenControlClient(context);
        startConfiguredStreams(context);
        publishStatus("streaming");
      } catch (...) {
        fatal_control_error = "system time synchronization failed: " +
                              exceptionText(sync_error) +
                              "; stream recovery failed: " +
                              exceptionText(std::current_exception());
        throw std::runtime_error(fatal_control_error);
      }
      std::rethrow_exception(sync_error);
    }

    try {
      reopenControlClient(context);
      startConfiguredStreams(context);
    } catch (...) {
      fatal_control_error =
          "system time synchronized, but stream restart failed: " +
          exceptionText(std::current_exception());
      throw std::runtime_error(fatal_control_error);
    }
    publishStatus("streaming");
    return fromSdk(result);
  }

  StreamState applyStreamCommand(ControlContext& context,
                                 StreamCommand command, bool camera,
                                 bool board_imu, bool lidar) {
    if (!camera && !board_imu && !lidar) {
      throw std::invalid_argument("select at least one stream");
    }
    if (command == StreamCommand::Restart &&
        ((camera && !config.enable_camera) ||
         (board_imu && !config.enable_board_imu) ||
         (lidar && !config.enable_lidar))) {
      throw std::invalid_argument("cannot restart a disabled stream");
    }

    const bool previous_camera = config.enable_camera;
    const bool previous_board_imu = config.enable_board_imu;
    const bool previous_lidar = config.enable_lidar;
    publishStatus("reconfiguring_streams");
    stopConfiguredStreams(context);

    if (command == StreamCommand::Start) {
      config.enable_camera = config.enable_camera || camera;
      config.enable_board_imu = config.enable_board_imu || board_imu;
      config.enable_lidar = config.enable_lidar || lidar;
    } else if (command == StreamCommand::Stop) {
      config.enable_camera = config.enable_camera && !camera;
      config.enable_board_imu = config.enable_board_imu && !board_imu;
      config.enable_lidar = config.enable_lidar && !lidar;
    }

    camera_assemblies.clear();
    device_time_resolver.reset();
    try {
      startConfiguredStreams(context);
      publishStatus(streamStateText(context));
      return streamState(context);
    } catch (...) {
      const auto operation_error = std::current_exception();
      try {
        stopConfiguredStreams(context);
      } catch (...) {
      }
      config.enable_camera = previous_camera;
      config.enable_board_imu = previous_board_imu;
      config.enable_lidar = previous_lidar;
      camera_assemblies.clear();
      device_time_resolver.reset();
      try {
        startConfiguredStreams(context);
        publishStatus(streamStateText(context));
      } catch (...) {
        fatal_control_error = "stream control failed: " +
                              exceptionText(operation_error) +
                              "; previous stream state recovery failed: " +
                              exceptionText(std::current_exception());
        throw std::runtime_error(fatal_control_error);
      }
      std::rethrow_exception(operation_error);
    }
  }

  void log(LogLevel level, const std::string& text) const {
    if (callbacks.log) callbacks.log(level, text);
  }

  prism::Client openClient() const {
    if (config.device_serial.empty()) return prism::Client::openFirst();
    const auto devices = prism::Client::enumerate();
    const auto found = std::find_if(
        devices.begin(), devices.end(), [this](const prism::DeviceInfo& info) {
          return narrow(info.serial_number) == config.device_serial;
        });
    if (found == devices.end()) {
      throw std::runtime_error("Prism device serial not found: " +
                               config.device_serial);
    }
    return prism::Client::open(*found);
  }

  void startDispatchers() {
    camera_dispatch.start();
    board_imu_dispatch.start();
    lidar_dispatch.start();
    lidar_imu_dispatch.start();
  }

  void stopDispatchers() {
    lidar_imu_dispatch.stop();
    lidar_dispatch.stop();
    board_imu_dispatch.stop();
    camera_dispatch.stop();
  }

  void dispatchBoardImu(const prism::ImuSample& sample) {
    if (config.require_synchronized_timestamps && !sample.timestamp_synced) {
      dropped_unsynchronized.fetch_add(1);
      return;
    }
    const uint64_t timestamp_ns = sample.timestamp_us * 1000u;
    if (sample.timestamp_synced) {
      device_time_resolver.observeBoard(timestamp_ns, steadyNowNs());
    }
    BoardImuSample output;
    output.sensor_id = sample.sensor_id;
    output.sample_id = sample.sample_id;
    output.timestamp_ns = timestamp_ns;
    output.fsync_event = sample.fsync_event;
    output.sample_gap = sample.sample_gap;
    for (size_t i = 0; i < 3; ++i) {
      output.acceleration_m_s2[i] =
          static_cast<double>(sample.accel_mg[i]) * kStandardGravity / 1000.0;
      output.angular_velocity_rad_s[i] =
          static_cast<double>(sample.gyro_mdps[i]) * kRadiansPerDegree / 1000.0;
    }
    output.temperature_c = static_cast<double>(sample.temp_milli_c) / 1000.0;
    if (sample.sensor_id < board_imu_samples.size()) {
      board_imu_samples[sample.sensor_id].fetch_add(1);
    }
    board_imu_dispatch.push(std::move(output));
  }

  void dispatchLidarPoints(const prism::LidarPointBatch& batch) {
    if (config.require_synchronized_timestamps && !batch.timestamp_synced) {
      dropped_unsynchronized.fetch_add(1);
      return;
    }
    const uint64_t sdk_timestamp_ns = batch.timestamp_utc_us * 1000u;
    const auto resolved = device_time_resolver.resolveLidar(
        sdk_timestamp_ns, batch.timestamp_raw, steadyNowNs());
    const bool reference_required =
        config.enable_camera || config.enable_board_imu;
    if (!resolved && reference_required &&
        config.require_synchronized_timestamps) {
      dropped_unsynchronized.fetch_add(1);
      return;
    }
    LidarPointBatch source_batch;
    source_batch.batch_id = batch.batch_id;
    source_batch.timestamp_ns =
        resolved ? resolved->timestamp_ns : sdk_timestamp_ns;
    if (resolved &&
        resolved->candidate == DeviceTimeCandidate::LidarRaw) {
      lidar_raw_timestamp_selected.fetch_add(1);
    } else {
      lidar_sdk_timestamp_selected.fetch_add(1);
    }
    source_batch.timestamp_raw = batch.timestamp_raw;
    source_batch.time_interval_100ns = batch.time_interval_100ns;
    source_batch.points.reserve(batch.points.size());
    for (const auto& point : batch.points) {
      source_batch.points.push_back(
          {static_cast<float>(point.x_mm) * 0.001F,
           static_cast<float>(point.y_mm) * 0.001F,
           static_cast<float>(point.z_mm) * 0.001F,
           point.reflectivity, point.tag, 0u});
    }
    lidar_batches.fetch_add(1);
    lidar_points.fetch_add(source_batch.points.size());
    for (auto& frame :
         lidar_frame_accumulator.append(std::move(source_batch))) {
      lidar_dispatch.push(std::move(frame));
    }
  }

  void dispatchLidarImu(const prism::LidarImuSample& sample) {
    if (config.require_synchronized_timestamps && !sample.timestamp_synced) {
      dropped_unsynchronized.fetch_add(1);
      return;
    }
    const uint64_t sdk_timestamp_ns = sample.timestamp_utc_us * 1000u;
    const auto resolved = device_time_resolver.resolveLidar(
        sdk_timestamp_ns, sample.timestamp_raw_ns, steadyNowNs());
    const bool reference_required =
        config.enable_camera || config.enable_board_imu;
    if (!resolved && reference_required &&
        config.require_synchronized_timestamps) {
      dropped_unsynchronized.fetch_add(1);
      return;
    }
    LidarImuSample output;
    output.sample_id = sample.sample_id;
    output.timestamp_ns = resolved ? resolved->timestamp_ns : sdk_timestamp_ns;
    if (resolved &&
        resolved->candidate == DeviceTimeCandidate::LidarRaw) {
      lidar_raw_timestamp_selected.fetch_add(1);
    } else {
      lidar_sdk_timestamp_selected.fetch_add(1);
    }
    output.timestamp_raw_ns = sample.timestamp_raw_ns;
    for (size_t i = 0; i < 3; ++i) {
      output.acceleration_m_s2[i] = sample.accel_m_s2[i];
      output.angular_velocity_rad_s[i] = sample.gyro_rad_s[i];
    }
    lidar_imu_samples.fetch_add(1);
    lidar_imu_dispatch.push(std::move(output));
  }

  void handleVideoChunk(prism::Client& client,
                        const prism::VideoChunkView& chunk) {
    if (chunk.camera_id >= 4 || chunk.encoded_size == 0 ||
        chunk.chunk_offset > chunk.encoded_size ||
        chunk.data_size > chunk.encoded_size - chunk.chunk_offset) {
      log(LogLevel::Warning, "discarded malformed camera chunk");
      return;
    }
    auto& assembly = camera_assemblies[chunk.frame_id];
    assembly.width = chunk.width;
    assembly.height = chunk.height;
    const size_t camera = chunk.camera_id;

    if (config.enable_camera) {
      auto& image = assembly.jpeg[camera];
      if (image.empty()) image.resize(chunk.encoded_size);
      if (image.size() != chunk.encoded_size ||
          assembly.received[camera] != chunk.chunk_offset) {
        assembly.invalid_mask = static_cast<uint8_t>(
            assembly.invalid_mask | static_cast<uint8_t>(1u << camera));
      } else if (chunk.data_size != 0) {
        std::memcpy(image.data() + chunk.chunk_offset, chunk.data,
                    chunk.data_size);
        assembly.received[camera] += static_cast<uint32_t>(chunk.data_size);
      }
      if (assembly.received[camera] == chunk.encoded_size) {
        assembly.complete_mask = static_cast<uint8_t>(
            assembly.complete_mask | static_cast<uint8_t>(1u << camera));
      }
    }

    if ((chunk.flags & kVideoChunkLast) != 0u) {
      assembly.last_mask = static_cast<uint8_t>(
          assembly.last_mask | static_cast<uint8_t>(1u << camera));
    }
    finishCameraFrame(client, chunk.frame_id);
    trimCameraAssemblies(client);
  }

  void handleVideoMeta(prism::Client& client, const prism::VideoMeta& meta) {
    if (meta.valid && meta.trigger_time_ns != 0u) {
      device_time_resolver.observeCamera(meta.trigger_time_ns, steadyNowNs());
    }
    auto& assembly = camera_assemblies[meta.host_frame_id];
    assembly.metadata_seen = true;
    assembly.metadata = meta;
    finishCameraFrame(client, meta.host_frame_id);
    trimCameraAssemblies(client);
  }

  void finishCameraFrame(prism::Client& client, uint32_t frame_id) {
    const auto found = camera_assemblies.find(frame_id);
    if (found == camera_assemblies.end()) return;
    auto& assembly = found->second;
    if (!assembly.metadata_seen || assembly.last_mask != camera_mask) return;

    const bool payload_complete =
        !config.enable_camera ||
        (assembly.complete_mask == camera_mask && assembly.invalid_mask == 0u);
    const bool timestamp_valid =
        assembly.metadata.valid && assembly.metadata.trigger_time_ns != 0u;

    CameraFrameSet output;
    if (config.enable_camera && payload_complete &&
        (!config.require_synchronized_timestamps || timestamp_valid)) {
      output.timestamp_ns = assembly.metadata.trigger_time_ns;
      output.host_frame_id = frame_id;
      output.carrier_frame_id = assembly.metadata.carrier_frame_id;
      output.width = assembly.width;
      output.height = assembly.height;
      output.jpeg = std::move(assembly.jpeg);
      output.exposure_us = assembly.metadata.exposure_us;
      output.analog_gain_x1024 = assembly.metadata.analog_gain_x1024;
      output.digital_gain_x1024 = assembly.metadata.digital_gain_x1024;
    } else if (config.enable_camera && !timestamp_valid) {
      dropped_unsynchronized.fetch_add(1);
    }

    // Release the Agent's next camera credit before ROS serialization.
    client.sendVideoAck(frame_id);
    camera_assemblies.erase(found);

    if (config.enable_camera && payload_complete &&
        (!config.require_synchronized_timestamps || timestamp_valid)) {
      camera_frame_sets.fetch_add(1);
      camera_dispatch.push(std::move(output));
    }
  }

  void trimCameraAssemblies(prism::Client& client) {
    while (camera_assemblies.size() > 8) {
      const auto found = camera_assemblies.begin();
      const uint32_t stale_frame_id = found->first;
      camera_assemblies.erase(found);
      client.sendVideoAck(stale_frame_id);
      log(LogLevel::Warning, "discarded incomplete camera frame set " +
                                 std::to_string(stale_frame_id));
    }
  }

  DriverStatus snapshot(const std::string& state_text,
                        const std::string& error_text = {}) const {
    DriverStatus output;
    output.running = running.load();
    output.usb3_connected = usb3_connected;
    output.sensor_board_online = sensor_board_online;
    output.sensor_board_time_synced = sensor_board_time_synced;
    output.product_serial = product_serial;
    output.state = state_text;
    output.error = error_text;
    output.camera_frame_sets = camera_frame_sets.load();
    for (size_t i = 0; i < output.board_imu_samples.size(); ++i) {
      output.board_imu_samples[i] = board_imu_samples[i].load();
    }
    output.lidar_batches = lidar_batches.load();
    output.lidar_points = lidar_points.load();
    output.lidar_imu_samples = lidar_imu_samples.load();
    output.lidar_raw_timestamp_selected =
        lidar_raw_timestamp_selected.load();
    output.lidar_sdk_timestamp_selected =
        lidar_sdk_timestamp_selected.load();
    output.dropped_unsynchronized = dropped_unsynchronized.load();
    output.dropped_camera_dispatch = camera_dispatch.dropped();
    output.dropped_lidar_dispatch = lidar_dispatch.dropped();
    output.dropped_imu_dispatch =
        board_imu_dispatch.dropped() + lidar_imu_dispatch.dropped();
    return output;
  }

  void publishStatus(const std::string& state_text,
                     const std::string& error_text = {}) const {
    if (callbacks.status) callbacks.status(snapshot(state_text, error_text));
  }

  void establishDeviceTimeReference(prism::Client& client,
                                    prism::ImuStream* imu_stream) {
    if (!config.require_synchronized_timestamps ||
        (!config.enable_camera && !config.enable_board_imu)) {
      return;
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    const auto reference_ready = [this]() {
      return config.enable_board_imu
                 ? device_time_resolver.hasBoardReference()
                 : device_time_resolver.hasReference();
    };
    while (!reference_ready() && std::chrono::steady_clock::now() < deadline) {
      try {
        auto frame = client.readFrame(1000);
        bool handled = imu_stream != nullptr && imu_stream->handleFrame(frame);
        if (!handled && frame.type == prism::FrameType::VideoChunk) {
          handleVideoChunk(client, prism::parseVideoChunkView(frame));
        } else if (!handled && frame.type == prism::FrameType::VideoMeta) {
          handleVideoMeta(client, prism::parseVideoMeta(frame));
        }
      } catch (const std::exception& error) {
        if (!isTimeout(error)) throw;
      }
    }
    if (!reference_ready()) {
      throw std::runtime_error(
          "camera/board IMU did not provide a synchronized device timestamp");
    }
    // Samples discarded while establishing the initial clock anchor are not
    // part of the advertised streaming interval.
    dropped_unsynchronized.store(0);
  }

  void run(const std::function<bool()>& keep_running) {
    if (!config.enable_camera && !config.enable_board_imu &&
        !config.enable_lidar) {
      throw std::invalid_argument("at least one Prism stream must be enabled");
    }

    stop_requested.store(false);
    fatal_control_error.clear();
    device_time_resolver.reset();
    lidar_frame_accumulator.reset();
    startDispatchers();
    prism::Client client = openClient();
    client.setKeepaliveEnabled(true);
    const auto info = client.deviceInfo();
    usb3_connected = info.usb3_connected;
    sensor_board_online = info.sensor_board_online;
    sensor_board_time_synced = info.sensor_board_time_synced;
    product_serial = info.product_serial;
    if ((config.enable_camera || config.enable_board_imu) &&
        !info.sensor_board_online) {
      stopDispatchers();
      throw std::runtime_error("sensor-board is offline");
    }

    // Cameras can occupy non-contiguous slots (e.g. present_mask 0b0101 =
    // slots 0 and 2). Deriving the mask from the count would request empty
    // slots and make sensor-board capture enable time out (-110).
    camera_mask = static_cast<uint8_t>(info.camera_present_mask & 0x0fu);
    if (camera_mask == 0u) camera_mask = 0x0fu;

    std::unique_ptr<prism::ImuStream> imu_stream;
    std::unique_ptr<prism::LidarStream> lidar_stream;
    uint32_t imu_sensor_count = config.imu_sensor_count;
    if (imu_sensor_count == 0) {
      imu_sensor_count = std::max<uint32_t>(1, info.detected_imu_count);
    }
    if (config.enable_board_imu) {
      imu_stream = std::make_unique<prism::ImuStream>(
          client, [this](const prism::ImuSample& sample) {
            dispatchBoardImu(sample);
          });
    }
    if (config.enable_lidar) {
      lidar_stream = std::make_unique<prism::LidarStream>(
          client,
          [this](const prism::LidarPointBatch& batch) {
            dispatchLidarPoints(batch);
          },
          [this](const prism::LidarImuSample& sample) {
            dispatchLidarImu(sample);
          });
    }
    bool video_started = false;
    bool imu_started = false;
    bool lidar_started = false;
    ControlContext control_context{client, imu_stream, lidar_stream,
                                   imu_sensor_count, video_started,
                                   imu_started, lidar_started};
    running.store(true);
    publishStatus("starting");

    try {
      startConfiguredStreams(control_context);
      enableControls();
      publishStatus("streaming");
      auto next_status = std::chrono::steady_clock::now();
      uint32_t consecutive_timeouts = 0;
      while (!stop_requested.load() && keep_running()) {
        processControls(control_context);
        if (!fatal_control_error.empty()) {
          throw std::runtime_error(fatal_control_error);
        }
        try {
          auto frame = client.readFrame(1000);
          consecutive_timeouts = 0;
          bool handled = false;
          if (imu_stream) handled = imu_stream->handleFrame(frame) || handled;
          if (lidar_stream) {
            handled = lidar_stream->handleFrame(frame) || handled;
          }
          if (!handled && frame.type == prism::FrameType::VideoChunk) {
            handleVideoChunk(client, prism::parseVideoChunkView(frame));
          } else if (!handled && frame.type == prism::FrameType::VideoMeta) {
            handleVideoMeta(client, prism::parseVideoMeta(frame));
          }
        } catch (const std::exception& error) {
          if (!isTimeout(error)) throw;
          if (!video_started && !imu_started && !lidar_started) {
            consecutive_timeouts = 0;
          } else if (++consecutive_timeouts >= 10u) {
            throw;
          }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= next_status) {
          publishStatus(streamStateText(control_context));
          next_status = now + std::chrono::seconds(1);
        }
      }
    } catch (const std::exception& error) {
      cancelControls(std::string("Prism driver stopped: ") + error.what());
      publishStatus("failed", error.what());
      log(LogLevel::Error, error.what());
      cleanup(client, lidar_stream.get(), imu_stream.get(), lidar_started,
              imu_started, video_started);
      running.store(false);
      stopDispatchers();
      throw;
    }

    cancelControls("Prism driver stopped");
    cleanup(client, lidar_stream.get(), imu_stream.get(), lidar_started,
            imu_started, video_started);
    running.store(false);
    publishStatus("stopped");
    stopDispatchers();
  }

  void cleanup(prism::Client& client, prism::LidarStream* lidar_stream,
               prism::ImuStream* imu_stream, bool lidar_started,
               bool imu_started, bool video_started) const noexcept {
    try {
      if (lidar_started && lidar_stream != nullptr) lidar_stream->stop();
    } catch (...) {
    }
    try {
      if (imu_started && imu_stream != nullptr) imu_stream->stop();
    } catch (...) {
    }
    try {
      if (video_started) client.stopVideo();
    } catch (...) {
    }
    try {
      client.close();
    } catch (...) {
    }
  }

  DriverConfig config;
  DriverCallbacks callbacks;
  DispatchQueue<CameraFrameSet> camera_dispatch;
  DispatchQueue<BoardImuSample> board_imu_dispatch;
  DispatchQueue<LidarPointBatch> lidar_dispatch;
  DispatchQueue<LidarImuSample> lidar_imu_dispatch;
  std::mutex control_mutex;
  std::deque<ControlCommand> control_commands;
  bool accepting_controls = false;
  std::string fatal_control_error;
  std::atomic<bool> stop_requested{false};
  std::atomic<bool> running{false};
  bool usb3_connected = false;
  bool sensor_board_online = false;
  bool sensor_board_time_synced = false;
  std::string product_serial;
  uint8_t camera_mask = 0x0fu;
  std::map<uint32_t, CameraAssembly> camera_assemblies;
  std::atomic<uint64_t> camera_frame_sets{0};
  std::array<std::atomic<uint64_t>, 2> board_imu_samples{};
  std::atomic<uint64_t> lidar_batches{0};
  std::atomic<uint64_t> lidar_points{0};
  std::atomic<uint64_t> lidar_imu_samples{0};
  std::atomic<uint64_t> lidar_raw_timestamp_selected{0};
  std::atomic<uint64_t> lidar_sdk_timestamp_selected{0};
  std::atomic<uint64_t> dropped_unsynchronized{0};
  DeviceTimeResolver device_time_resolver;
  LidarFrameAccumulator lidar_frame_accumulator;
};

Driver::Driver(DriverConfig config, DriverCallbacks callbacks)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(callbacks))) {}

Driver::~Driver() = default;

void Driver::run(const std::function<bool()>& keep_running) {
  impl_->run(keep_running);
}

void Driver::requestStop() noexcept { impl_->stop_requested.store(true); }

ExposureState Driver::getExposure() {
  return impl_->invokeControl<ExposureState>([](auto& context) {
    return fromSdk(context.client.cameraExposure());
  });
}

ExposureState Driver::setTargetBrightness(uint8_t target_brightness) {
  if (target_brightness == 0) {
    throw std::invalid_argument("target brightness must be in range 1..255");
  }
  return impl_->invokeControl<ExposureState>(
      [target_brightness](auto& context) {
        return fromSdk(context.client.setAutoExposureTargetBrightness(
            target_brightness));
      });
}

ExposureState Driver::setCameraExposure(uint8_t camera_index,
                                        CameraExposureMode mode,
                                        uint32_t exposure_time_us,
                                        uint32_t gain_x1024) {
  if (camera_index >= 4) {
    throw std::invalid_argument("camera index must be in range 0..3");
  }
  return impl_->invokeControl<ExposureState>(
      [camera_index, mode, exposure_time_us, gain_x1024](auto& context) mutable {
        if (mode == CameraExposureMode::Automatic &&
            (exposure_time_us == 0 || gain_x1024 == 0)) {
          const auto current = context.client.cameraExposure();
          if (exposure_time_us == 0) {
            exposure_time_us = current.manual_exposure_time_us[camera_index];
          }
          if (gain_x1024 == 0) {
            gain_x1024 = current.gain_x1024[camera_index];
          }
        }
        prism::CameraExposureConfiguration configuration;
        configuration.mode =
            mode == CameraExposureMode::Automatic
                ? prism::CameraExposureMode::Automatic
                : prism::CameraExposureMode::Manual;
        configuration.exposure_time_us = exposure_time_us;
        configuration.gain_x1024 = gain_x1024;
        return fromSdk(context.client.setCameraExposure(camera_index,
                                                        configuration));
      });
}

ExposureLimitsState Driver::getExposureLimits() {
  return impl_->invokeControl<ExposureLimitsState>([](auto& context) {
    return fromSdk(context.client.cameraExposureLimits());
  });
}

ExposureLimitsState Driver::setExposureLimits(uint32_t min_exposure_time_us,
                                              uint32_t max_exposure_time_us,
                                              uint32_t min_gain_x1024,
                                              uint32_t max_gain_x1024) {
  return impl_->invokeControl<ExposureLimitsState>(
      [min_exposure_time_us, max_exposure_time_us, min_gain_x1024,
       max_gain_x1024](auto& context) {
        prism::ExposureLimits limits;
        limits.min_exposure_time_us = min_exposure_time_us;
        limits.max_exposure_time_us = max_exposure_time_us;
        limits.min_gain_x1024 = min_gain_x1024;
        limits.max_gain_x1024 = max_gain_x1024;
        return fromSdk(context.client.setCameraExposureLimits(limits));
      });
}

SystemTimeSyncState Driver::synchronizeSystemTime() {
  return impl_->invokeControl<SystemTimeSyncState>(
      [this](auto& context) { return impl_->synchronizeSystemTime(context); });
}

DeviceState Driver::getDeviceInfo() {
  return impl_->invokeControl<DeviceState>([this](auto& context) {
    const auto hello = context.client.hello();
    const auto versions = context.client.deviceVersions();
    const auto info = context.client.deviceInfo();
    impl_->usb3_connected = info.usb3_connected;
    impl_->sensor_board_online = info.sensor_board_online;
    impl_->sensor_board_time_synced = info.sensor_board_time_synced;
    impl_->product_serial = info.product_serial;
    return fromSdk(hello, versions, info);
  });
}

DeviceConfigurationState Driver::getDeviceConfiguration() {
  return impl_->invokeControl<DeviceConfigurationState>([](auto& context) {
    return fromSdk(context.client.deviceConfiguration());
  });
}

DeviceConfigurationState Driver::setDeviceConfiguration(
    bool set_camera_fps, uint32_t camera_fps, bool set_imu_rate_hz,
    uint32_t imu_rate_hz, bool set_mjpeg_quality, uint32_t mjpeg_quality) {
  uint32_t field_mask = 0;
  if (set_camera_fps) field_mask |= prism::kDeviceConfigFieldCameraFps;
  if (set_imu_rate_hz) field_mask |= prism::kDeviceConfigFieldImuRateHz;
  if (set_mjpeg_quality) field_mask |= prism::kDeviceConfigFieldMjpegQuality;
  if (field_mask == 0) {
    throw std::invalid_argument("select at least one configuration field");
  }
  return impl_->invokeControl<DeviceConfigurationState>(
      [this, set_camera_fps, camera_fps, set_imu_rate_hz, imu_rate_hz,
       set_mjpeg_quality, mjpeg_quality, field_mask](auto& context) {
        return impl_->runIdleOperation<DeviceConfigurationState>(
            context, "saving_device_configuration",
            [this, set_camera_fps, camera_fps, set_imu_rate_hz, imu_rate_hz,
             set_mjpeg_quality, mjpeg_quality,
             field_mask](prism::Client& client) {
              auto configuration = client.deviceConfiguration();
              if (set_camera_fps) configuration.camera_fps = camera_fps;
              if (set_imu_rate_hz) configuration.imu_rate_hz = imu_rate_hz;
              if (set_mjpeg_quality) {
                configuration.mjpeg_quality = mjpeg_quality;
              }
              const auto saved =
                  client.saveDeviceConfiguration(configuration, field_mask);
              if (set_camera_fps) impl_->config.camera_fps = saved.camera_fps;
              if (set_imu_rate_hz) {
                impl_->config.imu_rate_hz = saved.imu_rate_hz;
              }
              return fromSdk(saved);
            });
      });
}

LidarStatusState Driver::getLidarStatus() {
  return impl_->invokeControl<LidarStatusState>([](auto& context) {
    return fromSdk(context.client.lidarStatus());
  });
}

LidarNetworkState Driver::getLidarNetwork() {
  return impl_->invokeControl<LidarNetworkState>([this](auto& context) {
    return impl_->runIdleOperation<LidarNetworkState>(
        context, "reading_lidar_network", [](prism::Client& client) {
          return fromSdk(client.lidarNetworkStatus());
        });
  });
}

LidarNetworkState Driver::setLidarNetwork(bool enabled, std::string host_ip,
                                          std::string netmask,
                                          std::string lidar_ip) {
  return impl_->invokeControl<LidarNetworkState>(
      [this, enabled, host_ip = std::move(host_ip),
       netmask = std::move(netmask),
       lidar_ip = std::move(lidar_ip)](auto& context) {
        return impl_->runIdleOperation<LidarNetworkState>(
            context, "saving_lidar_network",
            [enabled, host_ip, netmask,
             lidar_ip](prism::Client& client) {
              prism::LidarNetworkConfiguration configuration;
              configuration.enabled = enabled;
              configuration.host_ip = host_ip;
              configuration.netmask = netmask;
              configuration.lidar_ip = lidar_ip;
              return fromSdk(
                  client.saveLidarNetworkConfiguration(configuration));
            });
      });
}

LidarNetworkState Driver::probeLidarNetwork() {
  return impl_->invokeControl<LidarNetworkState>([this](auto& context) {
    return impl_->runIdleOperation<LidarNetworkState>(
        context, "probing_lidar_network", [](prism::Client& client) {
          return fromSdk(client.probeLidarNetwork());
        });
  });
}

StreamState Driver::getStreamState() {
  return impl_->invokeControl<StreamState>(
      [this](auto& context) { return impl_->streamState(context); });
}

StreamState Driver::controlStreams(StreamCommand command, bool camera,
                                   bool board_imu, bool lidar) {
  return impl_->invokeControl<StreamState>(
      [this, command, camera, board_imu, lidar](auto& context) {
        return impl_->applyStreamCommand(context, command, camera, board_imu,
                                         lidar);
      });
}

WifiHotspotState Driver::getWifiHotspot() {
  return impl_->invokeControl<WifiHotspotState>([this](auto& context) {
    return impl_->runIdleOperation<WifiHotspotState>(
        context, "reading_wifi_hotspot", [](prism::Client& client) {
          return fromSdk(client.wifiHotspotStatus());
        });
  });
}

WifiHotspotState Driver::setWifiHotspot(bool enabled) {
  return impl_->invokeControl<WifiHotspotState>(
      [this, enabled](auto& context) {
        return impl_->runIdleOperation<WifiHotspotState>(
            context, "saving_wifi_hotspot",
            [enabled](prism::Client& client) {
              return fromSdk(client.setWifiHotspotEnabled(enabled));
            });
      });
}

}  // namespace prism_ros_adapter

#include "ping/imu.hpp"

#include "ping/constants.hpp"

extern "C" {
#include <sh2_SensorValue.h>
#include <sh2_err.h>
}

namespace ping::imu {

namespace {

/// How long the hub may take to answer a request before the operation is
/// aborted (sh2 itself would wait forever).
constexpr std::uint32_t kResponseTimeoutUs = 1'000'000;
/// How long to wait for "reset complete" after the hub has booted, beyond
/// the 200 ms sh2_open itself waits.
constexpr std::uint32_t kResetTimeoutMs = 1'000;
/// How long one HAL read blocks on INT inside sh2's polling loops.
constexpr std::uint32_t kPollWaitUs = 5'000;

constexpr sh2_SensorId_t to_sh2(SensorId sensor)
{
    switch (sensor) {
    case SensorId::kGameRotationVector:
        return SH2_GAME_ROTATION_VECTOR;
    case SensorId::kMagneticFieldUncalibrated:
        return SH2_MAGNETIC_FIELD_UNCALIBRATED;
    case SensorId::kRotationVector:
        return SH2_ROTATION_VECTOR;
    case SensorId::kGeomagneticRotationVector:
        return SH2_GEOMAGNETIC_ROTATION_VECTOR;
    case SensorId::kMagneticField:
        return SH2_MAGNETIC_FIELD_CALIBRATED;
    case SensorId::kAccelerometer:
        return SH2_ACCELEROMETER;
    case SensorId::kLinearAcceleration:
        return SH2_LINEAR_ACCELERATION;
    case SensorId::kGravity:
        return SH2_GRAVITY;
    case SensorId::kGyroscope:
        return SH2_GYROSCOPE_CALIBRATED;
    case SensorId::kGyroscopeUncalibrated:
        break;
    }
    return SH2_GYROSCOPE_UNCALIBRATED;
}

geo::Quaternion quat(float i, float j, float k, float real) { return {.i = i, .j = j, .k = k, .real = real}; }
geo::Vec3 vec(float x, float y, float z) { return {.x = x, .y = y, .z = z}; }

/// Fills `out` from a decoded report; false for sensors this module does not
/// hand out.
bool decode(const sh2_SensorValue_t& value, Event& out)
{
    out = Event{};
    out.accuracy = static_cast<geo::Accuracy>(value.status & 0x03U);
    out.timestamp_us = value.timestamp;
    const auto& u = value.un;
    switch (value.sensorId) {
    case SH2_GAME_ROTATION_VECTOR:
        out.sensor = SensorId::kGameRotationVector;
        out.quaternion = quat(u.gameRotationVector.i, u.gameRotationVector.j, u.gameRotationVector.k,
                              u.gameRotationVector.real);
        return true;
    case SH2_ROTATION_VECTOR:
        out.sensor = SensorId::kRotationVector;
        out.quaternion = quat(u.rotationVector.i, u.rotationVector.j, u.rotationVector.k, u.rotationVector.real);
        out.heading_accuracy_rad = u.rotationVector.accuracy;
        return true;
    case SH2_GEOMAGNETIC_ROTATION_VECTOR:
        out.sensor = SensorId::kGeomagneticRotationVector;
        out.quaternion = quat(u.geoMagRotationVector.i, u.geoMagRotationVector.j, u.geoMagRotationVector.k,
                              u.geoMagRotationVector.real);
        out.heading_accuracy_rad = u.geoMagRotationVector.accuracy;
        return true;
    case SH2_MAGNETIC_FIELD_UNCALIBRATED:
        out.sensor = SensorId::kMagneticFieldUncalibrated;
        out.vector = vec(u.magneticFieldUncal.x, u.magneticFieldUncal.y, u.magneticFieldUncal.z);
        out.bias = vec(u.magneticFieldUncal.biasX, u.magneticFieldUncal.biasY, u.magneticFieldUncal.biasZ);
        return true;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
        out.sensor = SensorId::kMagneticField;
        out.vector = vec(u.magneticField.x, u.magneticField.y, u.magneticField.z);
        return true;
    case SH2_ACCELEROMETER:
        out.sensor = SensorId::kAccelerometer;
        out.vector = vec(u.accelerometer.x, u.accelerometer.y, u.accelerometer.z);
        return true;
    case SH2_LINEAR_ACCELERATION:
        out.sensor = SensorId::kLinearAcceleration;
        out.vector = vec(u.linearAcceleration.x, u.linearAcceleration.y, u.linearAcceleration.z);
        return true;
    case SH2_GRAVITY:
        out.sensor = SensorId::kGravity;
        out.vector = vec(u.gravity.x, u.gravity.y, u.gravity.z);
        return true;
    case SH2_GYROSCOPE_CALIBRATED:
        out.sensor = SensorId::kGyroscope;
        out.vector = vec(u.gyroscope.x, u.gyroscope.y, u.gyroscope.z);
        return true;
    case SH2_GYROSCOPE_UNCALIBRATED:
        out.sensor = SensorId::kGyroscopeUncalibrated;
        out.vector = vec(u.gyroscopeUncal.x, u.gyroscopeUncal.y, u.gyroscopeUncal.z);
        out.bias = vec(u.gyroscopeUncal.biasX, u.gyroscopeUncal.biasY, u.gyroscopeUncal.biasZ);
        return true;
    default:
        return false;
    }
}

Error from_status(int status)
{
    switch (status) {
    case SH2_ERR_TIMEOUT:
        return {.kind = Error::Kind::kNoResponse, .code = status};
    case SH2_ERR_HUB:
        return {.kind = Error::Kind::kRejected, .code = status};
    case SH2_ERR_IO:
        return {.kind = Error::Kind::kBus, .code = status};
    default:
        return {.kind = Error::Kind::kSh2, .code = status};
    }
}

} // namespace

const char* to_string(const Error& error)
{
    switch (error.kind) {
    case Error::Kind::kInit:
        return "SPI/GPIO setup failed";
    case Error::Kind::kBootTimeout:
        return "no INT after reset";
    case Error::Kind::kNoResetNotice:
        return "no reset-complete notice";
    case Error::Kind::kWakeTimeout:
        return "no answer to WAKE";
    case Error::Kind::kBus:
        return "SPI transfer failed";
    case Error::Kind::kNoResponse:
        return "no response";
    case Error::Kind::kRejected:
        return "command rejected";
    case Error::Kind::kMismatch:
        return "calibration setting not taken";
    case Error::Kind::kHubReset:
        return "hub reset";
    case Error::Kind::kNotStarted:
        return "not started";
    case Error::Kind::kSh2:
        break;
    }
    return "sh2 error";
}

Imu::Imu()
    : hal_(bno085::Pins{
          .host = kImuSpi,
          .mosi = kImuMosi,
          .miso = kImuMiso,
          .sck = kImuSck,
          .cs = kImuCs,
          .intr = kImuInt,
          .reset = kImuReset,
          .wake = kImuWake,
          .frequency_hz = kImuSpiFrequencyHz,
      })
{
}

void Imu::on_event(void* cookie, sh2_AsyncEvent_t* event)
{
    auto* self = static_cast<Imu*>(cookie);
    // The HAL's own made-up reset notice, used to abort an operation that
    // timed out, is not a hub reset.
    if (event->eventId == SH2_RESET && !self->hal_.spi().aborted()) {
        self->reset_seen_ = true;
        ++self->hub_resets_;
    } else if (event->eventId == SH2_SHTP_EVENT) {
        ++self->shtp_errors_;
    }
}

void Imu::on_sensor(void* cookie, sh2_SensorEvent_t* event)
{
    auto* self = static_cast<Imu*>(cookie);
    sh2_SensorValue_t value{};
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        return;
    }
    ++self->sensor_events_;
    self->last_event_us_ = self->hal_.now_us();
    Event decoded{};
    if (decode(value, decoded)) {
        self->push(decoded);
    }
}

void Imu::push(const Event& event)
{
    if (count_ == kQueueLen) {
        // The oldest report is the least useful one to keep.
        head_ = (head_ + 1) % kQueueLen;
        --count_;
        ++dropped_;
    }
    queue_[(head_ + count_) % kQueueLen] = event;
    ++count_;
}

std::optional<Event> Imu::pop()
{
    if (count_ == 0) {
        return std::nullopt;
    }
    const Event event = queue_[head_];
    head_ = (head_ + 1) % kQueueLen;
    --count_;
    return event;
}

bool Imu::wait_for_reset(std::uint32_t timeout_ms)
{
    const std::uint32_t start = hal_.now_us();
    while (!reset_seen_ && hal_.now_us() - start < timeout_ms * 1000U) {
        sh2_service();
    }
    return reset_seen_;
}

std::optional<Error> Imu::pending_error()
{
    if (!started_) {
        return Error{.kind = reset_seen_ ? Error::Kind::kHubReset : Error::Kind::kNotStarted};
    }
    if (reset_seen_) {
        started_ = false;
        return Error{.kind = Error::Kind::kHubReset};
    }
    return link_error();
}

std::optional<Error> Imu::link_error()
{
    const shtp::LinkCounters counters = hal_.spi().counters();
    std::optional<Error> error;
    if (counters.transfers_failed != seen_transfers_failed_) {
        error = Error{.kind = Error::Kind::kBus, .code = hal_.last_spi_error()};
    } else if (counters.wake_timeouts != seen_wake_timeouts_) {
        error = Error{.kind = Error::Kind::kWakeTimeout};
    }
    seen_transfers_failed_ = counters.transfers_failed;
    seen_wake_timeouts_ = counters.wake_timeouts;
    return error;
}

LinkStats Imu::link_stats() const
{
    const shtp::LinkCounters c = hal_.spi().counters();
    return LinkStats{
        .int_edges = c.int_edges,
        .last_int_us = c.last_int_us,
        .transfers_ok = c.transfers_ok,
        .transfers_failed = c.transfers_failed,
        .last_spi_error = hal_.last_spi_error(),
        .cargos_read = c.cargos_read,
        .empty_headers = c.empty_headers,
        .invalid_headers = c.invalid_headers,
        .oversized_cargos = c.oversized_cargos,
        .discarded_cargos = c.discarded_cargos,
        .wake_timeouts = c.wake_timeouts,
        .hub_resets = hub_resets_,
        .shtp_errors = shtp_errors_,
        .dropped_events = dropped_,
        .sensor_events = sensor_events_,
        .last_event_us = last_event_us_,
    };
}

template <typename Op> std::expected<void, Error> Imu::run(Op op)
{
    if (auto error = pending_error()) {
        return std::unexpected(*error);
    }
    hal_.spi().arm_deadline(kResponseTimeoutUs);
    const int status = op();
    const bool aborted = hal_.spi().aborted();
    hal_.spi().disarm_deadline();

    if (auto error = pending_error()) {
        return std::unexpected(*error);
    }
    if (aborted) {
        return std::unexpected(Error{.kind = Error::Kind::kNoResponse});
    }
    if (status != SH2_OK) {
        return std::unexpected(from_status(status));
    }
    return {};
}

std::expected<ProductId, Error> Imu::start()
{
    started_ = false;
    reset_seen_ = false;
    head_ = 0;
    count_ = 0;
    hal_.spi().disarm_deadline();
    hal_.spi().set_read_wait_us(kPollWaitUs);
    (void)link_error();

    // Harmless when sh2 is not open; otherwise it closes the HAL, which holds
    // the hub in reset until sh2_open opens it again.
    sh2_close();

    // The HAL's open rebuilds the bus and hard-resets the hub (CEVA's
    // sequence); sh2_open then waits up to 200 ms for the reset notice.
    const int status = sh2_open(hal_.sh2(), on_event, this);
    if (status != SH2_OK) {
        if (hal_.init_error() != ESP_OK) {
            return std::unexpected(Error{.kind = Error::Kind::kInit, .code = hal_.init_error()});
        }
        return std::unexpected(from_status(status));
    }
    if (hal_.spi().in_reset()) {
        return std::unexpected(Error{.kind = Error::Kind::kBootTimeout});
    }
    if (!wait_for_reset(kResetTimeoutMs)) {
        const auto error = link_error();
        return std::unexpected(error ? *error : Error{.kind = Error::Kind::kNoResetNotice});
    }

    sh2_setSensorCallback(on_sensor, this);
    started_ = true;
    reset_seen_ = false;

    sh2_ProductIds_t ids{};
    if (auto got = run([&] { return sh2_getProdIds(&ids); }); !got) {
        started_ = false;
        return std::unexpected(got.error());
    }
    if (ids.numEntries == 0) {
        started_ = false;
        return std::unexpected(Error{.kind = Error::Kind::kNoResponse});
    }
    const sh2_ProductId_t& id = ids.entry[0];
    return ProductId{
        .sw_major = id.swVersionMajor,
        .sw_minor = id.swVersionMinor,
        .sw_patch = id.swVersionPatch,
        .part_number = id.swPartNumber,
        .build_number = id.swBuildNumber,
        .reset_cause = id.resetCause,
    };
}

std::expected<void, Error> Imu::enable_report(SensorId sensor, std::uint32_t interval_us)
{
    sh2_SensorConfig_t config{};
    config.reportInterval_us = interval_us;
    return run([&] { return sh2_setSensorConfig(to_sh2(sensor), &config); });
}

std::expected<void, Error> Imu::configure_calibration(bool accel, bool gyro, bool mag)
{
    const auto wanted = static_cast<std::uint8_t>((accel ? SH2_CAL_ACCEL : 0) | (gyro ? SH2_CAL_GYRO : 0) |
                                                  (mag ? SH2_CAL_MAG : 0));
    if (auto set = run([&] { return sh2_setCalConfig(wanted); }); !set) {
        return set;
    }
    // sh2 checks only the status of the answer, so read the setting back.
    std::uint8_t enabled = 0;
    if (auto got = run([&] { return sh2_getCalConfig(&enabled); }); !got) {
        return got;
    }
    constexpr std::uint8_t kMask = SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG;
    if ((enabled & kMask) != wanted) {
        return std::unexpected(Error{.kind = Error::Kind::kMismatch, .code = enabled});
    }
    return {};
}

std::expected<void, Error> Imu::save_calibration()
{
    return run([] { return sh2_saveDcdNow(); });
}

std::expected<void, Error> Imu::tare()
{
    constexpr auto kAllAxes = static_cast<std::uint8_t>(SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z);
    return run([] { return sh2_setTareNow(kAllAxes, SH2_TARE_BASIS_ROTATION_VECTOR); });
}

std::expected<std::optional<Event>, Error> Imu::next(std::uint32_t timeout_ms)
{
    const std::uint32_t start = hal_.now_us();
    bool serviced = false;
    for (;;) {
        if (auto error = pending_error()) {
            return std::unexpected(*error);
        }
        if (auto event = pop()) {
            return event;
        }
        const std::uint32_t elapsed_ms = (hal_.now_us() - start) / 1000U;
        if (serviced && elapsed_ms >= timeout_ms) {
            return std::optional<Event>{};
        }
        // Block on INT for the rest of the timeout rather than sh2's usual
        // short poll, so an idle hub costs no CPU.
        hal_.spi().set_read_wait_us((elapsed_ms < timeout_ms ? timeout_ms - elapsed_ms : 0) * 1000U);
        sh2_service();
        hal_.spi().set_read_wait_us(kPollWaitUs);
        serviced = true;
    }
}

} // namespace ping::imu

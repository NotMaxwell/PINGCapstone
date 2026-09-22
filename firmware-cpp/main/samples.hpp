/// @file
/// What each sensor task hands to the ROS task, and the mailbox it hands it
/// through.
///
/// None of the vendor libraries are thread-safe, and micro-ROS publishers must
/// only be touched from the task running the executor. So each sensor lives
/// in its own FreeRTOS task, owns its library object outright, and drops its
/// newest reading into a @ref Latest mailbox. The ROS task takes whatever is
/// fresh and publishes it. Older readings a slow publisher never saw are
/// overwritten, which is what you want from sensor data.

#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

namespace ping {

/// A single-slot, overwrite-on-write mailbox.
template <typename T>
class Latest {
  public:
    void put(const T& value)
    {
        std::scoped_lock lock{mutex_};
        value_ = value;
    }

    /// The newest value, if one arrived since the last take.
    std::optional<T> take()
    {
        std::scoped_lock lock{mutex_};
        return std::exchange(value_, std::nullopt);
    }

  private:
    std::mutex mutex_;
    std::optional<T> value_;
};

/// Orientation, angular rate and acceleration from the BNO085, in the
/// sensor's own frame and ROS units.
struct ImuSample {
    /// When the rotation vector was measured, on the esp_timer clock (µs).
    std::int64_t stamp_us;
    float qx, qy, qz, qw;
    /// The hub's heading accuracy estimate, radians.
    float heading_accuracy_rad;
    float gyro_x, gyro_y, gyro_z;    ///< rad/s
    float accel_x, accel_y, accel_z; ///< m/s², gravity included (REP-145)
    /// SH-2 status accuracy: 0 unreliable .. 3 high.
    std::uint8_t calibration;
};

/// One TF03 frame.
struct RangeSample {
    std::int64_t stamp_us;
    std::uint16_t distance_cm;
    std::uint16_t strength;
    /// Strong enough and inside the measurable range.
    bool valid;
};

/// One UBX-NAV-PVT solution.
struct FixSample {
    std::int64_t stamp_us;
    double latitude_deg;
    double longitude_deg;
    double altitude_m; ///< Above the WGS-84 ellipsoid, as NavSatFix wants.
    double horizontal_accuracy_m;
    double vertical_accuracy_m;
    std::uint8_t fix_type; ///< 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS+DR, 5 time only
    bool fix_ok;           ///< Within the receiver's DOP and accuracy masks.
    bool differential;     ///< SBAS/RTK corrections applied.
    std::uint8_t satellites;
};

inline Latest<ImuSample> imu_mailbox;
inline Latest<RangeSample> range_mailbox;
inline Latest<FixSample> fix_mailbox;

} // namespace ping

#include "ping/imu.hpp"

#include "ping/constants.hpp"

#include <driver/gpio.h>

namespace ping::imu {

const char* to_string(BNO08xAccuracy accuracy)
{
    switch (accuracy) {
    case BNO08xAccuracy::HIGH:
        return "high";
    case BNO08xAccuracy::MED:
        return "med";
    case BNO08xAccuracy::LOW:
        return "low";
    case BNO08xAccuracy::UNRELIABLE:
        return "bad";
    case BNO08xAccuracy::UNDEFINED:
        break;
    }
    return "?";
}

bool Imu::start(std::uint32_t interval_us)
{
    if (running_) {
        return hub_->hard_reset() && hub_->rpt.rv.enable(interval_us);
    }

    // PS0 high as the library releases reset, so the hub comes up in SPI
    // mode.
    gpio_reset_pin(kImuWake);
    gpio_set_direction(kImuWake, GPIO_MODE_OUTPUT);
    gpio_set_level(kImuWake, 1);

    // A half-finished bring-up is undone by the library's destructor, so a
    // retry starts from a fresh object.
    hub_.reset();
    hub_.emplace(bno08x_config_t{
        kImuSpi,
        kImuMosi,
        kImuMiso,
        kImuSck,
        kImuCs,
        kImuInt,
        kImuReset,
        kImuSpiFrequencyHz,
    });
    if (!hub_->initialize()) {
        hub_.reset();
        return false;
    }
    running_ = true;
    return hub_->rpt.rv.enable(interval_us);
}

sh2_ProductId_t Imu::product_id()
{
    const sh2_ProductIds_t ids = hub_->get_product_IDs();
    // The first entry describes the SH-2 firmware itself.
    return ids.numEntries > 0 ? ids.entry[0] : sh2_ProductId_t{};
}

std::optional<Orientation> Imu::wait()
{
    if (!hub_ || !hub_->data_available() || !hub_->rpt.rv.has_new_data()) {
        return std::nullopt;
    }
    const bno08x_quat_t quat = hub_->rpt.rv.get_quat();
    return Orientation{
        .quaternion = {.i = quat.i, .j = quat.j, .k = quat.k, .real = quat.real},
        .heading_accuracy_rad = quat.rad_accuracy,
        .calibration = quat.accuracy,
    };
}

} // namespace ping::imu

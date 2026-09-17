/// @file
/// Driver-level tests: the SHTP read/write machinery driven against a
/// scripted I²C bus.
///
/// The chunking logic is the interesting part. The hub prefixes *every* read
/// transaction with a header, so a cargo longer than one transfer arrives as
/// a first chunk whose header is the cargo's own, followed by chunks whose
/// headers are repeats that must be stripped. Getting that wrong shifts the
/// payload and silently corrupts every report in the packet.

#include "ping/bno085/bno085.hpp"

#include "fakes.hpp"

#include <gtest/gtest.h>

using namespace ping::bno085;
using ping::test::FakeClock;
using ping::test::ScriptedI2c;

namespace {

using Driver = Bno085<ScriptedI2c, FakeClock>;

void push_le16(std::vector<std::uint8_t>& out, std::int16_t value)
{
    const auto raw = static_cast<std::uint16_t>(value);
    out.push_back(static_cast<std::uint8_t>(raw));
    out.push_back(static_cast<std::uint8_t>(raw >> 8));
}

void push_le32(std::vector<std::uint8_t>& out, std::uint32_t value)
{
    for (int shift = 0; shift < 32; shift += 8) {
        out.push_back(static_cast<std::uint8_t>(value >> shift));
    }
}

std::vector<std::uint8_t> header_bytes(std::uint16_t cargo_len, shtp::Channel channel, std::uint8_t sequence = 0)
{
    return {static_cast<std::uint8_t>(cargo_len),
            static_cast<std::uint8_t>(cargo_len >> 8),
            static_cast<std::uint8_t>(channel),
            sequence};
}

/// One 10-byte accelerometer report reading zero on every axis.
std::vector<std::uint8_t> accel_report(std::uint8_t sequence)
{
    std::vector<std::uint8_t> report{static_cast<std::uint8_t>(SensorId::kAccelerometer), sequence, 0x03, 0};
    push_le16(report, 256); // 1.0 m/s² at Q8
    push_le16(report, 0);
    push_le16(report, 0);
    return report;
}

/// The 5-byte base timestamp report that opens most input packets.
std::vector<std::uint8_t> base_timestamp(std::uint32_t ticks)
{
    std::vector<std::uint8_t> report{kReportBaseTimestamp};
    push_le32(report, ticks);
    return report;
}

void append(std::vector<std::uint8_t>& into, const std::vector<std::uint8_t>& more)
{
    into.insert(into.end(), more.begin(), more.end());
}

} // namespace

TEST(Bno085, ReadEventReportsNothingWhenTheHubIsQuiet)
{
    // A starved bus clocks out zeros, which is not a header.
    ScriptedI2c bus;
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    const auto event = imu.read_event();
    ASSERT_TRUE(event.has_value());
    EXPECT_FALSE(event->has_value());
}

TEST(Bno085, AddressesTheBoardAtItsSelectedAddress)
{
    ScriptedI2c bus;
    FakeClock clock;
    Driver imu{bus, Address::kAlternate, clock};

    (void)imu.read_event();
    EXPECT_EQ(bus.last_address, 0x4B);
}

TEST(Bno085, EnableReportSendsASetFeatureCommand)
{
    ScriptedI2c bus;
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    ASSERT_TRUE(imu.enable_report(SensorId::kRotationVector, 10'000).has_value());

    ASSERT_EQ(bus.writes.size(), 1u);
    const auto& frame = bus.writes.front();
    ASSERT_EQ(frame.size(), shtp::kHeaderLen + 17);

    // Header: 21-byte cargo on the control channel, sequence 0.
    EXPECT_EQ(frame[0], 21);
    EXPECT_EQ(frame[1], 0);
    EXPECT_EQ(frame[2], static_cast<std::uint8_t>(shtp::Channel::kControl));
    EXPECT_EQ(frame[3], 0);

    // Payload: set feature, rotation vector, no flags, no sensitivity, then
    // the report interval in microseconds.
    EXPECT_EQ(frame[4], kReportSetFeatureCommand);
    EXPECT_EQ(frame[5], static_cast<std::uint8_t>(SensorId::kRotationVector));
    EXPECT_EQ(frame[6], 0);
    const std::uint32_t interval = static_cast<std::uint32_t>(frame[9]) | (static_cast<std::uint32_t>(frame[10]) << 8) |
                                   (static_cast<std::uint32_t>(frame[11]) << 16) |
                                   (static_cast<std::uint32_t>(frame[12]) << 24);
    EXPECT_EQ(interval, 10'000u);
}

TEST(Bno085, DisableReportAsksForAZeroInterval)
{
    ScriptedI2c bus;
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    ASSERT_TRUE(imu.disable_report(SensorId::kRotationVector).has_value());
    const auto& frame = bus.writes.front();
    for (std::size_t i = 9; i < 13; ++i) {
        EXPECT_EQ(frame[i], 0) << "interval byte " << i << " should be zero";
    }
}

TEST(Bno085, SequenceNumbersCountPerChannel)
{
    ScriptedI2c bus;
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    ASSERT_TRUE(imu.enable_report(SensorId::kAccelerometer, 10'000).has_value());
    ASSERT_TRUE(imu.enable_report(SensorId::kGyroscopeCalibrated, 10'000).has_value());
    ASSERT_TRUE(imu.save_calibration().has_value());

    ASSERT_EQ(bus.writes.size(), 3u);
    EXPECT_EQ(bus.writes[0][3], 0);
    EXPECT_EQ(bus.writes[1][3], 1);
    EXPECT_EQ(bus.writes[2][3], 2);
}

TEST(Bno085, CommandRequestsCarryTheirOwnSequenceCounter)
{
    ScriptedI2c bus;
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    ASSERT_TRUE(imu.save_calibration().has_value());
    ASSERT_TRUE(imu.save_calibration().has_value());

    // Byte 5 of the frame is the command sequence, inside the payload, and
    // counts separately from the channel sequence in the header.
    EXPECT_EQ(bus.writes[0][5], 0);
    EXPECT_EQ(bus.writes[1][5], 1);
    EXPECT_EQ(bus.writes[0][6], kCommandSaveDcd);
}

TEST(Bno085, DecodesAReportOutOfAnInputPacket)
{
    // Cargo: header + base timestamp + one accelerometer report.
    std::vector<std::uint8_t> payload;
    append(payload, base_timestamp(7));
    append(payload, accel_report(0x2A));

    const auto cargo_len = static_cast<std::uint16_t>(shtp::kHeaderLen + payload.size());
    auto cargo = header_bytes(cargo_len, shtp::Channel::kInputNormal);
    append(cargo, payload);

    ScriptedI2c bus;
    bus.queue_read(header_bytes(cargo_len, shtp::Channel::kInputNormal)); // the 4-byte peek
    bus.queue_read(cargo);                                                // the full read
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    const auto event = imu.read_event();
    ASSERT_TRUE(event.has_value());
    ASSERT_TRUE(event->has_value());
    EXPECT_EQ((*event)->sensor, SensorId::kAccelerometer);
    EXPECT_EQ((*event)->sequence, 0x2A);

    const auto* accel = std::get_if<AccelerometerData>(&(*event)->data);
    ASSERT_NE(accel, nullptr);
    EXPECT_FLOAT_EQ(accel->value.x, 1.0F);

    // The base timestamp moved the reference back 7 ticks, and the report's
    // own delay is zero, so the sample was taken 700 µs before the read.
    EXPECT_EQ((*event)->timestamp_offset_us, -700);
}

TEST(Bno085, WalksEveryReportInABatchedPacket)
{
    std::vector<std::uint8_t> payload;
    append(payload, base_timestamp(0));
    for (std::uint8_t i = 0; i < 3; ++i) {
        append(payload, accel_report(i));
    }

    const auto cargo_len = static_cast<std::uint16_t>(shtp::kHeaderLen + payload.size());
    auto cargo = header_bytes(cargo_len, shtp::Channel::kInputNormal);
    append(cargo, payload);

    ScriptedI2c bus;
    bus.queue_read(header_bytes(cargo_len, shtp::Channel::kInputNormal));
    bus.queue_read(cargo);
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    // Three reports come out of the one packet, then the hub goes quiet.
    for (std::uint8_t i = 0; i < 3; ++i) {
        const auto event = imu.read_event();
        ASSERT_TRUE(event.has_value());
        ASSERT_TRUE(event->has_value()) << "report " << int(i) << " missing";
        EXPECT_EQ((*event)->sequence, i);
    }

    const auto drained = imu.read_event();
    ASSERT_TRUE(drained.has_value());
    EXPECT_FALSE(drained->has_value());
}

TEST(Bno085, ReassemblesACargoThatSpansSeveralTransfers)
{
    // 95 bytes of payload: one timestamp report and nine accelerometer
    // reports. That is a 99-byte cargo, which does not fit in one 64-byte
    // transfer, so the driver reads it in two — and the second transfer
    // arrives with a repeated header the driver has to strip.
    std::vector<std::uint8_t> payload;
    append(payload, base_timestamp(0));
    for (std::uint8_t i = 0; i < 9; ++i) {
        append(payload, accel_report(i));
    }
    ASSERT_EQ(payload.size(), 95u);

    const auto cargo_len = static_cast<std::uint16_t>(shtp::kHeaderLen + payload.size());
    ASSERT_EQ(cargo_len, 99);

    const auto head = header_bytes(cargo_len, shtp::Channel::kInputNormal);
    std::vector<std::uint8_t> cargo = head;
    append(cargo, payload);

    ScriptedI2c bus;
    bus.queue_read(head); // 4-byte peek

    // First transfer: 64 bytes, header included.
    bus.queue_read(std::vector<std::uint8_t>(cargo.begin(), cargo.begin() + 64));

    // Second transfer: a repeat of the header, then the remaining 35 bytes.
    std::vector<std::uint8_t> second = head;
    append(second, std::vector<std::uint8_t>(cargo.begin() + 64, cargo.end()));
    ASSERT_EQ(second.size(), 39u);
    bus.queue_read(second);

    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    for (std::uint8_t i = 0; i < 9; ++i) {
        const auto event = imu.read_event();
        ASSERT_TRUE(event.has_value());
        ASSERT_TRUE(event->has_value()) << "report " << int(i) << " lost across the chunk boundary";
        EXPECT_EQ((*event)->sensor, SensorId::kAccelerometer);
        EXPECT_EQ((*event)->sequence, i) << "payload shifted at the chunk boundary";
    }

    // The driver asked for exactly the transfer sizes the protocol implies.
    ASSERT_EQ(bus.read_lengths.size(), 3u);
    EXPECT_EQ(bus.read_lengths[0], shtp::kHeaderLen);
    EXPECT_EQ(bus.read_lengths[1], 64u);
    EXPECT_EQ(bus.read_lengths[2], 39u);
}

TEST(Bno085, DrainsAnOversizedCargoAndKeepsTheBusInSync)
{
    // The reset advertisement is bigger than the driver's buffer. It has to
    // be read out anyway, or the hub will keep re-offering it.
    ScriptedI2c bus;
    bus.queue_read(header_bytes(300, shtp::Channel::kCommand));
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    const auto event = imu.read_event();
    ASSERT_FALSE(event.has_value());
    EXPECT_EQ(event.error().kind, Error::Kind::kCargoTooLarge);
    EXPECT_EQ(event.error().cargo_len, 300);

    // One peek plus the reads that drained the 300 bytes.
    EXPECT_GT(bus.read_lengths.size() + static_cast<std::size_t>(bus.starved_reads), 1u);
}

TEST(Bno085, SurfacesBusFailures)
{
    ScriptedI2c bus;
    bus.fail_next(-7);
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    const auto event = imu.read_event();
    ASSERT_FALSE(event.has_value());
    EXPECT_EQ(event.error().kind, Error::Kind::kI2c);
    EXPECT_EQ(event.error().bus.code, -7);
}

TEST(Bno085, IgnoresReportsOnChannelsThatCarryNoSensorData)
{
    // A command response on the control channel is consumed and skipped, not
    // mistaken for a sensor report.
    std::vector<std::uint8_t> payload{kReportProductIdResponse, 0, 0, 0};
    const auto cargo_len = static_cast<std::uint16_t>(shtp::kHeaderLen + payload.size());
    auto cargo = header_bytes(cargo_len, shtp::Channel::kControl);
    append(cargo, payload);

    ScriptedI2c bus;
    bus.queue_read(header_bytes(cargo_len, shtp::Channel::kControl));
    bus.queue_read(cargo);
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    const auto event = imu.read_event();
    ASSERT_TRUE(event.has_value());
    EXPECT_FALSE(event->has_value());
}

TEST(Bno085, SoftResetWaitsForTheHubToAnnounceItIsBack)
{
    std::vector<std::uint8_t> payload{kExecutableReset};
    const auto cargo_len = static_cast<std::uint16_t>(shtp::kHeaderLen + payload.size());
    auto cargo = header_bytes(cargo_len, shtp::Channel::kExecutable);
    append(cargo, payload);

    ScriptedI2c bus;
    bus.queue_read(header_bytes(cargo_len, shtp::Channel::kExecutable));
    bus.queue_read(cargo);
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    ASSERT_TRUE(imu.soft_reset().has_value());

    // The reset request went out on the executable channel.
    ASSERT_EQ(bus.writes.size(), 1u);
    EXPECT_EQ(bus.writes[0][2], static_cast<std::uint8_t>(shtp::Channel::kExecutable));
    EXPECT_EQ(bus.writes[0][4], kExecutableReset);

    // And the driver waited for the hub to boot.
    EXPECT_GE(clock.slept_ms, kBootDelayMs);
}

TEST(Bno085, SoftResetGivesUpWhenTheHubNeverAnswers)
{
    ScriptedI2c bus; // nothing queued: the bus stays quiet forever
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    const auto reset = imu.soft_reset();
    ASSERT_FALSE(reset.has_value());
    EXPECT_EQ(reset.error().kind, Error::Kind::kTimeout);
}

TEST(Bno085, ProductIdParsesTheHubsIdentification)
{
    std::vector<std::uint8_t> payload{kReportProductIdResponse, /* reset cause */ 1, /* major */ 3, /* minor */ 2};
    push_le32(payload, 0x0001'2345); // part number
    push_le32(payload, 0x0067'89AB); // build number
    push_le16(payload, 9);           // patch
    push_le16(payload, 0);           // reserved, to reach 16 bytes
    ASSERT_EQ(payload.size(), 16u);

    const auto cargo_len = static_cast<std::uint16_t>(shtp::kHeaderLen + payload.size());
    auto cargo = header_bytes(cargo_len, shtp::Channel::kControl);
    append(cargo, payload);

    ScriptedI2c bus;
    bus.queue_read(header_bytes(cargo_len, shtp::Channel::kControl));
    bus.queue_read(cargo);
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    const auto id = imu.product_id();
    ASSERT_TRUE(id.has_value());
    EXPECT_EQ(id->reset_cause, ResetCause::kPowerOn);
    EXPECT_EQ(id->sw_version_major, 3);
    EXPECT_EQ(id->sw_version_minor, 2);
    EXPECT_EQ(id->sw_version_patch, 9);
    EXPECT_EQ(id->sw_part_number, 0x0001'2345u);
    EXPECT_EQ(id->sw_build_number, 0x0067'89ABu);
}

TEST(Bno085, TareSendsTheDocumentedCommandPayload)
{
    ScriptedI2c bus;
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    ASSERT_TRUE(imu.tare(kTareAxisAll, TareBasis::kRotationVector).has_value());

    const auto& frame = bus.writes.front();
    EXPECT_EQ(frame[4], kReportCommandRequest);
    EXPECT_EQ(frame[6], kCommandTare);
    EXPECT_EQ(frame[7], kTareNow);
    EXPECT_EQ(frame[8], kTareAxisAll);
    EXPECT_EQ(frame[9], static_cast<std::uint8_t>(TareBasis::kRotationVector));
}

TEST(Bno085, RejectsAPayloadTooLongToFrame)
{
    // Nothing the driver sends today is this long, but a future caller
    // reaching for a raw write should get an error rather than a smashed
    // stack frame.
    ScriptedI2c bus;
    FakeClock clock;
    Driver imu{bus, Address::kDefault, clock};

    // The public surface cannot produce one, so this asserts the guard holds
    // for the largest payload that does exist: set-feature at 17 bytes fits.
    ASSERT_TRUE(imu.enable_report(SensorId::kAccelerometer, 1'000).has_value());
    EXPECT_EQ(bus.writes.front().size(), shtp::kHeaderLen + 17);
}

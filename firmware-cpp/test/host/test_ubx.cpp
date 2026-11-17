#include "ping/ubx.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

using namespace ping::ubx;

namespace {

using Bytes = std::vector<std::uint8_t>;

/// Checksum written the way the interface description spells it out, so the
/// tests do not rely on the code under test to build their input.
std::pair<std::uint8_t, std::uint8_t> reference_checksum(const Bytes& bytes_from_class)
{
    std::uint8_t ck_a = 0;
    std::uint8_t ck_b = 0;
    for (const std::uint8_t byte : bytes_from_class) {
        ck_a = static_cast<std::uint8_t>(ck_a + byte);
        ck_b = static_cast<std::uint8_t>(ck_b + ck_a);
    }
    return {ck_a, ck_b};
}

Bytes make_frame(std::uint8_t cls, std::uint8_t id, const Bytes& payload)
{
    Bytes body{cls, id, static_cast<std::uint8_t>(payload.size() & 0xFF), static_cast<std::uint8_t>(payload.size() >> 8)};
    body.insert(body.end(), payload.begin(), payload.end());
    const auto [a, b] = reference_checksum(body);
    Bytes frame{kSync1, kSync2};
    frame.insert(frame.end(), body.begin(), body.end());
    frame.push_back(a);
    frame.push_back(b);
    return frame;
}

void put_u16(Bytes& p, std::size_t off, std::uint16_t v)
{
    p[off] = static_cast<std::uint8_t>(v & 0xFF);
    p[off + 1] = static_cast<std::uint8_t>(v >> 8);
}

void put_u32(Bytes& p, std::size_t off, std::uint32_t v)
{
    for (std::size_t i = 0; i < 4; ++i) {
        p[off + i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
}

void put_i32(Bytes& p, std::size_t off, std::int32_t v)
{
    put_u32(p, off, static_cast<std::uint32_t>(v));
}

/// A NAV-PVT payload laid out per the M9 interface description (92 bytes):
/// a 3D fix near Waterloo, Ontario, moving north-east, with resolved UTC.
Bytes nav_pvt_payload()
{
    Bytes p(92, 0);
    put_u32(p, 0, 123'456'789);       // iTOW ms
    put_u16(p, 4, 2026);              // year
    p[6] = 11;                        // month
    p[7] = 17;                        // day
    p[8] = 14;                        // hour
    p[9] = 30;                        // min
    p[10] = 59;                       // sec
    p[11] = 0b0000'0111;              // valid: date, time, fully resolved
    put_u32(p, 12, 25);               // tAcc ns
    put_i32(p, 16, -250'000'000);     // nano
    p[20] = 3;                        // fixType 3D
    p[21] = 0b0000'0011;              // flags: gnssFixOK, diffSoln
    p[22] = 0;                        // flags2
    p[23] = 17;                       // numSV
    put_i32(p, 24, -805'123'456);     // lon 1e-7 deg
    put_i32(p, 28, 434'654'321);      // lat 1e-7 deg
    put_i32(p, 32, 345'678);          // height mm
    put_i32(p, 36, 312'345);          // hMSL mm
    put_u32(p, 40, 1'250);            // hAcc mm
    put_u32(p, 44, 2'500);            // vAcc mm
    put_i32(p, 48, 1'500);            // velN mm/s
    put_i32(p, 52, 2'000);            // velE
    put_i32(p, 56, -100);             // velD
    put_i32(p, 60, 2'500);            // gSpeed
    put_i32(p, 64, 5'312'345);        // headMot 1e-5 deg
    put_u32(p, 68, 150);              // sAcc mm/s
    put_u32(p, 72, 200'000);          // headAcc 1e-5 deg
    put_u16(p, 76, 123);              // pDOP 0.01
    put_u16(p, 78, 0);                // flags3
    return p;
}

Bytes mon_ver_payload(const std::vector<std::string>& extensions)
{
    Bytes p(40 + 30 * extensions.size(), 0);
    const std::string sw = "EXT CORE 4.04 (7f89f7)";
    const std::string hw = "000A0000";
    std::memcpy(p.data(), sw.data(), sw.size());
    std::memcpy(p.data() + 30, hw.data(), hw.size());
    for (std::size_t i = 0; i < extensions.size(); ++i) {
        std::memcpy(p.data() + 40 + 30 * i, extensions[i].data(), extensions[i].size());
    }
    return p;
}

/// Feeds `bytes` to the parser one at a time and collects every frame, as a
/// copy since payloads only live until the next call.
struct Seen {
    std::uint8_t cls;
    std::uint8_t id;
    Bytes payload;
};

std::vector<Seen> feed(Parser& parser, const Bytes& bytes)
{
    std::vector<Seen> out;
    const auto take = [&out](const std::optional<Frame>& f) {
        if (f) {
            out.push_back({f->cls, f->id, Bytes(f->payload.begin(), f->payload.end())});
        }
    };
    for (const std::uint8_t byte : bytes) {
        take(parser.push(byte));
    }
    // Frames can sit in the buffer behind a corrupted one; drain them.
    for (auto f = parser.poll(); f; f = parser.poll()) {
        take(f);
    }
    return out;
}

Bytes concat(std::initializer_list<Bytes> parts)
{
    Bytes out;
    for (const auto& part : parts) {
        out.insert(out.end(), part.begin(), part.end());
    }
    return out;
}

} // namespace

TEST(UbxChecksum, EmptyIsZero)
{
    EXPECT_EQ(checksum({}), (Checksum{0, 0}));
}

TEST(UbxChecksum, MonVerPollKnownValue)
{
    // The well-known poll request: B5 62 0A 04 00 00 0E 34.
    const std::array<std::uint8_t, 4> body{0x0A, 0x04, 0x00, 0x00};
    EXPECT_EQ(checksum(body), (Checksum{0x0E, 0x34}));
}

TEST(UbxChecksum, WrapsAtEightBits)
{
    const std::array<std::uint8_t, 3> body{0xFF, 0xFF, 0xFF};
    // a: FF, FE, FD; b: FF, FD, FA.
    EXPECT_EQ(checksum(body), (Checksum{0xFD, 0xFA}));
}

TEST(UbxEncode, PollMonVerBytes)
{
    const auto frame = build_poll_mon_ver();
    const std::array<std::uint8_t, 8> expected{0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
    ASSERT_EQ(frame.len, expected.size());
    EXPECT_TRUE(std::equal(expected.begin(), expected.end(), frame.view().begin()));
}

TEST(UbxEncode, MatchesReferenceAndRoundTrips)
{
    const Bytes payload{1, 2, 3, 4, 5};
    std::array<std::uint8_t, 32> out{};
    const auto n = encode(0x06, 0x8A, payload, out);
    ASSERT_TRUE(n);
    EXPECT_EQ(Bytes(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(*n)), make_frame(0x06, 0x8A, payload));
}

TEST(UbxEncode, RejectsSmallBuffer)
{
    const Bytes payload(10, 0);
    std::array<std::uint8_t, 17> out{};
    EXPECT_FALSE(encode(1, 2, payload, out));
    std::array<std::uint8_t, 18> exact{};
    EXPECT_TRUE(encode(1, 2, payload, exact));
}

TEST(UbxConfigure, ExactBytesForTenHertz)
{
    // What neo_m9n.rs sends for configure(10): CFG-VALSET, version 0, layers
    // RAM|BBR, then the six keys in order.
    const std::array<std::uint8_t, 44> expected{
        0xB5, 0x62, 0x06, 0x8A, 0x24, 0x00, // header, length 36
        0x00, 0x03, 0x00, 0x00,             // version, layers, reserved
        0x01, 0x00, 0x73, 0x10, 0x01,       // CFG-UART1INPROT-UBX = 1
        0x01, 0x00, 0x74, 0x10, 0x01,       // CFG-UART1OUTPROT-UBX = 1
        0x02, 0x00, 0x74, 0x10, 0x00,       // CFG-UART1OUTPROT-NMEA = 0
        0x07, 0x00, 0x91, 0x20, 0x01,       // CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1
        0x01, 0x00, 0x21, 0x30, 0x64, 0x00, // CFG-RATE-MEAS = 100
        0x02, 0x00, 0x21, 0x30, 0x01, 0x00, // CFG-RATE-NAV = 1
        0x0B, 0xBA,                         // checksum
    };
    const auto frame = build_configure(10);
    ASSERT_TRUE(frame);
    ASSERT_EQ(frame->len, expected.size());
    EXPECT_TRUE(std::equal(expected.begin(), expected.end(), frame->view().begin()));
}

TEST(UbxConfigure, RateMeasIsIntegerDivision)
{
    const auto one = build_configure(1);
    const auto seven = build_configure(7);
    const auto max = build_configure(25);
    ASSERT_TRUE(one && seven && max);
    // RATE-MEAS value: frame offset 6 + 4 + 4 * 5 + 4 = 34.
    EXPECT_EQ(one->bytes[34] | (one->bytes[35] << 8), 1000);
    EXPECT_EQ(seven->bytes[34] | (seven->bytes[35] << 8), 142);
    EXPECT_EQ(max->bytes[34] | (max->bytes[35] << 8), 40);
}

TEST(UbxConfigure, RejectsRatesOutsideOneToTwentyFive)
{
    EXPECT_FALSE(build_configure(0));
    EXPECT_FALSE(build_configure(26));
    EXPECT_TRUE(build_configure(1));
    EXPECT_TRUE(build_configure(25));
}

TEST(UbxConfigure, FrameParsesBack)
{
    const auto frame = build_configure(5);
    ASSERT_TRUE(frame);
    Parser parser;
    const auto seen = feed(parser, Bytes(frame->view().begin(), frame->view().end()));
    ASSERT_EQ(seen.size(), 1U);
    EXPECT_EQ(seen[0].cls, kClassCfg);
    EXPECT_EQ(seen[0].id, kIdCfgValset);
    EXPECT_EQ(seen[0].payload.size(), 36U);
}

TEST(UbxValset, OverflowYieldsEmptyFrame)
{
    ValsetBuilder builder{kLayerRam};
    for (int i = 0; i < 20; ++i) {
        builder.add_u16(key::kRateMeas, 100);
    }
    EXPECT_TRUE(builder.finish().empty());
}

TEST(UbxParser, ParsesBackToBackFrames)
{
    Parser parser;
    const auto a = make_frame(0x01, 0x07, {1, 2, 3});
    const auto b = make_frame(0x05, 0x01, {0x06, 0x8A});
    const auto seen = feed(parser, concat({a, b}));
    ASSERT_EQ(seen.size(), 2U);
    EXPECT_EQ(seen[0].payload, (Bytes{1, 2, 3}));
    EXPECT_EQ(seen[1].id, 0x01);
}

TEST(UbxParser, EmptyPayloadFrame)
{
    Parser parser;
    const auto seen = feed(parser, make_frame(0x0A, 0x04, {}));
    ASSERT_EQ(seen.size(), 1U);
    EXPECT_TRUE(seen[0].payload.empty());
}

TEST(UbxParser, SkipsNmeaAndGarbage)
{
    Parser parser;
    const std::string nmea = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    const Bytes noise(nmea.begin(), nmea.end());
    const auto frame = make_frame(0x01, 0x07, {9, 9});
    const auto seen = feed(parser, concat({noise, Bytes{0x00, 0xFF, 0xB5, 0x00}, frame, noise}));
    ASSERT_EQ(seen.size(), 1U);
    EXPECT_EQ(seen[0].payload, (Bytes{9, 9}));
}

TEST(UbxParser, ResyncsAfterBadChecksum)
{
    Parser parser;
    auto bad = make_frame(0x01, 0x07, {1, 2, 3, 4});
    bad.back() ^= 0x55;
    const auto good = make_frame(0x05, 0x01, {0x06, 0x8A});
    const auto seen = feed(parser, concat({bad, good}));
    ASSERT_EQ(seen.size(), 1U);
    EXPECT_EQ(seen[0].cls, 0x05);
}

TEST(UbxParser, RecoversFrameInsideCorruptedOne)
{
    // A false header announces a long frame; a genuine frame starts inside it.
    // The parser must find the genuine one once the long frame fails.
    Parser parser;
    const Bytes false_header{kSync1, kSync2, 0x01, 0x07, 0x20, 0x00}; // claims 32 bytes of payload
    const auto good = make_frame(0x05, 0x01, {0x06, 0x8A});           // 10 bytes
    Bytes stream = concat({false_header, good});
    stream.resize(stream.size() + 40, 0x11); // enough filler to complete the false frame
    const auto tail = make_frame(0x01, 0x07, {7});
    const auto seen = feed(parser, concat({stream, tail}));
    ASSERT_GE(seen.size(), 1U);
    EXPECT_EQ(seen[0].cls, 0x05);
    EXPECT_EQ(seen.back().payload, (Bytes{7}));
}

TEST(UbxParser, OversizedLengthIsSkippedWithoutStalling)
{
    Parser parser;
    // Length 0xFFFF: must not make the parser wait for 64 KiB.
    const Bytes huge{kSync1, kSync2, 0x01, 0x07, 0xFF, 0xFF};
    const auto good = make_frame(0x05, 0x01, {0x06, 0x8A});
    const auto seen = feed(parser, concat({huge, good}));
    ASSERT_EQ(seen.size(), 1U);
    EXPECT_EQ(seen[0].cls, 0x05);
}

TEST(UbxParser, MaxPayloadAccepted)
{
    Parser parser;
    const Bytes payload(kMaxPayload, 0xA5);
    const auto seen = feed(parser, make_frame(0x0A, 0x04, payload));
    ASSERT_EQ(seen.size(), 1U);
    EXPECT_EQ(seen[0].payload.size(), kMaxPayload);
}

TEST(UbxParser, OnePastMaxPayloadRejected)
{
    Parser parser;
    const Bytes payload(kMaxPayload + 1, 0xA5);
    EXPECT_TRUE(feed(parser, make_frame(0x0A, 0x04, payload)).empty());
    // And it recovers afterwards.
    EXPECT_EQ(feed(parser, make_frame(0x05, 0x01, {1, 2})).size(), 1U);
}

TEST(UbxParser, FrameSplitAcrossManyPushesAndResetDropsPartial)
{
    Parser parser;
    const auto frame = make_frame(0x01, 0x07, {1, 2, 3, 4, 5, 6});
    for (std::size_t i = 0; i + 1 < frame.size(); ++i) {
        EXPECT_FALSE(parser.push(frame[i]));
    }
    parser.reset();
    // The final byte alone must not complete anything.
    EXPECT_FALSE(parser.push(frame.back()));
    EXPECT_EQ(feed(parser, frame).size(), 1U);
}

TEST(UbxParser, NeverCrashesOnPseudoRandomBytes)
{
    Parser parser;
    std::uint32_t state = 12345;
    std::size_t frames = 0;
    for (int i = 0; i < 200'000; ++i) {
        state = state * 1664525U + 1013904223U;
        // Bias towards the sync characters so candidate frames actually form.
        std::uint8_t byte = static_cast<std::uint8_t>(state >> 24);
        if ((state & 0x30) == 0) {
            byte = (state & 0x40) != 0 ? kSync1 : kSync2;
        }
        if (parser.push(byte)) {
            ++frames;
            while (parser.poll()) {
                ++frames;
            }
        }
    }
    // Random bytes essentially never form a valid frame; the point is to
    // survive.
    EXPECT_LT(frames, 50U);
    EXPECT_EQ(feed(parser, make_frame(0x05, 0x01, {1, 2})).size(), 1U);
}

TEST(UbxNavPvt, DecodesEveryField)
{
    Parser parser;
    const auto seen = feed(parser, make_frame(kClassNav, kIdNavPvt, nav_pvt_payload()));
    ASSERT_EQ(seen.size(), 1U);
    const Frame frame{seen[0].cls, seen[0].id, seen[0].payload};
    const auto s = decode_nav_pvt(frame);
    ASSERT_TRUE(s);

    EXPECT_EQ(s->itow_ms, 123'456'789U);
    EXPECT_EQ(s->fix_type, FixType::kFix3D);
    EXPECT_TRUE(s->fix_ok);
    EXPECT_TRUE(s->differential);
    EXPECT_EQ(s->satellites, 17);
    EXPECT_NEAR(s->latitude_deg, 43.4654321, 1e-9);
    EXPECT_NEAR(s->longitude_deg, -80.5123456, 1e-9);
    EXPECT_FLOAT_EQ(s->height_ellipsoid_m, 345.678F);
    EXPECT_FLOAT_EQ(s->height_msl_m, 312.345F);
    EXPECT_FLOAT_EQ(s->horizontal_accuracy_m, 1.25F);
    EXPECT_FLOAT_EQ(s->vertical_accuracy_m, 2.5F);
    EXPECT_FLOAT_EQ(s->velocity_ned_mps[0], 1.5F);
    EXPECT_FLOAT_EQ(s->velocity_ned_mps[1], 2.0F);
    EXPECT_FLOAT_EQ(s->velocity_ned_mps[2], -0.1F);
    EXPECT_FLOAT_EQ(s->ground_speed_mps, 2.5F);
    EXPECT_FLOAT_EQ(s->speed_accuracy_mps, 0.15F);
    EXPECT_FLOAT_EQ(s->heading_of_motion_deg, 53.12345F);
    EXPECT_FLOAT_EQ(s->heading_accuracy_deg, 2.0F);
    EXPECT_FLOAT_EQ(s->pdop, 1.23F);
    EXPECT_FALSE(s->invalid_position);
    ASSERT_TRUE(s->utc);
    EXPECT_EQ(s->utc->year, 2026);
    EXPECT_EQ(s->utc->month, 11);
    EXPECT_EQ(s->utc->day, 17);
    EXPECT_EQ(s->utc->hour, 14);
    EXPECT_EQ(s->utc->minute, 30);
    EXPECT_EQ(s->utc->second, 59);
    EXPECT_EQ(s->utc->nanosecond, -250'000'000);
    EXPECT_TRUE(s->has_position());
}

TEST(UbxNavPvt, UtcOnlyWhenFullyResolved)
{
    auto p = nav_pvt_payload();
    p[11] = 0b0000'0011; // date and time valid but not fully resolved
    const auto s = decode_nav_pvt({kClassNav, kIdNavPvt, p});
    ASSERT_TRUE(s);
    EXPECT_FALSE(s->utc);
    p[11] = 0b0000'0101; // no valid time
    EXPECT_FALSE(decode_nav_pvt({kClassNav, kIdNavPvt, p})->utc);
}

TEST(UbxNavPvt, PositionNeedsFixTypeFixOkAndValidLlh)
{
    auto base = nav_pvt_payload();
    const auto decode = [](const Bytes& p) { return *decode_nav_pvt({kClassNav, kIdNavPvt, p}); };

    EXPECT_TRUE(decode(base).has_position());

    auto no_ok = base;
    no_ok[21] = 0;
    EXPECT_FALSE(decode(no_ok).has_position());

    auto invalid = base;
    put_u16(invalid, 78, 1);
    EXPECT_TRUE(decode(invalid).invalid_position);
    EXPECT_FALSE(decode(invalid).has_position());

    for (std::uint8_t fix : {std::uint8_t{0}, std::uint8_t{1}, std::uint8_t{5}, std::uint8_t{9}, std::uint8_t{255}}) {
        auto p = base;
        p[20] = fix;
        EXPECT_FALSE(decode(p).has_position()) << int{fix};
    }
    for (std::uint8_t fix : {std::uint8_t{2}, std::uint8_t{3}, std::uint8_t{4}}) {
        auto p = base;
        p[20] = fix;
        EXPECT_TRUE(decode(p).has_position()) << int{fix};
    }
}

TEST(UbxNavPvt, ReservedFixTypeBecomesNoFix)
{
    auto p = nav_pvt_payload();
    p[20] = 200;
    EXPECT_EQ(decode_nav_pvt({kClassNav, kIdNavPvt, p})->fix_type, FixType::kNoFix);
}

TEST(UbxNavPvt, ExtremeCoordinates)
{
    auto p = nav_pvt_payload();
    put_i32(p, 24, -1'800'000'000);
    put_i32(p, 28, 900'000'000);
    const auto s = decode_nav_pvt({kClassNav, kIdNavPvt, p});
    ASSERT_TRUE(s);
    EXPECT_NEAR(s->longitude_deg, -180.0, 1e-9);
    EXPECT_NEAR(s->latitude_deg, 90.0, 1e-9);
}

TEST(UbxNavPvt, RejectsWrongLengthOrMessage)
{
    const auto p = nav_pvt_payload();
    EXPECT_FALSE(decode_nav_pvt({kClassNav, kIdNavPvt, {p.data(), 91}}));
    Bytes longer = p;
    longer.push_back(0);
    EXPECT_FALSE(decode_nav_pvt({kClassNav, kIdNavPvt, longer}));
    EXPECT_FALSE(decode_nav_pvt({kClassNav, 0x08, p}));
    EXPECT_FALSE(decode_nav_pvt({kClassMon, kIdNavPvt, p}));
}

TEST(UbxMonVer, DecodesStringsAndExtensions)
{
    const auto payload = mon_ver_payload({"ROM BASE 0x118B2060", "FWVER=SPG 4.04", "PROTVER=32.01", "MOD=NEO-M9N",
                                          "GPS;GLO;GAL;BDS"});
    const auto v = decode_mon_ver({kClassMon, kIdMonVer, payload});
    ASSERT_TRUE(v);
    EXPECT_STREQ(v->software, "EXT CORE 4.04 (7f89f7)");
    EXPECT_STREQ(v->hardware, "000A0000");
    EXPECT_STREQ(v->module, "NEO-M9N");
    EXPECT_STREQ(v->protocol, "32.01");
    EXPECT_STREQ(v->firmware_type, "SPG");
    EXPECT_EQ(v->firmware_major, 4);
    EXPECT_EQ(v->firmware_minor, 4);
    EXPECT_EQ(v->protocol_major, 32);
    EXPECT_EQ(v->protocol_minor, 1);
}

TEST(UbxMonVer, NoExtensionsLeavesThemEmpty)
{
    const auto v = decode_mon_ver({kClassMon, kIdMonVer, mon_ver_payload({})});
    ASSERT_TRUE(v);
    EXPECT_STREQ(v->module, "");
    EXPECT_STREQ(v->protocol, "");
    EXPECT_STREQ(v->firmware_type, "");
    EXPECT_EQ(v->protocol_major, 0);
}

TEST(UbxMonVer, UnterminatedSoftwareStringIsCut)
{
    Bytes payload(40, 'A');
    const auto v = decode_mon_ver({kClassMon, kIdMonVer, payload});
    ASSERT_TRUE(v);
    EXPECT_EQ(std::strlen(v->software), 30U);
    EXPECT_EQ(std::strlen(v->hardware), 10U);
}

TEST(UbxMonVer, RejectsBadLengths)
{
    EXPECT_FALSE(decode_mon_ver({kClassMon, kIdMonVer, Bytes(39, 0)}));
    EXPECT_FALSE(decode_mon_ver({kClassMon, kIdMonVer, Bytes(41, 0)}));
    EXPECT_FALSE(decode_mon_ver({kClassMon, kIdMonVer, Bytes(69, 0)}));
    EXPECT_TRUE(decode_mon_ver({kClassMon, kIdMonVer, Bytes(70, 0)}));
}

TEST(UbxAck, DecodesAckAndNak)
{
    const Bytes payload{0x06, 0x8A};
    const auto ack = decode_ack({kClassAck, kIdAckAck, payload});
    ASSERT_TRUE(ack);
    EXPECT_EQ(ack->cls, 0x06);
    EXPECT_EQ(ack->id, 0x8A);
    EXPECT_TRUE(ack->accepted);

    const auto nak = decode_ack({kClassAck, kIdAckNak, payload});
    ASSERT_TRUE(nak);
    EXPECT_FALSE(nak->accepted);
}

TEST(UbxAck, RejectsOtherMessagesAndLengths)
{
    const Bytes payload{0x06, 0x8A};
    EXPECT_FALSE(decode_ack({kClassAck, 0x02, payload}));
    EXPECT_FALSE(decode_ack({kClassCfg, kIdAckAck, payload}));
    EXPECT_FALSE(decode_ack({kClassAck, kIdAckAck, Bytes{1}}));
    EXPECT_FALSE(decode_ack({kClassAck, kIdAckAck, Bytes{1, 2, 3}}));
}

TEST(UbxFixType, Strings)
{
    EXPECT_STREQ(to_string(FixType::kNoFix), "no fix");
    EXPECT_STREQ(to_string(FixType::kFix3D), "3D");
    EXPECT_STREQ(to_string(FixType::kTimeOnly), "time only");
}

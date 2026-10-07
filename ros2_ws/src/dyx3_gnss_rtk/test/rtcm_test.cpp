#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>

#include "dyx3_gnss_rtk/correction_health.hpp"
#include "dyx3_gnss_rtk/gga_provider.hpp"
#include "dyx3_gnss_rtk/ntrip_client.hpp"
#include "dyx3_gnss_rtk/rtcm_parser.hpp"
#include "dyx3_gnss_rtk/rtcm_transport.hpp"

using namespace dyx3_gnss_rtk;

namespace {

// Independent bitwise CRC-24Q (no table) to build frames.
uint32_t crc_bitwise(const std::vector<uint8_t>& d, size_t n) {
  uint32_t crc = 0;
  for (size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint32_t>(d[i]) << 16;
    for (int k = 0; k < 8; ++k) {
      crc <<= 1;
      if (crc & 0x1000000U) crc ^= 0x1864CFBU;
    }
  }
  return crc & 0xFFFFFFU;
}

std::vector<uint8_t> make_frame(size_t payload, uint8_t seed, uint8_t reserved_bits = 0) {
  std::vector<uint8_t> f;
  f.push_back(0xD3);
  f.push_back(static_cast<uint8_t>(((payload >> 8) & 0x03) | (reserved_bits & 0xFC)));
  f.push_back(static_cast<uint8_t>(payload & 0xFF));
  for (size_t i = 0; i < payload; ++i) f.push_back(static_cast<uint8_t>(seed * 31U + i * 7U));
  const uint32_t crc = crc_bitwise(f, f.size());
  f.push_back(static_cast<uint8_t>(crc >> 16));
  f.push_back(static_cast<uint8_t>(crc >> 8));
  f.push_back(static_cast<uint8_t>(crc));
  return f;
}

}  // namespace

TEST(Crc24q, CatalogCheckValue) {
  const char* s = "123456789";
  EXPECT_EQ(crc24q(reinterpret_cast<const uint8_t*>(s), 9),
            0xCDE703U);  // CRC-24/LTE-A (RTCM CRC-24Q) check value
  EXPECT_EQ(crc24q(nullptr, 0), 0U);
}
TEST(Crc24q, TableMatchesBitwise) {
  for (size_t n : {1U, 5U, 33U, 200U, 1023U}) {
    const auto f = make_frame(n, static_cast<uint8_t>(n));
    EXPECT_EQ(crc24q(f.data(), f.size() - 3), crc_bitwise(f, f.size() - 3)) << n;
  }
}

TEST(RtcmParser, ExtractsConcatenatedFramesAndHoldsPartials) {
  RtcmParser p;
  const auto a = make_frame(40, 1), b = make_frame(300, 2), c = make_frame(7, 3);
  std::vector<uint8_t> stream;
  for (const auto* f : {&a, &b, &c}) stream.insert(stream.end(), f->begin(), f->end());
  // byte-at-a-time delivery must give the same frames in order
  std::vector<std::vector<uint8_t>> got;
  for (const uint8_t byte : stream) {
    for (auto& f : p.feed(&byte, 1)) got.push_back(f);
  }
  ASSERT_EQ(got.size(), 3U);
  EXPECT_EQ(got[0], a);
  EXPECT_EQ(got[1], b);
  EXPECT_EQ(got[2], c);
  EXPECT_EQ(p.buffered(), 0U);
  EXPECT_EQ(p.crc_failures(), 0U);
}
TEST(RtcmParser, GarbageBetweenFramesIsSkippedAndCounted) {
  RtcmParser p;
  const auto a = make_frame(20, 1), b = make_frame(21, 2);
  std::vector<uint8_t> s = {0x00, 0x11, 0x22};
  s.insert(s.end(), a.begin(), a.end());
  s.insert(s.end(), {0x55, 0x66});
  s.insert(s.end(), b.begin(), b.end());
  const auto got = p.feed(s.data(), s.size());
  ASSERT_EQ(got.size(), 2U);
  EXPECT_EQ(p.resync_bytes(), 5U);
}
TEST(RtcmParser, BadCrcDiscardsTheFrameAndResumesOneByteAfterThePreamble) {
  RtcmParser p;
  auto bad = make_frame(30, 9);
  bad[10] ^= 0x01;  // corrupt payload
  const auto good = make_frame(25, 4);
  // A valid frame hidden INSIDE the claimed span of the corrupt one must still be found.
  std::vector<uint8_t> s(bad.begin(), bad.begin() + 5);
  s.insert(s.end(), good.begin(), good.end());
  s.insert(s.end(), bad.begin() + 5, bad.end());
  const auto got = p.feed(s.data(), s.size());
  bool found = false;
  for (const auto& f : got) found = found || f == good;
  EXPECT_TRUE(found);
  EXPECT_GE(p.crc_failures(), 1U);
}
TEST(RtcmParser, ReservedBitsAreTolerated) {
  RtcmParser p;
  const auto f = make_frame(50, 3, 0xFC);
  const auto got = p.feed(f.data(), f.size());
  ASSERT_EQ(got.size(), 1U);
  EXPECT_EQ(got[0], f);
}
TEST(RtcmParser, BufferIsCappedNotUnbounded) {
  RtcmParser p(4096);
  // A header claiming a 1023-byte frame that never completes, followed by endless data without a
  // valid CRC.
  std::vector<uint8_t> junk(20000, 0xD3);
  for (size_t i = 1; i < junk.size(); i += 3) junk[i] = 0x03, junk[i + 1] = 0xFF;
  p.feed(junk.data(), junk.size());
  EXPECT_LE(p.buffered(), 4096U);
}
TEST(RtcmParser, MaxLengthFrame) {
  RtcmParser p;
  const auto f = make_frame(1023, 5);
  EXPECT_EQ(f.size(), 1029U);
  const auto got = p.feed(f.data(), f.size());
  ASSERT_EQ(got.size(), 1U);
}

// ---- chunking
// ---------------------------------------------------------------------------------------
TEST(Chunker, SmallFrameIsOneUnfragmentedChunkWithSequenceBits) {
  Chunker c;
  const auto f = make_frame(100, 1);
  const auto ch = c.split(f);
  ASSERT_EQ(ch.size(), 1U);
  EXPECT_EQ(ch[0].flags, 0U);  // seq 0, fragment 0, not fragmented
  EXPECT_EQ(ch[0].data, f);
  const auto ch2 = c.split(f);
  EXPECT_EQ(ch2[0].flags, 1U << 3);  // sequence advanced
}
TEST(Chunker, FragmentsCarryTheFlagsTheGpsDriverExpects) {
  Chunker c;
  c.split(make_frame(10, 1));          // advance the sequence to 1
  const auto f = make_frame(1023, 2);  // 1029 bytes -> 4 chunks of 300,300,300,129
  const auto ch = c.split(f);
  ASSERT_EQ(ch.size(), 4U);
  size_t total = 0;
  for (size_t k = 0; k < ch.size(); ++k) {
    EXPECT_EQ(ch[k].flags & 1U, 1U) << k;        // fragmented
    EXPECT_EQ((ch[k].flags >> 1) & 3U, k) << k;  // fragment id
    EXPECT_EQ(ch[k].flags >> 3, 1U) << k;        // same sequence id for all fragments of one frame
    total += ch[k].data.size();
  }
  EXPECT_EQ(total, f.size());
  EXPECT_EQ(ch[0].data.size(), 300U);
  EXPECT_EQ(ch[3].data.size(), 129U);
  std::vector<uint8_t> joined;
  for (const auto& k : ch) joined.insert(joined.end(), k.data.begin(), k.data.end());
  EXPECT_EQ(joined, f);
}
TEST(Chunker, TwoFragmentsAtTheBoundary) {
  Chunker c;
  EXPECT_EQ(c.split(std::vector<uint8_t>(300, 1)).size(), 1U);
  const auto ch = c.split(std::vector<uint8_t>(301, 1));
  ASSERT_EQ(ch.size(), 2U);
  EXPECT_EQ(ch[1].data.size(), 1U);
}
TEST(Chunker, SequenceWrapsModulo32AndOversizeIsRejectedNotTruncated) {
  Chunker c;
  for (int i = 0; i < 32; ++i) c.split(std::vector<uint8_t>(10, 1));
  EXPECT_EQ(c.next_sequence(), 0U);
  EXPECT_TRUE(c.split(std::vector<uint8_t>(1201, 1)).empty());
  EXPECT_TRUE(c.split({}).empty());
  EXPECT_EQ(c.rejected_frames(), 2U);
  EXPECT_EQ(c.next_sequence(), 0U);  // a rejected frame consumes no sequence number
}

// ---- NTRIP protocol
// --------------------------------------------------------------------------------
TEST(Base64, KnownVectors) {
  EXPECT_EQ(base64_encode(""), "");
  EXPECT_EQ(base64_encode("f"), "Zg==");
  EXPECT_EQ(base64_encode("fo"), "Zm8=");
  EXPECT_EQ(base64_encode("foo"), "Zm9v");
  EXPECT_EQ(base64_encode("foobar"), "Zm9vYmFy");
  EXPECT_EQ(base64_encode("user:pass"), "dXNlcjpwYXNz");
}
TEST(NtripProtocol, RequestMatchesThePrototype) {
  const auto r = build_request("caster.example", "MOUNT1", "user", "pass");
  EXPECT_EQ(r,
            "GET /MOUNT1 HTTP/1.0\r\nHost: caster.example\r\nNtrip-Version: "
            "Ntrip/2.0\r\nUser-Agent: NTRIP ROS2/1.0\r\n"
            "Authorization: Basic dXNlcjpwYXNz\r\n\r\n");
}
TEST(NtripProtocol, OnlyExactly200IsSuccess) {
  EXPECT_TRUE(response_is_success("ICY 200 OK"));
  EXPECT_TRUE(response_is_success("icy 200 ok"));
  EXPECT_TRUE(response_is_success("HTTP/1.1 200 OK\r\nServer: x"));
  EXPECT_TRUE(response_is_success("HTTP/1.0 200"));
  EXPECT_TRUE(response_is_success("  HTTP/2 200 OK"));
  EXPECT_FALSE(response_is_success("HTTP/1.1 2000 OK"));
  EXPECT_FALSE(response_is_success("HTTP/1.1 401 Unauthorized"));
  EXPECT_FALSE(response_is_success("ICY 2xx"));
  EXPECT_FALSE(response_is_success("SOURCETABLE 200 OK"));
  EXPECT_FALSE(response_is_success("HTTP/1.1200"));
  EXPECT_FALSE(response_is_success(""));
  EXPECT_FALSE(response_is_success("ICY200"));
}
TEST(NtripProtocol, HeaderParsing) {
  auto v = [](const std::string& s) { return std::vector<uint8_t>(s.begin(), s.end()); };
  const auto tail2 = std::string("\xD3\x00", 2);
  const auto tail3 = std::string("\xD3\x00\x01", 3);
  EXPECT_EQ(parse_response_header(v("HTTP/1.1 200 OK\r\nA: b\r\n")).status, HeaderStatus::NeedMore);
  const auto http = parse_response_header(v("HTTP/1.1 200 OK\r\nA: b\r\n\r\n" + tail2));
  EXPECT_EQ(http.status, HeaderStatus::Complete);
  EXPECT_EQ(http.leftover.size(), 2U);
  const auto icy = parse_response_header(v("ICY 200 OK\r\n" + tail3));
  EXPECT_EQ(icy.status, HeaderStatus::Complete);
  EXPECT_EQ(icy.header, "ICY 200 OK");
  EXPECT_EQ(icy.leftover.size(), 3U);
  EXPECT_EQ(parse_response_header(v(std::string(3000, 'x'))).status, HeaderStatus::TooLong);
}
TEST(NtripProtocol, Backoff) {
  EXPECT_DOUBLE_EQ(backoff_s(0, 5, 60), 5);
  EXPECT_DOUBLE_EQ(backoff_s(1, 5, 60), 10);
  EXPECT_DOUBLE_EQ(backoff_s(3, 5, 60), 40);
  EXPECT_DOUBLE_EQ(backoff_s(4, 5, 60), 60);
  EXPECT_DOUBLE_EQ(backoff_s(100, 5, 60), 60);
}

// ---- GGA
// ---------------------------------------------------------------------------------------------
TEST(Gga, ChecksumAndFormat) {
  EXPECT_EQ(nmea_checksum("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"),
            "47");  // canonical NMEA example
  GgaInput in;
  in.latitude_deg = 48.1173;
  in.longitude_deg = 11.516666667;
  in.altitude_msl_m = 545.4;
  in.utc_epoch_s = 1700000000;  // 2023-11-14 22:13:20 UTC
  in.fix_type = 6;
  in.satellites = 18;
  in.hdop = 0.6;
  const auto s = format_gga(in);
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(s->substr(0, 28), "$GPGGA,221320.00,4807.0380,N");
  EXPECT_NE(s->find(",E,4,18,0.6,545.4,M,0.0,M,,*"), std::string::npos);
  EXPECT_EQ(s->substr(s->size() - 2), "\r\n");
  // checksum is the XOR of everything between $ and *
  const auto star = s->find('*');
  EXPECT_EQ(s->substr(star + 1, 2), nmea_checksum(s->substr(1, star - 1)));
}
TEST(Gga, QualityMapping) {
  GgaInput in;
  in.latitude_deg = 1.0;
  in.longitude_deg = -2.0;
  for (const auto& kv : std::map<int, int>{{3, 1}, {4, 2}, {5, 5}, {6, 4}, {0, 1}}) {
    in.fix_type = static_cast<uint8_t>(kv.first);
    const auto s = *format_gga(in);
    EXPECT_NE(s.find("," + std::to_string(kv.second) + ",0,"), std::string::npos)
        << kv.first << " " << s;
    EXPECT_NE(s.find(",W,"), std::string::npos);
  }
}
TEST(Gga, UnusableFixesAreNotBackFed) {
  GgaInput in;
  in.latitude_deg = 48.0;
  in.longitude_deg = 11.0;
  EXPECT_TRUE(format_gga(in).has_value());
  in.position_age_s = 5.1;
  EXPECT_FALSE(format_gga(in).has_value());
  in.position_age_s = 0;
  in.latitude_deg = std::nan("");
  EXPECT_FALSE(format_gga(in).has_value());
  in.latitude_deg = 91.0;
  EXPECT_FALSE(format_gga(in).has_value());
  in.latitude_deg = 0;
  in.longitude_deg = -181.0;
  EXPECT_FALSE(format_gga(in).has_value());
  in.longitude_deg = 0;
  in.position_age_s = -1.0;
  EXPECT_FALSE(format_gga(in).has_value());
}

// ---- health
// ------------------------------------------------------------------------------------------
TEST(CorrectionHealth, AgeFreshnessAndRate) {
  CorrectionHealth h(10.0);
  EXPECT_FALSE(h.fresh(0.0, 10.0));
  EXPECT_GT(h.age_s(5.0), 1e8);
  h.on_connected(0.0);
  EXPECT_FALSE(h.fresh(1.0, 10.0));                      // connected, but no frame yet
  for (int i = 0; i < 6; ++i) h.on_frame(1.0 + i, 100);  // 1 Hz
  EXPECT_TRUE(h.fresh(6.0, 10.0));
  EXPECT_NEAR(h.age_s(6.5), 0.5, 1e-12);
  EXPECT_NEAR(h.rate_hz(6.5), 1.0, 1e-9);
  EXPECT_TRUE(h.fresh(16.0, 10.0));   // age exactly the limit: still fresh
  EXPECT_FALSE(h.fresh(16.1, 10.0));  // age 10.1
  h.on_disconnected();
  EXPECT_FALSE(h.fresh(6.0, 10.0));  // a disconnected stream is never fresh
  EXPECT_EQ(h.frames(), 6U);
  EXPECT_EQ(h.bytes(), 600U);
}
TEST(FixMonitor, TransitionsAndTheSuspiciousDrop) {
  FixMonitor m;
  EXPECT_FALSE(m.update(0.0, 3, false, 100).has_value());  // first sample: baseline
  EXPECT_FALSE(m.update(1.0, 3, true, 0.5).has_value());
  auto t = m.update(2.0, 5, true, 0.5);
  ASSERT_TRUE(t.has_value());
  EXPECT_EQ(t->from, 3);
  EXPECT_EQ(t->to, 5);
  EXPECT_FALSE(t->suspicious);
  t = m.update(3.0, 6, true, 0.5);
  ASSERT_TRUE(t.has_value());
  t = m.update(4.0, 3, true, 0.5);  // lost RTK while corrections are fresh
  ASSERT_TRUE(t.has_value());
  EXPECT_TRUE(t->suspicious);
  t = m.update(5.0, 6, true, 0.5);
  t = m.update(6.0, 3, false,
               30.0);  // lost RTK because corrections went stale: expected, not suspicious
  ASSERT_TRUE(t.has_value());
  EXPECT_FALSE(t->suspicious);
  EXPECT_EQ(m.transitions(), 5U);
  EXPECT_EQ(m.log().size(), 5U);
}

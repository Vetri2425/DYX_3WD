// Equivalence of the RTCM parser, CRC-24Q, NMEA checksum and GGA formatting against the prototype's
// own code (vectors: tools/gate4/gen_ntrip_vectors.py over PX4_DXP/ntrip_rtcm_node.py; the fixture
// records the source sha256).
#include <gtest/gtest.h>

#include <fstream>
#include <sstream>

#include "dyx3_gnss_rtk/gga_provider.hpp"
#include "dyx3_gnss_rtk/rtcm_parser.hpp"

using namespace dyx3_gnss_rtk;

namespace {

std::vector<uint8_t> from_hex(const std::string& h) {
  std::vector<uint8_t> out;
  if (h == "-") return out;
  for (size_t i = 0; i + 1 < h.size(); i += 2)
    out.push_back(static_cast<uint8_t>(std::stoi(h.substr(i, 2), nullptr, 16)));
  return out;
}

}  // namespace

TEST(NtripEquivalence, AgainstThePrototype) {
  std::ifstream f(DYX3_FIXTURES "/ntrip_vectors.txt");
  ASSERT_TRUE(f.good());
  std::string line;
  std::getline(f, line);
  ASSERT_EQ(line, "NTRIPVEC 1");
  size_t crc = 0, chk = 0, streams = 0, gga = 0, frames_total = 0;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#' || line.rfind("SRC ", 0) == 0) continue;
    std::istringstream in(line);
    std::string kind;
    in >> kind;
    if (kind == "CRC") {
      std::string hex, arrow;
      uint64_t want;
      in >> hex >> arrow >> want;
      const auto d = from_hex(hex);
      EXPECT_EQ(crc24q(d.data(), d.size()), want) << line.substr(0, 60);
      ++crc;
    } else if (kind == "CHK") {
      std::string body, arrow, want;
      in >> body >> arrow >> want;
      EXPECT_EQ(nmea_checksum(body), want);
      ++chk;
    } else if (kind == "STREAM") {
      size_t nchunks, nframes;
      in >> nchunks >> nframes;
      RtcmParser p(1 << 20);  // equivalence: the prototype's buffer is unbounded
      std::vector<std::vector<uint8_t>> got;
      for (size_t k = 0; k < nchunks; ++k) {
        std::getline(f, line);
        const auto bytes = from_hex(line.substr(2));
        for (auto& fr : p.feed(bytes.data(), bytes.size())) got.push_back(std::move(fr));
      }
      ASSERT_EQ(got.size(), nframes) << "stream " << streams;
      for (size_t k = 0; k < nframes; ++k) {
        std::getline(f, line);
        EXPECT_EQ(got[k], from_hex(line.substr(2))) << "stream " << streams << " frame " << k;
      }
      std::getline(f, line);  // END
      frames_total += nframes;
      ++streams;
    } else if (kind == "GGA") {
      double lat, lon, alt, cov;
      int st;
      long long stamp;
      std::string arrow;
      in >> lat >> lon >> alt >> st >> cov >> stamp >> arrow;
      std::string want;
      std::getline(in, want);
      want.erase(0, 1);
      GgaInput gi;
      gi.latitude_deg = lat;
      gi.longitude_deg = lon;
      gi.altitude_msl_m = alt;
      gi.utc_epoch_s = stamp;
      gi.satellites =
          8;  // the prototype used these placeholders; the C++ takes the receiver's own figures
      gi.hdop = 1.0;
      int quality = 1;
      gi.fix_type = 3;
      if (st == 2) {  // STATUS_GBAS_FIX
        quality = cov < 0.01 ? 4 : 5;
        gi.fix_type = quality == 4 ? 6 : 5;
      } else if (st == 1) {  // STATUS_SBAS_FIX
        quality = 2;
        gi.fix_type = 4;
      }
      const auto s = format_gga(gi);
      ASSERT_TRUE(s.has_value());
      std::string got = *s;
      while (!got.empty() && (got.back() == '\r' || got.back() == '\n')) got.pop_back();
      EXPECT_EQ(got, want) << line;
      ++gga;
    } else {
      FAIL() << "unknown vector kind " << kind;
    }
  }
  std::printf("  crc %zu  checksum %zu  streams %zu (frames %zu)  gga %zu\n", crc, chk, streams,
              frames_total, gga);
  EXPECT_GE(crc, 400U);
  EXPECT_GE(streams, 300U);
  EXPECT_GE(gga, 500U);
}

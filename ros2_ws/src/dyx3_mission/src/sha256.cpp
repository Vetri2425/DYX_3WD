// sha256 — FIPS 180-4. Verified against the NIST/RFC 6234 test vectors and against Python hashlib
// on the path-artifact corpus (see test/sha256_test.cpp).
#include "dyx3_mission/sha256.hpp"

#include <array>
#include <cstring>

namespace dyx3_mission {

namespace {

constexpr std::array<std::uint32_t, 64> kK = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

inline std::uint32_t rotr(std::uint32_t x, unsigned n) { return (x >> n) | (x << (32U - n)); }

void compress(std::array<std::uint32_t, 8>& h, const std::uint8_t* block) {
  std::array<std::uint32_t, 64> w{};
  for (std::size_t i = 0; i < 16; ++i) {
    w[i] = (static_cast<std::uint32_t>(block[4 * i]) << 24) |
           (static_cast<std::uint32_t>(block[4 * i + 1]) << 16) |
           (static_cast<std::uint32_t>(block[4 * i + 2]) << 8) |
           static_cast<std::uint32_t>(block[4 * i + 3]);
  }
  for (std::size_t i = 16; i < 64; ++i) {
    const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
    const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
  for (std::size_t i = 0; i < 64; ++i) {
    const std::uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
    const std::uint32_t ch = (e & f) ^ (~e & g);
    const std::uint32_t t1 = hh + s1 + ch + kK[i] + w[i];
    const std::uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
    const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    const std::uint32_t t2 = s0 + maj;
    hh = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }
  h[0] += a;
  h[1] += b;
  h[2] += c;
  h[3] += d;
  h[4] += e;
  h[5] += f;
  h[6] += g;
  h[7] += hh;
}

}  // namespace

std::string sha256_hex(const void* data, std::size_t len) {
  std::array<std::uint32_t, 8> h = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  const auto* p = static_cast<const std::uint8_t*>(data);
  std::size_t off = 0;
  while (len - off >= 64) {
    compress(h, p + off);
    off += 64;
  }
  std::array<std::uint8_t, 128> tail{};
  const std::size_t rem = len - off;
  if (rem > 0) {
    std::memcpy(tail.data(), p + off, rem);
  }
  tail[rem] = 0x80;
  const std::size_t padded = (rem + 1 + 8 <= 64) ? 64 : 128;
  const std::uint64_t bits = static_cast<std::uint64_t>(len) * 8U;
  for (int i = 0; i < 8; ++i) {
    tail[padded - 1 - static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(bits >> (8 * i));
  }
  compress(h, tail.data());
  if (padded == 128) {
    compress(h, tail.data() + 64);
  }
  static const char* kHex = "0123456789abcdef";
  std::string out(64, '0');
  for (std::size_t i = 0; i < 8; ++i) {
    for (std::size_t j = 0; j < 4; ++j) {
      const auto byte = static_cast<std::uint8_t>(h[i] >> (24 - 8 * j));
      out[8 * i + 2 * j] = kHex[byte >> 4];
      out[8 * i + 2 * j + 1] = kHex[byte & 0xF];
    }
  }
  return out;
}

}  // namespace dyx3_mission

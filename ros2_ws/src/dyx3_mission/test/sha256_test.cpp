// SHA-256 against the FIPS 180-4 / RFC 6234 vectors, block-boundary lengths, and Python hashlib.
#include "dyx3_mission/sha256.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

using dyx3_mission::sha256_hex;

TEST(Sha256, Fips180Vectors) {
  EXPECT_EQ(sha256_hex(std::string()),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(sha256_hex(std::string("abc")),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(sha256_hex(std::string("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  EXPECT_EQ(
      sha256_hex(std::string("abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmno"
                             "ijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu")),
      "cf5b16a778af8380036ce59e7b0492370b249b11e8f07a51afac45037afee9d1");
  EXPECT_EQ(sha256_hex(std::string(1000000, 'a')),
            "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(Sha256, PaddingBoundaries) {
  // Lengths around 55/56/63/64/119/120 bytes exercise the one- vs two-block padding paths.
  // Reference values were produced with Python hashlib (see the generator in HANDOFF/tools).
  struct {
    std::size_t n;
    const char* hex;
  } k[] = {
      {55, "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318"},
      {56, "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a"},
      {63, "7d3e74a05d7db15bce4ad9ec0658ea98e3f06eeecf16b4c6fff2da457ddc2f34"},
      {64, "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb"},
      {119, "31eba51c313a5c08226adf18d4a359cfdfd8d2e816b13f4af952f7ea6584dcfb"},
      {120, "2f3d335432c70b580af0e8e1b3674a7c020d683aa5f73aaaedfdc55af904c21c"},
  };
  for (const auto& c : k) {
    EXPECT_EQ(sha256_hex(std::string(c.n, 'a')), c.hex) << "length " << c.n;
  }
}

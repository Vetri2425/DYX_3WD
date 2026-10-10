// Synthetic ULog byte streams for the recorder tests (format: PX4 src/modules/logger/messages.h).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ulog_synth {

using Bytes = std::vector<uint8_t>;

inline Bytes msg(char type, const Bytes& payload) {
  Bytes m{static_cast<uint8_t>(payload.size() & 0xFF), static_cast<uint8_t>(payload.size() >> 8),
          static_cast<uint8_t>(type)};
  m.insert(m.end(), payload.begin(), payload.end());
  return m;
}
inline Bytes text(const std::string& s) { return Bytes(s.begin(), s.end()); }
inline Bytes cat(std::initializer_list<Bytes> parts) {
  Bytes o;
  for (const auto& p : parts) o.insert(o.end(), p.begin(), p.end());
  return o;
}

inline Bytes file_header(uint64_t ts = 123456) {
  Bytes h{'U', 'L', 'o', 'g', 0x01, 0x12, 0x35, 0x01};
  for (int i = 0; i < 8; ++i) h.push_back(static_cast<uint8_t>(ts >> (8 * i)));
  return h;
}
inline Bytes flag_bits() { return msg('B', Bytes(40, 0)); }
inline Bytes format(const std::string& f) { return msg('F', text(f)); }
inline Bytes info(const std::string& key, const std::string& value) {
  Bytes p{static_cast<uint8_t>(key.size())};
  const Bytes k = text(key), v = text(value);
  p.insert(p.end(), k.begin(), k.end());
  p.insert(p.end(), v.begin(), v.end());
  return msg('I', p);
}
inline Bytes param(const std::string& name, float v) {
  const std::string key = "float " + name;
  Bytes p{static_cast<uint8_t>(key.size())};
  const Bytes k = text(key);
  p.insert(p.end(), k.begin(), k.end());
  const auto* b = reinterpret_cast<const uint8_t*>(&v);
  p.insert(p.end(), b, b + 4);
  return msg('P', p);
}
inline Bytes add_logged(uint16_t msg_id, const std::string& name) {
  Bytes p{0, static_cast<uint8_t>(msg_id & 0xFF), static_cast<uint8_t>(msg_id >> 8)};
  const Bytes n = text(name);
  p.insert(p.end(), n.begin(), n.end());
  return msg('A', p);
}
// msg_id + uint64 timestamp + float x, padded with `pad` bytes so messages straddle chunks
inline Bytes data(uint16_t msg_id, uint64_t ts, size_t pad = 30) {
  Bytes p{static_cast<uint8_t>(msg_id & 0xFF), static_cast<uint8_t>(msg_id >> 8)};
  for (int i = 0; i < 8; ++i) p.push_back(static_cast<uint8_t>(ts >> (8 * i)));
  for (size_t i = 0; i < 4 + pad; ++i) p.push_back(static_cast<uint8_t>(ts + i));
  return msg('D', p);
}

// Header + definitions + one subscription: what the FCU sends once after LOGGING_START.
inline Bytes stream_head() {
  return cat({file_header(), flag_bits(), format("pos:uint64_t timestamp;float x;uint8_t[30] pad;"),
              info("char[3] sys_name", "PX4"), param("NAV_ACC_RAD", 2.0F), add_logged(0, "pos")});
}

struct Chunk {
  uint16_t seq;
  uint8_t first_message_offset;
  Bytes data;
};

// Cut `stream` into 249-byte chunks (the UlogStream payload) and compute first_message_offset from
// the message boundaries: `starts` are the byte offsets where messages (or the file header) begin.
inline std::vector<Chunk> chunk(const Bytes& stream, const std::vector<size_t>& starts,
                                uint16_t first_seq = 0, size_t len = 249) {
  std::vector<Chunk> out;
  uint16_t seq = first_seq;
  for (size_t off = 0; off < stream.size(); off += len) {
    const size_t end = std::min(stream.size(), off + len);
    uint8_t fmo = 255;
    for (size_t s : starts) {
      if (s >= off && s < end) {
        fmo = static_cast<uint8_t>(s - off);
        break;
      }
    }
    out.push_back(Chunk{seq++, fmo, Bytes(stream.begin() + off, stream.begin() + end)});
  }
  return out;
}

// A whole stream: head + n data messages. Returns the bytes and the message start offsets.
struct Stream {
  Bytes bytes;
  std::vector<size_t> starts;
  size_t data_start{0};  // offset of the first data message
  std::vector<Chunk> chunks;
};
inline Stream make_stream(int n_data, uint16_t first_seq = 0) {
  Stream s;
  s.starts.push_back(0);
  s.bytes = file_header();
  for (const Bytes& m :
       {flag_bits(), format("pos:uint64_t timestamp;float x;uint8_t[30] pad;"),
        info("char[3] sys_name", "PX4"), param("NAV_ACC_RAD", 2.0F), add_logged(0, "pos")}) {
    s.starts.push_back(s.bytes.size());
    s.bytes.insert(s.bytes.end(), m.begin(), m.end());
  }
  s.data_start = s.bytes.size();
  for (int i = 0; i < n_data; ++i) {
    const Bytes m = (i == n_data / 2) ? param("NAV_ACC_RAD", 3.0F) : data(0, 1000 + i);
    s.starts.push_back(s.bytes.size());
    s.bytes.insert(s.bytes.end(), m.begin(), m.end());
  }
  s.chunks = chunk(s.bytes, s.starts, first_seq);
  return s;
}

// Minimal reader: true when `f` is a ULog file (header + whole known messages to EOF). Fills
// `types` with the message types in order.
inline bool parse(const std::string& f, bool expect_header, std::string* types,
                  std::string* why = nullptr) {
  size_t pos = 0;
  if (expect_header) {
    if (f.size() < 16 || f.compare(0, 7, std::string("ULog\x01\x12\x35", 7)) != 0) {
      if (why) *why = "no ULog magic";
      return false;
    }
    pos = 16;
  }
  while (pos < f.size()) {
    if (f.size() - pos < 3) {
      if (why) *why = "truncated message header at " + std::to_string(pos);
      return false;
    }
    const size_t n = 3 + (static_cast<uint8_t>(f[pos]) | (static_cast<uint8_t>(f[pos + 1]) << 8));
    const char t = f[pos + 2];
    if (std::string("BFIMPQARDLCSO").find(t) == std::string::npos) {
      if (why) *why = std::string("unknown type at ") + std::to_string(pos);
      return false;
    }
    if (f.size() - pos < n) {
      if (why) *why = "truncated message at " + std::to_string(pos);
      return false;
    }
    if (types) *types += t;
    pos += n;
  }
  return true;
}

}  // namespace ulog_synth

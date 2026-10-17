#include <cassert>
#include <cstring>
#include <string>
#include "input/m4_serial_sample.h"
int main() {
  M4SampleFrame f;
  assert(!f.consume('a'));
  for (char c : std::string("\x1e" "0123abcd\n")) assert(f.consume(c));
  assert(f.ready() && std::strcmp(f.nonce(), "0123abcd") == 0);
  f.reset(); assert(!f.consume('a'));
  for (const auto& bad : {"arm", "0123abcd arm", "0123abc", "0123abcd0", "0123ABCd", "motor 4 255"}) {
    f.consume('\x1e');
    for (const char* p=bad; *p; ++p) assert(f.consume(*p));
    assert(f.consume('\n') && f.ready() && !f.nonce()); f.reset();
  }
  f.consume('\x1e');
  for (int i=0;i<10000;++i) assert(f.consume('x'));
  f.consume('\x1e');
  for (char c : std::string("abcd0123\r")) assert(f.consume(c));
  assert(f.ready() && f.nonce()); f.reset(); assert(!f.consume('\n'));
  char out[80];
  const char* boot="0123456789abcdef0123456789abcdef";
  assert(formatM4Sample(out,sizeof(out),"0123abcd",boot,4095,0xfffffff0,4,31)==73);
  assert(std::string(out)=="M4S1 0123abcd 0123456789abcdef0123456789abcdef 0fff fffffff0 00000004 1f\n");
  assert(!formatM4Sample(out,10,"0123abcd",boot,1,1,1,31));
  assert(!formatM4Sample(out,sizeof(out),"arm",boot,1,1,1,31));
  assert(!formatM4Sample(out,sizeof(out),"0123abcd",boot,4096,1,1,31));

  for (char c : std::string("\x1e" "20123abcd\n")) assert(f.consume(c));
  assert(f.fresh() && std::strcmp(f.nonce(), "0123abcd") == 0);
  f.reset();
  PendingM4Sample pending;
  assert(!pending.begin("arm", 100));
  assert(pending.begin("0123abcd", 100));
  assert(!pending.begin("abcd0123", 101)); // never replace a pending nonce
  assert(!pending.ready(99, 110, true));
  assert(!pending.ready(100, 110, true));
  assert(pending.ready(105, 110, true));
  assert(std::strcmp(pending.nonce(), "0123abcd") == 0);
  pending.cancel(); assert(!pending.ready(105, 110, true));
  assert(pending.begin("0123abcd", 100));
  assert(!pending.ready(120, 251, true) && !pending.active());
  assert(pending.begin("0123abcd", 100));
  assert(!pending.ready(120, 130, false) && !pending.active());
  assert(pending.begin("0123abcd", 0xfffffff8));
  assert(pending.ready(2, 4, true));
  char fresh[90];
  assert(formatFreshM4Sample(fresh,sizeof(fresh),pending.nonce(),boot,2000,2,4,pending.receivedMs(),31)==82);
  assert(std::string(fresh).find("00000002 00000004 fffffff8 1f\n") != std::string::npos);
  assert(!formatFreshM4Sample(fresh,sizeof(fresh),"0123abcd",boot,1,100,110,100,31));
  assert(!formatFreshM4Sample(fresh,sizeof(fresh),"0123abcd",boot,1,110,251,100,31));
  assert(!formatFreshM4Sample(fresh,80,"0123abcd",boot,1,110,120,100,31));

  // S3 selects only the representation; framing and the pending-read window
  // remain the same, including old S1 nonces beginning with a version digit.
  f.reset();
  for (char c : std::string("\x1e" "30123abcd\n")) assert(f.consume(c));
  assert(f.fresh() && f.compact() && std::strcmp(f.nonce(), "0123abcd") == 0);
  f.reset();
  for (char c : std::string("\x1e" "3abcdef0\n")) assert(f.consume(c));
  assert(!f.fresh() && !f.compact() && std::strcmp(f.nonce(), "3abcdef0") == 0);
  for (const auto& bad : {"40123abcd", "30123abcd0", "30123abcD", "3arm", "30123abcd motor 4 255"}) {
    f.reset(); f.consume('\x1e');
    for (const char* p = bad; *p; ++p) assert(f.consume(*p));
    assert(f.consume('\n') && f.ready() && !f.nonce() && !f.compact());
  }
  pending.cancel();
  assert(pending.begin("0123abcd", 100, true) && pending.compact());
  assert(!pending.begin("abcd0123", 101, false) && pending.compact());
  assert(pending.ready(101, 250, true));
  pending.cancel(); assert(!pending.compact());
  assert(pending.begin("0123abcd", 100) && !pending.compact());
  pending.cancel();
  assert(pending.begin("0123abcd", 100, true));
  assert(!pending.ready(120, 251, true) && !pending.active() && !pending.compact());
  assert(pending.begin("0123abcd", 100, true));
  assert(!pending.ready(120, 130, false) && !pending.compact());

  // Independent IEEE check vector, plus a Python struct/zlib/Base64 golden
  // vector preserving the full boot ID and wraparound MCU timestamps.
  assert(m4SampleCrc32(reinterpret_cast<const uint8_t*>("123456789"), 9) == 0xcbf43926u);
  assert(m4SampleCrc32(nullptr, 0) == 0);
  uint8_t payload[35] = {0x01,0x23,0xab,0xcd,0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef,
    0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef,0x07,0xd0,0,0,0,2,0,0,0,4,0xff,0xff,0xff,0xf8,0x1f};
  assert(m4SampleCrc32(payload, sizeof(payload)) == 0x45ce4e0bu);
  for (size_t i = 0; i < sizeof(payload); ++i) {
    for (uint8_t bit = 0; bit < 8; ++bit) {
      payload[i] ^= uint8_t(1u << bit);
      assert(m4SampleCrc32(payload, sizeof(payload)) != 0x45ce4e0bu);
      payload[i] ^= uint8_t(1u << bit);
    }
  }
  char compact[60]; memset(compact, '!', sizeof(compact));
  assert(formatCompactFreshM4Sample(compact,59,"0123abcd",boot,2000,2,4,0xfffffff8,31)==58);
  assert(std::string(compact)=="M4S3 ASOrzQEjRWeJq83vASNFZ4mrze8H0AAAAAIAAAAE////+B9Fzk4L\n");
  assert(compact[58]=='\0' && compact[59]=='!' && !std::strchr(compact,'='));
  assert(formatCompactFreshM4Sample(compact,59,"0123abcd",boot,4095,0xffffffff,0,0xfffffffe,31)==58);
  assert(std::string(compact)=="M4S3 ASOrzQEjRWeJq83vASNFZ4mrze8P//////8AAAAA/////h8oAsJy\n");
  assert(formatCompactFreshM4Sample(compact,59,"0123abcd",boot,0,101,250,100,31)==58);
  assert(std::string(compact)=="M4S3 ASOrzQEjRWeJq83vASNFZ4mrze8AAAAAAGUAAAD6AAAAZB8RGTKI\n");
  for (size_t capacity = 0; capacity < 59; ++capacity) {
    memset(compact,'!',sizeof(compact));
    assert(!formatCompactFreshM4Sample(compact,capacity,"0123abcd",boot,1,101,120,100,31));
    for (char c : compact) assert(c=='!');
  }
  assert(!formatCompactFreshM4Sample(nullptr,59,"0123abcd",boot,1,101,120,100,31));
  assert(!formatCompactFreshM4Sample(compact,59,nullptr,boot,1,101,120,100,31));
  assert(!formatCompactFreshM4Sample(compact,59,"0123abcd",nullptr,1,101,120,100,31));
  assert(!formatCompactFreshM4Sample(compact,59,"0123abcD",boot,1,101,120,100,31));
  assert(!formatCompactFreshM4Sample(compact,59,"0123abcd","0123456789abcdef0123456789abcdeFG",1,101,120,100,31));
  assert(!formatCompactFreshM4Sample(compact,59,"0123abcd",boot,4096,101,120,100,31));
  assert(!formatCompactFreshM4Sample(compact,59,"0123abcd",boot,1,100,120,100,31));
  assert(!formatCompactFreshM4Sample(compact,59,"0123abcd",boot,1,121,120,100,31));
  assert(!formatCompactFreshM4Sample(compact,59,"0123abcd",boot,1,101,251,100,31));
}

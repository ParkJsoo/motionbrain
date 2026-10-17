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
}

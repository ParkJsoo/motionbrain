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
}

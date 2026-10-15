#include "source_frame_metadata.h"
#include <cassert>
#include <cstring>
#include <limits>
#include <random>
#include <vector>
int main() {
 const char* boot="0123456789abcdef0123456789abcdef";
 std::mt19937_64 random(11);
 for (unsigned i=0;i<100000;i++) {
  const int64_t now=1+(random() % (std::numeric_limits<int64_t>::max()-1));
  const int64_t ts=1+(random()%now);
  auto m=camera_source::identify(boot,ts/1000000,ts%1000000,now);
  assert(m.valid && m.capturedUs==uint64_t(ts));
  char full[512];auto n=camera_source::headers(m,full,sizeof(full));
  assert(n>0 && n==strlen(full));
  size_t cap=i%514;
  std::vector<char> b(cap+2,'X');
  auto length=camera_source::headers(m,b.data()+1,cap);
  assert(b.front()=='X' && b.back()=='X');
  if(cap>n) assert(length==n && !strcmp(full,b.data()+1));
  else {assert(length==0);if(cap)assert(b[1]==0);}
 }
 for(size_t len=0;len<32;len++) {
  std::vector<char>b(len+1,'a');b[len]=0;
  assert(!camera_source::identify(b.data(),1,0,2000000).valid);
 }
 auto m=camera_source::identify(boot,9223372036854LL,775807,9223372036854775807LL);
 assert(m.valid && m.capturedUs==9223372036854775807ULL);
}

#include <initializer_list>
#include <cassert>
#include <string>
#include <utility>
#include <vector>
#include "network/output_page.h"
struct QueryServer {
  std::vector<std::pair<std::string,std::string>> arguments;
  int args() const { return static_cast<int>(arguments.size()); }
  std::string argName(int index) const { return arguments[index].first; }
  std::string arg(const char* key) const {
    for (const auto& argument:arguments) if (argument.first==key) return argument.second;
    return "";
  }
};
int main() {
  uint32_t value=99;
  for (const char* bad : {"", "-1", "+1", "1.0", "1x", " 1", "4294967296"}) {
    assert(!collection_http::unsignedArgument(bad, value)); assert(value==99);
  }
  assert(collection_http::unsignedArgument("4294967295",value) && value==UINT32_MAX);
  assert(collection_http::unsignedArgument("0",value) && value==0);
  bounded::AppliedOutputLog log;
  auto page=collection_http::outputPage(log,0,8); assert(page.begin==0 && page.end==0 && !page.gap && !page.hasMore);
  assert(collection_http::outputPage(log,1,8).gap);
  for (uint32_t n=1;n<=140;++n) log.append(n,0,1,0,bounded::OutputReason::ForcedOff);
  page=collection_http::outputPage(log,0,8);
  assert(page.oldest==13 && page.latest==140 && !page.gap && page.begin==0 && page.end==8 && page.hasMore);
  assert(collection_http::outputPage(log,11,8).gap);
  assert(!collection_http::outputPage(log,12,8).gap);
  page=collection_http::outputPage(log,138,8); assert(page.end-page.begin==2 && !page.hasMore && !page.gap);
  page=collection_http::outputPage(log,140,8); assert(page.end==page.begin && !page.hasMore && !page.gap);
  assert(collection_http::outputPage(log,141,8).gap);
  const char* boot="0123456789abcdef0123456789abcdef";
  assert(collection_http::latestOutputSequence(log)==140);
  assert(collection_http::stableOutputCursor(140,140,boot,boot));
  assert(!collection_http::stableOutputCursor(140,141,boot,boot));
  assert(!collection_http::stableOutputCursor(140,140,boot,"ffffffffffffffffffffffffffffffff"));
  assert(!collection_http::stableOutputCursor(0,0,boot,boot));
  assert(!collection_http::stableOutputCursor(140,140,"",boot));
  // OFF→nonzero→OFF cannot retain the old cursor, even if final PWM is zero.
  log.append(141,0,1,20,bounded::OutputReason::DriverWrite);
  log.append(142,0,1,0,bounded::OutputReason::DriverWrite);
  assert(collection_http::latestOutputSequence(log)==142);
  assert(!collection_http::stableOutputCursor(140,142,boot,boot));
  // 결합 endpoint는 정확히 두 개의 서로 다른 인수만 받는다.
  uint32_t cursor=99,limit=99;
  QueryServer query{{{"after","0"},{"limit","8"}}};
  assert(collection_http::observationPageArguments(query,cursor,limit) && cursor==0 && limit==8);
  query.arguments={{"limit","1"},{"after","4294967295"}};
  assert(collection_http::observationPageArguments(query,cursor,limit) && cursor==UINT32_MAX && limit==1);
  const std::vector<std::vector<std::pair<std::string,std::string>>> badQueries={
    {},{{"after","0"}},{{"limit","8"}},{{"after","0"},{"after","1"}},
    {{"after","0"},{"limit","8"},{"limit","8"}},{{"after","0"},{"unknown","8"}},
    {{"after","-1"},{"limit","8"}},{{"after","+1"},{"limit","8"}},
    {{"after","4294967296"},{"limit","8"}},{{"after"," 1"},{"limit","8"}},
    {{"after","1.0"},{"limit","8"}},{{"after",""},{"limit","8"}},
    {{"after","0"},{"limit","0"}},{{"after","0"},{"limit","9"}},
    {{"after","0"},{"limit","8x"}},{{"after","0"},{"limit","1.0"}}};
  for (const auto& args:badQueries) {
    query.arguments=args;cursor=99;limit=99;
    assert(!collection_http::observationPageArguments(query,cursor,limit));
    assert(cursor==99 && limit==99);
  }
  assert(collection_http::stableCollectionObservation(142,142,142,boot,boot,boot));
  assert(!collection_http::stableCollectionObservation(140,142,142,boot,boot,boot));
  assert(!collection_http::stableCollectionObservation(142,142,141,boot,boot,boot));
  assert(!collection_http::stableCollectionObservation(142,142,142,boot,boot,"ffffffffffffffffffffffffffffffff"));
  assert(!collection_http::stableCollectionObservation(142,142,142,boot,"ffffffffffffffffffffffffffffffff",boot));
  assert(!collection_http::stableCollectionObservation(0,0,0,boot,boot,boot));
  assert(!collection_http::stableCollectionObservation(142,142,142,boot,boot,nullptr));

}

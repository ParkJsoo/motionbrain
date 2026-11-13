#pragma once
#include <stdint.h>
#include <string.h>
#include "motor/applied_output_log.h"

namespace collection_http {
inline bool unsignedArgument(const char* text, uint32_t& value) {
  if (!text || !*text) return false;
  uint32_t parsed = 0;
  for (const char* p=text; *p; ++p) {
    if (*p<'0' || *p>'9') return false;
    const uint32_t digit = static_cast<uint32_t>(*p-'0');
    if (parsed > (UINT32_MAX-digit)/10) return false;
    parsed = parsed*10+digit;
  }
  value=parsed; return true;
}
struct OutputPage {
  size_t begin=0, end=0;
  uint32_t oldest=0, latest=0;
  bool gap=false, hasMore=false;
};
inline OutputPage outputPage(const bounded::AppliedOutputLog& log, uint32_t after, size_t limit) {
  OutputPage page;
  if (log.count()) {
    page.oldest=log.at(0)->sequence;
    page.latest=log.at(log.count()-1)->sequence;
  }
  page.gap = after > page.latest || (after != 0 && page.oldest > 0 && after < page.oldest-1);
  while (page.begin<log.count() && log.at(page.begin)->sequence<=after) ++page.begin;
  page.end=page.begin;
  while (page.end<log.count() && page.end-page.begin<limit) ++page.end;
  page.hasMore=page.end<log.count();
  return page;
}
inline uint32_t latestOutputSequence(const bounded::AppliedOutputLog& log) {
  return log.count() ? log.at(log.count()-1)->sequence : 0;
}
inline bool stableOutputCursor(uint32_t before, uint32_t after, const char* bootBefore, const char* bootAfter) {
  return before > 0 && before == after && bootBefore && bootAfter &&
    strlen(bootBefore) == 32 && strcmp(bootBefore, bootAfter) == 0;
}
}

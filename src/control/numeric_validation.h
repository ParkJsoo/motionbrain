#ifndef MOTIONBRAIN_NUMERIC_VALIDATION_H
#define MOTIONBRAIN_NUMERIC_VALIDATION_H
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

namespace NumericValidation {
inline bool integer(const char* text, long minimum, long maximum, long& out) {
  if (!text || !*text) return false;
  const char* digits = text;
  if (*digits == '-') ++digits;
  if (!*digits) return false;
  for (const char* p = digits; *p; ++p) if (*p < '0' || *p > '9') return false;
  errno = 0;
  char* end = nullptr;
  const long parsed = strtol(text, &end, 10);
  if (errno == ERANGE || *end || parsed < minimum || parsed > maximum) return false;
  out = parsed;
  return true;
}
inline bool real(const char* text, float minimum, float maximum, float& out) {
  if (!text || !*text) return false;
  for (const char* p = text; *p; ++p)
    if (!((*p >= '0' && *p <= '9') || *p == '-' || *p == '+' ||
          *p == '.' || *p == 'e' || *p == 'E')) return false;
  errno = 0;
  char* end = nullptr;
  const float parsed = strtof(text, &end);
  if (errno == ERANGE || end == text || *end || !isfinite(parsed) ||
      parsed < minimum || parsed > maximum) return false;
  out = parsed;
  return true;
}
}
#endif

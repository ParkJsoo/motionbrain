#include "finite_profile_contract.h"

#if MOTIONBRAIN_ENABLE_FINITE_PROFILE_CORE
#include <string.h>

namespace finite_profile {

bool validIdentifier(const char* value, size_t capacity) {
  if (!value || !capacity || !value[0]) return false;
  for (size_t i = 0; i < capacity; ++i) {
    const unsigned char ch = static_cast<unsigned char>(value[i]);
    if (!ch) return true;
    if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
          (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' || ch == '.' || ch == ':'))
      return false;
  }
  return false;
}

bool validProfile(const Profile& p) {
  if (p.contractVersion != 1 || p.profileVersion != 1 || p.timingPolicyVersion != 1 ||
      !validIdentifier(p.profileId, sizeof(p.profileId)) || p.motor != 2 ||
      (p.direction != 1 && p.direction != -1) || p.count < 2 || p.count > MAX_POINTS ||
      p.startLeadMs > MAX_WINDOW_MS || p.maxLatenessMs > MAX_WINDOW_MS ||
      p.offConfirmWindowMs == 0 || p.offConfirmWindowMs > MAX_WINDOW_MS ||
      p.admissionWindowMs == 0 || p.admissionWindowMs > MAX_WINDOW_MS)
    return false;
  // Lower-case SHA-256 text identifying the trusted catalogue. No truncation.
  for (size_t i = 0; i < 64; ++i) {
    const char ch = p.catalogHash[i];
    if (!((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f'))) return false;
  }
  if (p.catalogHash[64] != '\0' || p.points[0].offsetMs != 0 ||
      p.points[p.count-1].offsetMs > MAX_PROFILE_MS || p.points[p.count-1].pwm != 0)
    return false;
  for (size_t i = 0; i < p.count; ++i) {
    const Point& point = p.points[i];
    if (point.pwm < -255 || point.pwm > 255) return false;
    if (i+1 < p.count && (point.pwm == 0 || (point.pwm > 0 ? 1 : -1) != p.direction))
      return false;
    if (i && (point.offsetMs <= p.points[i-1].offsetMs ||
              point.offsetMs-p.points[i-1].offsetMs <= p.maxLatenessMs)) return false;
  }
  // Unused array slots are not part of the point list or execution semantics.
  return true;
}

std::string canonicalProfile(const Profile& p) {
  if (!validProfile(p)) return {};
  std::string out = "motionbrain.finite_profile.v1\n";
  out += "contractVersion=" + std::to_string(p.contractVersion) + "\n";
  out += "profileVersion=" + std::to_string(p.profileVersion) + "\n";
  out += "timingPolicyVersion=" + std::to_string(p.timingPolicyVersion) + "\n";
  out += "catalogHash="; out += p.catalogHash; out += '\n';
  out += "profileId="; out += p.profileId; out += '\n';
  out += "motor=" + std::to_string(p.motor) + "\n";
  out += "direction=" + std::to_string(p.direction) + "\n";
  out += "startLeadMs=" + std::to_string(p.startLeadMs) + "\n";
  out += "maxLatenessMs=" + std::to_string(p.maxLatenessMs) + "\n";
  out += "offConfirmWindowMs=" + std::to_string(p.offConfirmWindowMs) + "\n";
  out += "admissionWindowMs=" + std::to_string(p.admissionWindowMs) + "\n";
  out += "pointCount=" + std::to_string(p.count) + "\n";
  for (size_t i = 0; i < p.count; ++i)
    out += "point=" + std::to_string(p.points[i].offsetMs) + "," +
           std::to_string(p.points[i].pwm) + "\n";
  return out;
}

bool sameProfile(const Profile& a, const Profile& b) {
  if (!validProfile(a) || !validProfile(b)) return false;
  if (a.contractVersion != b.contractVersion || a.profileVersion != b.profileVersion ||
      a.timingPolicyVersion != b.timingPolicyVersion || strcmp(a.catalogHash,b.catalogHash) ||
      strcmp(a.profileId,b.profileId) || a.motor != b.motor || a.direction != b.direction ||
      a.startLeadMs != b.startLeadMs || a.maxLatenessMs != b.maxLatenessMs ||
      a.offConfirmWindowMs != b.offConfirmWindowMs ||
      a.admissionWindowMs != b.admissionWindowMs || a.count != b.count) return false;
  for (size_t i = 0; i < a.count; ++i)
    if (a.points[i].offsetMs != b.points[i].offsetMs || a.points[i].pwm != b.points[i].pwm)
      return false;
  return true;
}

} // namespace finite_profile
#endif

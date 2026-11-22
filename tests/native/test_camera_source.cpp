#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>
#include "source_frame_metadata.h"
#include "bounded_response_write.h"

void testBoundedResponseWriter() {
  using camera_http::SendResult;
  const uint8_t payload[] = {1, 2, 3, 4, 5};
  uint32_t now = 100;
  auto clock = [&]() { return now; };
  auto pause = [&]() { ++now; };
  size_t written = 0;
  unsigned calls = 0;
  assert(camera_http::writeBounded(payload, sizeof(payload), 3, now, 5, written,
      [&](const uint8_t* data, size_t count) -> SendResult {
        ++calls;
        assert(data == payload + written && count <= 3);
        if (calls == 1) return {0, true};
        return {count > 2 ? size_t{2} : count, false};
      }, clock, pause));
  assert(written == sizeof(payload) && calls == 4 && now == 101);

  // Header may have consumed almost the whole response deadline, even across
  // the millisecond counter wrap. A blocked body must get only what remains.
  const uint32_t responseStart = std::numeric_limits<uint32_t>::max() - 2;
  now = 0;
  calls = 0;
  assert(!camera_http::writeBounded(payload, sizeof(payload), 3, responseStart, 5, written,
      [&](const uint8_t*, size_t) -> SendResult { ++calls; return {0, true}; }, clock, pause));
  assert(written == 0 && calls == 2 && now == 2);

  calls = 0;
  assert(!camera_http::writeBounded(payload, sizeof(payload), 3, now, 5, written,
      [&](const uint8_t*, size_t) -> SendResult {
        return ++calls == 1 ? SendResult{2, false} : SendResult{0, false};
      }, clock, pause));
  assert(written == 2 && calls == 2);
}

int main(int argc, char** argv) {
  testBoundedResponseWriter();
  const char* boot = "0123456789abcdef0123456789abcdef";
  const auto first = camera_source::identify(boot, 12, 345, 13000000);
  assert(first.valid && first.capturedUs == 12000345 && first.sourceAgeUs == 999655);
  const auto repeated = camera_source::identify(boot, 12, 345, 14000000);
  assert(repeated.valid && !strcmp(first.frameId, repeated.frameId));
  assert(repeated.sourceAgeUs == first.sourceAgeUs + 1000000);
  const auto next = camera_source::identify(boot, 12, 346, 13000000);
  assert(next.valid && strcmp(next.frameId, first.frameId));
  const auto rebooted = camera_source::identify("abcdef0123456789abcdef0123456789", 12, 345, 13000000);
  assert(rebooted.valid && strcmp(first.frameId, rebooted.frameId));
  assert(!camera_source::identify(boot, -1, 0, 10).valid);
  assert(!camera_source::identify(boot, 0, 0, 10).valid);
  assert(!camera_source::identify(boot, 0, 11, 10).valid);
  assert(!camera_source::identify(boot, 0, -1, 10).valid);
  assert(!camera_source::identify(boot, 0, 1000000, 2000000).valid);
  assert(!camera_source::identify(boot, std::numeric_limits<int64_t>::max(), 0, 2000000).valid);
  assert(!camera_source::identify("bad\r\nid", 1, 0, 2000000).valid);
  assert(!camera_source::identify(nullptr, 1, 0, 2000000).valid);
  char tiny[8];
  assert(camera_source::headers(first, tiny, sizeof(tiny)) == 0 && tiny[0] == 0);
  char out[512];
  const size_t size = camera_source::headers(first, out, sizeof(out));
  assert(size == strlen(out) && size > 0);
  assert(strstr(out, "X-MotionBrain-Captured-Us: 12000345\r\n"));
  assert(strstr(out, "first_dma_buffer\r\n"));
  if (argc > 1 && !strcmp(argv[1], "headers")) std::cout << out;
}

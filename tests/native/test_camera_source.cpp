#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>
#include "source_frame_metadata.h"

int main(int argc, char** argv) {
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

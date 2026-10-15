#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>
#include "fresh_capture.h"
struct Frame { struct { int64_t tv_sec, tv_usec; } timestamp; };
static Frame frame(int64_t us) { return {{us/1000000, us%1000000}}; }
const char* boot = "0123456789abcdef0123456789abcdef";
struct Fixture {
  int64_t now = 1000000, step = 10000;
  bool connected = true, disconnectOnGet = false;
  unsigned gets = 0;
  std::vector<Frame*> frames, returned;
  camera_fresh::Result result;
  Frame* run(int64_t requested = 1000000, const char* id = boot) {
    return camera_fresh::acquire<Frame>(id, requested, result,
      [&]() { now += step; if (disconnectOnGet) connected = false;
              return gets < frames.size() ? frames[gets++] : (++gets, nullptr); },
      [&](Frame* f) { returned.push_back(f); }, [&]() { return now; },
      [&]() { return connected; }, [&]() { now += 1000; });
  }
};
int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string name = argv[1];
  if (name == "request_mode") {
    using camera_fresh::Mode;
    assert(camera_fresh::mode(0, nullptr) == Mode::Legacy);
    assert(camera_fresh::mode(1, "1") == Mode::Fresh);
    assert(camera_fresh::mode(2, "1") == Mode::Invalid);
    for (const char* value : {"0", "", "true", "01", " 1", "1 "})
      assert(camera_fresh::mode(1, value) == Mode::Invalid);
  } else if (name == "old_then_new") {
    auto old = frame(900000), fresh = frame(1010000);
    Fixture f; f.frames = {&old,&fresh};
    assert(f.run() == &fresh && f.gets == 2 && f.returned.size() == 1 && f.returned[0] == &old);
    assert(f.result.discarded == 1 && f.result.error == nullptr);
  } else if (name == "equal_repeated_limit") {
    auto equal = frame(1000000); Fixture f; f.frames = {&equal,&equal,&equal,&equal};
    assert(!f.run() && f.gets == 4 && f.returned.size() == 4);
    assert(!strcmp(f.result.error, "fresh_frame_limit"));
  } else if (name == "null") {
    Fixture f; assert(!f.run() && f.gets == 1 && f.returned.empty());
    assert(!strcmp(f.result.error, "fresh_capture_failed"));
  } else if (name == "invalid_timestamp") {
    for (int64_t stamp : {0LL, 9000000LL, -1LL}) {
      auto bad = frame(stamp); Fixture f; f.frames = {&bad};
      assert(!f.run() && f.returned.size() == 1);
      assert(!strcmp(f.result.error, "fresh_invalid_timestamp"));
    }
  } else if (name == "deadline") {
    Fixture expired; expired.now = 1200000;
    assert(!expired.run() && expired.gets == 0);
    for (int64_t delay : {200000LL, 4000000LL}) {
      auto candidate = frame(1000001); Fixture f; f.step = delay; f.frames = {&candidate};
      assert(!f.run() && f.gets == 1 && f.returned.size() == 1);
      assert(!strcmp(f.result.error, "fresh_budget_exceeded"));
    }
    auto old = frame(900000); Fixture f; f.step = 199999; f.frames = {&old};
    assert(!f.run() && f.gets == 1 && f.returned.size() == 1);
  } else if (name == "clock") {
    Fixture before; before.now = 999999; assert(!before.run() && before.gets == 0);
    auto candidate = frame(900000); Fixture f; f.step = -1; f.frames = {&candidate};
    assert(!f.run() && f.returned.size() == 1);
    assert(!strcmp(f.result.error, "fresh_invalid_clock"));
  } else if (name == "disconnect") {
    Fixture before; before.connected = false; assert(!before.run() && before.gets == 0);
    auto candidate = frame(1000001); Fixture f; f.frames = {&candidate}; f.disconnectOnGet = true;
    assert(!f.run() && f.returned.size() == 1);
    assert(!strcmp(f.result.error, "fresh_client_disconnected"));
  } else if (name == "boot_and_headers") {
    auto candidate = frame(1000001); Fixture bad; bad.frames = {&candidate};
    assert(!bad.run(1000000, "bad") && bad.returned.size() == 1);
    char tiny[8], out[192];
    assert(camera_fresh::headers(1000000,0,tiny,sizeof(tiny)) == 0 && tiny[0] == 0);
    assert(camera_fresh::headers(1000000,4,out,sizeof(out)) == 0);
    assert(camera_fresh::headers(0,0,out,sizeof(out)) == 0);
    assert(camera_fresh::headers(1000000,1,out,sizeof(out)) > 0);
    assert(strstr(out,"after_request\r\n"));
    char source[512];
    camera_source::headers(camera_source::identify(boot,1,1,1010000),source,sizeof(source));
    std::cout << source << out;
  } else { assert(false); }
}

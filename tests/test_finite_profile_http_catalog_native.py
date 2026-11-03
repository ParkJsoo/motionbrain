"""Actual finite HTTP handlers, real runtime/core/driver, fake HTTP and Arduino IO."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FiniteProfileHttpCatalogTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('clang++') or shutil.which('g++')
        if not compiler:
            raise unittest.SkipTest('C++ compiler unavailable')
        cls.temp = tempfile.TemporaryDirectory(prefix='finite-http-catalog-')
        cls.addClassCleanup(cls.temp.cleanup)
        folder = Path(cls.temp.name)
        string_header = folder/'http_string.h'
        string_header.write_text('#define String ExistingServiceString\n#include "'+
                                 str(ROOT/'tests/native/service_stubs/Arduino.h')+'"\n#undef String\n'+r'''
#pragma once
#include <string>
#include <type_traits>
class String {
public:
  std::string value;
  String() {}
  String(const char* text) : value(text) {}
  String(const std::string& text) : value(text) {}
  String(float number,unsigned precision) {
    char buffer[80];snprintf(buffer,sizeof(buffer),"%.*f",static_cast<int>(precision),number);value=buffer;
  }
  const char* c_str() const { return value.c_str(); }
  size_t length() const { return value.size(); }
  char operator[](size_t i) const { return value[i]; }
  bool operator==(const String& other) const { return value==other.value; }
  bool operator!=(const String& other) const { return value!=other.value; }
  String& operator+=(const String& text) { value+=text.value;return *this; }
  String& operator+=(const char* text) { value+=text;return *this; }
  String& operator+=(char text) { value+=text;return *this; }
  template<class T> typename std::enable_if<std::is_integral<T>::value,String&>::type
  operator+=(T n) { value+=std::to_string(n);return *this; }
};
''')
        source = (ROOT/'src/network/web_server.cpp').read_text()
        status = source.split('void MotionBrainWebServer::handleFiniteProfileStatus()',1)[1]
        status = 'void HTTPHarness::handleFiniteProfileStatus()' + status.split('void MotionBrainWebServer::handleFiniteProfile()',1)[0]
        post = source.split('void MotionBrainWebServer::handleFiniteProfile()',1)[1].split('\n#endif',1)[0]
        post = 'void HTTPHarness::handleFiniteProfile()' + post
        harness = r'''
#define main existing_runtime_main
#include "tests/native/test_finite_profile_runtime.cpp"
#undef main
#include "control/finite_profile_json.h"
#include "control/numeric_validation.h"
#include <iostream>
#include <vector>
struct FakeServer {
  std::vector<std::pair<String,String>> fields;
  int status=0;
  String response;
  int args() const { return fields.size(); }
  bool hasArg(const char* name) const { for(const auto& p:fields)if(p.first==name)return true;return false; }
  String arg(const char* name) const { for(const auto& p:fields)if(p.first==name)return p.second;return ""; }
  String argName(int i) const { return fields[i].first; }
  void send(int code,const char*,const String& body) {status=code;response=body;}
};
struct FakeESP { unsigned getFreeHeap() const {return 100000;} } ESP;
struct HTTPDebugLog { static unsigned suppressedDuringFiniteOutput(){return 0;} };
static void sendNoStoreHeaders(FakeServer&) {}
static size_t httpStrlcpy(char* dest,const char* source,size_t capacity) {
  const size_t length=strlen(source);
  if(capacity) {const size_t copied=length<capacity-1?length:capacity-1;memcpy(dest,source,copied);dest[copied]=0;}
  return length;
}
class HTTPHarness {
public:
  FakeServer server_;
  bool authorized=true;
  bool requireCommandAuth(){if(authorized)return true;server_.send(403,"application/json","{}");return false;}
  void sendErrorJson(int code,const char* reason){server_.send(code,"application/json",reason);}
  void handleFiniteProfileStatus();
  void handleFiniteProfile();
};
#define DebugLog HTTPDebugLog
#define strlcpy httpStrlcpy
''' + status + post + r'''
#undef DebugLog
#undef strlcpy
static void select(HTTPHarness& http,size_t index,const char* id) {
  const auto* p=runtime::profileAt(index);
  http.server_.fields={{"operation","prepare"},{"bootId",runtime::bootId()},{"commandId",id},
    {"epoch",std::to_string(runtime::driver()->epoch())},{"profileId",p->profileId},{"catalogHash",p->catalogHash}};
}
int main(int argc,char** argv) {
  assert(argc==2);assert(motorControl.init());systemState.state=SystemState::ARMED;BoundedActions::init();
  const TrialCatalog catalog;
  assert(runtime::init(motorControl,catalog.profiles,4,BoundedActions::executor()->bootId(),issuer));
  HTTPHarness http;
  if(!strcmp(argv[1],"status")) {
    http.handleFiniteProfileStatus();assert(http.server_.status==200);std::cout<<http.server_.response.value;
  } else if(!strcmp(argv[1],"selection")) {
    for(size_t i=0;i<4;++i) {
      const std::string id="http-"+std::to_string(i);select(http,i,id.c_str());http.handleFiniteProfile();
      assert(http.server_.status==200);
      const auto* r=runtime::driver()->find(runtime::bootId(),id.c_str());assert(r);
      assert(sameProfile(r->request.profile,*runtime::profileAt(i)));
      http.server_.fields={{"operation","cancel"},{"bootId",runtime::bootId()},{"commandId",id},
        {"epoch",std::to_string(r->request.expectedEpoch)}};
      http.handleFiniteProfile();assert(http.server_.status==200);
    }
    http.server_.fields.clear();http.handleFiniteProfileStatus();
    assert(nonzeroWrites==0);std::cout<<http.server_.response.value;
  } else if(!strcmp(argv[1],"forbidden")) {
    select(http,0,"reject");http.authorized=false;http.handleFiniteProfile();assert(http.server_.status==403);
    http.authorized=true;
    for(const char* field:{"pwm","durationMs","points"}) {
      select(http,0,"reject");http.server_.fields.push_back({field,"50"});http.handleFiniteProfile();assert(http.server_.status==400);
    }
    select(http,0,"reject");http.server_.fields.push_back({"profileId",runtime::profileAt(1)->profileId});
    http.handleFiniteProfile();assert(http.server_.status==400);
    select(http,0,"reject");http.server_.fields.back().second=runtime::profileAt(1)->catalogHash;
    http.handleFiniteProfile();assert(http.server_.status==400);
    select(http,0,"reject");http.server_.fields[4].second="not-trusted";
    http.handleFiniteProfile();assert(http.server_.status==400);
    assert(runtime::coreRecordsUsed()==0 && runtime::driverRecordsUsed()==0 && tokenCalls==0 && nonzeroWrites==0);
  } else return 2;
  return 0;
}
'''
        (folder/'test.cpp').write_text(harness)
        sdk = []
        if sys.platform == 'darwin':
            sdk = ['-isysroot',subprocess.check_output(['xcrun','--sdk','macosx','--show-sdk-path'],text=True).strip()]
        command = [compiler,'-std=c++11','-Wall','-Wextra','-Werror',*sdk,'-include',str(string_header)]
        for name in ('CORE','DRIVER','SCHEDULER','RUNTIME'):
            command.append('-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_'+name+'=1')
        for path in ('','tests/native/service_stubs','tests/native/stubs','src'):
            command += ['-I',str(ROOT/path)]
        sources = ['src/control/'+name+'.cpp' for name in ('finite_profile_runtime','finite_profile_scheduler',
            'finite_profile_service_environment','finite_profile_driver_bridge','finite_profile_executor',
            'finite_profile_contract','finite_profile_json','bounded_action_service','bounded_action_executor',
            'command_bus','finite_profile_legacy_guard')]+['src/motor/motor_driver.cpp']
        cls.binary = folder/'test'
        result = subprocess.run([*command,str(folder/'test.cpp'),*[str(ROOT/p) for p in sources],'-o',str(cls.binary)],
                                text=True,capture_output=True,timeout=60)
        if result.returncode:
            raise AssertionError(result.stderr)

    def run_case(self,name):
        result=subprocess.run([str(self.binary),name],text=True,capture_output=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)
        return json.loads(result.stdout) if result.stdout else None

    def test_idle_get_keeps_legacy_profile_and_lists_exact_four(self):
        status=self.run_case('status')
        self.assertEqual(status['profile'],status['profiles'][0])
        self.assertEqual([p['direction'] for p in status['profiles']],[1,-1,1,-1])
        self.assertEqual([len(p['points']) for p in status['profiles']],[8,8,10,10])
        self.assertEqual([status[k] for k in ('coreRecordsUsed','driverRecordsUsed','coreRecordCapacity','driverRecordCapacity')],[0,0,32,32])

    def test_prepare_selects_all_four_and_closed_reservations_remain_counted(self):
        status=self.run_case('selection')
        self.assertEqual(status['coreRecordsUsed'],4)
        self.assertEqual(status['driverRecordsUsed'],4)

    def test_auth_duplicate_unknown_and_arbitrary_waveform_rejected_without_reservation(self):
        self.run_case('forbidden')


if __name__=='__main__':unittest.main()

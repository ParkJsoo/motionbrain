#include <cassert>
#include <memory>
#include "network/idle_connection_web_server.h"
uint32_t now = 0;
uint32_t millis() { return now; }
struct Server : IdleConnectionWebServer {
  Server() : IdleConnectionWebServer(80) {}
  void seed(const std::shared_ptr<ClientState>& c, uint32_t at, HTTPClientStatus state=HC_WAIT_READ) {
    _currentClient.value=c; _currentStatus=state; _statusChange=at;
  }
};
int main() {
  // A request recovered by a 239-281 ms TCP retransmission remains admissible.
  {
    Server recovery; auto client=std::make_shared<ClientState>();
    recovery.seed(client,0); now=300; recovery.handleClient();
    assert(client->stops==0);
    client->bytes=32; recovery.handleClient();
    assert(recovery.handled==1 && client->stops==0);
  }
  Server s; auto idle=std::make_shared<ClientState>();
  auto request=std::make_shared<ClientState>(); request->bytes=32;
  s.seed(idle,1000); s.pending.push_back(request);
  now=1499; s.handleClient(); assert(idle->stops==0 && s.handled==0);
  now=1500; s.handleClient(); assert(idle->stops==1 && s.idleConnectionReleases()==1);
  s.handleClient(); assert(s.handled==1 && request->stops==0);
  // An old connection with request bytes is still parsed normally.
  auto data=std::make_shared<ClientState>(); data->bytes=1;
  s.seed(data,0); now=9000; s.handleClient(); assert(data->stops==0 && s.handled==2);
  // Disconnected clients are cleared by the base without an idle release.
  auto gone=std::make_shared<ClientState>(); gone->connected=false;
  s.seed(gone,0); s.handleClient(); assert(gone->stops==0 && s.idleConnectionReleases()==1);
  auto closing=std::make_shared<ClientState>();
  s.seed(closing,0,HC_WAIT_CLOSE); s.handleClient(); assert(closing->stops==0);
  // Unsigned 32-bit elapsed time across millis wraparound.
  auto wrapped=std::make_shared<ClientState>(); s.seed(wrapped,0xfffffff0u);
  now=483; s.handleClient(); assert(wrapped->stops==0);
  now=484; s.handleClient(); assert(wrapped->stops==1 && s.idleConnectionReleases()==2);
  // Two queued empty connections each get their own first-byte window.
  auto a=std::make_shared<ClientState>(), b=std::make_shared<ClientState>();
  s.pending={a,b}; now=2000; s.handleClient(); now=2500; s.handleClient();
  now=2550; s.handleClient(); now=3049; s.handleClient(); assert(b->stops==0);
  now=3050; s.handleClient(); assert(a->stops==1 && b->stops==1);
}

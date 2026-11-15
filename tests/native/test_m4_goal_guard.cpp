#include "control/m4_goal_guard.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
using namespace m4_goal;
Request request(unsigned nonce=1,bool up=false){
 Request r;snprintf(r.nonce,sizeof(r.nonce),"%032x",nonce);memset(r.bootId,'b',32);memset(r.scopeSha256,'c',64);
 strcpy(r.policySha256,POLICY_SHA);r.up=up;r.percent=up?50:35;r.durationMs=1200;r.expectedStartDeg=up?197.22f:197.75f;r.goalDeg=up?197.85f:197.22f;return r;
}
Snapshot snapshot(uint32_t now=1000,float angle=197.75f){return Snapshot{now,now,angle,true,true,true,true,true,false};}
bool begin(Guard& g,Request r,Snapshot s=snapshot()){return g.queue(r,r.bootId,s.now)&&g.begin(r.nonce,r.bootId,r.scopeSha256,r.policySha256,s);}
int main(int argc,char** argv){
 assert(argc==2);int n=atoi(argv[1]);
 if(n==0){Guard g;Request r=request();assert(!g.queue(r,r.bootId,0));assert(g.count()==0);return 0;}
 if(n==1){Guard g;auto r=request();assert(begin(g,r));auto s=snapshot(1020,197.22f);s.moving=true;assert(g.tick(s));assert(!strcmp(g.current()->reason,"goal"));assert(g.current()->outputMayHaveOccurred);g.confirmOff(1024,true);assert(g.current()->offVerified);g.confirmOff(1026,true);assert(g.current()->offVerified);assert(g.current()->offAtMs==1024);assert(!g.tick(s));}
 if(n==2){for(int fault=0;fault<8;++fault){Guard g;assert(begin(g,request()));auto s=snapshot(1020);const char* reasons[]={"sensor_fault","sensor_fault","state_changed","safety_fault","other_output","global_limit","local_limit","local_limit"};switch(fault){case 0:s.ready=false;break;case 1:s.sampleAt=900;break;case 2:s.armed=false;break;case 3:s.safetyClear=false;break;case 4:s.outputAllowed=false;break;case 5:s.angle=198.4f;break;case 6:s.angle=196.7f;break;case 7:s.angle=197.9f;break;}assert(g.tick(s));assert(!strcmp(g.current()->reason,reasons[fault]));}}
 if(n==3){Guard g;assert(begin(g,request()));assert(!g.tick(snapshot(1020)));assert(g.tick(snapshot(1061)));assert(!strcmp(g.current()->reason,"tick_gap"));assert(g.current()->maxTickGapMs==41);Guard h;assert(begin(h,request()));assert(h.tick(snapshot(1201)));assert(!strcmp(h.current()->reason,"start_deadline"));}
 if(n==4){Guard g;assert(begin(g,request()));for(unsigned t=1020;t<2200;t+=20)assert(!g.tick(snapshot(t)));assert(g.tick(snapshot(2200,197.22f)));assert(!strcmp(g.current()->reason,"deadline"));assert(g.current()->decisionAtMs==2200);}
 if(n==5){Guard g;for(unsigned i=1;i<=CAPACITY;++i){auto r=request(i);assert(g.queue(r,r.bootId,i));assert(g.cancel("operator_stop",i));g.confirmOff(i,true);}assert(g.count()==CAPACITY);assert(!g.queue(request(33),request().bootId,33));assert(g.find(request(1).nonce));assert(!g.queue(request(1),request().bootId,34));}
 if(n==6){Guard g;auto a=request(1),b=request(2);assert(g.queue(a,a.bootId,0));assert(!g.queue(b,b.bootId,0));assert(!g.begin(a.nonce,"wrong",a.scopeSha256,a.policySha256,snapshot()));assert(g.cancel("clear",0));g.confirmOff(0,true);assert(g.queue(b,b.bootId,0));assert(g.cancel("disarm",0));g.confirmOff(0,true);assert(!g.queue(a,a.bootId,0));assert(!strcmp(g.find(a.nonce)->reason,"clear"));}
 if(n==7){for(int f=0;f<8;++f){Guard g;auto r=request();auto s=snapshot();switch(f){case 0:s.ready=false;break;case 1:s.sampleAt=900;break;case 2:s.armed=false;break;case 3:s.safetyClear=false;break;case 4:s.allOff=false;break;case 5:s.angle=197.6f;break;case 6:s.angle=NAN;break;case 7:s.outputAllowed=false;break;}assert(!begin(g,r,s));assert(!g.owns());assert(!g.current()->outputMayHaveOccurred);assert(!g.begin(r.nonce,r.bootId,r.scopeSha256,r.policySha256,snapshot()));}}
 if(n==8){auto r=request();assert(Guard::valid(r));r.durationMs=1201;assert(!Guard::valid(r));r=request();r.percent=75;assert(!Guard::valid(r));r=request();r.goalDeg=197.21f;assert(!Guard::valid(r));r=request();memset(r.nonce,'a',sizeof(r.nonce));assert(!Guard::valid(r));r=request();memset(r.policySha256,'a',sizeof(r.policySha256));assert(!Guard::valid(r));}
 if(n==9){Guard g;auto r=request(1,true);assert(begin(g,r,snapshot(1000,197.31f)));assert(g.tick(snapshot(1020,197.85f)));assert(!strcmp(g.current()->reason,"goal"));Guard h;assert(!begin(h,request(2,true),snapshot(1000,197.33f)));}
 if(n==10){Guard g;auto r=request();assert(begin(g,r,snapshot(0xfffffff0U)));assert(!g.tick(snapshot(4)));assert(g.tick(snapshot(24,197.22f)));assert(g.current()->firstTickDelayMs==20);assert(g.current()->maxTickGapMs==20);}
 if(n==11){Guard g;assert(begin(g,request()));assert(g.cancel("owner_override",1010));assert(!g.cancel("clear",1020));assert(!strcmp(g.current()->reason,"owner_override"));g.confirmOff(1011,false);assert(!g.current()->offVerified);}
 if(n==12){for(int mode=0;mode<3;++mode){Guard g;auto r=request();assert(begin(g,r));assert(g.cancel("fault",1010));if(mode==1)g.confirmOff(1011,false);if(mode==2)g.confirmOff(1016,true);if(mode)g.confirmOff(1011,true);assert(!g.queue(request(2),r.bootId,1020));assert(!g.cancel("clear",1020));assert(!g.begin(r.nonce,r.bootId,r.scopeSha256,r.policySha256,snapshot(1020)));assert(!strcmp(g.find(r.nonce)->reason,"fault"));}}
 if(n==13){
  auto r=request(1,true);strcpy(r.policySha256,CURRENT_POLICY_SHA);r.tableReferenceDeg=194.32f;r.expectedStartDeg=194.32f;r.goalDeg=194.95f;
  assert(Guard::valid(r));Guard g;assert(g.queue(r,r.bootId,100));
  assert(!g.begin(r.nonce,r.bootId,r.scopeSha256,POLICY_SHA,snapshot(100,r.expectedStartDeg)));
  assert(g.begin(r.nonce,r.bootId,r.scopeSha256,CURRENT_POLICY_SHA,snapshot(100,r.expectedStartDeg)));
  auto bad=r;bad.tableReferenceDeg=197.4f;assert(!Guard::valid(bad));
  bad=r;bad.goalDeg+=.01f;assert(!Guard::valid(bad));
  bad=r;strcpy(bad.policySha256,POLICY_SHA);assert(!Guard::valid(bad));
  bad=r;bad.tableReferenceDeg=NAN;assert(!Guard::valid(bad));
  bad=r;bad.up=false;bad.percent=35;bad.expectedStartDeg=195.23f;bad.goalDeg=194.32f;assert(!Guard::valid(bad));
  bad.expectedStartDeg=194.95f;assert(Guard::valid(bad));
 }
 if(n==14){
  auto r=request();strcpy(r.policySha256,CURRENT_POLICY_SHA);r.tableReferenceDeg=194.32f;r.expectedStartDeg=195.22f;r.goalDeg=194.32f;
  assert(Guard::valid(r));Guard g;assert(!begin(g,r,snapshot(1000,195.30f)));assert(!g.current()->outputMayHaveOccurred);
  assert(!strcmp(g.current()->reason,"start_context_changed"));
  Guard h;r.expectedStartDeg=194.44f;assert(!begin(h,r,snapshot(1000,194.39f)));assert(!h.current()->outputMayHaveOccurred);
 }
 return 0;
}

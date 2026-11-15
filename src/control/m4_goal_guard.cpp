#include "m4_goal_guard.h"
#include <string.h>
#include <math.h>
namespace m4_goal {
namespace {
#if MOTIONBRAIN_ENABLE_M4_GOAL_GUARD
bool window=false;
#endif
bool hex(const char* s,size_t n) { if (!s) return false; for(size_t i=0;i<n;++i) if(!((s[i]>='0'&&s[i]<='9')||(s[i]>='a'&&s[i]<='f')))return false; return s[n]=='\0'; }
bool same(const char* a,const char* b) { return a && b && strcmp(a,b)==0; }
}
#if MOTIONBRAIN_ENABLE_M4_GOAL_GUARD
bool activeWindow(){return window;}
void setActiveWindow(bool x){window=x;}
#endif
Request::Request():nonce{},bootId{},scopeSha256{},policySha256{},up(false),percent(0),durationMs(0),expectedStartDeg(0),goalDeg(0),tableReferenceDeg(NAN){}
Record::Record():request(),state("empty"),reason("none"),queuedAtMs(0),runAtMs(0),decisionAtMs(0),offAtMs(0),sampleAtMs(0),sampleAgeMs(0),maxTickGapMs(0),firstTickDelayMs(0),startAngleDeg(0),lastAngleDeg(0),offVerified(false),outputMayHaveOccurred(false),offConfirmationRecorded(false){}
Guard::Guard():records_{},count_(0),current_(-1),previousTick_(0),ticked_(false){}
bool Guard::valid(const Request& r){
 if(!(hex(r.nonce,32)&&hex(r.bootId,32)&&hex(r.scopeSha256,64)&&hex(r.policySha256,64)
  &&r.durationMs==DURATION_MS&&r.percent==(r.up?50:35)&&isfinite(r.expectedStartDeg)&&isfinite(r.goalDeg)
  &&r.expectedStartDeg>=192.29f&&r.expectedStartDeg<=198.29f))return false;
 if(same(r.policySha256,CURRENT_POLICY_SHA)){
   // Float32 arithmetic around 194 degrees has ~0.000015 degree resolution.
   const float table=r.tableReferenceDeg;
   return isfinite(table)&&table>=192.29f&&table<=197.39f
     &&fabsf(r.goalDeg-(r.up?table+.63f:table))<=.00005f
     &&(r.up?fabsf(r.expectedStartDeg-table)<=.00005f:
        r.expectedStartDeg>table+.1f&&r.expectedStartDeg<=table+.9f);
 }
 return same(r.policySha256,POLICY_SHA)&&isnan(r.tableReferenceDeg)
  &&fabsf(r.goalDeg-(r.up?197.85f:197.22f))<.00001f
  &&(r.up?fabsf(r.expectedStartDeg-197.22f)<.00001f:r.expectedStartDeg>197.32f);
}
const Record* Guard::current()const{return current_>=0?&records_[current_]:nullptr;}
const Record* Guard::find(const char* nonce)const{for(size_t i=0;i<count_;++i)if(same(records_[i].request.nonce,nonce))return &records_[i];return nullptr;}
bool Guard::active()const{return current()&&same(current()->state,"running");}
bool Guard::owns()const{return current()&&(same(current()->state,"queued")||active());}
bool Guard::queue(const Request& r,const char* boot,uint32_t now){
 if(!MOTIONBRAIN_ENABLE_M4_GOAL_GUARD||!valid(r)||!same(r.bootId,boot)||owns()||(current()&&!current()->offVerified)||count_==CAPACITY||find(r.nonce))return false;
 current_=static_cast<int>(count_++);records_[current_]=Record();records_[current_].request=r;
 records_[current_].state="queued";records_[current_].queuedAtMs=now;return true;
}
bool Guard::terminal(const char* reason,uint32_t now){if(!owns())return false;Record& r=records_[current_];r.state="terminal";r.reason=reason;r.decisionAtMs=now;return true;}
bool Guard::begin(const char* nonce,const char* boot,const char* scope,const char* policy,const Snapshot& s){
 if(!current()||!same(current()->state,"queued")||!same(current()->request.nonce,nonce)||!same(current()->request.bootId,boot)
  ||!same(current()->request.scopeSha256,scope)||!same(policy,current()->request.policySha256))return false;
 Record& r=records_[current_];r.startAngleDeg=s.angle;r.lastAngleDeg=s.angle;r.sampleAtMs=s.sampleAt;r.sampleAgeMs=s.now-s.sampleAt;
 const char* bad=nullptr;
 if(!s.ready||r.sampleAgeMs>SENSOR_AGE_MS||!isfinite(s.angle))bad="sensor_fault";
 else if(!s.armed)bad="state_changed";
 else if(!s.safetyClear)bad="safety_fault";
 else if(!s.allOff||!s.outputAllowed)bad="other_output";
 else if(fabsf(s.angle-r.request.expectedStartDeg)>.1f)bad="start_context_changed";
 else if(same(r.request.policySha256,CURRENT_POLICY_SHA)&&!r.request.up
         &&(s.angle<=r.request.tableReferenceDeg+.1f||s.angle>r.request.tableReferenceDeg+.9f))bad="start_context_changed";
 else if(s.angle<192.29f||s.angle>198.29f||(r.request.up&&s.angle+.9f>198.29f))bad="global_limit";
 else if(r.request.up?s.angle>=r.request.goalDeg:s.angle<=r.request.goalDeg)bad="already_at_goal";
 if(bad){terminal(bad,s.now);return false;}
 r.runAtMs=s.now;r.state="running";previousTick_=s.now;ticked_=false;return true;
}
bool Guard::tick(const Snapshot& s){
 if(!active())return false;
 Record& r=records_[current_];
 uint32_t gap=s.now-previousTick_;if(!ticked_)r.firstTickDelayMs=gap;else if(gap>r.maxTickGapMs)r.maxTickGapMs=gap;
 previousTick_=s.now;r.lastAngleDeg=s.angle;r.sampleAtMs=s.sampleAt;r.sampleAgeMs=s.now-s.sampleAt;
 r.outputMayHaveOccurred=r.outputMayHaveOccurred||s.moving;
 const char* reason=nullptr;
 if(!s.ready||!isfinite(s.angle)||r.sampleAgeMs>SENSOR_AGE_MS)reason="sensor_fault";
 else if(!s.armed)reason="state_changed";
 else if(!s.safetyClear)reason="safety_fault";
 else if(!s.outputAllowed)reason="other_output";
 else if(!ticked_&&gap>200)reason="start_deadline";
 else if(ticked_&&gap>TICK_GAP_MS)reason="tick_gap";
 else if(s.angle<192.29f||s.angle>198.29f)reason="global_limit";
 else if(fabsf(s.angle-r.startAngleDeg)>.9f||(r.request.up?s.angle<r.startAngleDeg-.1f:s.angle>r.startAngleDeg+.1f))reason="local_limit";
 else if(s.now-r.runAtMs>=r.request.durationMs)reason="deadline";
 else if(r.request.up?s.angle>=r.request.goalDeg:s.angle<=r.request.goalDeg)reason="goal";
 ticked_=true;return reason?terminal(reason,s.now):false;
}
bool Guard::cancel(const char* reason,uint32_t now){return terminal(reason,now);}
void Guard::confirmOff(uint32_t at,bool verified){if(current()&&same(current()->state,"terminal")&&!current()->offConfirmationRecorded){records_[current_].offConfirmationRecorded=true;records_[current_].offAtMs=at;records_[current_].offVerified=verified && at-records_[current_].decisionAtMs<=5;}}
}

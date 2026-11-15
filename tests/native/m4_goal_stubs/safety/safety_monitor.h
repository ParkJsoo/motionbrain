#pragma once
struct SafetyMonitor {bool blocked=false;bool isMotionBlocked()const{return blocked;}bool hasLatchedFault()const{return blocked;}const char* getBlockReasonString()const{return "test";}};

#pragma once
#include "../stubs/Arduino.h"
#include <string>
#include <cstdio>
class String {
 public: std::string value;
 String(){}
 template<class T> String(T v):value(std::to_string(v)){}
 String(float v,unsigned digits){char b[80];snprintf(b,sizeof(b),"%.*f",int(digits),v);value=b;}
 String& operator+=(const char* s){value+=s;return *this;}
 String& operator+=(const String& s){value+=s.value;return *this;}
 template<class T> String& operator+=(T v){value+=std::to_string(v);return *this;}
};
struct SerialFake {std::string input;int available(){return input.size();}int read(){char c=input[0];input.erase(0,1);return c;}};
extern SerialFake Serial;

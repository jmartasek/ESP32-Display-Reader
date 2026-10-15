#include "../src/registration_core.h"
#include <cassert>
#include <iostream>

int main() {
  using namespace registration;
  assert(vote3(-1,-1,1) == -1);
  assert(vote3(0,-1,1) == -1);
  assert(vote3(-1,0,0) == 0);
  assert(vote3(1,-1,1) == 1);
  assert(vote3(-1,-1,-1) == -1);
  static const int patterns[] = {63,6,91,79,102,109,125,7,127,111};
  for (int i = 0; i < 10; ++i) assert(decode(patterns[i], false) == i);
  assert(decode(39,false) == -1);
  assert(decode(39,true) == 7);
  assert(decode(0,false) == -1);
  assert(decode(255,false) == -1);
  std::vector<uint8_t> pixels(120*80,30);
  std::vector<Digit> digits(2);
  for (int d = 0; d < 2; ++d) {
    auto &digit = digits[d];
    const int x = 40 + 16*d, y = 30;
    digit.bounds = {x,y,13,20};
    digit.samples[0] = {x+3,y+1,7,2};
    digit.samples[1] = {x+10,y+3,2,6};
    digit.samples[2] = {x+10,y+11,2,6};
    digit.samples[3] = {x+3,y+17,7,2};
    digit.samples[4] = {x+1,y+11,2,6};
    digit.samples[5] = {x+1,y+3,2,6};
    digit.samples[6] = {x+3,y+9,7,2};
  }
  Frame frame{pixels.data(),120,80};
  assert(!search(frame,digits,24,12).found);
  assert(!search(frame,digits,24,12).active);
  int tests=0;
  for (int v = 0; v < 10; ++v) {
    for (int dx : {-15,0,15}) {
      std::fill(pixels.begin(),pixels.end(),30);
      for (const auto &d : digits) for (int i=0;i<7;++i) {
        if (!(patterns[v] & (1<<i))) continue;
        Box b = shifted(d.samples[i],dx,5);
        for (int y=b.y;y<b.y+b.h;++y)
          for (int x=b.x;x<b.x+b.w;++x) pixels[y*120+x]=230;
      }
      auto r=search(frame,digits,24,12);
      assert(r.found && r.active);
      assert(r.values.size()==2 && r.values[0]==v && r.values[1]==v);
      ++tests;
    }
  }
  auto malformed=digits;
  malformed[0].samples[0]={-200,0,1,1};
  assert(!search(frame,malformed,24,12).found);
  assert(!search({nullptr,120,80},digits,24,12).found);
  assert(!search(frame,{},24,12).found);
  assert(!search(frame,std::vector<Digit>(13),24,12).found);
  std::cout << tests << " synthetic digit/movement cases plus blank/bounds checks passed\n";
}

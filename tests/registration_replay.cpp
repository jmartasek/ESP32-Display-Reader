#include "../src/registration_core.h"
#include <fstream>
#include <iostream>
#include <string>

// Binary PGM on stdin; calibration in a small text file. No alternative decoder.
int main(int argc, char **argv) {
  if (argc != 4) return 2;
  std::ifstream spec(argv[1]);
  int n; spec >> n;
  std::vector<registration::Digit> digits(n);
  for (auto &d : digits) {
    spec >> d.bounds.x >> d.bounds.y >> d.bounds.w >> d.bounds.h >> d.italic >> d.allow_blank;
    for (auto &s : d.samples) spec >> s.x >> s.y >> s.w >> s.h;
  }
  std::string magic; int w, h, maxv;
  std::cin >> magic >> w >> h >> maxv; std::cin.get();
  if (!spec || magic != "P5" || w <= 0 || h <= 0 || maxv != 255) return 2;
  std::vector<uint8_t> pixels(w*h);
  std::cin.read(reinterpret_cast<char *>(pixels.data()), pixels.size());
  if (!std::cin) return 2;
  auto r = registration::search({pixels.data(),w,h}, digits, std::stoi(argv[2]), std::stoi(argv[3]));
  std::cout << "{\"found\":" << r.found << ",\"active\":" << r.active
            << ",\"dx\":" << r.dx << ",\"dy\":" << r.dy
            << ",\"score\":" << r.score << ",\"separation\":" << r.separation
            << ",\"threshold\":" << r.threshold << ",\"digits\":[";
  for (size_t i=0;i<r.values.size();++i) std::cout << (i ? "," : "") << r.values[i];
  std::cout << "]}\n";
}

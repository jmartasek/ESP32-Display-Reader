#pragma once

// No Arduino dependencies: the native replay and firmware execute this code.
#include <algorithm>
#include <stdint.h>
#include <vector>

namespace registration {
// -1 is unknown; it must never contribute an OFF vote.
inline int vote3(int a, int b, int c) {
  const int on = int(a == 1) + int(b == 1) + int(c == 1);
  const int off = int(a == 0) + int(b == 0) + int(c == 0);
  return on >= 2 ? 1 : off >= 2 ? 0 : -1;
}
struct Box { int x, y, w, h; };
struct Digit {
  Box bounds;
  Box samples[7];
  bool italic = false;
  bool allow_blank = false;
};
struct Frame {
  const uint8_t *pixels;
  int width, height;
  bool inside(Box b) const {
    return b.w > 0 && b.h > 0 && b.x >= 0 && b.y >= 0 &&
           b.x + b.w <= width && b.y + b.h <= height;
  }
  int mean(Box b) const {
    int sum = 0;
    for (int y = b.y; y < b.y + b.h; ++y)
      for (int x = b.x; x < b.x + b.w; ++x) sum += pixels[y * width + x];
    return sum / (b.w * b.h);
  }
};
struct Result {
  bool found = false;
  bool active = false;
  bool activity_known = true;
  int dx = 0, dy = 0, threshold = 0;
  float score = 0, separation = 0;
  std::vector<int> values;
  std::vector<uint8_t> bits;
  std::vector<int> thresholds;
};
inline Box shifted(Box b, int dx, int dy) { b.x += dx; b.y += dy; return b; }
inline int decode(int bits, bool italic) {
  static const int patterns[] = {63,6,91,79,102,109,125,7,127,111};
  for (int i = 0; i < 10; ++i) if (bits == patterns[i]) return i;
  // Preserve the calibrated washer profile's sole existing correction.
  return italic && bits == 39 ? 7 : -1;
}
inline bool sameReading(const Result &a, const Result &b) { return a.values == b.values; }

inline Result search(Frame frame, const std::vector<Digit> &digits,
                     int radius_x, int radius_y, int minimum_contrast = 45,
                     int minimum_separation = 5) {
  Result best;
  if (!frame.pixels || digits.empty() || digits.size() > 12) return best;
  int total_area = 0;
  for (const auto &d : digits) {
    if (d.bounds.w <= 0 || d.bounds.h <= 0 || d.bounds.w > 128 || d.bounds.h > 128)
      return best;
    total_area += d.bounds.w * d.bounds.h;
  }
  if (total_area > 8192) return best;
  radius_x = std::max(0, std::min(radius_x, 32));
  radius_y = std::max(0, std::min(radius_y, 20));
  Result runner;
  bool active_evidence = false;
  bool possible_evidence = false;
  for (int dy = -radius_y; dy <= radius_y; ++dy) {
    for (int dx = -radius_x; dx <= radius_x; ++dx) {
      bool inside = true;
      int hist[256] = {}, count = 0;
      for (const auto &d : digits) {
        Box b = shifted(d.bounds, dx, dy);
        if (!frame.inside(b)) { inside = false; break; }
        for (int y = b.y; y < b.y + b.h; ++y)
          for (int x = b.x; x < b.x + b.w; ++x) {
            ++hist[frame.pixels[y * frame.width + x]]; ++count;
          }
      }
      if (!inside) continue;
      int cumulative = 0, background = 0, foreground = 255;
      bool background_set = false;
      for (int i = 0; i < 256; ++i) {
        cumulative += hist[i];
        if (!background_set && cumulative * 100 >= count * 35) {
          background = i; background_set = true;
        }
        if (cumulative * 100 >= count * 95) { foreground = i; break; }
      }
      if (foreground - background < minimum_contrast) continue;
      const int threshold = background + (foreground - background) * 3 / 5;
      Result candidate;
      candidate.dx = dx; candidate.dy = dy; candidate.threshold = threshold;
      bool valid = true, any_lit = false;
      int credible_digits = 0;
      float score = 0, weights = 0;
      for (const auto &d : digits) {
        const Box local = shifted(d.bounds, dx, dy);
        int local_hist[256] = {}, local_total = 0;
        for (int y = local.y; y < local.y + local.h; ++y)
          for (int x = local.x; x < local.x + local.w; ++x)
            ++local_hist[frame.pixels[y * frame.width + x]];
        int local_background = 0;
        for (int i = 0; i < 256; ++i) {
          local_total += local_hist[i];
          if (local_total * 100 >= local.w * local.h * 30) { local_background = i; break; }
        }
        const int threshold = local_background + (foreground - local_background) * 3 / 5;
        int avgs[7], bits = 0;
        bool sample_geometry = true;
        for (int i = 0; i < 7; ++i) {
          Box b = shifted(d.samples[i], dx, dy);
          if (!frame.inside(b)) { sample_geometry = false; break; }
          avgs[i] = frame.mean(b);
          if (avgs[i] > threshold) bits |= 1 << i;
        }
        if (!sample_geometry) { valid = false; break; }
        if (bits) any_lit = true;
        int value = decode(bits, d.italic);
        // An ON decision must have headroom; a reflected edge just above the
        // threshold must not turn 9 into 8 while searching neighbouring pixels.
        for (int i = 0; i < 7; ++i)
          if ((bits & (1 << i)) && (avgs[i] - threshold) * 100 < (foreground - local_background) * 8)
            value = -1;
        if (bits == 0 && d.allow_blank) value = -2;
        if (value == -1) valid = false;
        int margin = 255;
        float sum = 0;
        for (int i = 0; i < 7; ++i) {
          const int distance = bits & (1 << i) ? avgs[i] - threshold : threshold - avgs[i];
          margin = std::min(margin, distance); sum += distance;
        }
        // A weak single stroke must lower confidence even if six samples are strong.
        float unexplained = 0;
        Box bounds = shifted(d.bounds, dx, dy);
        for (int y = bounds.y; y < bounds.y + bounds.h; ++y) {
          for (int x = bounds.x; x < bounds.x + bounds.w; ++x) {
            int pixel = frame.pixels[y * frame.width + x];
            if (pixel <= threshold) continue;
            int nearest = 100;
            for (int i = 0; i < 7; ++i) {
              if (!(bits & (1 << i))) continue;
              Box b = shifted(d.samples[i], dx, dy);
              int sx = std::max(0, std::max(b.x - x, x - (b.x + b.w - 1)));
              int sy = std::max(0, std::max(b.y - y, y - (b.y + b.h - 1)));
              nearest = std::min(nearest, std::max(sx, sy));
            }
            if (nearest > 1) unexplained += (pixel - threshold) * std::min(nearest - 1, 3);
          }
        }
        const float weight = bits ? 1.0f : 0.15f;
        const float digit_score = (0.65f * margin + 0.35f * sum / 7 -
                  2 * unexplained / (bounds.w * bounds.h)) / (foreground - background);
        if (bits && digit_score >= 0.20f) possible_evidence = true;
        if (value >= 0 && digit_score >= 0.20f) ++credible_digits;
        score += weight * digit_score;
        weights += weight;
        candidate.values.push_back(value); candidate.bits.push_back(bits);
        candidate.thresholds.push_back(threshold);
      }
      if (credible_digits >= 2) active_evidence = true;
      // This is evidence in the full search area, independent of successful decoding.
      // Weak reflected strokes cannot by themselves establish activity.
      if (!valid || !any_lit) continue;
      candidate.active = true; candidate.found = true;
      candidate.score = 100 * score / weights;
      if (!best.found || candidate.score > best.score) {
        if (!sameReading(best, candidate)) runner = best;
        best = candidate;
      } else if (!sameReading(best, candidate) && candidate.score > runner.score) {
        runner = candidate;
      }
    }
  }
  if (!best.found) {
    best.active = active_evidence;
    best.activity_known = active_evidence || !possible_evidence;
    return best;
  }
  best.separation = best.score - runner.score;
  best.active = active_evidence || best.score >= 8;
  if (best.score < 8 || best.separation < minimum_separation) best.found = false;
  return best;
}
} // namespace registration

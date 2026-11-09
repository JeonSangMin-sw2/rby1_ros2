// LakiBeam1 packets and the turns they add up to, from packets built here; no ROS graph, no sensor.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "rby1_tools/lidar_model.hpp"

using namespace rby1_tools;

namespace {

void put(std::vector<uint8_t> & p, size_t at, uint16_t value) {
  p[at] = static_cast<uint8_t>(value & 0xFF);
  p[at + 1] = static_cast<uint8_t>(value >> 8);
}

// A data packet whose block b starts at azimuth `first` + b * 16 * `resolution` (0.01 deg); each
// return's distance is its azimuth / 10 mm (0 for azimuths in `blank`), intensity 7.
std::vector<uint8_t> packet(uint16_t first, uint16_t resolution, int blank_from = -1, int blank_to = -1) {
  std::vector<uint8_t> p(kLakibeamPacketSize, 0);
  for (size_t b = 0; b < 12; ++b) {
    const size_t at = b * 100;
    const uint16_t azimuth = static_cast<uint16_t>((first + b * 16 * resolution) % 36000);
    put(p, at, 0xEEFF);
    put(p, at + 2, azimuth);
    for (size_t i = 0; i < 16; ++i) {
      const int a = (azimuth + resolution * i) % 36000;
      const bool blank = a >= blank_from && a < blank_to;
      put(p, at + 4 + i * 6, blank ? 0 : static_cast<uint16_t>(a / 10));
      p[at + 4 + i * 6 + 2] = 7;
    }
  }
  return p;
}

}  // namespace

TEST(Lakibeam, PacketGivesItsReturns) {
  uint16_t resolution = 0;
  const auto p = packet(9000, 25);
  const auto returns = parse_lakibeam(p.data(), p.size(), resolution);
  EXPECT_EQ(resolution, 25);  // 0.25 deg, from two blocks 16 returns apart
  ASSERT_EQ(returns.size(), 12u * 16u);
  EXPECT_EQ(returns[0].azimuth, 9000);
  EXPECT_EQ(returns[1].azimuth, 9025);
  EXPECT_EQ(returns[16].azimuth, 9400);  // the second block
  EXPECT_EQ(returns[0].distance, 900);
  EXPECT_EQ(returns[0].intensity, 7);
}

TEST(Lakibeam, AnythingElseIsNotAPacket) {
  uint16_t resolution = 0;
  std::vector<uint8_t> short_one(100, 0);
  EXPECT_TRUE(parse_lakibeam(short_one.data(), short_one.size(), resolution).empty());
  std::vector<uint8_t> no_flag(kLakibeamPacketSize, 0);
  EXPECT_TRUE(parse_lakibeam(no_flag.data(), no_flag.size(), resolution).empty());
  EXPECT_EQ(resolution, 0);
}

TEST(Lakibeam, AzimuthWrapsAtAFullTurn) {
  uint16_t resolution = 0;
  const auto p = packet(35600, 25);  // its second block goes past 360 deg
  const auto returns = parse_lakibeam(p.data(), p.size(), resolution);
  EXPECT_EQ(resolution, 25);  // read across 0 too
  ASSERT_EQ(returns.size(), 12u * 16u);
  EXPECT_EQ(returns[15].azimuth, 35975);
  EXPECT_EQ(returns[16].azimuth, 0);
}

TEST(Assembler, AWholeTurnIsPublishedOnceAndThePartFirstOneIsNot) {
  ScanAssembler assembler;
  uint16_t resolution = 0;
  int finished = 0;
  const uint16_t per_packet = 12 * 16 * 25;  // 48 deg
  // Start mid-turn at 180 deg, then run two and a half turns; azimuths 45..60 deg have no return.
  for (int start = 18000; start < 18000 + 36000 * 2 + 18000; start += per_packet) {
    const auto p = packet(static_cast<uint16_t>(start % 36000), 25, 4500, 6000);
    if (assembler.add(parse_lakibeam(p.data(), p.size(), resolution), resolution)) ++finished;
  }
  EXPECT_EQ(finished, 2);  // the turns from 0 deg; the first, part one is dropped
  const auto & ranges = assembler.ranges();
  ASSERT_EQ(ranges.size(), 1440u);  // 360 / 0.25
  EXPECT_EQ(assembler.resolution(), 25);
  EXPECT_TRUE(std::isinf(ranges[0]));  // azimuth 0: distance 0 mm, no return
  EXPECT_FLOAT_EQ(ranges[400], 1.0f);  // azimuth 100 deg: 1000 mm
  EXPECT_TRUE(std::isinf(ranges[200]));  // 50 deg, in the blank part
  EXPECT_FLOAT_EQ(assembler.intensities()[400], 7.0f);
}

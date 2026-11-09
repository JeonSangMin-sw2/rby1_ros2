// A Richbeam LakiBeam1 2D LiDAR's UDP data packets, and the full turns they add up to; no ROS.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rby1_tools {

// One return: where (azimuth, 0.01 deg, 0..35999) and what (distance in mm, 0 for none;
// intensity).
struct LidarReturn {
  uint16_t azimuth = 0;
  uint16_t distance = 0;
  uint8_t intensity = 0;
};

// A data packet is 1206 bytes: 12 blocks of {0xFFEE flag, azimuth (0.01 deg), 16 x {distance 1
// (mm), intensity 1, distance 2, intensity 2}}, then a timestamp and a factory word, little
// endian. Each block's 16 returns are `resolution` apart from its azimuth.
inline constexpr size_t kLakibeamPacketSize = 1206;

// The returns of one packet, in order, and its resolution (0.01 deg) in `resolution` when the
// packet shows it (two blocks apart by 16 returns); empty for anything but a data packet.
std::vector<LidarReturn> parse_lakibeam(const uint8_t * data, size_t size, uint16_t & resolution);

// Collects returns into full turns, one bin per `resolution` from azimuth 0. A turn ends when the
// azimuth goes back to the start; the turn the sensor was in when the first returns came is
// dropped (it is not whole).
class ScanAssembler {
public:
  // True when `returns` finished a turn; its bins are then in ranges() and intensities().
  bool add(const std::vector<LidarReturn> & returns, uint16_t resolution);
  const std::vector<float> & ranges() const { return ranges_; }            // m, +inf where no return
  const std::vector<float> & intensities() const { return intensities_; }
  uint16_t resolution() const { return resolution_; }                      // of the finished turn
  bool started() const { return started_; }                                // a whole turn is being built

private:
  void reset(uint16_t resolution);

  bool started_ = false;
  int last_azimuth_ = -1;
  uint16_t resolution_ = 0;
  std::vector<float> building_ranges_, building_intensities_;
  std::vector<float> ranges_, intensities_;
};

}  // namespace rby1_tools

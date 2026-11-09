#include "rby1_tools/lidar_model.hpp"

#include <limits>

namespace rby1_tools {

namespace {

constexpr size_t kBlocks = 12, kReturnsPerBlock = 16, kBlockSize = 4 + kReturnsPerBlock * 6;
constexpr uint16_t kDataFlag = 0xEEFF;  // the bytes FF EE, read little endian
constexpr int kFullTurn = 36000;        // 0.01 deg

uint16_t word(const uint8_t * p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

}  // namespace

std::vector<LidarReturn> parse_lakibeam(const uint8_t * data, size_t size, uint16_t & resolution) {
  std::vector<LidarReturn> returns;
  if (size != kLakibeamPacketSize) return returns;
  const int step = (word(data + kBlockSize + 2) - word(data + 2) + kFullTurn) % kFullTurn;  // across 0 too
  if (word(data) == kDataFlag && word(data + kBlockSize) == kDataFlag && step > 0) {
    resolution = static_cast<uint16_t>(step / kReturnsPerBlock);
  }
  if (resolution == 0) return returns;
  returns.reserve(kBlocks * kReturnsPerBlock);
  for (size_t b = 0; b < kBlocks; ++b) {
    const uint8_t * block = data + b * kBlockSize;
    if (word(block) != kDataFlag) continue;
    const uint16_t azimuth = word(block + 2);
    for (size_t i = 0; i < kReturnsPerBlock; ++i) {
      const uint8_t * result = block + 4 + i * 6;
      LidarReturn r;
      r.azimuth = static_cast<uint16_t>((azimuth + resolution * i) % kFullTurn);
      r.distance = word(result);
      r.intensity = result[2];
      returns.push_back(r);
    }
  }
  return returns;
}

void ScanAssembler::reset(uint16_t resolution) {
  resolution_ = resolution;
  const size_t bins = static_cast<size_t>(kFullTurn / resolution);
  building_ranges_.assign(bins, std::numeric_limits<float>::infinity());
  building_intensities_.assign(bins, 0.0f);
}

bool ScanAssembler::add(const std::vector<LidarReturn> & returns, uint16_t resolution) {
  if (resolution == 0) return false;
  if (resolution != resolution_) {  // a new rate: start over
    reset(resolution);
    started_ = false;
    last_azimuth_ = -1;
  }
  bool finished = false;
  for (const auto & r : returns) {
    if (last_azimuth_ >= 0 && r.azimuth < last_azimuth_) {  // back past azimuth 0: a turn ended
      if (started_) {
        ranges_.swap(building_ranges_);
        intensities_.swap(building_intensities_);
        finished = true;
      }
      started_ = true;
      reset(resolution);
    }
    last_azimuth_ = r.azimuth;
    const size_t bin = r.azimuth / resolution;
    if (started_ && bin < building_ranges_.size() && r.distance > 0) {
      building_ranges_[bin] = static_cast<float>(r.distance) / 1000.0f;
      building_intensities_[bin] = r.intensity;
    }
  }
  return finished;
}

}  // namespace rby1_tools

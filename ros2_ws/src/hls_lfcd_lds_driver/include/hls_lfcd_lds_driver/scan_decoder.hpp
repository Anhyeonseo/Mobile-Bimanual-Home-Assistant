// Stream framing extension to ROBOTIS hls_lfcd_lds_driver 2.1.1.
// Distributed under the BSD license in this package's LICENSE.
#ifndef HLS_LFCD_LDS_DRIVER__SCAN_DECODER_HPP_
#define HLS_LFCD_LDS_DRIVER__SCAN_DECODER_HPP_
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace hls_lfcd_lds
{
struct DecodedScan
{
  std::array<float, 360> ranges;
  std::array<float, 360> intensities;
  unsigned sectors = 0;
  double rpm = 0.;
  double first_received = 0.;
};

class ScanDecoder
{
public:
  uint64_t checksum_errors = 0, discarded_bytes = 0, partial_scans = 0;
  uint64_t complete_scans = 0, rejected_scans = 0, duplicate_sectors = 0;

  void reset()
  {
    bytes_.clear();
    active_ = false;
  }

  std::optional<DecodedScan> feed(uint8_t byte, double received)
  {
    bytes_.push_back(byte);
    while (!bytes_.empty()) {
      if (bytes_[0] != 0xfa || (bytes_.size() > 1 &&
        (bytes_[1] < 0xa0 || bytes_[1] > 0xdb)))
      {
        bytes_.erase(bytes_.begin());
        ++discarded_bytes;
        continue;
      }
      if (bytes_.size() < 42) {return std::nullopt;}
      uint8_t checksum = 0xff;
      for (unsigned i = 0; i < 40; ++i) {checksum -= bytes_[i];}
      if (checksum != bytes_[40] && checksum != bytes_[41]) {
        ++checksum_errors;
        bytes_.erase(bytes_.begin());
        ++discarded_bytes;
        continue;
      }
      std::array<uint8_t, 42> packet{};
      std::copy_n(bytes_.begin(), 42, packet.begin());
      bytes_.clear();
      return sector(packet, received);
    }
    return std::nullopt;
  }

private:
  std::vector<uint8_t> bytes_;
  DecodedScan scan_{};
  bool active_ = false;
  unsigned last_ = 0;

  void begin(double received)
  {
    scan_ = DecodedScan{};
    scan_.ranges.fill(std::numeric_limits<float>::quiet_NaN());
    scan_.intensities.fill(0.f);
    scan_.first_received = received;
    active_ = true;
    last_ = 0;
  }

  std::optional<DecodedScan> finish(double received)
  {
    active_ = false;
    // At most two absent sectors. Larger corruption still interrupts scan health.
    // No timer can republish an old scan or manufacture clear-space rays.
    if (scan_.sectors < 58 || received - scan_.first_received > .30 ||
      received < scan_.first_received)
    {
      ++rejected_scans;
      return std::nullopt;
    }
    scan_.rpm /= scan_.sectors;
    if (scan_.rpm < 250. || scan_.rpm > 350.) {
      ++rejected_scans;
      return std::nullopt;
    }
    if (scan_.sectors == 60) {++complete_scans;} else {++partial_scans;}
    return scan_;
  }

  std::optional<DecodedScan> sector(const std::array<uint8_t, 42> & p, double received)
  {
    const unsigned index = p[1] - 0xa0;
    std::optional<DecodedScan> previous;
    if (index == 0) {
      if (active_) {previous = finish(received);}
      begin(received);
    } else if (!active_) {
      return std::nullopt;  // Await a measured rotation boundary.
    } else if (index == last_) {
      ++duplicate_sectors;
      return std::nullopt;
    } else if (index < last_ || received - scan_.first_received > .30 ||
      received < scan_.first_received)
    {
      ++rejected_scans;
      active_ = false;
      return std::nullopt;
    }
    last_ = index;
    scan_.rpm += (p[2] | (p[3] << 8)) / 10.;
    ++scan_.sectors;
    for (unsigned k = 0; k < 6; ++k) {
      const unsigned source = 4 + 6 * k, target = 359 - (6 * index + k);
      const float range = (p[source + 2] | (p[source + 3] << 8)) / 1000.f;
      if (range >= .12f && range <= 3.5f) {scan_.ranges[target] = range;}
      scan_.intensities[target] = p[source] | (p[source + 1] << 8);
    }
    if (index == 59) {return finish(received);}
    return previous;
  }
};
}  // namespace hls_lfcd_lds
#endif

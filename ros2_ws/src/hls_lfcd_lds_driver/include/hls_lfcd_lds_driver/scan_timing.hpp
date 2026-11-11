// Host-serial timing estimate; not device-clock synchronization or deskew.
#ifndef HLS_LFCD_LDS_DRIVER__SCAN_TIMING_HPP_
#define HLS_LFCD_LDS_DRIVER__SCAN_TIMING_HPP_
#include <cmath>
#include <optional>
#include "hls_lfcd_lds_driver/scan_decoder.hpp"
namespace hls_lfcd_lds
{
struct ScanTiming
{
  double index_zero_steady_s;
  double time_increment_s;
  double scan_time_s;
};
inline std::optional<ScanTiming> scan_timing(const DecodedScan & scan, double now)
{
  if (!std::isfinite(now) || !std::isfinite(scan.last_received) ||
    !std::isfinite(scan.rpm) || scan.rpm < 250. || scan.rpm > 350. ||
    scan.last_sector > 59 || scan.sectors < 58 || scan.last_received > now)
  {return std::nullopt;}
  const double step = 1. / (scan.rpm * 6.);
  // A complete 42-byte 8N1 packet takes at least this long on the wire.
  // Packet 59 contains ray 0. For a missing tail, extrapolate from the last
  // received sector, never from the next rotation's arrival or publication.
  const double index_zero = scan.last_received + (59 - scan.last_sector) * 6 * step - 420. / 230400.;
  if (index_zero > now || now - index_zero > .30) {return std::nullopt;}
  // Ranges are in increasing angle order, opposite to physical acquisition.
  return ScanTiming{index_zero, -step, 60. / scan.rpm};
}
}  // namespace hls_lfcd_lds
#endif

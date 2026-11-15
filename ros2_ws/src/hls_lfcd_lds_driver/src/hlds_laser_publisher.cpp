/*******************************************************************************
* Copyright (c) 2016, Hitachi-LG Data Storage
* Copyright (c) 2017, ROBOTIS
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Authors: Pyo, Darby Lim, SP Kong, JH Yang */
/* maintainer: Pyo */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/asio.hpp>
#include <hls_lfcd_lds_driver/lfcd_laser.hpp>
#include <hls_lfcd_lds_driver/scan_timing.hpp>
#include <chrono>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace hls_lfcd_lds
{
LFCDLaser::LFCDLaser(const std::string & port, uint32_t baud_rate, boost::asio::io_service & io)
: port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
{
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  serial_.set_option(boost::asio::serial_port_base::character_size(8));
  serial_.set_option(boost::asio::serial_port_base::parity(
      boost::asio::serial_port_base::parity::none));
  serial_.set_option(boost::asio::serial_port_base::stop_bits(
      boost::asio::serial_port_base::stop_bits::one));
  serial_.set_option(boost::asio::serial_port_base::flow_control(
      boost::asio::serial_port_base::flow_control::none));
  const int fd = serial_.native_handle();
  if (::ioctl(fd, TIOCEXCL) < 0 || ::fcntl(fd, F_SETFL, O_NONBLOCK) < 0) {
    throw boost::system::system_error(errno, boost::system::generic_category());
  }
  if (::write(fd, "b", 1) != 1) {
    throw boost::system::system_error(errno ? errno : EIO, boost::system::generic_category());
  }
}
LFCDLaser::~LFCDLaser()
{
  // One-byte nonblocking sensor pause. Never tcdrain on CP210x shutdown.
  if (serial_.is_open() && ::write(serial_.native_handle(), "e", 1) != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("hlds_laser_publisher"), "sensor pause write unconfirmed");
  }
  RCLCPP_INFO(rclcpp::get_logger("hlds_laser_publisher"),
    "scan_stats complete=%lu partial=%lu rejected=%lu checksum_errors=%lu backlog_resets=%lu",
    decoder_.complete_scans, decoder_.partial_scans, decoder_.rejected_scans,
    decoder_.checksum_errors, backlog_resets_);
}

bool LFCDLaser::poll(sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  const int fd = serial_.native_handle();
  struct pollfd descriptor{fd, POLLIN, 0};
  const int ready = ::poll(&descriptor, 1, 100);
  if (ready < 0) {
    if (errno == EINTR) {return false;}
    throw boost::system::system_error(errno, boost::system::generic_category());
  }
  if (ready == 0 || shutting_down_ || !rclcpp::ok()) {return false;}
  if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    throw boost::system::system_error(EIO, boost::system::generic_category());
  }
  int queued = 0;
  if (::ioctl(fd, FIONREAD, &queued) < 0) {
    throw boost::system::system_error(errno, boost::system::generic_category());
  }
  if (queued > 2520) {
    // Do not stamp a queued rotation as a fresh observation after a host stall.
    if (::tcflush(fd, TCIFLUSH) < 0) {
      throw boost::system::system_error(errno, boost::system::generic_category());
    }
    decoder_.reset();
    ++backlog_resets_;
    return false;
  }
  std::array<uint8_t, 42> bytes{};
  const auto count = ::read(fd, bytes.data(), bytes.size());
  if (count < 0 && (errno == EINTR || errno == EAGAIN)) {return false;}
  if (count <= 0) {
    throw boost::system::system_error(EIO, boost::system::generic_category());
  }
  const double received = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  std::optional<DecodedScan> frame;
  for (ssize_t i = 0; i < count; ++i) {
    const auto result = decoder_.feed(bytes[i], received);
    if (result) {frame = result;}
  }
  if (!frame) {return false;}
  scan->angle_increment = (2.0 * M_PI / 360.0);
  scan->angle_min = 0.0;
  scan->angle_max = 2.0 * M_PI - scan->angle_increment;
  scan->range_min = 0.12;
  scan->range_max = 3.5;
  scan->ranges.assign(frame->ranges.begin(), frame->ranges.end());
  scan->intensities.assign(frame->intensities.begin(), frame->intensities.end());
  rpms = static_cast<uint16_t>(frame->rpm);
  const auto timing = scan_timing(*frame, received);
  if (!timing) {return false;}
  index_zero_steady_s = timing->index_zero_steady_s;
  scan->time_increment = static_cast<float>(timing->time_increment_s);
  scan->scan_time = static_cast<float>(timing->scan_time_s);
  return true;
}
}  // namespace hls_lfcd_lds

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("hlds_laser_publisher");
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
  boost::asio::io_service io;

  std::string port;
  std::string frame_id;
  int baud_rate;

  node->declare_parameter<std::string>("port");
  node->declare_parameter<std::string>("frame_id");

  node->get_parameter_or<std::string>("port", port, "/dev/ttyUSB0");
  node->get_parameter_or<std::string>("frame_id", frame_id, "laser");

  baud_rate = 230400;

  RCLCPP_INFO(node->get_logger(), "Init hlds_laser_publisher Node Main");
  RCLCPP_INFO(node->get_logger(), "port : %s frame_id : %s", port.c_str(), frame_id.c_str());

  try {
    hls_lfcd_lds::LFCDLaser laser(port, baud_rate, io);
    laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan",
      rclcpp::QoS(rclcpp::SensorDataQoS()));

    while (rclcpp::ok()) {
      auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
      scan->header.frame_id = frame_id;
      if (!laser.poll(scan)) {continue;}
      const double steady_now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      scan->header.stamp = node->now() - rclcpp::Duration::from_seconds(
        steady_now - laser.index_zero_steady_s);
      laser_pub->publish(*scan);
    }
    laser.close();

    return 0;
  } catch (boost::system::system_error & ex) {
    RCLCPP_ERROR(node->get_logger(), "LDS serial error: %s", ex.what());
    return -1;
  }
}

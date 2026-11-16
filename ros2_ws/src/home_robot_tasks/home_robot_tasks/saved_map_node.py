"""Publish verified saved raw/body maps. No localization or actuator authority."""
import argparse
from pathlib import Path
from .map_bundle import read_bundle,occupancy_values
from .navigation import NavigationMap


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle',type=Path,required=True)
    known,ros_args=parser.parse_known_args(args)
    manifest,maps=read_bundle(known.bundle)
    rclpy.init(args=ros_args);node=rclpy.create_node('saved_home_maps')
    publishers={key:node.create_publisher(OccupancyGrid,key+'_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)) for key in maps}
    def publish():
        for key,document in maps.items():
            nav=NavigationMap(document);msg=OccupancyGrid();msg.header.frame_id=nav.frame_id
            msg.header.stamp=node.get_clock().now().to_msg()  # publication time, not sensor evidence
            msg.info.resolution=nav.resolution;msg.info.width=nav.width;msg.info.height=nav.height
            msg.info.origin.position.x=nav.ox;msg.info.origin.position.y=nav.oy;msg.info.origin.position.z=nav.floor_z
            msg.info.origin.orientation.w=1.;msg.data=occupancy_values(document);publishers[key].publish(msg)
    # A subscriber may receive the initial map and request shutdown immediately.
    # Include initial publication and logging in the same cleanup scope as spin.
    try:
        timer=node.create_timer(1.,publish);publish()
        node.get_logger().info('Saved geometry only: localization and live clearance still required; revision '+manifest['revision'])
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()

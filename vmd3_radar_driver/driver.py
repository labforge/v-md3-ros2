# coding: utf-8
"""
******************************************************************************
*  Copyright 2025 Labforge Inc.                                              *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License");            *
* you may not use this project except in compliance with the License.        *
* You may obtain a copy of the License at                                    *
*                                                                            *
*     https://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and        *
* limitations under the License.                                             *
******************************************************************************

ROS2 driver interface node that publishes dummy radar data for testing.

"""
__author__ = "Thomas Reidemeister <thomas@labforge.ca>"
__copyright__ = "Copyright 2025, Labforge Inc."

import threading
import time

import numpy as np
import rclpy
from radar_msgs.msg import RadarReturn, RadarScan
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Int32, String

from vmd3_radar_driver.vmd3 import VMD3Driver, VMD3Modes


class VMD3RadarNode(Node):
    """ROS2 node for interfacing with the VMD3 radar and publishing data."""
    def __init__(self):
        """Initialize the VMD3RadarNode and its publishers, parameters, and polling thread."""
        super().__init__('vmd3_radar_driver')
        self.get_logger().info("Initializing VMD3RadarNode")
        # Declare parameters with defaults from README
        self.declare_parameter('address', '192.168.100.201')
        self.declare_parameter('port', 6172)
        self.declare_parameter('rport', 4567)
        self.declare_parameter('mode', 0)
        self.declare_parameter('sensitivity', 5)
        self.declare_parameter('min_distance', 0)
        self.declare_parameter('max_distance', 100)
        self.declare_parameter('min_speed', 0)
        self.declare_parameter('max_speed', 100)
        self.declare_parameter('min_lifetime', 5)
        self.declare_parameter('max_lifetime', 100)
        self.declare_parameter('stationary', False)

        # Publishers
        self._raw_pub = self.create_publisher(RadarScan, '/vmd3/detections', 10)
        self._target_pub = self.create_publisher(RadarScan, '/vmd3/targets', 10)
        self._raw_pub_pcl = self.create_publisher(PointCloud2, '/vmd3/detections_pcl', 10)
        self._target_pub_pcl = self.create_publisher(PointCloud2, '/vmd3/targets_pcl', 10)
        self._frame_id_pub = self.create_publisher(Int32, '/vmd3/frame_id', 10)
        self._status_pub = self.create_publisher(String, '/vmd3/status', 10)

        self._frame_counter = 0
        self._running = True
        self._thread = threading.Thread(target=self.poll_radar, daemon=True)
        self._thread.start()

    def poll_radar(self):
        """Poll the radar for data and publish results to ROS topics."""
        self.get_logger().info("Starting radar polling thread")
        # Group radar parameters into a dictionary to reduce local variables
        params = {
            'address': self.get_parameter('address').get_parameter_value().string_value,
            'port': self.get_parameter('port').get_parameter_value().integer_value,
            'rport': self.get_parameter('rport').get_parameter_value().integer_value,
            'mode': self.get_parameter('mode').get_parameter_value().integer_value,
            'sensitivity': self.get_parameter('sensitivity').get_parameter_value().integer_value,
            'min_distance': self.get_parameter('min_distance').get_parameter_value().integer_value,
            'max_distance': self.get_parameter('max_distance').get_parameter_value().integer_value,
            'min_speed': self.get_parameter('min_speed').get_parameter_value().integer_value,
            'max_speed': self.get_parameter('max_speed').get_parameter_value().integer_value,
            'min_lifetime': self.get_parameter('min_lifetime').get_parameter_value().integer_value,
            'max_lifetime': self.get_parameter('max_lifetime').get_parameter_value().integer_value,
            'stationary': self.get_parameter('stationary').get_parameter_value().bool_value
        }

        try:
            driver = VMD3Driver(params['address'], params['port'], params['rport'])
            driver.mode(VMD3Modes(params['mode']))
            driver.sensitivity(params['sensitivity'])
            driver.minimum_detection_distance(params['min_distance'])
            driver.maximum_detection_distance(params['max_distance'])
            driver.minimum_detection_speed(params['min_speed'])
            driver.maximum_detection_speed(params['max_speed'])
            driver.minimum_lifetime(params['min_lifetime'])
            driver.maximum_lifetime(params['max_lifetime'])
            driver.enable_static_tracking(params['stationary'])
            driver.stream_enable()
            self.publish_status('Radar running')
            self.get_logger().info("Radar initialized successfully")
        except (ValueError, RuntimeError, OSError) as e:
            self.publish_status(f'Error: {e}')
            self.get_logger().error(f'Failed to initialize radar: {e}')
            return

        while rclpy.ok() and self._running:
            try:
                data_type, received_data = next(driver)
                if data_type in ['PDAT', 'TDAT']:
                    self.publish_scan(data_type, received_data)
                elif data_type == 'DONE':
                    self._frame_counter = received_data
            except (StopIteration, RuntimeError, OSError) as e:
                self.publish_status(f'Error: {e}')
                self.get_logger().error(f'Polling error: {e}')
                time.sleep(1)

    def publish_scan(self, data_type, received_data):
        """Publish radar scan and point cloud messages."""
        self.get_logger().info(f"Publishing scan data of type {data_type}")
        radar_scan = RadarScan()
        radar_scan.header.stamp = self.get_clock().now().to_msg()
        radar_scan.header.frame_id = 'radar_link'
        radar_scan.returns = []
        for det in received_data:
            radar_return = RadarReturn()
            radar_return.range = det.distance_cm / 100.0
            radar_return.azimuth = det.azimuth_rad
            radar_return.elevation = det.elevation_rad
            radar_return.doppler_velocity = det.speed_kmh / 3.6  # km/h to m/s
            radar_return.amplitude = det.magnitude
            radar_scan.returns.append(radar_return)
        if data_type == 'TDAT':
            self._target_pub.publish(radar_scan)
            self._target_pub_pcl.publish(self.create_pointcloud(received_data))
        else:
            self._raw_pub.publish(radar_scan)
            self._raw_pub_pcl.publish(self.create_pointcloud(received_data))
        frame_id_msg = Int32()
        frame_id_msg.data = self._frame_counter
        self._frame_id_pub.publish(frame_id_msg)

    def create_pointcloud(self, detections):
        """Convert radar detections to a PointCloud2 message."""
        points = [(det.distance_cm/100.0 * np.cos(det.azimuth_rad) * np.cos(det.elevation_rad),
                   det.distance_cm/100.0 * np.sin(det.azimuth_rad) * np.cos(det.elevation_rad),
                   det.distance_cm/100.0 * np.sin(det.elevation_rad)) for det in detections]
        if not points:
            points = [(0.0, 0.0, 0.0)]
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_data = np.array(points, dtype=np.float32).tobytes()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'radar_link'
        msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,
            row_step=12*len(points),
            data=point_data
        )
        return msg

    def publish_status(self, status):
        """Publish a status message to the /vmd3/status topic."""
        self.get_logger().info(f"Publishing status: {status}")
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def destroy_node(self):
        """Clean up the node and stop polling."""
        self.get_logger().info("Destroying VMD3RadarNode")
        self._running = False
        super().destroy_node()


def main(args=None):
    """Main entry point for the VMD3RadarNode ROS2 node."""
    rclpy.init(args=args)
    node = VMD3RadarNode()
    try:
        node.get_logger().info("Spinning VMD3RadarNode")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

import os
import rclpy
from rclpy.node import Node
from common.pointcloud import PointCloud
from sensor_msgs.msg import PointCloud2


class RSM1LidarListener(Node):
    def __init__(self,
                 front_lidar_topic_name: str,
                 left_lidar_topic_name: str,
                 right_lidar_topic_name: str,
                 pcd_dump_root_dir: str):
        """
        Constructor.

        :param front_lidar_topic_name: topic name of front lidar.
        :param left_lidar_topic_name: topic name of left lidar.
        :param right_lidar_topic_name: topic name of right lidar.
        :param pcd_dump_root_dir: pcd files dump root directory.
        """
        super().__init__('RSM1LidarListener')
        self._front_lidar_topic_name = front_lidar_topic_name
        self._left_lidar_topic_name = left_lidar_topic_name
        self._right_lidar_topic_name = right_lidar_topic_name
        self._pcd_dump_root_dir = pcd_dump_root_dir

        if not os.path.exists(self._pcd_dump_root_dir):
            os.makedirs(self._pcd_dump_root_dir)

        self._front_lidar_subscriber = self.create_subscription(
            PointCloud2, self._front_lidar_topic_name, self.front_lidar_pcd_dump_callback, 10)
        self._left_lidar_subscriber = self.create_subscription(
            PointCloud2, self._left_lidar_topic_name, self.left_lidar_pcd_dump_callback, 10)
        self._right_lidar_subscriber = self.create_subscription(
            PointCloud2, self._right_lidar_topic_name, self.right_lidar_pcd_dump_callback, 10)

    def front_lidar_pcd_dump_callback(self, msg: PointCloud2):
        timestamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        dump_pcd_full_path = os.path.join(self._pcd_dump_root_dir, "lidar__front__{}.pcd".format(timestamp))
        pc = PointCloud.from_msg(msg)
        pc.save_pcd(dump_pcd_full_path, compression="ascii")
        print("Save front lidar data into {}".format(dump_pcd_full_path))

    def left_lidar_pcd_dump_callback(self, msg: PointCloud2):
        timestamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        dump_pcd_full_path = os.path.join(self._pcd_dump_root_dir, "lidar__left__{}.pcd".format(timestamp))
        pc = PointCloud.from_msg(msg)
        pc.save_pcd(dump_pcd_full_path, compression="ascii")
        print("Save left lidar data into {}".format(dump_pcd_full_path))

    def right_lidar_pcd_dump_callback(self, msg: PointCloud2):
        timestamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        dump_pcd_full_path = os.path.join(self._pcd_dump_root_dir, "lidar__right__{}.pcd".format(timestamp))
        pc = PointCloud.from_msg(msg)
        pc.save_pcd(dump_pcd_full_path, compression="ascii")
        print("Save right lidar data into {}".format(dump_pcd_full_path))


if __name__ == '__main__':
    rclpy.init()
    robosense_m1_lidar_listener = RSM1LidarListener(
        front_lidar_topic_name="/rslidar_front_points",
        left_lidar_topic_name="/rslidar_left_points",
        right_lidar_topic_name="/rslidar_right_points",
        pcd_dump_root_dir="./pcd/"
    )
    rclpy.spin(robosense_m1_lidar_listener)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # minimal_subscriber.destroy_node()
    # rclpy.shutdown()

import rclpy

from rclpy.node import Node
from tf_transformations import quaternion_from_euler

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from pinky_msgs.msg import Encoder, HumanPos

import math
from .imu import IMU
from .battery import Battery

WHEEL_RAD = 0.028
PULSE_PER_ROT = 4096 
WHEEL_BASE = 0.0961
RPM2RAD = 2 * math.pi / 60
CIRCUMFERENCE = 2 * math.pi * WHEEL_RAD

IMU_FRAMEID = "imu"
ODOM_FRAMEID = "odometry"
ODOM_CHILDID = "odometry/filtered"
BATTERY_FRAMEID = "battery/present"
ENCODER_FRAMEID = "encoder"
LIDAR_FRAMEID = "scan"

class SensorManager(Node):
    def __init__(self):
        super().__init__('SensorManager')
        self.battery = Battery()
        # self.Imu = IMU()

        self.last_time = self.get_clock().now()
        self.last_encoder_l = 0
        self.last_encoder_r = 0
        self.theta = 0
        self.x = 0
        self.y = 0

        self.battery_timer_period = 5.0
        self.imu_timer_period = 1.0 / 60.0

        self.percentage_timer = self.create_timer(self.battery_timer_period, self.battery_percentage_callback)
        # self.imu_timer = self.create_timer(self.imu_timer_period, self.imu_sensor_callback)

        self.percentage_publisher = self.create_publisher(
            Float32,
            BATTERY_FRAMEID,
            10
        )

        # self.imu_raw_publisher = self.create_publisher(
        #     Imu,
        #     IMU_FRAMEID,
        #     10
        # )

        # self.fined_human_publisher = self.create_publisher(
        #     HumanPos,

        # )

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            LIDAR_FRAMEID,
            self.lidar_callback,
            10
        )


    def lidar_callback(self, msg):
        #TODO
        pass

    # def imu_sensor_callback(self):
    #     t = Imu()
    #     raw_data = self.Imu.read_imu_data()
        
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = IMU_FRAMEID
    #     t.orientation.w = raw_data['quaternion'][0]
    #     t.orientation.x = raw_data['quaternion'][1]
    #     t.orientation.y = raw_data['quaternion'][2]
    #     t.orientation.z = raw_data['quaternion'][3]
    #     t.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

    #     t.angular_velocity.x = raw_data['gyro'][0]
    #     t.angular_velocity.y = raw_data['gyro'][1]
    #     t.angular_velocity.z = raw_data['gyro'][2]
    #     t.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

    #     t.linear_acceleration.x = raw_data['acceleration'][0]
    #     t.linear_acceleration.y = raw_data['acceleration'][1]
    #     t.linear_acceleration.z = raw_data['acceleration'][2]
    #     t.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

    #     self.imu_raw_publisher.publish(t)

    def battery_percentage_callback(self):
        pct_msg = Float32()
        pct_msg.data = float(self.battery.battery_percentage())
        self.percentage_publisher.publish(pct_msg)

    def __del__(self):
        # self.Imu.close()
        self.battery.close()


def main(args=None):
    rclpy.init(args=args)
    node = SensorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        


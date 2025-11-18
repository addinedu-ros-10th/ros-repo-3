import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import pinkylib

IMU_FRAMEID = "imu"
ODOM_FRAMEID = "odom"
BATTERY_FRAMEID = "battery/present"

class SensorManager(Node):
    def __init__(self):
        super().__init__('SensorManager')
        self.battery = pinkylib.Battery()
        self.Imu = pinkylib.Imu()

        
        self.battery_timer_period = 5.0
        self.imu_timer_period = 0.1
        self.percentage_timer = self.create_timer(self.battery_timer_period, self.battery_percentage_callback)
        self.imu_timer = self.create_timer(self.imu_timer_period, self.imu_sensor_callback)

        self.percentage_publisher = self.create_publisher(
            Float32,
            BATTERY_FRAMEID,
            10
        )

        self.imu_raw_publisher = self.create_publisher(
            Imu,
            IMU_FRAMEID,
            10
        )

        self.odom_raw_publisher = self.create_publisher(
            Odometry,
            ODOM_FRAMEID,
            10
        )

        self.encoder_subscriber = self.create_subscription(
            
        )

    def imu_sensor_callback(self):
        t = Imu()
        raw_data = self.Imu.read_imu_data()

        t.header.stamp = self.get_clock.now()
        t.header.frame_id = IMU_FRAMEID
        t.orientation.w = raw_data['quaternion'][0]
        t.orientation.x = raw_data['quaternion'][1]
        t.orientation.y = raw_data['quaternion'][2]
        t.orientation.z = raw_data['quaternion'][3]
        t.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        t.angular_velocity.x = raw_data['gyro'][0]
        t.angular_velocity.y = raw_data['gyro'][1]
        t.angular_velocity.z = raw_data['gyro'][2]
        t.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        t.linear_acceleration.x = raw_data['acceleration'][0]
        t.linear_acceleration.y = raw_data['acceleration'][1]
        t.linear_acceleration.z = raw_data['acceleration'][2]
        t.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        self.imu_raw_publisher.publish(t)

    def battery_percentage_callback(self):
        pct_msg = Float32()
        pct_msg.data = float(self.battery.battery_percentage())
        self.percentage_publisher.publish(pct_msg)


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import time
import adafruit_icm20x

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        timer_period = 0.05  # seconds (20 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        try:
            i2c = board.I2C()
            self.icm =  adafruit_icm20x.ICM20649(i2c)
            self.get_logger().info('IMU ICM20649 initialized successfully!')
            
            # --- AGGIUNGI QUESTE RIGHE PER ESPLORARE ---
            self.get_logger().info(f"Attributi disponibili su self.icm: {dir(self.icm)}")
            # Stampa solo gli attributi pubblici e non i metodi speciali
            self.get_logger().info(f"Attributi pubblici di self.icm: {[attr for attr in dir(self.icm) if not attr.startswith('_')]}")
            # --- FINE DELL'AGGIUNTA ---
            
        except ValueError as e:
            self.get_logger().error(f"Failed to initialize ICM20649: {e}")
            rclpy.shutdown()
            return

        self.get_logger().info('IMU Publisher Node has been started.')

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        accel_x, accel_y, accel_z = self.icm.acceleration
        gyro_x, gyro_y, gyro_z = self.icm.gyro

        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z

        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

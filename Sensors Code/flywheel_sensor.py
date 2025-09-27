import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
import re

class FlywheelSensorReader(Node):
    def __init__(self):
        super().__init__('flywheels_sensors')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'flywheel_rpms', 10)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.serial_port_name = '/dev/ttyACM0'
        self.baud_rate = 115200

        self.get_logger().info(f'Attempting to open serial port: {self.serial_port_name} at {self.baud_rate} baud.')
        try:
            self.ser = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
            self.get_logger().info('Serial port opened successfully.')
            self.ser.flushInput()
            self.ser.flushOutput()
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port_name}: {e}")
            rclpy.shutdown()
            return

        self.pattern = re.compile(
            r"M1_vel:\s*(-?\d+\.?\d*)\s*RPM\s*\|\s*" +
            r"M1_vel_avg:\s*(-?\d+\.?\d*)\s*RPM\s*\|\s*" +
            r"M2_vel:\s*(-?\d+\.?\d*)\s*RPM\s*\|\s*" +
            r"M2_vel_avg:\s*(-?\d+\.?\d*)\s*RPM\s*\|\s*" +
            r"M3_vel:\s*(-?\d+\.?\d*)\s*RPM\s*\|\s*" +
            r"M3_vel_avg:\s*(-?\d+\.?\d*)\s*RPM"
        )
        
        self.get_logger().info("ROS2 Node 'flywheels_sensors' ready to publish 6 RPM values on '/flywheel_rpms'.")

    def timer_callback(self):
        if not self.ser.is_open:
            self.get_logger().error("Serial port is not open. Cannot read data.")
            return

        try:
            # Assicurati che ci siano dati in attesa di essere letti
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                # self.get_logger().info(f"Raw serial data received: '{line}'") # <--- RIGA DI DEBUG

                match = self.pattern.match(line)
                if match:
                    # self.get_logger().info(f"Regex matched! Extracted values: {match.groups()}") # <--- RIGA DI DEBUG
                    try:
                        rpm_m1_vel = float(match.group(1))
                        rpm_m1_vel_avg = float(match.group(2))
                        rpm_m2_vel = float(match.group(3))
                        rpm_m2_vel_avg = float(match.group(4))
                        rpm_m3_vel = float(match.group(5))
                        rpm_m3_vel_avg = float(match.group(6))

                        msg = Float32MultiArray()
                        msg.data = [rpm_m1_vel, rpm_m1_vel_avg, rpm_m2_vel, rpm_m2_vel_avg, rpm_m3_vel, rpm_m3_vel_avg]

                        self.publisher_.publish(msg)
                        # self.get_logger().info(f"Published: {msg.data}") # <--- RIGA DI DEBUG
                    except ValueError:
                        self.get_logger().warn(f"Could not convert values in line: '{line}'")
                else:
                    self.get_logger().warn(f"No regex match for line: '{line}'") # <--- RIGA DI DEBUG (se non c'  match)
            else:
                self.get_logger().debug("No data in serial buffer.") # <--- Utile per capire se il buffer   vuoto
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    flywheel_sensor_reader = FlywheelSensorReader()
    try:
        rclpy.spin(flywheel_sensor_reader)
    except KeyboardInterrupt:
        pass
    finally:
        flywheel_sensor_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

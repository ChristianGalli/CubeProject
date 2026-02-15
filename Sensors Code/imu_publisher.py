import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu

import board
import numpy as np
# Import specifici per configurare i Range massimi
from adafruit_icm20x import ICM20649, AccelRange, GyroRange

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # --------------------------------------------------
        # PARAMETRI
        # --------------------------------------------------
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 200.0)    # Hz
        self.declare_parameter('calibration_duration', 30.0)
        
        # Coefficienti Filtro (Tuo Design)
        self.declare_parameter('acc_b1', 0.0198)
        self.declare_parameter('acc_b2', 0.0198)
        self.declare_parameter('acc_a2', -0.9604)
        self.declare_parameter('gyro_b1', 0.4469)
        self.declare_parameter('gyro_b2', 0.4469)
        self.declare_parameter('gyro_a2', -0.1061)

        self.update_parameters()
        self.add_on_set_parameters_callback(self.parameters_callback)

        # --------------------------------------------------
        # HARDWARE SETUP (ALTE PRESTAZIONI)
        # --------------------------------------------------
        try:
            i2c = board.I2C()
            self.icm = ICM20649(i2c)
            
            # 1. RANGE MASSIMI (Cruciale per non divergere in rotazione)
            self.icm.accelerometer_range = AccelRange.RANGE_30G
            self.icm.gyro_range = GyroRange.RANGE_4000_DPS
            
            # 2. DATA RATE ALTO (Riduce latenza e aliasing)
            self.icm.accelerometer_data_rate = 1100 
            self.icm.gyro_data_rate = 1100
            
            self.get_logger().info('IMU Configurata: 30G, 4000dps (Anti-Saturazione Attivo)')
        except Exception as e:
            self.get_logger().fatal(f'Errore Hardware IMU: {e}')
            raise e

        # --------------------------------------------------
        # PUBLISHERS & STATO
        # --------------------------------------------------
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        # Topic per Madgwick (Veloce, non filtrato)
        self.raw_publisher_ = self.create_publisher(Imu, 'imu/data_raw_unfiltered', qos)
        # Topic per Controllo (Filtrato, pulito)
        self.filtered_publisher_ = self.create_publisher(Imu, 'imu/data_raw', qos)

        self.calibrating = True
        self.start_time = self.get_clock().now()
        
        self.acc_offsets = np.zeros(3)
        self.gyro_offsets = np.zeros(3)
        self.sample_count = 0 

        # Variabili per il filtro (n-1)
        self.prev_acc_raw = np.zeros(3)
        self.prev_acc_filt = np.zeros(3)
        self.prev_gyro_raw = np.zeros(3)
        self.prev_gyro_filt = np.zeros(3)
        self.first_run = True

        self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)
        self.get_logger().info(f'Avvio Calibrazione ({self.calib_duration}s). NON MUOVERE IL ROBOT.')

    def update_parameters(self):
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.calib_duration = self.get_parameter('calibration_duration').value
        self.acc_b1 = self.get_parameter('acc_b1').value
        self.acc_b2 = self.get_parameter('acc_b2').value
        self.acc_a2 = self.get_parameter('acc_a2').value
        self.gyro_b1 = self.get_parameter('gyro_b1').value
        self.gyro_b2 = self.get_parameter('gyro_b2').value
        self.gyro_a2 = self.get_parameter('gyro_a2').value

    def parameters_callback(self, params):
        self.update_parameters()
        return SetParametersResult(successful=True)

    def timer_callback(self):
        now = self.get_clock().now()
        try:
            # Lettura Hardware
            acc_raw = np.array(self.icm.acceleration)
            gyro_raw = np.array(self.icm.gyro)
        except OSError:
            return 

        # --- FASE 1: CALIBRAZIONE (SILENZIOSA) ---
        if self.calibrating:
            self.sample_count += 1
            self.acc_offsets += (acc_raw - self.acc_offsets) / self.sample_count
            self.gyro_offsets += (gyro_raw - self.gyro_offsets) / self.sample_count

            elapsed = (now - self.start_time).nanoseconds * 1e-9
            if elapsed >= self.calib_duration:
                # Correzione Z per la gravit�
                if abs(self.acc_offsets[2]) > 8.0: 
                     self.acc_offsets[2] -= 9.80665 * np.sign(self.acc_offsets[2])
                
                self.calibrating = False
                self.get_logger().info(f'--- CALIBRAZIONE COMPLETATA --- Bias Gyro: {self.gyro_offsets}')
                self.get_logger().info('Inizio pubblicazione dati...')
                self.first_run = True
            
            # IMPORTANTE: RETURN QUI. 
            # Non pubblichiamo nulla finch� non siamo calibrati.
            # Madgwick rester� in attesa.
            return

        # --- FASE 2: ELABORAZIONE & PUBBLICAZIONE ---
        acc_corr = acc_raw - self.acc_offsets
        gyro_corr = gyro_raw - self.gyro_offsets

        # Filtro IIR
        if self.first_run:
            acc_filt = acc_corr
            gyro_filt = gyro_corr
            self.first_run = False
        else:
            acc_filt = (self.acc_b1 * acc_corr) + (self.acc_b2 * self.prev_acc_raw) - (self.acc_a2 * self.prev_acc_filt)
            gyro_filt = (self.gyro_b1 * gyro_corr) + (self.gyro_b2 * self.prev_gyro_raw) - (self.gyro_a2 * self.prev_gyro_filt)

        self.prev_acc_raw = acc_corr
        self.prev_acc_filt = acc_filt
        self.prev_gyro_raw = gyro_corr
        self.prev_gyro_filt = gyro_filt

        # A) Pubblica RAW per Madgwick (Veloce)
        self.publish_imu(self.raw_publisher_, now, acc_corr, gyro_corr)

        # B) Pubblica FILTERED per Controllo (Pulito)
        self.publish_imu(self.filtered_publisher_, now, acc_filt, gyro_filt)

    def publish_imu(self, publisher, time, acc, gyro):
        msg = Imu()
        msg.header.stamp = time.to_msg()
        msg.header.frame_id = self.frame_id
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.orientation_covariance[0] = -1.0 
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

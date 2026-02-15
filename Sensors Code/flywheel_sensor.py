#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from smbus2 import SMBus

# ============================================================
# ADS1115 CONSTANTS
# ============================================================
ADS1115_REG_CONVERSION = 0x00
ADS1115_REG_CONFIG     = 0x01

# Configurazione base: OS=1, PGA=4.096V, Mode=Single-Shot, DataRate=860sps
CONFIG_BASE = 0x83E0 

# Multiplexer Configs (Single Ended)
MUX_AIN0 = 0x4000
MUX_AIN1 = 0x5000
MUX_AIN2 = 0x6000
MUX_AIN3 = 0x7000

# LSB size per PGA +/- 4.096V
LSB_SIZE = 4.096 / 32767.0

class FlywheelSensorReader(Node):
    def __init__(self):
        super().__init__('flywheels_sensors')

        # -------------------------------
        # 1. PARAMETRI FISICI
        # -------------------------------
        self.declare_parameter('v_driver_min', 0.2)     # -5000 RPM
        self.declare_parameter('v_driver_max', 3.2)     # +5000 RPM
        self.declare_parameter('rpm_driver_min', -5000.0)
        self.declare_parameter('rpm_driver_max', 5000.0)
        self.declare_parameter('amp_driver_min', -3.5)
        self.declare_parameter('amp_driver_max', 3.5)
        
        # Soglie e Tempi
        self.declare_parameter('v_cutoff_safety', 0.15) # Sotto 0.15V -> Output 0
        self.declare_parameter('v_deadzone', 0.05)      # Deadzone in Volt
        
        # Tempi di calibrazione
        self.declare_parameter('time_calib_volt', 20.0) # Fase 1: Tara voltaggio
        self.declare_parameter('time_calib_rpm', 10.0)  # Fase 2: Verifica zero RPM
        
        # Configurazione I2C
        self.declare_parameter('i2c_bus', 3)
        self.declare_parameter('ads_addresses', [0x48, 0x49, 0x4A])
        self.declare_parameter('publish_rate', 40.0)

        # -------------------------------
        # 2. CALCOLO INTERNO SLOPE
        # -------------------------------
        v_min = self.get_parameter('v_driver_min').value
        v_max = self.get_parameter('v_driver_max').value
        r_min = self.get_parameter('rpm_driver_min').value
        r_max = self.get_parameter('rpm_driver_max').value
        
        # Slope = (Delta RPM) / (Delta Volt)
        if (v_max - v_min) != 0:
            self.slope = (r_max - r_min) / (v_max - v_min)
        else:
            self.slope = 0.0
            self.get_logger().error("Errore parametri: Delta Volt = 0. Slope RPM impostata a 0.")

        self.get_logger().info(f"Slope RPM calcolata: {self.slope:.2f} RPM/Volt")
        
        # --- PARAMETRI AMPERE (Coppia 2) ---
               
        a_min = self.get_parameter('amp_driver_min').value
        a_max = self.get_parameter('amp_driver_max').value
        
        # Slope Ampere = (Delta Ampere) / (Delta Volt)
        if (v_max - v_min) != 0:
            self.slope_amp = (a_max - a_min) / (v_max - v_min)
        else:
            self.slope_amp = 0.0
            self.get_logger().error("Errore parametri: Delta Volt = 0. Slope Ampere impostata a 0.")
            
        self.get_logger().info(f"Slope Ampere calcolata: {self.slope:.2f} A/Volt")

        # Recupero Parametri
        self.cutoff_limit = self.get_parameter('v_cutoff_safety').value
        self.deadzone = self.get_parameter('v_deadzone').value
        self.t_calib_v = self.get_parameter('time_calib_volt').value
        self.t_calib_r = self.get_parameter('time_calib_rpm').value
        
        self.bus_id = self.get_parameter('i2c_bus').value
        self.addresses = self.get_parameter('ads_addresses').value
        self.rate = self.get_parameter('publish_rate').value

        # -------------------------------
        # 3. INIT I2C
        # -------------------------------
        try:
            self.bus = SMBus(self.bus_id)
        except Exception as e:
            self.get_logger().fatal(f"Errore apertura I2C Bus {self.bus_id}: {e}")
            raise e

        # -------------------------------
        # 4. VARIABILI DI STATO
        # -------------------------------
        # Offsets definitivi (tara voltaggio): 3 chip x 2 coppie
        self.offsets = [[2.1, 2.1] for _ in self.addresses] 
        
        # Stato Calibrazione
        # 0 = Calib Volts (Tara), 1 = Calib RPM (Check Noise), 2 = Running
        self.calib_state = 0 
        self.start_time = self.get_clock().now()
        
        # Accumulatori per le medie
        self.samples_v = 0
        self.accum_v = [[0.0, 0.0] for _ in self.addresses]
        
        self.samples_r = 0
        self.accum_r_noise = [[0.0, 0.0] for _ in self.addresses] # Per misurare il rumore in RPM

        # ROS
        self.pub_rpm = self.create_publisher(Float32MultiArray, 'flywheel_rpms', 10)
        self.pub_amp = self.create_publisher(Float32MultiArray, 'flywheel_amps', 10)
        self.timer = self.create_timer(1.0 / self.rate, self.loop_callback)
        
        self.get_logger().info(f"START: Fase 1 TARA VOLTAGGIO ({self.t_calib_v}s)...")

    # ---------------------------------------------------------
    # UTILS
    # ---------------------------------------------------------
    def read_single_channel(self, addr, mux_config):
        try:
            cfg = CONFIG_BASE | mux_config
            self.bus.write_i2c_block_data(addr, ADS1115_REG_CONFIG, [(cfg >> 8) & 0xFF, cfg & 0xFF])
            time.sleep(0.002) # Wait conversion
            d = self.bus.read_i2c_block_data(addr, ADS1115_REG_CONVERSION, 2)
            raw = (d[0] << 8) | d[1]
            if raw > 32767: raw -= 65536
            return raw * LSB_SIZE
        except OSError:
            return 0.0

    # ---------------------------------------------------------
    # MAIN LOOP
    # ---------------------------------------------------------
    def loop_callback(self):
        # Calcolo tempo trascorso
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        # --- GESTIONE STATI ---
        # Cambio Stato 0 -> 1 (Fine Tara Volt)
        if self.calib_state == 0 and elapsed >= self.t_calib_v:
            self.finalize_calibration_volt()
            self.calib_state = 1
            self.get_logger().info(f"FASE 1 COMPLETATA. Inizio Fase 2: CHECK RPM ({self.t_calib_r}s)...")

        # Cambio Stato 1 -> 2 (Fine Check RPM -> Running)
        if self.calib_state == 1 and elapsed >= (self.t_calib_v + self.t_calib_r):
            self.finalize_calibration_rpm()
            self.calib_state = 2
            self.get_logger().info("FASE 2 COMPLETATA. PUBBLICAZIONE ATTIVA.")

        # --- ACQUISIZIONE DATI ---
        # Struttura: [ [(v_pos, diff), (v_pos, diff)], ... ]
        current_data = []

        for addr in self.addresses:
            # Coppia 1 (Ch0-Ch1)
            v0 = self.read_single_channel(addr, MUX_AIN0)
            v1 = self.read_single_channel(addr, MUX_AIN1)
            diff1 = v0 - v1
            
            # Coppia 2 (Ch2-Ch3)
            v2 = self.read_single_channel(addr, MUX_AIN2)
            v3 = self.read_single_channel(addr, MUX_AIN3)
            diff2 = v2 - v3
            
            current_data.append([(v0, diff1), (v2, diff2)])

        # --- AZIONE IN BASE ALLO STATO ---

        # 1. FASE CALIBRAZIONE VOLTAGGI (TARA)
        if self.calib_state == 0:
            for i in range(len(self.addresses)):
                self.accum_v[i][0] += current_data[i][0][1] # Accumula diff1
                self.accum_v[i][1] += current_data[i][1][1] # Accumula diff2
            self.samples_v += 1
            return # Non pubblicare

        # 2. FASE CALIBRAZIONE RPM e AMPERE (VERIFICA RUMORE)
        elif self.calib_state == 1:
            for i in range(len(self.addresses)):
                # --- Rumore Coppia 1: RPM ---
                diff_0 = current_data[i][0][1]
                offset_0 = self.offsets[i][0]
                rpm_inst = (diff_0 - offset_0) * self.slope
                # Usiamo l'indice [0] dell'accumulatore per il rumore RPM
                self.accum_r_noise[i][0] += abs(rpm_inst)
        
                # --- Rumore Coppia 2: AMPERE ---
                diff_1 = current_data[i][1][1]
                offset_1 = self.offsets[i][1]
                amp_inst = (diff_1 - offset_1) * self.slope_amp
                # Usiamo l'indice [1] dell'accumulatore per il rumore Ampere
                self.accum_r_noise[i][1] += abs(amp_inst)
        
            self.samples_r += 1
            return # Non pubblicare durante la calibrazione

        # 3. RUNNING (Pubblicazione)
        elif self.calib_state == 2:
          out_rpms = []
          out_amps = []
          
          for i in range(len(self.addresses)):
              # --- COPPIA 1: RPM (Ch 0-1) ---
              v_pos_0, v_diff_0 = current_data[i][0]
              offset_0 = self.offsets[i][0]
              
              if v_pos_0 < self.cutoff_limit:
                  val_rpm = 0.0
              else:
                  delta_0 = v_diff_0 - offset_0
                  if abs(delta_0) < self.deadzone: delta_0 = 0.0
                  val_rpm = delta_0 * self.slope
              out_rpms.append(float(val_rpm))
      
              # --- COPPIA 2: AMPERE (Ch 2-3) ---
              v_pos_2, v_diff_2 = current_data[i][1]
              offset_1 = self.offsets[i][1] 
              
              if v_pos_2 < self.cutoff_limit:
                  val_amp = 0.0
              else:
                  delta_1 = v_diff_2 - offset_1
                  if abs(delta_1) < self.deadzone: delta_1 = 0.0
                  val_amp = delta_1 * self.slope_amp
              out_amps.append(float(val_amp))
      
          # Pubblicazione
          msg_rpm = Float32MultiArray(data=out_rpms)
          msg_amp = Float32MultiArray(data=out_amps)
          self.pub_rpm.publish(msg_rpm)
          self.pub_amp.publish(msg_amp)

    # ---------------------------------------------------------
    # METODI DI TRANSIZIONE
    # ---------------------------------------------------------
    def finalize_calibration_volt(self):
        """Calcola la media dei voltaggi accumulati nei primi 20s"""
        if self.samples_v > 0:
            for i in range(len(self.addresses)):
                self.offsets[i][0] = self.accum_v[i][0] / self.samples_v
                self.offsets[i][1] = self.accum_v[i][1] / self.samples_v
            self.get_logger().info(f"Offsets Calcolati (Volt): {self.offsets}")
        else:
            self.get_logger().warn("Nessun campione per calibrazione Volt!")

    def finalize_calibration_rpm(self):
        """Analizza il rumore medio per i giri (Coppia Ch0-Ch1)"""
        self.get_logger().info("--- [REPORT RUMORE: RPM] ---")
        if self.samples_r > 0:
            for i in range(len(self.addresses)):
                # L'indice [i][0] contiene l'accumulo della prima coppia (RPM)
                n_rpm = self.accum_r_noise[i][0] / self.samples_r
                addr_hex = hex(self.addresses[i])
                self.get_logger().info(f"Chip {i} ({addr_hex}): Noise RPM = +-{n_rpm:.1f}")
        else:
            self.get_logger().warn("Nessun campione per calibrazione RPM!")

    def finalize_calibration_ampere(self):
        """Analizza il rumore medio per la corrente (Coppia Ch2-Ch3)"""
        self.get_logger().info("--- [REPORT RUMORE: AMPERE] ---")
        if self.samples_r > 0:
            for i in range(len(self.addresses)):
                # L'indice [i][1] contiene l'accumulo della seconda coppia (Ampere)
                n_amp = self.accum_r_noise[i][1] / self.samples_r
                addr_hex = hex(self.addresses[i])
                self.get_logger().info(f"Chip {i} ({addr_hex}): Noise Ampere = +-{n_amp:.3f} A")
            
            self.get_logger().info("----------------------------------------------")
        else:
            self.get_logger().warn("Nessun campione per calibrazione Ampere!")

def main(args=None):
    rclpy.init(args=args)
    node = FlywheelSensorReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

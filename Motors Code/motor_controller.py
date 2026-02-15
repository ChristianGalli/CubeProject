# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from gpiozero import OutputDevice, PWMOutputDevice
import numpy as np
import threading
import time


class Motor:
    """Gestione hardware di un singolo motore con cache dello stato."""
    def __init__(self, pins: list, pwm_freq: float, i_max: float):
        if len(pins) != 3:
            raise ValueError("La lista dei pin deve contenere esattamente 3 valori [DIR_A, DIR_B, PWM]")
        self.dir_a = OutputDevice(pins[0])
        self.dir_b = OutputDevice(pins[1])
        self.pwm = PWMOutputDevice(pins[2], frequency=pwm_freq, initial_value=0.0)
        self.i_max = i_max
        self._current_state = (False, False, 0.0)

    def update(self, direzione: float, corrente: float):
        """Aggiorna lo stato fisico solo se diverso dal precedente."""
        pwm_val = 0.1 + (np.clip(abs(corrente) / self.i_max, 0.0, 1.0) * 0.8)
        if direzione > 0.1:
            new_state = (True, False, pwm_val)
        elif direzione < -0.1:
            new_state = (False, True, pwm_val)
        else:
            new_state = (False, False, 0.0)

        if new_state != self._current_state:
            self.dir_a.value = new_state[0]
            self.dir_b.value = new_state[1]
            self.pwm.value = new_state[2]
            self._current_state = new_state

    def stop(self):
        """Ferma il motore."""
        self.update(0.0, 0.0)

    def close(self):
        """Chiude in sicurezza tutti i pin GPIO."""
        try:
            self.stop()
        finally:
            for d in (self.dir_a, self.dir_b, self.pwm):
                try:
                    d.close()
                except Exception:
                    pass


class HighFreqMotorController(Node):
    """Nodo ROS2 ad alta frequenza per controllo flywheel."""
    def __init__(self):
        super().__init__('motor_controller')

        # Parametri
        self.declare_parameters('', [
            ('motors_config', [25, 24, 13, 23, 22, 19, 27, 17, 18]),
            ('i_max', 3.21),
            ('pwm_freq', 1000),
            ('timeout_sec', 0.5),
            ('update_hz', 1000.0)
        ])

        # Lettura parametri
        pins_raw = self.get_parameter('motors_config').value
        self.i_max = self.get_parameter('i_max').value
        self.pwm_freq = self.get_parameter('pwm_freq').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        self.update_hz = self.get_parameter('update_hz').value
        self.period = 1.0 / self.update_hz

        # Inizializzazione motori
        if len(pins_raw) % 3 != 0:
            raise ValueError("motors_config deve essere multiplo di 3")
        self.motors = [Motor(pins_raw[i:i+3], self.pwm_freq, self.i_max)
                       for i in range(0, len(pins_raw), 3)]
        self.num_motors = len(self.motors)

        # Buffer thread-safe
        self.cmd_lock = threading.Lock()
        self.latest_data = np.zeros(self.num_motors * 2)
        self.last_rx_time = time.perf_counter()
        self.is_active = False

        # Sottoscrizione ROS
        self.sub = self.create_subscription(Float64MultiArray, '/flywheels', self.callback, 10)

        # Thread di controllo motori
        self.running = True
        self.motor_thread = threading.Thread(target=self._motor_loop, daemon=True)
        self.motor_thread.start()

        self.get_logger().info(f"Flywheel Controller Ready: {self.num_motors} motori, {self.update_hz:.1f} Hz")

    def callback(self, msg: Float64MultiArray):
        """Ricezione comandi ROS 2."""
        with self.cmd_lock:
            self.latest_data = np.array(msg.data)
            self.last_rx_time = time.perf_counter()
            self.is_active = True

    def _motor_loop(self):
        """Thread ad alta frequenza per aggiornamento motori."""
        while self.running:
            t_start = time.perf_counter()

            current_time = time.perf_counter()
            data = None

            # Watchdog e copia dati
            with self.cmd_lock:
                if not self.is_active or (current_time - self.last_rx_time > self.timeout_sec):
                    data = None
                    if self.is_active:
                        self.get_logger().warn("Motor Watchdog Triggered: Stop all motori")
                        self.is_active = False
                else:
                    data = self.latest_data

            # Aggiornamento motori solo con dati validi
            if data is not None:
                for i, motor in enumerate(self.motors):
                    idx = i * 2
                    # Verifica che ci siano dati validi per il motore
                    if idx + 1 < len(data):
                        motor.update(data[idx], data[idx + 1])
                    else:
                        self.get_logger().warn(f"Dati incompleti per motore {i}, fermando il motore.")
                        motor.stop()
            else:
                # Ferma tutti i motori se non ci sono dati validi
                for motor in self.motors:
                    motor.stop()

            # Sincronizzazione loop
            elapsed = time.perf_counter() - t_start
            sleep_time = self.period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop_all(self):
        """Ferma tutti i motori e chiude GPIO."""
        self.running = False
        if self.motor_thread.is_alive():
            self.motor_thread.join(timeout=1.0)
        for motor in self.motors:
            motor.close()
        self.get_logger().info("Hardware rilasciato correttamente.")


def main(args=None):
    rclpy.init(args=args)
    node = HighFreqMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

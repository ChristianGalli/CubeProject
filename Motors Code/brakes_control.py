import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray, String
from rcl_interfaces.msg import SetParametersResult
import gpiod
import threading
import time
import json
from concurrent.futures import ThreadPoolExecutor


class BrakesControl(Node):
    def __init__(self, max_workers=3):
        super().__init__('brakes_control')

        # =========================
        # 1. Parametri configurabili
        # =========================
        self.declare_parameter('chip_name', 'gpiochip4')
        self.declare_parameter('lock_time', 1.0)
        self.declare_parameter('motor_map', [1, 16, 2, 6, 3, 5])

        self.chip_name = self.get_parameter('chip_name').value
        self.t_post_cycle_lock = self.get_parameter('lock_time').value
        self.motor_pins = self._parse_motor_map(self.get_parameter('motor_map').value)

        # =========================
        # 2. Stato motori e sincronizzazione
        # =========================
        self.lines = {}  # GPIO lines
        self.motor_state_flags = {m_id: 'idle' for m_id in self.motor_pins}  # idle, running, locked
        self.state_lock = threading.Lock()

        # =========================
        # 3. Thread pool per esecuzione motori
        # =========================
        self.executor_pool = ThreadPoolExecutor(max_workers=max_workers)

        # =========================
        # 4. Hardware GPIO
        # =========================
        self._init_gpio()

        # =========================
        # 5. ROS interfaces
        # =========================
        self.subscription = self.create_subscription(
            Float64MultiArray, '/brakes', self.listener_callback, 10
        )
        self.state_publisher = self.create_publisher(String, '/brakes/status', 10)
        self.create_timer(1.0, self.publish_status)

        # Parametri runtime
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f"Nodo BrakesControl attivo su {self.chip_name} con {len(self.motor_pins)} motori")

    # -------------------------
    # Parsing della motor map
    # -------------------------
    def _parse_motor_map(self, flat_list):
        if len(flat_list) % 2 != 0:
            self.get_logger().error("Motor map non valida! Lunghezza dispari.")
            return {}
        return {int(flat_list[i]): int(flat_list[i+1]) for i in range(0, len(flat_list), 2)}

    # -------------------------
    # Aggiornamento parametri runtime
    # -------------------------
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'lock_time' and param.type_ == Parameter.Type.DOUBLE:
                self.t_post_cycle_lock = param.value
                self.get_logger().info(f"Nuovo lock_time: {self.t_post_cycle_lock:.2f}s")
        return SetParametersResult(successful=True)

    # -------------------------
    # Inizializzazione GPIO
    # -------------------------
    def _init_gpio(self):
        try:
            self.chip = gpiod.Chip(self.chip_name)
            for motor_id, pin in self.motor_pins.items():
                line = self.chip.get_line(pin)
                line.request(
                    consumer=f'brakes_ctrl_{motor_id}',
                    type=gpiod.LINE_REQ_DIR_OUT,
                    default_vals=[0]
                )
                self.lines[motor_id] = line
                self.get_logger().info(f"GPIO Motore {motor_id} pronto (Pin {pin})")
        except Exception as e:
            self.get_logger().error(f"Errore hardware critico: {e}")
            raise SystemExit("Hardware non rilevato.")

    # -------------------------
    # Publisher dello stato
    # -------------------------
    def publish_status(self):
        with self.state_lock:
            msg = String()
            msg.data = json.dumps(self.motor_state_flags)
            self.state_publisher.publish(msg)

    # -------------------------
    # Listener comandi motori
    # -------------------------
    def listener_callback(self, msg: Float64MultiArray):
        data = msg.data
        for idx, motor_id in enumerate(self.motor_pins.keys()):
            # Controllo dati sufficienti
            if (idx * 2 + 1) >= len(data):
                self.get_logger().warn(f"Dati insufficienti per motore {motor_id}")
                continue

            stato = int(data[idx * 2])
            t_movement = float(data[idx * 2 + 1])

            if stato == 1:
                self._start_motor_cycle(motor_id, t_movement)
            elif stato == 0:
                # Stop immediato se motore in esecuzione o locked
                with self.state_lock:
                    if self.motor_state_flags[motor_id] in ['running', 'locked']:
                        self.motor_state_flags[motor_id] = 'idle'
                        try:
                            self.lines[motor_id].set_value(0)
                        except: pass
                        self.get_logger().info(f"Motore {motor_id} fermato immediatamente")

    # -------------------------
    # Avvio ciclo motore
    # -------------------------
    def _start_motor_cycle(self, motor_id, t_movement):
        with self.state_lock:
            if self.motor_state_flags[motor_id] != 'idle':
                self.get_logger().debug(f"Motore {motor_id} occupato")
                return
            self.motor_state_flags[motor_id] = 'running'

        # Esecuzione tramite thread pool
        self.executor_pool.submit(self._motor_execution_logic, motor_id, t_movement)
        self.get_logger().info(f"Motore {motor_id} avviato per {t_movement:.2f}s")

    # -------------------------
    # Logica ciclo motore
    # -------------------------
    def _motor_execution_logic(self, motor_id, t_movement):
        line = self.lines[motor_id]
        try:
            line.set_value(1)
            time.sleep(max(0.0, t_movement))
            line.set_value(0)

            with self.state_lock:
                self.motor_state_flags[motor_id] = 'locked'
            
            time.sleep(self.t_post_cycle_lock)
        except Exception as e:
            self.get_logger().error(f"Errore Motore {motor_id}: {e}")
        finally:
            self._safe_release_line(motor_id)
            self.get_logger().info(f"Motore {motor_id} ciclo completato")

    # -------------------------
    # Safety GPIO
    # -------------------------
    def _safe_release_line(self, motor_id):
        try:
            self.lines[motor_id].set_value(0)
        except:
            pass
        with self.state_lock:
            self.motor_state_flags[motor_id] = 'idle'

    def _release_all(self):
        self.get_logger().info("Spegnimento motori e rilascio GPIO...")
        for motor_id in self.lines:
            self._safe_release_line(motor_id)
            try:
                self.lines[motor_id].release()
            except:
                pass
        if hasattr(self, 'chip'):
            self.chip.close()

    def destroy_node(self):
        self._release_all()
        self.executor_pool.shutdown(wait=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BrakesControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Stop richiesto dall'utente.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

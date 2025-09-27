import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import gpiod
import threading
import time

class BrakesControl(Node):
    def __init__(self):
        super().__init__('brakes_control')

        self.get_logger().info("Inizializzazione nodo BrakesControl...")

        try:
            self.chip = gpiod.Chip('gpiochip4')
            self.get_logger().info(f"Accesso al chip GPIO: {self.chip.name} ({self.chip.label})")
        except Exception as e:
            self.get_logger().error(f"Impossibile accedere al chip GPIO 'gpiochip4': {e}. Assicurati che il chip sia corretto e i permessi siano impostati.")
            raise SystemExit("Errore di inizializzazione GPIO.")

        self.motor_pins = {
            1: (17, 18),
            2: (16, 26),
            3: (12, 13)
        }

        self.lines = {}
        for motor_id, (pin_in1, pin_in2) in self.motor_pins.items():
            try:
                line_in1 = self.chip.get_line(pin_in1)
                line_in2 = self.chip.get_line(pin_in2)
                
                line_in1.request(consumer=f'brakes_control_motor_{motor_id}_in1', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
                line_in2.request(consumer=f'brakes_control_motor_{motor_id}_in2', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
                
                self.lines[motor_id] = (line_in1, line_in2)
                self.get_logger().info(f"Pin Motore {motor_id} ({pin_in1}, {pin_in2}) inizializzati con successo.")
            except Exception as e:
                self.get_logger().error(f"Errore nell'inizializzazione dei pin per Motore {motor_id}: {e}")
                self._release_all_gpio_lines() 
                raise SystemExit(f"Errore di inizializzazione dei pin per Motore {motor_id}.")

        # <<< MODIFICA: Ora definiamo un tempo di pausa fisso, il tempo di movimento arriverà dal topic.
        self.t_pause_cycle = 5.0  # Breve pausa (in sec) tra il movimento di andata e ritorno
        self.t_post_cycle_lock = 5.0 # Tempo di blocco dopo ogni ciclo completo

        self.motor_threads = {}
        self.motor_state_flags = {} 
        
        for motor_id in self.motor_pins.keys():
            self.motor_state_flags[motor_id] = 'idle'
            self.get_logger().info(f"Stato iniziale Motore {motor_id}: {self.motor_state_flags[motor_id]}.")

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/brakes',
            self.listener_callback,
            10
        )
        self.get_logger().info("Sottoscritto al topic '/brakes'")

    def listener_callback(self, msg: Float64MultiArray):
        """
        Callback per i messaggi ROS 2.
        # <<< MODIFICA: Aggiornata la descrizione del messaggio.
        Il messaggio Float64MultiArray.data dovrebbe contenere 6 valori:
        [stato_motore_1, t_movimento_motore_1, stato_motore_2, t_movimento_motore_2, ...]
        stato: 1 per avviare ciclo (se motore in 'idle')
        t_movimento: tempo di movimento in secondi (float)
        """
        self.get_logger().info(f"Ricevuto messaggio: {msg.data}")

        if len(msg.data) < 6:
            self.get_logger().warn(f"Dati del messaggio Float64MultiArray non validi. Ignoro.")
            return

        for i in range(3):
            motor_id = i + 1
            stato_index = i * 2
            t_movement_index = i * 2 + 1 # <<< MODIFICA: L'indice ora si riferisce al tempo di movimento

            try:
                stato = int(msg.data[stato_index])
                # <<< MODIFICA: La variabile ora si chiama t_movement per chiarezza
                t_movement = float(msg.data[t_movement_index])
            except (ValueError, IndexError):
                self.get_logger().warn(f"Valori malformati per motore {motor_id}. Ignoro.")
                continue

            if motor_id not in self.motor_pins:
                self.get_logger().warn(f"Motore {motor_id} non definito nel nodo.")
                continue
                
            if stato == 1:
                if self.motor_state_flags.get(motor_id) == 'idle':
                    self.motor_state_flags[motor_id] = 'running'
                    # <<< MODIFICA: Passiamo t_movement al thread
                    t = threading.Thread(target=self.motor_cycle_single, args=(motor_id, t_movement), daemon=True)
                    self.motor_threads[motor_id] = t
                    t.start()
                    self.get_logger().info(f"Motore {motor_id}: ciclo avviato con t_movement={t_movement:.2f}s.")
                else:
                    self.get_logger().warn(f"Motore {motor_id}: Impossibile avviare, stato attuale '{self.motor_state_flags.get(motor_id)}'.")
            elif stato == 0:
                self.get_logger().info(f"Motore {motor_id}: ricevuto comando di stop (0), nessuna azione implementata.")
            else:
                self.get_logger().warn(f"Stato non valido '{stato}' per motore {motor_id}.")

    # <<< MODIFICA: La funzione ora accetta t_movement come argomento
    def motor_cycle_single(self, motor_id: int, t_movement: float):
        """
        Esegue un ciclo di movimento usando il tempo specificato dal messaggio.
        """
        if motor_id not in self.lines:
            self.get_logger().error(f"Motore {motor_id} non inizializzato.")
            self.motor_state_flags[motor_id] = 'idle'
            return

        line_in1, line_in2 = self.lines[motor_id]
        self.get_logger().info(f"Motore {motor_id}: Inizio ciclo.")

        try:
            # 1. Movimento Orario (usando il tempo dal messaggio)
            # <<< MODIFICA: Usa t_movement ricevuto come parametro
            self.get_logger().debug(f"Motore {motor_id}: Movimento Orario per {t_movement:.2f}s.")
            line_in1.set_value(1)
            line_in2.set_value(0)
            time.sleep(t_movement)

            # 2. Pausa fissa tra i movimenti
            # <<< MODIFICA: Usa la variabile di classe fissa per la pausa
            self.get_logger().debug(f"Motore {motor_id}: Pausa per {self.t_pause_cycle:.2f}s.")
            line_in1.set_value(0)
            line_in2.set_value(0)
            time.sleep(self.t_pause_cycle)

            # 3. Movimento Antiorario (usando il tempo dal messaggio)
            # <<< MODIFICA: Usa t_movement ricevuto come parametro
            self.get_logger().debug(f"Motore {motor_id}: Movimento Antiorario per {t_movement:.2f}s.")
            line_in1.set_value(0)
            line_in2.set_value(1)
            time.sleep(t_movement)

            self.stop_motor(motor_id)
            self.get_logger().info(f"Motore {motor_id}: Ciclo completato.")

            # 4. Periodo di blocco post-ciclo
            self.motor_state_flags[motor_id] = 'locked'
            self.get_logger().info(f"Motore {motor_id}: In blocco per {self.t_post_cycle_lock:.2f}s.")
            time.sleep(self.t_post_cycle_lock)
            self.motor_state_flags[motor_id] = 'idle'
            self.get_logger().info(f"Motore {motor_id}: Blocco terminato. Pronto.")

        except Exception as e:
            self.get_logger().error(f"Errore ciclo per Motore {motor_id}: {e}")
            self.motor_state_flags[motor_id] = 'idle'
        finally:
            self.stop_motor(motor_id)
            if motor_id in self.motor_threads:
                del self.motor_threads[motor_id]

    def stop_motor(self, motor_id):
        if motor_id in self.lines:
            line_in1, line_in2 = self.lines[motor_id]
            line_in1.set_value(0)
            line_in2.set_value(0)
            self.get_logger().debug(f"Motore {motor_id}: pin resettati a 0.")
        else:
            self.get_logger().warn(f"Tentativo di fermare un motore non valido: {motor_id}")

    def _release_all_gpio_lines(self):
        self.get_logger().info("Rilascio tutte le linee GPIO.")
        for motor_id, (line1, line2) in self.lines.items():
            try:
                line1.set_value(0)
                line2.set_value(0)
                line1.release()
                line2.release()
            except Exception as e:
                self.get_logger().error(f"Errore rilascio linee per motore {motor_id}: {e}")
        self.lines.clear()

    def destroy_node(self):
        self.get_logger().info("Distruzione nodo BrakesControl.")
        self._release_all_gpio_lines()
        if hasattr(self, 'chip') and self.chip:
            self.chip.close()
            self.get_logger().info("Chip GPIO rilasciato.")
        super().destroy_node()
        self.get_logger().info("Nodo BrakesControl distrutto.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BrakesControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Interruzione da tastiera. Spegnimento...")
    except SystemExit as e:
        print(f"Errore critico durante l'inizializzazione: {e}")
        if node:
            node.get_logger().error(f"Terminazione per errore critico: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

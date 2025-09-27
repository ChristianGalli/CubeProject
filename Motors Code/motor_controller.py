# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
# Importiamo Float64MultiArray invece di String
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import serial

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Seriale su Arduino Nano
        # Assicurati che il percorso della porta seriale sia corretto per il tuo sistema
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Seriale aperta su /dev/ttyAMC0 a 115200 baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Errore nell'apertura della porta seriale: {e}. Assicurati che l'Arduino sia collegato e che tu abbia i permessi.")
            # Potresti voler uscire o gestire l'errore in modo più robusto qui
            self.ser = None # Imposta a None per evitare errori se la seriale non si apre

        # Sottoscrizione al topic, ora con tipo Float64MultiArray
        self.subscription = self.create_subscription(
            Float64MultiArray, # Cambiato da String a Float64MultiArray
            '/flywheels',
            self.listener_callback,
            10 # Dimensione della coda (queue_size)
        )
        self.get_logger().info("Sottoscritto al topic '/flywheels' con tipo Float64MultiArray")

    def listener_callback(self, msg):
        # Il messaggio Float64MultiArray ha un campo 'data' che è una lista di float
        # Ci aspettiamo che il vettore contenga almeno due elementi: [direzione, velocita]
        if len(msg.data) >= 6:
            direzione1 = msg.data[0]
            velocita1 = msg.data[1]
            direzione2 = msg.data[2]
            velocita2 = msg.data[3]
            direzione3 = msg.data[4]
            velocita3 = msg.data[5]
            
            # Formattiamo i float in una stringa "direzione,velocita" per l'invio seriale
            comando_stringa = f"{direzione1},{velocita1},{direzione2},{velocita2},{direzione3},{velocita3}"
            
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write((comando_stringa + '\n').encode())
                    self.get_logger().info(f"Inviato seriale: '{comando_stringa}'")
                except serial.SerialException as e:
                    self.get_logger().error(f"Errore durante l'invio seriale: {e}")
            else:
                self.get_logger().warn("La porta seriale non e aperta o non e stata inizializzata correttamente.")
        else:
            self.get_logger().warn("Comando non valido. Il messaggio Float64MultiArray dovrebbe contenere almeno due elementi: [direzione, velocita]")

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo interrotto da tastiera.")
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
            node.get_logger().info("Porta seriale chiusa.")
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("ROS2 spento.")

if __name__ == '__main__':
    main()

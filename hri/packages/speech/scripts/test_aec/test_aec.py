#!/usr/bin/env python3
"""
Script de prueba para el sistema AEC
Verifica que todos los componentes estén funcionando correctamente.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from frida_interfaces.msg import AudioData
from frida_interfaces.srv import Speak
import time
import threading


class AECTester(Node):
    def __init__(self):
        super().__init__("aec_tester")

        # Subscribers para monitorear el sistema
        self.create_subscription(
            AudioData, "/rawAudioChunk", self.raw_audio_callback, 10
        )
        self.create_subscription(
            AudioData, "/cleanAudioChunk", self.clean_audio_callback, 10
        )
        self.create_subscription(
            AudioData, "/farAudioChunk", self.far_audio_callback, 10
        )
        self.create_subscription(String, "/speech/oww", self.wakeword_callback, 10)
        self.create_subscription(
            String, "/speech/interrupt", self.interrupt_callback, 10
        )
        self.create_subscription(Bool, "/saying", self.saying_callback, 10)

        # Cliente para el servicio de speech
        self.speak_client = self.create_client(Speak, "/speech/speak")

        # Contadores para verificar flujo de datos
        self.raw_audio_count = 0
        self.clean_audio_count = 0
        self.far_audio_count = 0
        self.wakeword_count = 0
        self.interrupt_count = 0

        # Estado del sistema
        self.is_speaking = False

        # Timer para reportes periódicos
        self.create_timer(5.0, self.report_status)

        self.get_logger().info("AEC Tester iniciado. Monitoreando sistema...")

    def raw_audio_callback(self, msg):
        self.raw_audio_count += 1

    def clean_audio_callback(self, msg):
        self.clean_audio_count += 1

    def far_audio_callback(self, msg):
        self.far_audio_count += 1

    def wakeword_callback(self, msg):
        self.wakeword_count += 1
        self.get_logger().info(f"Wake word detectado: {msg.data}")

    def interrupt_callback(self, msg):
        self.interrupt_count += 1
        self.get_logger().info(f"Interrupción detectada: {msg.data}")

    def saying_callback(self, msg):
        self.is_speaking = msg.data
        if self.is_speaking:
            self.get_logger().info("Robot comenzó a hablar")
        else:
            self.get_logger().info("Robot terminó de hablar")

    def report_status(self):
        """Reporta el estado del sistema cada 5 segundos."""
        self.get_logger().info("=== Estado del Sistema AEC ===")
        self.get_logger().info(f"Audio crudo recibido: {self.raw_audio_count}")
        self.get_logger().info(f"Audio limpio recibido: {self.clean_audio_count}")
        self.get_logger().info(f"Audio de referencia recibido: {self.far_audio_count}")
        self.get_logger().info(f"Wake words detectados: {self.wakeword_count}")
        self.get_logger().info(f"Interrupciones detectadas: {self.interrupt_count}")
        self.get_logger().info(
            f"Estado del speaker: {'Hablando' if self.is_speaking else 'Silencio'}"
        )

        # Verificar si el sistema está funcionando
        if self.raw_audio_count == 0:
            self.get_logger().warn("⚠️  No se está recibiendo audio del micrófono")
        elif self.clean_audio_count == 0:
            self.get_logger().warn("⚠️  El nodo AEC no está funcionando")
        else:
            self.get_logger().info("✅ Sistema funcionando correctamente")

        # Reset counters
        self.raw_audio_count = 0
        self.clean_audio_count = 0
        self.far_audio_count = 0

    def test_speech(self):
        """Prueba el sistema de speech."""
        if not self.speak_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Servicio de speech no disponible")
            return

        request = Speak.Request()
        request.text = "Hola, estoy probando el sistema de cancelación de eco. Por favor, di 'frida' para interrumpirme."

        future = self.speak_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info("✅ Prueba de speech exitosa")
        else:
            self.get_logger().error("❌ Fallo en la prueba de speech")


def main(args=None):
    rclpy.init(args=args)

    tester = AECTester()

    # Crear un thread para las pruebas
    def run_tests():
        time.sleep(10)  # Esperar a que el sistema se estabilice
        tester.get_logger().info("Iniciando pruebas del sistema...")

        # Prueba 1: Verificar recepción de audio
        tester.get_logger().info("Prueba 1: Verificando recepción de audio...")
        time.sleep(5)

        # Prueba 2: Probar speech
        tester.get_logger().info("Prueba 2: Probando sistema de speech...")
        tester.test_speech()

        # Prueba 3: Esperar por wake word
        tester.get_logger().info(
            "Prueba 3: Di 'frida' para probar detección de wake word..."
        )

    test_thread = threading.Thread(target=run_tests)
    test_thread.daemon = True
    test_thread.start()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

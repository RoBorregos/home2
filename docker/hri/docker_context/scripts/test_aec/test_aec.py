# # #!/usr/bin/env python3
# # """
# # Script de prueba para el sistema AEC
# # Verifica que todos los componentes estén funcionando correctamente.
# # """

# # import rclpy
# # from rclpy.node import Node
# # from std_msgs.msg import String, Bool
# # from frida_interfaces.msg import AudioData
# # from frida_interfaces.srv import Speak
# # import time
# # import threading


# # class AECTester(Node):
# #     def __init__(self):
# #         super().__init__("aec_tester")

# #         # Subscribers para monitorear el sistema
# #         self.create_subscription(
# #             AudioData, "/rawAudioChunk", self.raw_audio_callback, 10
# #         )
# #         self.create_subscription(
# #             AudioData, "/cleanAudioChunk", self.clean_audio_callback, 10
# #         )
# #         self.create_subscription(
# #             AudioData, "/farAudioChunk", self.far_audio_callback, 10
# #         )
# #         self.create_subscription(String, "/speech/oww", self.wakeword_callback, 10)
# #         self.create_subscription(
# #             String, "/speech/interrupt", self.interrupt_callback, 10
# #         )
# #         self.create_subscription(Bool, "/saying", self.saying_callback, 10)

# #         # Cliente para el servicio de speech
# #         self.speak_client = self.create_client(Speak, "/speech/speak")

# #         # Contadores para verificar flujo de datos
# #         self.raw_audio_count = 0
# #         self.clean_audio_count = 0
# #         self.far_audio_count = 0
# #         self.wakeword_count = 0
# #         self.interrupt_count = 0

# #         # Estado del sistema
# #         self.is_speaking = False

# #         # Timer para reportes periódicos
# #         self.create_timer(5.0, self.report_status)

# #         self.get_logger().info("AEC Tester iniciado. Monitoreando sistema...")

# #     def raw_audio_callback(self, msg):
# #         self.raw_audio_count += 1

# #     def clean_audio_callback(self, msg):
# #         self.clean_audio_count += 1

# #     def far_audio_callback(self, msg):
# #         self.far_audio_count += 1

# #     def wakeword_callback(self, msg):
# #         self.wakeword_count += 1
# #         self.get_logger().info(f"Wake word detectado: {msg.data}")

# #     def interrupt_callback(self, msg):
# #         self.interrupt_count += 1
# #         self.get_logger().info(f"Interrupción detectada: {msg.data}")

# #     def saying_callback(self, msg):
# #         self.is_speaking = msg.data
# #         if self.is_speaking:
# #             self.get_logger().info("Robot comenzó a hablar")
# #         else:
# #             self.get_logger().info("Robot terminó de hablar")

# #     def report_status(self):
# #         """Reporta el estado del sistema cada 5 segundos."""
# #         self.get_logger().info("=== Estado del Sistema AEC ===")
# #         self.get_logger().info(f"Audio crudo recibido: {self.raw_audio_count}")
# #         self.get_logger().info(f"Audio limpio recibido: {self.clean_audio_count}")
# #         self.get_logger().info(f"Audio de referencia recibido: {self.far_audio_count}")
# #         self.get_logger().info(f"Wake words detectados: {self.wakeword_count}")
# #         self.get_logger().info(f"Interrupciones detectadas: {self.interrupt_count}")
# #         self.get_logger().info(
# #             f"Estado del speaker: {'Hablando' if self.is_speaking else 'Silencio'}"
# #         )

# #         # Verificar si el sistema está funcionando
# #         if self.raw_audio_count == 0:
# #             self.get_logger().warn("⚠️  No se está recibiendo audio del micrófono")
# #         elif self.clean_audio_count == 0:
# #             self.get_logger().warn("⚠️  El nodo AEC no está funcionando")
# #         else:
# #             self.get_logger().info("✅ Sistema funcionando correctamente")

# #         # Reset counters
# #         self.raw_audio_count = 0
# #         self.clean_audio_count = 0
# #         self.far_audio_count = 0

# #     def test_speech(self):
# #         """Prueba el sistema de speech."""
# #         if not self.speak_client.wait_for_service(timeout_sec=5.0):
# #             self.get_logger().error("Servicio de speech no disponible")
# #             return

# #         request = Speak.Request()
# #         request.text = "Hola, estoy probando el sistema de cancelación de eco. Por favor, di 'frida' para interrumpirme."

# #         future = self.speak_client.call_async(request)
# #         rclpy.spin_until_future_complete(self, future)

# #         if future.result().success:
# #             self.get_logger().info("✅ Prueba de speech exitosa")
# #         else:
# #             self.get_logger().error("❌ Fallo en la prueba de speech")


# # def main(args=None):
# #     rclpy.init(args=args)

# #     tester = AECTester()

# #     # Crear un thread para las pruebas
# #     def run_tests():
# #         time.sleep(10)  # Esperar a que el sistema se estabilice
# #         tester.get_logger().info("Iniciando pruebas del sistema...")

# #         # Prueba 1: Verificar recepción de audio
# #         tester.get_logger().info("Prueba 1: Verificando recepción de audio...")
# #         time.sleep(5)

# #         # Prueba 2: Probar speech
# #         tester.get_logger().info("Prueba 2: Probando sistema de speech...")
# #         tester.test_speech()

# #         # Prueba 3: Esperar por wake word
# #         tester.get_logger().info(
# #             "Prueba 3: Di 'frida' para probar detección de wake word..."
# #         )

# #     test_thread = threading.Thread(target=run_tests)
# #     test_thread.daemon = True
# #     test_thread.start()

# #     try:
# #         rclpy.spin(tester)
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         tester.destroy_node()
# #         rclpy.shutdown()


# # if __name__ == "__main__":
# #     main()
# import wave
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from frida_interfaces.msg import AudioData
# from aec import SpeexAEC


# class AECTester(Node):
#     def __init__(self):
#         super().__init__("aec_tester")

#         # Publicador para enviar audio limpio
#         self.clean_audio_publisher = self.create_publisher(AudioData, "/cleanAudioChunk", 10)

#         # Inicializar el sistema AEC
#         self.get_logger().info("Iniciando pruebas del sistema AEC...")

#         # Procesar los archivos WAV
#         self.process_audio_files()

#     def process_audio_files(self):
#         """Procesa los archivos de audio y aplica AEC."""
#         try:
#             # Cargar los archivos WAV
#             voice_data, sample_rate = self.load_wav_file("voice.wav")
#             music_data, _ = self.load_wav_file("music.wav")

#             # Asegurarse de que ambos archivos tengan la misma longitud
#             max_length = max(len(voice_data), len(music_data))
#             if len(voice_data) < max_length:
#                 voice_data = np.pad(voice_data, (0, max_length - len(voice_data)), mode="constant")
#             if len(music_data) < max_length:
#                 music_data = np.pad(music_data, (0, max_length - len(music_data)), mode="constant")


#             # Dividir los datos en frames
#             frame_size = 512
#             filter_length = 1600  # Ajusta según sea necesario
#             aec = SpeexAEC(frame_size, filter_length, sample_rate)


#             processed_audio = []
#             for i in range(0, len(voice_data), frame_size):
#                 near_frame = voice_data[i : i + frame_size]
#                 far_frame = music_data[i : i + frame_size]

#                 clean_frame = aec.process_frame(near_frame, far_frame)
#                 processed_audio.extend(clean_frame)

#                 # Publicar los datos procesados
#                 self.publish_clean_audio(near_frame, far_frame)
#             processed_audio = np.array(processed_audio, dtype=np.int16)
#             self.save_wav_file("/workspace/cleaned_audio.wav", processed_audio, sample_rate)

#             self.get_logger().info("✅ Pruebas de AEC completadas exitosamente.")

#         except Exception as e:
#             self.get_logger().error(f"Error al procesar los archivos de audio: {e}")

#     def load_wav_file(self, file_path):
#         """Carga un archivo WAV y devuelve los datos como un array numpy."""
#         with wave.open(file_path, "rb") as wav_file:
#             sample_rate = wav_file.getframerate()
#             num_frames = wav_file.getnframes()
#             audio_data = wav_file.readframes(num_frames)
#             audio_array = np.frombuffer(audio_data, dtype=np.int16)
#         return audio_array, sample_rate

#     def publish_clean_audio(self, near_frame, far_frame):
#         """Publica los datos de audio limpio procesados."""
#         # Simular procesamiento AEC (esto debería conectarse al nodo AEC real)
#         clean_frame = near_frame - far_frame  # Ejemplo simple de cancelación

#         # Crear el mensaje de audio limpio
#         clean_audio_msg = AudioData(data=clean_frame.tobytes())
#         self.clean_audio_publisher.publish(clean_audio_msg)

#     def save_wav_file(self, file_path, audio_data, sample_rate):
#         """Guarda los datos de audio en un archivo WAV."""
#         self.get_logger().info(f"Guardando archivo WAV en {file_path}...")
#         with wave.open(file_path, "wb") as wav_file:
#             wav_file.setnchannels(1)  # Mono
#             wav_file.setsampwidth(2)  # 16 bits = 2 bytes
#             wav_file.setframerate(sample_rate)
#             wav_file.writeframes(audio_data.tobytes())
#         self.get_logger().info(f"Archivo WAV guardado exitosamente en {file_path}.")


# def main(args=None):
#     rclpy.init(args=args)
#     tester = AECTester()

#     try:
#         rclpy.spin(tester)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         tester.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
import os
import wave
import numpy as np
import rclpy
from rclpy.node import Node
from frida_interfaces.msg import AudioData
from aec import SpeexAEC  # Importar la clase SpeexAEC desde aec.py
from scipy.signal import resample


class AECTester(Node):
    def __init__(self):
        super().__init__("aec_tester")

        # Publicador para enviar audio limpio
        self.clean_audio_publisher = self.create_publisher(AudioData, "/cleanAudioChunk", 10)

        # Inicializar el sistema AEC
        self.get_logger().info("Iniciando pruebas del sistema AEC...")

        # Procesar los archivos WAV
        self.process_audio_files()

    def process_audio_files(self):
        """Procesa los archivos de audio y aplica AEC."""
        try:
            # Combinar voz y música para simular eco
            self.combine_audio_files("voice.wav", "music.wav", "combined.wav")

            # Cargar los archivos WAV
            near_data, sample_rate = self.load_wav_file("combined.wav")  # Audio combinado
            far_data, _ = self.load_wav_file("music.wav")  # Audio de referencia (música)

            # Asegurarse de que ambos archivos tengan la misma longitud
            max_length = max(len(near_data), len(far_data))
            if len(near_data) < max_length:
                near_data = np.pad(near_data, (0, max_length - len(near_data)), mode="constant")
            if len(far_data) < max_length:
                far_data = np.pad(far_data, (0, max_length - len(far_data)), mode="constant")

            # Inicializar SpeexDSP
            frame_size = 512
            filter_length = 1600  # Ajusta según sea necesario
            aec = SpeexAEC(frame_size, filter_length, sample_rate)

            # Dividir los datos en frames y procesarlos
            processed_audio = []
            for i in range(0, len(near_data), frame_size):
                near_frame = near_data[i : i + frame_size]
                far_frame = far_data[i : i + frame_size]

                # Procesar el frame usando SpeexDSP
                clean_frame = aec.process_frame(near_frame, far_frame)
                processed_audio.extend(clean_frame)

                # Publicar los datos procesados
                self.publish_clean_audio(near_frame, far_frame)

            # Guardar el audio procesado en un archivo WAV
            processed_audio = np.array(processed_audio, dtype=np.int16)
            self.save_wav_file("/workspace/cleaned_audio.wav", processed_audio, sample_rate)

            self.get_logger().info("✅ Pruebas de AEC completadas exitosamente. Archivo guardado en /workspace/cleaned_audio.wav")

        except Exception as e:
            self.get_logger().error(f"Error al procesar los archivos de audio: {e}")

    def combine_audio_files(self, voice_file, music_file, output_file, voice_weight=0.8, music_weight=0.2):
        """Combina voz y música en un solo archivo para simular eco."""
        try:
            with wave.open(voice_file, "rb") as vf, wave.open(music_file, "rb") as mf:
                voice_rate = vf.getframerate()
                music_rate = mf.getframerate()
                voice_channels = vf.getnchannels()
                music_channels = mf.getnchannels()
                assert vf.getsampwidth() == mf.getsampwidth(), "Sample widths must match"

                sample_rate = max(voice_rate, music_rate)  # Usar la mayor frecuencia de muestreo
                voice_frames = vf.readframes(vf.getnframes())
                music_frames = mf.readframes(mf.getnframes())

                # Convertir a numpy arrays
                voice_data = np.frombuffer(voice_frames, dtype=np.int16)
                music_data = np.frombuffer(music_frames, dtype=np.int16)

                # Convertir ambos a mono si es necesario
                voice_data = self.convert_to_mono(voice_data, voice_channels)
                music_data = self.convert_to_mono(music_data, music_channels)

                # Resamplear si es necesario
                voice_data = self.resample_audio(voice_data, voice_rate, sample_rate)
                music_data = self.resample_audio(music_data, music_rate, sample_rate)

                # Asegurarse de que ambos tengan la misma longitud
                min_length = min(len(voice_data), len(music_data))
                voice_data = voice_data[:min_length]
                music_data = music_data[:min_length]

                # Combinar voz y música
                combined_data = (voice_weight * voice_data + music_weight * music_data).astype(np.int16)

                # Guardar el archivo combinado
                with wave.open(output_file, "wb") as out:
                    out.setnchannels(1)  # Guardar como mono
                    out.setsampwidth(vf.getsampwidth())
                    out.setframerate(sample_rate)
                    out.writeframes(combined_data.tobytes())

            self.get_logger().info(f"Archivo combinado guardado en {output_file}")

        except Exception as e:
            self.get_logger().error(f"Error al combinar los archivos de audio: {e}")

    def convert_to_mono(self, audio_data, num_channels):
        """Convierte datos de audio a mono si tienen más de un canal."""
        if num_channels == 1:
            return audio_data
        # Si es estéreo, promediar los canales izquierdo y derecho
        audio_data = audio_data.reshape(-1, num_channels)
        mono_data = audio_data.mean(axis=1).astype(np.int16)
        return mono_data

    def resample_audio(self, audio_data, original_rate, target_rate):
        """Resamplea los datos de audio a la frecuencia de muestreo objetivo."""
        if original_rate == target_rate:
            return audio_data
        num_samples = int(len(audio_data) * target_rate / original_rate)
        return resample(audio_data, num_samples).astype(np.int16)

    def load_wav_file(self, file_path):
        """Carga un archivo WAV y devuelve los datos como un array numpy."""
        with wave.open(file_path, "rb") as wav_file:
            sample_rate = wav_file.getframerate()
            num_frames = wav_file.getnframes()
            audio_data = wav_file.readframes(num_frames)
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
        return audio_array, sample_rate

    def publish_clean_audio(self, near_frame, far_frame):
        """Publica los datos de audio limpio procesados."""
        clean_frame = near_frame - far_frame  # Esto se reemplaza con SpeexDSP
        clean_audio_msg = AudioData(data=clean_frame.tobytes())
        self.clean_audio_publisher.publish(clean_audio_msg)

    def save_wav_file(self, file_path, audio_data, sample_rate):
        """Guarda los datos de audio en un archivo WAV."""
        self.get_logger().info(f"Guardando archivo WAV en {file_path}...")
        with wave.open(file_path, "wb") as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16 bits = 2 bytes
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(audio_data.tobytes())
        self.get_logger().info(f"Archivo WAV guardado exitosamente en {file_path}.")


def main(args=None):
    rclpy.init(args=args)
    tester = AECTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
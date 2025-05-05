# tts_server.py
import grpc
from concurrent import futures
import os
import subprocess
import tts_pb2
import tts_pb2_grpc
import time


class TTSService(tts_pb2_grpc.TTSServiceServicer):
    def Synthesize(self, request, context):
        print(f"Received request to synthesize text: {request.text}")
        try:
            model_path = (
                request.model
                if request.model.endswith(".onnx")
                else "models/en-us-danny-low.onnx"
            )
            if not os.path.exists(model_path):
                return tts_pb2.SynthesizeResponse(
                    success=False, error_message="Model not found"
                )
            print(f"Using model: {model_path}")
            output_path = "models/" + request.output_path
            cmd = [
                "piper",
                "--model",
                model_path,
                "--data-dir",
                "models",
                "--output_file",
                output_path,
                "--cuda",
            ]

            proc = subprocess.run(
                cmd,
                input=request.text.encode(),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            if proc.returncode != 0:
                return tts_pb2.SynthesizeResponse(
                    success=False, error_message=proc.stderr.decode()
                )

            # Ensure the file has been written
            timeout = 3  # seconds
            interval = 0.05  # check every 50ms
            waited = 0
            while not os.path.isfile(output_path):
                if waited >= timeout:
                    return tts_pb2.SynthesizeResponse(
                        success=False,
                        error_message="Output file not found after synthesis",
                    )
                time.sleep(interval)
                waited += interval

            return tts_pb2.SynthesizeResponse(success=True)

        except Exception as e:
            print(f"Error during TTS synthesis: {e}")
            return tts_pb2.SynthesizeResponse(success=False, error_message=str(e))


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    tts_pb2_grpc.add_TTSServiceServicer_to_server(TTSService(), server)
    server.add_insecure_port("[::]:50050")
    print("Starting TTS server on port 50050...")
    server.start()
    server.wait_for_termination()


if __name__ == "__main__":
    serve()

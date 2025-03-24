import grpc
from PIL import Image
import io
from concurrent import futures
import moondream_proto_pb2
import moondream_proto_pb2_grpc

class HelloWorldService(moondream_proto_pb2_grpc.HelloWorldServiceServicer):
    def HelloWorld(self, request, context):
        try:
            image_bytes = request.image_data
            image = Image.open(io.BytesIO(image_bytes))
            print("Received img")

            # Return a success response
            return moondream_proto_pb2.HelloWorldResponse(
                success=True,
                message="Image received and processed successfully."
            )
        except Exception as e:
            # Return an error response
            return moondream_proto_pb2.HelloWorldResponse(
                success=False,
                message=f"Error processing image: {str(e)}"
            )

# Run the gRPC server
def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    moondream_proto_pb2_grpc.add_HelloWorldServiceServicer_to_server(HelloWorldService(), server)
    server.add_insecure_port("[::]:50052")
    print("gRPC server started on port 50052...")
    server.start()
    server.wait_for_termination()

if __name__ == "__main__":
    serve()
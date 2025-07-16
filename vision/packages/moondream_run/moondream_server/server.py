import grpc
from concurrent import futures
import moondream_proto_pb2
import moondream_proto_pb2_grpc
import argparse
from moondream_lib import MoonDreamModel
from enum import Enum


class Position(Enum):
    LEFT = "left"
    CENTER = "center"
    RIGHT = "right"
    NOT_FOUND = "not found"


class MoonDreamServicer(moondream_proto_pb2_grpc.MoonDreamServiceServicer):
    def __init__(self, model):
        self.model = model

    def EncodeImage(self, request, context):
        print("Encoding image...")
        encoded_data = self.model.encode_image(request.image_data)
        return moondream_proto_pb2.EncodedImageResponse(encoded_image=encoded_data)

    def FindBeverage(self, request, context):
        position = self.model.find_beverage(request.encoded_image, request.subject)
        return moondream_proto_pb2.BeveragePositionResponse(position=position)

    def Query(self, request, context):
        print("Querying image...")
        answer = self.model.query(request.encoded_image, request.query)
        return moondream_proto_pb2.QueryResponse(answer=answer)


# Run the gRPC server
def serve(**kwargs):
    # Increase max message size for both request and response to 200MB
    options = [
        ("grpc.max_receive_message_length", 200 * 1024 * 1024),
        ("grpc.max_send_message_length", 200 * 1024 * 1024),
    ]
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10), options=options)
    md_model = MoonDreamModel(**kwargs)
    moondream_proto_pb2_grpc.add_MoonDreamServiceServicer_to_server(
        MoonDreamServicer(md_model), server
    )
    server.add_insecure_port("[::]:50052")
    print("gRPC server started on port 50052...")
    server.start()
    server.wait_for_termination()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MoonDream Server")
    parser.add_argument(
        "--model_name",
        type=str,
        default="vikhyatk/moondream2",
        help="Name of the MoonDream model to load from Hugging Face Hub",
    )
    parser.add_argument(
        "--revision",
        type=str,
        default="2025-01-09",
        help="Revision of the MoonDream model to load from Hugging Face Hub",
    )

    parser.add_argument(
        "--device_map",
        type=str,
        default="jetson",
        help="Device map for loading the model (e.g., 'jetson', 'cpu', 'cuda')",
    )
    args = parser.parse_args()

    # oddly specific but this is how it works lol
    if args.device_map == "jetson":
        args.device_map = {"": 0}
    elif args.device_map == "cpu":
        args.device_map = None
    elif args.device_map == "cuda":
        args.device_map = {"": "cuda"}

    serve(
        model_name=args.model_name, revision=args.revision, device_map=args.device_map
    )

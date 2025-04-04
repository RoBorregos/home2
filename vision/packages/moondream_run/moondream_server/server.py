import grpc
from concurrent import futures
import moondream_proto_pb2
import moondream_proto_pb2_grpc
from PIL import Image
import io
from transformers import AutoModelForCausalLM

# import moondream as md
import pickle
from enum import Enum
import argparse


class Position(Enum):
    LEFT = "left"
    CENTER = "center"
    RIGHT = "right"
    NOT_FOUND = "not found"


class MoonDreamModel:
    def __init__(
        self,
        model_name="vikhyatk/moondream2",
        revision="2025-01-09",
        device_map={"": 0},
        **kwargs,
    ):
        self.model = AutoModelForCausalLM.from_pretrained(
            model_name,
            revision=revision,
            trust_remote_code=True,
            device_map=device_map,
            **kwargs,
        )

    def encode_image(self, image_data):
        image = Image.open(io.BytesIO(image_data))

        with io.BytesIO() as img_io:
            image.save(img_io, format="JPEG", quality=90)
            img_bytes = img_io.getvalue()

        encoded = Image.open(io.BytesIO(img_bytes))
        # encoded = self.model.encode_image(Image.open(io.BytesIO(img_bytes))) # dont remove this in case of interoperability
        return pickle.dumps(encoded)

    def find_beverage(self, encoded_image_data, subject):
        encoded_image = pickle.loads(encoded_image_data)
        detect_result = self.model.detect(encoded_image, subject)

        if not detect_result["objects"]:
            return Position.NOT_FOUND.value
        else:
            for obj in detect_result["objects"]:
                x_center = (obj["x_min"] + obj["x_max"]) / 2
                if x_center < 1 / 3:
                    return Position.LEFT.value
                elif x_center > 2 / 3:
                    return Position.RIGHT.value
                else:
                    return Position.CENTER.value
            return Position.NOT_FOUND.value

    def generate_description(self, encoded_image_data, query):
        encoded_image = pickle.loads(encoded_image_data)
        answer = self.model.query(encoded_image, query)["answer"]
        return answer


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

    def GeneratePersonDescription(self, request, context):
        print("Generating description...")
        answer = self.model.generate_description(request.encoded_image, request.query)
        return moondream_proto_pb2.DescriptionResponse(answer=answer)


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

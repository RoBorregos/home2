from transformers import AutoModelForCausalLM
from PIL import Image
import io
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
                print(x_center)
                if x_center <= 0.4:
                    return Position.LEFT.value
                elif x_center >= 0.6:
                    return Position.RIGHT.value
                else:
                    return Position.CENTER.value
            return Position.NOT_FOUND.value

    def query(self, encoded_image_data, query):
        encoded_image = pickle.loads(encoded_image_data)
        answer = self.model.query(encoded_image, query)["answer"]
        return answer


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

    model = MoonDreamModel(
        model_name=args.model_name, revision=args.revision, device_map=args.device_map
    )
    image = image = Image.open("test_img.png")

    if image.mode == "RGBA":
        image = image.convert("RGB")

    buffer = io.BytesIO()
    image.save(buffer, format="JPEG")
    image_bytes = buffer.getvalue()
    encoded_image = model.encode_image(image_bytes)
    test_query = "Describe the person"
    result = model.query(encoded_image, test_query)
    print(f"Query result: {result}")

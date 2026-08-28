from transformers import AutoModelForCausalLM
from PIL import Image
import io
import pickle
import argparse

NOT_FOUND = "not found"

order_labels = [
    "first",
    "second",
    "third",
    "fourth",
    "fifth",
    "sixth",
    "seventh",
    "eighth",
]


# class Position(Enum):
#     LEFT = "left"
#     CENTER = "center"
#     RIGHT = "right"
#     NOT_FOUND = "not found"


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

    def query(self, encoded_image_data, query):
        encoded_image = pickle.loads(encoded_image_data)
        answer = self.model.query(encoded_image, query)["answer"]
        return answer

    def detect(self, encoded_image_data, subject):
        """Open-vocabulary detection: normalized bboxes for `subject`."""
        encoded_image = pickle.loads(encoded_image_data)
        detections = self.model.detect(encoded_image, subject)

        objects = (
            detections["objects"]
            if isinstance(detections, dict) and "objects" in detections
            else detections
        )

        objects_out = []
        for obj in objects or []:
            objects_out.append(
                {
                    "x_min": float(obj.get("x_min", obj.get("xmin", 0.0))),
                    "y_min": float(obj.get("y_min", obj.get("ymin", 0.0))),
                    "x_max": float(obj.get("x_max", obj.get("xmax", 0.0))),
                    "y_max": float(obj.get("y_max", obj.get("ymax", 0.0))),
                    "name": obj.get("name", obj.get("label", subject)),
                }
            )
        return objects_out

    def find_object_points(self, encoded_image_data, subject):
        encoded_image = pickle.loads(encoded_image_data)
        result = self.model.point(encoded_image, subject)

        if not result["points"]:
            return []

        points_out = []
        for pt in result["points"]:
            points_out.append(
                {
                    "x": pt["x"],
                    "y": pt["y"],
                }
            )

        return points_out


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

from transformers import AutoModelForCausalLM
from PIL import Image
import cv2
from enum import Enum


class Position(Enum):
    LEFT = "left"
    CENTER = "center"
    RIGHT = "right"
    NOT_FOUND = "not found"


class MoonDreamModel:
    def __init__(self):
        self.model = AutoModelForCausalLM.from_pretrained(
            "vikhyatk/moondream2",
            revision="2025-01-09",
            trust_remote_code=True,
            # Uncomment for GPU acceleration & pip install accelerate
            # device_map={"": "cuda"}
        )

    def encode_image(self, image):
        img = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        return self.model.encode_image(img)

    def generate_person_description(self, encoded_image, query, stream=False):
        if stream:
            print("Streaming answer:", end=" ", flush=True)
            for chunk in self.model.query(encoded_image, query, stream=True)["answer"]:
                print(chunk, end="", flush=True)
            print()
        else:
            answer = self.model.query(encoded_image, query)["answer"]
            return answer

    def find_beverage(self, encoded_image, subject):
        detect_result = self.model.detect(encoded_image, subject)

        if not detect_result["objects"]:
            return Position.NOT_FOUND
        else:
            for obj in detect_result["objects"]:
                x_center = (obj["x_min"] + obj["x_max"]) / 2
                if x_center < 1 / 3:
                    return Position.LEFT
                elif x_center > 2 / 3:
                    return Position.RIGHT
                else:
                    return Position.CENTER
            return Position.NOT_FOUND


# Test beverage location
if __name__ == "__main__":
    model_path = "vision/vision_general/scripts/moondream-2b-int8.mf.gz"
    md_model = MoonDreamModel(model_path)
    img = cv2.imread("vision/vision_general/scripts/gatorade.jpg")
    encoded_image = md_model.encode_image(img)
    md_model.find_beverage(encoded_image, "purple gatorade")

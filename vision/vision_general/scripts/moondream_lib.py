import moondream as md
from PIL import Image
import cv2
# ===== STEP 1: Install Dependencies =====
# pip install moondream  # Install dependencies in your project directory


# ===== STEP 2: Download Model =====
# Download model (593 MiB download size, 996 MiB memory usage)
# Use: wget (Linux and Mac) or curl.exe -O (Windows)
# wget https://huggingface.co/vikhyatk/moondream2/resolve/9dddae84d54db4ac56fe37817aeaeb502ed083e2/moondream-2b-int8.mf.gz


class MoonDreamModel:
    def __init__(self, model_path):
        self.model = md.vl(model=model_path)

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
            # print("Answer:", answer)
            return answer

    def find_beverage(self, encoded_image, subject):
        detect_result = self.model.detect(encoded_image, subject)

        if not detect_result["objects"]:
            return "not found"
        else:
            for obj in detect_result["objects"]:
                x_center = (obj["x_min"] + obj["x_max"]) / 2
                if x_center < 1 / 3:
                    position = "left"
                elif x_center > 2 / 3:
                    position = "right"
                else:
                    position = "center"
                return position


# Test beverage location
if __name__ == "__main__":
    model_path = "vision/vision_general/scripts/moondream-2b-int8.mf.gz"
    md_model = MoonDreamModel(model_path)
    img = cv2.imread("vision/vision_general/scripts/gatorade.jpg")
    encoded_image = md_model.encode_image(img)
    md_model.find_beverage(encoded_image, "purple gatorade")

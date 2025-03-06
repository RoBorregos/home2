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
        print("Detected:", detect_result["objects"])
        if detect_result["objects"]:
            obj = detect_result["objects"][0]  # Consider the first detected object
            xmin, ymin, xmax, ymax = (
                obj["x_min"],
                obj["y_min"],
                obj["x_max"],
                obj["y_max"],
            )
            print(f"xmin: {xmin}, ymin: {ymin}, xmax: {xmax}, ymax: {ymax}")
            return int(xmin), int(ymin), int(xmax), int(ymax)
        return None, None, None, None

    def determine_position(self, image, xmin, ymin, xmax, ymax):
        image_height, image_width, _ = image.shape
        bbox_center_x = (xmin + xmax) / 2
        bbox_center_y = (ymin + ymax) / 2
        image_center_x = image_width / 2
        image_center_y = image_height / 2

        horizontal_position = "left" if bbox_center_x < image_center_x else "right"
        vertical_position = "up" if bbox_center_y < image_center_y else "down"

        return horizontal_position, vertical_position

import moondream as md
import cv2
# ===== STEP 1: Install Dependencies =====
# pip install moondream  # Install dependencies in your project directory


# ===== STEP 2: Download Model =====
# Download model (593 MiB download size, 996 MiB memory usage)
# Use: wget (Linux and Mac) or curl.exe -O (Windows)
# wget https://huggingface.co/vikhyatk/moondream2/resolve/9dddae84d54db4ac56fe37817aeaeb502ed083e2/moondream-0_5b-int8.mf.gz


class MoonDreamModel:
    def __init__(self, model_path):
        self.model = md.vl(model=model_path)

    def encode_image(self, image):
        return self.model.encode_image(image)

    def generate_person_description(self, encoded_image, query, stream=False):
        if stream:
            print("Streaming answer:", end=" ", flush=True)
            for chunk in self.model.query(encoded_image, query, stream=True)["answer"]:
                print(chunk, end="", flush=True)
            print()
        else:
            answer = self.model.query(encoded_image, query)["answer"]
            print("Answer:", answer)
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


# Example usage
if __name__ == "__main__":
    model_path = "vision/vision_general/scripts/moondream-2b-int8.mf.gz"
    image_path = (
        "/Users/jvelarde/Desktop/home2/vision/vision_general/scripts/beverage.jpeg"
    )
    image = cv2.imread(image_path)
    moon_dream = MoonDreamModel(model_path)
    prompt_person_desc = "Describe the clothing of the person in the image in a detailed and specific manner. Include the type of clothing, colors, patterns, and any notable accessories. Ensure that the description is clear and distinct."
    object = "fanta"
    encoded_image = moon_dream.encode_image(image_path)
    moon_dream.generate_person_description(
        encoded_image,
        "What are the people wearing on the image, mention it as people1:, people2: and so on in json format, you can choose any keys and values",
        stream=False,
    )
    xmin, ymin, xmax, ymax = moon_dream.find_beverage(encoded_image, object)
    if xmin is not None:
        cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)
        horizontal_position, vertical_position = moon_dream.determine_position(
            image, xmin, ymin, xmax, ymax
        )
        print(
            f"The beverage is located at the {horizontal_position}-{vertical_position} of the image."
        )
        cv2.imshow("Detected Object", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

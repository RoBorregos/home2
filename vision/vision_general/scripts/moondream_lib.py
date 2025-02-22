# ===== STEP 1: Install Dependencies =====
# pip install moondream  # Install dependencies in your project directory


# ===== STEP 2: Download Model =====
# Download model (593 MiB download size, 996 MiB memory usage)
# Use: wget (Linux and Mac) or curl.exe -O (Windows)
# wget https://huggingface.co/vikhyatk/moondream2/resolve/9dddae84d54db4ac56fe37817aeaeb502ed083e2/moondream-0_5b-int8.mf.gz

import moondream as md
from PIL import Image


class MoonDreamModel:
    def __init__(self, model_path):
        self.model = md.vl(model=model_path)

    def encode_image(self, image_path):
        image = Image.open(image_path)
        return self.model.encode_image(image)

    def caption_image(self, encoded_image, length="normal", stream=False):
        if stream:
            print("Streaming caption:", end=" ", flush=True)
            for chunk in self.model.caption(encoded_image, stream=True)["caption"]:
                print(chunk, end="", flush=True)
            print()
        else:
            caption = self.model.caption(encoded_image, length=length)["caption"]
            print("Caption:", caption)
            return caption

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
        return detect_result["objects"]


# Example usage
if __name__ == "__main__":
    model_path = "/Users/jvelarde/Desktop/home2/moondream-2b-int8.mf.gz"
    image_path = "vision/vision_general/scripts/person3.png"
    moon_dream = MoonDreamModel(model_path)
    prompt_person_desc = "Describe the clothing of the person in the image in a detailed and specific manner. Include the type of clothing, colors, patterns, and any notable accessories. Ensure that the description is clear and distinct."
    prompt_find_beverage = "tbd"
    encoded_image = moon_dream.encode_image(image_path)
    # moon_dream.caption_image(encoded_image)
    # moon_dream.caption_image(encoded_image, stream=True)
    # moon_dream.query_image(encoded_image, "What are the people wearing on the image, mention it as people1:, people2: and so on in json format, you can choose any keys and values")
    moon_dream.generate_person_description(
        encoded_image, prompt_person_desc, stream=True
    )
    # moon_dream.find_beverage(encoded_image, "subject")

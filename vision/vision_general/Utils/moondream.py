# ===== STEP 1: Install Dependencies =====
# pip install moondream  # Install dependencies in your project directory

# ===== STEP 2: Download Model =====
# Download model (593 MiB download size, 996 MiB memory usage)
# Use: wget (Linux and Mac) or curl.exe -O (Windows)
# wget https://huggingface.co/vikhyatk/moondream2/resolve/9dddae84d54db4ac56fe37817aeaeb502ed083e2/moondream-0_5b-int8.mf.gz

import moondream as md
from PIL import Image

def load_model(model_path):
    '''
    Function to load model, this should not be called more than 2 times to improve performance
    '''
    try:
        model = md.vl(model=model_path)
        print("Model loaded successfully")
        return model
    except Exception as e:
        print(f"Error loading model: {e}")
        exit(1)

def load_image(image_path):
    '''
        Load image to model
    '''
    try:
        image = Image.open(image_path)
        print("Image loaded successfully")
        return image
    except Exception as e:
        print(f"Error loading image: {e}")
        exit(1)

def encode_image(model, image):
    '''
    Encode image, this should only be done once and then querying is possible
    '''
    try:
        encoded_image = model.encode_image(image)
        print("Image encoded successfully")
        return encoded_image
    except Exception as e:
        print(f"Error encoding image: {e}")
        exit(1)

def generate_caption(model, encoded_image):
    '''
    Used for general description
    '''
    try:
        caption = model.caption(encoded_image)["caption"]
        print("Caption:", caption)
    except Exception as e:
        print(f"Error generating caption: {e}")

def stream_caption(model, encoded_image):
    print("Streaming caption:", end=" ", flush=True)
    try:
        for chunk in model.caption(encoded_image, stream=True)["caption"]:
            print(chunk, end="", flush=True)
    except Exception as e:
        print(f"Error streaming caption: {e}")

def query_image(model, encoded_image, query):
    '''
    Used to prompt the image
    '''
    try:
        answer = model.query(encoded_image, query)["answer"]
        print("\nAnswer:", answer)
    except Exception as e:
        print(f"Error querying image: {e}")

def stream_query(model, encoded_image, query):
    print("Streaming answer:", end=" ", flush=True)
    try:
        for chunk in model.query(encoded_image, query, stream=True)["answer"]:
            print(chunk, end="", flush=True)
    except Exception as e:
        print(f"Error streaming answer: {e}")

def detect_objects(model, encoded_image, subject):
    '''
    Detect opbject and obtain bounding box (x, y)
    '''
    try:
        detect_result = model.detect(encoded_image, subject)
        print("\nDetected:", detect_result["objects"])
    except Exception as e:
        print(f"Error detecting objects: {e}")

def main():
    model_path = './moondream-0_5b-int8.mf.gz'
    image_path = 'people.jpg'
    query_text = "Describe people"
    subject = "apple"

    model = load_model(model_path)
    image = load_image(image_path)
    encoded_image = encode_image(model, image)

    generate_caption(model, encoded_image)
    stream_caption(model, encoded_image)
    query_image(model, encoded_image, query_text)
    stream_query(model, encoded_image, query_text)
    detect_objects(model, encoded_image, subject)

if __name__ == "__main__":
    main()
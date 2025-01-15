import moondream as md
from PIL import Image


class MoondreamWrapper:
    def __init__(self, model_path):
        """
        Initialize the handler with the model path.
        """
        self.model_path = model_path
        self.model = self._load_model()

    def _load_model(self):
        """
        Load the model (private method).
        """
        try:
            model = md.vl(model=self.model_path)
            print("Model loaded successfully")
            return model
        except Exception as e:
            print(f"Error loading model: {e}")
            exit(1)

    def load_image(self, image_path):
        """
        Load an image for processing.
        """
        try:
            image = Image.open(image_path)
            print("Image loaded successfully")
            return image
        except Exception as e:
            print(f"Error loading image: {e}")
            exit(1)

    def encode_image(self, image):
        """
        Encode the image for querying and captioning.
        """
        try:
            encoded_image = self.model.encode_image(image)
            print("Image encoded successfully")
            return encoded_image
        except Exception as e:
            print(f"Error encoding image: {e}")
            exit(1)

    def generate_caption(self, encoded_image):
        """
        Generate a general description of the image.
        """
        try:
            caption = self.model.caption(encoded_image)["caption"]
            print("Caption:", caption)
        except Exception as e:
            print(f"Error generating caption: {e}")

    def stream_caption(self, encoded_image):
        """
        Stream a caption of the image in chunks.
        """
        print("Streaming caption:", end=" ", flush=True)
        try:
            for chunk in self.model.caption(encoded_image, stream=True)["caption"]:
                print(chunk, end="", flush=True)
        except Exception as e:
            print(f"Error streaming caption: {e}")

    def query_image(self, encoded_image, query):
        """
        Query the image with a prompt.
        """
        try:
            answer = self.model.query(encoded_image, query)["answer"]
            print("\nAnswer:", answer)
        except Exception as e:
            print(f"Error querying image: {e}")

    def stream_query(self, encoded_image, query):
        """
        Stream a query answer in chunks.
        """
        print("Streaming answer:", end=" ", flush=True)
        try:
            for chunk in self.model.query(encoded_image, query, stream=True)["answer"]:
                print(chunk, end="", flush=True)
        except Exception as e:
            print(f"Error streaming answer: {e}")

    def detect_objects(self, encoded_image, subject):
        """
        Detect objects in the image and get bounding boxes (x,y).
        """
        try:
            detect_result = self.model.detect(encoded_image, subject)
            print("\nDetected:", detect_result["objects"])
        except Exception as e:
            print(f"Error detecting objects: {e}")


# Example usage:
if __name__ == "__main__":
    model_path = "./moondream-0_5b-int8.mf.gz"
    image_path = "people.jpg"
    query_text = "Describe people"
    subject = "apple"

    handler = MoondreamWrapper(model_path)
    image = handler.load_image(image_path)
    encoded_image = handler.encode_image(image)

    handler.generate_caption(encoded_image)
    handler.stream_caption(encoded_image)
    handler.query_image(encoded_image, query_text)
    handler.stream_query(encoded_image, query_text)
    handler.detect_objects(encoded_image, subject)

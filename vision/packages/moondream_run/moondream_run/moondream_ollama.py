import base64

import requests

image_path = "../../vision_general/Utils/angle_test_images/2.jpeg"
with open(image_path, "rb") as image_file:
    encoded_image = base64.b64encode(image_file.read()).decode("utf-8")

url = "http://localhost:11434/api/generate"
data = {
    "model": "moondream",
    "prompt": "Describe the image in detail.",
    "images": [encoded_image],
    "stream": False,
}

response = requests.post(url, json=data)

print(response.json())

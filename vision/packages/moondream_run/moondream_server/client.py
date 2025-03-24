import grpc
import moondream_proto_pb2
import moondream_proto_pb2_grpc
import cv2


def run():
    # Increase max message size for the gRPC channel to 200MB
    options = [
        ("grpc.max_receive_message_length", 200 * 1024 * 1024),
        ("grpc.max_send_message_length", 200 * 1024 * 1024),
    ]
    channel = grpc.insecure_channel("localhost:50052", options=options)
    stub = moondream_proto_pb2_grpc.MoonDreamServiceStub(channel)

    # Load image
    img_path = "pantry.jpeg"
    img = cv2.imread(img_path)

    # Compress image to JPEG format to reduce size
    _, img_bytes = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

    # Send the encoded image to the server
    response = stub.EncodeImage(
        moondream_proto_pb2.ImageRequest(image_data=img_bytes.tobytes())
    )
    encoded_image = response.encoded_image

    # Find beverage
    beverage_response = stub.FindBeverage(
        moondream_proto_pb2.FindBeverageRequest(
            encoded_image=encoded_image, subject="purple gatorade"
        )
    )
    print(f"Beverage position: {beverage_response.position}")

    # Generate description
    description_response = stub.GeneratePersonDescription(
        moondream_proto_pb2.DescriptionRequest(
            encoded_image=encoded_image, query="Describe the image"
        )
    )
    print(f"Description: {description_response.answer}")


if __name__ == "__main__":
    run()

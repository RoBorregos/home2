"""gRPC server exposing LiquidAI LFM2.5-VL through the Moondream service contract."""

import argparse
import io
from concurrent import futures

import grpc
import moondream_proto_pb2
import moondream_proto_pb2_grpc
import torch
from PIL import Image
from transformers import AutoModelForImageTextToText, AutoProcessor

PORT = 50053


class LfmServicer(moondream_proto_pb2_grpc.MoonDreamServiceServicer):
    def __init__(self, model_name: str, revision: str, max_new_tokens: int):
        self.processor = AutoProcessor.from_pretrained(model_name, revision=revision)
        self.model = AutoModelForImageTextToText.from_pretrained(
            model_name, revision=revision, dtype=torch.bfloat16, device_map={"": 0}
        ).eval()
        self.max_new_tokens = max_new_tokens

    def EncodeImage(self, request, context):
        # LFM has no separate image encoding step, so the JPEG bytes are passed back as-is
        return moondream_proto_pb2.EncodedImageResponse(
            encoded_image=request.image_data
        )

    def Query(self, request, context):
        image = Image.open(io.BytesIO(request.encoded_image)).convert("RGB")
        conversation = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": image},
                    {"type": "text", "text": request.query},
                ],
            }
        ]
        inputs = self.processor.apply_chat_template(
            conversation,
            add_generation_prompt=True,
            return_tensors="pt",
            return_dict=True,
            tokenize=True,
        ).to(self.model.device)
        with torch.inference_mode():
            output = self.model.generate(
                **inputs, max_new_tokens=self.max_new_tokens, do_sample=False
            )
        answer = self.processor.batch_decode(
            output[:, inputs["input_ids"].shape[1] :], skip_special_tokens=True
        )[0].strip()
        print(f"Query: {request.query!r} -> {answer!r}")
        return moondream_proto_pb2.QueryResponse(answer=answer)


def serve(model_name: str, revision: str, max_new_tokens: int):
    options = [
        ("grpc.max_receive_message_length", 200 * 1024 * 1024),
        ("grpc.max_send_message_length", 200 * 1024 * 1024),
    ]
    # One worker: generate() is not safe to run concurrently on the same model
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1), options=options)
    print(f"Loading {model_name}@{revision}...")
    moondream_proto_pb2_grpc.add_MoonDreamServiceServicer_to_server(
        LfmServicer(model_name, revision, max_new_tokens), server
    )
    server.add_insecure_port(f"[::]:{PORT}")
    print(f"LFM gRPC server started on port {PORT}...")
    server.start()
    server.wait_for_termination()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="LFM2.5-VL Server")
    parser.add_argument("--model_name", type=str, default="LiquidAI/LFM2.5-VL-450M")
    parser.add_argument(
        "--revision", type=str, default="fc6221ca597f3315e4f82fc2df606783267b34ba"
    )
    parser.add_argument("--max_new_tokens", type=int, default=8)
    args = parser.parse_args()
    serve(args.model_name, args.revision, args.max_new_tokens)

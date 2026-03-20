# Proto Interfaces Package

This package contains Protocol Buffer definitions and gRPC service interfaces for the HRI microservices.

## Contents

- **speech_pb2.py / speech_pb2_grpc.py**: Speech-to-Text service definitions
- **tts_pb2.py / tts_pb2_grpc.py**: Text-to-Speech service definitions

## Usage

### ROS2 Nodes
```python
from proto_interfaces import speech_pb2_grpc, speech_pb2

# Create gRPC channel and stub
channel = grpc.insecure_channel("localhost:50051")
stub = speech_pb2_grpc.SpeechStreamStub(channel)
```

### Microservices
```python
from proto_interfaces import speech_pb2_grpc, speech_pb2

class WhisperServicer(speech_pb2_grpc.SpeechStreamServicer):
    def Transcribe(self, request_iterator, context):
        # Implementation
        pass
```

## Regenerating Proto Files

If you modify the `.proto` files, regenerate the Python code:

```bash
cd hri/proto_interfaces

# Install grpcio-tools
pip install grpcio-tools

# Generate speech protos
python -m grpc_tools.protoc -I. --python_out=proto_interfaces --grpc_python_out=proto_interfaces proto/speech.proto

# Generate TTS protos  
python -m grpc_tools.protoc -I. --python_out=proto_interfaces --grpc_python_out=proto_interfaces proto/tts.proto
```

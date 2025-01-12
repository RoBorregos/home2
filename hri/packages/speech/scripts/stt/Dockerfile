FROM python:3.9-slim

WORKDIR /app

COPY scripts/stt/requirements.txt .

# Install dependencies
RUN apt-get update && apt-get install -y \
    libsndfile1 \
    ffmpeg \
    portaudio19-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir -r requirements.txt

RUN apt-get update && apt-get install -y ffmpeg

# Expose the port for the gRPC server
EXPOSE 50051

CMD ["bash", "-c", "python -u $SCRIPT_NAME --port $PORT --model_size $MODEL_SIZE"]

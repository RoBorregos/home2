#!/bin/sh

ask_for_model() {
    case " $MODELS_TO_DOWNLOAD " in
        *" all "*) return 0 ;;
        *" $1 "*) return 0 ;;
        *) return 1 ;;
    esac
}

echo "Which models do you want to download?"
echo "  1) qwen3"
echo "  2) nomic-embed-text"
echo "  3) rbrgs"
echo "  4) DeepFilterNet3"
echo "  a) all"
echo "  n) none"
printf "Enter choices separated by spaces [default: all]: "
read -r MODELS_TO_DOWNLOAD

if [ -z "$MODELS_TO_DOWNLOAD" ]; then
    MODELS_TO_DOWNLOAD="all"
fi

# Download model and Modelfile to the directory where this script is located
SCRIPT_DIR="../../hri/packages/nlp/assets"

if ask_for_model rbrgs || ask_for_model 3; then
    [ ! -f "$SCRIPT_DIR/rbrgs.F16.gguf" ] && curl -L https://huggingface.co/diegohc/rbrgs-finetuning/resolve/paraphrased-dataset/q4/unsloth.Q4_K_M.gguf -o "$SCRIPT_DIR/rbrgs.F16.gguf"
fi

# Download and unzip DeepFilterNet model
DF_MODEL_DIR="../../hri/packages/speech/assets/downloads"
DF_MODEL_URL="https://github.com/Rikorose/DeepFilterNet/raw/main/models/DeepFilterNet3.zip"
ZIP_FILE="$DF_MODEL_DIR/DeepFilterNet3.zip"

if ask_for_model DeepFilterNet3 || ask_for_model 4; then
    if [ ! -d "$DF_MODEL_DIR/DeepFilterNet3" ]; then
        echo "Downloading DeepFilterNet3 model..."
        mkdir -p "$DF_MODEL_DIR"
        curl -L "$DF_MODEL_URL" -o "$ZIP_FILE"
        echo "Unzipping the model..."
        unzip "$ZIP_FILE" -d "$DF_MODEL_DIR"
        rm "$ZIP_FILE"
        echo "DeepFilterNet3 model downloaded successfully."
    else
        echo "DeepFilterNet3 model already exists. Skipping download."
    fi
fi

# Detect available image
if docker images | grep -q "dustynv/ollama"; then
    IMAGE="dustynv/ollama:0.6.8-r36.4"
    COMMAND="ollama serve"
elif docker images | grep -q "ollama/ollama"; then
    IMAGE="ollama/ollama"
    COMMAND=""
else
    echo "Error: No compatible Ollama image found. Pulling the default image..."
    docker pull ollama/ollama:latest
    IMAGE="ollama/ollama"
    COMMAND=""
fi

echo "Running: docker run -d --rm --runtime=nvidia -v \"$SCRIPT_DIR\":/ollama -e OLLAMA_MODELS=/ollama $IMAGE $COMMAND"

# Don't quote $COMMAND to allow for multiple word commands
CONTAINER_ID=$(docker run -d --rm --runtime=nvidia -v "$SCRIPT_DIR":/ollama -e OLLAMA_MODELS=/ollama "$IMAGE" $COMMAND)

if ask_for_model qwen3 || ask_for_model 1; then
    docker exec "$CONTAINER_ID" ollama pull qwen3
fi

if ask_for_model nomic-embed-text || ask_for_model 2; then
    docker exec "$CONTAINER_ID" ollama pull nomic-embed-text
fi

if ask_for_model rbrgs || ask_for_model 3; then
    docker exec "$CONTAINER_ID" ollama create -f /ollama/Modelfile rbrgs
fi

docker stop "$CONTAINER_ID"

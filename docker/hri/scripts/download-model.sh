#!/bin/sh

ask_for_model() {
    case " $MODELS_TO_DOWNLOAD " in
        *" all "*) return 0 ;;
        *" a "*) return 0 ;;
        *" $1 "*) return 0 ;;
        *" $2 "*) return 0 ;;
        *) return 1 ;;
    esac
}

echo "Which models do you want to download?"
echo "  1) qwen3"
echo "  2) nomic-embed-text"
echo "  3) rbrgs"
echo "  4) DeepFilterNet3"
echo "  5) ei-door (Door detection)"
echo "  6) ei-oww  (Keyword wakeword)"
echo "  a) all"
echo "  n) none"
printf "Enter choices separated by spaces [default: all]: "
read -r MODELS_TO_DOWNLOAD

if [ -z "$MODELS_TO_DOWNLOAD" ]; then
    MODELS_TO_DOWNLOAD="all"
fi

# Download model and Modelfile to the directory where this script is located
SCRIPT_DIR="../../hri/packages/nlp/assets"

if ask_for_model rbrgs 3; then
    [ ! -f "$SCRIPT_DIR/rbrgs.F16.gguf" ] && curl -L https://huggingface.co/diegohc/rbrgs-finetuning/resolve/paraphrased-dataset/q4/unsloth.Q4_K_M.gguf -o "$SCRIPT_DIR/rbrgs.F16.gguf"
fi

# Download and unzip DeepFilterNet model
DF_MODEL_DIR="../../hri/packages/speech/assets/downloads"
DF_MODEL_URL="https://github.com/Rikorose/DeepFilterNet/raw/main/models/DeepFilterNet3.zip"
ZIP_FILE="$DF_MODEL_DIR/DeepFilterNet3.zip"

if ask_for_model DeepFilterNet3 4; then
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

# ── Edge Impulse model downloads ──────────────────────────────────────────────
# Downloads models via the EI inference container using an API key.
# The container downloads the .eim file, then we copy it out.
# API keys are read from environment variables: EI_API_KEY_DOOR, EI_API_KEY_OWW
# You can also set them in docker/hri/.env
EI_DOWNLOAD_DIR="../../hri/packages/speech/assets/downloads"
EI_IMAGE="public.ecr.aws/g7a8t7v6/inference-container-jetson-orin-6-0:v1.92.11"

download_ei_model() {
    local model_name="$1"
    local api_key="$2"
    local output_dir="$EI_DOWNLOAD_DIR/$model_name"
    local port="$3"

    if [ -z "$api_key" ]; then
        printf "Enter Edge Impulse API key for %s: " "$model_name"
        read -r api_key
    fi

    if [ -z "$api_key" ]; then
        echo "Error: No API key provided for $model_name. Skipping."
        return 1
    fi

    mkdir -p "$output_dir"

    echo "Downloading Edge Impulse model: $model_name ..."
    CONTAINER_ID=$(docker run -d --rm \
        -p "$port:$port" \
        "$EI_IMAGE" \
        --api-key "$api_key" \
        --run-http-server "$port")

    # Wait for the model to download and the server to start
    echo "Waiting for EI container to download and build the model (this may take a few minutes)..."
    MAX_WAIT=300
    WAITED=0
    while [ $WAITED -lt $MAX_WAIT ]; do
        if docker logs "$CONTAINER_ID" 2>&1 | grep -q "HTTP Server now running"; then
            echo "EI model $model_name downloaded successfully."
            break
        fi
        if ! docker ps -q --filter "id=$CONTAINER_ID" | grep -q .; then
            echo "Error: EI container exited unexpectedly for $model_name."
            docker logs "$CONTAINER_ID" 2>&1 | tail -20
            return 1
        fi
        sleep 5
        WAITED=$((WAITED + 5))
        echo "  Still waiting... (${WAITED}s / ${MAX_WAIT}s)"
    done

    if [ $WAITED -ge $MAX_WAIT ]; then
        echo "Error: Timed out waiting for EI model $model_name to download."
        docker stop "$CONTAINER_ID" 2>/dev/null
        return 1
    fi

    # Copy the model file from the container
    # EI stores models in /root/.ei-linux-runner/models/
    echo "Copying model from container..."
    docker cp "$CONTAINER_ID:/root/.ei-linux-runner/models/" "/tmp/ei-models-$model_name"

    # Find the .eim file and copy it to the output directory
    EIM_FILE=$(find "/tmp/ei-models-$model_name" -name "model.eim" -type f | head -1)
    if [ -n "$EIM_FILE" ]; then
        cp "$EIM_FILE" "$output_dir/model.eim"
        echo "Model saved to $output_dir/model.eim"
    else
        echo "Warning: Could not find model.eim in container. Listing available files:"
        find "/tmp/ei-models-$model_name" -type f
    fi

    rm -rf "/tmp/ei-models-$model_name"
    docker stop "$CONTAINER_ID" 2>/dev/null
}

if ask_for_model ei-door 5; then
    if [ -f "$EI_DOWNLOAD_DIR/door/model.eim" ]; then
        echo "Edge Impulse door model already exists. Skipping download."
    else
        download_ei_model "door" "${EI_API_KEY_DOOR:-}" "1337"
    fi
fi

if ask_for_model ei-oww 6; then
    if [ -f "$EI_DOWNLOAD_DIR/oww/model.eim" ]; then
        echo "Edge Impulse OWW model already exists. Skipping download."
    else
        download_ei_model "oww" "${EI_API_KEY_OWW:-}" "1338"
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

if ask_for_model qwen3 1; then
    docker exec "$CONTAINER_ID" ollama pull qwen3
fi

if ask_for_model nomic-embed-text 2; then
    docker exec "$CONTAINER_ID" ollama pull nomic-embed-text
fi

if ask_for_model rbrgs 3; then
    docker exec "$CONTAINER_ID" ollama create -f /ollama/Modelfile rbrgs
fi

docker stop "$CONTAINER_ID"

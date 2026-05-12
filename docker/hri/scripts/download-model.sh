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
echo "  1) qwen3-1.7b       (Qwen3-1.7B Q4_K_M ~1GB GGUF, for llama.cpp — fast NLP tasks)"
echo "  2) qwen3-8b         (Qwen3-8B Q4_K_M ~5.2GB GGUF, for llama.cpp — same model as Ollama)"
echo "  3) qwen3            (Qwen3 latest via Ollama pull, for Ollama comparison)"
echo "  4) nomic-embed-text (embeddings model, for Ollama)"
echo "  5) rbrgs            (fine-tuned command interpreter GGUF)"
echo "  6) DeepFilterNet3"
echo "  7) ei-door          (Door detection)"
echo "  8) ei-kws           (Keyword wakeword)"
echo "  a) all"
echo "  n) none"
printf "Enter choices separated by spaces [default: all]: "
read -r MODELS_TO_DOWNLOAD

if [ -z "$MODELS_TO_DOWNLOAD" ]; then
    MODELS_TO_DOWNLOAD="all"
fi

SCRIPT_DIR="../../hri/packages/nlp/assets"

# ── qwen3-1.7b GGUF for llama.cpp (fast NLP tasks) ───────────────────────────
if ask_for_model qwen3-1.7b 1; then
    if [ ! -f "$SCRIPT_DIR/qwen3-1.7b.Q4_K_M.gguf" ]; then
        echo "Downloading qwen3-1.7b GGUF for llama.cpp..."
        curl -L https://huggingface.co/unsloth/Qwen3-1.7B-GGUF/resolve/main/Qwen3-1.7B-Q4_K_M.gguf \
             -o "$SCRIPT_DIR/qwen3-1.7b.Q4_K_M.gguf"
    else
        echo "qwen3-1.7b GGUF already exists. Skipping."
    fi
fi

# ── qwen3-8b GGUF for llama.cpp (same model as Ollama default) ───────────────
if ask_for_model qwen3-8b 2; then
    if [ ! -f "$SCRIPT_DIR/qwen3-8b.Q4_K_M.gguf" ]; then
        echo "Downloading qwen3-8b GGUF for llama.cpp..."
        curl -L https://huggingface.co/unsloth/Qwen3-8B-GGUF/resolve/main/Qwen3-8B-Q4_K_M.gguf \
             -o "$SCRIPT_DIR/qwen3-8b.Q4_K_M.gguf"
    else
        echo "qwen3-8b GGUF already exists. Skipping."
    fi
fi

# ── rbrgs GGUF for llama.cpp ──────────────────────────────────────────────────
if ask_for_model rbrgs 5; then
    if [ ! -f "$SCRIPT_DIR/rbrgs.F16.gguf" ]; then
        echo "Downloading rbrgs GGUF..."
        curl -L https://huggingface.co/diegohc/rbrgs-finetuning/resolve/paraphrased-dataset/q4/unsloth.Q4_K_M.gguf \
             -o "$SCRIPT_DIR/rbrgs.F16.gguf"
    else
        echo "rbrgs GGUF already exists. Skipping."
    fi
fi

# ── DeepFilterNet3 ────────────────────────────────────────────────────────────
DF_MODEL_DIR="../../hri/packages/speech/assets/downloads"
DF_MODEL_URL="https://github.com/Rikorose/DeepFilterNet/raw/main/models/DeepFilterNet3.zip"
ZIP_FILE="$DF_MODEL_DIR/DeepFilterNet3.zip"

if ask_for_model DeepFilterNet3 6; then
    if [ ! -d "$DF_MODEL_DIR/DeepFilterNet3" ]; then
        echo "Downloading DeepFilterNet3 model..."
        mkdir -p "$DF_MODEL_DIR"
        curl -L "$DF_MODEL_URL" -o "$ZIP_FILE"
        echo "Unzipping the model..."
        unzip "$ZIP_FILE" -d "$DF_MODEL_DIR"
        rm "$ZIP_FILE"
        echo "DeepFilterNet3 model downloaded successfully."
    else
        echo "DeepFilterNet3 model already exists. Skipping."
    fi
fi

# ── Edge Impulse model downloads ──────────────────────────────────────────────
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
    CONTAINER_ID=$(docker run -d \
        --runtime=nvidia \
        -p "$port:$port" \
        "$EI_IMAGE" \
        --api-key "$api_key" \
        --run-http-server "$port" \
        --force-target runner-linux-aarch64-jetson-orin-6-0 \
        --force-variant float32)

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
            docker rm "$CONTAINER_ID" 2>/dev/null
            return 1
        fi
        sleep 5
        WAITED=$((WAITED + 5))
        echo "  Still waiting... (${WAITED}s / ${MAX_WAIT}s)"
    done

    if [ $WAITED -ge $MAX_WAIT ]; then
        echo "Error: Timed out waiting for EI model $model_name."
        docker stop "$CONTAINER_ID" 2>/dev/null
        docker rm "$CONTAINER_ID" 2>/dev/null
        return 1
    fi

    echo "Copying model from container..."
    docker cp "$CONTAINER_ID:/root/.ei-linux-runner/models/" "/tmp/ei-models-$model_name"
    EIM_FILE=$(find "/tmp/ei-models-$model_name" -name "model.eim" -type f | head -1)
    if [ -n "$EIM_FILE" ]; then
        cp "$EIM_FILE" "$output_dir/model.eim"
        echo "Model saved to $output_dir/model.eim"
    else
        echo "Warning: Could not find model.eim. Listing available files:"
        find "/tmp/ei-models-$model_name" -type f
    fi

    rm -rf "/tmp/ei-models-$model_name"
    docker stop "$CONTAINER_ID" 2>/dev/null
    docker rm "$CONTAINER_ID" 2>/dev/null
}

if ask_for_model ei-door 7; then
    if [ -f "$EI_DOWNLOAD_DIR/door/model.eim" ]; then
        echo "Edge Impulse door model already exists. Skipping."
    else
        download_ei_model "door" "${EI_API_KEY_DOOR:-}" "1337"
    fi
fi

if ask_for_model ei-kws 8; then
    if [ -f "$EI_DOWNLOAD_DIR/kws/model.eim" ]; then
        echo "Edge Impulse kws model already exists. Skipping."
    else
        download_ei_model "kws" "${EI_API_KEY_KWS:-}" "1338"
    fi
fi

# ── Ollama models (qwen3 + nomic-embed-text + rbrgs) ─────────────────────────
# qwen3 is pulled via Ollama (Ollama 0.6.8 compatible).
# rbrgs is imported from its GGUF using a Modelfile.
OLLAMA_IMAGE="dustynv/ollama:0.6.8-r36.4"
ABSOLUTE_SCRIPT_DIR="$(cd "$SCRIPT_DIR" && pwd)"

needs_ollama=false
ask_for_model qwen3 3            && [ ! -d "$SCRIPT_DIR/manifests/registry.ollama.ai/library/qwen3" ]           && needs_ollama=true
ask_for_model nomic-embed-text 4 && [ ! -d "$SCRIPT_DIR/manifests/registry.ollama.ai/library/nomic-embed-text" ] && needs_ollama=true
ask_for_model rbrgs 5 && [ -f "$SCRIPT_DIR/rbrgs.F16.gguf" ] && \
    [ ! -d "$SCRIPT_DIR/manifests/registry.ollama.ai/library/rbrgs" ]                                       && needs_ollama=true

if $needs_ollama; then
    echo "Starting temporary Ollama container to pull/import models..."
    OLLAMA_CONTAINER=$(docker run -d \
        --runtime=nvidia \
        -e OLLAMA_MODELS=/ollama \
        -v "$ABSOLUTE_SCRIPT_DIR:/ollama" \
        "$OLLAMA_IMAGE" \
        bash -c "ollama serve & sleep infinity")

    echo "Waiting for Ollama to start..."
    MAX_WAIT=60; WAITED=0
    while [ $WAITED -lt $MAX_WAIT ]; do
        if docker exec "$OLLAMA_CONTAINER" curl -sf http://localhost:11434/api/version >/dev/null 2>&1; then
            echo "Ollama ready."
            break
        fi
        sleep 2; WAITED=$((WAITED + 2))
    done

    if ask_for_model qwen3 3 && \
       [ ! -d "$SCRIPT_DIR/manifests/registry.ollama.ai/library/qwen3" ]; then
        echo "Pulling qwen3 via Ollama..."
        docker exec "$OLLAMA_CONTAINER" ollama pull qwen3
        echo "qwen3 pulled."
    fi

    if ask_for_model nomic-embed-text 4 && \
       [ ! -d "$SCRIPT_DIR/manifests/registry.ollama.ai/library/nomic-embed-text" ]; then
        echo "Pulling nomic-embed-text via Ollama..."
        docker exec "$OLLAMA_CONTAINER" ollama pull nomic-embed-text
        echo "nomic-embed-text pulled."
    fi

    if ask_for_model rbrgs 5 && [ -f "$SCRIPT_DIR/rbrgs.F16.gguf" ] && \
       [ ! -d "$SCRIPT_DIR/manifests/registry.ollama.ai/library/rbrgs" ]; then
        printf 'FROM /ollama/rbrgs.F16.gguf\n' > "$SCRIPT_DIR/Modelfile.rbrgs"
        echo "Importing rbrgs into Ollama..."
        docker exec "$OLLAMA_CONTAINER" ollama create -f /ollama/Modelfile.rbrgs rbrgs
        echo "rbrgs imported."
    fi

    docker stop "$OLLAMA_CONTAINER" >/dev/null && docker rm "$OLLAMA_CONTAINER" >/dev/null
    echo "Ollama models ready."
fi

echo "All selected models downloaded."

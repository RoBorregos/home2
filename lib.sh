#!/usr/bin/env bash

# --- guard against multiple sourcing ---
if [[ -n "${__HOME2_LIB_SOURCED:-}" ]]; then
  return 0
fi
__HOME2_LIB_SOURCED=1

# --- helpers ---

check_image_exists() {
  local image_name=$1
  if ! docker images --format "{{.Repository}}:{{.Tag}}" | grep -q "^${image_name}$"; then
    echo "Image $image_name does not exist. Building it..."
    return 1
  else
    echo "Image $image_name already exists. Skipping build."
    return 0
  fi
}

add_or_update_variable() {
  local file=$1 variable=$2 value=$3

  # escape sed-sensitive chars in value
  local escaped_value
  escaped_value=$(printf '%s\n' "$value" | sed -e 's/[&/\]/\\&/g')

  if grep -q "^${variable}=" "$file" 2>/dev/null; then
    sed -i "s|^${variable}=.*|${variable}=${escaped_value}|" "$file"
  else
    printf '%s=%s\n' "$variable" "$value" >> "$file"
  fi
}

# Build and push all roborregos/home2 images referenced in a compose file
upload_images() {
  local compose_file=${1:-docker-compose.yaml}
  if [ ! -f "$compose_file" ]; then
    echo "Compose file $compose_file not found" >&2
    return 1
  fi

  docker login

  echo "Building images from compose: $compose_file"
  if ! docker compose -f "$compose_file" build; then
    echo "Compose build failed" >&2
    return 1
  fi

  local images
  images=$(docker compose -f "$compose_file" config 2>/dev/null \
    | awk '/^\s*image:/ {print $2}' | sort -u || true)
  images=$(echo "$images" | grep '^roborregos/home2' || true)

  if [ -z "$images" ]; then
    echo "Nothing to push."
    return 1
  fi

  local rc=0
  echo "Images to push:"
  echo "$images"
  while IFS= read -r img; do
    [ -z "$img" ] && continue
    echo "Pushing $img ..."
    if ! docker push "$img"; then
      echo "Failed to push $img" >&2
      rc=1
    fi
  done <<< "$images"

  if [ $rc -eq 0 ]; then
    echo "All pushes finished."
  else
    echo "One or more pushes failed." >&2
  fi
  return $rc

}

# Build and pull all roborregos/home2 images referenced in the env/compose file
pull_image() {
  if [ "$1" = "--env-file" ]; then
    local envfile=${2:-.env}
    if [ ! -f "$envfile" ]; then
      echo "Env file $envfile not found" >&2
      return 1
    fi
    local image
    image=$(grep -E '^IMAGE_NAME=' "$envfile" | cut -d'=' -f2- | tr -d '"' | tr -d "'")
    if [ -z "$image" ]; then
      echo "IMAGE_NAME not found in $envfile" >&2
      return 1
    fi
    echo "Pulling image from env: $image"
    if ! docker pull "$image"; then
      echo "Failed to pull $image" >&2
      return 1
    fi
    return 0

  elif [ "$1" = "--compose" ]; then
    local compose_file=${2:-docker-compose.yaml}
    if [ ! -f "$compose_file" ]; then
      echo "Compose file $compose_file not found" >&2
      return 1
    fi

    local images
    images=$(docker compose -f "$compose_file" config 2>/dev/null \
      | awk '/^\s*image:/ {print $2}' | sort -u || true)
    images=$(echo "$images" | grep '^roborregos/home2' || true)

    if [ -z "$images" ]; then
      echo "No matching images to pull from $compose_file."
      return 1
    fi

    local rc=0
    echo "Images to pull from $compose_file:"
    echo "$images"
    while IFS= read -r img; do
      [ -z "$img" ] && continue
      echo "Pulling $img ..."
      if ! docker pull "$img"; then
        echo "Failed to pull $img" >&2
        rc=1
      fi
    done <<< "$images"

    if [ $rc -ne 0 ]; then
      echo "One or more pulls failed." >&2
    else
      echo "All pulls finished."
    fi
    return $rc

  else
    local image=${1:-}
    if [ -z "$image" ]; then
      echo "pull_image: no image specified" >&2
      return 1
    fi
    echo "Pulling image: $image"
    if ! docker pull "$image"; then
      echo "Failed to pull $image" >&2
      return 1
    fi
    return 0
  fi
}
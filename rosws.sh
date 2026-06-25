#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME=${IMAGE_NAME:-ros2-irs-rascl-wp22}
CONTAINER_NAME=${CONTAINER_NAME:-$IMAGE_NAME}

if ${NO_ATTACH:-false} || ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  if ${REBUILD:-false}; then
    echo "Rebuilding container image without cache..."
    docker buildx build --network host --no-cache -t "$IMAGE_NAME" .
  elif ${SOFT_REBUILD:-false} || ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "Building container image..."
    docker buildx build --network host -t "$IMAGE_NAME" .
  else
    echo "Container image already exists. Set REBUILD=true to rebuild from scratch."
  fi

  mkdir -p .devcontainer
  touch .devcontainer/.bash_history

  X11_MOUNT=""
  if [[ -n "${DISPLAY:-}" ]]; then
    if grep -qiE 'microsoft|wsl' /proc/version 2>/dev/null; then
      X11_MOUNT="-v /mnt/wslg/.X11-unix:/tmp/.X11-unix"
    else
      X11_MOUNT="-v /tmp/.X11-unix:/tmp/.X11-unix"
    fi
  fi

  WAYLAND_MOUNT=""
  if [[ -n "${XDG_RUNTIME_DIR:-}" && -n "${WAYLAND_DISPLAY:-}" && -e "${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}" ]]; then
    WAYLAND_MOUNT="-v ${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}:/run/user/0/${WAYLAND_DISPLAY}"
  fi

  DEVICE_ARGS=""
  if [[ -e /dev/cpu_dma_latency ]]; then
    DEVICE_ARGS="--device /dev/cpu_dma_latency"
  fi

  docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --privileged \
    --network=host \
    --cap-add CAP_NET_RAW \
    --cap-add CAP_NET_ADMIN \
    --cap-add CAP_IPC_LOCK \
    --cap-add CAP_SYS_NICE \
    -v "$(pwd):/root/ws" \
    -v "$(pwd)/.devcontainer/.bash_history:/root/.bash_history" \
    ${X11_MOUNT} \
    ${WAYLAND_MOUNT} \
    -e DISPLAY="${DISPLAY:-}" \
    -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
    -e XDG_RUNTIME_DIR="/run/user/0" \
    -e TERM="xterm-256color" \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=0 \
    -e MESA_GL_VERSION_OVERRIDE=3.3 \
    --log-driver=none \
    ${DEVICE_ARGS} \
    "$IMAGE_NAME"
else
  echo "Attaching to running container..."
  docker exec -it "$CONTAINER_NAME" bash
fi

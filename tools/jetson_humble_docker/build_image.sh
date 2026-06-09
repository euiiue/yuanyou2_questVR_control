#!/usr/bin/env bash
set -Eeuo pipefail

IMAGE_NAME="${IMAGE_NAME:-questarmteleop:humble-jetson}"
BASE_IMAGE="${BASE_IMAGE:-ros:humble-ros-base-jammy}"
PROXY_HOST="${PROXY_HOST:-192.168.31.179}"
PROXY_PORT="${PROXY_PORT:-7890}"
PROXY_URL="http://${PROXY_HOST}:${PROXY_PORT}"
DOCKER_MIRROR="${DOCKER_MIRROR:-https://docker.m.daocloud.io}"
NO_PROXY_VALUE="${NO_PROXY_VALUE:-localhost,127.0.0.1,::1,192.168.0.0/16,10.0.0.0/8,172.16.0.0/12,docker.m.daocloud.io,docker.1ms.run,dockerpull.com}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MIRROR_HOST="${DOCKER_MIRROR#http://}"
MIRROR_HOST="${MIRROR_HOST#https://}"
MIRROR_HOST="${MIRROR_HOST%/}"
MIRROR_IMAGE_CANDIDATES="${MIRROR_IMAGE_CANDIDATES:-docker.1ms.run/library/ros:humble-ros-base-jammy ${MIRROR_HOST}/library/ros:humble-ros-base-jammy docker.m.daocloud.io/library/ros:humble-ros-base-jammy dockerpull.com/library/ros:humble-ros-base-jammy}"

echo "[1/4] Configuring Docker daemon mirror: ${DOCKER_MIRROR}"
sudo mkdir -p /etc/docker
if [[ -f /etc/docker/daemon.json ]]; then
  sudo cp -a /etc/docker/daemon.json "/etc/docker/daemon.json.bak-$(date +%Y%m%d_%H%M%S)"
fi
sudo tee /etc/docker/daemon.json >/dev/null <<EOF
{
  "registry-mirrors": ["${DOCKER_MIRROR}"]
}
EOF

echo "[2/4] Configuring Docker daemon proxy for non-mirror traffic: ${PROXY_URL}"
sudo mkdir -p /etc/systemd/system/docker.service.d
sudo tee /etc/systemd/system/docker.service.d/http-proxy.conf >/dev/null <<EOF
[Service]
Environment="HTTP_PROXY=${PROXY_URL}" "HTTPS_PROXY=${PROXY_URL}" "NO_PROXY=${NO_PROXY_VALUE}"
EOF
sudo systemctl daemon-reload
sudo systemctl restart docker

echo "[3/4] Pulling ROS Humble base image"
if sudo docker image inspect "${BASE_IMAGE}" >/dev/null 2>&1; then
  echo "Base image already exists locally: ${BASE_IMAGE}"
else
  pulled_image=""
  tried_images=" "
  for candidate in ${MIRROR_IMAGE_CANDIDATES}; do
    if [[ "${tried_images}" == *" ${candidate} "* ]]; then
      continue
    fi
    tried_images="${tried_images}${candidate} "
    echo "Trying mirror image: ${candidate}"
    if sudo docker pull "${candidate}"; then
      pulled_image="${candidate}"
      break
    fi
  done

  if [[ -n "${pulled_image}" ]]; then
    sudo docker tag "${pulled_image}" "${BASE_IMAGE}"
  else
    echo "Mirror pulls failed; falling back to Docker Hub image: ${BASE_IMAGE}"
    sudo docker pull "${BASE_IMAGE}"
  fi
fi

echo "[4/4] Building ${IMAGE_NAME}"
sudo docker build \
  --network host \
  --build-arg http_proxy="${PROXY_URL}" \
  --build-arg https_proxy="${PROXY_URL}" \
  --build-arg HTTP_PROXY="${PROXY_URL}" \
  --build-arg HTTPS_PROXY="${PROXY_URL}" \
  --build-arg no_proxy="${NO_PROXY_VALUE}" \
  --build-arg NO_PROXY="${NO_PROXY_VALUE}" \
  -t "${IMAGE_NAME}" \
  -f "${SCRIPT_DIR}/Dockerfile" \
  "${SCRIPT_DIR}"

echo "Built image: ${IMAGE_NAME}"

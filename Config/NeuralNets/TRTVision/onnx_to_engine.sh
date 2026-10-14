#!/bin/bash
set -e

ONNX=$1
ROBOT_IP=$2
ROBOT_USER=${3:-root}

if [[ -z "$ONNX" || -z "$ROBOT_IP" ]]; then
    echo "Usage: $0 <model.onnx> <robot_ip> [robot_user]"
    exit 1
fi

ENGINE="${ONNX%.onnx}.engine"
REMOTE_DIR="/tmp/trt_convert"

ONNX_BASE=$(basename "$ONNX")
ENGINE_BASE=$(basename "$ENGINE")

rsync -av "$ONNX" "${ROBOT_USER}@${ROBOT_IP}:${REMOTE_DIR}/"

ssh "${ROBOT_USER}@${ROBOT_IP}" \
    "/usr/src/tensorrt/bin/trtexec --onnx=${REMOTE_DIR}/${ONNX_BASE} --saveEngine=${REMOTE_DIR}/${ENGINE_BASE} --fp16 --verbose"

rsync -av "${ROBOT_USER}@${ROBOT_IP}:${REMOTE_DIR}/${ENGINE_BASE}" .

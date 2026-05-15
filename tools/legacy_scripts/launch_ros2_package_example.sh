#!/bin/bash
# run.sh

IMAGE_NAME="wildbot_workspace"
DOCKERFILE="docker/images/base/Dockerfile"
MARKER=".docker_build_hash"

# 計算目前 Dockerfile 的 hash
CURRENT_HASH=$(sha256sum $DOCKERFILE | awk '{print $1}')

# 確認是否需要重新 build
NEED_BUILD=false

if ! docker image inspect $IMAGE_NAME &>/dev/null; then
  echo "[wildbot] Image 不存在，開始 build..."
  NEED_BUILD=true
elif [ ! -f $MARKER ]; then
  echo "[wildbot] 找不到 build 記錄，重新 build..."
  NEED_BUILD=true
elif [ "$CURRENT_HASH" != "$(cat $MARKER)" ]; then
  echo "[wildbot] Dockerfile 有變更，重新 build..."
  NEED_BUILD=true
fi

if [ "$NEED_BUILD" = true ]; then
  docker build -f $DOCKERFILE -t $IMAGE_NAME . || { echo "[wildbot] Build failed"; exit 1; }
  echo $CURRENT_HASH > $MARKER
  echo "[wildbot] Build 完成"
fi

NETWORK_NAME="igvi_robot_robot_net"
COMPOSE_PROJECT="igvi_robot"
COMPOSE_NETWORK="robot_net"

# 確保 network 存在（加上 compose 預期的 labels）
if ! docker network inspect $NETWORK_NAME &>/dev/null; then
  echo "[wildbot] Network 不存在，建立 $NETWORK_NAME..."
  docker network create \
    --driver bridge \
    --label com.docker.compose.network=$COMPOSE_NETWORK \
    --label com.docker.compose.project=$COMPOSE_PROJECT \
    --label com.docker.compose.version="$(docker compose version --short 2>/dev/null || echo 2.0.0)" \
    $NETWORK_NAME
fi

# 啟動
echo "[wildbot] starting container..."
docker run -it \
  --name wildbot \
  --rm \
  --network $NETWORK_NAME \
  --env-file ./docker/compose/.env \
  -v $(pwd)/robot_ws:/workspaces \
  -v $(pwd)/robot_ws/src/wildbot_bringup/config:/configs \
  $IMAGE_NAME \
  ros2 run demo_nodes_cpp talker

### ↑↑↑↑↑↑  copy the .sh file, change the command above, and run it ↑↑↑↑↑↑ ###

# 清理 network（如果沒有其他 container 在用）
if docker network inspect $NETWORK_NAME &>/dev/null; then
  CONNECTED=$(docker network inspect $NETWORK_NAME --format '{{len .Containers}}')
  if [ "$CONNECTED" -eq 0 ]; then
    echo "[wildbot] 沒有其他 container 使用 $NETWORK_NAME，移除..."
    docker network rm $NETWORK_NAME
  else
    echo "[wildbot] $NETWORK_NAME 仍有 $CONNECTED 個 container 在使用，保留"
  fi
fi

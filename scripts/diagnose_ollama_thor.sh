#!/bin/bash
# Diagnose Ollama container issues on Thor

echo "=========================================="
echo "Ollama Container Diagnostic"
echo "=========================================="
echo ""

CONTAINER_NAME="ollama"

echo "1. Container Status:"
docker ps -a | grep "$CONTAINER_NAME" || echo "No container found"
echo ""

echo "2. Recent Container Logs:"
docker logs --tail 50 "$CONTAINER_NAME" 2>/dev/null || echo "Cannot get logs"
echo ""

echo "3. Container Inspect (networking):"
docker inspect "$CONTAINER_NAME" --format='{{json .NetworkSettings}}' 2>/dev/null | python3 -m json.tool || echo "Cannot inspect"
echo ""

echo "4. Check if Ollama process is running inside container:"
docker exec "$CONTAINER_NAME" ps aux 2>/dev/null || echo "Cannot execute in container"
echo ""

echo "5. Try to access Ollama API from inside container:"
docker exec "$CONTAINER_NAME" curl -s http://localhost:11434/api/tags 2>/dev/null || echo "API not responding"
echo ""

echo "6. Check container environment:"
docker exec "$CONTAINER_NAME" env 2>/dev/null | grep OLLAMA || echo "Cannot check env"
echo ""

echo "=========================================="
echo "Suggested Actions:"
echo "=========================================="
echo ""
echo "If container is crash-looping:"
echo "  docker rm -f $CONTAINER_NAME"
echo "  # Try starting without --gpus flag first"
echo "  docker run -d --name $CONTAINER_NAME -p 11434:11434 ghcr.io/nvidia-ai-iot/ollama:r38.2.arm64-sbsa-cu130-24.04"
echo ""
echo "If API not responding:"
echo "  docker exec -it $CONTAINER_NAME /bin/bash"
echo "  # Inside container: check if ollama binary exists"
echo "  which ollama"
echo "  ollama serve &"
echo ""

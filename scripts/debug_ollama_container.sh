#!/bin/bash
# Debug Ollama container issues

echo "=== Ollama Container Debug ==="
echo ""

CONTAINER_NAME="ollama"

echo "1. Is container running?"
docker ps | grep "$CONTAINER_NAME"
echo ""

echo "2. Container restart count:"
docker inspect "$CONTAINER_NAME" --format='RestartCount: {{.RestartCount}}' 2>/dev/null || echo "Cannot inspect"
echo ""

echo "3. Container state:"
docker inspect "$CONTAINER_NAME" --format='State: {{.State.Status}} | Running: {{.State.Running}} | ExitCode: {{.State.ExitCode}}' 2>/dev/null || echo "Cannot inspect"
echo ""

echo "4. Last 30 lines of logs:"
docker logs --tail 30 "$CONTAINER_NAME" 2>&1
echo ""

echo "5. Check if Ollama process is running inside container:"
docker exec "$CONTAINER_NAME" ps aux 2>/dev/null | grep -E "ollama|PID" || echo "Cannot exec into container"
echo ""

echo "6. Try to hit API from inside container:"
docker exec "$CONTAINER_NAME" curl -s http://localhost:11434/api/tags 2>/dev/null || echo "API not responding from inside"
echo ""

echo "7. Check what's listening on ports inside container:"
docker exec "$CONTAINER_NAME" netstat -tulpn 2>/dev/null | grep -E "11434|LISTEN" || echo "Cannot check ports"
echo ""

echo "8. Container environment variables:"
docker exec "$CONTAINER_NAME" env 2>/dev/null | grep OLLAMA || echo "Cannot get env"
echo ""

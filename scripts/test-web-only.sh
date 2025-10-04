#!/bin/bash
# Test web interface without ROS
# Good for checking if web server works before full launch

cd "$(dirname "$0")/.."

echo "Testing web interface standalone..."
echo ""
echo "This will test the web server without ROS/robot connection"
echo "Useful for debugging web issues in isolation"
echo ""

python3 -c "
import sys
sys.path.insert(0, 'src/shadowhound_mission_agent')

from shadowhound_mission_agent.web_interface import WebInterface

print('Starting web interface on http://localhost:8080')
print('Press Ctrl+C to stop')
print('')

web = WebInterface(port=8080, mission_callback=lambda x: print(f'Mission received: {x}'))
web.start()

print('Web interface started!')
print('Open http://localhost:8080 in your browser')
print('')

import time
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print('\nShutting down...')
"

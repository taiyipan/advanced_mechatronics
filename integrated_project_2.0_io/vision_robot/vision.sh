#!/usr/bin/bash

while true;
do
  source /home/pi/venv/bin/activate && python3 /home/pi/vision_robot.py || echo "Program crashing... restarting..."
  echo "Press Ctrl-C to quit" && sleep 5
done

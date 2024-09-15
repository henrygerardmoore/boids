#!/bin/bash

cd "$(dirname "$0")"
./build/Release/boids &
sleep 2
wmctrl -F -r boid_desktop_bg -b add,skip_taskbar
wmctrl -F -r boid_desktop_bg -b add,below

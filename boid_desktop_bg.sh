#!/bin/bash

cd "$(dirname "$0")"
./build/boids &
sleep 0.5
wmctrl -F -r boid_desktop_bg -b add,skip_taskbar
wmctrl -F -r boid_desktop_bg -b add,below

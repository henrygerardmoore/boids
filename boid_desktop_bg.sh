#!/bin/bash

cd "$(dirname "$0")"
./build/boids &
sleep 2
wmctrl -F -r boid_desktop_bg -b add,below
wmctrl -F -r boid_desktop_bg -b add,skip_taskbar

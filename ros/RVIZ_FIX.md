# RViz2 Snap Library Conflict Fix

## Problem

RViz2 crashes on launch with:

```
symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```

## Solution

Before launching, preload the system libpthread:

```bash
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0
ros2 launch robot_arm_teleop visual.launch.py
```

## Cause

The snap `core20` runtime's `libpthread.so.0` gets loaded instead of the system one, causing a glibc symbol mismatch. The snap paths don't appear in `LD_LIBRARY_PATH` or `/etc/ld.so.conf.d/`, so the exact injection mechanism is unclear. `LD_PRELOAD` forces the correct system library to load first.

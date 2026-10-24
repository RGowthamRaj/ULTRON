# Rigid3D / Sphere Physics Prototype

A lightweight 3D rigid body physics engine prototype focusing on sphere-sphere and sphere-plane contact dynamics using SI units.

## Features
- **Rigid Body Spheres**: Mass, radius, inertia, velocity, angular velocity (omega), and quaternion orientations.
- **Static Infinite Planes**: Half-space boundary collisions with normal orientation and offset.
- **Impulse-based Solver**: Normal impulse resolution, Coulomb friction model, and positional drift correction.
- **Interactive Tkinter 3D Viewport**: Orbit camera controls, sphere spawning, real-time stepping.

## Requirements
- Python 3.10+
- Tkinter (standard library, `python3-tk` on Debian/Ubuntu)

## Running the Sandbox
```bash
python main.py
```

### Controls
- **Space**: Pause / Resume simulation
- **N**: Spawn a new sphere
- **R**: Reset simulation
- **Right Arrow**: Single step (while paused)
- **Mouse Drag (Left Click)**: Orbit camera
- **Mouse Wheel**: Zoom in / out
- **Esc**: Exit

## Running Unit Tests
```bash
python -m unittest test_engine.py
```

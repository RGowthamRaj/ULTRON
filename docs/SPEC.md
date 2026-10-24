# ULTRON Engine — Specification & Architecture (C++)

> **Status**: `DRAFT`  
> **Engine Target**: High-Performance 3D Rigid Body Physics & Simulation Engine in C++20.

---

## 1. Vision & Overview
**ULTRON** is a high-performance, modular 3D physics and simulation engine written in modern C++ (C++20). Designed to transition from early mathematical and algorithmic exploration (validated in the Python prototype) into a production-grade, cache-coherent, multi-threaded engine capable of simulating complex rigid body dynamics, contact manifolds, constraints, and scene graphs.

---

## 2. System Architecture

```
                  ┌───────────────────────────────┐
                  │       Application Layer       │
                  │   (Renderers, Viewports, CLI) │
                  └───────────────┬───────────────┘
                                  │
                  ┌───────────────▼───────────────┐
                  │          World / Scene        │
                  │ (Integrator, Pipeline Manager)│
                  └───────┬───────────────┬───────┘
                          │               │
        ┌─────────────────▼──┐         ┌──▼──────────────────┐
        │   Broadphase /     │         │   Narrowphase /     │
        │   Spatial Partition│         │   Collision Pairs   │
        │  (BVH, Octree,     │         │ (GJK, EPA, Sphere,  │
        │   Dynamic AABB)    │         │  Half-space, Box)   │
        └─────────────────┬──┘         └──┬──────────────────┘
                          │               │
                          └───────┬───────┘
                                  │
                  ┌───────────────▼───────────────┐
                  │      Constraint Solver        │
                  │ (Projected Gauss-Seidel /     │
                  │  Sequential Impulses)         │
                  └───────────────┬───────────────┘
                                  │
                  ┌───────────────▼───────────────┐
                  │       Core Math & Memory      │
                  │ (SIMD Vec4, Quat, Mat4,       │
                  │  Linear Arena Allocators)     │
                  └───────────────────────────────┘
```

---

## 3. Goals & Roadmap

### Phase 1: Core Foundation & Mathematics
- **Linear Algebra**: SIMD-accelerated vectors, quaternions, transformations, and inertia tensors.
- **Memory Management**: Custom arena and slab allocators to maintain cache locality and avoid per-frame dynamic allocations.

### Phase 2: Collision Detection Pipeline
- **Primitives**: Spheres, half-space infinite planes, oriented bounding boxes (OBBs), and capsules.
- **Broadphase**: Dynamic AABB tree / bounding volume hierarchy (BVH).
- **Narrowphase**: Analytical tests for primitive pairs, GJK (Gilbert-Johnson-Keerthi) & EPA (Expanding Polytope Algorithm) for convex hulls.

### Phase 3: Dynamics & Constraint Solver
- **Numerical Integration**: Semi-implicit Euler / Symplectic Euler with sub-stepping support.
- **Contact Resolution**: Sequential impulse solver with contact manifold generation, Baumgarte stabilization / position projection, and friction cones.

### Phase 4: Threading & Performance
- Task-based job system for parallel broadphase culling, collision pairing, and island solving.

---

## 4. Technical Requirements

| Component | Target Spec | Description |
|-----------|-------------|-------------|
| **Language** | C++20 | Modern standard, strong type safety, concepts, constexpr math |
| **Build System** | CMake 3.22+ | Cross-platform build support (Windows, Linux, macOS) |
| **Math SIMD** | Direct / Intrinsics | SSE2 / AVX2 / NEON alignment |
| **Memory** | Cache-friendly SoA/AoS | Data-oriented layout for rigid body states and transforms |
| **Testing** | Catch2 / GTest | Comprehensive unit and integration test coverage |

---

## 5. Prototype Validation
Early physics formulations (impulses, quaternion integration, restitution, and friction) have been prototyped and verified in `prototype/`:
- [`prototype/engine.py`](../prototype/engine.py): Prototyping sphere and plane collisions, restitution, and rotational integration.
- [`prototype/test_engine.py`](../prototype/test_engine.py): Empirical unit test suite verifying momentum conservation and friction response.

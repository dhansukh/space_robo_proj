# Cave Generation Pipeline — Walkthrough

## What Was Done

Made the cave generation pipeline fully functional by fixing two critical bugs and running the complete 5-stage pipeline.

## Bugs Fixed

### 1. SDF Carving Performance (~100× speedup)

**Problem**: `_carve_graph()` computed a full-grid SDF sphere for every spline point. With ~1,080 sphere operations on a 12.7M voxel grid (323×309×127), generation would hang indefinitely.

**Fix**: Added `_carve_sphere_local()` that uses bounding-box slicing — only touches voxels within `2× radius` of each sphere centre. A 5m-radius sphere now touches ~4,000 voxels instead of 12.7M.

**Result**: Full 80×80×25m cave at 0.5m resolution generates in **< 1 second** (mesh step), down from 5+ minutes.

> [!IMPORTANT]
> File modified: [cave_mesh.py](file:///home/smalld/Downloads/space_robo_proj/cave_drone_sim/src/cave_generator/cave_generator/cave_mesh.py)

### 2. OpenSimplex Import Crash

**Problem**: `opensimplex` imported `numba`, which crashed with `AttributeError: module 'coverage' has no attribute 'types'`. The `except ImportError` didn't catch this.

**Fix**: 
- Broadened exception handling to `except Exception`
- Added 3-tier noise fallback: opensimplex → `noise` package → pure-numpy multi-octave sin-based pseudo-noise
- Cave generation now **always succeeds** regardless of installed packages

## Pipeline Output

Generated cave with: `--nodes 25 --bounds 80 80 25 --seed 42 --resolution 0.5`

| Stage | Output | Time |
|-------|--------|------|
| 1. Graph topology | 25 nodes, 27 edges, 5 squeeze passages | < 0.1s |
| 2. Mesh generation | 50,169 verts / 100,342 faces | 0.8s |
| 3. Obstacle placement | 200 stalactites + rubble | ~4s |
| 4. Gazebo export | model.sdf + cave_world.sdf | < 0.1s |
| 5. Debug visualizations | 4 PNG files | ~2s |

**Total: ~7 seconds** for a full-size cave.

## Generated Files

```
output/cave_001/
├── cave_graph.json              ← topology (nodes + edges)
├── cave_world.sdf               ← Gazebo world file
├── cave_model_001/
│   ├── model.config
│   ├── model.sdf
│   └── meshes/
│       ├── cave_visual.obj      ← high-res visual mesh
│       └── cave_collision.stl   ← collision geometry
├── obstacles/
│   ├── stalactite_0000.stl … stalactite_0199.stl
│   └── rubble_0000.stl …
└── debug/
    ├── graph.png                ← 3D graph topology
    ├── splines.png              ← catmull-rom tunnel paths  
    ├── sdf_slice.png            ← SDF cross-section at Z=0
    └── composite.png            ← all three combined
```

## Debug Visualizations

### Graph Topology (25 nodes, MST + loop edges)
![Cave graph topology showing 25 chamber nodes connected by passages](/home/smalld/Downloads/space_robo_proj/cave_drone_sim/output/cave_001/debug/graph.png)

### SDF Cross-Section (Z=0 slice)
![SDF field cross-section showing carved tunnels and chambers in blue against solid rock in red](/home/smalld/Downloads/space_robo_proj/cave_drone_sim/output/cave_001/debug/sdf_slice.png)

### Composite View
![Composite debug view with graph, splines, and SDF slice](/home/smalld/Downloads/space_robo_proj/cave_drone_sim/output/cave_001/debug/composite.png)



## How to Run

```bash
# Basic generation
python3 scripts/generate_cave.py --nodes 25 --seed 42 --output ./output/cave_001

# Quick test (smaller cave, coarser resolution)
python3 scripts/generate_cave.py --nodes 10 --bounds 40 40 15 --resolution 0.8 --output ./output/quick_test

# Full with visualizations
python3 scripts/generate_cave.py --nodes 25 --bounds 80 80 25 --seed 42 --resolution 0.5 --output ./output/cave_001 --visualize

# Skip obstacles and Gazebo export (mesh-only)
python3 scripts/generate_cave.py --no-obstacles --no-gazebo --visualize
```

ired even when the drone hit a wall. Now transit uses   
  position-based arrival (dist < 3.5 m). The drone won't   
  advance to the next waypoint until it physically gets  
  there. TRANSIT_MAX_SECS=120s is the only fallback.       
                                                           
  Going through walls / bad SLAM map — LiDAR polar         
  histogram is back. Every time a 2m step completes, the
  next step is recomputed from the histogram: goal         
  direction checked first, then _best_sector() if blocked
  (high W_ALIGN=0.65 keeps the drone tracking toward the   
  target chamber instead of wandering). The drone now stays
   inside tunnels.

  New cave auto-detection — _find_cave() picks the most    
  recently modified cave_graph.json under output/. You can
  also pass --cave:                                        
                                                         
  python3 scripts/generate_cave.py --nodes 25 --seed 42
  --output ./output/cave_001                               
   
  # Auto-detects the new cave_001:                         
  python3 scripts/cave_explorer.py                       

  # Or explicit:
  python3 scripts/cave_explorer.py --cave ./output/cave_002

  Better SLAM map — SCAN_SECS=15s (up from 12s) gives each 
  chamber longer dwell time, STEP_M=1.5m at STEP_SECS=2.0s
  = 0.75 m/s transit speed (slow enough for SLAM to keep   
  up). The drone no longer exits the cave geometry so LiDAR
   data stays geometrically consistent.

✻ Churned for 12m 31s


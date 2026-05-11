### Using Bullet as a collision checker [experimental]
To use Bullet as a collision checker you can manually switch to Bullet using the planning_scene.setActiveCollisionDetector()

### Speed Benchmarks
For speed comparisons to FCL please see (relative link to README of benchmark script will be added).

### Current Limitations
Collision detection using Bullet is thread safe but parallelization does not improve performance.

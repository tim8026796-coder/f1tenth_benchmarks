# Module Summary

| Module | Main Function | Purpose | Current Limitation |
|--------|---------------|---------|--------------------|
| Prediction V1 | `predict_polyline` / `predict_candidate` | Generate timed future positions by XY-polyline arc length | Constant speed; no acceleration; open finite path; yaw from segment tangent |
| Collision V1 | `evaluate_collision` | Evaluate contact between two explicitly time-aligned predictions | Discrete-time only; bounding-circle approximation; no continuous sweep or control approval |

Both modules keep `inspection_only=true` and `planning_allowed=false`.

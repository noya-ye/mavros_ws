# Diff-Planner source provenance

Upstream: https://github.com/DifferentialRobotics/Diff-Planner

Pinned commit: `5f8551203426b371c22de55e5961d04cf7639c60`.

The three headers in `include/ego_2d_planner_pkg/diff_planner/vendor/` are copied
unchanged from upstream `src/diff_planner/traj_opt/include/optimizer/`:
`poly_traj_utils.hpp`, `root_finder.hpp`, and `lbfgs.hpp`. They supply the actual
MINCO minimum-jerk banded solver, its adjoint gradients, polynomial utilities,
and L-BFGS. The upstream GPL-3.0 license is retained in this directory.

`DiffOptimizer2D` adapts Diff's virtual-time mapping, quadrature P/V/A penalties,
obstacle base-point/direction constraints and gradient propagation to the existing
ROS 2 interfaces. Only XY waypoint coordinates and piece times are variables;
the vendor solver's third spatial column stays identically zero. Flight height
is attached at execution. ROS 1, swarm, depth-camera and attitude-control modules
are not included.

The virtual-time mapping adds a 0.05 s lower bound per piece. Very short goals
use at least two pieces: upstream's single-piece shortcut omits the factorized
adjoint system and time-power arrays required by optimization.

The old EGO cubic optimizer and evaluator remain as historical reference files
but are absent from the core build and execution graph.

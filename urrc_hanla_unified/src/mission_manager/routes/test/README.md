# Test-only reference routes

These synthetic routes are for deterministic software tests. They are not surveyed competition routes.

The integrated workspace uses numeric course sections required by the route
loader: former NORMAL samples use section 1, SLOPE uses 2, T_PARK uses 7,
and PARALLEL_PARK uses 10. These assignments describe these synthetic fixtures;
they are not a general conversion rule for surveyed routes.

`07_circle_loop` is the RViz-only 3 m radius, 72-waypoint, forward-only loop.

`08_circle_forward_max27` and `09_circle_reverse_max27` are 144-point,
wheelbase-derived max-steering validation loops. They do not replace a real route.

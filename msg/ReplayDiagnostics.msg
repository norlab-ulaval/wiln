builtin_interfaces/Time stamp

float64 dist_to_path_m       # Lateral deviation from trajectory
float64 heading_error_rad    # Heading error relative to trajectory
uint32  closest_global_idx   # Index of closest pose in full trajectory

float64 deform_time_ms       # Time spent in PathDeformer::deform()
uint32  obstacle_count       # Number of obstacle points used for deformation
float64 max_displacement_m   # Maximum path deformation applied
bool    deform_fallback      # True if last valid plan was reused (deform failed)
bool    time_budget_exceeded # True if deform exceeded its time budget

float64 kappa_max_current    # Current max curvature (varies with articulation for MTT)

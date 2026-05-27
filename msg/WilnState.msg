builtin_interfaces/Time stamp

# State constants
uint8 IDLE      = 0
uint8 RECORDING = 1
uint8 PLAYING   = 2
uint8 LOADING   = 3
uint8 SAVING    = 4
uint8 FAULTED   = 5

uint8  state
string detail            # Human-readable reason for current state
string active_route      # Filename of loaded route (empty if none)
uint32 trajectory_poses  # Total pose count in loaded/recorded trajectory
float64 trajectory_length_m

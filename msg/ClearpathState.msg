#
# Message for "raw" state from a Clearpath SC motor
#

Header header

int32 serial_number

# Motor position in raw counts
int64 position

# Rad per count
float32 position_resolution

# Motor position in raw counts
int64 velocity

# Rad/sec per count
float32 velocity_resolution

# Current in amps
float32 current

bool enabled

# Protocol
This document describes the protocol for communication with the rotator being
developed by the UNL Aerospace Rocketry Club.

All commands are 4 characters long. All arguments are separated by spaces.
Commands are terminated with a newline. The final argument of all commands must
be a checksum (TBD).

Optional arguments are specified with `{}`.

Horizontal axis is azimuth, vertical axis is altitude.

Positive and negative must be specified.

Commands respond with `OK\n` if successful, `ERR <REASON>\n` if not. Some commands
respond with return arguments after the `OK` and before the `\n`.

# Version 1.2.0

## Types
command:    `ABCD` # 4 ASCII characters

string arg: `ABC`  # 3 ASCII characters

float:      `1234.567` or `-1234.567` # Signed floating point, unlimited number of leading and trailing digits

integer:    `1234` or `-1234` # Signed integer unlimited number of digits


## Commands
### `DVER`
Args: `float`

Description: Degrees to move to in the vertical axis.

### `DHOR`
Args: `float`

Description: Degrees to move to in the horizontal axis.

### `CALV`
Args: `{SET}`

Description: Automagically calibrates vertical axis. Specifying SET sets 
vertical calibration position to current position.

### `CALH`
Args:

Description: Sets horizontal calibration position to current position.

### `MOVV`
Args: `integer`

Description: Moves by the specified number of steps in the vertical axis.

### `MOVH`
Args: `integer`

Description: Moves by the specified number of steps in the horizontal axis.

### `GETP`
Args:

Returns: `float float`

Description: Gets the current position for both Vertical then Horizontal.

### `INFO`
Args:

Returns: `TBD`

Description: Gets info!

### `SSPD`
Args: `integer, {VER integer}, {HOR integer}, {RST}`

Description: Sets speed for reqested axes. if no axis requested sets speed for both.

### `GSPD`
Args:

Returns: `integer`

Description: Gets speed for both axes.

### `VERS`
Args:

Returns: `string`

Description: Gets the current version of the software.

### `GETC`
Args:

Returns: `string`

Description: Returns `true` or `false` depending on the current calibration status.

### `HALT`
Args:

Description: Immediately stops both motors by locking them to perform an emergency stop.

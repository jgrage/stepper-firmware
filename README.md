## Serial communication

The controller expects a serial connection with 9600 baud and 8-N-1 configuration. All command strings expect a line feed character (\n) at the end.
The commands are in SCPI style. For every keyword in the command tree, there is both a long and a short version. All commands shown in the tables below show both versions. The uppercase characters show the short keyword and the lowercase characters show the completion to the full keyword.
This means, :SYSTem:ERRor? expands to either :SYST:ERR?, :SYST:ERROR?, :SYSTEM:ERR? or :SYSTEM:ERROR? which are all valid commands.

### System commands
Possible motor states are "MOVING", "STOPPED", "LIM+", "LIM-", "FAULT".

| command        | action                    |
|----------------|---------------------------|
| *IDN?          | get identification string |
| :SYSTem:ERRor? | print last error message  |
| :MOTor:STate?  | get motor status          |
### Movement
The motor driver uses 4 microsteps per step. However, all position values are in full steps. All move commands expect a float (.25 for one microstep) or an integer value.
The controller has no non-volatile-memory. All values will be set to a default value on startup. Since the position counter is initialized with 0.00, it is recommended to save the position counter value to a file before shutdown. After startup, the previous position can be restored from that file.

| command                   | action                             |
|---------------------------|------------------------------------|
| :MOTor:POSition?          | get position counter value         |
| :MOTor:POSition $val      | set position counter to $val       |
| :MOTor:MOVe:RELative $val | move $val steps                    |
| :MOTor:MOVe:ABSolute $val | move motor to position $val        |
| :MOTor:STOP               | stop current movement              |


### Configuration
All get commands return an integer value of the current speed, acceleration or deceleration. Set commands accept both integer and floating point numbers but will be rounded to the next integer.
Non default values will be reset to the default settings after restarting the controller. Calling the set functions with arguments "DEFAULT", "MIN" or "MAX" is also possible.
TODO: add commands to change microstepping mode
TODO: maybe constrain set values to the range MIN-MAX.

| command                  | action                             | default | min | max |
|--------------------------|------------------------------------|---------------------|
| :MOTor:SPeed?            | returns top speed in steps/s       |         |     |     |
| :MOTor:SPeed $val        | sets top speed to $val steps/s     | 200     | 10  | 800 |
| :MOTor:ACCeleration?     | returns acceleration in steps/s²   |         |     |     |
| :MOTor:ACCeleration $val | sets acceleration to $val steps/s² | 100     | 10  | 400 |
| :MOTor:DECeleration?     | returns deceleration in steps/s²   |         |     |     |
| :MOTor:DECeleration $val | sets deceleration to $val steps/s² | 100     | 10  | 400 |

### Limits
The controller supports mechanical limit switches for protection and referencing. Once a switch is activated, the motor state turns to "LIM+" ("LIM-") for the positive (negative) limit switch. Activation of both switches results in a "FAULT" state.
Additionally, softlimits can be set to custom positions. The set commands expect both integer and floating point numbers. The softlimits will be reset to their default values after restart.
If a movement command violates the softlimits, the motor will not move and instead an error message will be pushed onto the error buffer. Receive the error message by issuing the :SYST:ERR? command to the controller.

| command                    | action                            |
|----------------------------|-----------------------------------|
| :MOTor:LIMit:POSitive $val | set $val as positive softlimit    |
| :MOTor:LIMit:POSitive?     | get positive softlimit value      |
| :MOTor:LIMit:NEGative $val | set $val as negative softlimit    |
| :MOTor:LIMit:NEGative?     | get negative softlimit value      |
| :MOTor:HOMe:POSitive       | home run to positive limit switch |
| :MOTor:HOMe:NEGative       | home run to negative limit switch |

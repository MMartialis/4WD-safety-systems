# 4WD safety systems

This is a university project for creating a microcontroller program, that adds safety functionalities to an electric vehicle such as an electric skateboard. This 3rd-semester project is being done at Aalborg University Esbjerg on the Applied Industrial Engineering study program.

## Usage

The project is developed using platform.io, and is tested on an ESP32-C3-MINI-1 microprocessor.

## Features

It is planned to add the following safety features to the setup:
- Logging motor properties like rpm and current load
- Detecting unsafe conditions, like slipping, sliding, brake blocking, etc.
- Modifying motor controls in order to return to safe driving conditions
- These systems can be
  - ABS (Anti-lock Braking System)
  - ASC (Automatic Stability Control)
  - TCS (Traction Control System)
  - Torque vectoring
  - Speed wobble prevention system

## Serial commands

"ABS"					          -- return if ABS function is enalbled or not
"ABS 0"				        	-- turn off ABS
"ABS 1"				        	-- turn on abs
"ESTOP"                 -- enable estop, set estop to 1
"RESET"                 -- reset estop, set estop to 0
"STATUS"                -- return status
"STATUS 0"              -- set status
"RESTART"               -- restart


## License

[MIT](https://choosealicense.com/licenses/mit/)
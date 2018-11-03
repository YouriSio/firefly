<img src="http://fireflyeindhoven.nl/wp-content/uploads/2017/06/cropped-Artboard-1-2.png" width="30%">

The Github repository for Firefly.

## Teams
The following teams are working in this repository:

#### Modeling
Consisting of Thomas, Bram, Endi and Aabharan. Check out their [branch](https://github.com/wagenaartje/Firefly/tree/modeling). To-do:

 - [x] Refresh file saving
 - [ ] Clean up file system
 - [ ] Fix bug in 6-DOF-Euler Angles, crash at 90 degrees
 - [ ] Fix bug in clicks2traj, only double click results in crash
 - [ ] Make simulink output actual trajectory
 - [ ] Make simulink actually predict trajectory
 - [ ] Make sure drone stops at last point
 - [ ] Simulink should optimize trajectory (don't overshoot points)

#### Installation
No installation is required besides downloading the repository and the MATLAB Toolboxes necessary. Note that when `websocket.m` is ran, it will automatically add a `.jar` file to the MATLAB Java static path and restart MATLAB.

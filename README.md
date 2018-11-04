<img src="http://fireflyeindhoven.nl/wp-content/uploads/2017/06/cropped-Artboard-1-2.png" width="30%">

The Github repository for Firefly.


**Warning:. Always add/rename/delete files using the Simulink project pane. 
This ensures that references to the file will be changed everywhere.**

## To-do list

 - [x] Refresh file saving
 - [x] Clean up file system
 - [ ] Fix bug in 6-DOF-Euler Angles, crash at 90 degrees
 - [x] Make simulink output actual trajectory
 - [x] Make simulink actually predict trajectory
 - [ ] Make sure drone stops at last point
 - [ ] Get rid of `evalin()` and `assignin` calls
 - [ ] Optimize actual trajectory to reference trajectory
 - [ ] Implement actual trajectory checks through MSE and collision detection
 - [ ] Pass the actual trajectory and information back to the browser

## Installation
No installation is required besides downloading the repository and the 
MATLAB Toolboxes necessary. Note that when `websocket.m` is ran, it will 
automatically add a `.jar` file to the MATLAB Java static path and restart 
MATLAB. Furthermore, before running any scripts: open the project. This
ensures that all `.m` files in subfolders are in the MATLAB path.

## Usage

1. Download the files
2. Open `firefly.prj` in MATLAB
3. Run `m/run.m`
4. Create trajectory on `https://yourisio.github.io/trajectory-editor/`
5. Press export

What will happen next is as follows:

1. The MATLAB server gets the waypoints and passes them to `checktrajectory.m`
2. `points2reftraj.m` converts the points to a mathematical optimal reference 
trajectory using `lqtrajgeneration.m`
3. `points2actualtraj.m` converts the reference trajectory to an actual
trajectory by simulating
4. `points2actualtraj.m` plots the waypoints, reference trajectory and
actual trajectory for comparison
5. All trajectories are saved in a unique ID folder in the `data` folder
6. `checktrajectory.m` performs checks on the actual trajectory to see if 
the reference trajectory is safe and doable (not yet done)
7 . The actual trajectory will be passed back to the browser with information
about safety and possibility


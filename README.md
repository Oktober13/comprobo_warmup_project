# comprobo_warmup_project
## Project summary

## Teleoperation UI

I implemented a teleoperation UI to control a Neato ground robot remotely using standard terminal input. The arrow keys control linear and angular motion, for ease of memory, the number keys change the speed of the robot, and the space bar is a large, easy-to-hit emergency stop and will cancel all motion. Pressing an arrow key begins motion, while double pressing it cancels the motion. Contradictory motions, such as moving forwards and backwards simultaneously, or changing speeds, will cancel the previous contradicting command.

By isolating drive commands, it becomes possible to control both linear and angular velocities simultaneously through stdin, whereas stdin can normally only accept one keypress command at a time. This allows for intuitive, natural-looking control of the robot.

Closing out of the program via CTRL-C or other handled exceptions will result in an exit handler running and cancelling the robot’s current velocity.

One difficulty I encountered the process of writing this teleoperation UI was arrow key presses. Arrow keys are actually a combination of three simultaneous key presses: the ‘ESC’ key, the ‘]’ key, and a capital letter A through D. It was difficult to decode three keypresses with single character input, as each keypress was ostensibly a separate event, but was in reality one arrow keypress. While this issue would have been easily avoided by using letter keys instead, I felt strongly that using arrow keys to control motion was more intuitive.

If I had more time in the future, I would make changing speeds dynamic, so that the new speed is instantly applied to the previous command(s) retroactively. In addition, I would investigate alternative keyboard inputs, such as pycharm or curses, which would capture keypresses independently, avoiding single character input bottlenecks.

## Square Drive behavior

I created a Neato behavior to drive in a square by intuiting that this behavior was actually the combination of two behaviors- a straight drive, and a turn. By using a timer and my knowledge of the Neato’s current linear and angular velocity, I was able to predict when the Neato had completed a side or a turn, and issue the next velocity command to the rostopic /neato_node/cmd_vel in a loop.

Using timers to generate the square path allowed me to quickly generate a fairly accurate square shape, but it was a fairly rigid architecture; any change in the planned path meant the Neato had no way to react or adjust its route.

If I were to return to this function, I would likely base the turn timing implementation on the odometry and accelerometer. This would allow for the implementation of more complex behaviors such as regaining pose estimation after being shoved or picked up.

## Wall Follow behavior

I implemented a wall following behavior consisting of two steps: finding the LIDAR scan angle with the smallest distance (the wall), finding the error between the current angle and that LIDAR scan angle, then using proportional control to change the angular velocity to course correct. The scan angle is visualized in rviz as a blue arrow pointing from the center of the Neato at the desired angle, to a point on the unit circle. When the error is greater than 20 degrees, the linear velocity is set to 0 to focus on the turning behavior; otherwise, it is set to the maximum speed (0.1 m/s). The new velocity Twist message is then published to the /neato_node/cmd_vel topic at a rate of 10 times a second. Wall following ends when the program is killed or when the bump sensor is triggered, at which point a cleanup function will kill any Neato velocity and gracefully exit the program.

The wall follow behavior contains relatively simple conditions, but the overall challenge of creating this module was fairly high. I decided to create a proportional control machine as I was running low on time at this point. This proved to be quite challenging; the hardest part of this project turned out to be the parameter tuning. Too small of a value, and corrections would be miniscule; too much correction and the Neato would spin wildly. In the end, I included functionality for later integral and derivative correction , but was only able to tune the gain for the proportional error correction. Finally, one additional change I would make is to shift the wall determination criteria to be a cluster of points, rather than a single point that is close to the Neato. By finding the major axis of the cluster ellipse, I could find the line equation for the position of the wall at any given point. This would allow me to determine where corners are, and would also provide a steady “desirable” angle to follow that will not shift from LIDAR scan to LIDAR scan, both of which are more useful behaviors than chasing the closest thing.

## Person Follow behavior

## Obstacle Avoidance

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

The person follow behaves similarly to the wall follow behavior- a 0-360 degree scan of distances is collected with the LIDAR, and the closest angle becomes the desired angle of travel. The person follow differs from the wall follower in that I got tired of trying to tune the proportional gain, so I implemented a simpler system that simply tracks whether the desired angle is less than, greater than, or equal to the current pose orientation angle, then sets the angular velocity to -max speed, max speed, and 0 respectively. The new velocity Twist message is then published to the /neato_node/cmd_vel topic at a rate of 10 times a second. This approach worked considerably better than the finicky proportional control, and I was able to capture a rosbag of the Neato following me. Once again, similar to the wall follower, the desired angle is visualized in rvis as a blue arrow pointing from the center of the Neato at the desired angle, to a point on the unit circle. Person following also ends when the program is killed or when the bump sensor is triggered, at which point a cleanup function will kill any Neato velocity and gracefully exit the program.
One difficulty that I had was in getting the Neato to respond as quickly as I could move; my program was pretty laggy.

If I had more time to revise this behavior, I would cluster the distance datapoints, constraining clusters to those about the ellipse size of human legs, which would eliminate false positives with other obstacles. I would also make speed proportional to the distance or angle that must be covered. Lastly, I would investigate the source of the lag, and see whether there were any parameters I could tune.

## Obstacle Avoidance

I implemented a probabilistic obstacle avoidance ability. First, from 0-360 degrees, I collect the distances returned by the LIDAR scan into an array, using a ROS subscriber and a callback I wrote previously and reused. This array is converted into a series of coordinate points relative to the Neato, which is then clustered using Density Based Scanning (DBSCAN). I chose to use DBSCAN as I was looking for a clustering algorithm that would be able to generate clusters and cluster centroids without knowing the number of categories. As an algorithm centered around finding the point with the most neighbors (point density measurement), DBSCAN was ideal for this purpose, particularly as most of my clusters were elliptical or circular in shape.

The coordinates of the centroid, or core sample, of each cluster were converted back into an angle, and used to generate negative normal probability distributions. A positive normal probability distribution was generated using the angle to the goal. These probabilities were all summed and the resultant array normalized. The index of the resultant array (angle of maximum probability) is our desired angle.

I reused my wall follower’s proportional control code to determine the appropriate angular velocity changes to reach the desired angle.

I’m still in the process of debugging this behavior- so far, the most challenging problem has been trying to identify why the desired angle is not changing as the orientation of the Neato changes.

If I had more time, I would continue to debug the desired angle code, tune the rest of the PID parameters, and ensure this code can work with teleop as well.

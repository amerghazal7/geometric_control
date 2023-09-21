#!/usr/bin/env python3

import math
from turtle import pos, position
from controller2d import Controller2D
import rospy, time
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
import numpy as np
import csv
import matplotlib
matplotlib.use("Agg")  # or whichever backend you wish to use
import matplotlib.pyplot as plt


import live_plotter as lv   # Custom live plotting library
PLAYER_START_INDEX = 1      # spawn index for player (keep to 1)
FIGSIZE_X_INCHES   = 16      # x figure size of feedback in inches
FIGSIZE_Y_INCHES   = 8      # y figure size of feedback in inches
PLOT_LEFT          = 0.1    # in fractions of figure width and height
PLOT_BOT           = 0.1    
PLOT_WIDTH         = 0.8
PLOT_HEIGHT        = 1
live_plot_period   = 0.1

class Timer(object):
    """ Timer Class
    
    The steps are used to calculate FPS, while the lap or seconds since lap is
    used to compute elapsed time.
    """
    def __init__(self, period):
        self.step = 0
        self._lap_step = 0
        self._lap_time = time.time()
        self._period_for_lap = period

    def tick(self):
        self.step += 1

    def has_exceeded_lap_period(self):
        if self.elapsed_seconds_since_lap() >= self._period_for_lap:
            return True
        else:
            return False

    def lap(self):
        self._lap_step = self.step
        self._lap_time = time.time()

    def ticks_per_second(self):
        return float(self.step - self._lap_step) /\
                     self.elapsed_seconds_since_lap()

    def elapsed_seconds_since_lap(self):
        return time.time() - self._lap_time

live_plot_timer = Timer(live_plot_period)
TOTAL_EPISODE_FRAMES = 2300
INTERP_MAX_POINTS_PLOT = 10
INTERP_LOOKAHEAD_DISTANCE = 5   # lookahead in meters
INTERP_DISTANCE_RES       = 0.5 # distance between interpolated points


enable_live_plot = False
odom_topic = 'odom'
waypoints_file = ""
waypoints = []
waypoints_np = []
waypoints_sorted = []
wp_distance = []
wp_interp = []
wp_interp_hash = []
waypoints_window = 5
waypoints_curr = 0
position = [0, 0]
p_position = [0, 0]
yaw = 0
v = 0
start_t = time.time()
t = time.time()
p_t = time.time()
output_throttle = 0
output_steering = 0
output_break    = 0
DIST_THRESHOLD_TO_LAST_WAYPOINT = 0.7
reached_end = False
closest_index    = 0
dt = 0.0001
throttle_publisher = None
steering_publisher = None

controller: Controller2D = None

def find_nearest():
    global closest_index
    closest_distance = np.linalg.norm(np.array([
                    waypoints_np[closest_index, 0] - position[0],
                    waypoints_np[closest_index, 1] - position[1]]))
    new_distance = closest_distance
    new_index = closest_index
    while new_distance <= closest_distance:
        closest_distance = new_distance
        closest_index = new_index
        new_index += 1
        if new_index >= waypoints_np.shape[0]:  # End of path
            break
        new_distance = np.linalg.norm(np.array([
                waypoints_np[new_index, 0] - position[0],
                waypoints_np[new_index, 1] - position[1]]))
    new_distance = closest_distance
    new_index = closest_index
    while new_distance <= closest_distance:
        closest_distance = new_distance
        closest_index = new_index
        new_index -= 1
        if new_index < 0:  # Beginning of path
            break
        new_distance = np.linalg.norm(np.array([
                waypoints_np[new_index, 0] - position[0],
                waypoints_np[new_index, 1] - position[1]]))

    # Once the closest index is found, return the path that has 1
    # waypoint behind and X waypoints ahead, where X is the index
    # that has a lookahead distance specified by 
    # INTERP_LOOKAHEAD_DISTANCE
    waypoint_subset_first_index = closest_index - 1
    if waypoint_subset_first_index < 0:
        waypoint_subset_first_index = 0

    waypoint_subset_last_index = closest_index
    total_distance_ahead = 0
    while total_distance_ahead < INTERP_LOOKAHEAD_DISTANCE:
        total_distance_ahead += wp_distance[waypoint_subset_last_index]
        waypoint_subset_last_index += 1
        if waypoint_subset_last_index >= waypoints_np.shape[0]:
            waypoint_subset_last_index = waypoints_np.shape[0] - 1
            break

    # Use the first and last waypoint subset indices into the hash
    # table to obtain the first and last indicies for the interpolated
    # list. Update the interpolated waypoints to the controller
    # for the next controller update.
    new_waypoints = \
            wp_interp[wp_interp_hash[waypoint_subset_first_index]:\
                        wp_interp_hash[waypoint_subset_last_index] + 1]
    return new_waypoints

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def apply_control(new_waypoints):
    global reached_end, controller, waypoints, waypoints_window, position, yaw, v, t, output_steering, output_throttle, output_break, dt
    
    controller.update_waypoints(new_waypoints)
    controller.update_values(position[0], position[1], yaw, v, t - start_t, True)
    controller.update_controls(dt)

    output_throttle, output_steering, output_break = controller.get_commands()
    # output_throttle = v + output_throttle * dt - output_break * dt
    output_throttle = min(max(0, output_throttle), 2.0)
    rospy.loginfo(f"OUT DR: {output_throttle}")
    rospy.loginfo(f"OUT ST: {output_steering}")
    output_throttle = int(translate(output_throttle, 0, 2, 50, 100))
    output_throttle = min(output_throttle, 60)

    # output_steering = int(translate(-1 * output_steering, -0.70, 0.70, 0, 100))
    output_steering = int(translate(-1 * output_steering, -1, 1, 0, 100))

    dist = np.linalg.norm(np.asarray([position[0], position[1]]) - waypoints[-1][:2])
    if dist <= DIST_THRESHOLD_TO_LAST_WAYPOINT:
        reached_end = True


def odometryCb(msg:Odometry):
    global position, yaw, v, t, p_t, p_position, controller, dt
    position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    t = time.time()
    dt = t - p_t
    dis = math.sqrt((position[0] - p_position[0])**2 + (position[1] - p_position[1])**2)
    v = max(dis / dt, 0.001)

    controller.update_values(position[0], position[1], yaw, v, t, True)
    
    p_position = position
    p_t = t


def init_pub_sub():
    global position, wp_distance, wp_interp, wp_interp_hash, enable_live_plot, reached_end, controller, odom_topic, waypoints_file, waypoints_np, waypoints, waypoints_sorted, throttle_publisher, steering_publisher, output_throttle, output_steering

    rospy.init_node('geometric_control', anonymous=True)

    enable_live_plot = bool(rospy.get_param('~enable_plotter', default="False"))
    odom_topic = rospy.get_param('~odom_topic', default="odom")
    waypoints_file = rospy.get_param('~waypoints_file', default="") # should be csv file
    if waypoints_file == "" or (".csv" not in waypoints_file):
        rospy.loginfo("Waypoints file is incorrect, exitting!")
        return
    
    with open(waypoints_file) as waypoints_file_handle:    
        waypoints = list(csv.reader(waypoints_file_handle, 
                                            delimiter=',',
                                            quoting=csv.QUOTE_NONNUMERIC))
        waypoints_np = np.array(waypoints)
        waypoints_np[:, 1] = waypoints_np[:, 1]

    wp_distance = []   # distance array
    for i in range(1, waypoints_np.shape[0]):
        wp_distance.append(
                np.sqrt((waypoints_np[i, 0] - waypoints_np[i-1, 0])**2 +
                        (waypoints_np[i, 1] - waypoints_np[i-1, 1])**2))
    wp_distance.append(0)  # last distance is 0 because it is the distance
                            # from the last waypoint to the last waypoint

    # Linearly interpolate between waypoints and store in a list
    wp_interp      = []    # interpolated values 
                            # (rows = waypoints, columns = [x, y, v])
    wp_interp_hash = []    # hash table which indexes waypoints_np
                            # to the index of the waypoint in wp_interp
    interp_counter = 0     # counter for current interpolated point index
    for i in range(waypoints_np.shape[0] - 1):
        # Add original waypoint to interpolated waypoints list (and append
        # it to the hash table)
        wp_interp.append(list(waypoints_np[i]))
        wp_interp_hash.append(interp_counter)   
        interp_counter+=1
        
        # Interpolate to the next waypoint. First compute the number of
        # points to interpolate based on the desired resolution and
        # incrementally add interpolated points until the next waypoint
        # is about to be reached.
        num_pts_to_interp = int(np.floor(wp_distance[i] /\
                                        float(INTERP_DISTANCE_RES)) - 1)
        wp_vector = waypoints_np[i+1] - waypoints_np[i]
        wp_uvector = wp_vector / np.linalg.norm(wp_vector)
        for j in range(num_pts_to_interp):
            next_wp_vector = INTERP_DISTANCE_RES * float(j+1) * wp_uvector
            wp_interp.append(list(waypoints_np[i] + next_wp_vector))
            interp_counter+=1
    # add last waypoint at the end
    wp_interp.append(list(waypoints_np[-1]))
    wp_interp_hash.append(interp_counter)   
    interp_counter += 1
    new_waypoints = waypoints_np
    # waypoints_sorted = waypoints[waypoints[:,1].argsort(kind='mergesort')]
    # waypoints_sorted = waypoints_sorted[waypoints_sorted[:,0].argsort(kind='mergesort')]
    
    lp_traj = lv.LivePlotter(tk_title="Trajectory Trace")
    lp_1d = lv.LivePlotter(tk_title="Controls Feedback")
    
    trajectory_fig = lp_traj.plot_new_dynamic_2d_figure(
            title='Vehicle Trajectory',
            figsize=(FIGSIZE_X_INCHES, FIGSIZE_Y_INCHES),
            edgecolor="black",
            rect=[PLOT_LEFT, PLOT_BOT, PLOT_WIDTH, PLOT_HEIGHT])
    
    # trajectory_fig.set_invert_x_axis() # Because UE4 uses left-handed 
                                        # coordinate system the X
                                        # axis in the graph is flipped
    trajectory_fig.set_axis_equal()    # X-Y spacing should be equal in size

    # Add waypoint markers
    trajectory_fig.add_graph("waypoints", window_size=waypoints_np.shape[0],
                                x0=waypoints_np[:,0], y0=waypoints_np[:,1],
                                linestyle="-", marker="", color='g')
    
    # Add trajectory markers
    trajectory_fig.add_graph("trajectory", window_size=TOTAL_EPISODE_FRAMES,
                                x0=[position[0]]*TOTAL_EPISODE_FRAMES, 
                                y0=[position[1]]*TOTAL_EPISODE_FRAMES,
                                color=[1, 0.5, 0])

    trajectory_fig.add_graph("start_pos", window_size=1, 
                                 x0=[0], y0=[0],
                                 marker=11, color=[1, 0.5, 0], 
                                 markertext="Start", marker_text_offset=1)
    # Add end position marker
    trajectory_fig.add_graph("end_pos", window_size=1, 
                                x0=[waypoints_np[-1, 0]], 
                                y0=[waypoints_np[-1, 1]],
                                marker="D", color='r', 
                                markertext="End", marker_text_offset=1)
    
    # Add car marker
    trajectory_fig.add_graph("car", window_size=1, 
                                marker="s", color='b', markertext="Car",
                                marker_text_offset=1)

    # Add lookahead path
    trajectory_fig.add_graph("lookahead_path", 
                                window_size=INTERP_MAX_POINTS_PLOT,
                                x0=[position[0]]*INTERP_MAX_POINTS_PLOT, 
                                y0=[position[1]]*INTERP_MAX_POINTS_PLOT,
                                color=[0, 0.7, 0.7],
                                linewidth=4)


    ###
    # Add 1D speed profile updater
    ###
    forward_speed_fig =\
            lp_1d.plot_new_dynamic_figure(title="Forward Speed (m/s)")
    forward_speed_fig.add_graph("forward_speed", 
                                label="forward_speed", 
                                window_size=TOTAL_EPISODE_FRAMES)
    forward_speed_fig.add_graph("reference_signal", 
                                label="reference_Signal", 
                                window_size=TOTAL_EPISODE_FRAMES)

    # Add throttle signals graph
    throttle_fig = lp_1d.plot_new_dynamic_figure(title="Throttle")
    throttle_fig.add_graph("throttle", 
                            label="throttle", 
                            window_size=TOTAL_EPISODE_FRAMES)
    # Add brake signals graph
    brake_fig = lp_1d.plot_new_dynamic_figure(title="Brake")
    brake_fig.add_graph("brake", 
                            label="brake", 
                            window_size=TOTAL_EPISODE_FRAMES)
    # Add steering signals graph
    steer_fig = lp_1d.plot_new_dynamic_figure(title="Steer")
    steer_fig.add_graph("steer", 
                            label="steer", 
                            window_size=TOTAL_EPISODE_FRAMES)


    if not enable_live_plot:
            lp_traj._root.withdraw()
            lp_1d._root.withdraw()  


    controller = Controller2D(waypoints=waypoints)
    rospy.Subscriber(odom_topic, Odometry, odometryCb)
    # rospy.spin()

    throttle_publisher = rospy.Publisher('/velocity', Float32, queue_size=10)
    steering_publisher = rospy.Publisher('/steering_rad', Float32, queue_size=10)
    rate = rospy.Rate(10) # 10hFlopublishat32
    while not rospy.is_shutdown():
        if reached_end:
            rospy.loginfo("Reached the end!!")
            output_steering = output_throttle = 0
        else:
            new_waypoints = find_nearest()
            apply_control(new_waypoints)
        throttle_publisher.publish(output_throttle)
        steering_publisher.publish(output_steering)

        # Update live plotter with new feedback
        trajectory_fig.roll("trajectory", position[0], position[1])
        trajectory_fig.roll("car", position[0], position[1])
        # When plotting lookahead path, only plot a number of points
        # (INTERP_MAX_POINTS_PLOT amount of points). This is meant
        # to decrease load when live plotting
        new_waypoints_np = np.array(new_waypoints)
        path_indices = np.floor(np.linspace(0, 
                                            new_waypoints_np.shape[0]-1,
                                            INTERP_MAX_POINTS_PLOT))
        trajectory_fig.update("lookahead_path", 
                new_waypoints_np[path_indices.astype(int), 0],
                new_waypoints_np[path_indices.astype(int), 1],
                new_colour=[0, 0.7, 0.7])

        forward_speed_fig.roll("forward_speed", 
                                t - start_t, 
                                v)
        forward_speed_fig.roll("reference_signal", 
                                t - start_t, 
                                controller._desired_speed)

        throttle_fig.roll("throttle", t, output_throttle)
        brake_fig.roll("brake", t - start_t, output_break)
        steer_fig.roll("steer", t - start_t, output_steering)

        # Refresh the live plot based on the refresh rate 
        # set by the options
        if enable_live_plot and live_plot_timer.has_exceeded_lap_period():
            lp_traj.refresh()
            lp_1d.refresh()
            live_plot_timer.lap()

        rate.sleep()

if __name__ == '__main__':
    
    try:
        init_pub_sub()

    except rospy.ROSInterruptException:
        ser.close()


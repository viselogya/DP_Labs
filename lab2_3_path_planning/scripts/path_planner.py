#!/usr/bin/env python3
"""Plan on the warehouse costmap and track the route using wheel odometry."""
import heapq
import math

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Range
from std_msgs.msg import String


STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1),
         (1, 1), (1, -1), (-1, 1), (-1, -1))
RANGE_TOPICS = ("/proximity_front", "/proximity_front_left",
                "/proximity_front_right")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class Grid:
    def __init__(self, msg, radius, margin, inflation, scaling):
        info = msg.info
        origin = info.origin
        self.width, self.height = info.width, info.height
        self.resolution = info.resolution
        self.origin = origin.position.x, origin.position.y
        self.data = msg.data
        if (msg.header.frame_id != "map" or abs(yaw_of(origin.orientation)) > 1e-6
                or not self.width or not self.height or self.resolution <= 0
                or len(self.data) != self.width * self.height):
            raise ValueError("expected a nonempty axis-aligned costmap in map")

        # Allow for discretization when testing clearance around the hexagon.
        clearance = radius + margin + self.resolution * math.sqrt(2)
        if clearance > inflation:
            raise ValueError("increase inflation_radius to at least %.3f m" % clearance)
        # Costmap2D applies exponential inflation from the inscribed radius,
        # then publishes internal costs 1..252 as occupancy values 1..98.
        inscribed = radius * math.sqrt(3) / 2
        internal = int(252 * math.exp(-scaling * (clearance - inscribed)))
        if internal < 1:
            raise ValueError("decrease cost_scaling_factor to retain a clearance band")
        self.blocked_cost = 1 + (97 * (internal - 1)) // 251

    def cell(self, point):
        return tuple(math.floor((value - origin) / self.resolution)
                     for value, origin in zip(point, self.origin))

    def center(self, cell):
        return tuple(origin + (index + 0.5) * self.resolution
                     for index, origin in zip(cell, self.origin))

    def cost(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.data[y * self.width + x]
        return -1

    def free(self, x, y):
        return 0 <= self.cost(x, y) < self.blocked_cost


def astar(grid, start, goal, cost_weight):
    if not grid.free(*start) or not grid.free(*goal):
        raise ValueError("start or goal is outside the free costmap")

    queue = [(math.dist(start, goal), 0.0, start)]
    distance = {start: 0.0}
    parent = {}
    while queue:
        _, walked, cell = heapq.heappop(queue)
        if walked != distance[cell]:
            continue
        if cell == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return path[::-1]

        x, y = cell
        for dx, dy in STEPS:
            neighbor = x + dx, y + dy
            if not grid.free(*neighbor):
                continue
            if dx and dy and not (grid.free(x + dx, y) and grid.free(x, y + dy)):
                continue
            step = math.hypot(dx, dy) * (1 + cost_weight * grid.cost(*neighbor) / 100)
            candidate = walked + step
            if candidate < distance.get(neighbor, math.inf):
                distance[neighbor] = candidate
                parent[neighbor] = cell
                heapq.heappush(queue, (candidate + math.dist(neighbor, goal),
                                       candidate, neighbor))
    raise ValueError("no route to goal")


def route_points(grid, cells, goal):
    """Retain the endpoints of straight runs in the grid route."""
    waypoints = [grid.center(cells[0])]
    for previous, current, following in zip(cells, cells[1:], cells[2:]):
        if (2 * current[0], 2 * current[1]) != (previous[0] + following[0],
                                               previous[1] + following[1]):
            waypoints.append(grid.center(current))
    if len(cells) > 1:
        waypoints.append(grid.center(cells[-1]))
    if waypoints[-1] != goal:
        waypoints.append(goal)
    return waypoints


class Navigator:
    def __init__(self):
        costmap = "/costmap_node/costmap"
        self.radius = rospy.get_param(costmap + "/robot_radius")
        self.inflation = rospy.get_param(costmap + "/inflation_layer/inflation_radius")
        self.scaling = rospy.get_param(costmap + "/inflation_layer/cost_scaling_factor")
        for key in ("cost_weight", "clearance_margin", "waypoint_tolerance",
                    "max_linear", "max_angular", "linear_gain", "angular_gain",
                    "heading_tolerance", "stop_distance", "resume_distance",
                    "range_timeout", "odom_timeout"):
            setattr(self, key, float(rospy.get_param("~" + key)))
        if not (0 < self.radius < self.inflation and self.scaling > 0
                and self.cost_weight >= 0 and self.clearance_margin >= 0
                and 0 < self.stop_distance < self.resume_distance
                and 0 < self.heading_tolerance < math.pi / 2
                and min(self.waypoint_tolerance, self.max_linear, self.max_angular,
                        self.linear_gain, self.angular_gain,
                        self.range_timeout, self.odom_timeout) > 0):
            raise ValueError("invalid planner, controller or proximity configuration")

        self.goal_request = None
        if rospy.get_param("~autostart"):
            self.goal_request = (float(rospy.get_param("~goal_x")),
                                 float(rospy.get_param("~goal_y")))
        self.odom = None
        self.costmap = None
        self.ranges = {}
        self.active_goal = None
        self.used_map = None
        self.waypoints = []
        self.next_waypoint = 0
        self.plan_failed = False
        self.proximity_blocked = True
        self.last_status = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.cmd = rospy.Publisher("/diff_drive_controller/cmd_vel", Twist, queue_size=1)
        self.path = rospy.Publisher("~path", Path, queue_size=1, latch=True)
        self.status = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber(costmap + "/costmap", OccupancyGrid, self.on_map, queue_size=1),
            rospy.Subscriber("/diff_drive_controller/odom", Odometry, self.on_odom, queue_size=1),
            rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_goal, queue_size=1),
        ]
        self.subscribers.extend(
            rospy.Subscriber(topic, Range, self.on_range, callback_args=topic, queue_size=1)
            for topic in RANGE_TOPICS
        )
        rospy.on_shutdown(self.stop)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.update, reset=True)

    def on_map(self, msg):
        self.costmap = msg

    def on_odom(self, msg):
        self.odom = msg

    def on_range(self, msg, topic):
        self.ranges[topic] = msg

    def on_goal(self, msg):
        point = msg.pose.position
        if msg.header.frame_id != "map" or not all(map(math.isfinite, (point.x, point.y))):
            rospy.logwarn("Ignoring goal: expected finite coordinates in map")
            return
        self.goal_request = point.x, point.y

    def stop(self):
        self.cmd.publish(Twist())

    def set_status(self, status):
        if status != self.last_status:
            self.status.publish(String(data=status))
            rospy.loginfo("%s", status)
            self.last_status = status

    @staticmethod
    def fresh(msg, now, timeout):
        return (msg is not None and not msg.header.stamp.is_zero()
                and 0 <= (now - msg.header.stamp).to_sec() <= timeout)

    def map_pose(self, odom):
        if odom.header.frame_id != "odom" or odom.child_frame_id != "base_link":
            raise ValueError("expected odometry from odom to base_link")
        transform = self.tf_buffer.lookup_transform(
            "map", odom.header.frame_id, odom.header.stamp).transform
        angle = yaw_of(transform.rotation)
        px, py = odom.pose.pose.position.x, odom.pose.pose.position.y
        x = transform.translation.x + math.cos(angle) * px - math.sin(angle) * py
        y = transform.translation.y + math.sin(angle) * px + math.cos(angle) * py
        heading = angle + yaw_of(odom.pose.pose.orientation)
        if not all(map(math.isfinite, (x, y, heading))):
            raise ValueError("invalid odometry pose")
        return (x, y), heading

    def check_proximity(self, now):
        cutoff = self.resume_distance if self.proximity_blocked else self.stop_distance
        for topic in RANGE_TOPICS:
            reading = self.ranges.get(topic)
            if (not self.fresh(reading, now, self.range_timeout)
                    or reading.radiation_type != Range.ULTRASOUND
                    or not math.isfinite(reading.max_range)
                    or reading.max_range <= cutoff or math.isnan(reading.range)
                    or reading.range <= cutoff):
                self.proximity_blocked = True
                return "proximity stop: " + topic
        self.proximity_blocked = False
        return None

    def publish_path(self, points, stamp):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = stamp
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1
            msg.poses.append(pose)
        self.path.publish(msg)

    def update(self, _event):
        now = rospy.Time.now()
        goal, odom, costmap = self.goal_request, self.odom, self.costmap
        proximity = self.check_proximity(now)
        if goal is None or costmap is None or not self.fresh(odom, now, self.odom_timeout):
            self.stop()
            self.set_status("waiting for goal, costmap and fresh wheel odometry")
            return
        try:
            point, heading = self.map_pose(odom)
        except (tf2_ros.TransformException, ValueError) as error:
            self.stop()
            self.set_status(str(error))
            return

        if (goal is not self.active_goal or self.used_map is None
                or costmap.info != self.used_map.info or costmap.data != self.used_map.data):
            self.stop()
            self.active_goal, self.used_map = goal, costmap
            self.next_waypoint = 0
            try:
                self.grid = Grid(costmap, self.radius, self.clearance_margin,
                                 self.inflation, self.scaling)
                cells = astar(self.grid, self.grid.cell(point), self.grid.cell(goal),
                              self.cost_weight)
                self.waypoints = route_points(self.grid, cells, goal)
            except (ValueError, OverflowError) as error:
                self.plan_failed = True
                self.publish_path([], now)
                self.set_status("planning failed: " + str(error))
                return
            self.plan_failed = False
            self.publish_path([point] + self.waypoints, now)
            return

        if self.plan_failed:
            self.stop()
            return
        if proximity is not None:
            self.stop()
            self.set_status(proximity)
            return
        if not self.grid.free(*self.grid.cell(point)):
            self.stop()
            self.set_status("robot left the traversable costmap")
            return

        while self.next_waypoint < len(self.waypoints):
            tx, ty = self.waypoints[self.next_waypoint]
            dx, dy = tx - point[0], ty - point[1]
            distance = math.hypot(dx, dy)
            if distance > self.waypoint_tolerance:
                break
            self.next_waypoint += 1
        else:
            self.stop()
            self.set_status("goal reached")
            return

        error = math.atan2(dy, dx) - heading
        error = math.atan2(math.sin(error), math.cos(error))
        command = Twist()
        command.angular.z = max(-self.max_angular,
                                min(self.max_angular, self.angular_gain * error))
        if abs(error) <= self.heading_tolerance:
            command.linear.x = min(self.max_linear, self.linear_gain * distance)
        self.cmd.publish(command)
        self.set_status("tracking %d/%d" % (self.next_waypoint + 1, len(self.waypoints)))


if __name__ == "__main__":
    rospy.init_node("path_planner")
    navigator = Navigator()
    rospy.spin()

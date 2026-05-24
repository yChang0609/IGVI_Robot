#!/usr/bin/env python3
import math
import time
import yaml
import threading

import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose

from wildbot_grasp.action import SearchAndRetrieve
from wildbot_grasp_nodes.retrieve_base import RetrieveBase

WAYPOINTS_PATH = "/maps/waypoints.yaml"

class ArenaMissionServer(RetrieveBase):
    _feedback_class = SearchAndRetrieve.Feedback

    def __init__(self):
        super().__init__(
            "arena_mission_server",
            standoff_distance=0.45,
            visual_servo_kp=0.005,
        )
        self.declare_parameter("base_exclusion_radius_m", 0.3)
        self.declare_parameter("blacklist_timeout_sec", 60.0)
        
        self._goal_lock = threading.Lock()
        self._active_goal = False
        self._patrol_idx = 0
        
        self.search_client = ActionClient(
            self, SearchAndRetrieve, "search_retrieve",
            callback_group=self.callback_group,
        )
        
        self.action_server = ActionServer(
            self,
            SearchAndRetrieve,
            "arena_mission",
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("Ready: /arena_mission")

    def goal_callback(self, goal_request):
        with self._goal_lock:
            if self._active_goal:
                self.get_logger().warn("Rejecting goal: arena mission is already active")
                return GoalResponse.REJECT
            self._active_goal = True
        self.get_logger().info("Received goal to start Arena Mission")
        return GoalResponse.ACCEPT

    def load_waypoints(self):
        try:
            with open(WAYPOINTS_PATH) as f:
                data = yaml.safe_load(f) or {}
            raw = data.get("waypoints") if isinstance(data, dict) else None
            out = {}
            if isinstance(raw, dict):
                for name, pose in raw.items():
                    if isinstance(pose, dict):
                        out[name] = {
                            "x": float(pose.get("x", 0.0)),
                            "y": float(pose.get("y", 0.0)),
                            "yaw": float(pose.get("yaw", 0.0)),
                        }
            return out
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return {}

    def get_robot_xy(self):
        pose = self.get_robot_pose()
        return (pose[0], pose[1]) if pose else (0.0, 0.0)

    def _dist(self, x1, y1, x2, y2):
        return math.hypot(x1 - x2, y1 - y2)

    def execute_callback(self, goal_handle):
        result = SearchAndRetrieve.Result()
        blacklist = {} # target_id -> timestamp (monotonic)

        try:
            self.clear_costmaps()  # Clear costmap at start of mission to ensure clean slate
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Cancel requested! Returning from execute_callback...")
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Arena mission canceled"
                    return result
                
                waypoints = self.load_waypoints()
                our_base = waypoints.get("our_base")
                enemy_base = waypoints.get("enemy_base")
                
                if not our_base:
                    self.publish_feedback(goal_handle, "error", 0.0, "Missing 'our_base' in waypoints")
                    result.success = False
                    result.message = "Missing 'our_base' in waypoints"
                    goal_handle.abort()
                    return result

                # 1. Clean blacklist
                now = time.monotonic()
                timeout = float(self.get_parameter("blacklist_timeout_sec").value)
                blacklist = {k: v for k, v in blacklist.items() if now - v < timeout}

                # 2. Find best bear
                best_bear = None
                best_dist = float('inf')
                
                robot_xy = self.get_robot_xy()
                radius = float(self.get_parameter("base_exclusion_radius_m").value)

                with self.memory_lock:
                    memory_copy = list(self.latest_memory.values())
                    
                for obj in memory_copy:
                    if obj.get("class_name") != "xiong":
                        continue
                    obj_id = obj.get("id")
                    if not obj_id or obj_id in blacklist:
                        continue
                        
                    pos = obj.get("position", {})
                    bx, by = pos.get("x"), pos.get("y")
                    if bx is None or by is None:
                        continue
                        
                    # Check exclusion radius
                    if our_base and self._dist(bx, by, our_base["x"], our_base["y"]) < radius:
                        continue
                    if enemy_base and self._dist(bx, by, enemy_base["x"], enemy_base["y"]) < radius:
                        continue
                        
                    d_robot = self._dist(bx, by, robot_xy[0], robot_xy[1])
                    if d_robot < best_dist:
                        best_dist = d_robot
                        best_bear = obj

                # If found a valid bear, go grab it
                if best_bear:
                    target_id = best_bear['id']
                    self.publish_feedback(goal_handle, "grabbing", 0.0, f"Found {target_id} at {best_dist:.1f}m. Calling search_retrieve.")
                    
                    if not self.search_client.wait_for_server(timeout_sec=5.0):
                        self.get_logger().error("search_retrieve server not available")
                        time.sleep(1.0)
                        continue
                        
                    req = SearchAndRetrieve.Goal()
                    req.target_id = target_id
                    req.home_pose_x = float(our_base["x"])
                    req.home_pose_y = float(our_base["y"])
                    req.home_pose_yaw = float(our_base["yaw"])
                    
                    send_future = self.search_client.send_goal_async(req)
                    while rclpy.ok() and not send_future.done():
                        if goal_handle.is_cancel_requested:
                            break
                        time.sleep(0.1)
                        
                    if goal_handle.is_cancel_requested:
                        continue
                        
                    search_goal_handle = send_future.result()
                    if not search_goal_handle.accepted:
                        self.publish_feedback(goal_handle, "rejected", 0.0, f"search_retrieve rejected goal for {target_id}")
                        blacklist[target_id] = time.monotonic()
                        continue
                        
                    result_future = search_goal_handle.get_result_async()
                    cancel_sent = False
                    while rclpy.ok() and not result_future.done():
                        if goal_handle.is_cancel_requested and not cancel_sent:
                            search_goal_handle.cancel_goal_async()
                            cancel_sent = True
                        time.sleep(0.2)
                        
                    wrapped_result = result_future.result()
                    if wrapped_result and wrapped_result.status == 4: # STATUS_SUCCEEDED
                        self.publish_feedback(goal_handle, "grab_success", 0.0, f"Successfully retrieved {target_id}.")
                        # Actively remove the object from memory
                        self.remove_object_from_memory(target_id)
                    else:
                        self.publish_feedback(goal_handle, "grab_failed", 0.0, f"Failed to retrieve {target_id}. Blacklisting.")
                        blacklist[target_id] = time.monotonic()
                        self.clear_costmaps()
                    
                    time.sleep(1.0) # slight delay before next cycle
                    continue

                # 3. Patrol Phase
                patrols = [ (k, v) for k, v in waypoints.items() if k.startswith("patrol_") ]
                patrols.sort(key=lambda x: x[0])
                
                if not patrols:
                    self.publish_feedback(goal_handle, "waiting", 0.0, "No valid bears and no patrols set. Waiting.")
                    time.sleep(2.0)
                    continue
                    
                patrol_name, patrol_wp = patrols[self._patrol_idx % len(patrols)]
                self._patrol_idx += 1
                
                self.publish_feedback(goal_handle, "patrolling", 0.0, f"Patrolling to {patrol_name} ({patrol_wp['x']:.2f}, {patrol_wp['y']:.2f})")
                
                nav_pose = self.make_pose(patrol_wp["x"], patrol_wp["y"], patrol_wp["yaw"])
                nav_goal = NavigateToPose.Goal()
                nav_goal.pose = nav_pose
                
                if not self.nav_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error("Nav2 server not available")
                    time.sleep(1.0)
                    continue
                    
                send_future = self.nav_client.send_goal_async(nav_goal)
                while rclpy.ok() and not send_future.done():
                    if goal_handle.is_cancel_requested:
                        break
                    time.sleep(0.1)
                    
                if goal_handle.is_cancel_requested:
                    continue
                    
                nav_goal_handle = send_future.result()
                if not nav_goal_handle.accepted:
                    continue
                    
                result_future = nav_goal_handle.get_result_async()
                patrol_interrupted = False
                
                # Nav2 only plans the path, so result_future completes immediately.
                # We must loop and check the actual distance to the patrol point!
                arrival_tolerance = float(self.get_parameter("arrival_tolerance").value)
                patrol_start_time = time.time()
                patrol_timeout = 60.0
                
                while rclpy.ok():
                    if goal_handle.is_cancel_requested:
                        nav_goal_handle.cancel_goal_async()
                        break
                        
                    if time.time() - patrol_start_time > patrol_timeout:
                        self.get_logger().warn(f"Patrol to {patrol_name} timed out")
                        self.clear_costmaps()  # Clear costmap on timeout to help recover from ghost obstacles
                        break
                        
                    # Check distance
                    robot_xy = self.get_robot_xy()
                    dist_to_patrol = self._dist(robot_xy[0], robot_xy[1], patrol_wp["x"], patrol_wp["y"])
                    if dist_to_patrol <= arrival_tolerance:
                        break  # Arrived!
                        
                    # Memory interrupt check
                    with self.memory_lock:
                        memory_copy = list(self.latest_memory.values())
                        
                    for obj in memory_copy:
                        obj_id = obj.get("id")
                        if obj.get("class_name") == "xiong" and obj_id not in blacklist:
                            pos = obj.get("position", {})
                            bx, by = pos.get("x"), pos.get("y")
                            if bx is None or by is None:
                                continue
                            
                            valid = True
                            if our_base and self._dist(bx, by, our_base["x"], our_base["y"]) < radius:
                                valid = False
                            if enemy_base and self._dist(bx, by, enemy_base["x"], enemy_base["y"]) < radius:
                                valid = False
                                
                            if valid:
                                patrol_interrupted = True
                                break
                    
                    if patrol_interrupted:
                        self.publish_feedback(goal_handle, "interrupt", 0.0, "Bear spotted during patrol! Interrupting.")
                        nav_goal_handle.cancel_goal_async()
                        break
                        
                    time.sleep(0.3)
                    
                if not patrol_interrupted:
                    self.get_logger().info(f"Finished patrol {patrol_name}")
                    time.sleep(1.0) # pause at patrol point

        except Exception as e:
            self.get_logger().error(f"Arena mission error: {e}")
            result.success = False
            result.message = f"Error: {e}"
            goal_handle.abort()
            return result

        finally:
            with self._goal_lock:
                self._active_goal = False

def main(args=None):
    rclpy.init(args=args)
    node = ArenaMissionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

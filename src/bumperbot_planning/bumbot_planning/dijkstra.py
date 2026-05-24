#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, LookUpException #Robot Lo
from queue import PriorityQueue


#abstract grah class
class GraphNode:
        def __init__(self, x, y, cost = 0, prev=None):
                """Represents a node in the graph for Dijkstra's algorithm"""
                self.x = x
                self.y = y
                self.prev = prev
                self.cost = cost 
       
        def __lt__(self, other):
              """Defines less than for priority queue based on cost"""
              return self.cost < other.cost 
       
        def __eq__(self, other):
              return self.x == other.x and self.y == other.y
        
        def __hash__(self):
            return hash((self.x,self.y))
        
        def __add__(self, other):
                return GraphNode(self.x + other[0], self.y + other[1])


class DijkstraPlanner(Node):
        def __init__(self):
                super().__init__("dijkstra_node") 
                map_qos = QoSProfile(depth=10)
               
                map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
                self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
                self.pose_sub = self.create_subscription(PoseStamped, "/goal_pose",self.goal_callback, 10)
                self.path_pub = self.create_publisher(Path, "/dijkstra/path",10)
                self.map_pub = self.create_publisher(OccupancyGrid, "/dijkstra/visited_map",10)

                self.map_ = None
                self.visted_map_ = OccupancyGrid()

                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)

        def map_callback(self, map_msg : OccupancyGrid):
                self.map = map_msg
                self.visted_map_.header._frame_id = map_msg.header.frame_id
                self.visited_map_.info = map_msg.info
                self.visited_map.data = [-1] * (map_msg.info.height * map_msg.info.width)
        
        def goal_callback(self, pose: PoseStamped):
                if self.map_ is None:
                        self.get_logger().error ("No map received!")
                        return 
                self.visited_map.data = [-1] * (self.map.info.height * self.map.info.width)
                try: 
                  map_to_base_tf = self.tf_buffer.lookup_transform(self.map_.header.frame_id, "base_footprint", rclpy.time.Time())
                except LookUpException:
                    self.get_logger().error("could not transform map to base footprint")
                    return 
                #current pos of robot 
                map_to_base_pose = Pose()
                map_to_base_pose.position.x = map_to_base_tf.transform.translation.x
                map_to_base_pose.position.y = map_to_base_tf.transform.translation.y
                map_to_base_pose.orientation = map_to_base_tf.transform.rotation

                path =  self.plan(map_to_base_pose, pose.pose) #calculated path 

                if path.poses: 
                   self.get_logger().info("Shortest path found")
                   self.path_pub.publish(path)
                else:
                    self.logger().info.warn("No path found")
                
        def plan(self, start, goal):
            explored_directions = [{-1,0}, {1,0}, {0,-1}, {0,1}] #4-connected grid
            pending_nodes = PriorityQueue() #priority queue for pending nodes
            visited_nodes = set()             #set for visited nodes
            start_node = self.world_to_grid(start) #convert start and goal to grid coordinates
            pending_nodes.put(start_node) #add start node to pending nodes

            while not pending_nodes.empty() and rclpy.ok():
                  active_node = pending_nodes.get() #get node with lowest cost from pending nodes
                  if active_node == self.world_to_grid(goal):
                        break #if we reached the goal, break out of the loop
                  for dir_x,dir_y in explored_directions: # explore neighbors in 4 directions 
                        new_node: GraphNode = active_node + (dir_x, dir_y) #create new node by adding direction to active node
                        if new_node not in visited_nodes and self.is_pos_valid(new_node) and self.map_.data[self.pose_to_cell(new_node)] == 0: #check if new node is valid and not an obstacle
                                new_node.cost = active_node.cost + 1 #update cost of new node assume all edges have cost of 1
                                new_node.prev = active_node #set previous node of new node to active node
                                pending_nodes.put(new_node) #add new node to pending nodes
                                visited_nodes.add(new_node) #add new node to visited nodes
                  self.visited_map.data[self.pose_to_cell(active_node)] = 100 #mark active node as visited in visited map
                  
                
                        
                  
        def is_pos_valid(self,node: GraphNode):
                """Checks if a node is within the map bounds and not an obstacle"""
                return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.width
        
        def pose_to_cell(self, node: GraphNode) :
              """Converts grid coordinates to cell index in the occupancy grid data array"""
              return node.y * self.map_.info.width + node.x #converts grid coordinates to cell index in the occupancy grid data array
        
        def world_to_grid(self, pose:Pose) -> GraphNode:
                """Converts world coordinates to grid coordinates"""
                grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution) #converts world x to grid x info res is the size of each grid cell in meters
                grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution) #converts world y to grid y info res is the size of each grid cell in meters
                return GraphNode(grid_x, grid_y) #returns a GraphNode with the grid coordinates

def main():
     rclpy.init()
     node = DijkstraPlanner()
     rclpy.spin(node)
     rclpy.shutdown

if __name__ == '__main__':
    main()
        

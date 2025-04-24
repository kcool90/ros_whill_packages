#!/usr/bin/env python3
import osmnx as ox
import networkx as nx
import logging
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Set up logging (optional, for debugging)
logging.basicConfig(level=logging.DEBUG)

class RoutePublisher(Node):
    def __init__(self):
        super().__init__('route_publisher')
        # Publisher that will publish the list of GPS waypoints on /gps_waypoints
        self.pub = self.create_publisher(String, 'gps_waypoints', 10)
        self.compute_and_publish_route()

    def compute_and_publish_route(self):
        # Define your start and stop coordinates (latitude, longitude)
        start_coord = (38.635891, -90.227772)
        stop_coord  = (38.637483, -90.241264)

        self.get_logger().info(f"Start coordinate: {start_coord}")
        self.get_logger().info(f"Stop coordinate: {stop_coord}")

        # Create a bounding box that comfortably includes both points
        offset = 0.001
        north = max(start_coord[0], stop_coord[0]) + offset
        south = min(start_coord[0], stop_coord[0]) - offset
        east  = max(start_coord[1], stop_coord[1]) + offset
        west  = min(start_coord[1], stop_coord[1]) - offset

        bbox = (west, south, east, north)
        self.get_logger().info(f"Bounding box: {bbox}")
        self.get_logger().info("Downloading graph...")

        G = ox.graph_from_bbox(bbox, network_type='walk')

        self.get_logger().info("Graph downloaded.")
        self.get_logger().info(f"Number of nodes: {len(G.nodes)}")
        self.get_logger().info(f"Number of edges: {len(G.edges)}")

        # Find the nearest network nodes to the start and stop coordinates
        self.get_logger().info("Finding nearest nodes...")
        start_node = ox.distance.nearest_nodes(G, start_coord[1], start_coord[0])
        stop_node  = ox.distance.nearest_nodes(G, stop_coord[1], stop_coord[0])

        self.get_logger().info(f"Start node: {start_node}")
        self.get_logger().info(f"Stop node: {stop_node}")

        # Compute the shortest path between the two nodes using the edge length as the weight
        self.get_logger().info("Computing shortest path...")
        route = nx.shortest_path(G, start_node, stop_node, weight='length')

        self.get_logger().info(f"Route computed. Route length (number of nodes): {len(route)}")

        # Extract the list of waypoints (latitude, longitude) for the route
        route_coords = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in route]

        self.get_logger().info("Route waypoints:")
        for coord in route_coords:
            self.get_logger().info(str(coord))
        
        # Convert the list to a JSON string
        waypoints_json = json.dumps(route_coords)
        msg = String()
        msg.data = waypoints_json

        # Publish the message on the /gps_waypoints topic
        self.pub.publish(msg)
        self.get_logger().info("Published gps_waypoints.")

def main(args=None):
    rclpy.init(args=args)
    node = RoutePublisher()
    # We can spin briefly so the message is published.
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

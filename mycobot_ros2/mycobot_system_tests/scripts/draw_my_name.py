from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class YourNode(Node):
    def __init__(self):
        # ... your existing initialization ...
        self.marker_array = MarkerArray()

    def publish_all_trails(self):
        # Publish all stored trails
        self.marker_array.markers.clear()
        for idx, trail_points in enumerate(all_trails_of_letters):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'letters_trails'
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.01
            marker.color.g = 1.0
            marker.color.a = 1.0
            marker.points = trail_points
            self.marker_array.markers.append(marker)
        self.marker_pub.publish(self.marker_array)


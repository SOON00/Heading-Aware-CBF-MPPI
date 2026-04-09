#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped
import math

class FootprintCollisionCounter(Node):
    def __init__(self):
        super().__init__('footprint_collision_counter')

        self.costmap = None
        self.collision_count = 0
        self.is_colliding = False

        # 데이터 수신 확인용 플래그
        self.received_costmap = False
        self.received_footprint = False

        # 토픽 이름은 환경에 따라 다를 수 있으니 ros2 topic list로 꼭 확인하세요!
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10)

        self.footprint_sub = self.create_subscription(
            PolygonStamped,
            '/local_costmap/published_footprint',
            self.footprint_callback,
            10)

        self.get_logger().info('Footprint Collision Counter Node 시작! 데이터 수신 대기 중...')

    def costmap_callback(self, msg):
        if not self.received_costmap:
            self.get_logger().info('✅ Costmap 데이터를 성공적으로 수신했습니다!')
            self.received_costmap = True
        self.costmap = msg

    def footprint_callback(self, msg):
        if not self.received_footprint:
            self.get_logger().info('✅ Footprint 데이터를 성공적으로 수신했습니다!')
            self.received_footprint = True

        if self.costmap is None:
            return 

        points = msg.polygon.points
        if len(points) < 3:
            return 

        collision_detected = False

        for i in range(len(points)):
            p1 = points[i]
            p2 = points[(i + 1) % len(points)]

            if self.check_edge_collision(p1, p2):
                collision_detected = True
                break 

        if collision_detected:
            if not self.is_colliding:
                self.collision_count += 1
                self.is_colliding = True
                self.get_logger().warn(f'💥 충돌 감지! (cost=100) | 총 충돌 횟수: {self.collision_count}회')
        else:
            self.is_colliding = False

    def check_edge_collision(self, p1, p2):
        res = self.costmap.info.resolution
        if res <= 0.0:
            return False

        dist = math.hypot(p2.x - p1.x, p2.y - p1.y)
        steps = max(1, int(dist / res))

        for i in range(steps + 1):
            t = i / steps if steps > 0 else 0
            x = p1.x + t * (p2.x - p1.x)
            y = p1.y + t * (p2.y - p1.y)

            cost = self.get_cost(x, y)
            
            # Nav2 OccupancyGrid에서 Lethal Obstacle(내부 254)은 100으로 매핑됩니다.
            # Inscribed obstacle(내부 253)까지 포함하려면 cost >= 99 로 변경하세요.
            if cost == 100:  
                return True
        return False

    def get_cost(self, x, y):
        info = self.costmap.info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution

        grid_x = int((x - origin_x) / res)
        grid_y = int((y - origin_y) / res)

        if grid_x < 0 or grid_x >= info.width or grid_y < 0 or grid_y >= info.height:
            return -1 

        index = grid_y * info.width + grid_x
        return self.costmap.data[index]

def main(args=None):
    rclpy.init(args=args)
    node = FootprintCollisionCounter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료합니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

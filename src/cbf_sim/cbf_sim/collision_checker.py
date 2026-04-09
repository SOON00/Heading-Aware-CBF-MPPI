import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class CollisionChecker(Node):
    def __init__(self):
        super().__init__('collision_checker')
        
        # /scan 토픽 구독 (라이다 데이터)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # 실제 사용하는 라이다 토픽 이름으로 변경
            self.scan_callback,
            10)
        
        # 🌟 실제 로봇의 물리적 직사각형 크기 (제어기 파라미터와 무관하게 고정!)
        self.robot_length = 0.9  # X축 전체 길이 (앞뒤 0.45m)
        self.robot_width = 0.4   # Y축 전체 너비 (좌우 0.2m)
        
        # 충돌 판단 마진 (센서 노이즈 고려, 0.01~0.02m 정도 여유를 줌)
        self.margin = 0.02
        
        self.collision_count = 0
        self.is_collided = False
        
        self.get_logger().info("Collision Checker Node Started. Using Rectangular Footprint [0.9m x 0.4m]")

    def scan_callback(self, msg):
        angle = msg.angle_min
        
        for r in msg.ranges:
            # 1. 무효한 라이다 데이터(inf, NaN) 무시
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue
                
            # 2. 극좌표계(r, angle)를 직교좌표계(x, y)로 변환 (기준: base_link)
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            # 3. 측정된 장애물(점)이 로봇의 '진짜 직사각형' 내부로 들어왔는지 검사
            in_x_bound = (-self.robot_length/2) <= x <= (self.robot_length/2)
            in_y_bound = (-self.robot_width/2) <= y <= (self.robot_width/2)
            
            if in_x_bound and in_y_bound:
                if not self.is_collided:
                    self.collision_count += 1
                    self.get_logger().error(f"💥 COLLISION DETECTED! (Count: {self.collision_count}) at Local X: {x:.2f}, Y: {y:.2f}")
                    self.is_collided = True
                break  # 충돌을 감지했으면 해당 프레임 검사 종료
            
            angle += msg.angle_increment
            
        else:
            # for 문이 break 없이 정상 종료되었다면 (충돌점이 없다면)
            self.is_collided = False

def main(args=None):
    rclpy.init(args=args)
    collision_checker = CollisionChecker()
    try:
        rclpy.spin(collision_checker)
    except KeyboardInterrupt:
        collision_checker.get_logger().info(f"Final Collision Count: {collision_checker.collision_count}")
    finally:
        collision_checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

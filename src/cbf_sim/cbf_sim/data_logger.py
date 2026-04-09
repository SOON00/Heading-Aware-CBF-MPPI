import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import csv
import time

# 외부 라이브러리 없이 Quaternion을 Euler(Yaw)로 변환하는 함수 직접 구현
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # 라디안(radians) 단위 반환

class MetricLogger(Node):
    def __init__(self):
        super().__init__('metric_logger')
        
        # /odom 토픽 구독 (실제 로봇이나 시뮬레이터의 odom 토픽 이름 확인 필요)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  
            self.odom_callback,
            10)
        
        self.start_time = time.time()
        
        # CSV 파일 생성 (실험 모델에 맞게 파일명 변경: m1_result.csv, m6_result.csv 등)
        self.csv_filename = 'm5_1.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # CSV 헤더 작성
        self.csv_writer.writerow(['timestamp', 'x', 'y', 'yaw', 'linear_v_x', 'linear_v_y', 'angular_w'])
        
        self.get_logger().info(f"Data Logger Started! Saving to {self.csv_filename}...")

    def odom_callback(self, msg):
        current_time = time.time() - self.start_time
        
        # 1. 위치 (X, Y)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 2. Quaternion -> Euler (Yaw 추출)
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        # 3. 속도 (Linear, Angular)
        linear_v_x = msg.twist.twist.linear.x
        linear_v_y = msg.twist.twist.linear.y
        angular_w = msg.twist.twist.angular.z
        
        # 데이터 저장
        self.csv_writer.writerow([current_time, x, y, yaw, linear_v_x, linear_v_y, angular_w])

def main(args=None):
    rclpy.init(args=args)
    metric_logger = MetricLogger()
    try:
        rclpy.spin(metric_logger)
    except KeyboardInterrupt:
        metric_logger.get_logger().info("Data Logger Stopped. CSV file saved.")
    finally:
        # 종료 시 파일 안전하게 닫기
        metric_logger.csv_file.close()
        metric_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

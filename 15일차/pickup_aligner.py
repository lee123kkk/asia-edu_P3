import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String
from rclpy.qos import qos_profile_sensor_data
import math
import time

class PickupAligner(Node):
    def __init__(self):
        super().__init__('pickup_aligner')
        
        self.mode_sub = self.create_subscription(Int32, '/internal_mode', self.mode_callback, 10)
        self.target_sub = self.create_subscription(Point, '/pickup_target', self.target_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ack_pub = self.create_publisher(Int32, '/AGV_mode_ack', 10)
        self.status_pub = self.create_publisher(Int32, '/AGV_status', 10)
        self.log_pub = self.create_publisher(String, '/AGV_log', 10)
        
        self.current_agv_mode = 0
        self.state = 'WAITING'  # WAITING -> CRAB_WALK -> ALIGNING -> DONE
        
        self.target_y = 0.0
        self.crab_start_time = 0.0
        self.crab_duration = 0.0
        self.crab_direction = 0.0
        
        self.get_logger().info('🧩 픽업 정밀 밀착 노드 가동 (평행 우선 정렬 및 20cm 절대 정지 적용)')

    def send_log(self, text, level='info'):
        if level == 'info': self.get_logger().info(text)
        elif level == 'warn': self.get_logger().warn(text)
        msg = String()
        msg.data = f"[픽업 밀착] {text}"
        self.log_pub.publish(msg)

    def mode_callback(self, msg):
        self.current_agv_mode = msg.data
        if self.current_agv_mode != 51:
            self.state = 'WAITING'

    def target_callback(self, msg):
        if self.current_agv_mode != 51: return
        
        if self.state == 'WAITING':
            self.target_y = msg.y
            self.send_log(f'🎯 목표 좌표 수신 (Y오차: {self.target_y:.2f}m). 52번 신호 발송.')
            
            for _ in range(3):
                ack_msg = Int32()
                ack_msg.data = 52
                self.ack_pub.publish(ack_msg)
            
            speed_y = 0.1 
            self.crab_duration = abs(self.target_y) / speed_y
            self.crab_direction = 1.0 if self.target_y > 0 else -1.0
            
            self.crab_start_time = time.time()
            self.state = 'CRAB_WALK'

    def scan_callback(self, msg):
        if self.current_agv_mode != 51: return
        if self.state == 'WAITING' or self.state == 'DONE': return
        
        current_time = time.time()
        twist = Twist()
        
        if self.state == 'CRAB_WALK':
            if current_time - self.crab_start_time < self.crab_duration:
                twist.linear.y = 0.1 * self.crab_direction
                self.cmd_vel_pub.publish(twist)
            else:
                self.send_log('🦀 측면 정렬 완료. 전방 평행 밀착을 시작합니다.')
                self.state = 'ALIGNING'
                
        elif self.state == 'ALIGNING':
            min_f = float('inf') 
            min_l = float('inf') 
            min_r = float('inf') 
            emergency_dist = float('inf')
            
            for i, r in enumerate(msg.ranges):
                if r <= 0.01 or r > 2.0 or math.isinf(r) or math.isnan(r): continue
                angle = msg.angle_min + i * msg.angle_increment
                deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
                
                # 비상 브레이크용
                if -30 <= deg <= 30:
                    emergency_dist = min(emergency_dist, r)
                
                # 정밀 정렬용
                if -12 <= deg <= 12: min_f = min(min_f, r)
                elif 15 <= deg <= 30: min_l = min(min_l, r)
                elif -30 <= deg <= -15: min_r = min(min_r, r)
                
            if math.isinf(min_f): min_f = 2.0
            if math.isinf(min_l): min_l = 2.0
            if math.isinf(min_r): min_r = 2.0
            
            diff = min_l - min_r
            
            # ----------------------------------------------------
            # 순차적 제어 로직: 1. 평행 맞추기 -> 2. 전진 -> 3. 20cm 정지
            # ----------------------------------------------------
            
            # [1단계] 각도 오차가 2cm보다 크면, 무조건 제자리 회전으로 평행부터 맞춤 (전진 금지)
            if abs(diff) > 0.02:
                twist.linear.x = 0.0  
                twist.angular.z = 0.15 if diff > 0 else -0.15
                
            # [2단계] 평행이 완벽히 맞춰졌다면 직진 제어 시작
            else:
                twist.angular.z = 0.0  # 평행 완료, 회전 중지
                
                # [3단계] 라이다 기준 정면 20cm(0.20m) 도달 확인
                if min_f <= 0.20:
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    self.cmd_vel_pub.publish(twist)
                    
                    self.send_log('✅ 평행 정렬 후 라이다 기준 20cm 정지 완료! 동작을 종료하고 1번 신호를 발송합니다.')
                    
                    # 동작이 끝났으므로 1번 신호 발송
                    for _ in range(3):
                        status_msg = Int32()
                        status_msg.data = 1
                        self.status_pub.publish(status_msg)
                    
                    self.state = 'DONE'
                    return
                    
                # 아직 20cm에 도달하지 않았다면 앞으로 전진
                elif emergency_dist < 0.15:
                    # 물리적 안전장치: 20cm 도달 조건이 어긋나 15cm까지 파고들 경우 강제 정지
                    twist.linear.x = 0.0
                    self.send_log('🚨 긴급 브레이크 작동! 라이다 기준 15cm 이내로 들어왔습니다.', 'warn')
                else:
                    twist.linear.x = 0.05  # 5cm/s 속도로 안전하게 직진
                
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PickupAligner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()

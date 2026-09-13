import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class Litus(Node):
    def __init__(self, action_plan):
        super().__init__('Litus_test_node')
        
        # --- Настройки одометрии (датчиков) ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0 
        
        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        # --- План действий, который мы передаем из main() ---
        self.steps = action_plan
        self.current_step_index = 0

        # Конфигурация движения (можно менять под себя)
        self.target_distance = 1.0       # Сколько метров ехать вперед
        self.target_angle = math.pi / 2  # Угол поворота (90 градусов в радианах)

        # ROS 2 Издатель и Подписчик
        self.publisher_ = self.create_publisher(Twist, '/RMC2/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Odometry, '/RMC2/odometry', self.odom_callback, 10)
        
        # Главный цикл (аналог void loop) - крутится каждые 0.1 сек
        self.timer = self.create_timer(0.1, self.arduino_loop)

    def odom_callback(self, msg):
        """Просто считываем показания датчиков положения"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def reset_step_odometry(self):
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_yaw = self.current_yaw

    def arduino_loop(self):
        """Аналог void loop(). Управляет шагами по очереди."""
        if self.current_step_index >= len(self.steps):
            return

        current_action = self.steps[self.current_step_index]

        if self.start_x is None:
            self.reset_step_odometry()
            self.get_logger().info(f'Выполняю команду: {current_action}')

        # Вызов нужной функции под текущую команду
        if current_action == 'FORWARD':
            self.move_forward()
        elif current_action == 'TURN_LEFT':
            self.turn_left()
        elif current_action == 'TURN_RIGHT':
            self.turn_right()
        elif current_action == 'STOP':
            self.stop_robot()
            self.get_logger().info('Маршрут завершен. Робот выключен.')
            rclpy.shutdown()

    # =========================================================================
    # БЛОК ФУНКЦИЙ ДВИЖЕНИЯ
    # =========================================================================

    def move_forward(self):
        distance = math.sqrt((self.current_x - self.start_x)**2 + (self.current_y - self.start_y)**2)
        if distance < self.target_distance:
            msg = Twist()
            msg.linear.x = 0.2  
            self.publisher_.publish(msg)
        else:
            self.next_step()

    def turn_left(self):
        angle_turned = self.current_yaw - self.start_yaw
        angle_turned = math.atan2(math.sin(angle_turned), math.cos(angle_turned))
        if abs(angle_turned) < self.target_angle:
            msg = Twist()
            msg.angular.z = 0.3  
            self.publisher_.publish(msg)
        else:
            self.next_step()

    def turn_right(self):
        angle_turned = self.current_yaw - self.start_yaw
        angle_turned = math.atan2(math.sin(angle_turned), math.cos(angle_turned))
        if abs(angle_turned) < self.target_angle:
            msg = Twist()
            msg.angular.z = -0.3  
            self.publisher_.publish(msg)
        else:
            self.next_step()

    def stop_robot(self):
        msg = Twist()
        self.publisher_.publish(msg)

    def next_step(self):
        self.stop_robot()
        self.current_step_index += 1
        self.start_x = None  


# =========================================================================
# ТВОЯРАБОЧАЯ ЗОНА (КОНСТРУКТОР МАРШРУТА)
# =========================================================================
def main(args=None):
    rclpy.init(args=args)
    
    # ВОТ ЗДЕСЬ пиши последовательность команд!
    # Доступные команды: 'FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'STOP'
    # Метод подбора: просто добавляй или удаляй элементы из этого списка.
    
    my_route = [
	'TURN_RIGHT',
        'FORWARD', 
        'TURN_RIGHT', 
        'FORWARD', 
        'TURN_LEFT',
        'FORWARD',
        'STOP'  # Всегда оставляй 'STOP' в самом конце, чтобы робот выключался
    ]
    
    node = Litus(my_route)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()


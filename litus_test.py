import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class Litus(Node):
    def __init__(self, spisok):
        super().__init__('Litus_test_node')
        
        # Ваши параметры из переданного списка
        self.nach = spisok
        self.com = spisok
        self.get_logger().info(f'Старт с параметрами: nach={self.nach}, com={self.com}')
        
        # Переменные для контроля 1 метра пути
        self.start_x = None
        self.start_y = None
        self.target_distance = 1.0  # Цель в метрах
        self.is_moving = True

        # Создаем издателя (отправка скорости)
        self.publisher_ = self.create_publisher(Twist, '/RMC2/cmd_vel', 10)
        
        # Создаем подписчика (чтение координат из Webots)
        self.subscription_ = self.create_subscription(
            Odometry,
            '/RMC2/odometry',
            self.odom_callback,
            10
        )
        
        # Таймер движения (каждые 0.1 сек)
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        """Срабатывает каждый раз при получении координат от симулятора"""
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Запоминаем первую точку старта
        if self.start_x is None and self.start_y is None:
            self.start_x = current_x
            self.start_y = current_y
            self.get_logger().info('Стартовая позиция зафиксирована')
            return

        if not self.is_moving:
            return

        # Считаем пройденное расстояние
        distance_traveled = math.sqrt((current_x - self.start_x)**2 + (current_y - self.start_y)**2)
        
        # Если проехали 1 метр — останавливаемся и выключаемся
        if distance_traveled >= self.target_distance:
            self.is_moving = False
            self.stop_robot()
            self.get_logger().info(f'Робот проехал {distance_traveled:.2f} м и остановился.')
            rclpy.shutdown()

    def control_loop(self):
        """Отправка скорости пока не проехали нужную дистанцию"""
        if self.start_x is None or not self.is_moving:
            return

        msg = Twist()
        msg.linear.x = 0.2  # Скорость вперед (м/с)
        self.publisher_.publish(msg)

    def stop_robot(self):
        """Мгновенная остановка робота"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        self.publisher_.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Ваш список по умолчанию
    my_spisok = [0, 24]
    
    node = Litus(my_spisok)
    
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

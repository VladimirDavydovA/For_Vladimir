import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time  # Важно: это наш аналог delay()

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        # Создаем издателя на топик скорости
        self.pub = self.create_publisher(Twist, '/RMC2/cmd_vel', 10)
        # Пауза 1 секунда, чтобы ROS 2 успел установить связь с симулятором
        time.sleep(1.0)

    def run_command(self, linear_x, angular_z, duration):
        """Универсальная функция: задаем скорость -> ждем -> останавливаемся"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)
        
        time.sleep(duration)  # Ждем, пока робот едет/крутится
        
        # Мгновенная остановка после действия
        stop_msg = Twist()
        self.pub.publish(stop_msg)
        time.sleep(0.2)  # Короткая пауза для гашения инерции

def main(args=None):
    rclpy.init(args=args)
    bot = RobotControl()

    # =========================================================================
    # ТВОЙ МАРШРУТ (Метод подбора времени в секундах)
    # =========================================================================
    # Формат: bot.run_command( линейная_скорость, угловая_скорость, время_в_секундах )
    
    bot.run_command(0.0, -0.3, 1.5)  # Поворот НАПРАВО
    bot.run_command(0.2,  0.0, 5.0)  # ВПЕРЕД
    bot.run_command(0.0, -0.3, 1.5)  # Поворот НАПРАВО
    bot.run_command(0.2,  0.0, 5.0)  # ВПЕРЕД
    bot.run_command(0.0,  0.3, 1.5)  # Поворот НАЛЕВО
    bot.run_command(0.2,  0.0, 5.0)  # ВПЕРЕД

    # Выключение ноды
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

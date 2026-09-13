import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotMover(Node):
    def __init__(self, spisok):
        super().__init__('robot_mover')
        # Создаем паблишер в топик /cmd_vel, который слушает Webots
        self.publisher_ = self.create_publisher(Twist, '/RMC2/cmd_vel', 10)
        # Таймер будет вызывать функцию каждые 0.1 секунды
        self.timer = self.create_timer(0.1, self.move_robot)
	self.nach = spisok[0]
	self.con = spisok[1]

    def move_robot(self):
        msg = Twist()
        msg.linear.x = 0.2   # Скорость вперед: 0.2 м/с
        msg.angular.z = 0.5  # Поворот: 0.5 рад/с
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

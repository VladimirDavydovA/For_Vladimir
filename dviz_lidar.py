import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan # Импортируем Лидар
import sys

class LitusCompetitionNode(Node):
    def __init__(self, route):
        super().__init__('litus_competition_node')
        
        # Издатель скорости
        self.pub = self.create_publisher(Twist, '/RMC2/cmd_vel', 10)
        
        # Подписчик на Лидар
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/RMC2/scan', # Топик может называться /scan или /RMC2/scan, уточни на роботе
            self.lidar_callback,
            10
        )
        
        # Переменные управления
        self.route = route
        self.current_target_index = 0
        self.obstacle_detected = False
        self.movement_active = False

        # Главный цикл (void loop) - работает каждые 0.1 секунды
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Выводим построенный маршрут для эксперта (Требование 1.1 и 2.1)
        self.get_logger().info(f"ПОСТРОЕННЫЙ МАРШРУТ: {self.route}")
        print(f"\n[МАРШРУТ ДЛЯ ЭКСПЕРТА]: {self.route}\n")
        
        # Ждем нажатия Enter в терминале перед движением (Требование 1.2 и 2.2)
        input("Нажмите ENTER в терминале, чтобы отправить РМК-2 по маршруту...")
        
        # Явный вывод старта движения (Требование регламента!)
        self.get_logger().info("movement_start")
        print("movement_start")
        self.movement_active = True

    def lidar_callback(self, msg):
        """Анализируем данные с Лидара"""
        # Берем центральные лучи перед роботом (например, от -15 до +15 градусов)
        # В массиве msg.ranges углы идут по кругу. Передние лучи обычно в начале и конце массива
        # Для простоты проверим небольшой сектор спереди:
        front_sectors = msg.ranges[:20] + msg.ranges[-20:]
        
        # Очищаем от нулевых или ошибочных значений
        valid_ranges = [r for r in front_sectors if r > 0.05]
        
        if valid_ranges:
            min_distance = min(valid_ranges)
            # Если до препятствия меньше 0.4 метра — бьем тревогу
            if min_distance < 0.4:
                if not self.obstacle_detected:
                    self.get_logger().warn(f"⚠️ ПРЕПЯТСТВИЕ ОБНАРУЖЕНО! Дистанция: {min_distance:.2f}м")
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False

    def control_loop(self):
        """Аналог void loop(). Вызывается по таймеру."""
        if not self.movement_active:
            return

        # Если обнаружили препятствие — немедленно стоп! (Система безопасности)
        if self.obstacle_detected:
            self.stop_robot()
            self.get_logger().error("Робот остановлен системой безопасности Лидара!")
            return

        # Тут должна быть твоя логика движения от маркера к маркеру.
        # В рамках теста мы просто едем вперед. 
        # Когда робот доедет до финального маркера, вызываем завершение:
        # self.finish_movement()
        
        msg = Twist()
        msg.linear.x = 0.2 
        self.pub.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.pub.publish(msg)

    def finish_movement(self):
        """Функция финиша"""
        self.movement_active = False
        self.stop_robot()
        # Явный вывод остановки движения (Требование регламента!)
        self.get_logger().info("movement_stop")
        print("movement_stop")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # Эксперт выдает ID целевого маркера (например, 22)
    # Твой алгоритм (поиск по графу) генерирует массив. Пока пропишем его вручную:
    built_route = [0, 1, 2, 7, 12, 17, 22] 
    
    node = LitusCompetitionNode(built_route)
    
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

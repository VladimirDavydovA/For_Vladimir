import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def angle_to_pi(angle):
    """
    Приводит угол к диапазону от -pi до +pi.
    """

    while angle > math.pi:
        angle -= 2 * math.pi

    while angle < -math.pi:
        angle += 2 * math.pi

    return angle


def quaternion_to_yaw(q):
    """
    ROS хранит ориентацию как quaternion.
    Для движения по плоскости нам нужен только угол вокруг Z (yaw).
    """

    sin_yaw = 2 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1 - 2 * (q.y * q.y + q.z * q.z)

    return math.atan2(sin_yaw, cos_yaw)


class Motion:
    """
    Простое движение робота по готовому маршруту.

    Класс:
    - получает odometry;
    - знает положение робота;
    - поворачивает робота к следующему маркеру;
    - едет к нему;
    - после достижения переходит к следующему.
    """

    def __init__(self, node, graph):

        self.node = node
        self.graph = graph

        # Команды скорости робота
        self.publisher = node.create_publisher(
            Twist,
            "/RMC2/cmd_vel",
            10
        )

        # Положение робота
        node.create_subscription(
            Odometry,
            "/RMC2/odometry",
            self.odom_callback,
            10
        )

        # Управление вызывается каждые 0.05 секунды
        node.create_timer(
            0.05,
            self.control
        )

        self.x = None
        self.y = None
        self.yaw = None

        self.route = []
        self.index = 0

        # Пока active == True, робот выполняет маршрут.
        self.active = False

        # Скорости
        self.linear_speed = 0.20
        self.angular_speed = 0.60

        # Допуски
        self.distance_tolerance = 0.12
        self.angle_tolerance = math.radians(5)

    def odom_callback(self, msg):
        """Получаем текущее положение робота."""

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(
            msg.pose.pose.orientation
        )

    def start(self, route):
        """Запускает движение по маршруту."""

        self.route = route

        # route[0] — клетка, где робот уже находится.
        # Поэтому начинаем с route[1].
        self.index = 1

        if len(route) < 2:
            self.active = False
            return

        self.active = True

        self.node.get_logger().info(
            f"movement_start route={route}"
        )

    def stop(self):
        """Посылает нулевую скорость."""
        self.publisher.publish(Twist())

    def finish(self):
        """Останавливает робот и завершает маршрут."""

        self.stop()
        self.active = False

        self.node.get_logger().info(
            "movement_stop"
        )

    def control(self):
        """
        Основной алгоритм движения.

        1. Берём следующий маркер.
        2. Вычисляем, где он находится.
        3. Если робот смотрит не туда — поворачиваемся.
        4. Если смотрит туда — едем прямо.
        5. Если приехали — берём следующий маркер.
        """

        if not self.active:
            return

        # Ждём первую odometry.
        if self.x is None:
            return

        # Если маршрут закончился.
        if self.index >= len(self.route):
            self.finish()
            return

        target = self.route[self.index]

        target_x, target_y = self.graph.position(target)

        dx = target_x - self.x
        dy = target_y - self.y

        distance = math.hypot(dx, dy)

        # Угол, куда нужно смотреть.
        target_yaw = math.atan2(dy, dx)

        # Ошибка по углу.
        error = angle_to_pi(
            target_yaw - self.yaw
        )

        # --------------------------------------------------
        # Уже приехали к маркеру
        # --------------------------------------------------

        if distance < self.distance_tolerance:

            self.node.get_logger().info(
                f"marker_reached: {target}"
            )

            self.stop()
            self.index += 1

            if self.index >= len(self.route):
                self.finish()

            return

        command = Twist()

        # --------------------------------------------------
        # Сначала поворачиваемся
        # --------------------------------------------------

        if abs(error) > self.angle_tolerance:

            command.linear.x = 0.0

            if error > 0:
                command.angular.z = self.angular_speed
            else:
                command.angular.z = -self.angular_speed

        # --------------------------------------------------
        # Потом едем прямо
        # --------------------------------------------------

        else:

            command.linear.x = self.linear_speed

            # Небольшая коррекция курса.
            command.angular.z = 1.2 * error

            # Ограничиваем скорость поворота.
            if command.angular.z > 0.30:
                command.angular.z = 0.30

            if command.angular.z < -0.30:
                command.angular.z = -0.30

        self.publisher.publish(command)

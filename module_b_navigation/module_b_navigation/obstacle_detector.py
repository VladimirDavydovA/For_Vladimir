import math

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ObstacleDetector:
    """
    Ищет ОДНО новое препятствие.

    Идея очень простая:

    1. До появления препятствия запоминаем lidar.
    2. Пользователь ставит препятствие.
    3. Получаем новый lidar.
    4. Ищем лучи, у которых расстояние сильно уменьшилось.
    5. Переводим эти точки в координаты мира.
    6. Смотрим, возле какого маркера больше всего таких точек.
    """

    def __init__(self, node, graph):

        self.node = node
        self.graph = graph

        # Последнее сообщение лидара.
        self.scan = None

        # Расстояния до появления препятствия.
        self.baseline = None

        node.create_subscription(
            LaserScan,
            "/RMC2/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Если расстояние уменьшилось минимум на 20 см,
        # считаем, что появился новый объект.
        self.min_change = 0.20

        # Насколько далеко точка может быть от центра клетки.
        self.max_marker_distance = 0.75

        # Минимум lidar-точек, чтобы поверить результату.
        self.min_votes = 3

    def scan_callback(self, msg):
        """Просто сохраняем самый свежий scan."""
        self.scan = msg

    def save_baseline(self):
        """
        Запоминаем lidar до появления препятствия.
        """

        if self.scan is None:
            return False

        self.baseline = list(self.scan.ranges)
        return True

    def find_obstacle(self, robot_x, robot_y, robot_yaw, excluded=None):
        """
        Возвращает номер занятой клетки.
        Если препятствие не найдено — возвращает None.
        """

        if excluded is None:
            excluded = set()

        if self.scan is None:
            return None

        if self.baseline is None:
            return None

        if len(self.baseline) != len(self.scan.ranges):
            return None

        # votes:
        # ключ   = номер маркера
        # значение = сколько lidar-точек попало возле него
        votes = {}

        # Для красивого лога сохраняем координаты точек.
        points = {}

        for i in range(len(self.scan.ranges)):

            old_range = self.baseline[i]
            new_range = self.scan.ranges[i]

            # Текущая дальность должна быть нормальной.
            if not math.isfinite(new_range):
                continue

            if new_range < self.scan.range_min:
                continue

            if new_range > self.scan.range_max:
                continue

            # Проверяем, появился ли здесь новый объект.
            new_object = False

            # Раньше лидар ничего не видел,
            # а теперь увидел объект.
            if not math.isfinite(old_range):
                new_object = True

            # Или объект стал как минимум на 20 см ближе.
            elif old_range - new_range >= self.min_change:
                new_object = True

            if not new_object:
                continue

            # Угол конкретного луча лидара.
            angle = (
                self.scan.angle_min
                + i * self.scan.angle_increment
            )

            # ------------------------------------------
            # Координаты точки относительно робота
            # ------------------------------------------

            local_x = new_range * math.cos(angle)
            local_y = new_range * math.sin(angle)

            # ------------------------------------------
            # Переводим точку в координаты мира
            # ------------------------------------------

            world_x = robot_x + (
                local_x * math.cos(robot_yaw)
                - local_y * math.sin(robot_yaw)
            )

            world_y = robot_y + (
                local_x * math.sin(robot_yaw)
                + local_y * math.cos(robot_yaw)
            )

            # Определяем ближайший маркер.
            marker = self.graph.nearest_marker(
                world_x,
                world_y,
                self.max_marker_distance,
                excluded
            )

            if marker is None:
                continue

            # Голосуем за этот маркер.
            votes[marker] = votes.get(marker, 0) + 1

            if marker not in points:
                points[marker] = []

            points[marker].append(
                (world_x, world_y)
            )

        if not votes:
            return None

        # Маркер с максимальным количеством голосов.
        best_marker = max(
            votes,
            key=votes.get
        )

        self.node.get_logger().info(
            f"obstacle_votes={dict(sorted(votes.items()))}"
        )

        if votes[best_marker] < self.min_votes:
            return None

        # Средняя координата lidar-точек,
        # которые проголосовали за победивший маркер.
        best_points = points[best_marker]

        avg_x = sum(p[0] for p in best_points) / len(best_points)
        avg_y = sum(p[1] for p in best_points) / len(best_points)

        self.node.get_logger().info(
            f"obstacle_world_position≈({avg_x:.3f}, {avg_y:.3f})"
        )

        return best_marker

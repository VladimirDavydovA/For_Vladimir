import time

import rclpy
from rclpy.node import Node

from module_b_navigation.graph import Graph
from module_b_navigation.planner import Planner
from module_b_navigation.motion_controller import Motion
from module_b_navigation.obstacle_detector import ObstacleDetector


def wait_robot(node, motion):
    """
    Пока робот едет, даём ROS обрабатывать сообщения.

    spin_once означает:
    "обработай новые сообщения ROS один раз".
    """

    while motion.active and rclpy.ok():
        rclpy.spin_once(
            node,
            timeout_sec=0.1
        )


def wait_for_new_scan(node):
    """
    После того как пользователь поставил препятствие,
    несколько раз обрабатываем ROS-сообщения,
    чтобы точно получить свежий lidar scan.
    """

    for i in range(10):
        rclpy.spin_once(
            node,
            timeout_sec=0.1
        )


def print_route(title, start, target, blocked, route, distance, turns):
    """Красиво печатает найденный маршрут."""

    print()
    print("========================================")
    print(title)
    print("========================================")
    print("Start marker :", start)
    print("Target marker:", target)
    print("Blocked      :", sorted(blocked))
    print()
    print("ROUTE    :", route)
    print("DISTANCE :", distance)
    print("TURNS    :", turns)
    print("========================================")
    print()


def main(args=None):

    # ------------------------------------------------------
    # Запускаем ROS
    # ------------------------------------------------------

    rclpy.init(args=args)

    node = Node("module_b_navigation")

    # ------------------------------------------------------
    # Параметры запуска
    #
    # ros2 run ... --ros-args
    #     -p start_id:=0
    #     -p target_id:=22
    # ------------------------------------------------------

    node.declare_parameter("start_id", 0)
    node.declare_parameter("target_id", 22)

    start = node.get_parameter("start_id").value
    target = node.get_parameter("target_id").value

    # ------------------------------------------------------
    # Создаём основные части программы
    # ------------------------------------------------------

    graph = Graph()
    planner = Planner(graph)
    motion = Motion(node, graph)
    detector = ObstacleDetector(node, graph)

    try:

        # ==================================================
        # 1. СТРОИМ МАРШРУТ ТУДА
        # ==================================================

        blocked = set()

        node.get_logger().info(
            "OUTBOUND_PLANNING_START"
        )

        route, distance, turns = planner.plan(
            start,
            target,
            blocked
        )

        print_route(
            "OUTBOUND ROUTE",
            start,
            target,
            blocked,
            route,
            distance,
            turns
        )

        # ==================================================
        # 2. ЕДЕМ К ЦЕЛИ
        # ==================================================

        node.get_logger().info(
            "OUTBOUND_MOVEMENT_START"
        )

        motion.start(route)

        wait_robot(
            node,
            motion
        )

        node.get_logger().info(
            "OUTBOUND_MOVEMENT_STOP"
        )

        # ==================================================
        # 3. ЗАПОМИНАЕМ ЛИДАР ДО ПРЕПЯТСТВИЯ
        # ==================================================

        if not detector.save_baseline():

            node.get_logger().error(
                "BASELINE_SCAN_NOT_AVAILABLE"
            )

            return

        node.get_logger().info(
            "BASELINE_SCAN_CAPTURED"
        )

        # ==================================================
        # 4. ПОЛЬЗОВАТЕЛЬ СТАВИТ ПРЕПЯТСТВИЕ
        # ==================================================

        print()
        print("========================================")
        print("РОБОТ ОСТАНОВИЛСЯ В ЦЕЛЕВОЙ ТОЧКЕ")
        print("========================================")
        print("Поставьте ОДНО препятствие в Webots.")
        print("После этого введите y и нажмите Enter.")
        print("========================================")

        while True:

            answer = input(
                "Препятствие установлено? [y]: "
            )

            if answer.lower().strip() == "y":
                break

        # ==================================================
        # 5. ПОЛУЧАЕМ СВЕЖИЙ ЛИДАР
        # ==================================================

        wait_for_new_scan(node)

        # ==================================================
        # 6. ИЩЕМ ЗАНЯТУЮ КЛЕТКУ
        # ==================================================

        node.get_logger().info(
            "OBSTACLE_SCAN_START"
        )

        blocked_marker = detector.find_obstacle(
            motion.x,
            motion.y,
            motion.yaw,
            excluded={start, target}
        )

        if blocked_marker is None:

            print()
            print("Препятствие не найдено.")
            print()

            return

        print()
        print("========================================")
        print(
            "OBSTACLE DETECTED ON MARKER",
            blocked_marker
        )
        print("========================================")
        print()

        # Теперь эта клетка запрещена для планировщика.
        blocked = {blocked_marker}

        # ==================================================
        # 7. СТРОИМ ОБРАТНЫЙ МАРШРУТ
        # ==================================================

        node.get_logger().info(
            "RETURN_PLANNING_START"
        )

        return_route, distance, turns = planner.plan(
            target,
            start,
            blocked
        )

        print_route(
            "RETURN ROUTE",
            target,
            start,
            blocked,
            return_route,
            distance,
            turns
        )

        # ==================================================
        # 8. ЕДЕМ ОБРАТНО
        # ==================================================

        node.get_logger().info(
            "RETURN_MOVEMENT_START"
        )

        motion.start(return_route)

        wait_robot(
            node,
            motion
        )

        node.get_logger().info(
            "RETURN_MOVEMENT_STOP"
        )

        node.get_logger().info(
            "MODULE_B_DONE"
        )

        print()
        print("========================================")
        print("MODULE B COMPLETE")
        print("Robot returned to the start marker.")
        print("========================================")
        print()

    except KeyboardInterrupt:
        pass

    finally:

        # На всякий случай всегда останавливаем робот.
        motion.stop()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

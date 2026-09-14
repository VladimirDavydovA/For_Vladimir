from collections import deque


class Planner:
    """
    Строит маршрут по клеткам.

    Критерии:
    1. Сначала ищем самый короткий маршрут.
    2. Если коротких маршрутов несколько,
       выбираем маршрут с меньшим количеством поворотов.
    3. Если и это одинаково,
       выбираем маршрут с меньшими ID маркеров.
    """

    def __init__(self, graph):
        self.graph = graph

    def count_turns(self, route):
        """Считает количество поворотов в маршруте."""

        if len(route) < 3:
            return 0

        turns = 0

        old_direction = self.graph.direction(
            route[0],
            route[1]
        )

        for i in range(1, len(route) - 1):

            new_direction = self.graph.direction(
                route[i],
                route[i + 1]
            )

            if new_direction != old_direction:
                turns += 1

            old_direction = new_direction

        return turns

    def plan(self, start, target, blocked=None):
        """
        Возвращает:

            route, distance, turns

        Например:

            [0, 1, 2, 7, 12, 17, 22], 6, 1
        """

        if blocked is None:
            blocked = set()

        if not self.graph.valid(start):
            raise ValueError("Неверный стартовый маркер")

        if not self.graph.valid(target):
            raise ValueError("Неверный целевой маркер")

        if start in blocked:
            raise ValueError("Стартовая клетка занята")

        if target in blocked:
            raise ValueError("Целевая клетка занята")

        if start == target:
            return [start], 0, 0

        # В очереди лежат целые маршруты.
        queue = deque()
        queue.append([start])

        # Здесь будем хранить все самые короткие маршруты до цели.
        found_routes = []
        shortest_length = None

        while queue:

            route = queue.popleft()

            # Если уже нашли более короткий маршрут,
            # этот маршрут продолжать бессмысленно.
            if shortest_length is not None:
                if len(route) > shortest_length:
                    break

            current = route[-1]

            if current == target:
                shortest_length = len(route)
                found_routes.append(route)
                continue

            for next_marker in self.graph.neighbors(current, blocked):

                # Не заходим второй раз в клетку,
                # которая уже есть в этом маршруте.
                if next_marker in route:
                    continue

                new_route = route + [next_marker]
                queue.append(new_route)

        if not found_routes:
            raise RuntimeError("Маршрут не найден")

        # Из всех кратчайших маршрутов выбираем:
        # сначала по количеству поворотов,
        # потом по самим ID маркеров.
        best_route = min(
            found_routes,
            key=lambda route: (
                self.count_turns(route),
                route
            )
        )

        distance = len(best_route) - 1
        turns = self.count_turns(best_route)

        return best_route, distance, turns

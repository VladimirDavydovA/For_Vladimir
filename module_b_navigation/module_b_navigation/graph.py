import math


class Graph:
    """
    Простая карта поля 5x5.

    Нумерация маркеров:

        20 21 22 23 24
        15 16 17 18 19
        10 11 12 13 14
         5  6  7  8  9
         0  1  2  3  4

    Расстояние между соседними маркерами принимаем равным 1 метру.
    """

    SIZE = 5

    def valid(self, marker):
        """Проверяет, существует ли такой маркер."""
        return 0 <= marker < 25

    def position(self, marker):
        """
        Возвращает координаты маркера.

        Например:
        marker = 12

        x = 12 % 5  = 2
        y = 12 // 5 = 2

        Значит маркер 12 находится в точке (2, 2).
        """
        x = marker % self.SIZE
        y = marker // self.SIZE
        return float(x), float(y)

    def neighbors(self, marker, blocked=None):
        """
        Возвращает соседние клетки.

        Порядок:
        вправо -> вверх -> влево -> вниз.
        """

        if blocked is None:
            blocked = set()

        row = marker // self.SIZE
        col = marker % self.SIZE

        result = []

        # Вправо
        if col < self.SIZE - 1:
            result.append(marker + 1)

        # Вверх
        if row < self.SIZE - 1:
            result.append(marker + self.SIZE)

        # Влево
        if col > 0:
            result.append(marker - 1)

        # Вниз
        if row > 0:
            result.append(marker - self.SIZE)

        # Убираем занятые клетки
        return [m for m in result if m not in blocked]

    def direction(self, a, b):
        """
        Возвращает направление движения между двумя соседними маркерами.
        Нужно только для подсчёта поворотов.
        """

        ax, ay = self.position(a)
        bx, by = self.position(b)

        if bx > ax:
            return "RIGHT"
        if bx < ax:
            return "LEFT"
        if by > ay:
            return "UP"
        if by < ay:
            return "DOWN"

    def nearest_marker(self, x, y, max_distance=0.75, excluded=None):
        """
        По координатам точки ищет ближайший маркер.

        Это используется для лидара:
        лидар дал координату препятствия -> определяем номер клетки.
        """

        if excluded is None:
            excluded = set()

        best_marker = None
        best_distance = 999999.0

        for marker in range(25):

            if marker in excluded:
                continue

            mx, my = self.position(marker)

            distance = math.hypot(x - mx, y - my)

            if distance < best_distance:
                best_distance = distance
                best_marker = marker

        if best_distance > max_distance:
            return None

        return best_marker

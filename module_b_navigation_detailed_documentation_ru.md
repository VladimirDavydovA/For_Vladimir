# Подробная документация проекта `module_b_navigation`

## 1. Назначение

Эта документация относится к текущей максимально упрощённой версии кода из пяти файлов:

```text
module_b_navigation/
├── main.py
├── graph.py
├── planner.py
├── motion_controller.py
└── obstacle_detector.py
```

Цель документа — объяснить проект человеку, который знает базовый Python, но пока мало знаком с ROS 2. Здесь разобраны структура, алгоритмы, математическая логика, синтаксис Python, ROS 2-механизмы, команды запуска, способы настройки, адаптация к другому роботу и другому полю, а также типичные ошибки.

---

## 2. Что делает программа

Полный сценарий:

```text
1. Получить start_id и target_id
2. Построить маршрут от start до target
3. Проехать маршрут
4. Остановиться
5. Сохранить текущий lidar scan как baseline
6. Пользователь ставит одно препятствие
7. Пользователь вводит y
8. Получить свежий lidar scan
9. Найти точки, которые стали ближе
10. Перевести их из координат лидара в мировые координаты
11. Определить ближайший marker
12. Выбрать marker с максимальным числом голосов
13. Добавить его в blocked
14. Построить новый маршрут target -> start
15. Вернуться обратно
```

Ключевая идея проекта:

```text
Graph            знает карту
Planner          выбирает маршрут
Motion           двигает робот
ObstacleDetector определяет занятую клетку
main.py          задаёт порядок работы
```

---

## 3. Архитектура

```text
                         main.py
                            |
          +-----------------+------------------+
          |                 |                  |
          v                 v                  v
       Graph             Planner             Motion
          ^                 |                  |
          |                 |                  |
          +-----------------+------------------+
          |
          v
   ObstacleDetector
```

`Graph` используется почти всеми остальными компонентами, потому что он содержит единую модель поля.

В текущей версии создаётся один объект ROS `Node`, а `Motion` и `ObstacleDetector` создают через него publisher, subscriber и timer. Это упрощает код по сравнению с архитектурой, где каждый класс является отдельным ROS-узлом.


---

## 4. ROS 2: что именно используется

### `/RMC2/odometry`

Тип:

```text
nav_msgs/msg/Odometry
```

Нужен для получения текущих:

```text
x
y
orientation
```

Из quaternion ориентации вычисляется `yaw`.

### `/RMC2/cmd_vel`

Тип:

```text
geometry_msgs/msg/Twist
```

Используются:

```text
linear.x   — движение вперёд
angular.z  — вращение
```

### `/RMC2/scan`

Тип:

```text
sensor_msgs/msg/LaserScan
```

Используются:

```text
angle_min
angle_increment
range_min
range_max
ranges[]
```

---

## 5. Поле 5×5

Нумерация:

```text
20 21 22 23 24
15 16 17 18 19
10 11 12 13 14
 5  6  7  8  9
 0  1  2  3  4
```

Координаты вычисляются:

```python
x = marker % 5
y = marker // 5
```

Пример для `17`:

```text
17 % 5  = 2
17 // 5 = 3
```

Значит:

```text
marker 17 -> (2, 3)
```

### `%`

Остаток от деления.

### `//`

Целочисленное деление.

Это очень удобная схема для регулярной прямоугольной сетки.


---

# 6. `graph.py`

## 6.1 Назначение

`graph.py` не управляет роботом и не работает с ROS. Он описывает только карту.

Он умеет:

- проверять существование marker;
- получать координаты marker;
- получать соседей;
- учитывать blocked;
- определять направление перехода;
- находить ближайший marker к произвольной точке.

## 6.2 `class Graph`

```python
class Graph:
```

Класс — шаблон объекта. Создание:

```python
graph = Graph()
```

## 6.3 `SIZE = 5`

Размер одной стороны сетки.

Сейчас код рассчитан на 25 клеток.

## 6.4 `valid()`

```python
return 0 <= marker < 25
```

Это сокращение для:

```python
return marker >= 0 and marker < 25
```

## 6.5 `position()`

```python
x = marker % self.SIZE
y = marker // self.SIZE
return float(x), float(y)
```

`return a, b` фактически возвращает кортеж `(a, b)`.

Поэтому:

```python
x, y = graph.position(12)
```

— это распаковка кортежа.

## 6.6 `neighbors()`

Соседи формируются в порядке:

```text
RIGHT
UP
LEFT
DOWN
```

В конце:

```python
return [m for m in result if m not in blocked]
```

Это list comprehension. Развёрнутый эквивалент:

```python
filtered = []
for m in result:
    if m not in blocked:
        filtered.append(m)
return filtered
```

## 6.7 Почему `blocked=None`

Безопасный шаблон:

```python
def neighbors(..., blocked=None):
    if blocked is None:
        blocked = set()
```

Не стоит писать изменяемый объект вроде `set()` прямо в аргументах по умолчанию.

## 6.8 `set`

Множество:

```python
blocked = {12, 17}
```

Плюсы:

- нет дубликатов;
- быстрое `in`;
- логично описывает набор запрещённых клеток.

## 6.9 `direction()`

Используется планировщиком для подсчёта поворотов.

Например:

```text
12 -> 13 = RIGHT
12 -> 17 = UP
```

## 6.10 `nearest_marker()`

Перебирает все marker ID и считает:

```python
distance = math.hypot(x - mx, y - my)
```

`math.hypot(dx, dy)` — расстояние по Пифагору.

Порог:

```python
max_distance=0.75
```

не позволяет присвоить marker точке, которая находится слишком далеко от центра любой клетки.

`excluded` позволяет исключить start и target из кандидатов при поиске препятствия.


---

# 7. `planner.py`

## 7.1 Назначение

Планировщик получает:

```text
start
target
blocked
```

и возвращает:

```text
route
distance
turns
```

## 7.2 Алгоритм BFS

Используется поиск в ширину — Breadth-First Search.

Идея:

```text
сначала все маршруты длины 1
потом длины 2
потом длины 3
...
```

Поэтому найденная минимальная глубина соответствует минимальному числу переходов.

## 7.3 `deque`

```python
from collections import deque
```

Очередь:

```python
queue.append(value)
queue.popleft()
```

Первым обрабатывается тот элемент, который был добавлен раньше.

## 7.4 Почему в очереди лежит маршрут целиком

Например:

```text
[0]
[0, 1]
[0, 5]
[0, 1, 2]
...
```

Это не самый экономный вариант для огромных карт, но для 25 клеток он очень понятен.

## 7.5 Защита от цикла

```python
if next_marker in route:
    continue
```

Не допускает:

```text
0 -> 1 -> 0 -> 1 -> ...
```

`continue` означает: пропустить остаток текущей итерации цикла.

## 7.6 Почему ищутся все кратчайшие маршруты

Нужен не только минимальный путь, но и меньшее число поворотов.

Поэтому сначала собираются все варианты минимальной длины:

```python
found_routes
```

а затем выбирается лучший.

## 7.7 `count_turns()`

Пример:

```text
0 -> 1 RIGHT
1 -> 2 RIGHT
2 -> 7 UP
7 ->12 UP
12->17 UP
17->22 UP
```

Смена направления одна:

```text
RIGHT -> UP
```

Значит `turns = 1`.

## 7.8 Самая сложная строка

```python
best_route = min(
    found_routes,
    key=lambda route: (
        self.count_turns(route),
        route
    )
)
```

`min(..., key=...)` говорит Python: сравнивать элементы по специальному критерию.

`lambda route: ...` — маленькая безымянная функция.

Эквивалент понятнее:

```python
def route_key(route):
    return self.count_turns(route), route

best_route = min(found_routes, key=route_key)
```

Сначала сравнивается число поворотов, а если оно одинаково — сами списки ID. Это даёт детерминированный выбор.


---

# 8. `motion_controller.py`

## 8.1 Что делает Motion

`Motion`:

- читает odometry;
- хранит `x`, `y`, `yaw`;
- принимает готовый маршрут;
- выбирает следующую точку;
- вычисляет расстояние и требуемый угол;
- поворачивает;
- едет вперёд;
- фиксирует достижение marker;
- останавливается после окончания маршрута.

## 8.2 Publisher

```python
node.create_publisher(
    Twist,
    "/RMC2/cmd_vel",
    10
)
```

Создаёт отправителя сообщений типа `Twist`.

Отправка:

```python
self.publisher.publish(command)
```

## 8.3 Subscriber

```python
node.create_subscription(
    Odometry,
    "/RMC2/odometry",
    self.odom_callback,
    10
)
```

Когда приходит сообщение, ROS вызывает:

```python
self.odom_callback(msg)
```

Это callback.

## 8.4 Timer

```python
node.create_timer(
    0.05,
    self.control
)
```

`control()` вызывается примерно 20 раз в секунду.

## 8.5 `self`

`self` — конкретный экземпляр класса.

```python
self.x
```

хранится внутри конкретного объекта `motion`.

## 8.6 `__init__`

Конструктор автоматически вызывается:

```python
motion = Motion(node, graph)
```

## 8.7 `None`

Изначально:

```python
self.x = None
```

Это означает: odometry ещё не получена.

Правильная проверка:

```python
if self.x is None:
```

## 8.8 Quaternion -> yaw

ROS Odometry хранит ориентацию как quaternion.

Для плоской навигации нужен угол вокруг Z — `yaw`.

Функция `quaternion_to_yaw()` получает quaternion и возвращает угол в радианах.

## 8.9 Нормализация угла

`angle_to_pi()` приводит ошибку к:

```text
[-pi, +pi]
```

Это важно около перехода `+180°/-180°`.

## 8.10 `start(route)`

```python
self.index = 1
```

Почему 1?

Если маршрут:

```text
[0, 1, 2]
```

робот уже находится на `0`, поэтому первая цель — `route[1]`.

## 8.11 `active`

```text
True  -> маршрут выполняется
False -> маршрут завершён
```

Эта переменная позволяет `main.py` ждать окончания движения.

## 8.12 Алгоритм `control()`

```text
если не active -> return
если нет odometry -> return
если маршрут закончился -> finish
взять target marker
получить target_x, target_y
посчитать dx, dy
посчитать distance
посчитать target_yaw
посчитать angular error
если близко -> marker reached
иначе если угол большой -> вращаться
иначе -> ехать вперёд с коррекцией
```

## 8.13 Расстояние

```python
distance = math.hypot(dx, dy)
```

## 8.14 Угол на цель

```python
target_yaw = math.atan2(dy, dx)
```

`atan2` корректно учитывает четверть координатной плоскости.

## 8.15 Допуски

```python
self.distance_tolerance = 0.12
self.angle_tolerance = math.radians(5)
```

Marker считается достигнутым в радиусе 12 см.

Перед движением вперёд ошибка угла должна быть меньше примерно 5°.

## 8.16 Простое управление

Если ошибка угла большая:

```text
linear.x = 0
angular.z = ±angular_speed
```

Если направление достаточно хорошее:

```text
linear.x = linear_speed
angular.z = 1.2 * error
```

Последняя строка — простейшая пропорциональная коррекция курса.

## 8.17 Ограничение коррекции

```text
-0.30 <= angular.z <= 0.30
```

чтобы робот не делал слишком резкий поворот одновременно с движением вперёд.

## 8.18 Остановка

```python
self.publisher.publish(Twist())
```

Новый `Twist()` имеет нулевые скорости.


---

# 9. `obstacle_detector.py`

## 9.1 Основная идея

Детектор не пытается распознать объект по форме.

Он использует контролируемый эксперимент:

```text
scan ДО препятствия
scan ПОСЛЕ препятствия
```

и ищет направления, где объект стал заметно ближе.

## 9.2 QoS

Подписка использует:

```python
qos_profile_sensor_data
```

Это критично.

Ранее обычный QoS привёл к:

```text
incompatible QoS
RELIABILITY
```

и `/RMC2/scan` не принимался.

Поэтому `qos_profile_sensor_data` нельзя убирать без проверки QoS publisher.

## 9.3 `scan_callback`

Просто сохраняет последний scan:

```python
self.scan = msg
```

## 9.4 `save_baseline()`

```python
self.baseline = list(self.scan.ranges)
```

Создаёт копию исходных расстояний.

## 9.5 Проверки диапазона

Отбрасываются:

- `inf`;
- `NaN`;
- значения ниже `range_min`;
- значения выше `range_max`.

```python
math.isfinite(value)
```

проверяет, что число конечное.

## 9.6 Условие нового препятствия

Основное:

```python
old_range - new_range >= self.min_change
```

Сейчас:

```text
min_change = 0.20 м
```

Пример:

```text
было 3.0 м
стало 1.4 м
разница 1.6 м
-> новый объект
```

Если старое расстояние было `inf`, а теперь появилось конечное измерение, это тоже считается новым объектом.

## 9.7 Угол луча

```python
angle = angle_min + i * angle_increment
```

`i` — индекс элемента в `ranges`.

## 9.8 Полярные -> локальные XY

```python
local_x = r * cos(angle)
local_y = r * sin(angle)
```

## 9.9 Локальные -> мировые XY

```python
world_x = robot_x + (
    local_x * cos(robot_yaw)
    - local_y * sin(robot_yaw)
)

world_y = robot_y + (
    local_x * sin(robot_yaw)
    + local_y * cos(robot_yaw)
)
```

Математически:

```text
[world_x]   [robot_x]   [ cos(yaw) -sin(yaw)] [local_x]
[world_y] = [robot_y] + [ sin(yaw)  cos(yaw)] [local_y]
```

Это поворот точки и перенос на положение робота.

## 9.10 Важнейшее геометрическое допущение

Код предполагает, что система координат LaserScan согласована с базовой системой робота.

Если лидар на реальном роботе:

- смещён;
- развернут;
- публикует данные в другом frame;

нужно использовать TF или явное преобразование sensor frame -> base frame.

## 9.11 Голосование

Каждая новая lidar-точка получает ближайший marker.

```python
votes[marker] = votes.get(marker, 0) + 1
```

`.get(marker, 0)` означает: если ключа нет — считать его значение 0.

Пример:

```python
votes = {
    12: 30,
    21: 6,
    23: 7
}
```

## 9.12 Выбор победителя

```python
best_marker = max(votes, key=votes.get)
```

Выбирается ключ с максимальным значением.

## 9.13 `min_votes`

```python
self.min_votes = 3
```

Защита от единичных ложных точек.

## 9.14 Средняя координата

`points[marker]` хранит мировые координаты точек, проголосовавших за marker. Среднее используется для диагностического лога:

```text
obstacle_world_position≈(...)
```


---

# 10. `main.py`

## 10.1 Почему он простой

Вместо большого конечного автомата программа выполняется почти последовательно сверху вниз.

Это сознательное упрощение.

## 10.2 Инициализация

```python
rclpy.init(args=args)
node = Node("module_b_navigation")
```

`rclpy.init()` запускает ROS-контекст.

`Node(...)` создаёт ROS-узел.

## 10.3 ROS parameters

```python
node.declare_parameter("start_id", 0)
node.declare_parameter("target_id", 22)
```

Значения можно переопределить из терминала.

```python
start = node.get_parameter("start_id").value
```

## 10.4 Создание частей системы

```python
graph = Graph()
planner = Planner(graph)
motion = Motion(node, graph)
detector = ObstacleDetector(node, graph)
```

## 10.5 Планирование туда

```python
blocked = set()

route, distance, turns = planner.plan(
    start,
    target,
    blocked
)
```

## 10.6 Распаковка

Функция возвращает три значения:

```python
return route, distance, turns
```

Поэтому:

```python
route, distance, turns = ...
```

## 10.7 `wait_robot()`

```python
while motion.active and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
```

Это центр упрощённой архитектуры.

`spin_once()` даёт ROS один раз обработать:

- сообщения odometry;
- LaserScan;
- timer `Motion.control()`.

Потом управление возвращается в Python-цикл.

## 10.8 Почему не `rclpy.spin(node)`

`spin()` обычно блокирует выполнение до завершения node.

Тогда пришлось бы переносить сценарий в callback-и, state machine или потоки.

`spin_once()` позволяет сохранить обычный читаемый сценарий.

## 10.9 Ожидание пользователя

```python
while True:
    answer = input(...)
    if answer.lower().strip() == "y":
        break
```

`strip()` убирает пробелы.

`lower()` приводит строку к нижнему регистру.

`break` завершает цикл.

## 10.10 Почему блокирующий `input()` допустим здесь

Пока `input()` ждёт пользователя, ROS callbacks не обрабатываются.

В общем случае это плохо, но здесь:

- робот уже остановлен;
- baseline уже сохранён;
- мы специально хотим паузу;
- после `y` вызывается серия `spin_once()` для получения свежего scan.

Поэтому для учебного сценария это приемлемый компромисс.

## 10.11 Свежий scan

```python
for i in range(10):
    rclpy.spin_once(node, timeout_sec=0.1)
```

Лучше стилистически можно написать:

```python
for _ in range(10):
```

если индекс не используется.

## 10.12 Поиск препятствия

```python
blocked_marker = detector.find_obstacle(
    motion.x,
    motion.y,
    motion.yaw,
    excluded={start, target}
)
```

## 10.13 Почему маршрут назад строится заново

```python
return_route, distance, turns = planner.plan(
    target,
    start,
    blocked
)
```

Это принципиально не:

```python
route[::-1]
```

или `reversed(route)`.

Если на старом пути появился obstacle, обратный старый путь был бы опасен.

## 10.14 `try / except / finally`

`KeyboardInterrupt` ловит Ctrl+C.

`finally` выполняется в любом случае:

```python
motion.stop()
node.destroy_node()
rclpy.shutdown()
```

Для робота особенно важно гарантированно отправить нулевую скорость.


---

# 11. Полный пример выполнения

Команда:

```bash
ros2 run module_b_navigation module_b     --ros-args     -p start_id:=0     -p target_id:=22
```

Планировщик:

```text
[0, 1, 2, 7, 12, 17, 22]
```

Motion последовательно достигает:

```text
1
2
7
12
17
22
```

После остановки:

```text
BASELINE_SCAN_CAPTURED
```

Пользователь ставит obstacle и вводит `y`.

Детектор может вывести:

```text
obstacle_votes={12: 30, 14: 3, 15: 1, 19: 1, 20: 2, 21: 6, 23: 7}
obstacle_world_position≈(2.025, 2.265)
```

Результат:

```text
blocked_marker = 12
blocked = {12}
```

Обратный план:

```text
[22, 21, 20, 15, 10, 5, 0]
```

Робот возвращается:

```text
21
20
15
10
5
0
```

И завершает сценарий.


---

# 12. Команды сборки и запуска

## 12.1 Перейти в workspace

```bash
cd ~/ros2_ws
```

## 12.2 Подключить системный ROS 2, если нужно

```bash
source /opt/ros/jazzy/setup.bash
```

## 12.3 Сборка

```bash
colcon build --symlink-install
```

## 12.4 Подключить workspace

```bash
source install/setup.bash
```

## 12.5 Проверить пакет

```bash
ros2 pkg list | grep module_b_navigation
```

## 12.6 Проверить executable

```bash
ros2 pkg executables module_b_navigation
```

## 12.7 Запуск

```bash
ros2 run module_b_navigation module_b     --ros-args     -p start_id:=0     -p target_id:=22
```

Одной строкой:

```bash
ros2 run module_b_navigation module_b --ros-args -p start_id:=0 -p target_id:=22
```

Обратный слеш `\` в Bash означает продолжение команды на следующей строке.

## 12.8 Автоматический source

В `~/.bashrc` можно добавить:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 12.9 Важное замечание про `setup.py`

Эти пять исходных файлов не содержат packaging-файл пакета. Чтобы `ros2 run module_b_navigation module_b` работал, существующий пакет должен экспортировать executable. Для Python ROS 2 package типовая идея:

```python
'console_scripts': [
    'module_b = module_b_navigation.main:main',
]
```

Это пример, а не утверждение о точном текущем `setup.py`.


---

# 13. Диагностика ROS

Список топиков:

```bash
ros2 topic list
```

Одно сообщение odometry:

```bash
ros2 topic echo /RMC2/odometry --once
```

Одно сообщение лидара:

```bash
ros2 topic echo /RMC2/scan --once
```

Подробный QoS лидара:

```bash
ros2 topic info /RMC2/scan --verbose
```

Частота:

```bash
ros2 topic hz /RMC2/scan
ros2 topic hz /RMC2/odometry
```

Проверка команд скорости:

```bash
ros2 topic echo /RMC2/cmd_vel
```

Если `cmd_vel` публикуется, а робот не движется, проблема, вероятно, ниже уровня данного Python-кода.


---

# 14. Настраиваемые параметры

## Движение

```python
self.linear_speed = 0.20
self.angular_speed = 0.60
self.distance_tolerance = 0.12
self.angle_tolerance = math.radians(5)
```

Также:

```python
command.angular.z = 1.2 * error
```

`1.2` — коэффициент коррекции.

Ограничение:

```text
±0.30
```

## Детектор

```python
self.min_change = 0.20
self.max_marker_distance = 0.75
self.min_votes = 3
```

### Если obstacle не находится

Можно уменьшать `min_change` или `min_votes`, но растёт риск ложных срабатываний.

### Если выбирается неправильная клетка

Проверить:

- ориентацию LaserScan;
- yaw;
- размеры сетки;
- положение робота;
- `max_marker_distance`;
- наличие смещения сенсора;
- неподвижность робота между сканами.


---

# 15. Адаптация

## 15.1 Другой шаг сетки

Сейчас предполагается 1 м.

Для 0.5 м:

```python
CELL_SIZE = 0.5
x = (marker % SIZE) * CELL_SIZE
y = (marker // SIZE) * CELL_SIZE
```

## 15.2 Другая квадратная сетка

Для 6×6:

```python
SIZE = 6
```

и заменить жёсткие `25` на:

```python
SIZE * SIZE
```

## 15.3 Прямоугольная сетка

Лучше:

```python
WIDTH = 8
HEIGHT = 4

x = marker % WIDTH
y = marker // WIDTH
```

## 15.4 Нерегулярные точки

Для реальной фермы координаты могут быть произвольными:

```python
POSITIONS = {
    0: (0.0, 0.0),
    1: (1.2, 0.1),
    2: (2.5, 0.1),
}
```

Тогда `position()` возвращает данные из словаря.

## 15.5 Нерегулярные связи

Можно явно хранить:

```python
NEIGHBORS = {
    0: [1, 5],
    1: [0, 2],
}
```

Это удобнее для настоящих рядов и технологических проходов.

## 15.6 Другие topic names

Если новый робот использует:

```text
/odom
/cmd_vel
/scan
```

заменяются только строки топиков.

## 15.7 Другой тип управления

Если робот управляется не `Twist`, а командами колёс или CAN, менять в первую очередь `Motion`. `Graph` и `Planner` можно сохранить.

## 15.8 Нет odometry

Нужен другой источник `x, y, yaw`:

- wheel odometry;
- visual odometry;
- SLAM;
- ArUco localization;
- GPS/RTK;
- внешняя система позиционирования.

## 15.9 Важное замечание про ArUco

Текущие пять файлов не распознают ArUco-изображения. Marker ID используется как логический номер узла графа. Локализация выполняется по odometry.

## 15.10 Несколько препятствий

Сейчас:

```python
blocked = {blocked_marker}
```

Для накопления:

```python
blocked.add(blocked_marker)
```

Но детектор также нужно расширить, чтобы возвращать несколько occupied markers.

## 15.11 Препятствия во время движения

Текущая архитектура обнаруживает obstacle только после прибытия в target. Для online replanning понадобится постоянная проверка лидара и более событийная архитектура/state machine.


---

# 16. Перенос на клубничную ферму

Удобно разделить систему на уровни.

## Глобальный граф

Узлы:

```text
начало ряда
рабочая точка возле растения
переход между рядами
зона выгрузки
зарядная станция
```

## Локальная навигация

Реальному роботу дополнительно понадобятся:

- учёт ширины платформы;
- безопасные расстояния;
- локальные obstacles;
- люди;
- другие роботы;
- растения;
- плавные траектории.

## Манипулятор

После прибытия:

```text
камера
-> поиск спелой ягоды
-> 3D координаты
-> точка захвата
-> манипулятор
-> сбор
```

Текущий модуль является навигационно-транспортным слоем этой более крупной системы.


---

# 17. Ограничения текущей версии

### Планировщик

Хранит целые маршруты и все кратчайшие варианты. Для 25 узлов это нормально, для больших графов лучше A*, Dijkstra или BFS с parent.

### Motion

Не учитывает:

- ускорение;
- тормозной путь;
- динамические obstacles;
- ширину робота;
- кинематические ограничения;
- costmap;
- плавность траектории.

### Detector

Предполагает:

- робот неподвижен;
- baseline корректный;
- появляется один новый объект;
- остальная сцена не меняется;
- frame лидара согласован с robot frame;
- obstacle находится достаточно близко к одному из marker centers.

### `input()`

Во время ожидания ввода ROS не spin'ится. Для текущего сценария это допустимо, но для реального постоянно работающего робота лучше перейти к асинхронной архитектуре.


---

# 18. Типичные ошибки

## `Package not found`

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## `No executable found`

Проверить:

```bash
ros2 pkg executables module_b_navigation
```

и packaging.

## Нет odometry

```bash
ros2 topic echo /RMC2/odometry --once
```

## Нет scan

```bash
ros2 topic echo /RMC2/scan --once
```

## `BASELINE_SCAN_NOT_AVAILABLE`

Проверить `/RMC2/scan` и QoS.

## `incompatible QoS RELIABILITY`

Оставить `qos_profile_sensor_data` и проверить QoS publisher.

## Робот едет в неправильном направлении

Проверить соответствие:

```text
world X
world Y
yaw
Graph.position()
```

Мы уже сталкивались с ошибочной геометрией, когда логическое RIGHT превращалось в физическое движение вверх. Текущая рабочая схема — `x = col`, `y = row`.

## Неверный obstacle marker

Проверить:

- frame_id scan;
- ориентацию лидара;
- TF;
- offset сенсора;
- yaw;
- CELL_SIZE;
- `max_marker_distance`.


---

# 19. Рекомендуемые тесты

1. `0 -> 1`: проверка +X.
2. `0 -> 5`: проверка +Y.
3. `0 -> 22`: длинный маршрут.
4. obstacle на `12`: проверка replanning.
5. obstacle вне старого маршрута.
6. изменение `min_votes`.
7. изменение `min_change`.
8. остановка Ctrl+C.
9. отсутствие scan.
10. отсутствие odometry.

Для каждого теста полезно фиксировать:

```text
route
marker_reached
robot pose
obstacle_votes
blocked marker
return route
```


---

# 20. Синтаксис Python — шпаргалка

```python
import math
```
Импорт модуля.

```python
from collections import deque
```
Импорт конкретного объекта.

```python
class Graph:
```
Объявление класса.

```python
def position(self, marker):
```
Метод.

```python
self.x
```
Поле конкретного объекта.

```python
None
```
Отсутствие значения.

```python
if x is None:
```
Проверка на отсутствие.

```python
route = [0, 1, 2]
```
Список.

```python
blocked = {12}
```
Множество.

```python
votes = {12: 30}
```
Словарь.

```python
x, y = graph.position(12)
```
Распаковка.

```python
for i in range(10):
```
10 итераций, `i=0..9`.

```python
if marker in blocked:
```
Проверка наличия.

```python
f"marker={marker}"
```
f-string.

```python
continue
```
Перейти к следующей итерации цикла.

```python
break
```
Выйти из цикла.

```python
return
```
Закончить функцию.

```python
try / except / finally
```
Обработка ошибок и гарантированное завершение.


---

# 21. ROS 2 — шпаргалка

```python
rclpy.init()
```
Инициализация ROS.

```python
Node("name")
```
Создание node.

```python
create_publisher(...)
```
Отправка сообщений.

```python
create_subscription(...)
```
Получение сообщений.

```python
create_timer(...)
```
Периодический callback.

```python
rclpy.spin_once(...)
```
Однократная обработка событий.

```python
node.get_logger().info(...)
```
Лог.

```python
node.destroy_node()
rclpy.shutdown()
```
Корректное завершение.


---

# 22. Что можно менять безопаснее, а что осторожно

Относительно безопасные настройки:

```text
linear_speed
angular_speed
distance_tolerance
angle_tolerance
min_change
min_votes
max_marker_distance
start_id
target_id
```

Критические места:

```text
Graph.position()
quaternion_to_yaw()
angle_to_pi()
local -> world transform
qos_profile_sensor_data
self.index = 1
planner.plan(target, start, blocked)
```

Изменение критических мест требует обязательного повторного теста геометрии.


---

# 23. Рекомендуемый порядок адаптации к новому роботу

```text
1. Проверить /odometry
2. Проверить X/Y
3. Проверить yaw
4. Проверить /cmd_vel
5. Тест на один соседний marker
6. Тест RIGHT
7. Тест UP
8. Длинный маршрут
9. Проверить /scan
10. Проверить QoS
11. Проверить local lidar XY
12. Проверить world lidar XY
13. Проверить nearest_marker
14. Только потом включить replanning
```

Так проще найти, на каком именно уровне появилась ошибка.


---

# 24. Ментальная модель проекта

Самое короткое объяснение:

> `Graph` знает, **где находятся клетки и куда можно перейти**.  
> `Planner` решает, **по каким клеткам идти**.  
> `Motion` решает, **как физически доехать до следующей клетки**.  
> `ObstacleDetector` решает, **какую клетку нужно запретить**.  
> `main.py` решает, **когда вызвать каждую из этих частей**.

Алгоритмически:

```text
карта
  ↓
маршрут
  ↓
движение
  ↓
изменение среды
  ↓
измерение
  ↓
обновление карты
  ↓
новый маршрут
```


---

# 25. Полные исходные коды текущей версии


---

## `graph.py`

```python
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
```


---

## `planner.py`

```python
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
```


---

## `motion_controller.py`

```python
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
```


---

## `obstacle_detector.py`

```python
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
```


---

## `main.py`

```python
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
```


---

# 26. Финальная памятка перед запуском

```text
[ ] Webots запущен
[ ] ROS 2 Jazzy sourced
[ ] workspace собран
[ ] workspace sourced
[ ] /RMC2/odometry существует
[ ] /RMC2/scan существует
[ ] /RMC2/cmd_vel подключён
[ ] package виден ROS
[ ] executable module_b виден ROS
```

Команда:

```bash
ros2 run module_b_navigation module_b     --ros-args     -p start_id:=0     -p target_id:=22
```

После прибытия:

```text
1. дождаться полной остановки
2. поставить ровно одно препятствие
3. ввести y
4. проверить obstacle_votes
5. проверить выбранный marker
6. проверить return route
7. наблюдать возврат
```

---

# 27. Итог

Текущая версия намеренно не является «идеальной промышленной ROS-архитектурой». Её задача — максимально прозрачно показать весь цикл автономного поведения:

```text
ROS 2
+ граф
+ BFS
+ odometry
+ Twist
+ LaserScan
+ преобразование координат
+ голосование
+ blocked nodes
+ replanning
```

Это хорошая учебная база: каждый модуль можно понимать отдельно, а затем постепенно заменять более сложной реализацией без изменения общей идеи системы.

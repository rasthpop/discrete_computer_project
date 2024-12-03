# Звіт про виконання проекту: Найкоротший шлях мiж мiстами 
## Вступ
Під час виконання цього комп’ютерного проєкту наша команда розробила програмне забезпечення для пошуку найкоротшого шляху між точками на карті. Для цього ми провели дослідження щодо різних алгоритмів, які ми могли б реалізувати, і вирішили залишити ваш вибір на алгоритмах Дейкстри та A*. На початку роботи ми планували реалізувати тільки алгоритм Дейкстри, але в процесі розробки вирішили додати також алгоритм А*, щоб покращити ефективність, протестувати, як ці алгоритми працюють у різних ситуаціях та проаналізувати відмінність у результатах виконання.

### Чому саме ці алгоритми?
Ми обрали алгоритми Дейкстри та A*, оскільки вони є ефективними методами для пошуку найкоротших шляхів у графах, зокрема для задач навігації на картах. Навіть такі застосунки, як Google Maps використовують саме ці два алгоритми. Алгоритм Дейкстри гарантує знаходження найкоротшого шляху, перевіряючи всі можливі варіанти, що робить його універсальним для будь-яких типів графів. Алгоритм A* доповнює Дейкстру, додаючи передбачення напрямку до фінішної точки, через що пошук відбувається значно швидше. Такий вибір алгоритмів дав нам можливість порівняти їх ефективність і зрозуміти, як різні підходи можуть впливати на ефективність, точність та час виконння реальних задач.

Проект складався з декількох частин, кожна з яких виконувалася різними учасниками команди.
## Розподіл роботи
#### Тарас Копач: 
* дизайн інтерфейсу
* візуалізація алгоритму
#### Андрій Говоров:
* алгоритм Дейкстри
#### Марина Огінська:
* алгоритм А*
#### Аліна Боднар:
* алгоритм А*
* написання фінального звіту

## Принципи дискретної математики
У цьому проекті ми працювали з графами для знаходження найкоротших шляхів між точками на карті. В дискретній математиці графи – це структури, які складаються з вершин та ребр. В нашому проекті вершини це точки на карті, наприклад, перехрестя або кінцеві точки доріг, а ребра це дороги між вершинами, тобто шляхи, якими можна дістатись з однієї вершини в іншу. 
Нашою задачею було знаходження найкоротшого шляху, тобто шляху між двома вершинами, який має найменшу сумарну вагу. Вага це значення, яке прив'язане до кожного ребра. Наприклад, вагою може бути довжина дороги.
Для створення графа, який відображає карту світу, ми використали бібліотеку OSMnx, за допомогою якої ми отримали дані з OpenStreetMap. OpenStreetMap  це база даних, що містить інформацію про географічні об'єкти всього світу. Це можуть бути дороги, вулиці, будівлі, річки. Бібліотека OSMnx дозволяє нам перетворити ці дані на граф, в якому вершинами будуть координати точок на карті, а ребрами – дороги, які їх з’єднують.

## Як працює алгоритм Дейкстри
Алгоритм Дейкстри — це спосіб знайти найкоротший шлях між двома точками на графі. Його основна ідея полягає в тому, щоб почати зі стартової точки і поступово перевіряти всі сусідні вершини, поки не дійдемо до кінцевої точки.
1.	Спочатку всім вершинам на графі присвоюється нескінченна відстань, окрім початкової вершини, відстань до якої дорівнює нулю.
2.	Далі використовується спеціальна структура, яка називається черга з пріоритетами. У цій черзі вершини розташовані так, що вершина із найменшою відстанню буде обррблятись першою.
3.	Алгоритм покроково вибирає вершину із найменшою відстанню та оновлює відстані до її сусідів, якщо знайдено коротший шлях.
4.	Процес триває, поки не буде досягнута кінцева точка. Тоді алгоритм видає найкоротший шлях та його довжину. 

## Наша реалізація алгоритму в Python
### Імпорти та бібліотеки:
* `math`: для того, щоб присвоїти вершинам нескінченне значення на початку алгоритму 
* `heapq`: реалізовує чергу з пріоритетом за допомогою купи.
* `networkx`: бібліотека для роботи з графами та створення графу вулиць.
* `osmnx`: бібліотека для отримання та обробки OpenStreetMap даних.
* `time`: бібліотека для вимірювання часу виконання функцій

Для початку, ми реалізували декоратор, який вимірює час виконання функції, який ми потім порівнювали з результатами алгоритму Дейкстри. Він зберігає час до виконання функції, потім викликає саму функцію алгоритму А*, і після її виконання вимірює, скільки часу пройшло, і виводить це на екран.

```python
def track_time(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        print(f"Execution time of {func.__name__}: {
              execution_time:.6f} seconds")
        return result

    return wrapper
```

#### На початку роботи алгоритму ми повинні підготувати дані:
* Всім вершинам графа ми присвоюємо нескінченну початкову відстань, за допомогою модуля math ```(math.inf)```, окрім стартової вершини, яка отримує відстань 0.
* Створюємо словник ```graph_for_path_restoration```, щоб зберігати попередню вершину для кожного вершини. Після завершення роботи функцій це буде потрібно для відновлення шляху.
* Створюємо множину ```visited_nodes```, щоб відстежувати, які вершини ми вже опрацювали.
* Використовуємо чергу з пріоритетами ```(PriorityQueue)```, щоб вибирати вершину із найменшою поточною відстанню.

``` python
node_distances, path_restore = {}, {}
for node in graph.nodes:
    node_distances[node] = math.inf
    path_restore[node] = None

node_distances[start] = 0
visited_nodes = set()

queue = PriorityQueue()
queue.put(start, 0)
```

 ### Клас PriorityQueue 
 В цьому класі ми реалізовуємо чергу з пріоритетами, використовуючи бібліотеку `Python heapq` для вибору вершин з найменшою відстанню. 
 * ```is_empty()``` — перевіряє, чи порожня черга.
 * ```put(item, priority)``` — додає елемент з певним пріоритетом у чергу.
 * ```get()``` — витягує елемент з найменшим пріоритетом.

```python
class PriorityQueue:
    def __init__(self):
        self._container = []

    def is_empty(self):
        return len(self._container) == 0

    def put(self, item, priority):
        heapq.heappush(self._container, (priority, item))

    def get(self):
        return heapq.heappop(self._container)[1]
```

### Основний цикл Дейкстри
Алгоритм працює в циклі, поки черга з пріоритетами не стане порожньою. На кожному кроці:
1.	Вибирається вершина із найменшою поточною відстанню ```(curr_node)```.
2.	Якщо вершина вже відвідана, ми її пропускаємо.
3.	Додаємо вершину до множини ```visited_nodes```.

```python
while not queue.is_empty():
    curr_node = queue.get()

    if curr_node in visited_nodes:
        continue
    if curr_node == end:
        break
    visited_nodes.add(curr_node)

    curr_distance = node_distances[curr_node]
```

Для кожної сусідньої вершини `adj_node` виконуємо перевірку:
* Визначаємо вагу ребра між поточною вершиною і її сусідом ```(edge_distance)```, використовуючи `length`.
* Далі змінній new_distance присвоюємо значення суми поточної відстані та ваги ребра.
* Якщо нова відстань менша за ту, яка була записана раніше, оновлюємо цю відстань у словнику ```node_distances``` та записуємо попередній вершину у ```path_restore```.

```python
for adj_node, attributes in graph[curr_node].items():
    edge_distance = attributes[0].get('length', 1)
    new_distance = curr_distance + edge_distance

    if new_distance < node_distances[adj_node]:
        node_distances[adj_node] = new_distance
        path_restore[adj_node] = curr_node
        queue.put(adj_node, new_distance)
```

Алгоритм завершує свою роботу, коли черга з пріоритетами порожня, тобто ми пройшлись по всіх вершинах графа або ми досягли кінцевої вершини end, і можемо завершити пошук.

```python
if curr_node == end:
    break
```

### Функція restoration для відновлення найкоротшого шляху
Після завершення основного циклу ми маємо словник ```path_restore```, тобто інформацію про попередні вершини для відновлення шляху. Ми також маємо довжину найкоротшого шляху до кінцевої вершини.
Використовуючи цей словник, функція будує сам шлях, починаючи з кінцевої вершини і так до першої. В результаті функція повертає список вершин, через які проходить найкоротший шлях. 

```python
def path_restoration(path_restore, curr_node):
    '''
    Reconstruct the path by following the nodes from goal to start.
    '''
    path = []
    while curr_node is not None:
        path.append(curr_node)
        curr_node = path_restore[curr_node]
    path.reverse()
    return path
```

## Як працює алгоритм А*?
Алгоритм А* працює дуже схоже на Дейкстру, але використовує додаткову оцінку, яка допомагає передбачити, наскільки близька вершина до кінцевої точки. Це допомагає алгоритму працювати швидше, оскільки він більше фокусується на тих вершинах, які ведуть до бажаної точки. Завдяки цьому А* часто шукає найкоротший шлях швидше, ніж Дейкстра.
Спочатку для кожної вершини на карті встановлюється велика відстань. Для стартової точки встановлюємо відстань 0, оскільки ми вже знаходимося в цьому місці. Потім стартова точка додається в чергу з пріоритетом, бо її відстань до себе дорівнює нулю. В черзі ми завжди вибираємо вершину, до якої можна дістатись найшвидше. Це ми рахуємо за допомогою суми двох показників: скільки вже пройдено і скільки ще залишилось пройти.
Для кожної вершини ми перевіряємо сусідні до неї, тобто ті вершини, до яких ми можемо дістатись на пряму. Якщо відстань до цього сусіда коротша, ніж була раніше, ми оновлюємо відстань і додаємо цю вершину у чергу. Ми також зберігаємо, через яку вершину було вигідніше дістатись до кожного з них. Це дозволяє нам відновити шлях пізніше.
Як тільки ми досягаємо кінцевої точки, ми можемо відновити найкоротший шлях, рухаючись назад через збережені попередні вершини.

## Наша реалізація алгоритму в Python
### Імпорти та бібліотеки:
Такі самі, як в алгоритмі Дейкстри. Далі так само, як в алгоритмі Дейкстри, ми створили клас ```PriorityQueue``` для реалізації черги з пріоритетами. Ця черга допомагає ефективно вибирати наступну вершину для обробки.

### Функція path_restoration:
Ця функція аналогічно до алгоритму Дейкстри відновлює шлях від кінцевої вершини до початкової після того, як завершується робота основного алгоритму. Вона рухається з кінцевої вершини назад через збережені попередні вузли, що було відновлено під час виконання алгоритму A*. Після цього шлях перевертається, щоб отримати від початку до кінця.

### Основна частина алгоритму A*
Початок роботи функції:
* ```node_distances``` це словник, який зберігає найкоротшу відому відстань до кожної вершини, яка спочатку дорівнює нескінченності.
* ```path_restore```: словник, що зберігає попередню вершину для кожної вершини, щоб потім відновити шлях.
* ```visited_nodes```: множина, за допомогою якої будуть відстежуватись вершини, які ми вже відвідали.
* ```queue```: черга з пріоритетами, де вершини обробляються у порядку зростання значення відстані.
* ```end_lat та end_lon```: координати кінцевої вершини до якої треба дістатись. Вони потрібні, щоб приблизно оцінити, наскільки далеко залишилося до цілі.

```python
node_distances = {node: math.inf for node in graph.nodes}
path_restore = {node: None for node in graph.nodes}
heuristic_distances = {}

node_distances[start] = 0
visited_nodes = set()

queue = PriorityQueue()
queue.put(start, 0)

end_lat, end_lon = graph.nodes[end]['y'], graph.nodes[end]['x']
```
 
### Основний цикл 
В основному алгоритмі всі вершини обробляються в циклі, поки ми не обробимо усі вершини та черга не стане порожньою, або поки не буде знайдено кінцеву вершину.
Спочатку  за допомогою ```queue.get()``` вибирається вершина за найменшою оцінкою маршруту через вершину. Це число показує, наскільки вдалим буде вибір конкретної вершини, враховуючи шлях, який ми вже пройшли та те, скільки приблизно залишилось до фінішної точки.
Далі якщо вузол вже був оброблений, алгоритм його ігнорує ```(if curr_node in visited_nodes)```.
Якщо поточний вузол – той, до якого ми хочемо дістатись, основний цикл завершується та ми переходимо до функції ```path_restoration```, щоб відновити пройдений шлях, і робота завершується.
Потім вершини, по яких ми пройшлись, додаються в список відвіданих.

```python
while not queue.is_empty():
    curr_node = queue.get()

    if curr_node in visited_nodes:
        continue

    if curr_node == end:
        path = path_restoration(path_restore, end)
        print(len(visited))
        return path, node_distances[end]

    visited_nodes.add(curr_node)
    curr_distance = node_distances[curr_node]
```

### Обробка сусідів
Для кожного сусіда ```(adj_node)``` поточної вершини ми обчислюємо  відстань до нього через цю поточну вершину. Відстань ребра ми визначаємо за допомогою ```length```, а за замовчуванням приймаємо 1.
Далі ми перевіряємо чи нова відстань менша за поточну відому, якщо це так, то ми оновлюємо це значення. 
Потім в словник path_restore ми зберігаємо інформацію, що поточний вузол є попереднім для його сусіда.

Щоб оцінити пряму відстань від координатами сусіда та фінішною вершиною, ми використовуємо функцію ```ox.distance.great_circle``` 
Далі ми розраховуємо `f(n) = g(n) + h(n)` ```for priority in queue```, де `g(n)` - нова відома відстань до вершни, а `h(n)` - наближена відстань від поточної вершини до фінішної. 
Тоді ця вершина додається в чергу з порахованим пріоритетом `f(n)`.

```python
for adj_node, attributes in graph[curr_node].items():
    visited.append(curr_node)
    edge_distance = attributes[0].get('length', 1)
    new_distance = curr_distance + edge_distance

    if new_distance < node_distances[adj_node]:
        node_distances[adj_node] = new_distance
        path_restore[adj_node] = curr_node

        lat, lon = graph.nodes[adj_node]['y'], graph.nodes[adj_node]['x']
        heuristic_distance = pow(pow(lat - end_lat, 2) + pow(lon - end_lon, 2), 0.5)

        # f(n) = g(n) + h(n) for priority in queue
        total_cost = new_distance + heuristic_distance
        queue.put(adj_node, total_cost)
```

### Завершення роботи
Якщо черга порожня і фінішна вершина не була знайдена, алгоритм повертає None та нескінченну відстань:

```python
return None, math.inf
```

## Різниця між алгоритмами
Дейкстра завжди розглядає вершини на основі того, наскільки короткий шлях до них уже знайдено, і поступово перевіряє всі можливі шляхи. Він не знає, наскільки далеко знаходиться ціль, тому обходить багато вершин, навіть якщо вони не ведуть в сторону цілі.
А* працює ефективніше, бо використовує не тільки вже знайдену довжину шляху (як Дейкстра), а й додаткову оцінку, наскільки далеко залишилося до цілі (це називається евристикою). Якщо ця оцінка правильна, алгоритм може уникнути перевірки зайвих вершин і швидше знайти шлях.

## Порівняння результатів виконання алгоритмів
Для порівняння алгоритмів Дейкстри та A*, ми вибрали два міста: `Київ` та `Житомир` і знайшли найкоротший шлях між ними, використовуючи кожен з алгоритмів. Результати, які ми отримали під час виконання, дозволили нам порівняти ефективність обох алгоритмів та проаналізувати їхні переваги та недоліки.
### Дейкстра:
Кількість відвіданих вузлів: `261279` <br />
Час виконання алгоритму Дейкстри: `0.356368 секунд` <br />
Загальний час виконання програми: `54.187821 секунд`

### A:*
Кількість відвіданих вузлів: `122364` <br />
Час виконання алгоритму A*: `0.212818 секунд` <br />
Загальний час виконання програми: `55.140640 секунд`

### Порівняння алгоритмів:
#### Час виконання:
Алгоритм A* показав швидший час виконання безпосередньо для пошуку шляху — `0.212818` секунд, що є швидше, ніж у Дейкстри `(0.356368 секунд)`.
Однак загальний час виконання програми в A* `(55.140640 секунд)` більший за загальний час виконання програми в Дейкстри `(54.187821 секунд)`. Це може бути повʼязано нестабільним підключенням до інтернету та часом виконання головної функції, яка будує однакові графи при виконанні Дейкстри та A*.

#### Кількість відвіданих вершин:
Алгоритм A* відвідав значно менше вузлів — `122364` порівняно з `261279` вузлами у Дейкстри. Це свідчить про те, що A* більш ефективно вибирає вершини для обробки, спрямовуючи пошук до цілі і тим самим зменшуючи кількість непотрібних перевірок.
Переваги та недоліки:

Проаналізувавши отримані дані після запуску обох функцій, ми виділили такі основні переваги та недоліки обох алгоритмів:
### Алгоритм Дейкстри:
#### Переваги:
Проста реалізація та стабільні результати на всіх типах графів.
Не потребує додаткових даних, таких як оцінка відстані до кінцевої точки.

#### Недоліки:
Може бути менш ефективним на великих графах, оскільки розглядає всі можливі шляхи, навіть якщо вони ведуть далеко від цілі.
Потребує більше часу на пошук шляху, особливо в складних мережах.

### Алгоритм A*:
#### Переваги:
Працює швидше за Дейкстру, оскільки активно спрямовує пошук до цілі, відвідуючи менше вершин.
Зазвичай має кращу ефективність на великих графах, особливо якщо правильна оцінка відстані до цілі.
#### Недоліки:
Потрібно мати хорошу оцінку відстані до кінця шляху (що може бути складно реалізувати в деяких випадках).
У деяких випадках може виявитися повільнішим за Дейкстру через додаткові обчислення для оцінки вартості.


## Інтерфейс (Tkinter) та NetworkX
У цьому проекті ми використали бібліотеку ```Tkinter``` для створення графічного інтерфейсу користувача (GUI) та бібліотеку ```NetworkX``` для роботи з графами і знаходження найкоротших шляхів.

### Імпорти
* ```tkinter``` - дозволяє використовувати всі елементи графічного інтерфейсу з бібліотеки Tkinter (створення вікон, кнопок, полів вводу, текстів)
* ```from osmnx._errors import InsufficientResponseError``` - дозволяє програмі розуміти, коли даних, які ввів користувач з карти недостатньо, щоб побудувати маршрут або граф. Ось повідомлення, яке в такому випадку отримає користувач: 'Querry Error', 'Unable to find the start or destination point. Please try again'
* ```from requests.exceptions import ConnectionError``` Імпортується помилка, яка виникає, коли є проблеми з підключенням до Інтернету або сервера. Ось повідомлення, яке в такому випадку отримає користувач: 'Connection Error', 'Please check your internet connection and try again'
* ```matplotlib.figure``` - для візуалізації графа та шляху, який ми знайшли через алгоритми

### Створення основного вікна та інтерфейсу
* ```root = Tk()``` - створюємо головне вікно програми
* ```side_nav``` - це рамка для навігації, де знаходяться поля вводу, кнопки та інші елементи.
* ```vis_frame``` — рамка для відображення графа на карті.
* Дизайн інтерфейсу задається через ```root.call("source", "Azure/azure.tcl")``` і ```root.call("set_theme", "dark")```

### Візуалізація графа
* ```fi = Figure((6,6), dpi=100)```: створюється об'єкт для графіка розміром 6x6 із щільністю(кількістю точок або пікселів) 100 точок на дюйм.
* ```ax = fi.add_subplot()``` — додається місце на графіку, де будем малюватиcь граф.
* ```ax.axis('off')``` — вимикає осі, щоб на графіку не було видно координат. Тобто, замість того, щоб на графіку були показані осі з числами або лініями, вони будуть приховані, щоб не відволікати увагу від самого графіка.
* ```canvas = FigureCanvasTkAgg(fi, vis_frame)```: дозволяє відображати графік у вікні Tkinter.
* ```ox.plot_graph_route```: накладає маршрут на вже зображений граф.
* ```route_color='#007fff'``` — це параметр, який встановлює синій колір маршруту.
Маршрут накладається поверх графа, показуючи, яким шляхом йти від стартової точки до кінцевої.

### Взаємодія з графом
* ```canvas.get_tk_widget().pack()```: вставляє графік у вікно програми.
* ```NavigationToolbar2Tk(canvas)``` — додає панель інструментів, щоб користувач міг взаємодіяти з графіком. Ця панель дозволяє користувачу збільшувати, зменшувати, переміщати та зберігати графік.
* ```e1 = ttk.Entry(entry_frame, font=LG)``` - поля вводу, які дозволяють користувачу вводити текст. В нашому випадку це початкова та кінцева точки для пошуку шляху.
* ```h1 = Label(root, text='Find Shortest Path.', font=('Courier', 13, 'bold'))``` - виведення тексту на екран.
* ```op1 = ttk.OptionMenu(entry_frame, value_string, 'Select an algorithm', 'Dijkstra', 'A*')``` - дозволяє користувачу вибрати один з алгоритмів для пошуку шляху (Dijkstra або A*).
* ```action = ttk.Button(entry_frame, text="Find shortest distance:", command=handle_click)``` - кнопка, яка запускає функцію пошуку найкоротшого шляху. 

### Виведення інформації
* Час виконання та довжина маршруту відображаються в полях ```duration``` і ```distance```.
Значення оновлюються через ```config()```:
```duration.config(text=f"Time: {exec_time:.2f}s")```
```distance.config(text=f"Time: {(shor_distance / 1000):.2f}km")```

### Фідбек та враження
Матеріал з дискретної математики по графах, необхідний для реалізації цього проекту, ми частково опрацьовували самостійно. Це виявилося непростим завданням, але водночас цікавим, оскільки це дозволило нам краще зрозуміти та засвоїти основні поняття та алгоритми, і ми одазу змогли протестувати їх на практиці.
Нас справді зацікавила тема, яку ми обрали, адже ми можемо побачити її вікористання в повсякденному житті, коли користуємось Google Maps або іншими застосунками з картами. Це зробило проект дійсно цікавим і корисним досвідом. Для нас було важливим те, що ми могли звернутися до викладачів і асистентів із запитаннями в будь-який момент, що значно полегшило нашу роботу над цим проектом. Особливо корисними були поради нашого ментора Олега Басистого, які допомогли вирішити технічні питання та допомогли краще зрозуміти складні деталі проекту.

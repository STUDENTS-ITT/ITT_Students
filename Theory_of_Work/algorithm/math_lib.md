# `src/core/math_lib/` — линейная алгебра, статистика, преобразования

Математический инструментарий фильтра и счисления. Всё — заголовочные
`inline`-функции; отдельной библиотеки нет. Матрицы хранятся построчно
(row-major, см. `utils.md`), число столбцов передаётся явно.

---

## `math_lib/constants.hpp`

Прокси-заголовок для обратной совместимости: просто подключает
`../utils/constants.h`. Единый источник констант — `utils/constants.h`.

---

## `math_lib/interpolation.h`

Численное интегрирование методом трапеций (используется в `ins_filter.h` и
`position_calc.h` для перехода «ускорение → скорость» и «скорость → координаты»).

### Функции

- `inline double v_integral(double v_now_dot, double V0, double v_pred_dot, double dt)` —
  одномерная трапеция: `V0 + (v_now_dot + v_pred_dot)/2 · dt`.
- `inline Vector v_integral(const Vector &v_now_dot, const Vector &V0, const Vector &v_pred_dot, double dt)` —
  поэлементно то же для векторов.

---

## `math_lib/matrix_ops.h`

Минимальная библиотека линейной алгебры для фильтра Калмана.

### Доступ и размеры

- `inline double &at(Matrix &A, int i, int j, int cols)` и
  `inline const double &at(const Matrix &A, int i, int j, int cols)` — доступ к
  элементу `(i, j)` при известном числе столбцов. Формула `A[i*cols + j]`.
- `inline int getRows(const Matrix &A, int cols)` — число строк `A.size()/cols`.

### Арифметика матриц

- `inline Vector multiply_m(const Matrix &A, const Vector &v, int cols_A)` —
  `A (n×m) · v (m)` → `n`.
- `inline Matrix multiply_matrix(const Matrix &A, const Matrix &B, int cols_A, int cols_B)` —
  `A (n×p) · B (p×m)` → `n×m`.
- `inline Matrix transpose_m(const Matrix &A, int cols)` — `A (n×m)` → `Aᵀ (m×n)`.
- `inline Matrix matrix_sum(const Matrix &A, const Matrix &B, int cols)` — поэлементно `A + B`.
- `inline Matrix matrix_diff(const Matrix &A, const Matrix &B, int cols)` — поэлементно `A − B`.

### Обратная матрица и заготовки

- `inline Matrix return_matrix(const Matrix &A, int cols)` — обратная к
  квадратной `A (n×n)` методом Гаусса–Жордана с выбором ведущего элемента.
  Если ведущий элемент по модулю `< 1e-15` (вырождение), возвращает **единичную**
  матрицу (защита от деления на ноль; в согласованном фильтре не срабатывает).
- `inline Matrix E_matrix(std::size_t n)` — единичная `n×n`.
- `inline Matrix H_matrix(std::size_t m, std::size_t n)` — матрица наблюдения
  `m×n` с единицами на главной диагонали (измеряем первые `m` компонент
  состояния).

### Векторы

- `inline Vector vector_product(const Vector &a, const Vector &b)` — векторное
  произведение `a × b`.
- `inline Vector vector_diff(const Vector &a, const Vector &b)` — `a − b`; при
  несовпадении размеров печатает `err size vec` в `stderr` и возвращает `{}`.
- `inline Vector vector_sum(const Vector &a, const Vector &b)` — `a + b`; так же
  проверяет размеры.

---

## `math_lib/statistics.h`

Статистика для выставки.

### Функции

- `inline double Average(const Vector &angle)` — среднее арифметическое с
  переводом **рад → град** (для логов). Пустой вектор → 0.
- `inline double Mean(const Vector &values)` — среднее арифметическое без
  перевода единиц.
- `inline double circularMean(const Vector &angles)` — средний угол по единичной
  окружности: `atan2(mean(sin), mean(cos))`. Корректен на стыке `±π`, в отличие
  от простого среднего.
- `inline Vector movingAverage(const Vector &data, int window)` — скользящее
  среднее с окном `window` (окно приводится к нечётному, ограничивается длиной
  данных). Реализовано через префиксные суммы за `O(n)`. Применяется в
  `aligner.hpp` для сглаживания ИМУ при выставке.

---

## `math_lib/transformations.h`

Переходы между СК тела и навигационной СК, приведение углов, перевод единиц.

### Функции

- `inline double normalize_angle(double a)` — приведение угла в `[-π, π]`
  (`fmod` + сдвиги). Используется повсеместно.
- `inline double rad_to_deg(double a)` / `inline double deg_to_rad(double a)` —
  перевод единиц через `RAD_TO_DEG`/`DEG_TO_RAD`.
- `inline Matrix bodyToNavMatrix(double heading, double pitch, double roll)` —
  матрица перехода `C_б^н` (3×3), последовательность Z-Y-X: `heading (ψ)` вокруг
  Z, `pitch (θ)` вокруг Y, `roll (φ)` вокруг X.
  Навигационная СК — (Север, Вверх, Восток); курс отсчитывается от Севера по
  часовой стрелке, поэтому при `ψ = 90°` продольная ось тела смотрит на восток.
- `inline Vector bodyToNav(const Matrix &C, const Vector &v_body)` — `v_nav = C · v_body`.
- `inline Vector navToBody(const Matrix &C, const Vector &v_nav)` — `v_body = Cᵀ · v_nav`
  (обратное преобразование).

### Элементы матрицы `C_б^н`

Для контроля при чтении кода (`c_vals[r][c]`):

```
C[0] = { cosθ·cosψ,  −cosφ·cosψ·sinθ − sinφ·sinψ,   sinφ·cosψ·sinθ − cosφ·sinψ }
C[1] = { sinθ,        cosφ·cosθ,                    −sinφ·cosθ }
C[2] = { cosθ·sinψ,  −cosφ·sinψ·sinθ + sinφ·cosψ,   sinφ·sinψ·sinθ + cosφ·cosψ }
```

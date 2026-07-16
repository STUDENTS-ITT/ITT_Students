# Руководство по интеграции и тестированию

## 1. API интерфейс

### 1.1. Класс `SINS_Algorithm`

Основной интерфейс для интеграции с внешними системами.

```cpp
class SINS_Algorithm {
public:
    // Инициализация (вызывается один раз)
    void initializeAlignment(
        double T_sys, double T_reg, double T_rem, double T_nav,
        double nav_start_T,
        const Vec3& accel_meas, const Vec3& gyro_meas,
        double init_lat, double init_lon, double init_alt
    );

    // Основной цикл (вызывается для каждого сэмпла)
    void updateNavigation(
        double current_T_sys,
        const Vec3& accel_meas,
        const Vec3& gyro_meas
    );

    // Получение текущего состояния
    NavState getCurrentState() const;
};
```

### 1.2. Потоковый ввод данных

```cpp
// Открытие и чтение IMU
IMU_Stream stream;
stream.open("path/to/IMU.txt");
IMU_Record rec;
while (stream.readNext(rec)) {
    // rec.Time, rec.Ax, rec.Ay, rec.Az, rec.Wx, rec.Wy, rec.Wz
}

// Чтение Nav.dat
Nav_Record nav;
loadNav("path/to/Nav.dat", nav);
```

### 1.3. Интерфейс данных

Все данные передаются через структуры `Vec3` и простые типы:

```cpp
Vec3 accel = {rec.Ax, rec.Ay, rec.Az};          // м/с²
Vec3 gyro = {deg2rad(rec.Wx), deg2rad(rec.Wy), deg2rad(rec.Wz)};  // рад/с
```

Это обеспечивает совместимость с любой библиотекой векторной алгебры.

---

## 2. Тестовые сценарии

### Сценарий А: Статика (неподвижная платформа)

**Цель:** Проверка, что координаты не дрейфуют при нулевых скоростях.

**Условия:**
- IMU-данные с неподвижной платформы
- Все акселерометры: `Ay ≈ 9.81 м/с²` (сила тяжести), `Ax ≈ Az ≈ 0`
- Все гироскопы: `Wx ≈ Wy ≈ Wz ≈ 0`

**Ожидаемый результат:**
- Pitch ≈ `asin(Ax/g)` ≈ 0°
- Roll ≈ `atan2(-Az, Ay)` ≈ 0°
- V_north ≈ 0, V_east ≈ 0
- Lat, Lon не изменяются

**Проверка в коде:**
```
T=332.053s | Lat=54.9522° Lon=38.5388° | ... | Vn=0.002 m/s Ve=0.001 m/s
```
Скорости малы (< 0.01 м/с) — допустимый шум.

---

### Сценарий Б: Движение по прямой

**Цель:** Проверка корректности интегрирования координат.

**Условия:**
- Постоянная скорость ~10 м/с на Север
- Gyro ≈ 0 (без вращения)

**Ожидаемый результат:**
- Lat линейно растёт
- Lon практически не меняется
- V_north ≈ 10 м/с (постоянная)
- Yaw ≈ 0°

---

### Сценарий В: Поворот

**Цель:** Проверка изменения рыскания.

**Условия:**
- Поворот на 90° (из Север в Восток)
- `Wz > 0` во время поворота

**Ожидаемый результат:**
- Yaw плавно изменяется от 0° до 90°
- После поворота V_east > 0, V_north ≈ 0

---

### Сценарий Г: Вертикальный канал

**Цель:** Проверка блокировки высоты.

**Условия:**
- Любые данные

**Ожидаемый результат:**
- `pos.z` (высота) всегда ≈ 0 (или init_alt при первом шаге, затем 0)
- `vel.y` (вертикальная скорость) = 0

---

## 3. Требования к производительности

### 3.1. Частота обработки

| Параметр | Значение |
|----------|----------|
| Частота IMU | 400 Гц (шаг 0.0025 с) |
| Требование к `updateNavigation` | < 2500 мкс (< шага дискретизации) |
| Типичное время (C++17, -O2) | ~50–100 мкс |

### 3.2. Потребление памяти

| Компонент | Память |
|-----------|--------|
| `SINS_Algorithm` (один экземпляр) | ~200 байт |
| Буфер выставки (125K записей × 56 байт) | ~7 МБ |
| Строковый буфер `IMU_Stream` | ~100 байт |
| **Итого** | **~7 МБ** |

### 3.3. Оптимизации

- `inline`-функции для всех вычислений (без накладных расходов вызова)
- Потоковое чтение IMU (без загрузки всего файла в память)
- Минимум аллокаций памяти в навигационном цикле

---

## 4. Пример интеграции

### Сценарий: замена файлового ввода на CAN-шиину

```cpp
SINS_Algorithm sins;
bool aligned = false;

void onIMUData(double time, double ax, double ay, double az,
               double wx, double wy, double wz) {
    Vec3 accel = {ax, ay, az};
    Vec3 gyro = {deg2rad(wx), deg2rad(wy), deg2rad(wz)};

    if (!aligned) {
        // Накопление данных для выставки
        accumulationBuffer.push_back({time, ax, ay, az, wx, wy, wz});
        if (time >= ALIGNMENT_END_T) {
            Vec3 a_mean = computeMean(accumulationBuffer, ACCEL);
            Vec3 g_mean = computeMean(accumulationBuffer, GYRO);
            sins.initializeAlignment(..., a_mean, g_mean, ...);
            aligned = true;
        }
    } else {
        sins.updateNavigation(time, accel, gyro);
        NavState state = sins.getCurrentState();
        // Отправка в CAN-шину или GUI
    }
}
```

### Сценарий: запись результатов в CSV

```cpp
std::ofstream csv("output.csv");
csv << "T,Lat,Lon,Vn,Ve,Yaw,Pitch,Roll\n";

while (imu_stream.readNext(rec)) {
    sins.updateNavigation(rec.Time, accel, gyro);
    if (counter++ % 100 == 0) {
        NavState s = sins.getCurrentState();
        csv << rec.Time << ","
            << s.pos.x * 180/PI << ","
            << s.pos.y * 180/PI << ","
            << s.vel.x << ","
            << s.vel.z << ","
            << s.euler.x * 180/PI << ","
            << s.euler.y * 180/PI << ","
            << s.euler.z * 180/PI << "\n";
    }
}
```

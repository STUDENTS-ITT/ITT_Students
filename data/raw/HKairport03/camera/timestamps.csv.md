# camera/timestamps.csv — индекс кадров камеры

## Источник
Топик `/left_camera/image/compressed`, тип `sensor_msgs/CompressedImage`, 10 Гц. Камера Hikvision CA-050-11UC, объектив 5 мм, hardware sync с LiDAR (PPS 10 Гц).

## Ориентация осей
Камера Hikvision CA-050-11UC, глобальный затвор, 10 Гц, смотрит **вниз** (рядом с Livox).

Оси изображения (стандарт ROS optical):

- **u** (столбец пикселя) — вправо по кадру;
- **v** (строка) — вниз по кадру;
- **Z_opt** — из объектива вперёд, в MARS-LVIG **к земле**.

Разрешение в датасете: **2448 × 2048** пикселей (RGB). Файлы `.jpg` — исходные байты `sensor_msgs/CompressedImage` без перекодирования.

## Параметры
| Параметр | Описание | Размерность |
|---|---|---|
| `bag_time_ns` | Время записи сообщения в ROS bag (Unix epoch) | нс |
| `bag_time_utc` | То же время в UTC ISO-8601 (вычислено из bag_time_ns, измерения не меняет) | дата-время UTC |
| `header.seq` | Порядковый номер сообщения ROS (std_msgs/Header.seq) | безразмерный |
| `header.stamp.sec` | Секунды метки времени датчика (UTC GPS после синхронизации PPS) | с |
| `header.stamp.nanosec` | Наносекунды метки времени датчика | нс |
| `header.frame_id` | Имя системы координат ROS, к которой отнесены данные | строка |
| `header_seq` | То же, что header.seq (для файлов камеры/LiDAR) | безразмерный |
| `stamp_sec` | То же, что header.stamp.sec | с |
| `stamp_nsec` | То же, что header.stamp.nanosec | нс |
| `stamp_ns` | Полная метка датчика: sec·10⁹ + nsec | нс |
| `stamp.sec` | Секунды штампа (сообщения без Header) | с |
| `stamp.nanosec` | Наносекунды штампа | нс |
| `frame_id` | Имя системы координат | строка |
| `topic` | Исходный ROS-топик в bag | строка |
| `index` | Порядковый номер кадра при извлечении (0, 1, 2, …) | безразмерный |
| `filename` | Имя файла кадра в этой папке | строка |
| `format` | Поле CompressedImage.format, обычно `rgb8; jpeg compressed bgr8` | строка |
| `size_bytes` | Размер JPEG в байтах (длина исходного поля data) | байт |

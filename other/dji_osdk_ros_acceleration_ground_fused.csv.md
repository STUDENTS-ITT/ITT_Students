# other/dji_osdk_ros_acceleration_ground_fused.csv — ускорение, fused, земля

## Источник
Топик `/dji_osdk_ros/acceleration_ground_fused`, тип `geometry_msgs/Vector3Stamped`, ~100 Гц.

## Ориентация осей
Вектор в навигационной (земной) СК DJI. На стоянке модуль близок к 0 — гравитация, как правило, исключена. Оси согласованы с fused-навигацией DJI (практически NED: X север, Y восток, Z вниз).

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
| `vector.x` | Ускорение, ось X земли | м/с² |
| `vector.y` | Ось Y земли | м/с² |
| `vector.z` | Ось Z земли | м/с² |

# other/dji_osdk_ros_gimbal_angle.csv — углы подвеса DJI L1

## Источник
Топик `/dji_osdk_ros/gimbal_angle`, тип `geometry_msgs/Vector3Stamped`, ~50 Гц. Подвес DJI Zenmuse L1.

## Ориентация осей
Углы подвеса. По фактическим значениям в bag это **градусы** (yaw порядка ±180, не радианы). Соглашение DJI Vector3: **X — roll, Y — pitch, Z — yaw** подвеса относительно корпуса/земли (как публикует OSDK).

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
| `vector.x` | Крен подвеса (roll) | градус |
| `vector.y` | Тангаж подвеса (pitch) | градус |
| `vector.z` | Рыскание подвеса (yaw) | градус |

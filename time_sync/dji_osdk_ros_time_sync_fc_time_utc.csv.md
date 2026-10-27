# time_sync/dji_osdk_ros_time_sync_fc_time_utc.csv — UTC полётного контроллера

## Источник
Топик `/dji_osdk_ros/time_sync_fc_time_utc`, тип `dji_osdk_ros/FCTimeInUTC`, 1 Гц.

## Ориентация осей
Время синхронизации PPS, не пространственные оси.

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
| `fc_timestamp_us` | Внутренний штамп полётного контроллера | мкс |
| `fc_utc_yymmdd` | Дата UTC: YYMMDD (например 221129 = 2022-11-29) | YYMMDD |
| `fc_utc_hhmmss` | Время UTC: HHMMSS как целое (61608 = 06:16:08) | HHMMSS |

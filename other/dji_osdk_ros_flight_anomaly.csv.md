# other/dji_osdk_ros_flight_anomaly.csv — флаги аномалий полёта

## Источник
Топик `/dji_osdk_ros/flight_anomaly`, тип `dji_osdk_ros/FlightAnomaly`.

## Ориентация осей
Битовая маска, не оси. Биты: IMPACT_IN_AIR=1, RANDOM_FLY=2, VERTICAL_CONTROL_FAIL=4, HORIZONTAL_CONTROL_FAIL=8, YAW_CONTROL_FAIL=16, AIRCRAFT_IS_FALLING=32, STRONG_WIND_LEVEL1=64, STRONG_WIND_LEVEL2=128, COMPASS_INSTALLATION_ERROR=256, IMU_INSTALLATION_ERROR=512, ESC_TEMPERATURE_HIGH=1024, ESC_DISCONNECTED=2048, GPS_YAW_ERROR=4096.

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
| `data` | Битовая маска аномалий (0 — нет) | битовая маска |

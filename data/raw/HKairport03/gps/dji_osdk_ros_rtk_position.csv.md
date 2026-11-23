# gps/dji_osdk_ros_rtk_position.csv — позиция RTK (ground truth локализации)

## Источник
Топик `/dji_osdk_ros/rtk_position`, тип `sensor_msgs/NavSatFix`, 5 Гц. RTK DJI M300: заявленная точность 1 см + 1 ppm·D по горизонтали, 1.5 см + 1 ppm·D по вертикали.

## Ориентация осей
Геодезическая СК **WGS84**:

- широта — градусы, север > 0;
- долгота — градусы, восток > 0;
- высота — метры над эллипсоидом WGS84 (как в `sensor_msgs/NavSatFix`).

Горизонтальная скорость RTK/GPS в топиках DJI — в **NED** (м/с), не в широте/долготе.

Это опорная траектория локализации датасета MARS-LVIG.

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
| `status.status` | Качество GNSS-фикса ROS NavSatStatus: −1 нет, 0 фикс, 1 SBAS, 2 GBAS | код |
| `status.service` | Битовая маска систем: 1 GPS, 2 GLONASS, 4 BeiDou, 8 Galileo (в bag часто 0) | битовая маска |
| `latitude` | Геодезическая широта WGS84, север положительный | градус |
| `longitude` | Геодезическая долгота WGS84, восток положительный | градус |
| `altitude` | Высота; в ROS NavSatFix — над эллипсоидом WGS84 | м |
| `position_covariance_0` | Элемент [0,0] ковариации позиции (ряд-major 3×3), обычно var(E) | м² |
| `position_covariance_1` | Элемент [0,1] ковариации позиции | м² |
| `position_covariance_2` | Элемент [0,2] ковариации позиции | м² |
| `position_covariance_3` | Элемент [1,0] ковариации позиции | м² |
| `position_covariance_4` | Элемент [1,1] ковариации позиции, обычно var(N) | м² |
| `position_covariance_5` | Элемент [1,2] ковариации позиции | м² |
| `position_covariance_6` | Элемент [2,0] ковариации позиции | м² |
| `position_covariance_7` | Элемент [2,1] ковариации позиции | м² |
| `position_covariance_8` | Элемент [2,2] ковариации позиции, обычно var(U) | м² |
| `position_covariance_type` | Тип ковариации: 0 неизвестна, 1 приближённая, 2 диагональ, 3 полная | код |

# Расчёт сигм для P0 в NoQT (математическое обоснование)

## 1. Общий принцип

В NoQT для первой секунды выставки используется `nav::AveragingWindow` из `src/navigation/averaging.*`. Алгоритм строит оценки неопределённости начальных условий, не считая классическое несмещённое СКО по всем отклонениям от выборочного среднего, а используя **максимальное отклонение от среднего по окну**.

Ключевой момент:

- **Номинал** координат/скорости/курса берётся из **последнего** отсчёта окна `t <= T_align` (`res.sns_last`). Именно эти значения подставляются в `NavState` (`lat/lon/alt/V/att.heading` по мере инициализации).
- **Постоянные/медленно меняющиеся** за время покоя параметры (наклон, смещения `ba`, `bg`) оцениваются по **среднему арифметическому** по окну (`f_mean`, `w_mean`, `ba_est`, `bg_est`).
- **Неопределённость** этих оценок принципиально разная: у "последнего отсчёта" она сопоставима с неопределённостью **одного измерения**, у "среднего по окну" она уменьшается как $1/\sqrt{N}$.

## 2. Оценка СКО через максимум отклонения

Пусть за время выставки объект находится в покое и измерения можно приближённо записать как

$$
x_i(t) \approx x_{true} + \nu_i(t), \quad \nu_i(t) \sim N(0,\sigma_i^2)\,(\text{приближённо, независимы по смыслу оценки})
$$

По окну из $N$ отсчётов вычисляется выборочное среднее

$$
\bar{x}_i = \frac{1}{N}\sum_{t=1}^{N} x_i(t)
$$

Далее находится максимум абсолютного отклонения от этого среднего

$$
\Delta_i = \max_{t=1,\ldots,N} \left| x_i(t) - \bar{x}_i \right|
$$

Для стандартизированной нормальной случайной величины $Z_j \sim N(0,1)$ вводится математическое ожидание экстремального модуля

$$
k(N) = \mathbb{E}\left[\max_{j=1,\ldots,N} |Z_j|\right]
$$

Отсюда оценка СКО **одного отсчёта** имеет вид

$$
\hat{\sigma}_{i,\text{sample}} \approx \frac{\Delta_i}{k(N)}
$$

Оценка СКО **среднего арифметического** $\bar{x}_i$

$$
\hat{\sigma}_{i,\text{mean}} = \frac{\hat{\sigma}_{i,\text{sample}}}{\sqrt{N}} = \frac{\Delta_i}{k(N)\sqrt{N}}
$$

### Откуда берётся `k(N)`

В `src/navigation/averaging.cpp::maxDeviationFactor(int n)` реализована следующая логика:

- Есть статические массивы `n_tab[]` и `k_tab[]` со значениями $k(N)=E[\max|N(0,1)|]$, полученными методом Монте-Карло (~4e5 выборок).
- Для значений $N$ между точками таблицы применяется линейная интерполяция.
- Если $N >= \max(n_tab)$, используется асимптотическая аппроксимация для экстремумов стандартного нормального

$$
k(N) \approx \sqrt{2\ln N}
$$

Почему используется именно $\Delta/k(N)$, а не классическое выборочное СКО $s = \sqrt{\frac{1}{N-1}\sum (x_i-\bar{x})^2}$? В условиях короткого окна выставки (обычно $N \sim 200$ по ИМУ, $N \sim 9-20$ по СНС) оценка по экстремальному отклонению даёт робастную прикидку масштаба разброса и меньше чувствительна к редким выбросам, при этом хорошо согласуется с методикой, принятой в данном проекте.

## 3. Расчёт в `AveragingWindow::finish`

### 3.1 ИМУ ($\approx200$ Гц)

Обрабатываются каналы $[a_x,a_y,a_z], [\omega_x,\omega_y,\omega_z]$.

```cpp
// 1) Средние по окну
for each ImuRow r: res.f_mean[k] += r.a[k]; res.w_mean[k] += r.w[k]
res.f_mean[k] *= 1.0/N; res.w_mean[k] *= 1.0/N

// 2) Максимум отклонения от среднего
for each ImuRow r:
  res.df_max[k] = max(res.df_max[k], |r.a[k]-res.f_mean[k]|)
  res.dw_max[k] = max(res.dw_max[k], |r.w[k]-res.w_mean[k]|)

// 3) Перевод в СКО
const double kf  = maxDeviationFactor(n_imu);
const double sqN = sqrt((double)n_imu);
res.sig_a_sample[k] = res.df_max[k] / kf;  // σ̂ одного отсчёта
res.sig_w_sample[k] = res.dw_max[k] / kf;
res.sig_a_mean[k]   = res.sig_a_sample[k] / sqN; // σ̂ среднего
res.sig_w_mean[k]   = res.sig_w_sample[k] / sqN;
```

Поля в `AveragingResult`: `f_mean/w_mean`, `df_max/dw_max`, `sig_a_sample/sig_w_sample`, `sig_a_mean/sig_w_mean`.

### 3.2 СНС ($\approx10$ Гц)

Обрабатываются $[lat,lon,alt,vn,vh,ve,heading]$. Для курса используется круговое среднее из-за периодичности угла.

```cpp
// 1) Средние (круговое для heading)
sum_lat/sum_lon/sum_alt/sum_vn/sum_vh/sum_ve; sum_sin += sin(hdg); sum_cos += cos(hdg)
lat_mean/lon_mean/alt_mean/vn_mean/vh_mean/ve_mean = sum/N
heading_circ_mean = atan2(sum_sin,sum_cos)

// 2) Максимум отклонения от среднего в метрических величинах
const double cos_lat = max(|cos(lat_mean)|, 1e-10);
for each SnsRow r:
  res.d_lat_max = max(res.d_lat_max, |r.s.lat-lat_mean|*R_EARTH)
  res.d_lon_max = max(res.d_lon_max, |r.s.lon-lon_mean|*R_EARTH*cos_lat)
  res.d_alt_max = max(res.d_alt_max, |r.s.alt-alt_mean|)
  dv = max3(|r.s.vn-vn_mean|, |r.s.vh-vh_mean|, |r.s.ve-ve_mean|)
  res.d_v_max   = max(res.d_v_max, dv)
  if(res.has_heading): res.d_heading_max = max(res.d_heading_max, |normalize_angle(r.s.heading-heading_circ_mean)|)

// 3) Перевод в СКО
const double kf = maxDeviationFactor(n_sns);
res.sig_pos_sample     = res.d_lat_max / kf;
res.sig_pos_lon_sample = res.d_lon_max / kf;
res.sig_alt_sample     = res.d_alt_max / kf;
res.sig_v_sample       = res.d_v_max   / kf;
res.sig_hdg_sample     = res.d_heading_max / kf;
```

Поля: `sns_last`, `lat_mean/lon_mean/alt_mean/vn_mean/vh_mean/ve_mean`, `heading_circ_mean/heading_last`, `d_*_max`, `sig_pos_sample/sig_pos_lon_sample/sig_alt_sample/sig_v_sample/sig_hdg_sample`.

Важно: для СНС деление на $\sqrt{N}$ **не** применяется. В `initialCovarianceFromAveraging` используются именно `*_sample`.

### 3.3 Оценка ориентации, ba, bg, гейт покоя

- **Углы уровнения**: по `f_mean` вычисляются `pitch = asin(f_mean_x/g)`, `roll = atan2(-f_mean_z, f_mean_y)`. Курс берётся либо из `angle.dat` (`has_heading_`), либо по гирокомпасу (`gyro_compass_heading`) из усреднённых угловых скоростей.
- **ba_est**: $ba_{est} = \bar{f}_{body} - C_b^n(\psi,\theta,\gamma)^T \begin{bmatrix}0&g&0\end{bmatrix}^T$, где $\bar{f}_{body} = f_{mean}$.
- **bg_est**: $bg_{est}$ оценивается по усреднённым угловым скоростям $\bar{\omega}_{body}$:
$$bg_{est} = \bar{\omega}_{body} - C_b^n(\psi,\theta,\gamma)^T \omega_{earth}(\varphi), \qquad \omega_{earth}(\varphi)=\begin{bmatrix}U_E\cos\varphi&U_E\sin\varphi&0\end{bmatrix}^T\,.$$
- **Гейт покоя**: `checkRest` анализирует модуль скорости по СНС, отклонение $\|\bar{\omega}_{body} - C_b^n \omega_{earth}\|$, отклонение $|\bar{f}_{body,y} - g|$. Пороги соотносятся с оценками СКО (`REST_SIGMA_FACTOR = 4.0`, `REST_OMEGA_BIAS_LIMIT`, `REST_FY_LIMIT_MG`). Результат `avg.rest_ok/rest_reason`.

## 4. Маппинг в `x0` и `P0`

### 4.1 Формирование `x0`

```cpp
// src/main.cpp
state.x = nav::initialErrorsFromAveraging(avg, avg.rest_ok);
```

`initialErrorsFromAveraging` строит вектор начальных ошибок ESKF из соображения

$$
x_0^{(i)} \approx (\text{номинал}_i - \bar{x}_i^{(window)})
$$

Для компонентов, у которых в качестве **номинала** принят **последний отсчёт окна** (`sns_last`), разность `(последний - среднее)` трактуется как характерная величина начальной неопределённости привязки/начальных ошибок. Для `bg_est` также учитывается, что он получен из среднего угловых скоростей. В комментариях `main.cpp` прямо указано: `x0` формируется как *"последний отсчёт − среднее"*.

### 4.2 Формирование `P0`

```cpp
// src/main.cpp
state.P = nav::initialCovarianceFromAveraging(ins::kalman_cfg, avg, state.lat, has_angle, sig_heading0);
```

Ниже приведено соответствие между компонентами 15-мерного вектора ошибок и теми сигмами, которые из `AveragingResult` используются.

| Индексы | Компонента | Источник в `AveragingResult` | Обоснование |
|---|---|---|---|
| `[0]` | Ошибка широты | `sig_pos_sample` | Номинал = `sns_last.lat`. Неопределённость привязки по СНС берётся как $\hat{\sigma}_{pos,\text{sample}}$. |
| `[1]` | Ошибка долготы | `sig_pos_lon_sample` | То же. Учитывается масштабирование по $R_E\cos\varphi$. |
| `[2]` | Ошибка высоты | `sig_alt_sample` | Номинал = `sns_last.alt`, используется $\hat{\sigma}_{alt,\text{sample}}$. |
| `[3..5]` | Ошибки скорости `[Vn,Vh,Ve]` | `sig_v_sample` | Номинал берётся из `sns_last` по скорости, поэтому в P0 кладётся $\hat{\sigma}_{v,\text{sample}}$. |
| `[6]` | Ошибка курса (heading) | `sig_hdg_sample` либо `cfg.sig_bg / \Omega_h$ либо `sig_heading0` из `angle.dat` | Если есть достоверный заголовок из `angle.dat` — приоритет отдаётся ему. Иначе используется оценка либо по разбросу курса по СНС, либо по гирошуму/проекции вращения Земли при гирокомпасе. |
| `[7]` | Ошибка тангажа (pitch) | Оценка по уровнянию | Определяется по `f_mean`. Точность уровнения связана с точностью усреднённой удельной силы. В начальной ковариации задаётся соответствующая величина с учётом точности среднего. |
| `[8]` | Ошибка крена (roll) | Оценка по уровнянию | Аналогично pitch, по `f_mean`. |
| `[9..11]` | Ошибки смещений акселерометров `ba_x,ba_y,ba_z` | Близко к `sig_a_mean` | `ba_est` считается по усреднённому вектору удельной силы $\bar{f}_{body}$. Неопределённость оценки по среднему уменьшается как $1/\sqrt{N}$, поэтому для компонент `ba` в начальной ковариации используется оценка, соответствующая $\hat{\sigma}_{a,\text{mean}}$. Дополнительно вводятся корреляции `P[7,9] = +\sigma_{ba}^2/g`, `P[9,7]=+\sigma_{ba}^2/g`, `P[8,11] = -\sigma_{ba}^2/g`, `P[11,8]=-\sigma_{ba}^2/g`. Они отражают зависимость начальной ошибки тангажа/крена от неопределённости оценки смещения акселерометров при выставке по уровнянию. |
| `[12..14]` | Ошибки смещений гироскопов `bg_x,bg_y,bg_z` | Близко к `sig_w_mean` | $bg_{est}$ вычисляется по **усреднённым** угловым скоростям $\bar{\omega}_{body}$: $bg_{est} = \bar{\omega}_{body} - C_b^n \omega_{earth}(\varphi)$. Неопределённость такой оценки уменьшается как $1/\sqrt{N}$, поэтому для компонент $bg$ в начальной ковариации используется $\hat{\sigma}_{w,\text{mean}}$. |

Кроме того, `initialCovarianceFromAveraging` применяет **floors** (минимально допустимые значения диагональных элементов). Это сделано для того, чтобы в редких случаях (очень малый разброс по окну, шумоподавленный участок) P0 не получилась излишне "уверенной" и не приводила к занижению коэффициента усиления Калмана в начале счисления.

## 5. Почему именно такое разделение

Кратко суть:

1. **Номинал = последний отсчёт.** Если начальное значение берётся как конкретный кадр, а не как среднее по окну, то его неопределённость корректнее приписывать в масштабе **одного отсчёта**: $\hat{\sigma}_{sample}$.
2. **Величина оценена по усреднению.** Если параметр вычислен через $\bar{x}$ (`ba_est`, `bg_est`, углы по уровнянию), то влияние случайного шума ослаблено усреднением. Неопределённость такой оценки падает как $1/\sqrt{N}$, поэтому используется $\hat{\sigma}_{mean}$.
3. **Экстремальное отклонение вместо дисперсии.** Оценка через $\Delta/k(N)$ даёт рабочую оценку масштаба разброса по окну для данной методики. $k(N)$ нормирует максимум модуля отклонений от среднего к эквивалентному СКО стандартного нормального.
4. **Согласованность x0/P0.** Разность `(последний − среднее)` используется для формирования `x0`, а выбранные $\hat{\sigma}_{sample}/\hat{\sigma}_{mean}$ — для задания начальной ковариации. Такое разделение даёт физически осмысленную, но при этом робастную инициализацию под короткое статичное окно в 1 секунду.

**Главный итог:** `AveragingWindow` одновременно формирует $\hat{\sigma}_{sample}$ и $\hat{\sigma}_{mean}$ для ИМУ и $\hat{\sigma}_{sample}$ для СНС. `initialErrorsFromAveraging` переводит `(последний − среднее)` в `x0`, `initialCovarianceFromAveraging` раскидывает $\hat{\sigma}_{sample}$ по компонентам с номиналом из последнего отсчёта, а $\hat{\sigma}_{mean}$ — по компонентам, оценённым по среднему, с учётом корреляций `наклон ↔ ba` и наложением `floors`. Именно по этой схеме в NoQT рассчитываются сигмы для P0.
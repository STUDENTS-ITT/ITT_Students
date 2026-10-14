# Kalman Filter Matrices (ESKF, 15-state) - 13A (truth tuning)
# Источник: лучшие параметры из 13A truth-прогонов (presets/truth/tuned_<set>.ini + temp_tune_truth_<set>/best_params.txt)
# Формирование матриц Q/R/P0 и структуры фильтра описано в исходном коде (src/core/ins/ins_filter.*)
# Данный документ содержит сводку настроечных шумов (диагонали/ключевые параметры) для 8 наборов данных.

Наборы: static300, static_lownoise_align, square650, square_harsh, straight_10ms, figure8_8ms, spiral_climb, high_dynamics

Файл: tools/kalman_tuner/13A_truth_best_params.csv

Также сохранены:
- CSV: tools/kalman_tuner/13A_truth_best_params.csv
- CSV: tools/kalman_tuner/13B_best_params.csv (13B — асимметричные границы + wall penalty)

## Замечания
- sigma_ang в пресетах соответствует sigma_hdg (курс). В тюнере параметр называется sigma_ang.
- bg/ba — смещения гироскопа/акселерометра (bias), их шумы дрейфа — скорости изменения смещений.
- Параметры — настроечные (process/measurement noise scaling) для текущей реализации ESKF (15 состояний). Структура матриц Q,R,P0 не переписывалась (только подбор).
- Полные матрицы Q,R в явном виде можно выгрузить из GUI
  (**«Экспорт → Матрицы фильтра (Q/R/P0/H)…»**, для текущего датасета и
  настроек σ; форматы CSV/Markdown/JSON) — реализовано 2026-10-14.
- В 9-мерной коррекции по СНС тангаж и крен напрямую не наблюдают: они
  обрабатываются отдельным tilt-контуром по акселерометру, поэтому
  R[7,7] = R[8,8] = `kSigTiltIgnore` · `DEG_TO_RAD` ≈ 3.05e8 (заведомо
  большое значение, см. `src/core/ins/ins_filter.h`). Остальные диагонали
  R: `σ_pos²`, `σ_pos²`, `σ_h²`, `σ_v²`×3, `σ_hdg²`.

См. также: docs/kalman_tuning_report.md (раздел 13A), presets/truth/tuned_<set>.ini

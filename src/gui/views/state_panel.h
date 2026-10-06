// state_panel.h — Панель состояния фильтра БИНС.
//
// Показывает выведенные параметры коррекции (смещения ДУС и акселерометра),
// текущие координаты/углы/скорости и диагностику точности. Данные приходят
// из рабочего потока с частотой 10 Гц, отрисовка — из GUI-потока.

#pragma once

#include <QWidget>

#include "core/navigation/simulation.h"
#include "core_bridge/telemetry_buffer.h"

class QDoubleSpinBox;
class QLabel;
class QTableWidget;

namespace gui
{

class StatePanel : public QWidget
{
    Q_OBJECT

public:
    explicit StatePanel(QWidget *parent = nullptr);

    // Обновление из телеметрии (GUI-поток).
    void updateSample(const TelemetrySample &sample);

    void reset();

private:
    QLabel *time_ = nullptr;
    QLabel *step_ = nullptr;
    QLabel *status_ = nullptr;
    QTableWidget *nav_table_ = nullptr;
    QTableWidget *bias_table_ = nullptr;
    QLabel *accuracy_ = nullptr;
};

} // namespace gui
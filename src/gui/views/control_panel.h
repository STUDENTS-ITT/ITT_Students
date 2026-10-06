// control_panel.h — Панель управления прогоном.
//
// Выбор каталога данных и результатов, настроек, запуск/остановка, прогресс.

#pragma once

#include <QWidget>

#include "core/navigation/simulation.h"
#include "settings/app_settings.h"

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;
class QSlider;

namespace gui
{

class ControlPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ControlPanel(QWidget *parent = nullptr);

    void setSettings(const AppSettings &settings);
    AppSettings settings() const { return settings_; }

    nav::RunOptions runOptions() const;

    void setRunning(bool running);
    void setProgress(int percent, double time_s);
    void setSummaryText(const QString &text);

    // Пауза расчёта (кнопка «Пауза»/«Продолжить»).
    void setPaused(bool paused);
    bool isPaused() const;

    // Темп воспроизведения: 0 — максимум, > 0 — во столько раз медленнее
    // реального времени.
    double speedFactor() const;

Q_SIGNALS:
    void startRequested();
    void pauseToggled(bool paused);
    void speedFactorChanged(double factor);
    void stopRequested();
    void settingsChanged();

private Q_SLOTS:
    void browseDataDir();
    void browseOutputDir();
    void browseSettingsFile();
    void onEditChanged();
    void onPauseClicked();
    void onSpeedChanged(int position);

private:
    AppSettings settings_;

    QLineEdit *data_dir_ = nullptr;
    QLineEdit *output_dir_ = nullptr;
    QLineEdit *settings_file_ = nullptr;
    QPushButton *start_button_ = nullptr;
    QPushButton *pause_button_ = nullptr;
    QPushButton *stop_button_ = nullptr;
    QCheckBox *write_files_ = nullptr;
    QCheckBox *telemetry_ = nullptr;
    QCheckBox *save_history_ = nullptr;
    QSlider *speed_slider_ = nullptr;
    QLabel *speed_label_ = nullptr;
    QProgressBar *progress_ = nullptr;
    QLabel *progress_text_ = nullptr;
    QLabel *summary_ = nullptr;
    bool updating_ = false;
    bool paused_ = false;
};

} // namespace gui
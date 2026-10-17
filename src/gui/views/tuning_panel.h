// tuning_panel.h — Вкладка «Подбор». Запуск подбора коэффициентов фильтра
// Калмана (tools/kalman_tuner/tune_kalman.py) как дочернего процесса.
//
// Панель сама по себе не знает о параметрах фильтра: готовый пресет она
// отдаёт приложению сигналом applyPresetRequested, а приложение применяет
// его к таблице параметров (loadSettings + applyToCore).

#pragma once

#include <QByteArray>
#include <QProcess>
#include <QScopedPointer>
#include <QString>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;

namespace gui
{

class TuningPanel : public QWidget
{
    Q_OBJECT

public:
    explicit TuningPanel(QWidget *parent = nullptr);
    ~TuningPanel() override;

    // Каталог данных для подбора (обычно тот же, что и у прогона).
    void setDataDir(const QString &dir);

    // Каталог с тюнером, найденный по дереву проекта относительно exe
    // ("" — скрипт не найден).
    static QString tunerDir();

    bool isRunning() const;

    // Путь к выбранному конфигу подбора (tuner_config*.json или
    // truth_cfg*/cfg_*.json), "" — нет.
    QString currentConfigPath() const;

public Q_SLOTS:
    // Запуск подбора / принудительное завершение процесса.
    void startTuning();
    void stopTuning();

Q_SIGNALS:
    // События подбора для главного лога (запуск, завершение, ошибки).
    void logMessage(const QString &text, int level = 0);
    // Подбор завершён (code == 0 — успешно).
    void tuningFinished(bool ok);

    // Пользователь выбрал пресет для применения к параметрам фильтра.
    void applyPresetRequested(const QString &path);

    // Запрошена сборка «отчёта о подборе» по указанному конфигу тюнера.
    void tuningReportRequested(const QString &config_path);

private Q_SLOTS:
    void onProcessOutput();
    void onProcessFinished(int code, QProcess::ExitStatus status);
    void onBrowseDataDir();
    void onApplyPreset();
    void onBuildReport();
    // При смене конфига подставить его каталог данных в поле «Каталог данных».
    void onConfigChanged();
    // Проверить truth.txt текущего каталога и обновить чекбокс/статус.
    void updateTruthStatus();

private:
    // Путь к пресету: из вывода тюнера или presets/tuned.ini.
    QString presetPath() const;
    void appendLog(const QString &text);
    void setRunning(bool running);
    QString scriptPath() const;
    // Заполнить список конфигов подбора (tuner_config*.json и
    // truth_cfg*/cfg_*.json) рядом со скриптом.
    void rebuildConfigCombo();

    QLineEdit *data_dir_edit_ = nullptr;
    QLineEdit *python_edit_ = nullptr;
    QComboBox *mode_combo_ = nullptr;
    QComboBox *config_combo_ = nullptr;
    QCheckBox *truth_check_ = nullptr;   // сравнение с истиной (--ref-truth)
    QLabel *truth_status_ = nullptr;     // состояние truth.txt каталога данных
    // Пользователь сам снимал/ставил галочку истины: тогда «по умолчанию
    // включено» больше не навязываем до смены набора.
    bool truth_touched_ = false;
    QLabel *status_label_ = nullptr;
    QPushButton *start_button_ = nullptr;
    QPushButton *stop_button_ = nullptr;
    QPushButton *apply_button_ = nullptr;
    QPushButton *report_button_ = nullptr;
    QPlainTextEdit *log_view_ = nullptr;

    QScopedPointer<QProcess> process_;
    QByteArray pending_;   // остаток вывода, не уместившийся в строку
    QString last_preset_;  // путь пресета из вывода тюнера

    static QString findTunerDir();
};

} // namespace gui
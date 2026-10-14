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

private Q_SLOTS:
    void onProcessOutput();
    void onProcessFinished(int code, QProcess::ExitStatus status);
    void onBrowseDataDir();
    void onApplyPreset();

private:
    // Путь к пресету: из вывода тюнера или presets/tuned.ini.
    QString presetPath() const;
    void appendLog(const QString &text);
    void setRunning(bool running);
    QString scriptPath() const;

    QLineEdit *data_dir_edit_ = nullptr;
    QLineEdit *python_edit_ = nullptr;
    QComboBox *mode_combo_ = nullptr;
    QLabel *status_label_ = nullptr;
    QPushButton *start_button_ = nullptr;
    QPushButton *stop_button_ = nullptr;
    QPushButton *apply_button_ = nullptr;
    QPlainTextEdit *log_view_ = nullptr;

    QScopedPointer<QProcess> process_;
    QByteArray pending_;   // остаток вывода, не уместившийся в строку
    QString last_preset_;  // путь пресета из вывода тюнера

    static QString findTunerDir();
};

} // namespace gui
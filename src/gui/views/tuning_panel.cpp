// tuning_panel.cpp — Вкладка «Подбор»: выполнение tune_kalman.py из GUI.
//
// Подбор идёт в фоновом процессе QProcess, вывод тюнера пишется в собственную
// консоль вкладки. Готовый пресет (по умолчанию presets/tuned.ini) применяется
// к параметрам фильтра через сигнал applyPresetRequested.

#include "views/tuning_panel.h"

#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFontDatabase>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QVBoxLayout>

#include "settings/app_settings.h"
#include "settings/dataset_truth.h"
#include "settings/tuner_configs.h"

namespace gui
{

namespace
{
// Имя запускаемого скрипта тюнера.
const char *kTunerScript = "tune_kalman.py";
// Максимум строк консоли вкладки (вывод подбора занимает много места).
const int kMaxConsoleLines = 20000;

// Каталог данных из конфига подбора (для подписи в списке), "" — если нет.
QString dataDirFromConfig(const QString &config_path)
{
    QFile f(config_path);
    if (!f.open(QIODevice::ReadOnly)) return QString();
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    if (!doc.isObject()) return QString();
    return doc.object().value(QStringLiteral("data_dir")).toString();
}
} // namespace

// ------------------------------------------------------------------------
// Поиск каталога tools/kalman_tuner по дереву проекта. exe лежит на
// произвольной глубине (build/ или build/src/gui/), поэтому поднимаемся
// на несколько уровней от каталога приложения.
// ------------------------------------------------------------------------
QString TuningPanel::findTunerDir()
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int level = 0; level < 7; level++)
    {
        if (dir.exists(QString("tools/kalman_tuner/%1").arg(QLatin1String(kTunerScript))))
        {
            return dir.filePath(QString("tools/kalman_tuner"));
        }
        if (!dir.cdUp()) break;
    }
    return QString();
}

QString TuningPanel::tunerDir()
{
    return findTunerDir();
}

TuningPanel::TuningPanel(QWidget *parent) : QWidget(parent)
{
    auto *root = new QVBoxLayout(this);

    // --- Каталог данных ---
    auto *data_row = new QHBoxLayout;
    data_row->addWidget(new QLabel(tr("Каталог данных")));
    data_dir_edit_ = new QLineEdit;
    data_dir_edit_->setPlaceholderText(tr("imu.dat / gps.dat / angle.dat / StartupNav.ini"));
    data_row->addWidget(data_dir_edit_, 1);
    auto *browse_button = new QPushButton(tr("Обзор…"));
    data_row->addWidget(browse_button);
    connect(browse_button, &QPushButton::clicked, this, &TuningPanel::onBrowseDataDir);
    root->addLayout(data_row);

    // --- Python и режим подбора ---
    auto *run_row = new QHBoxLayout;
    run_row->addWidget(new QLabel(tr("Python")));
    python_edit_ = new QLineEdit(QLatin1String("python"));
    python_edit_->setToolTip(tr("Интерпретатор с numpy/scipy (команда или путь к exe)"));
    run_row->addWidget(python_edit_);

    run_row->addWidget(new QLabel(tr("Режим")));
    mode_combo_ = new QComboBox;
    mode_combo_->addItem(tr("Полный подбор"));
    mode_combo_->addItem(tr("Быстрый (--quick)"));
    mode_combo_->addItem(tr("Глобальная сетка (--global-only)"));
    run_row->addWidget(mode_combo_);
    run_row->addStretch(1);
    root->addLayout(run_row);

    // --- Конфиг подбора (по одному конфигу на набор данных) ---
    auto *config_row = new QHBoxLayout;
    config_row->addWidget(new QLabel(tr("Конфиг")));
    config_combo_ = new QComboBox;
    config_combo_->setToolTip(
        tr("Набор параметров подбора: tuner_config*.json рядом со скриптом "
           "(у наборов dataset_shum свой конфиг, например tuner_config.static300.json) "
           "и конфиги экспериментов с истиной truth_cfg*/cfg_*.json (13A/13B/13G)"));
    config_row->addWidget(config_combo_, 1);
    root->addLayout(config_row);

    // --- Сравнение с наземной истиной (truth.txt) ---
    // Тюнер умеет считать ошибку по истинной траектории набора, но только
    // если файл есть и не повреждён; иначе остаётся эталон СНС.
    auto *truth_row = new QHBoxLayout;
    truth_check_ = new QCheckBox(tr("Сравнивать с истиной (truth.txt)"));
    truth_check_->setToolTip(
        tr("Считать ошибку по истинной траектории набора (truth.txt), а не по "
           "эталону СНС (reference.txt). Доступно только для синтетических "
           "наборов, где truth.txt корректен."));
    truth_row->addWidget(truth_check_);
    truth_status_ = new QLabel;
    truth_row->addWidget(truth_status_, 1);
    root->addLayout(truth_row);

    // --- Кнопки управления ---
    auto *buttons_row = new QHBoxLayout;
    start_button_ = new QPushButton(tr("Запустить подбор"));
    stop_button_ = new QPushButton(tr("Остановить"));
    stop_button_->setEnabled(false);
    apply_button_ = new QPushButton(tr("Применить пресет…"));
    apply_button_->setToolTip(tr("Загрузить подобранный пресет в параметры фильтра"));
    report_button_ = new QPushButton(tr("Отчёт о подборе…"));
    report_button_->setToolTip(
        tr("Собрать PDF-отчёт о подборе по best_params.txt и eval.json выбранного конфига"));
    status_label_ = new QLabel;
    buttons_row->addWidget(start_button_);
    buttons_row->addWidget(stop_button_);
    buttons_row->addWidget(apply_button_);
    buttons_row->addWidget(report_button_);
    buttons_row->addWidget(status_label_, 1);
    root->addLayout(buttons_row);

    connect(start_button_, &QPushButton::clicked, this, &TuningPanel::startTuning);
    connect(stop_button_, &QPushButton::clicked, this, &TuningPanel::stopTuning);
    connect(apply_button_, &QPushButton::clicked, this, &TuningPanel::onApplyPreset);
    connect(report_button_, &QPushButton::clicked, this, &TuningPanel::onBuildReport);
    connect(config_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &TuningPanel::onConfigChanged);
    // Проверка truth.txt при смене каталога данных вручную.
    connect(data_dir_edit_, &QLineEdit::editingFinished, this,
            &TuningPanel::updateTruthStatus);
    // Пользователь сам снял/поставил галочку: дальше «по умолчанию включено»
    // не навязываем до смены набора.
    connect(truth_check_, &QCheckBox::clicked, this, [this]() { truth_touched_ = true; });

    // --- Консоль вывода тюнера ---
    log_view_ = new QPlainTextEdit;
    log_view_->setReadOnly(true);
    log_view_->setMaximumBlockCount(kMaxConsoleLines);
    log_view_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    log_view_->setPlaceholderText(tr("Вывод подбора появится здесь…"));
    root->addWidget(log_view_, 1);

    // --- Подпись с пояснением ---
    auto *hint = new QLabel(
        tr("Подбор выполняет tools/kalman_tuner/tune_kalman.py в фоне и занимает время "
           "(полный режим — десятки минут; быстрый/глобальная сетка — заметно меньше). "
           "Параметры поиска берутся из выбранного конфига (tuner_config*.json "
           "или truth_cfg*/cfg_*.json), "
           "готовый пресет записывается в presets/."));
    hint->setWordWrap(true);
    root->addWidget(hint);

    // Каталог данных по умолчанию — как у прогона.
    setDataDir(AppSettings::defaultDataDir());
    rebuildConfigCombo();

    // Если пресет уже был подобран ранее — кнопка применения доступна сразу.
    apply_button_->setEnabled(!presetPath().isEmpty());
    report_button_->setEnabled(!currentConfigPath().isEmpty());
}

TuningPanel::~TuningPanel()
{
    if (process_ != nullptr && process_->state() != QProcess::NotRunning)
    {
        process_->kill();
        process_->waitForFinished(3000);
    }
}

bool TuningPanel::isRunning() const
{
    return process_ != nullptr && process_->state() != QProcess::NotRunning;
}

void TuningPanel::setDataDir(const QString &dir)
{
    data_dir_edit_->setText(dir);
    // Новый набор: снова применяем «по умолчанию — истина».
    truth_touched_ = false;
    updateTruthStatus();
}

QString TuningPanel::scriptPath() const
{
    const QString dir = tunerDir();
    if (dir.isEmpty()) return QString();
    return QDir(dir).filePath(QLatin1String(kTunerScript));
}

QString TuningPanel::currentConfigPath() const
{
    if (config_combo_ == nullptr || config_combo_->currentIndex() < 0) return QString();
    return config_combo_->currentData().toString();
}

// Заполнить список конфигов подбора (tuner_config*.json и truth_cfg*/cfg_*.json
// рядом со скриптом).
void TuningPanel::rebuildConfigCombo()
{
    if (config_combo_ == nullptr) return;
    const QString previous = currentConfigPath();

    config_combo_->blockSignals(true);
    config_combo_->clear();

    const QString dir = tunerDir();
    if (!dir.isEmpty())
    {
        const QDir d(dir);
        const QStringList configs = listTunerConfigs(dir);
        for (const QString &rel : configs)
        {
            const QString path = d.filePath(rel);
            // Подписью показываем набор данных из конфига, если он читается.
            QString label = rel;
            const QString data_dir = dataDirFromConfig(path);
            if (!data_dir.isEmpty())
                label = QStringLiteral("%1  ·  %2").arg(QDir(data_dir).dirName(), rel);
            config_combo_->addItem(label, path);
        }
    }

    // Выбор по умолчанию — прежний `tuner_config.json`, иначе первый в списке.
    int idx = -1;
    if (!previous.isEmpty()) idx = config_combo_->findData(previous);
    if (idx < 0 && !dir.isEmpty())
        idx = config_combo_->findData(QDir(dir).filePath(QStringLiteral("tuner_config.json")));
    if (idx < 0 && config_combo_->count() > 0) idx = 0;
    if (idx >= 0) config_combo_->setCurrentIndex(idx);
    config_combo_->blockSignals(false);

    // Подтянуть каталог данных и доступность кнопки под выбранный конфиг.
    onConfigChanged();
}

void TuningPanel::onConfigChanged()
{
    const QString path = currentConfigPath();
    if (report_button_ != nullptr)
        report_button_->setEnabled(!path.isEmpty());

    // Каталог данных из конфига подставляем в поле — у наборов dataset_shum
    // он совпадает с конфигом, и подбор идёт по тому же набору.
    const QString data_dir = dataDirFromConfig(path);
    if (!data_dir.isEmpty()) data_dir_edit_->setText(data_dir);

    // Смена набора — снова применяем «по умолчанию — истина».
    truth_touched_ = false;
    updateTruthStatus();
}

// Проверить truth.txt выбранного каталога: статус показываем рядом, а чекбокс
// «Сравнивать с истиной» включаем автоматически (сравнение по умолчанию — с
// истиной), но только пока пользователь не трогал его сам.
void TuningPanel::updateTruthStatus()
{
    if (truth_check_ == nullptr || truth_status_ == nullptr) return;

    const TruthCheck check = checkDatasetTruth(data_dir_edit_->text().trimmed());
    truth_status_->setText(truthStatusText(check));

    const bool ok = (check.state == TruthState::Ok);
    truth_check_->setEnabled(ok);
    if (!ok)
        truth_check_->setChecked(false);
    else if (!truth_touched_)
        truth_check_->setChecked(true);
}

void TuningPanel::onBuildReport()
{
    const QString config = currentConfigPath();
    if (config.isEmpty())
    {
        appendLog(tr("Отчёт: не выбран конфиг подбора."));
        return;
    }
    Q_EMIT tuningReportRequested(config);
}

void TuningPanel::startTuning()
{
    if (isRunning()) return;

    const QString script = scriptPath();
    if (script.isEmpty() || !QFileInfo::exists(script))
    {
        appendLog(tr("Ошибка: не найден скрипт тюнера (%1)").arg(
                      QDir(tunerDir()).filePath(QLatin1String(kTunerScript))));
        Q_EMIT logMessage(tr("Подбор: скрипт тюнера не найден"), 2);
        return;
    }

    const QString data_dir = data_dir_edit_->text().trimmed();
    if (data_dir.isEmpty())
    {
        appendLog(tr("Ошибка: укажите каталог данных."));
        return;
    }

    // Режим подбора: как в main() тюнера, только --data-dir и флаги задаём
    // из GUI; остальное (сетки, метрика, пресет) — из выбранного конфига.
    const QString config = currentConfigPath();
    if (config.isEmpty())
    {
        appendLog(tr("Ошибка: не найден конфиг подбора (tuner_config*.json / "
                     "truth_cfg*/cfg_*.json) рядом со скриптом."));
        Q_EMIT logMessage(tr("Подбор: не найден конфиг подбора"), 2);
        return;
    }

    QStringList args;
    args << QLatin1String("-u") << script
         << QString("--config=%1").arg(QDir::toNativeSeparators(config))
         << QString("--data-dir=%1").arg(QDir::toNativeSeparators(data_dir));
    switch (mode_combo_->currentIndex())
    {
    case 1: args << QLatin1String("--quick"); break;
    case 2: args << QLatin1String("--global-only"); break;
    default: break;
    }

    // Сравнение с наземной истиной: добавляем --ref-truth только если
    // truth.txt на месте и не повреждён (иначе тюнер молча вернулся бы к СНС).
    if (truth_check_ != nullptr && truth_check_->isChecked())
    {
        const TruthCheck check = checkDatasetTruth(data_dir);
        if (check.state == TruthState::Ok)
        {
            args << QLatin1String("--ref-truth");
        }
        else
        {
            QMessageBox::warning(
                this, tr("Подбор"),
                tr("В наборе нет корректного truth.txt (%1).\n"
                   "Подбор пойдёт по эталону СНС (reference.txt).")
                    .arg(check.detail));
            truth_check_->setChecked(false);
            updateTruthStatus();
        }
    }

    process_.reset(new QProcess(this));
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert(QLatin1String("PYTHONIOENCODING"), QLatin1String("utf-8"));
    process_->setProcessEnvironment(env);
    process_->setProgram(python_edit_->text().trimmed().isEmpty()
                             ? QLatin1String("python")
                             : python_edit_->text().trimmed());
    process_->setArguments(args);
    // Рабочий каталог — рядом со скриптом: так повторяются штатные запуски
    // тюнера (полные пути в конфиге и так писать нельзя удалять).
    process_->setWorkingDirectory(tunerDir());
    process_->setProcessChannelMode(QProcess::MergedChannels);

    connect(process_.data(), &QProcess::readyReadStandardOutput, this,
            &TuningPanel::onProcessOutput);
    connect(process_.data(),
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
            &TuningPanel::onProcessFinished);

    pending_.clear();
    log_view_->clear();
    appendLog(tr("Запуск: %1 %2").arg(process_->program(), args.join(QLatin1Char(' '))));
    status_label_->setText(tr("Подбор выполняется…"));
    setRunning(true);

    process_->start();
    if (!process_->waitForStarted(5000))
    {
        appendLog(tr("Ошибка: не удалось запустить %1 (%2)")
                      .arg(process_->program(), process_->errorString()));
        setRunning(false);
        status_label_->clear();
        return;
    }

    Q_EMIT logMessage(tr("Запущен подбор коэффициентов (каталог данных: %1)").arg(data_dir));
}

void TuningPanel::stopTuning()
{
    if (!isRunning()) return;
    appendLog(tr("Остановка процесса…"));
    process_->kill();
    process_->waitForFinished(3000);
}

void TuningPanel::onProcessOutput()
{
    pending_.append(process_->readAllStandardOutput());

    int pos = 0;
    while (true)
    {
        const int nl = pending_.indexOf('\n', pos);
        if (nl < 0) break;
        const QByteArray line = pending_.mid(pos, nl - pos);
        pos = nl + 1;
        appendLog(QString::fromUtf8(line));
    }
    pending_.remove(0, pos);

    // Пресет сохраняется тюнером в конце: «Пресет сохранён: <путь>».
    const QString text = log_view_->toPlainText();
    const int mark = text.lastIndexOf(tr("Пресет сохранён:"));
    if (mark >= 0)
    {
        const QString path = text.mid(mark + tr("Пресет сохранён:").size()).trimmed();
        if (!path.isEmpty()) last_preset_ = path;
    }
}

void TuningPanel::onProcessFinished(int code, QProcess::ExitStatus status)
{
    // Хвост вывода без завершающего перевода строки.
    if (!pending_.isEmpty())
    {
        appendLog(QString::fromUtf8(pending_));
        pending_.clear();
    }

    const bool ok = (status == QProcess::NormalExit && code == 0);
    status_label_->setText(ok ? tr("Готово. Пресет: %1").arg(presetPath())
                              : tr("Подбор завершён с ошибкой (код %1)").arg(code));
    appendLog(ok ? tr("Подбор завершён.") : tr("Подбор прерван или завершился с ошибкой (код %1).")
                                                    .arg(code));
    setRunning(false);

    apply_button_->setEnabled(!presetPath().isEmpty());
    Q_EMIT logMessage(ok ? tr("Подбор завершён, пресет: %1").arg(presetPath())
                         : tr("Подбор завершился с ошибкой (код %1)").arg(code),
                      ok ? 0 : 2);
    Q_EMIT tuningFinished(ok);
}

QString TuningPanel::presetPath() const
{
    if (!last_preset_.isEmpty() && QFileInfo::exists(last_preset_)) return last_preset_;
    // Тюнер не сообщил путь (например, режим --global-only) — ищем дефолт.
    const QString fallback = QDir(tunerDir()).filePath(QLatin1String("../presets/tuned.ini"));
    if (QFileInfo::exists(fallback)) return fallback;
    return QString();
}

void TuningPanel::onBrowseDataDir()
{
    const QString dir = QFileDialog::getExistingDirectory(
        this, tr("Каталог данных подбора"),
        data_dir_edit_->text().isEmpty() ? AppSettings::defaultDataDir()
                                         : data_dir_edit_->text());
    if (!dir.isEmpty()) data_dir_edit_->setText(dir);
}

void TuningPanel::appendLog(const QString &text)
{
    if (text.isEmpty()) return;
    log_view_->appendPlainText(text);
}

void TuningPanel::onApplyPreset()
{
    const QString current = presetPath();
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Применить пресет подбора"),
        current.isEmpty() ? QDir(tunerDir()).filePath(QLatin1String("../presets"))
                          : current,
        tr("INI-файлы (*.ini);;Все файлы (*)"));
    if (path.isEmpty()) return;
    last_preset_ = path;
    Q_EMIT applyPresetRequested(path);
}

void TuningPanel::setRunning(bool running)
{
    start_button_->setEnabled(!running);
    stop_button_->setEnabled(running);
    data_dir_edit_->setEnabled(!running);
    python_edit_->setEnabled(!running);
    mode_combo_->setEnabled(!running);
    config_combo_->setEnabled(!running);
    report_button_->setEnabled(!running && !currentConfigPath().isEmpty());
    if (truth_check_ != nullptr)
    {
        if (running) truth_check_->setEnabled(false);
        else updateTruthStatus();
    }
    if (!running) apply_button_->setEnabled(!presetPath().isEmpty());
}

} // namespace gui
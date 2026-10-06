// control_panel.cpp — Панель управления прогоном.

#include "views/control_panel.h"

#include <QCheckBox>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>

namespace gui
{
namespace
{
// Позиции ползунка темпа: 0 — максимум, дальше «во столько раз медленнее
// реального времени».
const double kSpeedFactors[] = {0.0, 32.0, 16.0, 8.0, 4.0, 2.0, 1.0};

QString speedText(double factor)
{
    if (factor <= 0.0) return QObject::tr("максимум");
    return QObject::tr("1/%1 от реального времени").arg(factor, 0, 'g', 3);
}
} // namespace

ControlPanel::ControlPanel(QWidget *parent) : QWidget(parent)
{
    auto *root = new QVBoxLayout(this);
    root->setSpacing(6);

    auto *form = new QFormLayout;
    form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

    data_dir_ = new QLineEdit;
    auto *data_btn = new QPushButton(tr("…"));
    data_btn->setMaximumWidth(32);
    auto *data_row = new QHBoxLayout;
    data_row->setContentsMargins(0, 0, 0, 0);
    data_row->addWidget(data_dir_, 1);
    data_row->addWidget(data_btn);
    form->addRow(tr("Каталог данных"), data_row);

    output_dir_ = new QLineEdit;
    auto *out_btn = new QPushButton(tr("…"));
    out_btn->setMaximumWidth(32);
    auto *out_row = new QHBoxLayout;
    out_row->setContentsMargins(0, 0, 0, 0);
    out_row->addWidget(output_dir_, 1);
    out_row->addWidget(out_btn);
    form->addRow(tr("Каталог результатов"), out_row);

    settings_file_ = new QLineEdit;
    auto *set_btn = new QPushButton(tr("…"));
    set_btn->setMaximumWidth(32);
    auto *set_row = new QHBoxLayout;
    set_row->setContentsMargins(0, 0, 0, 0);
    set_row->addWidget(settings_file_, 1);
    set_row->addWidget(set_btn);
    form->addRow(tr("settings.ini"), set_row);

    root->addLayout(form);

    auto *options = new QHBoxLayout;
    write_files_ = new QCheckBox(tr("Писать файлы"));
    telemetry_ = new QCheckBox(tr("Телеметрия"));
    save_history_ = new QCheckBox(tr("В историю (SQLite)"));
    options->addWidget(write_files_);
    options->addWidget(telemetry_);
    options->addWidget(save_history_);
    options->addStretch(1);
    root->addLayout(options);

    auto *buttons = new QHBoxLayout;
    start_button_ = new QPushButton(tr("Запустить"));
    pause_button_ = new QPushButton(tr("Пауза"));
    pause_button_->setEnabled(false);
    pause_button_->setCheckable(true);
    stop_button_ = new QPushButton(tr("Стоп"));
    stop_button_->setEnabled(false);
    buttons->addWidget(start_button_);
    buttons->addWidget(pause_button_);
    buttons->addWidget(stop_button_);
    buttons->addStretch(1);
    root->addLayout(buttons);

    // Темп воспроизведения: максимум по умолчанию, чтобы прогон занимал
    // столько же времени, сколько и без GUI.
    auto *speed_row = new QHBoxLayout;
    speed_slider_ = new QSlider(Qt::Horizontal);
    speed_slider_->setRange(0, 6);
    speed_slider_->setValue(0);
    speed_slider_->setToolTip(tr("Темп расчёта относительно реального времени"));
    speed_label_ = new QLabel(speedText(kSpeedFactors[0]));
    speed_label_->setMinimumWidth(170);
    speed_row->addWidget(speed_slider_, 1);
    speed_row->addWidget(speed_label_, 0);
    root->addLayout(speed_row);

    progress_ = new QProgressBar;
    progress_->setRange(0, 100);
    progress_->setValue(0);
    progress_->setTextVisible(false);
    root->addWidget(progress_);

    auto *info = new QHBoxLayout;
    progress_text_ = new QLabel(tr("Готов"));
    summary_ = new QLabel;
    summary_->setWordWrap(true);
    summary_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    info->addWidget(progress_text_, 0);
    info->addWidget(summary_, 1);
    root->addLayout(info);

    connect(start_button_, &QPushButton::clicked, this, &ControlPanel::startRequested);
    connect(stop_button_, &QPushButton::clicked, this, &ControlPanel::stopRequested);
    connect(pause_button_, &QPushButton::clicked, this, &ControlPanel::onPauseClicked);
    connect(speed_slider_, &QSlider::valueChanged, this, &ControlPanel::onSpeedChanged);

    connect(data_btn, &QPushButton::clicked, this, &ControlPanel::browseDataDir);
    connect(out_btn, &QPushButton::clicked, this, &ControlPanel::browseOutputDir);
    connect(set_btn, &QPushButton::clicked, this, &ControlPanel::browseSettingsFile);

    connect(data_dir_, &QLineEdit::editingFinished, this, &ControlPanel::onEditChanged);
    connect(output_dir_, &QLineEdit::editingFinished, this, &ControlPanel::onEditChanged);
    connect(settings_file_, &QLineEdit::editingFinished, this, &ControlPanel::onEditChanged);
    connect(write_files_, &QCheckBox::toggled, this, &ControlPanel::onEditChanged);
    connect(telemetry_, &QCheckBox::toggled, this, &ControlPanel::onEditChanged);
    connect(save_history_, &QCheckBox::toggled, this, &ControlPanel::onEditChanged);
}

void ControlPanel::setSettings(const AppSettings &s)
{
    updating_ = true;
    settings_ = s;
    data_dir_->setText(s.data_dir);
    output_dir_->setText(s.output_dir);
    settings_file_->setText(s.settings_file);
    write_files_->setChecked(s.write_files);
    telemetry_->setChecked(s.capture_telemetry);
    save_history_->setChecked(s.save_to_history);
    updating_ = false;
}

nav::RunOptions ControlPanel::runOptions() const
{
    nav::RunOptions opts;
    opts.data_dir = data_dir_->text().toStdString();
    opts.output_dir = output_dir_->text().toStdString();
    opts.settings_file = settings_file_->text().toStdString();
    opts.write_files = write_files_->isChecked();
    opts.capture_telemetry = telemetry_->isChecked();
    return opts;
}

void ControlPanel::setRunning(bool running)
{
    start_button_->setEnabled(!running);
    stop_button_->setEnabled(running);
    pause_button_->setEnabled(running);
    data_dir_->setEnabled(!running);
    output_dir_->setEnabled(!running);
    settings_file_->setEnabled(!running);
    if (running)
    {
        setPaused(false);
        progress_->setValue(0);
        progress_text_->setText(tr("Прогон…"));
    }
}

void ControlPanel::setPaused(bool paused)
{
    paused_ = paused;
    if (pause_button_ != nullptr)
    {
        pause_button_->setChecked(paused);
        pause_button_->setText(paused ? tr("Продолжить") : tr("Пауза"));
    }
    if (!paused_)
    {
        progress_text_->setText(tr("Прогон…"));
    }
}

bool ControlPanel::isPaused() const
{
    return paused_;
}

double ControlPanel::speedFactor() const
{
    if (speed_slider_ == nullptr) return 0.0;
    const int pos = speed_slider_->value();
    const int n = static_cast<int>(sizeof(kSpeedFactors) / sizeof(kSpeedFactors[0]));
    if (pos < 0 || pos >= n) return 0.0;
    return kSpeedFactors[pos];
}

void ControlPanel::onPauseClicked()
{
    setPaused(pause_button_->isChecked());
    Q_EMIT pauseToggled(paused_);
}

void ControlPanel::onSpeedChanged(int position)
{
    const int n = static_cast<int>(sizeof(kSpeedFactors) / sizeof(kSpeedFactors[0]));
    const double factor =
        (position >= 0 && position < n) ? kSpeedFactors[position] : 0.0;
    if (speed_label_ != nullptr) speed_label_->setText(speedText(factor));
    Q_EMIT speedFactorChanged(factor);
}

void ControlPanel::setProgress(int percent, double time_s)
{
    if (percent < 0)
    {
        // Число строк неизвестно — показываем «неизвестно».
        progress_->setRange(0, 0);
        progress_text_->setText(tr("t = %1 с").arg(time_s, 0, 'f', 1));
    }
    else
    {
        progress_->setRange(0, 100);
        progress_->setValue(percent);
        progress_text_->setText(tr("%1%, t = %2 с").arg(percent).arg(time_s, 0, 'f', 1));
    }
}

void ControlPanel::setSummaryText(const QString &text)
{
    summary_->setText(text);
}

void ControlPanel::browseDataDir()
{
    const QString dir = QFileDialog::getExistingDirectory(
        this, tr("Каталог с imu.dat / gps.dat / angle.dat / StartupNav.ini"),
        data_dir_->text());
    if (dir.isEmpty()) return;
    data_dir_->setText(dir);
    onEditChanged();
}

void ControlPanel::browseOutputDir()
{
    const QString dir = QFileDialog::getExistingDirectory(
        this, tr("Каталог для result.txt / reference.txt / errors.txt"),
        output_dir_->text());
    if (dir.isEmpty()) return;
    output_dir_->setText(dir);
    onEditChanged();
}

void ControlPanel::browseSettingsFile()
{
    const QString file = QFileDialog::getOpenFileName(
        this, tr("Файл параметров фильтра"), settings_file_->text(),
        tr("INI-файлы (*.ini);;Все файлы (*)"));
    if (file.isEmpty()) return;
    settings_file_->setText(file);
    onEditChanged();
}

void ControlPanel::onEditChanged()
{
    if (updating_) return;

    settings_.data_dir = data_dir_->text();
    settings_.output_dir = output_dir_->text();
    settings_.settings_file = settings_file_->text();
    settings_.write_files = write_files_->isChecked();
    settings_.capture_telemetry = telemetry_->isChecked();
    settings_.save_to_history = save_history_->isChecked();
    Q_EMIT settingsChanged();
}

} // namespace gui
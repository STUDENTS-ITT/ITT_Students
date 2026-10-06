// log_panel.h — Панель лога: сообщения ядра, GUI и ошибки.

#pragma once

#include <QWidget>

class QCheckBox;
class QPlainTextEdit;
class QPushButton;

namespace gui
{

class LogPanel : public QWidget
{
    Q_OBJECT

public:
    explicit LogPanel(QWidget *parent = nullptr);

    // Уровни: 0 — информация, 1 — предупреждение, 2 — ошибка.
    void append(const QString &text, int level = 0);

    // Максимум строк в буфере (по умолчанию 5000).
    void setMaxLines(int lines);

    void saveToFile(const QString &fileName) const;
    QString text() const;

public Q_SLOTS:
    void clear();

private:
    QPlainTextEdit *view_ = nullptr;
    QCheckBox *autoscroll_ = nullptr;
    QPushButton *clear_button_ = nullptr;
    QPushButton *save_button_ = nullptr;
    int max_lines_ = 5000;
};

} // namespace gui
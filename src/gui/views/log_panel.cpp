// log_panel.cpp — Панель лога.

#include "views/log_panel.h"

#include <QCheckBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QPalette>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSaveFile>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QTextDocument>
#include <QVBoxLayout>

namespace gui
{

LogPanel::LogPanel(QWidget *parent) : QWidget(parent)
{
    auto *root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);

    auto *tools = new QHBoxLayout;
    clear_button_ = new QPushButton(tr("Очистить"));
    save_button_ = new QPushButton(tr("Сохранить…"));
    autoscroll_ = new QCheckBox(tr("Прокрутка"));
    autoscroll_->setChecked(true);
    tools->addWidget(clear_button_);
    tools->addWidget(save_button_);
    tools->addWidget(autoscroll_);
    tools->addStretch(1);
    root->addLayout(tools);

    view_ = new QPlainTextEdit;
    view_->setReadOnly(true);
    view_->setMaximumBlockCount(max_lines_);
    view_->setLineWrapMode(QPlainTextEdit::NoWrap);
    QFont f("Consolas", 9);
    f.setStyleHint(QFont::Monospace);
    view_->setFont(f);
    root->addWidget(view_, 1);

    connect(clear_button_, &QPushButton::clicked, this, &LogPanel::clear);
    connect(save_button_, &QPushButton::clicked, this, [this]() {
        const QString path = QFileDialog::getSaveFileName(
            this, tr("Сохранить лог"), QString(), tr("Текст (*.txt);;Все файлы (*)"));
        if (!path.isEmpty()) saveToFile(path);
    });
}

void LogPanel::setMaxLines(int lines)
{
    max_lines_ = qMax(100, lines);
    view_->setMaximumBlockCount(max_lines_);
}

void LogPanel::append(const QString &text, int level)
{
    QString prefix;
    QColor color = view_->palette().color(QPalette::Text);
    switch (level)
    {
    case 2: prefix = QStringLiteral("[ОШИБКА] "); color = QColor(176, 32, 32); break;
    case 1: prefix = QStringLiteral("[ВНИМАНИЕ] "); color = QColor(160, 96, 0); break;
    default: break;
    }

    QTextCharFormat fmt;
    fmt.setForeground(color);

    // Пишем в конец документа, не трогая позицию курсора пользователя.
    QTextCursor cursor = view_->textCursor();
    cursor.movePosition(QTextCursor::End);

    const bool needs_block = view_->document()->characterCount() > 1;
    if (needs_block) cursor.insertBlock();

    cursor.insertText(prefix + text, fmt);

    if (autoscroll_->isChecked())
    {
        view_->moveCursor(QTextCursor::End);
    }
}

void LogPanel::clear()
{
    view_->clear();
}

QString LogPanel::text() const
{
    return view_->toPlainText();
}

void LogPanel::saveToFile(const QString &fileName) const
{
    QSaveFile f(fileName);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    f.write(view_->toPlainText().toUtf8());
    f.commit();
}

} // namespace gui
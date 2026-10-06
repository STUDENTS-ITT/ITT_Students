// input_tree.h — Модель дерева входных данных.
//
// Показывает imu.dat / gps.dat / angle.dat / StartupNav.ini с размерами,
// числом строк и подсветкой отсутствующих файлов. Число строк imu.dat
// (200 Гц × длительность) считается один раз и кешируется.

#pragma once

#include <QAbstractItemModel>
#include <QString>
#include <QVector>

namespace gui
{

// Описание одного входного файла.
struct InputFile
{
    QString name;
    QString path;
    bool required = true;
    bool exists = false;
    qint64 size = 0;
    qint64 lines = 0;
    QString note;
};

class InputTreeModel : public QAbstractItemModel
{
    Q_OBJECT

public:
    enum Roles
    {
        PathRole = Qt::UserRole + 1,
        ExistsRole,
        LinesRole,
        SizeRole,
        NoteRole
    };

    explicit InputTreeModel(QObject *parent = nullptr);

    // Сканировать каталог данных и обновить дерево.
    void setDataDir(const QString &dir);
    QString dataDir() const { return dir_; }

    const QVector<InputFile> &files() const { return files_; }

    // QAbstractItemModel
    QModelIndex index(int row, int column,
                      const QModelIndex &parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex &child) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;

    // Суммарное число строк ИМУ — для оценки длительности прогона.
    qint64 imuRowCount() const;

private:
    struct Node
    {
        QString name;
        int file_index = -1;  // -1 — каталог
        QVector<int> children;
        int parent = -1;
    };

    void scan();
    static qint64 countLines(const QString &path);

    QString dir_;
    QVector<InputFile> files_;
    QVector<Node> nodes_;
};

} // namespace gui
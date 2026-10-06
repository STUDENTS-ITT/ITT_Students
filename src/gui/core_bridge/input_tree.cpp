// input_tree.cpp — Модель дерева входных данных.

#include "core_bridge/input_tree.h"

#include <QBrush>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QStringList>

namespace gui
{

InputTreeModel::InputTreeModel(QObject *parent) : QAbstractItemModel(parent)
{
    // Корень — сам каталог данных.
    Node root;
    root.name = "data/raw";
    nodes_.push_back(root);

    const struct
    {
        const char *name;
        bool required;
        const char *note;
    } spec[] = {
        {"imu.dat", true, "ИМУ: время, гироскоп, акселерометр"},
        {"gps.dat", true, "СНС: координаты и скорости (1 Гц)"},
        {"angle.dat", false, "Эталон углов ориентации (опционально)"},
        {"StartupNav.ini", true, "Координаты и время выставки"},
    };

    for (const auto &s : spec)
    {
        InputFile f;
        f.name = QString::fromLatin1(s.name);
        f.required = s.required;
        f.note = QString::fromUtf8(s.note);
        files_.push_back(f);

        Node n;
        n.name = f.name;
        n.file_index = files_.size() - 1;
        n.parent = 0;
        nodes_.push_back(n);
        nodes_[0].children.push_back(nodes_.size() - 1);
    }
}

qint64 InputTreeModel::countLines(const QString &path)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return 0;

    qint64 lines = 0;
    const qint64 block = 1 << 20;
    QByteArray buffer;
    while (!f.atEnd())
    {
        buffer = f.read(block);
        lines += buffer.count('\n');
    }
    // Последняя строка без перевода строки.
    if (!buffer.isEmpty() && !buffer.endsWith('\n')) lines++;
    return lines;
}

void InputTreeModel::setDataDir(const QString &dir)
{
    beginResetModel();
    dir_ = dir;
    nodes_[0].name = dir.isEmpty() ? QString("data/raw") : dir;
    scan();
    endResetModel();
}

void InputTreeModel::scan()
{
    for (InputFile &f : files_)
    {
        const QString path = dir_.isEmpty() ? QString() : QDir(dir_).filePath(f.name);
        f.path = path;
        const QFileInfo info(path);
        f.exists = info.exists() && info.isFile();
        f.size = f.exists ? info.size() : 0;
        f.lines = f.exists ? countLines(path) : 0;
        if (f.lines > 0 && f.name == "imu.dat")
        {
            // Первая строка — заголовок.
            f.lines -= 1;
        }
    }
}

QModelIndex InputTreeModel::index(int row, int column, const QModelIndex &parent) const
{
    if (column != 0) return QModelIndex();

    int parent_node = 0;
    if (parent.isValid())
    {
        parent_node = static_cast<int>(parent.internalId());
    }

    if (row < 0 || row >= nodes_[parent_node].children.size()) return QModelIndex();

    const int node = nodes_[parent_node].children[row];
    return createIndex(row, column, static_cast<quintptr>(node));
}

QModelIndex InputTreeModel::parent(const QModelIndex &child) const
{
    if (!child.isValid()) return QModelIndex();

    const int node = static_cast<int>(child.internalId());
    const int p = nodes_[node].parent;
    if (p < 0) return QModelIndex();

    // Номер строки среди детей родителя.
    const int row = nodes_[p].children.indexOf(node);
    return createIndex(row, 0, static_cast<quintptr>(p));
}

int InputTreeModel::rowCount(const QModelIndex &parent) const
{
    if (parent.column() > 0) return 0;
    const int node = parent.isValid() ? static_cast<int>(parent.internalId()) : 0;
    return nodes_[node].children.size();
}

int InputTreeModel::columnCount(const QModelIndex &) const
{
    return 1;
}

QVariant InputTreeModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid()) return QVariant();

    const int node_id = static_cast<int>(index.internalId());
    const Node &node = nodes_[node_id];

    if (node.file_index < 0)
    {
        if (role == Qt::DisplayRole) return node.name;
        return QVariant();
    }

    const InputFile &f = files_[node.file_index];

    switch (role)
    {
    case Qt::DisplayRole:
        if (f.exists)
        {
            return QString("%1  (%2 строк, %3 МБ)")
                .arg(f.name)
                .arg(f.lines)
                .arg(f.size / (1024 * 1024));
        }
        return QString("%1  —  %2").arg(f.name, f.required ? "нет файла" : "нет (опционально)");

    case Qt::ForegroundRole:
        if (!f.exists)
        {
            return f.required ? QBrush(Qt::red) : QBrush(Qt::darkYellow);
        }
        return QVariant();

    case Qt::ToolTipRole:
    {
        QString tip = f.note;
        if (f.exists)
        {
            tip += QString("\n%1\nСтрок: %2").arg(f.path).arg(f.lines);
        }
        return tip;
    }

    case PathRole: return f.path;
    case ExistsRole: return f.exists;
    case LinesRole: return f.lines;
    case SizeRole: return f.size;
    case NoteRole: return f.note;
    default: return QVariant();
    }
}

QVariant InputTreeModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole) return QVariant();
    return QString("Входные данные");
}

qint64 InputTreeModel::imuRowCount() const
{
    for (const InputFile &f : files_)
    {
        if (f.name == "imu.dat") return f.lines;
    }
    return 0;
}

} // namespace gui
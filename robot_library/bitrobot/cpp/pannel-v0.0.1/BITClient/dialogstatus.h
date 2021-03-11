#ifndef DIALOGSTATUS_H
#define DIALOGSTATUS_H

#include <QDialog>
#include <QTimerEvent>
#include <QAbstractTableModel>
#include "wrapstat.h"
#include <map>




class MyModel : public QAbstractTableModel
{
    Q_OBJECT
public:
    enum {
        colX = 0, colY, colTh, colV, colW, colState, colTask, colProg, colNum
    };

    MyModel(QObject *parent = nullptr);
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

public slots:
    void onNewSTAT(const qSTAT& q);
    void onClearData();

private:
    void refreshIndex();
    struct Data {
        int index;
        qSTAT q;
    };
    std::vector<int> index2ID;
    std::map<int, Data> m_data;
};

inline void MyModel::onNewSTAT(const qSTAT& q) {
    beginResetModel();
    auto it = m_data.find(q.id());
    if (it != m_data.end()) {
        it->second.q = q;
    }
    else {
        auto& one = m_data[q.id()];
        one.q = q;
        refreshIndex();
    }
    endResetModel();
}
inline void MyModel::onClearData() {
    beginResetModel();
    m_data.clear();
    index2ID.clear();
    endResetModel();
}

inline void MyModel::refreshIndex() {
    index2ID.clear();
    for (auto& item: m_data) {
        int id = item.first;
        int index = (int)index2ID.size();
        index2ID.push_back(id);
        item.second.index = index;
    }
}


namespace Ui {
class DialogStatus;
}

class DialogStatus : public QDialog
{
    Q_OBJECT

public:
    explicit DialogStatus(QWidget *parent = nullptr);
    ~DialogStatus();

protected:
    void timerEvent(QTimerEvent* event);

private:
    bool plotted = false;
    int timerOnce = -1;
    int timerRefresh = -1;
    Ui::DialogStatus *ui;
public:
    MyModel* data;
};

#endif // DIALOGSTATUS_H

#pragma once
#include "winfunction.hpp"
#include <QTimerEvent>
#include <queue>
#include <unordered_map>
#include <unordered_set>

struct XYTheta {
    double x = 0, y = 0, th = 0;
};
struct XYThetaStat {
    XYThetaStat(int n = 0): data(n), diff(n) {}
    void resize(int n) { data.resize(n); diff.resize(n); }
    void update() {
        int num = (int)data.size();
        for (int i=0;i<num;++i) {

        }
    }
    QString toString();

    XYTheta mean;
    std::vector<XYTheta> data;
    std::vector<XYTheta> diff;
};

class FormationHRI : public QObject
{
    Q_OBJECT
public:
    void onTaskBegin();
    void onTaskStop();
    bool started() const { return m_started; }

    bool eventFilter(QObject *watched, QEvent *event);
    void onNewPointsInGraph(float x, float y);
    void onRemovePointsInGraph();

signals:
    void newFormationInfo(const QString& info);

public slots:
    void onConfirm();
    void onSTAT(const qSTAT& stat);

protected:
    void timerEvent(QTimerEvent *event);

private:
    void onConfirmNoPlan();
    void onConfirmDoPlan();
    QString idset2string();

public:
    bool m_started = false;
    bool m_confirmed = false;
    WindowFunction window;

    int pathVelocity = 30;
    int edgeLength = 160;
    int formationGroup = 0;
    int timerStat = -1;
    int dt_stat = 100;

    std::string buffer;
    QCPCurve* pathLine;

    std::unordered_set<int> robotUsed;
    std::vector<int> robotUsed_IDList;  // temporary, for fast check
    std::unordered_map<int, XYTheta> robotData;
    XYThetaStat currStat;
};


inline void FormationHRI::onTaskBegin() {
    if (m_started) return;

    // Test If Has Required Items
    if (!window.window) return;
    if (!window.graph) return;
    if (!window.btnConfirm) return;
    if (!window.robot_position) return;
    if (!window.bit_client) return;
    if (!window.winTask) return;

    buffer.clear();
    buffer.resize(3000, '\0');
    if (pathLine != nullptr) {
        window.mapitems->tempdata().remove(pathLine);
        pathLine = nullptr;
    }

    robotUsed.clear();
    robotData.clear();

    // Connect Events
    window.graph->installEventFilter(this);
    window.window->installEventFilter(this);
    QObject::connect(window.btnConfirm, &QPushButton::clicked, this, &FormationHRI::onConfirm);
    //QObject::connect(this, &FormationHRI::newFormationInfo, window.winTask, &DialogTask::newInfo);
    QObject::connect(window.bit_client, &QBITClient::newSTAT, this, &FormationHRI::onSTAT);
    m_started = true;
    m_confirmed = false;
    timerStat = -1;
}

inline void FormationHRI::onTaskStop() {
    if (!m_started) return;
    if (pathLine)
        window.mapitems->tempdata().remove(pathLine);
    pathLine = nullptr;

    window.graph->removeEventFilter(this);
    window.window->removeEventFilter(this);
    QObject::disconnect(window.btnConfirm, &QPushButton::clicked, this, &FormationHRI::onConfirm);
    QObject::disconnect(this, &FormationHRI::newFormationInfo, window.winTask, &DialogTask::newInfo);

    if (timerStat != -1) {
        killTimer(timerStat);
        timerStat = -1;
    }
    m_started = false;
    m_confirmed = false;
}

inline QString FormationHRI::idset2string() {
    QString info;
    for (int one: robotUsed) {
        if (info.isEmpty()) {
            info += QString("%1").arg(one);
        }
        else {
            info += QString(",%1").arg(one);
        }
    }
    return info;
}


inline bool FormationHRI::eventFilter(QObject *obj, QEvent *event) {
    if (event->type() == QEvent::KeyPress && !m_confirmed) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        int key = keyEvent->key();
        auto it = window.keyToRobot.find(key);
        if (it != window.keyToRobot.end()) {
            int id = it->second;
            auto it = robotUsed.find(id);
            if (it != robotUsed.end()) {
                robotUsed.erase(it);
            }
            else {
                robotUsed.insert(id);
            }
            window.status->showMessage(QString("Robot Used: %1").arg(idset2string()));
            return true;
        }
    }
    if (event->type() == QEvent::MouseButtonPress && !m_confirmed) {
        if (obj == window.graph && !robotUsed.empty()) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                auto pos = mouseEvent->pos();
                float x = window.graph->xAxis->pixelToCoord(pos.x());
                float y = window.graph->yAxis->pixelToCoord(pos.y());
                onNewPointsInGraph(x,y);
                window.status->showMessage(QString("New Points (%1, %2)").arg(int(x)).arg(int(y)));
                return true;
            }
            if (mouseEvent->button() == Qt::RightButton) {
                onRemovePointsInGraph();
                return true;
            }
        }
    }

    // Standard processing
    return QObject::eventFilter(obj, event);
}

inline void FormationHRI::onNewPointsInGraph(float x, float y) {
    if (pathLine == nullptr) {
        QPointF pos;
        int nRobot = 0;
        for (int id: robotUsed) {
            QPointF tmp;
            if (window.robot_position(id, tmp)) {
                ++nRobot;
                pos += tmp;
            }
        }
        if (nRobot == 0) return;
        pos /= nRobot;
        pathLine = window.mapitems->tempdata().add_curve({}, {});
        pathLine->setLineStyle(QCPCurve::LineStyle::lsLine);
        pathLine->addData(pos.x(), pos.y());
        pathLine->addData(x, y);

        return ;
    }
    pathLine->addData(x, y);
    // window.graph->replot();
}
inline void FormationHRI::onRemovePointsInGraph() {
    if (pathLine != nullptr) {
        int num = pathLine->data()->dataRange().size();
        if (num > 1) {
            double ss = pathLine->data()->at(num - 1)->sortKey();
            pathLine->data()->remove(ss);
        }
    }
}


inline void FormationHRI::onConfirm() {
    if (m_confirmed) return;
    if (robotUsed.size() < 2) {
        window.status->showMessage("To few robots selected");
        return;
    }
    window.status->showMessage(QString("Confirm Path Points for %1").arg(idset2string()));
    protobit::FORM form;
    form.type() = protobit::FORM::Basic;
    form.elen() = edgeLength;
    form.nrobot() = (int)robotUsed.size();
    form.fmgroup() = formationGroup;
    form.robot.clear();
    for (int id: robotUsed) {
        form.robot.push_back(id);
    }
    curveToProto(pathLine, form, window.coord(), pathVelocity);

    auto& client = window.bit_client->client;
    int nwrite = form.write((char*)buffer.c_str(), (int)buffer.size());
    int nsend = client.send(buffer.c_str(), nwrite);
    if (nwrite != nsend) {
        window.status->showMessage(QString("Warning: failed to send FORM"));
    }
    else {
        robotUsed_IDList.clear();
        for (int id: robotUsed) {
            robotUsed_IDList.push_back(id);
        }
        timerStat = startTimer(dt_stat, Qt::PreciseTimer);
        m_confirmed = true;
    }
}


inline void FormationHRI::timerEvent(QTimerEvent *event) {
    if (event->timerId() == timerStat && m_confirmed) {
        int nForm = (int)robotUsed.size();
        if (robotData.size() == robotUsed.size()) {
            currStat.resize(nForm);
            for (int i = 0; i < nForm; ++i) {
                int id = robotUsed_IDList[i];
                currStat.data[i] = robotData[id];
            }
            currStat.update();
        }
    }
}

inline void FormationHRI::onSTAT(const qSTAT& stat) {
    int id = stat.id();
    if (robotUsed.find(id) != robotUsed.end()) {
        auto& one = robotData[id];
        one.x  = (double)stat.x();
        one.y  = (double)stat.y();
        one.th = (double)stat.th();
    }
}

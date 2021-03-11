#pragma once
#include "winfunction.hpp"

/*
 * UI Logic:
 * 1. User left click to select a series of points
 * 2. User right click to cancel the lastest added point
 * IF m_noplan
 *    3. User click "confirm" again, send the path points to robots
 * ELSE
 *    3. User click "confirm", send MPLN request to robots (asking them to plan)
 *    4. Store received PATH from robots
 *    5. User click "confirm" again, send the received PATH to robots
*/
class TrackingHRI : public QObject
{
    Q_OBJECT
public:
    void onTaskBegin();
    void onTaskStop();
    bool started() const { return m_started; }

    bool eventFilter(QObject *watched, QEvent *event);
    void onNewPointsInGraph(float x, float y);
    void onRemovePointsInGraph();

public slots:
    void onConfirm();
    void onSTAT(const qSTAT& stat) {}
    void onPATH(const qPATH& path);

private:
    void onConfirmNoPlan();
    void onConfirmDoPlan();

public:
    bool m_noplan = false;
    bool m_started = false;
    bool m_requested = false;
    WindowFunction window;

    int currRobotID = -1;
    int pathVelocity = 40;
    int safeDistance = 100;
    QCPCurve* line = nullptr;
    std::string buffer;

    std::map<int, qPATH> received;
    std::map<int, int> robotToKey;
    std::map<int, QCPCurve*> robotLines;
};

inline void TrackingHRI::onTaskBegin() {
    if (m_started) return;

    // Test If Has Required Items
    if (!window.window) return;
    if (!window.graph) return;
    if (!window.btnConfirm) return;
    if (!window.robot_position) return;
    if (!m_noplan && !window.bit_client) return;

    buffer.clear();
    buffer.resize(3000, '\0');
    if (!robotLines.empty()) {
        for (auto& one: robotLines) {
            window.graph->removePlottable(one.second);
        }
        robotLines.clear();
    }
    line = nullptr;
    currRobotID = -1;

    robotToKey.clear();
    for (auto& one: window.keyToRobot) {
        int key = one.first;
        int id = one.second;
        robotToKey[id] = key;
    }

    // Connect Events
    window.graph->installEventFilter(this);
    window.window->installEventFilter(this);
    QObject::connect(window.btnConfirm, &QPushButton::clicked, this, &TrackingHRI::onConfirm);
    if (!m_noplan) {
        QObject::connect(window.bit_client, &QBITClient::newPATH, this, &TrackingHRI::onPATH);
        received.clear();
    }

    m_requested = false;
    m_started = true;
}

inline void TrackingHRI::onTaskStop() {
    if (!m_started) return;
    for (auto one: robotLines) {
        window.mapitems->tempdata().remove(one.second);
    }
    robotLines.clear();
    currRobotID = -1;

    window.graph->removeEventFilter(this);
    window.window->removeEventFilter(this);
    QObject::disconnect(window.btnConfirm, &QPushButton::clicked, this, &TrackingHRI::onConfirm);
    if (!m_noplan) {
        QObject::disconnect(window.bit_client, &QBITClient::newPATH, this, &TrackingHRI::onPATH);
    }

    m_started = false;
}

inline bool TrackingHRI::eventFilter(QObject *obj, QEvent *event) {
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        int key = keyEvent->key();
        auto it = window.keyToRobot.find(key);
        if (it != window.keyToRobot.end()) {
            currRobotID = it->second;
            line = robotLines[currRobotID];
            window.status->showMessage(QString("Setting Path For R%1").arg(currRobotID));
            return true;
        }
        if (key == window.keyCancelSelect && currRobotID != -1) {
            currRobotID = -1;
            window.status->showMessage(QString("Setting Path For None"));
            return true;
        }
    }
    if (event->type() == QEvent::MouseButtonPress) {
        if (obj == window.graph && currRobotID != -1) {
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

inline void TrackingHRI::onNewPointsInGraph(float x, float y) {
    if (currRobotID == -1) return;
    if (line == nullptr) {
        QPointF pos;
        if (window.robot_position(currRobotID, pos)) {
            line = window.mapitems->tempdata().add_curve({}, {});
            line->setLineStyle(QCPCurve::LineStyle::lsLine);
            line->addData(pos.x(), pos.y());
            line->addData(x, y);
            robotLines[currRobotID] = line;
        }
        return ;
    }
    line->addData(x, y);
    // window.graph->replot();
}
inline void TrackingHRI::onRemovePointsInGraph() {
    if (currRobotID == -1) return;
    if (line != nullptr) {
        int num = line->data()->dataRange().size();
        if (num > 1) {
            double ss = line->data()->at(num - 1)->sortKey();
            line->data()->remove(ss);
        }
    }
}
inline void TrackingHRI::onConfirm() {
    if (m_noplan) onConfirmNoPlan();
    else onConfirmDoPlan();
    currRobotID = -1;
}



inline void TrackingHRI::onConfirmNoPlan() {
    window.status->showMessage("Confirm Path Points (No Plan)");
    auto& client = window.bit_client->client;
    for (const auto& idline: robotLines) {
        int ID = idline.first;
        auto one = idline.second;
        if (one && one->dataCount() > 1) {
            protobit::PATH path;
            curveToProto(one, path, window.coord(), pathVelocity);
            path.id = ID;

            int nwrite = path.write((char*)buffer.c_str(), (int)buffer.size());
            int nsend = client.send(buffer.c_str(), nwrite);
            if (nwrite != nsend) {
                window.status->showMessage(QString("Warning: failed to send PATH"));
            }
        }
    }
}
inline void TrackingHRI::onConfirmDoPlan() {
    auto& client = window.bit_client->client;
    if (!m_requested) {
        // Send Path Planning Requests To Robots
        for (auto& idline: robotLines) {
            int ID = idline.first;
            auto one = idline.second;
            if (one && one->dataCount() > 1) {
                protobit::MPLN path;
                curveToProto(one, path, window.coord(), pathVelocity);
                path.id = ID;
                path.type = protobit::MPLN::Basic;
                path.d = safeDistance;

                int nwrite = path.write((char*)buffer.c_str(), (int)buffer.size());
                int nsend = client.send(buffer.c_str(), nwrite);
                if (nwrite != nsend) {
                    window.status->showMessage(QString("Warning: failed to send MPLN"));
                }
            }
        }

        m_requested = true;
    }
    else {
        // Send Confirmed Path To Robots
        for (auto& one: received) {
            qPATH& path = one.second;
            int nwrite = path.write((char*)buffer.c_str(), buffer.size());
            int nsend = client.send(buffer.c_str(), nwrite);
            if (nwrite != nsend) {
                window.status->showMessage(QString("Warning: failed to send PATH"));
            }
        }
    }
}

inline void TrackingHRI::onPATH(const qPATH& path) {
    int id = path.id();
    received[id] = path;
}

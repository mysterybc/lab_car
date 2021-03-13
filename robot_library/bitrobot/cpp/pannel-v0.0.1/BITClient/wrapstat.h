#pragma once
#include <QObject>
#include <QTimerEvent>
#include <ssnet/sstcp_protobit.hpp>
#include <map>

#define __declare_wrap(name) \
    struct q##name: public protobit::name { \
        q##name(){} \
        q##name(const protobit::name& dat): protobit::name(dat) {} \
    }

__declare_wrap(STAT);
__declare_wrap(PATH);
__declare_wrap(ALGO);

Q_DECLARE_METATYPE(qSTAT);
Q_DECLARE_METATYPE(qPATH);
Q_DECLARE_METATYPE(qALGO);
inline void init_customtypes_main() {
    qRegisterMetaType<qSTAT>();
    qRegisterMetaType<qPATH>();
    qRegisterMetaType<qALGO>();
}



template<class T>
struct TimedItem {
    T& operator() () { return data; }
    const T& operator() () const { return data; }

    void newdata(const T& newData) {
        data = newData; handled = false; tick = 0;
    }
    void update() {
        tick = tick < tick_max ? tick + 1 : tick_max;
    }
    void update(bool is_handled) {
        update();
        handled = is_handled;
    }

    T data;
    bool handled = true;
    int  tick = 10000;
    int  tick_max = 10000;
};


struct TimedInfo {
    TimedItem<protobit::STAT> stat;
    TimedItem<protobit::ALGO> algo;
    TimedItem<protobit::PATH> path;
};


class QBITHandler: public protobit::DataHandler {
public:
    void onSTAT(protobit::STAT& data, SendBuffer&) {
        if (data.valid()) {
            received[data.id()].stat.newdata(data);
        }
    }
    void onALGO(protobit::ALGO &data, SendBuffer&) {
        if (data.valid()) {
            received[data.id()].algo.newdata(data);
        }
    }
    void onPATH(protobit::PATH& data, SendBuffer&) {
        if (data.valid()) {
            received[data.id()].path.newdata(data);
        }
    }
    std::map<int, TimedInfo> received;
};

class QBITClient: public QObject {
    Q_OBJECT
public:
    QBITClient(QObject* parent = 0): QObject(parent) {}
    void connect(NetworkAddress addr, int checkInterval = 20) {
        onClient.received.clear();
        client.onData = &onClient;
        client.setsocketopt_rcvbuf(5000);
        client.setsocketopt_sndbuf(5000);
        client.connect(addr);
        if (tm_check != -1) killTimer(tm_check);
        tm_check = startTimer(checkInterval, Qt::PreciseTimer);
    }
    bool connected() {
        return client.is_connected();
    }

signals:
    void messageInfo(const QString& msg);
    void newSTAT(const qSTAT& stat);
    void newPATH(const qPATH& path);
    void newALGO(const qALGO& algo);

protected:
    void timerEvent(QTimerEvent *event) override {
        if (event->timerId() == tm_check) {
            if (client.is_connected()) {
                client.recv_pending();
                for (auto& one: onClient.received) {
                    if (!one.second.stat.handled) {
                        emit newSTAT(one.second.stat());
                        one.second.stat.update(true);
                    }
                    if (!one.second.algo.handled) {
                        emit newALGO(one.second.algo());
                        one.second.algo.update(true);
                    }
                    if (!one.second.path.handled) {
                        emit newPATH(one.second.path());
                        one.second.path.update(true);
                    }
                }
            }
            else {
                emit messageInfo("Not Connected");
            }
        }
    }

public:
    int tm_check = -1;
    QBITHandler onClient;
    protobit::BITClient client;
};



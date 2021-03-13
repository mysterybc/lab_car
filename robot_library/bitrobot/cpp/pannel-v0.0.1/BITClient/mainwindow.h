#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "gamepad.hpp"
#include "items/robotitem.hpp"
#include "tasks/tracking.hpp"
#include "tasks/formation.hpp"
#include "graphcontrol.hpp"
#include "mapitems.hpp"
#include "wrapstat.h"
#include <ssconfig.hpp>
#include <dialogstatus.h>
#include <dialogtask.h>
#include <ssnet/sstcp_protobit.hpp>
#include <QMainWindow>
#include <QCloseEvent>
#include <QMap>
#include <memory>


using protobit::BITServer;
using protobit::BITClient;
using BITHandler = protobit::DataHandler;
using pServer = std::shared_ptr<BITServer>;
using pClient = std::shared_ptr<BITClient>;
using pHandler = std::shared_ptr<BITHandler>;
using protobit::STAT;
using protobit::COMD;
using protobit::TURN;
using protobit::PATH;
using protobit::OBST;
using protobit::TELE;
using protobit::TALG;
using protobit::TDEG;
using protobit::ALGO;
using protobit::PLAN;
using CompoundRobot = DItems::CompoundRobot;
using pCompoundRobot = std::shared_ptr<DItems::CompoundRobot>;
using pBIT = std::shared_ptr<QBITClient>;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


struct RobotIP {
    std::string ip;
    int port;
    int robotID;
};


template<class T>
struct TimedData {
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
struct ExSTAT {
    TimedData<STAT> stat;
    TimedData<ALGO> algo;
    TimedData<PATH> path;
};

class ClientHandler: public BITHandler {
public:
    void onSTAT(STAT& data, SendBuffer&) {
        if (data.valid()) {
            received[data.id()].stat.newdata(data);
        }
    }
    void onALGO(ALGO &data, SendBuffer&) {
        if (data.valid()) {
            received[data.id()].algo.newdata(data);
        }
    }
    void onPATH(PATH& data, SendBuffer&) {
        if (data.valid()) {
            received[data.id()].path.newdata(data);
        }
    }
    std::map<int, ExSTAT> received;
};


struct ServerInfo {
    std::string ip;
    int port = 0;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void connect_server(NetworkAddress ip_port);
    void connect_server() {
        connect_server(server_addr);
    }

    void update_display();
    void check_and_recv();
    const CoordTrans& coord() const { return currCoord;  }

    void plot_map(const std::string& fname);
    void configHRI(sscfg::ConfigFile& config);

public slots:
    void onNewSTAT(const qSTAT& stat);
    void onNewPATH(const qPATH& path);

signals:
    void newSTAT(const qSTAT& state);
    void newPATH(const qPATH& path);

protected:
    void timerEvent(QTimerEvent *event);
    void onFirstPlot();
	void resizeEvent(QResizeEvent *event);
    void closeEvent(QCloseEvent *event);
    int sendDataToServer(const char* data, size_t size);

private slots:
    void on_btnClear_clicked();
    void on_cbNED_stateChanged(int arg1);
    void on_btnConnect_clicked();
    void on_btnTracking_clicked();
    void on_btnStop_clicked();
    void on_btnSendUser_clicked();
    void on_btnMap_clicked();
    void on_coord_changed(CoordTrans newCoord);
    void on_btnInfo_clicked();

    void on_btnTask_clicked();

    void on_btnForm_clicked();

private:
    CoordTrans currCoord;
    Ui::MainWindow *ui;
    NetworkAddress server_addr;
    ClientHandler onClient;
    pClient client;

    bool use_bitclient;
    QBITClient bit_client;

    int tm_check;
    int tm_plot;
    bool plot_inited = false;    
    std::map<int, CompoundRobot*> robots;
    MapItems mapitems;
    WindowFunction myfunc;
    TrackingHRI trackingHRI;
    FormationHRI formationHRI;
    protobit::PackList parsers;

    DialogStatus* diag;
    DialogTask* diagTask;
public:
    std::string default_mapfile;
};
#endif // MAINWINDOW_H

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCheckBox>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <ssnet/sstcp_protobit.hpp>
#include <memory>
#include "gamepad.hpp"

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
using protobit::FORM;
using protobit::EIGT;
using protobit::ALGO;
using protobit::PLAN;
using protobit::MPLN;
using pDataAlgo = std::shared_ptr<ALGO>;

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

struct ClientHandler: BITHandler {
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

class MainWindow;
struct ServerHandler: BITHandler {
    ServerHandler(MainWindow* window): window(window), buffer(3000, '\0') {}
    void onCOMD(COMD& data, SendBuffer&);
    void onPATH(PATH& data, SendBuffer&);
    void onTURN(TURN& data, SendBuffer&);
    void onOBST(OBST& data, SendBuffer&);
    void onTELE(TELE& data, SendBuffer&);
    void onTALG(TALG& data, SendBuffer&);
    void onTDEG(TDEG& data, SendBuffer&);
    void onFORM(FORM& data, SendBuffer&);
    void onEIGT(EIGT& data, SendBuffer&);
    void onPLAN(PLAN& data, SendBuffer&);
    void onMPLN(MPLN& data, SendBuffer&);

    MainWindow* window;
    std::string buffer;
};

struct ClientInfo {
    int robotID = 0;
    int groupIndex = 0;

    std::string ip;
    int port = 0;
    pClient client = nullptr;

    QCheckBox* cbox = nullptr;
    QLineEdit* lineEdit = nullptr;
    void reconnect(BITHandler* onClient);
};

using OneGroup = std::vector<int>;
using GroupList = std::vector<OneGroup>;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    friend struct ServerHandler;
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void init_server(const std::vector<RobotIP>& topo, GroupList gpList);
    void pub_state();
    void check_and_accept();
    void recv_info();
    void check_js_tele();

    bool is_selected(int ID);
    void log_packinfo(PackBase& pack);
    void log_senderror(PackBase& pack, const std::string& msg, int nwrite, int nsend);

protected:
    void timerEvent(QTimerEvent *event);
    void update_display();

private slots:
    void on_btnClear_clicked();
    void on_btnConnect_clicked();
    void on_btnPause_clicked();
    void on_btnResume_clicked();
    void on_btnStop_clicked();
    void on_btnTele_clicked();
    void on_btnStopAll_clicked();
    void on_cbJoy_clicked();

    void on_btnLogStart_clicked();
    void on_btnLogStop_clicked();
	void on_btnReload_clicked();

private:
    Ui::MainWindow *ui;
    pServer server;
    std::map<int, ClientInfo> client;
    GroupList groupInfo;
    ServerHandler onServer;
    ClientHandler onClient;
    int timer_snd;
    int timer_rcv;
    int timer_chk;
    int timer_plt;
    int timer_js;
    FILE* memfile;
    gamepad::Joystick js;
    std::string buffer;

    bool tele_enabled;
};
#endif // MAINWINDOW_H

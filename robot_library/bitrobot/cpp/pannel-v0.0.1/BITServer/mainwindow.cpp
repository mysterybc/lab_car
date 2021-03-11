#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
#include <cstdio>
#include <set>

FILE *open_memfile(const char* fname, const char *mode);

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow), onServer(this)
{
    ui->setupUi(this);
    sstcp_initialization();
    server = nullptr;

    memfile = open_memfile("bitserver.log.tmp", "wt+");
    timer_js  = 0;
    timer_chk = 0;
    timer_rcv = 0;
    timer_snd = 0;
    timer_plt = 0;

    this->setWindowTitle(QString("无人车平台指控终端服务器"));
}

MainWindow::~MainWindow()
{
    delete ui;
    fclose(memfile);
    server = nullptr;
    for (auto& one: client) {
        one.second.client = nullptr;
    }

    sstcp_cleanup();
}

void ClientInfo::reconnect(BITHandler* onClient) {
    client = std::make_shared<BITClient>();
    client->setsocketopt_rcvbuf(1000);
    client->setsocketopt_sndbuf(1000);
    client->onData = onClient;
    client->connect(NetworkAddress(ip.c_str(), port));
}

void MainWindow::init_server(const std::vector<RobotIP>& topo, GroupList gpList) {
    QFont ftConsolas12;
    ftConsolas12.setFamily("Consolas");
    ftConsolas12.setPointSize(12);

    std::set<int> idall;
    for (auto& one: gpList) {
        for (int id: one) idall.insert(id);
    }
    bool all_connect = gpList.empty();

    for (auto& one: topo) {
        if (one.robotID == 0) {
            // This is bitserver
            server = std::make_shared<BITServer>();
            server->setsocketopt_rcvbuf(1000);
            server->setsocketopt_sndbuf(1000);
            server->onData = &onServer;
            int ret = server->bind_and_listen(NetworkAddress(one.ip.c_str(), one.port));
            if (ret != 0) {
                qDebug() << QString("Fail to bind server on %1:%2").arg(one.ip.c_str()).arg(one.port);
                exit(-1);
            }
            setWindowTitle(QString("无人车平台指控终端服务器 %1:%2").arg(one.ip.c_str()).arg(one.port));
        }
        else if (all_connect || idall.find(one.robotID) != idall.end()) {
            // This is a robot
            auto& it = client[one.robotID];
            it.ip = one.ip;
            it.port = one.port;
            it.robotID = one.robotID;
            it.groupIndex = 0;
            it.reconnect(&onClient);
            ui->tb->append(QString("连接无人车 %1, %2:%3").arg(one.robotID).arg(one.ip.c_str()).arg(one.port));
        }
    }
    groupInfo = gpList;
    for (int k=0;k<(int)gpList.size();++k) {
        for (int id: gpList[k]) {
            auto it = client.find(id);
            if (it != client.end()) {
                it->second.groupIndex = k;
            }
        }
    }

    // Initialize UI Components
    for (auto& one: client) {
        auto& it = one.second;
        it.cbox = new QCheckBox(this);
        it.cbox->setText(QString("R%1").arg(it.robotID));
        it.cbox->setStyle(ui->cbAll->style());
        it.cbox->setFont(ui->cbAll->font());
        it.cbox->setSizePolicy(ui->cbAll->sizePolicy());
        ui->hzLayoutCheckRobot->addWidget(it.cbox);

        QLabel* lab = new QLabel(this); // This is not important, we will not store it
        lab->setText(QString("车%1/组%2").arg(one.first).arg(it.groupIndex));
        lab->setFont(ftConsolas12);

        it.lineEdit = new QLineEdit(this);
        it.lineEdit->setReadOnly(true);
        it.lineEdit->setFont(ftConsolas12);
        ui->fmLayoutRobot->addRow(lab, it.lineEdit);
    }


    timer_snd = startTimer(20, Qt::PreciseTimer);
    timer_rcv = startTimer(20, Qt::PreciseTimer);
    timer_chk = startTimer(100, Qt::CoarseTimer);
    timer_plt = startTimer(100, Qt::CoarseTimer);
    ui->tb->document()->setMaximumBlockCount(100);
}

void MainWindow::pub_state() {
    buffer.resize(2048, '\0');

    if (server) {
        for (auto& one: onClient.received) {
            bool send_stat = !one.second.stat.handled;
            bool send_path = !one.second.path.handled;
            if (send_stat) {
                int nwrite = one.second.stat().write((char*)buffer.c_str(), (int)buffer.size());
                server->sendtoall((char*)buffer.c_str(), nwrite);
            }
            if (send_path) {
                int nwrite = one.second.path().write((char*)buffer.c_str(), (int)buffer.size());
                server->sendtoall((char*)buffer.c_str(), nwrite);

                fprintf(memfile, "Get PLAN of Robot-%d:\n", one.first);
                one.second.path().print_data(memfile);
            }
        }
    }


    for (auto& one: onClient.received) {
        bool send_stat = !one.second.stat.handled;
        bool send_algo = !one.second.algo.handled;

        int srcID = one.first;
        int srcGP = client[srcID].groupIndex;
        STAT& stat = one.second.stat();
        ALGO& algo = one.second.algo();

        for (auto& cc: client) {
            if (cc.second.client && cc.second.groupIndex != srcGP) {
                auto& one = cc.second.client;
                int nwrite = 0;
                if (send_stat) {
                    nwrite = stat.write((char*)buffer.c_str(), (int)buffer.size());
                    int nsend = one->send((char*)buffer.c_str(), nwrite);
                    if (nwrite != nsend) {
                        qDebug() << "Failed to send STAT of rob" << srcID << " to " << cc.second.robotID;
                    }
                }
                if (send_algo) {
                    nwrite = algo.write((char*)buffer.c_str(), (int)buffer.size());
                    int nsend = one->send((char*)buffer.c_str(), nwrite);
                    if (nwrite != nsend) {
                        qDebug() << "Failed to send ALGO of rob" << srcID << " to " << cc.second.robotID;
                    }
                }
            }
        }
    }

    for (auto& one: onClient.received) {
        one.second.stat.update(true);
        one.second.algo.update(true);
        one.second.path.update(true);
    }
}

void MainWindow::check_and_accept() {
    if (server) {
        server->remove_unreachable();
        server->accept_and_recv();
    }
}

void MainWindow::recv_info() {
    for (auto& one: client) {
        auto cc = one.second.client;
        if (cc && cc->is_connected()) {
            cc->recv_pending();
        }
    }
    if (server)
        server->accept_and_recv();
}

void MainWindow::check_js_tele() {
    if (!js.connected()) js.connect(0);
    if (!js.connected()) js.connect(1);
    if (js.read() >= 0) {
        int id = -1;

        QString btn;
        TELE tele;
        TALG talg;
        if (js.A()) { id = 3; btn = "A Pressed";}
        if (js.B()) { id = 1; btn = "B Pressed";}
        if (js.X()) { id = 4; btn = "X Pressed";}
        if (js.Y()) { id = 2; btn = "Y Pressed";}
        if (id != -1) {
            auto xy = js.AxisLS();
            float v = float(xy.second * 50); //  0.5m/s max
            float w = float(xy.first * 40);   // 90deg/s max
            float vx = float(xy.first * 50); //  0.5m/s max
            float vy = float(xy.second * 50); //  0.5m/s max
            if (std::abs(xy.second) < 0.1) { v = 0; vy = 0; }
            if (std::abs(xy.first) < 0.1) { w = 0; vx = 0; }

            if (ui->cbTeleAlg->isChecked()) {
                talg.id = id;
                if (ui->cbTeleXY->isChecked())
                    talg.set_vxy(vx, vy);
                else
                    talg.set_vw(v, w);
            }
            else {
                tele.id = id;
                if (ui->cbTeleXY->isChecked()) {
                    tele.type = TELE::ByAlgoVxVy;
                    tele.v = int(vx);
                    tele.w = int(vy);
                }
                else {
                    tele.type = TELE::Direct;
                    tele.v = int(v);
                    tele.w = int(w);
                }
            }

            std::string buffer(128, '\0');
            int nwrite = 0;
            if (ui->cbTeleAlg->isChecked()) {
                nwrite = talg.write((char*)buffer.c_str(), (int)buffer.size());
            }
            else {
                nwrite = tele.write((char*)buffer.c_str(), (int)buffer.size());
            }

            auto it = client.find(id);
            if (it != client.end()) {
                auto& cc = it->second.client;
                if (cc && nwrite > 0) {
                    int nsend = cc->send(buffer.c_str(), nwrite);
                    if (nsend != nwrite) {
                        if (ui->cbTeleAlg->isChecked()) {
                            log_senderror(talg, buffer, nwrite, nsend);
                        }
                        else {
                            log_senderror(tele, buffer, nwrite, nsend);
                        }
                    }
					else {
						ui->statusbar->showMessage(QString::fromStdString(std::string(buffer.c_str(), nwrite)), 100);
					}
                }
            }
        }
        if (ui->cbJoy->isChecked()) {
            QString info;
            if (ui->cbTeleAlg->isChecked()) {
                info = QString("Joystick TAlg: %1, tele: id %2, v1 %3, v2 %4")
                        .arg(btn).arg(talg.id()).arg(talg.v1()).arg(talg.v2());
            }
            else {
                info = QString("Joystick Tele: %1, tele: id %2, v %3, w %4")
                        .arg(btn).arg(tele.id()).arg(tele.v()).arg(tele.w());
            }
            ui->tb->append(info);
        }
    }
}

void MainWindow::timerEvent(QTimerEvent *event) {
    int id = event->timerId();
    if (id == timer_chk) return check_and_accept();
    if (id == timer_rcv) return recv_info();
    if (id == timer_snd) return pub_state();
    if (id == timer_js)  return check_js_tele();
    if (id == timer_plt) return update_display();
}

template<class PackT>
void onPackBIT(PackT& data, MainWindow& window, std::map<int, ClientInfo>& client, std::string& buffer) {
    window.log_packinfo(data);
    if (!data.valid()) return ;
    int id = data.id();
    auto it = client.find(id);
    if (it != client.end()) {
        auto& cc = it->second;
        if (cc.client) {
            int nwrite = data.write((char*)buffer.data(), (int)buffer.size());
            int nsend = cc.client->send((char*)buffer.data(), nwrite);
            if (nsend != nwrite) {
                window.log_senderror(data, buffer, nwrite, nsend);
            }
        }
    }
}
template<class PackT>
void onPackBIT_batch(const std::vector<protobit::ID>& id, PackT& data, MainWindow& window, std::map<int, ClientInfo>& client, std::string& buffer) {
    window.log_packinfo(data);
    if (!data.valid()) return ;
    for (unsigned k = 0;k<id.size(); ++k) {
        auto it = client.find(id[k]());
        if (it != client.end()) {
            auto& cc = it->second;
            if (cc.client) {
                int nwrite = data.write((char*)buffer.data(), (int)buffer.size());
                int nsend = cc.client->send((char*)buffer.data(), nwrite);
                if (nsend != nwrite) {
                    window.log_senderror(data, buffer, nwrite, nsend);
                }
            }
        }
    }
}


void ServerHandler::onCOMD(COMD& data, SendBuffer&) {
    return onPackBIT(data, *window, window->client, buffer);
}
void ServerHandler::onPATH(PATH& data, SendBuffer&) {
    return onPackBIT(data, *window, window->client, buffer);
}
void ServerHandler::onTURN(TURN& data, SendBuffer&) {
    return onPackBIT(data, *window, window->client, buffer);
}
void ServerHandler::onOBST(OBST& data, SendBuffer&) {
    return onPackBIT(data, *window, window->client, buffer);
}
void ServerHandler::onTELE(TELE& data, SendBuffer&) {
    return onPackBIT(data, *window, window->client, buffer);
}
void ServerHandler::onTALG(TALG& data, SendBuffer&) {
    return onPackBIT(data, *window, window->client, buffer);
}
void ServerHandler::onTDEG(TDEG& data, SendBuffer&) {
    return onPackBIT(data, *window, window->client, buffer);
}
void ServerHandler::onPLAN(PLAN& data, SendBuffer&) {
    return onPackBIT(data, *window, window->client, buffer);
}
void ServerHandler::onMPLN(MPLN& data, SendBuffer&) {
    return onPackBIT(data, *window, window->client, buffer);
}
void ServerHandler::onFORM(FORM &data, SendBuffer&) {
    return onPackBIT_batch(data.robot, data, *window, window->client, buffer);
}
void ServerHandler::onEIGT(EIGT &data, SendBuffer&) {
    return onPackBIT_batch(data.ids, data, *window, window->client, buffer);
}


/*
 * GUI Related
 */
void MainWindow::log_packinfo(PackBase& pack) {
    pack.print_data(memfile, "");
}

void MainWindow::log_senderror(PackBase& pack, const std::string& msg, int nwrite, int nsend) {
    QString info("[Failed] nsend %1, nwrite %2, type %3\n[Failed] %4\n");
    ui->tb->append(info.arg(nsend).arg(nwrite).arg(pack.pack_name()).arg(msg.c_str()));
}

void MainWindow::update_display() {
    // Update Information Received From Clients
    rewind(memfile);
    static char buf[512] = {0};
    while(fgets(buf, 512, memfile)) {
        ui->tb->append(QString::fromLocal8Bit(buf));
    }

    fclose(memfile);
    memfile = open_memfile("bitserver.log.tmp", "wt+");

    // Update Robot Status
    for (auto& one: client) {
        ClientInfo& robdata = one.second;
        if (!robdata.lineEdit) continue;
        if (robdata.client) {
            auto& cc = robdata.client;
            QString info;
            if (cc->is_connected()) info.append("[y]");
            else info.append("[n]");
            auto pstat = onClient.received.find(robdata.robotID);
            if (pstat == onClient.received.end()) {
                info.append(" 无数据");
            }
            else {
                auto& rec = pstat->second;
                QString data;
                data = QString("[%9] x %1, y %2, th %3, v %4, w %5, tp %6, st %7, pg %8")
                        .arg(rec.stat().x(), 5)
                        .arg(rec.stat().y(), 5)
                        .arg(rec.stat().th(), 4)
                        .arg(rec.stat().v(), 5)
                        .arg(rec.stat().w(), 4)
                        .arg(rec.stat().type(), 2)
                        .arg(rec.stat().status(), 2)
                        .arg(rec.stat().progress(), 3)
                        .arg((rec.stat.tick/2)*2);
                info.append(data);
            }
            robdata.lineEdit->setText(info);
        }
        else {
            robdata.lineEdit->setText("[n] 无数据 (未连接)");
        }
    }
}

bool MainWindow::is_selected(int ID) {
    if (ui->cbAll->isChecked()) return true;
    if (ID == -1) return ui->cbAll->isChecked();
    if (client[ID].cbox) {
        return client[ID].cbox->isChecked();
    }
    return false;
}
void MainWindow::on_btnClear_clicked() {
    ui->tb->clear();
    for (auto& one: onClient.received) {
        int id = one.first;
        int gp = client[id].groupIndex;
        QString info("R-%1, GP-%2, LagStat-%3, LagAlgo-%4");
        ui->tb->append(info.arg(id).arg(gp).arg(one.second.stat.tick).arg(one.second.algo.tick));
    }
}
void MainWindow::on_btnConnect_clicked() {
    for (auto& one: client) {
        one.second.reconnect(&onClient);
    }
}

template<class T>
void on_send_pack(T& pack, std::map<int, ClientInfo>& client, MainWindow* window, bool toAll = false) {
    std::string buffer(128, '\0');
    for (auto one: client) {
        auto& cc = one.second;
        if ((window->is_selected(cc.robotID) || toAll) && cc.client) {
            pack.id = cc.robotID;
            int nwrite = pack.write((char*)buffer.c_str(), (int)buffer.size());
            int nsend = cc.client->send(buffer.c_str(), nwrite);
            if (nwrite != nsend) {
                window->log_senderror(pack, buffer, nwrite, nsend);
            }
        }
    }
}

void MainWindow::on_btnPause_clicked() {
    COMD cmd;
    cmd.cmd = COMD::Pause;
    on_send_pack(cmd, client, this);
}

void MainWindow::on_btnResume_clicked() {
    COMD cmd;
    cmd.cmd = COMD::Resume;
    on_send_pack(cmd, client, this);
}

void MainWindow::on_btnStop_clicked() {
    COMD cmd;
    cmd.cmd = COMD::Stop;
    on_send_pack(cmd, client, this);
}
void MainWindow::on_btnStopAll_clicked() {
    COMD cmd;
    cmd.cmd = COMD::Stop;
    on_send_pack(cmd, client, this, true);
    if (timer_js != 0) {
        killTimer(timer_js);
        timer_js = 0;
    }
}

void MainWindow::on_btnTele_clicked() {
    if (!js.connected()) js.connect();
    if (timer_js == 0) {
        timer_js = startTimer(50, Qt::PreciseTimer);
        ui->btnTele->setText("停止遥控");
    }
    else {
        killTimer(timer_js);
        timer_js = 0;
        ui->btnTele->setText("遥控");
        COMD cmd;
        cmd.cmd = COMD::Stop;
        on_send_pack(cmd, client, this);
    }
}
void MainWindow::on_cbJoy_clicked() {

}

void MainWindow::on_btnLogStart_clicked() {
    COMD cmd;
    cmd.cmd = COMD::LogTraceBegin;
    on_send_pack(cmd, client, this);
}

void MainWindow::on_btnLogStop_clicked() {
    COMD cmd;
    cmd.cmd = COMD::LogTraceStop;
    on_send_pack(cmd, client, this);
}

void MainWindow::on_btnReload_clicked() {
	COMD cmd;
	cmd.cmd = COMD::ConfigReload;
	on_send_pack(cmd, client, this);
}

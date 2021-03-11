#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMouseEvent>
#include <QKeyEvent>


void MainWindow::configHRI(sscfg::ConfigFile& config) {
    config.get("noplan", trackingHRI.m_noplan);
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("嵌入式仿真指控终端");


    use_bitclient = true;

    if (!use_bitclient) {
        tm_check = startTimer(20, Qt::PreciseTimer);
        tm_plot = startTimer(20, Qt::PreciseTimer);
    }
    else {
        tm_plot = startTimer(50, Qt::CoarseTimer);
    }

    mapitems = MapItems(ui->graph, currCoord);

    myfunc.window = this;
    myfunc.graph = ui->graph;
    myfunc.mapitems = &mapitems;
    myfunc.winTask = diagTask;
    myfunc.status = ui->statusbar;
    myfunc.coord = [this](){ return this->coord(); };
    myfunc.btnConfirm = ui->btnConfirm;
    myfunc.bit_client = use_bitclient ? (&this->bit_client) : nullptr;
    myfunc.robot_position = [this](int k, QPointF& pos)->bool {
        auto it = this->robots.find(k);
        if (it != this->robots.end()) {
            pos = it->second->mainBody()->position();
            return true;
        }
        return false;
    };


    myfunc.keyToRobot.clear();
    myfunc.keyToRobot[Qt::Key_1] = 1;
    myfunc.keyToRobot[Qt::Key_2] = 2;
    myfunc.keyToRobot[Qt::Key_3] = 3;
    myfunc.keyToRobot[Qt::Key_4] = 4;
    myfunc.keyToRobot[Qt::Key_5] = 5;
    myfunc.keyToRobot[Qt::Key_6] = 6;
    myfunc.keyToRobot[Qt::Key_7] = 7;
    myfunc.keyToRobot[Qt::Key_8] = 8;
    myfunc.keyToRobot[Qt::Key_9] = 9;
    myfunc.keyCancelSelect = Qt::Key_Escape;


    trackingHRI.window = myfunc;
    formationHRI.window = myfunc;

    diag = new DialogStatus();
    diag->hide();
    if (!use_bitclient) {
        QObject::connect(this, &MainWindow::newSTAT, diag->data, &MyModel::onNewSTAT);
    }
    else {
        QObject::connect(&bit_client, &QBITClient::newSTAT, diag->data, &MyModel::onNewSTAT);
        QObject::connect(&bit_client, &QBITClient::newSTAT, this, &MainWindow::onNewSTAT);
        QObject::connect(&bit_client, &QBITClient::newPATH, this, &MainWindow::onNewPATH);
    }
    diagTask = new DialogTask;
    diagTask->hide();
}

MainWindow::~MainWindow() {
    client = nullptr;
    sstcp_cleanup();
    for (auto one: robots) {
        if (one.second) {
            one.second->remove();
            ui->graph->removeItem(one.second);
            one.second = nullptr;
        }
    }
    delete ui;
}

void MainWindow::connect_server(NetworkAddress ip_port) {
    server_addr = ip_port;

    if (!use_bitclient) {
        // old interface
        client = std::make_shared<BITClient>();
        client->onData = &onClient;
        client->connect(server_addr);
    }
    else{
        // new interface
        bit_client.connect(server_addr);
    }

    QString addr_str = QString::fromStdString(ip_port.get_addr_string());
    this->setWindowTitle(QString("嵌入式仿真指控终端 目标地址 %1").arg(addr_str));
}
void MainWindow::timerEvent(QTimerEvent *event) {
    int tm = event->timerId();
    if (tm == tm_check && !use_bitclient) check_and_recv();
    if (tm == tm_plot) {
        if (!plot_inited) onFirstPlot();
        else update_display();
    }
}
void MainWindow::check_and_recv() {
    if (!client) return;
    if (client->is_connected())
        client->recv_pending();
}
CompoundRobot* create_newrobot(QCustomPlot* graph, int id) {
    CompoundRobot* pRobot = new CompoundRobot(graph);
    pRobot->initialize();
    pRobot->withHeading(true);
    pRobot->withTrajectory(true);
    pRobot->withLabel(true);
    pRobot->setName(QString("Robot-%1").arg(id), 12);
    pRobot->setShape(45.0f, 40.0f, DItems::CompoundRobot::ShapeRectangle);
    return pRobot;
}
void MainWindow::onFirstPlot() {
    // You cannot do this in the constructor since
    // the window has not been created, the dimensions are wrong
    auto graph = ui->graph;
    graph->xAxis->setRange(-1000, 1000);
    graph->yAxis->setScaleRatio(ui->graph->xAxis, 1.0);
    QCP::Interactions act = QCP::Interaction::iRangeDrag | QCP::Interaction::iRangeZoom;
    graph->setInteractions(act);
    on_btnMap_clicked();  // Load map on start
    graph->replot();
    plot_inited = true;
}
void MainWindow::resizeEvent(QResizeEvent *) {
	auto graph = ui->graph;
	graph->yAxis->setScaleRatio(ui->graph->xAxis, 1.0);
	graph->replot();
}

void MainWindow::onNewSTAT(const qSTAT& stat) {
    int id = stat.id();
    auto pRobot = robots[id];
    if (pRobot == nullptr) {
        pRobot = create_newrobot(ui->graph, id);
        robots[id] = pRobot;
    }
    auto& cc = coord();
    auto pos = cc.dataToXY(QPoint(stat.x(), stat.y()));
    auto th = DItems::fAngle::Degree(cc.dataToAngleDegree(stat.th()));

    pRobot->newData(pos.x(), pos.y(), th);
}
void MainWindow::onNewPATH(const qPATH& path) {
    auto& cc = coord();
    QPen pen;
    pen.setColor(Qt::black);
    pen.setWidth(2);
    pen.setStyle(Qt::DashLine);
    for (int i = 0; i < path.n() - 1; ++i) {
        QPointF p(path.x[i](), path.y[i]());
        QPointF q(path.x[i + 1](), path.y[i + 1]());
        auto line = mapitems.pathdata().add_line(p, q);
        line->setPen(pen);
    }
}

void MainWindow::update_display() {
    if (use_bitclient) {
        // Do Nothing
    }
    else {

        for (auto& one: onClient.received) {
            if (!one.second.stat.handled) {
                STAT& stat = one.second.stat();
                onNewSTAT(stat);
                emit newSTAT(stat);
            }
            if (!one.second.path.handled) {
                PATH& path = one.second.path();
                onNewPATH(path);
                one.second.path.handled = true;
                emit newPATH(path);
            }
        }
        for (auto& one: onClient.received) {
            one.second.algo.update(true);
            one.second.stat.update(true);
            one.second.path.update(true);
        }
    }

    ui->graph->replot();

    auto pos = ui->graph->mapFromGlobal(QCursor::pos());
    int x = int(ui->graph->xAxis->pixelToCoord(pos.x())+0.5);
    int y = int(ui->graph->yAxis->pixelToCoord(pos.y())+0.5);
    auto pMouse = coord().xyToData(QPoint(x, y));
    ui->labMouse->setText(QString("(%1, %2)").arg(pMouse.x()).arg(pMouse.y()));
}

int MainWindow::sendDataToServer(const char* data, size_t size) {
    if (use_bitclient) {
        return bit_client.client.send(data, size);
    }
    else {
        if (client) return client->send(data, size);
        return -1;
    }
}


void MainWindow::on_btnClear_clicked() {
    for (auto& one: robots) {
        one.second->remove();  // this will remove shapes/traj of this robot
        if (ui->graph->hasItem(one.second))
            ui->graph->removeItem(one.second); // this will remove the abstract item of this robot
        one.second = nullptr;
    }
    robots.clear();
    mapitems.tempdata().clear();
    mapitems.pathdata().clear();
    ui->graph->replot();
}

void MainWindow::on_coord_changed(CoordTrans newCoord) {
    mapitems.set_coordtrans(newCoord);
    for (auto one: robots) {
        auto rob = one.second;
        coord_change(rob->trajectory, currCoord, newCoord);
    }
    currCoord = newCoord;
}

void MainWindow::on_cbNED_stateChanged(int arg1) {
    CoordTrans newCoord = currCoord;
    if (arg1 == Qt::Checked) {
        ui->statusbar->showMessage("Using NED Coord", 200);
        newCoord.dataIsNED = true;
    }
    if (arg1 == Qt::Unchecked) {
        ui->statusbar->showMessage("Using ENU Coord", 200);
        newCoord.dataIsNED = false;
    }
    on_coord_changed(newCoord);
}

void MainWindow::on_btnConnect_clicked() {
    connect_server();
}



void MainWindow::on_btnTracking_clicked() {
    if (formationHRI.started())
        formationHRI.onTaskStop();
    trackingHRI.onTaskBegin();
}
void MainWindow::on_btnForm_clicked() {
    if (trackingHRI.started())
        trackingHRI.onTaskStop();
    formationHRI.onTaskBegin();
}

void MainWindow::on_btnStop_clicked() {
    if (trackingHRI.started())
        trackingHRI.onTaskStop();
    if (formationHRI.started())
        formationHRI.onTaskStop();

    COMD cmd;
    cmd.cmd = COMD::Stop;
    char buf[64];
    for (auto one: robots) {
        cmd.id() = one.first;
        int nwrite = cmd.write(buf, 64);
        int nsend = sendDataToServer(buf, nwrite);
        if (nwrite != nsend) {
            qDebug()<<"Error, nwrite != nsend";
        }
    }
}

void MainWindow::on_btnSendUser_clicked() {
    auto str = ui->lnUser->text().toStdString();
    str.append("\n");

    unsigned nParser = parsers.pack.size();
    for (unsigned i = 0;i<nParser;++i) {
        parsers.pack[i]->reset();
        int nread = parsers.pack[i]->read(str.c_str(), (int)str.length());
        if (nread > 0) {
            if (parsers.pack[i]->valid()) {
                ui->statusbar->showMessage(QString("Sending %1").arg(parsers.pack[i]->pack_name()));
                int nsend = sendDataToServer(str.c_str(), nread);
                if (nsend != nread) {
                    ui->statusbar->showMessage("Send Failure");
                }
                return;
            }
        }
    }
    ui->statusbar->showMessage("User Pack Invalid");
}

void MainWindow::on_btnMap_clicked() {
    if (default_mapfile.empty()) {
        ui->statusbar->showMessage("Map Empty");
    }
    else {
        mapitems.load_mapinfo(default_mapfile);
    }
}

void MainWindow::on_btnInfo_clicked() {
    if (diag) {
        diag->show();
        diag->activateWindow();
    }
}
void MainWindow::closeEvent(QCloseEvent *) {
    if (diag) diag->close();
    if (diagTask) diagTask->close();
    delete diag;
    delete diagTask;
    diag = nullptr;
    diagTask = nullptr;
}

void MainWindow::on_btnTask_clicked() {
    if (diagTask) diagTask->show();
}



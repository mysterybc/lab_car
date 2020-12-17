#include "mainwindow.h"
#include "wrapstat.h"
#include "sscmd.hpp"
#include <QApplication>



int main(int argc, char *argv[]) {
    sstcp_initialization();

    sscfg::ConfigFile cmd = sscfg::load_fromCMD(argc, argv);
    std::string ip = "127.0.0.1";
    std::string mapfile = "obstacles.txt";
    int port = 33791;
    cmd.get("ip", ip);
    cmd.get("port", port);
    cmd.get("map", mapfile);
    if (mapfile == "null" || mapfile == "none") mapfile.clear();

    QApplication a(argc, argv);
    init_customtypes_main();

    MainWindow w;
    w.default_mapfile = mapfile;
    w.configHRI(cmd);
    w.show();
    w.connect_server(NetworkAddress(ip.c_str(), port));
    return a.exec();
}

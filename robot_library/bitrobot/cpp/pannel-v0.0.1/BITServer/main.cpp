#include "mainwindow.h"
#include "ssconfig.hpp"
#include "sscmd.hpp"
#include <QApplication>
#include <fstream>
#include <sstream>
#include <QDebug>

void add_split(std::string& s) {
    if (!s.empty()) {
        char c = s.back();
        if (c != '\\' && c != '/') {
            s.append("\\");
        }
    }
}

std::vector<RobotIP> load_iplist(const std::string& fname, sscfg::ConfigFile& cmd);
GroupList load_gplist(const std::string& fname, sscfg::ConfigFile& cmd);

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);

    sscfg::ConfigFile cmd = sscfg::load_fromCMD(argc, argv);
    MainWindow w;

    GroupList gp = load_gplist("serverConfig.txt", cmd);
    auto ip = load_iplist("serverConfig.txt", cmd);
    w.init_server(ip, gp);

    w.show();
    return a.exec();
}

void get_default_ip(std::stringstream& config_stream);

bool impl_getgplist(GroupList& gp, sscfg::ConfigFile& config) {
    unsigned sz0 = gp.size();
    // A Maximum Of 10 Groups
    for (int i=1;i<=10;++i) {
        std::string name = "gp0";
        name[2] = '0' + i;
        if (config.exist(name)) {
            std::vector<int> id;
            config.get(name, id);
            gp.push_back(id);
        }
    }
    return gp.size() - sz0 > 0;
}

GroupList load_gplist(const std::string& fname_no_prefix, sscfg::ConfigFile& cmd) {
    GroupList gp;
    if (impl_getgplist(gp, cmd)) {
        return gp;
    }

    // Load Config From File
    // Load Options In CMD arguments
    std::string prefix;
    cmd.get("path", prefix);
    add_split(prefix);

    sscfg::ConfigFile config;
    std::string fname = prefix + fname_no_prefix;
    std::ifstream config_file(prefix + fname, std::ios_base::in);
    if (!config_file) return gp; // Emptry Group Info

    impl_getgplist(gp, config);
    return gp;
}


std::vector<RobotIP> load_iplist(const std::string& fname_no_prefix, sscfg::ConfigFile& cmd) {
    // Load Options In CMD arguments
    std::string prefix;
    cmd.get("path", prefix);
    add_split(prefix);

    int server_port = 0;
    std::string server_ip;
    int port_shift = 0;
    int port_shift_server = 0;
    int port_shift_robot = 0;
    cmd.get("port", server_port);
    cmd.get("ip", server_ip);
    cmd.get("shift", port_shift);
    cmd.get("shift_server", port_shift_server);
    cmd.get("shift_robot", port_shift_robot);
    if (port_shift != 0 && port_shift_server == 0) port_shift_server = port_shift;
    if (port_shift != 0 && port_shift_robot == 0) port_shift_robot = port_shift;

    // Load Config From File
    sscfg::ConfigFile config;
    std::string fname = prefix + fname_no_prefix;
    std::ifstream config_file(prefix + fname, std::ios_base::in);
    if (config_file) {
        printf("Using Config File: %s\n", fname.c_str());
        config = sscfg::ConfigFile::load(config_file);
    }
    else {
        std::stringstream default_config;
        get_default_ip(default_config);
        printf("Using Default Configurations:\n%s\n", default_config.str().c_str());
        config =  sscfg::ConfigFile::load(default_config);
    }

    std::vector<RobotIP> ret;
    RobotIP me;
    me.robotID = 0;
    if (server_ip.empty()) config.get("server_IP", me.ip);
    else me.ip = server_ip;
    if (server_port == 0) config.get("server_port", me.port);
    else me.port = server_port;
    me.port += port_shift_server;
    ret.push_back(me);

    std::vector<int> m_id, m_port_str;
    std::vector<std::string> m_ip;
    config.get("tcpcom_id", m_id);
    config.get("tcpcom_ip", m_ip);
    config.get("strcom_port", m_port_str);
    for (unsigned k=0;k<m_id.size();++k) {
        if (k<m_port_str.size() && k<m_ip.size()) {
            RobotIP one;
            one.robotID = m_id[k];
            one.ip = m_ip[k];
            one.port = m_port_str[k] + port_shift_robot;
            ret.push_back(one);
        }
    }
    return ret;
}


void get_default_ip(std::stringstream& ss) {
    ss << "server_IP    127.0.0.1\n";
    ss << "server_port  33791\n";
    ss << "tcpcom_id    1          2          3          4           5           6           7           8\n";
    ss << "tcpcom_ip    192.168.1.31  192.168.1.32  192.168.1.33  192.168.1.34   127.0.0.1   127.0.0.1   127.0.0.1   127.0.0.1\n";
    //ss << "tcpcom_ip    192.168.1.31  192.168.1.32  192.168.1.33  192.168.1.34   127.0.0.1   127.0.0.1   127.0.0.1   127.0.0.1\n";
    ss << "tcpcom_port  18321      18322      18323      18324       18325       18326       18327       18328\n";
    ss << "strcom_port  18421      18422      18423      18424       18425       18426       18427       18428\n";
}

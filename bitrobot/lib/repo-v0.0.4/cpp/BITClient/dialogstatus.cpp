#include "dialogstatus.h"
#include "ui_dialogstatus.h"


// From BIT.h
/**** ExpGeneral的功能编号 ****/
#define FUNC_UNSET           0  // 功能未设置
#define FUNC_RENDEZVOUS      1	// 聚集到一起
#define FUNC_TRACKING_SINGLE 2	// 单独沿轨线前进
#define FUNC_TRACKING_GROUP  3	// 编队沿轨线前进
#define FUNC_TRACE_HUMAN     4  // 编队跟随人绕过障碍
#define FUNC_TURN_TO         5  // 原地转向目标
#define FUNC_TELE_BASIC      6  // 最基本的遥控控制
#define FUNC_TELE_DXY        7  // 一个通过指定偏移量进行遥控的方法
#define FUNC_TEST_EIGHT      11 // 绕八的任务
#define FUNC_TEST_MOTION     12 // 运动测试任务
#define FUNC_FAULT_DETECT    (1<<8)   // 进行故障检测
#define FUNC_HUMAN_TELE      (1<<9)   // 允许人为干预
#define FUNC_ACT_FAULTY      (1<<10)  // 假装出现故障
#define FUNC_LATENCY_TEST    (1<<11)  // 进行通讯延迟测试
#define FUNC_ALL             0xffff	  // 用于关闭所有功能

/**** Error 编号设置 *****/
#define STATE_GOOD           0	// 正常运行
#define STATE_TARGET_REACHED 1	// 到达目标
#define OTHER_STATE_ERROR    2	// 异常错误
#define STATE_FAULT_DETECTED 3	// 检测出本机故障(故障检测模块返回)

QString funcName(int baseFunction) {
    switch (baseFunction) {
    case FUNC_UNSET:       return "无";
    case FUNC_RENDEZVOUS:  return "集结";
    case FUNC_TRACKING_SINGLE: return "轨迹跟踪";
    case FUNC_TRACKING_GROUP:  return "编队行进";
    case FUNC_TURN_TO:         return "原地转向";
    case FUNC_TELE_BASIC:      return "遥控";
    case FUNC_TELE_DXY:        return "遥控Dxy";
    case FUNC_TEST_EIGHT:      return "绕八测试";
    case FUNC_TRACE_HUMAN:     return "目标跟随";
    }
    return "未知任务";
}
QString stateName(int state) {
    switch (state) {
    case STATE_GOOD: return "正常";
    case STATE_TARGET_REACHED: return "已完成";
    case STATE_FAULT_DETECTED: return "本机故障";
    case OTHER_STATE_ERROR: return "其他故障";
    }
    return "未知状态";
}



DialogStatus::DialogStatus(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogStatus)
{
    ui->setupUi(this);
    setWindowTitle(QString("无人车状态信息"));
    data = new MyModel(this);

    timerOnce = startTimer(100);
    //timerRefresh = startTimer(200);
}

DialogStatus::~DialogStatus()
{
    delete ui;
}


void DialogStatus::timerEvent(QTimerEvent* event) {
    if (event->timerId() == timerOnce) {
        ui->tableView->setModel(data);
        int ncol = data->columnCount();
        for (int i=0;i<ncol;++i) {
            ui->tableView->setColumnWidth(i, 70);
        }
        ui->tableView->setColumnWidth(MyModel::colTask, 80);

        event->accept();
        killTimer(timerOnce);
    }
}



MyModel::MyModel(QObject *parent)
    : QAbstractTableModel(parent)
{
}

int MyModel::rowCount(const QModelIndex & /*parent*/) const
{
   return (int)index2ID.size();
}

int MyModel::columnCount(const QModelIndex & /*parent*/) const
{
    return colNum;
}

QVariant MyModel::data(const QModelIndex &index, int role) const
{
    if (role == Qt::TextAlignmentRole && index.row() < (int)index2ID.size()) {
        return Qt::AlignCenter;
    }
    else if (role == Qt::DisplayRole && index.row() < (int)index2ID.size()) {
        auto it = m_data.find(index2ID[index.row()]);
        if (it == m_data.end())
            return QVariant();

        const auto& q = it->second.q;
        switch (index.column()) {
        case colX:     return q.x();
        case colY:     return q.y();
        case colTh:    return q.th();
        case colV:     return q.v();
        case colW:     return q.w();
        case colState: return stateName(q.status());
        case colTask:  return funcName(q.type());
        case colProg:  return q.progress();
        }
    }
    return QVariant();
}

QVariant MyModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::DisplayRole) {
        if (orientation == Qt::Horizontal) {
            switch (section) {
            case 0:
                return QString("北(x)");
            case 1:
                return QString("东(y)");
            case 2:
                return QString("朝向");
            case 3:
                return QString("线速度");
            case 4:
                return QString("角速度");
            case 5:
                return QString("状态");
            case 6:
                return QString("任务");
            case 7:
                return QString("进度");
            }
        }
        if (orientation == Qt::Vertical) {
            if (section >= (int)index2ID.size()) {
                return QVariant();
            }
            return QString("%1号车").arg(index2ID[section]);
        }
    }

    return QVariant();
}


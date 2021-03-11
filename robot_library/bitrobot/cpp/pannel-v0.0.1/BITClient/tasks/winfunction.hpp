#pragma once
#include "graphcontrol.hpp"
#include "wrapstat.h"
#include "../mapitems.hpp"
#include "../dialogtask.h"
#include "qcustomplot.h"
#include <ssnet/sstcp_protobit.hpp>
#include <QObject>
#include <QEvent>
#include <QDebug>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPushButton>
#include <QCursor>
#include <functional>


struct WindowFunction {
    using BITClient = protobit::BITClient;


    std::map<int, int> keyToRobot;
    int keyCancelSelect;

    QMainWindow* window = nullptr;
    DialogTask* winTask = nullptr;
    QCustomPlot* graph = nullptr;
    QStatusBar* status = nullptr;
    QPushButton* btnConfirm = nullptr;
    QBITClient* bit_client = nullptr;
    MapItems* mapitems = nullptr;
    std::function<bool (int, QPointF&)> robot_position = nullptr;
    std::function<CoordTrans ()> coord = nullptr;
};


template<class PathType>
void curveToPath_impl(QCPCurve* curve, PathType& path, CoordTrans coord, int vpath) {
    int ndata = curve->dataCount();
    path.x.resize(ndata); path.y.resize(ndata); path.v.resize(ndata);
    for (int k=0;k<ndata;++k) {
        const auto& val = curve->data()->at(k);
        QPointF p(val->mainKey(), val->mainValue());
        p = coord.xyToData(p);
        path.x[k] = int(p.x()+0.5);
        path.y[k] = int(p.y()+0.5);
        path.v[k] = vpath;
    }
}


inline void curveToProto(QCPCurve* curve, protobit::PATH& path, CoordTrans coord, int vpath) {
    curveToPath_impl(curve, path, coord, vpath);
    path.n() = curve->dataCount();
}
inline void curveToProto(QCPCurve* curve, protobit::MPLN& path, CoordTrans coord, int vpath) {
    curveToPath_impl(curve, path, coord, vpath);
    path.n() = curve->dataCount();
}
inline void curveToProto(QCPCurve* curve, protobit::FORM& path, CoordTrans coord, int vpath) {
    curveToPath_impl(curve, path, coord, vpath);
    path.npoint() = curve->dataCount();
}







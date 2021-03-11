#pragma once
#include <QPoint>
#include <QPointF>
#include "items/basicitemex.hpp"
#include "qcustomplot.h"
#include "graphcontrol.hpp"
#include "mapinfo/Obstacles.hpp"
#include <vector>
#include <set>


inline void coord_change(DItems::QCPItemShapes* one, const CoordTrans& coord, const CoordTrans& newCoord) {
    if (!one) return ;
    one->setPosition(coord.xyInNewCoord(one->position(), newCoord));
}
inline void coord_change(DItems::QCPItemCPoly* one, const CoordTrans& coord, const CoordTrans& newCoord) {
    if (!one) return ;
    for (auto& one: one->nodeList) {
        one = coord.xyInNewCoord(one, newCoord);
    }
}
inline void coord_change(QCPItemLine* one, const CoordTrans& coord, const CoordTrans& newCoord) {
    if (!one) return ;
    auto p = one->start->coords();
    auto q = one->end->coords();
    one->start->setCoords(coord.xyInNewCoord(p, newCoord));
    one->end->setCoords(coord.xyInNewCoord(q, newCoord));
}
inline void coord_change(QCPCurve* one, const CoordTrans& coord, const CoordTrans& newCoord) {
    if (!one) return ;
    int ndata = one->dataCount();
    if (ndata <= 0) return ;

    QVector<qreal> key(ndata), value(ndata);
    for (int k = 0;k < ndata; ++k) {
        auto pk = one->data()->at(k);
        QPointF p(pk->key, pk->value);
        p = coord.xyInNewCoord(p, newCoord);
        key[k] = p.x();
        value[k] = p.y();
    }
    one->setData(key, value);
}
inline void coord_change(QCPItemText* one, const CoordTrans& coord, const CoordTrans& newCoord) {
    if (!one) return ;
    QPointF q(one->position->key(), one->position->value());
    if (one->position->type() == QCPItemPosition::ptPlotCoords) {
        q = coord.xyInNewCoord(q, newCoord);
        one->position->setCoords(q);
    }
}


struct ItemCollection {
    void clear();
    void remove(DItems::QCPItemShapes* one);
    void remove(DItems::QCPItemCPoly* one);
    void remove(QCPItemLine* one);
    void remove(QCPCurve* one);
    void remove(QCPItemText* one);

    void replot(const CoordTrans& newCoord);
    DItems::QCPItemShapes* add_circle(const QPointF& center, qreal radius);
    DItems::QCPItemCPoly* add_rect(const QPointF& tl, const QPointF& tr, const QPointF& br, const QPointF& bl);
    DItems::QCPItemCPoly* add_cpoly(const std::vector<QPointF>& points);
    QCPCurve* add_curve(const QVector<qreal>& key, const QVector<qreal>& value);
    QCPItemText* add_text(const QString& message, const QPointF& position,
                          QCPItemPosition::PositionType posType = QCPItemPosition::ptPlotCoords,
                          Qt::Alignment posAlign = Qt::AlignLeft | Qt::AlignBottom);

    QCPItemLine* add_line(const QPointF& p, const QPointF& q);
    void set_coordtrans(const CoordTrans& coord);
    void load_fromfile(const std::string& fname);

    QCustomPlot* plot = nullptr;
    CoordTrans coord;
    std::set<DItems::QCPItemShapes*> circles;
    std::set<DItems::QCPItemCPoly*> rects;
    std::set<QCPItemLine*> lines;
    std::set<QCPCurve*>    curves;
    std::set<QCPItemText*> texts;
};

struct MapItems {
    MapItems(){}
    MapItems(QCustomPlot* plot, CoordTrans coord): datalist(3) {
        for (auto& one: datalist) {
            one.plot = plot;
            one.coord = coord;
        }
    }
    void clear_all() {
        for (auto& one : datalist) one.clear();
    }
    void set_coordtrans(CoordTrans coord) {
        for (auto& one : datalist) one.set_coordtrans(coord);
    }
    void load_mapinfo(const std::string& fname) {
        mapdata().load_fromfile(fname);
    }

    ItemCollection& mapdata()  { return datalist[0]; }
    ItemCollection& tempdata() { return datalist[1]; }
    ItemCollection& pathdata() { return datalist[2]; }
    std::vector<ItemCollection> datalist;
};

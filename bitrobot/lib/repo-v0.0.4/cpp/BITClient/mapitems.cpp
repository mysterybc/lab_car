#include "mapitems.hpp"
#include "ssconfig.hpp"

static DItems::QCPItemShapes* oneCircle(QCustomPlot* plot, const QPointF& center, qreal radius) {
    DItems::QCPItemShapes* one = new DItems::QCPItemShapes(plot);
    one->setPen(QPen(Qt::black));
    one->setBrush(QBrush(Qt::gray));
    one->setPosition(center);
    one->setRadiusX(radius);
    one->setRadiusY(radius);
    one->setShapeIndex(DItems::QCPItemShapes::spEllipse);
    return one;
}
static QCPItemLine* oneLine(QCustomPlot* plot, const QPointF& p, const QPointF& q) {
    QCPItemLine* one = new QCPItemLine(plot);
    one->setPen(QPen(Qt::black));
    one->start->setType(QCPItemPosition::PositionType::ptPlotCoords);
    one->start->setCoords(p);
    one->end->setType(QCPItemPosition::PositionType::ptPlotCoords);
    one->end->setCoords(q);
    return one;
}
static DItems::QCPItemCPoly* oneCPoly(QCustomPlot* plot, std::vector<QPointF> points) {
    DItems::QCPItemCPoly* one = new DItems::QCPItemCPoly(plot);
    one->pen= QPen(Qt::black);
    one->brush = QBrush(Qt::gray);
    one->nodeList = std::move(points);
    return one;
}

void ItemCollection::clear() {
    if (!plot) return;
    for (auto one: circles)
        if (one && plot->hasItem(one)) plot->removeItem(one);
    for (auto one: rects)
        if (one && plot->hasItem(one)) plot->removeItem(one);
    for (auto one: lines)
        if (one && plot->hasItem(one)) plot->removeItem(one);
    circles.clear();
    rects.clear();
    lines.clear();
}
void ItemCollection::remove(DItems::QCPItemShapes* one) {
    if (one && plot) {
        if (plot->hasItem(one))
            plot->removeItem(one);
        circles.erase(one);
    }
}
void ItemCollection::remove(DItems::QCPItemCPoly* one) {
    if (one && plot) {
        if (plot->hasItem(one))
            plot->removeItem(one);
        rects.erase(one);
    }
}
void ItemCollection::remove(QCPItemLine* one) {
    if (one && plot) {
        if (plot->hasItem(one))
            plot->removeItem(one);
        lines.erase(one);
    }
}
void ItemCollection::remove(QCPCurve* one) {
    if (one && plot) {
        if (plot->hasPlottable(one))
            plot->removePlottable(one);
        curves.erase(one);
    }
}
void ItemCollection::remove(QCPItemText* one) {
    if (one && plot) {
        if (plot->hasItem(one))
            plot->removeItem(one);
        texts.erase(one);
    }
}

DItems::QCPItemShapes* ItemCollection::add_circle(const QPointF& center, qreal radius) {
    if (!plot) return nullptr;
    auto it = oneCircle(plot, coord.dataToXY(center), radius);
    circles.insert(it);
    return it;
}
DItems::QCPItemCPoly* ItemCollection::add_cpoly(const std::vector<QPointF>& points) {
    if (!plot) return nullptr;
    std::vector<QPointF> p;
    for (auto one: points) p.push_back(coord.dataToXY(one));
    auto it = oneCPoly(plot, p);
    rects.insert(it);
    return it;
}
DItems::QCPItemCPoly* ItemCollection::add_rect(const QPointF& tl, const QPointF& tr, const QPointF& br, const QPointF& bl) {
    return add_cpoly({tl, tr, br, bl});
}
QCPItemLine* ItemCollection::add_line(const QPointF& p, const QPointF& q) {
    if (!plot) return nullptr;
    auto it = oneLine(plot, coord.dataToXY(p), coord.dataToXY(q));
    lines.insert(it);
    return it;
}
QCPCurve* ItemCollection::add_curve(const QVector<qreal>& key, const QVector<qreal>& value) {
    if (!plot) return nullptr;
    auto it = new QCPCurve(plot->xAxis, plot->yAxis);
    it->setData(key, value);
    curves.insert(it);
    return it;
}
QCPItemText* ItemCollection::add_text(const QString& message, const QPointF& position, QCPItemPosition::PositionType posType, Qt::Alignment posAlign) {
    if (!plot) return nullptr;
    auto it = new QCPItemText(plot);
    it->setText(message);
    it->position->setCoords(position);
    it->position->setType(posType);
    it->setPositionAlignment(posAlign);
    texts.insert(it);
    return it;
}


void ItemCollection::set_coordtrans(const CoordTrans& newCoord) {
    replot(newCoord);
    coord = newCoord;
}


void ItemCollection::replot(const CoordTrans& newCoord) {
    if (!plot) return;
    for (auto one: circles) {
        if (one) coord_change(one, coord, newCoord);
    }
    for (auto one: rects) {
        if (one) coord_change(one, coord, newCoord);
    }
    for (auto one: lines) {
        if (one) coord_change(one, coord, newCoord);
    }
    for (auto one: curves) {
        if (one) coord_change(one, coord, newCoord);
    }
    for (auto one: texts) {
        if (one) coord_change(one, coord, newCoord);
    }
}
void ItemCollection::load_fromfile(const std::string& fname) {
    if (!plot) return;

    clear();
    impl::MapData mapinfo;
    mapinfo.load(fname);

    auto unit = mapinfo.obUnit;
    for (const auto& one: mapinfo.obLine) {
        add_line(QPointF(one.p.x, one.p.y)*unit, QPointF(one.q.x, one.q.y)*unit);
    }
    for (const auto& one: mapinfo.obPoint) {
        add_circle(QPointF(one.q.x, one.q.y)*unit, one.r*unit);
    }
    for (const auto& one: mapinfo.obCPoly) {
        unsigned num = one.size();
        std::vector<QPointF> point;
        for (unsigned i = 0; i< num - 1;++i) {
            point.push_back(QPointF(one[i].x, one[i].y)*unit);
        }
        add_cpoly(point);
    }
}

#pragma once
#include "qcustomplot.h"
#include "util.hpp"
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif


namespace DItems{

class QCPItemShapes
    : public QCPAbstractItem
{
    Q_OBJECT
public:
    enum AnchorIndex {aiCenter, aiTop, aiTopLeft, aiTopRight, aiRight, aiBottom, aiBottomRight, aiBottomLeft, aiLeft};
    enum ShapeIndex  {spRect, spEllipse};
    explicit QCPItemShapes(QCustomPlot *parentPlot);
    virtual ~QCPItemShapes(){}
    
    // getters:
    QPen pen() const             { return mPen; }
    QPen selectedPen() const     { return mSelectedPen; }
    QBrush brush() const         { return mBrush; }
    QBrush selectedBrush() const { return mSelectedBrush; }
    QPointF shape() const        { return mShape; }
    qreal radiusX() const        { return mShape.x(); }
    qreal radiusY() const        { return mShape.y(); }
    qreal angle() const          { return mAngle.degree(); }
    int   shapeIndex() const     { return mShapeIndex; }
    QPointF position() const     { return mCenter; }
    qAngle angleEx() const       { return mAngle; }

    // setters;
    void setPen(const QPen &pen)         { mPen = pen; }
    void setSelectedPen(const QPen &pen) { mSelectedPen = pen;}
    void setBrush(const QBrush &brush)   { mBrush = brush; }
    void setSelectedBrush(const QBrush &brush){ mSelectedBrush = brush; }
    void setShape(const QPointF& shape)  { mShape = shape; }
    void setRadiusX(qreal x)             { mShape.setX(x); }
    void setRadiusY(qreal y)             { mShape.setY(y); }
    void setAngle(qreal thDeg)           { mAngle.setDegree(thDeg); }
    void setShapeIndex(int shapeIdx)     { mShapeIndex = shapeIdx;  }
    void setPosition(const QPointF& coords){ mCenter = coords; }
    
    // SelectTest : Reimplementing QCPAbstractItem
    virtual double selectTest(const QPointF &pos, bool onlySelectable, QVariant *details=0) const Q_DECL_OVERRIDE;
    
    // Anchors
    QCPItemAnchor * const center;
    QCPItemAnchor * const top;
    QCPItemAnchor * const topLeft;
    QCPItemAnchor * const left;
    QCPItemAnchor * const bottomLeft;
    QCPItemAnchor * const bottom;
    QCPItemAnchor * const bottomRight;
    QCPItemAnchor * const right;
    QCPItemAnchor * const topRight;
    
protected:
    // property members:
    QPen    mPen, mSelectedPen;
    QBrush  mBrush, mSelectedBrush;
    QPointF mShape;
    QPointF mCenter;
    qAngle  mAngle;
    int     mShapeIndex;
    
     // reimplemented virtual methods:
    virtual void draw(QCPPainter *painter) Q_DECL_OVERRIDE;
    virtual QPointF anchorPixelPosition(int anchorId) const Q_DECL_OVERRIDE;

    // non-virtual methods:
    QPointF toPixel(qreal x, qreal y) const{
        x = parentPlot()->xAxis->coordToPixel(x);
        y = parentPlot()->yAxis->coordToPixel(y);
        return QPointF(x, y);
    }
    QPointF toPixel(const QPointF& q) const{
        return toPixel(q.x(), q.y());
    }
    QPen mainPen() const{
        return mSelected ? mSelectedPen : mPen;
    }
    QBrush mainBrush() const{
        return mSelected ? mSelectedBrush : mBrush;
    }
    
    QRectF myBoundingRect() const;
};
    
class QCPItemCPoly
    : public QCPAbstractItem
{
    Q_OBJECT
public:
    explicit QCPItemCPoly(QCustomPlot *parentPlot):
        QCPAbstractItem(parentPlot)
    {
        pen = QPen(Qt::black);
        brush = Qt::NoBrush;
        penSelected = QPen(Qt::blue, 4);
        brushSelected = Qt::NoBrush;
    }
    virtual ~QCPItemCPoly(){}
    virtual double selectTest (const QPointF &pos, bool onlySelectable, QVariant *details=0) const {
        return -1;
    }

    std::vector<QPointF> nodeList; // In pltCoord
    QPen pen, penSelected;
    QBrush brush, brushSelected;
protected:
    // non-virtual methods:
    QPointF toPixel(qreal x, qreal y) const{
        x = parentPlot()->xAxis->coordToPixel(x);
        y = parentPlot()->yAxis->coordToPixel(y);
        return QPointF(x, y);
    }
    QPointF toPixel(const QPointF& q) const{
        return toPixel(q.x(), q.y());
    }
    QRectF boundingRect() const {
        if (nodeList.empty()) return {};
        //qreal xmin = std::numeric_limits<qreal>::max();
        //qreal xmax = std::numeric_limits<qreal>::min();
		qreal xmin = -1e100, xmax = 1e100;
        qreal ymin = xmin, ymax = xmax;
        for (auto& one: nodeList) {
            xmin = std::min(xmin, one.x());
            xmax = std::max(xmax, one.x());
            ymin = std::min(ymin, one.y());
            ymax = std::max(ymax, one.y());
        }
        QRectF bounding(toPixel(QPointF(xmin, ymax)), toPixel(QPointF(xmax, ymin)));
        return bounding.normalized();
    }
    QPen& mainPen() {
        // mSelected is inherited
        return mSelected ? penSelected: pen;
    }
    QBrush& mainBrush() {
        // mSelected is inherited
        return mSelected ? brushSelected: brush;
    }
    virtual void draw (QCPPainter *painter);
};
    

}

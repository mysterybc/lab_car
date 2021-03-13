#include "basicitemex.hpp"



namespace DItems{
    
QCPItemShapes::QCPItemShapes(QCustomPlot *parentPlot) :
  QCPAbstractItem(parentPlot),
  top(createAnchor(QLatin1String("top"), aiTop)),
  topLeft(createAnchor(QLatin1String("topLeft"), aiTopLeft)),
  left(createAnchor(QLatin1String("left"), aiLeft)),
  bottomLeft(createAnchor(QLatin1String("bottomLeft"), aiBottomLeft)),
  bottom(createAnchor(QLatin1String("bottom"), aiBottom)),
  bottomRight(createAnchor(QLatin1String("bottomRight"), aiBottomRight)),
  right(createAnchor(QLatin1String("right"), aiRight)),
  topRight(createAnchor(QLatin1String("topRight"), aiTopRight)),
  center(createAnchor(QLatin1String("center"), aiCenter)),
  mShape(0, 0),
  mCenter(0, 0),
  mShapeIndex(spRect)
{
  mAngle.setDegree(0);
  
  setPen(QPen(Qt::black));
  setSelectedPen(QPen(Qt::blue,2));
  setBrush(Qt::NoBrush);
  setSelectedBrush(Qt::NoBrush);
}

QRectF QCPItemShapes::myBoundingRect() const{
    qreal x1 = mCenter.x() - mShape.x();
    qreal y1 = mCenter.y() + mShape.y();
    qreal x2 = mCenter.x() + mShape.x();
    qreal y2 = mCenter.y() - mShape.y();
    
    return QRectF(toPixel(QPointF(x1, y1)), toPixel(QPointF(x2, y2))).normalized();
}    
    
double QCPItemShapes::selectTest(const QPointF &pos, bool onlySelectable, QVariant *details) const{
    Q_UNUSED(details)
    if (onlySelectable && !mSelectable)
        return -1;
    
    QRectF rect = myBoundingRect();
    bool filledRect = mBrush.style() != Qt::NoBrush && mBrush.color().alpha() != 0;
    return rectDistance(rect, pos, filledRect);
}    

void QCPItemShapes::draw(QCPPainter *painter){
    if (radiusX() == 0 || radiusY() == 0)
        return ;
    
    QRectF boundingRect = myBoundingRect();
    double clipPad = mainPen().widthF();
    boundingRect = boundingRect.adjusted(-clipPad, -clipPad, clipPad, clipPad);
    if (boundingRect.intersects(clipRect())){
        // only draw if bounding rect of rect item is visible in cliprect
        qreal dx = radiusX();
        qreal dy = radiusY();

        QPointF base = toPixel(0, 0);
        QRectF drect(toPixel(-dx, dy) - base, toPixel(dx, -dy) - base);
        //QRectF drect(QPointF(-10, 5), QPointF(10, -5));
        //qDebug() <<"drect2 "<<drect2;

        painter->save();
        painter->setPen(mainPen());
        painter->setBrush(mainBrush());
        painter->translate(toPixel(mCenter));
        painter->rotate(-angle());
        
        if (mShapeIndex == spRect){
            painter->drawRect(drect);
        }
        else if (mShapeIndex == spEllipse){
            painter->drawEllipse(drect);
        }
        else {
            qDebug() << "Unknown Shape Index "<<mShapeIndex <<", treating as Rect";
            painter->drawRect(drect);
        }
        
        painter->restore();
    }
}

// Valid anchorId:
// aiTop, aiTopLeft, aiTopRight, aiRight, aiBottom, aiBottomRight, aiBottomLeft, aiLeft
QPointF QCPItemShapes::anchorPixelPosition(int anchorId) const{
    QPointF dq;
    switch (anchorId){
    case aiCenter:   dq = QPointF(0, 0); break;
    case aiTop:      dq = QPointF(0, radiusY()); break;
    case aiBottom:   dq = QPointF(0, -radiusY()); break;
    case aiLeft:     dq = QPointF(-radiusX(), 0); break;
    case aiRight:    dq = QPointF(radiusX(), 0); break;
    case aiTopLeft:  dq = QPointF(-radiusX(), radiusY()); break;
    case aiTopRight: dq = QPointF(radiusX(), radiusY()); break;
    case aiBottomLeft:  dq = QPointF(-radiusX(), -radiusY()); break;
    case aiBottomRight: dq = QPointF(radiusX(), -radiusY()); break;
    default:
        qDebug() << "invalid anchorId: " << anchorId;
        return dq;
    }
    dq = util::rotate(dq,  -mAngle);
    return toPixel(mCenter + dq);
}


void QCPItemCPoly::draw (QCPPainter *painter) {
    if (nodeList.empty()) return;

    auto bound = boundingRect();
    double clipPad = pen.widthF();
    bound = bound.adjusted(-clipPad, -clipPad, clipPad, clipPad);
    if (bound.intersects(clipRect())){
        // only draw if bounding rect of rect item is visible
        // in cliprect
        painter->save();
        painter->setPen(mainPen());
        painter->setBrush(mainBrush());

        // This has to be converted everytime
        int n = (int)nodeList.size();
        std::vector<QPointF> pixelNode(nodeList.size());
        for (int i=0;i<n;++i) {
            pixelNode[i] = toPixel(nodeList[i]);
        }
        painter->drawPolygon(pixelNode.data(), n);
        painter->restore();
    }
}



} // namespace DItems

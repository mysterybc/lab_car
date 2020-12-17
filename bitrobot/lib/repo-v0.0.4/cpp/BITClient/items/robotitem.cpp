#include "robotitem.hpp"


namespace DItems{

//conf::RobotConfig* CompoundRobot::default_config = nullptr;

bool CompoundRobot::valid(){
    return body != nullptr && parent != nullptr;
}

bool CompoundRobot::initialize(){
    body       = new QCPItemShapes(parent);
    return true;
}

bool CompoundRobot::remove(){
    if (!parent)    return false;
    
    if (body && parent->hasItem(body))       { parent->removeItem(body); body = nullptr; }
    if (heading && parent->hasItem(heading))    { parent->removeItem(heading); heading = nullptr; }
    if (trajectory && parent->hasPlottable(trajectory)) { parent->removePlottable(trajectory); trajectory = nullptr; }
    if (label && parent->hasItem(label))      { parent->removeItem(label); label = nullptr; }
    for (auto one_pair: lines){
        if (parent->hasItem(one_pair.second)) {
            parent->removeItem(one_pair.second);    // Remove the wrapper
                                                    // This will delete one_pair.second automatically
                                                    // Upon destruction, one_pair.second will call remove() to things inside
        }
    }
    lines.clear();

    return true;
}
void CompoundRobot::removeVector(std::uint8_t line_id){
    auto iter = lines.find(line_id);
    if (iter != lines.end()){
        if (parent->hasItem(iter->second)) {
            parent->removeItem(iter->second);
        }
        lines.erase(iter);
    }
}

int CompoundRobot::newData(float x, float y, fAngle angle){
    if (!valid()) initialize();
    
    qnow = QPointF(x, y);
    body->setPosition(qnow);
    body->setAngle(angle.degree());
    if (trajectory){
        trajectory->addData(x, y);
    }
    for (auto& item : lines){
        item.second->setPosition(x, y);
    }
    return 0;
}

int CompoundRobot::onLineVec(std::uint8_t line_id, float dx, float dy, bool arrow_head){
    if (valid()){
        CompoundVector* vec;
        auto iter = lines.find(line_id);
        if (iter == lines.end()){
            lines[line_id] = new CompoundVector(parent);
            vec = lines[line_id];
            vec->initialize();
            vec->setPosition(qnow.x(), qnow.y());
            vec->setText("");
        }
        else{
            vec = iter->second;
        }
        vec->setShape(dx, dy);
        vec->isArrow(arrow_head);

        qDebug()<<QString("onLineVec %1: (%2, %3)").arg(line_id).arg(dx).arg(dy);
        return 0;
    }
    return -1;
}

void CompoundRobot::setName(const QString& name, float font_size){
    QFont f;
    f.setPointSize(font_size);
    if (valid()){
        if (label) { label->setText(name); label->setFont(f); }
        if (trajectory) { trajectory->setName(name); }
    }
}

void CompoundRobot::withHeading(bool with){
    if (!valid()) return;
    
    if (with && !heading){
        heading = new QCPItemLine(parent);
        heading->start->setParentAnchor(body->center);
        heading->end->setParentAnchor(body->right);
    }
    if (!with && heading){
        parent->removeItem(heading);
        heading = nullptr;
    }
}

void CompoundRobot::withLegend(bool with){
    if (!valid() || !trajectory) return;
    if (with){
        trajectory->addToLegend();
    }
    else{
        trajectory->removeFromLegend();
    }
}

void CompoundRobot::withTrajectory(bool with){
    if (!valid()) return;
    
    if (with && !trajectory){
        trajectory = new QCPCurve(parent->xAxis, parent->yAxis);
    }
    if (!with && trajectory){
        parent->removePlottable(trajectory);
        trajectory = nullptr;
    }
}
void CompoundRobot::withLabel(bool with){
    if (!valid()) return;
    
    if (with && !label){
        label = new QCPItemText(parent);
        label->position->setParentAnchor(body->bottomRight);
    }
    if (!with && label){
        parent->removeItem(label);
        label = nullptr;
    }
}

void CompoundRobot::setShape(float rx, float ry, int shapeIndex){
    if (!valid()) return;
    
    body->setRadiusX(rx);
    body->setRadiusY(ry);
    if (shapeIndex == ShapeEllipse){
        body->setShapeIndex(QCPItemShapes::spEllipse);
    }
    if (shapeIndex == ShapeRectangle){
        body->setShapeIndex(QCPItemShapes::spRect);
    }
}

void CompoundRobot::setFillColor(const QColor& color){
    if (!valid()) return;

    QBrush brush = body->brush();
    brush.setStyle(Qt::SolidPattern);
    brush.setColor(color);
    body->setBrush(brush);
}

void CompoundRobot::setLineColor(const QColor& color){
    if (!valid()) return ;

    QPen pen = body->pen();
    pen.setColor(color);
    pen.setStyle(Qt::SolidLine);
    body->setPen(pen);
    if (trajectory){
        trajectory->setPen(pen);
    }
}
double CompoundRobot::selectTest(const QPointF &pos, bool onlySelectable, QVariant *details) const{
    if (!body) return -1;
    return body->selectTest(pos, onlySelectable, details);
}

void CompoundRobot::selectEvent(QMouseEvent *event, bool additive, const QVariant &, bool *selectionStateChanged){
    event->accept();
    if (additive){
        isSelected = true;
        body->setSelected(true);
        if (selectionStateChanged) *selectionStateChanged = true;
    }
    else if (!isSelected){
        isSelected = true;
        body->setSelected(true);
        if (selectionStateChanged) *selectionStateChanged = true;
    }
    else{
        if (selectionStateChanged) *selectionStateChanged = false;
    }
    tmpDragStart = body->position();
}

void CompoundRobot::deselectEvent(bool *selectionStateChanged){
    //qDebug()<<"on Deselect Evet";
    if (isSelected && selectionStateChanged){
        *selectionStateChanged = true;
    }
    isSelected = false;
    body->setSelected(false);
}

void CompoundRobot::mousePressEvent(QMouseEvent *event, const QVariant &){
    if (isSelected){
        event->accept();
        if (event->button() == Qt::LeftButton){
            tmpDragStart = body->position();
        }
        else if (event->button() == Qt::RightButton){
            if (trajectory){
                //clearGraph();
                trajectory->data()->clear();
                parentPlot()->replot();
            }
        }
        else{
            event->ignore();
        }
    }
}

QPointF toCoord(const QPoint& pixel, const QCPAxis* xAxis, const QCPAxis* yAxis){
    return QPointF(xAxis->pixelToCoord(pixel.x()), yAxis->pixelToCoord(pixel.y()));
}

void CompoundRobot::mouseMoveEvent(QMouseEvent *event, const QPointF &startPos){
    if (isSelected){
        event->accept();
        if (event->buttons() == Qt::LeftButton){
            auto p0 = toCoord(startPos.toPoint(), parentPlot()->xAxis, parentPlot()->yAxis);
            auto p1 = toCoord(event->pos(), parentPlot()->xAxis, parentPlot()->yAxis);
            auto c0 = tmpDragStart + p1 - p0;;
            newData(c0.x(), c0.y(), body->angleEx().to<float>());
            parentPlot()->replot();
        }
    }
}

void CompoundRobot::mouseReleaseEvent(QMouseEvent *event, const QPointF &startPos){
    if (isSelected){
        event->accept();
    }
}

void CompoundRobot::mouseDoubleClickEvent(QMouseEvent *event, const QPointF &){
    event->ignore();
    qDebug() <<"Mouse double clicked";
}

void CompoundRobot::wheelEvent(QWheelEvent *event){
    if (isSelected){
        event->accept();
        qreal dth = event->angleDelta().y() / 120.0 * 10;
        const auto& c = body->position();
        qreal th = body->angle() + dth;
        newData(c.x(), c.y(), fAngle::Degree(th));
        parentPlot()->replot();
    }
    //qDebug() <<"wheelEvent" << event->delta();
}


} // namespace DItems

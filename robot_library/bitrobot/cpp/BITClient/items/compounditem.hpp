#pragma once
#include "qcustomplot.h"
#include "util.hpp"
#include <QDebug>
#include <memory>

namespace DItems{
    
    
class QCPCompoundItem : public QCPAbstractItem{
    Q_OBJECT
public:
    explicit QCPCompoundItem(QCustomPlot *plot)
        : QCPAbstractItem(plot)
    {
        setVisible(true);
    }
    virtual ~QCPCompoundItem(){

    }
    virtual bool initialize() = 0;
    virtual bool remove()     = 0;

    // Inherited from QCPLayerable, return -1 if it's not selectable
    double selectTest (const QPointF &, bool , QVariant *details=0) const { return -1; }

protected:
    QCP::Interaction selectionCategory() const { return QCP::iSelectOther; }
    void applyDefaultAntialiasingHint(QCPPainter*) const {}
    void draw(QCPPainter *){}
};


class CompoundVector
    : public QCPCompoundItem
{
public:
    explicit CompoundVector(QCustomPlot *parent = nullptr)
        : QCPCompoundItem(parent), parent(parent)
    {
        q  = QPointF(0, 0);
        dq = QPointF(0, 0);
        line = nullptr;
        text = nullptr;
    }
    CompoundVector(const CompoundVector&) = delete;

    bool initialize(){
        if (!line) line = new QCPItemLine(parent);
        if (!text) text = new QCPItemText(parent);
        text->setText("");
        line->setHead(QCPLineEnding::esSpikeArrow);
        return true;
    }
    bool remove(){
        if (!parent) return false;
        if (line && parent->hasItem(line)) { parent->removeItem(line); line = nullptr; }
        if (text && parent->hasItem(text)) { parent->removeItem(text); text = nullptr; }
        return true;
    }
    
    CompoundVector& isArrow(bool is){
        if (line){
            if (is) line->setHead(QCPLineEnding::esSpikeArrow);
            else    line->setHead(QCPLineEnding::esNone);
        }
        return *this;
    }
    CompoundVector& setPosition(float x, float y){
        q = QPointF(x, y);
        update();
        return *this;
    }
    CompoundVector& setShape(float dx, float dy){
        dq = QPointF(dx, dy);
        update();
        return *this;
    }
    CompoundVector& setData(QPointF q, QPointF dq){
        this->q = q;
        this->dq = dq;
        update();
        return *this;
    }
    CompoundVector& setText(const QString& str){
        if (line && text){
            text->setText(str);
        }
        return *this;
    }
    
    QCPItemLine* Line() { return line; }
    QCPItemText* Text() { return text; }

private:
    void update(){
        if (line){
            line->start->setCoords(q);
            line->end->setCoords(q + dq);
        }
        if (text) text->position->setCoords(q + dq);
    }
    QCustomPlot *parent = nullptr;
    QCPItemLine *line = nullptr;
    QCPItemText *text = nullptr;
    QPointF q, dq;
};
    
    
}

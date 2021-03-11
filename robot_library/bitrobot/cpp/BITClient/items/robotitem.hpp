#pragma once
#include "basicitemex.hpp"
#include "compounditem.hpp"
#include <map>
#include <cstdint>

namespace DItems{

class CompoundRobot
    : public QCPCompoundItem
{
public:
    enum{
        ShapeEllipse   = 0,
        ShapeRectangle = 1,
        ShapeUnset     = 9999
    };
    
    explicit CompoundRobot(QCustomPlot* parent = nullptr)
        : QCPCompoundItem(parent), parent(parent)
    {
        qnow = QPointF(0, 0);
    }
    CompoundRobot(const CompoundRobot&) = delete;

    // Inherited Interface
    bool initialize();
    bool remove();
    
    // Data
    int newData(float x, float y, fAngle angle);
    int onLineVec(std::uint8_t line_id, float dx, float dy, bool arrow_head = true);
    QCPItemShapes* mainBody() const { return body; }


    // Remove
    bool valid();
    void removeVector(std::uint8_t line_id);
    void clearGraph();
    
    // Configurations    
    void withHeading(bool with);
    void withTrajectory(bool with);
    void withLabel(bool with);
    void withLegend(bool with);
    void setName(const QString& name, float font_size = 12);
    void setShape(float rx, float ry, int shapeIndex);
    void setFillColor(const QColor& color);
    void setLineColor(const QColor& color);

    // On User Interactions
    double selectTest (const QPointF &pos, bool onlySelectable, QVariant *details=0) const;

protected:
    // On User Interactions
    void selectEvent(QMouseEvent *event, bool additive, const QVariant &details, bool *selectionStateChanged);
    void deselectEvent(bool *selectionStateChanged);

    void mousePressEvent(QMouseEvent *event, const QVariant &details);
    void mouseMoveEvent(QMouseEvent *event, const QPointF &startPos);
    void mouseReleaseEvent(QMouseEvent *event, const QPointF &startPos);
    void mouseDoubleClickEvent(QMouseEvent *event, const QPointF &startPos);
    void wheelEvent(QWheelEvent *event);

public:
    // Basic Components
    QCustomPlot   *parent     = nullptr;
    QCPItemShapes *body       = nullptr;    // MainItem Always Exists
    QCPItemLine   *heading    = nullptr;
    QCPCurve      *trajectory = nullptr;
    QCPItemText   *label      = nullptr;

    QPointF        qnow;
    
    // Annotations
    //using pCompundVector =  std::shared_ptr<CompoundVector>;
    std::map<std::uint8_t, CompoundVector*> lines;
private:
    bool isSelected = false;
    QPointF tmpDragStart;
};





} // namespace DItems

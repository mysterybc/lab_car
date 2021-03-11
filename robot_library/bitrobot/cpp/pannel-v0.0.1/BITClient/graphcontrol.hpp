#pragma once
#include <QPointF>
#include <QPoint>
#include <ssnet/sstcp_protobit.hpp>
#include "items/basicitemex.hpp"
#include "items/robotitem.hpp"

template<class T>
T impl_reverse_point(const T& p) {
    return T(p.y(), p.x());
}

struct CoordTrans {
    CoordTrans(bool dataIsNED = false) : dataIsNED(dataIsNED) {}

    template<class T> T dataToXY(const T& p) const {
        return dataIsNED ? impl_reverse_point(p) : p;
    }
    template<class T> T xyToData(const T& p) const {
        return dataIsNED ? impl_reverse_point(p) : p;
    }
    template<class T> T dataToAngleDegree(T value) const {
        return dataIsNED ? 90 - value : value;
    }
    template<class T> T angleToDataDegree(T value) const {
        return dataIsNED ? 90 - value : value;
    }
    template<class T> T xyInNewCoord(const T& p, const CoordTrans& new_coord) const {
        if (dataIsNED == new_coord.dataIsNED) return p;
        return impl_reverse_point(p);
    }
    template<class T> T angleInNewCoordDegree(T value, const CoordTrans& new_coord) const {
        if (dataIsNED == new_coord.dataIsNED) return value;
        return 90 - value;
    }

    bool dataIsNED;
};

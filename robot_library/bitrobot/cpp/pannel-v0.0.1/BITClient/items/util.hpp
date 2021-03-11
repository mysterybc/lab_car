#pragma once
#include <QPointF>

namespace DItems{

template<class _Real>
class Angle{
public:
    Angle(){
        value = static_cast<_Real>(0);
    }
    static Angle Degree(_Real th){
        Angle v;
        v.setDegree(th);
        return v;
    }
    static Angle Radian(_Real th){
        Angle v;
        v.setRadian(th);
        return v;
    }
    
    void setRadian(_Real rad){
        value = rad;
    }
    void setDegree(_Real deg){
        value = deg2rad(deg);
    }
    _Real radian() const{
        return value;
    }
    _Real degree() const{
        return rad2deg(value);
    }
    Angle& normalized(){
        const _Real pi = static_cast<_Real>(3.141592654);
        while (value > pi)  value -= 2*pi;
        while (value < -pi) value += 2*pi;
        return *this;
    }
	template<class newReal>
	Angle<newReal> to() const{
		return Angle<newReal>::Radian(static_cast<newReal>(radian()));
	}
public:
    // Operator Overload
    Angle operator- () const{
        Angle th = *this;
        th.value = -th.value;
        return th;
    }
    
    Angle operator+ (const Angle& th2) const{
        Angle th = *this;
        th.value += th2.value;
        return th;
    }
    Angle operator- (const Angle& th2) const{
        Angle th = *this;
        th.value -= th2.value;
        return th;
    }
    
    Angle& operator += (const Angle& th2) {
        value += th2.value;
        return *this;
    }
    Angle& operator -= (const Angle& th2) {
        value -= th2.value;
        return *this;
    }
    
public:
    static _Real deg2rad(_Real angle){
        const _Real _deg2rad = static_cast<_Real>(3.141592654 / 180);
        return angle * _deg2rad;
    }
    static _Real rad2deg(_Real angle){
        const _Real _rad2deg = static_cast<_Real>(180 / 3.141592654);
        return angle * _rad2deg;
    }
protected:
    _Real value;
};
using qAngle = Angle<qreal>;
using fAngle = Angle<float>;
using dAngle = Angle<double>;

namespace util{
    template<class _Real>
    QPointF rotate(const QPointF& pt, const Angle<_Real>& theta){
        qreal th = static_cast<qreal>(theta.radian());
        qreal cn = std::cos(th);
        qreal sn = std::sin(th);
        qreal x = pt.x() * cn + pt.y() * sn;
        qreal y = -pt.x()* sn + pt.y() * cn;
        return QPointF(x, y);
    }
}


} // namespace DItems

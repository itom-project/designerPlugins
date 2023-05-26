// *************************************************************************************************
//
// This code is part of the Sample Application to demonstrate the use of the QPropertyEditor library.
// It is distributed as public domain and can be modified and used by you without any limitations.
//
// Your feedback is welcome!
//
// Author: Volker Wiendl
// Enum enhancement by Roman alias banal from qt-apps.org
// *************************************************************************************************

#ifndef CUSTOMTYPES_H_
#define CUSTOMTYPES_H_

#include <qvariant.h>

class Property;
class QObject;

struct Vec3f
{
    Vec3f() : X(0.0f), Y(0.0f), Z(0.0f) {}
    Vec3f(float x, float y, float z) : X(x), Y(y), Z(z) {}
    float X, Y, Z;

    bool operator == (const Vec3f& other) const {return X == other.X && Y == other.Y && Z == other.Z;}
    bool operator != (const Vec3f& other) const {return X != other.X || Y != other.Y || Z != other.Z;}

};
Q_DECLARE_METATYPE(Vec3f)

struct CoordSys
{
    CoordSys() : m_x(0.0f), m_y(0.0f), m_z(0.0f), m_scale(1.0f), m_visible(true) {}
    CoordSys(float x, float y, float z, float scale, bool visible) : m_x(x), m_y(y), m_z(z), m_scale(scale), m_visible(visible) {}
    float m_x, m_y, m_z, m_scale;
    bool m_visible;

    bool operator == (const CoordSys& other) const {return m_x == other.m_x && m_y == other.m_y && m_z == other.m_z && m_scale == other.m_scale && m_visible == other.m_visible;}
    bool operator != (const CoordSys& other) const {return m_x != other.m_x && m_y != other.m_y && m_z != other.m_z && m_scale != other.m_scale && m_visible != other.m_visible;}
};
Q_DECLARE_METATYPE(CoordSys)



namespace CustomTypes
{
    void registerTypes();
    Property* createCustomProperty(const QString& name, QObject* propertyObject, Property* parent);

}
#endif

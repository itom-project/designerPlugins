#ifndef COORDSYSPROPERTY_H
#define COORDSYSPROPERTY_H

#include "QPropertyEditor/Property.h"

/**
 * A custom property for Vec3f
 *
 * This class provides a QLineEdit editor for all three values of a Vec3f struct (x, y, z)
 * and also contains three QDoubleSpinBox instances to manipulate each value separately
 */
class CoordSysProperty : public Property
{
    Q_OBJECT
    Q_PROPERTY(float visible READ visible WRITE setVisible DESIGNABLE true USER true)
    Q_PROPERTY(float x READ x WRITE setX DESIGNABLE true USER true)
    Q_PROPERTY(float y READ y WRITE setY DESIGNABLE true USER true)
    Q_PROPERTY(float z READ z WRITE setZ DESIGNABLE true USER true)
    Q_PROPERTY(float scale READ scale WRITE setScale DESIGNABLE true USER true)

public:
    CoordSysProperty(const QString& name = QString(), QObject* propertyObject = 0, QObject* parent = 0);

    QVariant value(int role = Qt::UserRole) const;
    virtual void setValue(const QVariant& value);

    void setEditorHints(const QString& hints);

    float x() const;
    void setX(float x);

    float y() const;
    void setY(float y);

    float z() const;
    void setZ(float z);

    float scale() const;
    void setScale(float scale);

    bool visible() const;
    void setVisible(bool visible);

private:
    QString parseHints(const QString& hints, const QChar component);

    Property*	m_x;
    Property*	m_y;
    Property*	m_z;
    Property*   m_scale;
    Property*   m_visible;
};

#endif

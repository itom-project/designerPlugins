/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut für Technische Optik (ITO),
   Universität Stuttgart, Germany

   This file is part of the designer widget 'vtk3dVisualizer' for itom.

   itom is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   itom is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */


#include "Vec3fProperty.h"
#include "CustomTypes.h"

#include <qregularexpression.h>

Vec3fProperty::Vec3fProperty(const QString& name /*= QString()*/, QObject* propertyObject /*= 0*/, QObject* parent /*= 0*/) : Property(name, propertyObject, parent)
{
    m_x = new Property("x", this, this);
    m_y = new Property("y", this, this);
    m_z = new Property("z", this, this);
    setEditorHints("minimumX=-2147483647;maximumX=2147483647;minimumY=-2147483647;maximumY=2147483647;minimumZ=-2147483647;maximumZ=2147483647;");
}

QVariant Vec3fProperty::value(int role) const
{
    QVariant data = Property::value();
    if (data.isValid() && role != Qt::UserRole)
    {
        switch (role)
        {
        case Qt::DisplayRole:
            return tr("[%1, %2, %3]").arg(data.value<Vec3f>().X).arg(data.value<Vec3f>().Y).arg(data.value<Vec3f>().Z);
        case Qt::EditRole:
            return tr("%1, %2, %3").arg(data.value<Vec3f>().X).arg(data.value<Vec3f>().Y).arg(data.value<Vec3f>().Z);
        };
    }
    return data;
}

void Vec3fProperty::setValue(const QVariant& value)
{
    if (value.type() == QVariant::String)
    {
        QString v = value.toString();
        QRegularExpression rx("([+-]?([0-9]*[\\.,])?[0-9]+(e[+-]?[0-9]+)?)", QRegularExpression::CaseInsensitiveOption);

        int count = 0;
        int pos = 0;
        float x = 0.0f, y = 0.0f, z = 0.0f;
        QRegularExpressionMatch match;

        while ((match = rx.match(v, pos)).hasMatch())
        {
            if (count == 0)
            {
                x = match.captured(1).toDouble();
            }
            else if (count == 1)
            {
                y = match.captured(1).toDouble();
            }
            else if (count == 2)
            {
                z = match.captured(1).toDouble();
            }
            else if (count > 2)
            {
                break;
            }

            ++count;
            pos = match.capturedEnd();
        }

        m_x->setProperty("x", x);
        m_y->setProperty("y", y);
        m_z->setProperty("z", z);
        Property::setValue(QVariant::fromValue(Vec3f(x, y, z)));
    }
    else
    {
        Property::setValue(value);
    }
}

void Vec3fProperty::setEditorHints(const QString& hints)
{
    m_x->setEditorHints(parseHints(hints, 'X'));
    m_y->setEditorHints(parseHints(hints, 'Y'));
    m_z->setEditorHints(parseHints(hints, 'Z'));
}

float Vec3fProperty::x() const
{
    return value().value<Vec3f>().X;
}

void Vec3fProperty::setX(float x)
{
    Property::setValue(QVariant::fromValue(Vec3f(x, y(), z())));
}

float Vec3fProperty::y() const
{
    return value().value<Vec3f>().Y;
}

void Vec3fProperty::setY(float y)
{
    Property::setValue(QVariant::fromValue(Vec3f(x(), y, z())));
}

float Vec3fProperty::z() const
{
    return value().value<Vec3f>().Z;
}

void Vec3fProperty::setZ(float z)
{
    Property::setValue(QVariant::fromValue(Vec3f(x(), y(), z)));
}

QString Vec3fProperty::parseHints(const QString& hints, const QChar component )
{
    QStringList hintList = hints.split(";");
    QString hintTrimmed;
    QString pattern = QString("^(.*)(%1)(=\\s*)(.*)$").arg(component);
    QRegularExpression rx(pattern);
    QRegularExpressionMatch match;
    QStringList componentHints;
    QString name, value;

    foreach(const QString & hint, hintList)
    {
        hintTrimmed = hint.trimmed();

        if (hintTrimmed != "")
        {
            if ((match = rx.match(hintTrimmed)).hasMatch())
            {
                name = match.captured(1).trimmed();
                value = match.captured(4).trimmed();
                componentHints += QString("%1=%2").arg(name).arg(value);
            }
        }
    }

    return componentHints.join(";");
}

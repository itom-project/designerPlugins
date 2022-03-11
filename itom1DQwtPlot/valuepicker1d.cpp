/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut fuer Technische Optik (ITO), 
   Universitaet Stuttgart, Germany 
 
   This file is part of itom.

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

#include "valuepicker1d.h"

#include <qpainter.h>
#include <qbrush.h>
#include <qwt_text.h>

ValuePicker1D::ValuePicker1D(int xAxis, int yAxis, QWidget* parent) : QwtPlotPicker(xAxis, yAxis, parent)
{
}

ValuePicker1D::~ValuePicker1D()
{
}

QwtText ValuePicker1D::trackerTextF( const QPointF &pos ) const
{
    QString x, y;

    if (qAbs(pos.x()) < 100000)
    {
        x = QString::number(pos.x(), 'f', 3);
    }
    else
    {
        x = QString::number(pos.x(), 'g', 5);
    }

    if (qAbs(pos.y()) < 100000)
    {
        y = QString::number(pos.y(), 'f', 3);
    }
    else
    {
        y = QString::number(pos.y(), 'g', 5);
    }

    QString text = QString("x: %1, y: %2").arg(x, y);
    return text;
}

void ValuePicker1D::drawTracker( QPainter *painter ) const
{
    const QRect textRect = trackerRect( painter->font() );
    if ( !textRect.isEmpty() )
    {
        const QwtText label = trackerText( trackerPosition() );
        if ( !label.isEmpty() )
        {
            painter->fillRect(textRect, m_rectFillBrush);
            label.draw( painter, textRect );
        }
    }
}

void ValuePicker1D::setBackgroundFillBrush( const QBrush &brush )
{
    if(brush != this->m_rectFillBrush)
    {
        m_rectFillBrush = brush;
        updateDisplay();
    }
}

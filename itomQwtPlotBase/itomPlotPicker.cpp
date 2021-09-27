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

#include "itomPlotPicker.h"

#include <qpainter.h>
#include <qbrush.h>
#include <qwt_plot_canvas.h>
#include <qwt_text.h>

//----------------------------------------------------------------------------------------------------------------------------------
ItomPlotPicker::ItomPlotPicker( QWidget *parent ) : 
    QwtPlotPicker(parent)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomPlotPicker::ItomPlotPicker( int xAxis, int yAxis, QWidget *parent) : 
    QwtPlotPicker(xAxis, yAxis, parent)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomPlotPicker::ItomPlotPicker( int xAxis, int yAxis, RubberBand rubberBand, DisplayMode trackerMode, QWidget *parent ) : 
    QwtPlotPicker(xAxis, yAxis, rubberBand, trackerMode, parent)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomPlotPicker::~ItomPlotPicker()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomPlotPicker::drawTracker( QPainter *painter ) const
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

//----------------------------------------------------------------------------------------------------------------------------------
void ItomPlotPicker::setBackgroundFillBrush( const QBrush &brush )
{
    if(brush != this->m_rectFillBrush)
    {
        m_rectFillBrush = brush;
        updateDisplay();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------

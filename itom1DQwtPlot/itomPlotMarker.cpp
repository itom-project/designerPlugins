/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut für Technische Optik (ITO), 
   Universität Stuttgart, Germany 
 
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

#include "itomPlotMarker.h"

//----------------------------------------------------------------------------------------------------------------------------------
ItomPlotMarker::ItomPlotMarker(bool labelState,  PlotType type, Qt::Alignment align, Qt::Orientation orient ) : QwtPlotMarker(), m_plotType(type), m_labelState(labelState)
{ 
    setLabelAlignment(align);
    setLabelOrientation(orient);
    setLabelEnabled(m_labelState);
}
//----------------------------------------------------------------------------------------------------------------------------------
void ItomPlotMarker::setPlotType(const PlotType value)
{
    m_plotType = value;
    switch(value)
    {
        case Default:
            setLineStyle(QwtPlotMarker::NoLine);
            break;
        case Multiline:
            setLineStyle(QwtPlotMarker::NoLine);
            break;
        case RangeMarker:
            setLineStyle(QwtPlotMarker::VLine);
            setLinePen( Qt::gray, 1.0, Qt::DashLine );
            break;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void ItomPlotMarker::updateLabelValue()
{
    if(m_labelState)
    {
        QwtText tmp(QString("%1\n%2").arg(this->xValue(),0,'g',4).arg(this->yValue(),0,'g',4 )  );
        tmp.setBackgroundBrush(QBrush(QColor(255, 255, 255, 180), Qt::SolidPattern));
        setLabel(tmp);
    }
    else
    {
        QwtText tmp;
        tmp.setBackgroundBrush(QBrush(QColor(255, 255, 255, 0), Qt::SolidPattern));
        setLabel(tmp);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------

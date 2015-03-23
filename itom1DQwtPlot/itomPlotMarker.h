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

#ifndef PLOT1DWIDGET_H
#define PLOT1DWIDGET_H

#include "common/sharedStructures.h"
#include "DataObject/dataobj.h"
#include "common/sharedStructuresPrimitives.h"

#include <qwidget.h>
#include <qstring.h>
#include <qevent.h>
#include <qpainter.h>
#include <qpoint.h>
#include <qtimer.h>
#include <qcoreapplication.h>
#include <qapplication.h>
#include <qgraphicsview.h>
#include <qgraphicsscene.h>
#include <qgraphicsitem.h>
#include <qqueue.h>
#include <qmenu.h>

#include <qwt_plot_marker.h>

class Itom1DQwtPlot;

class ItomPlotMarker : public QwtPlotMarker
{
    public:

    enum PlotType { Default, RangeMarker, Multiline };

    ItomPlotMarker(bool labelState,  PlotType type, Qt::Alignment align, Qt::Orientation orient );

    void setPlotType(const PlotType value):

    int getPlotType() const {return m_plotType;}

    void setLabelEnabled(const bool state)
    {
        m_labelState = state;
        updateLabelValue();
    }

    void setXValue( double val)
    {
        QwtPlotMarker::setXValue(val);
        updateLabelValue();
    }
    void setYValue( double val)
    {
        QwtPlotMarker::setYValue(val);
        updateLabelValue();
    }
    void setValue( double valx, double valy)
    {
        QwtPlotMarker::setValue(valx, valy);
        updateLabelValue();
    }
    void setValue( const QPointF & val)
    {
        QwtPlotMarker::setValue(val);
        updateLabelValue();
    }

private:

    void updateLabelValue();

    PlotType m_plotType;
    bool m_labelState;

};

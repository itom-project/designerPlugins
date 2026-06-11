/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#ifndef ITOMPLOTPICKERMARKER_H
#define ITOMPLOTPICKERMARKER_H

#include "common/sharedStructures.h"
#include <qwt_plot_marker.h>

#include "itomQwtPlotEnums.h"

class ItomPlotMarker : public QwtPlotMarker
{
    public:

    ItomPlotMarker(bool labelState, ItomQwtPlotEnums::PlotPickerType type, Qt::Alignment align, Qt::Orientation orient);

    void setPlotType(const ItomQwtPlotEnums::PlotPickerType value);

    ItomQwtPlotEnums::PlotPickerType getPlotType() const { return m_plotType; }

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
protected:
    // taken form QWT: qwt_plot_marker.h
    void drawLabel( QPainter * p, const QRectF &, const QPointF & ) const;

private:

    void updateLabelValue();

    ItomQwtPlotEnums::PlotPickerType m_plotType;
    bool m_labelState;

};

#endif //ITOMPLOTPICKERMARKER_H

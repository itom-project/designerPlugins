/* ********************************************************************
 i tom measurement sys*tem
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

#include "drawItem.h"
#include "plotCanvas.h"

#include <qwt_symbol.h>

//----------------------------------------------------------------------------------------------------------------------------------
DrawItem::DrawItem(QwtPlot *parent, char type, const QString &title) : m_pparent(parent), m_type(type), m_active(0), x1(-1), y1(-1),
    x2(-1), y2(-1)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
DrawItem::~DrawItem()
{
    for (int n = 0; n < m_marker.size(); n++)
    {
        m_marker[n]->detach();
//        m_marker.remove(n);
//        delete m_marker[n];
    }
    m_marker.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setShape(const QPainterPath &path)
{
    QwtPlotMarker *marker = NULL;
    QwtPlotShapeItem::setShape(path);
    if (m_marker.size() > 0)
    {

        for (int n = 0; n < m_marker.size(); n++)
        {
            m_marker[n]->detach();
            m_marker.remove(n);
//            delete m_marker[n];
        }
//        m_marker.clear();
    }
    if (path.length() >= 1)
    {
        QPainterPath::Element el;
        marker = new QwtPlotMarker();
        marker->setLinePen(QPen(Qt::green));
        marker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green),3),  QSize(7,7) ));

        switch (m_type)
        {
            case PlotCanvas::tLine:
            case PlotCanvas::tRect:
                el = path.elementAt(0);
                x1 = el.x;
                y1 = el.y;
            break;

            case PlotCanvas::tEllipse:
                el = path.elementAt(6);
                x1 = el.x;
                el = path.elementAt(9);
                y1 = el.y;
            break;
        }

        marker->setXValue(x1);
        marker->setYValue(y1);
        marker->setVisible(true);
        marker->attach(m_pparent);
        m_marker.append(marker);
//        m_active = 1;
    }
    if (path.length() >= 2)
    {
        QPainterPath::Element el;
        marker = new QwtPlotMarker();
        marker->setLinePen(QPen(Qt::green));
        marker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green),3),  QSize(7,7) ));

        switch (m_type)
        {
            case PlotCanvas::tLine:
                el = path.elementAt(1);
                x2 = el.x;
                y2 = el.y;
            break;

            case PlotCanvas::tRect:
                el = path.elementAt(2);
                x2 = el.x;
                y2 = el.y;
            break;

            case PlotCanvas::tEllipse:
                el = path.elementAt(0);
                x2 = el.x;
                el = path.elementAt(3);
                y2 = el.y;
            break;
        }

        marker->setXValue(x2);
        marker->setYValue(y2);
        marker->setVisible(true);
        marker->attach(m_pparent);
        m_marker.append(marker);
//        m_active = 2;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------

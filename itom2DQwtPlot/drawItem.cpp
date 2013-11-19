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

int DrawItem::m_idx = 0;

//----------------------------------------------------------------------------------------------------------------------------------
DrawItem::DrawItem(QwtPlot *parent, char type, const QString &title) : m_pparent(parent), m_type(type), m_active(0), x1(-1), y1(-1),
    x2(-1), y2(-1)
{
    m_idx++;
}

//----------------------------------------------------------------------------------------------------------------------------------
DrawItem::~DrawItem()
{
    detach();
    for (int n = 0; n < m_marker.size(); n++)
    {
        m_marker[n]->detach();
//        m_marker.remove(n);
        delete m_marker[n];
    }
    m_marker.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setActive(int active)
{
    for (int n = 0; n < m_marker.size(); n++)
        m_marker[n]->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(((PlotCanvas*)m_pparent)->m_inverseColor1),
            QPen(QBrush(((PlotCanvas*)m_pparent)->m_inverseColor1), 3),  QSize(7,7) ));

    if (active == 1)
    {
        if (m_marker.size() >= 1)
            m_marker[0]->setSymbol(new QwtSymbol(QwtSymbol::Rect, QBrush(((PlotCanvas*)m_pparent)->m_inverseColor1),
                QPen(QBrush(((PlotCanvas*)m_pparent)->m_inverseColor1), 3),  QSize(7,7) ));
    }
    else if (active == 2)
    {
        if (m_marker.size() >= 2)
            m_marker[1]->setSymbol(new QwtSymbol(QwtSymbol::Rect, QBrush(((PlotCanvas*)m_pparent)->m_inverseColor1),
                QPen(QBrush(((PlotCanvas*)m_pparent)->m_inverseColor1), 3),  QSize(7,7) ));
    }
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
//            m_marker.remove(n);
            delete m_marker[n];
        }
        m_marker.clear();
    }
    //if (path.length() >= 1) // len gives the physical length, not the number of elements!!!
    if (path.elementCount() >= 1)
    {
        QPainterPath::Element el;
        marker = new QwtPlotMarker();
        if (((PlotCanvas*)m_pparent)->m_inverseColor1.isValid())
        {
            marker->setLinePen(QPen(((PlotCanvas*)m_pparent)->m_inverseColor1));
            marker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(((PlotCanvas*)m_pparent)->m_inverseColor1),
                QPen(QBrush(((PlotCanvas*)m_pparent)->m_inverseColor1), 3),  QSize(7,7) ));
        }
        else
        {
            marker->setLinePen(QPen(Qt::green));
            marker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green),3),  QSize(7,7) ));
        }


        switch (m_type)
        {
            default:
            case PlotCanvas::tPoint:
            case PlotCanvas::tLine:
            case PlotCanvas::tRect:
                el = path.elementAt(0);
                x1 = el.x;
                y1 = el.y;
            break;

            case PlotCanvas::tEllipse:
                //if (path.length() >= 7) // len gives the physical length, not the number of elements!!!
                if (path.elementCount() >= 7)
                {
                    el = path.elementAt(6);
                    x1 = el.x;
                }
                //if (path.length() >= 10) // len gives the physical length, not the number of elements!!!
                if (path.elementCount() >= 10)
                {
                    el = path.elementAt(9);
                    y1 = el.y;
                }
            break;

        }

        marker->setXValue(x1);
        marker->setYValue(y1);
        marker->setVisible(true);
        marker->attach(m_pparent);
        m_marker.append(marker);
//        m_active = 1;
    }
    //if (path.length() >= 2) // len gives the physical length, not the number of elements!!!
    if (path.elementCount() >= 2)
    {
        QPainterPath::Element el;
        marker = new QwtPlotMarker();
        if (((PlotCanvas*)m_pparent)->m_inverseColor1.isValid())
        {
            marker->setLinePen(QPen(((PlotCanvas*)m_pparent)->m_inverseColor1));
            marker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(((PlotCanvas*)m_pparent)->m_inverseColor1),
                QPen(QBrush(((PlotCanvas*)m_pparent)->m_inverseColor1), 3),  QSize(7, 7) ));
        }
        else
        {
            marker->setLinePen(QPen(Qt::green));
            marker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green),3),  QSize(7,7) ));
        }

        switch (m_type)
        {
            default:
            case PlotCanvas::tLine:
                el = path.elementAt(1);
                x2 = el.x;
                y2 = el.y;
            break;

            case PlotCanvas::tRect:
                //if (path.length() >= 3) // len gives the physical length, not the number of elements!!!
                if (path.elementCount() >= 3)
                {
                    el = path.elementAt(2);
                    x2 = el.x;
                    y2 = el.y;
                }
            break;

            case PlotCanvas::tEllipse:
                el = path.elementAt(0);
                x2 = el.x;
                //if (path.length() >= 4) // len gives the physical length, not the number of elements!!!
                if (path.elementCount() >= 4)
                {
                    el = path.elementAt(3);
                    y2 = el.y;
                }
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

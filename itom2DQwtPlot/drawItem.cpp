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

#include <qwt_symbol.h>

//----------------------------------------------------------------------------------------------------------------------------------
DrawItem::DrawItem(QwtPlot *parent, const QString &title) : m_pparent(parent)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
DrawItem::~DrawItem()
{
/*
    for (int n = 0; n < m_marker.size(); n++)
    {
        delete m_marker.at(n);
    }
*/
    m_marker.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DrawItem::setShape(const QPainterPath &path)
{
    QwtPlotMarker *marker = NULL;
    QwtPlotShapeItem::setShape(path);
    if (path.length() >= 1)
    {
        marker = new QwtPlotMarker();
        marker->setLinePen(QPen(Qt::green));
        marker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green),3),  QSize(7,7) ));
        QPainterPath::Element el = path.elementAt(0);
        marker->setXValue(el.x);
        marker->setYValue(el.y);
        marker->setVisible(true);
        marker->attach(m_pparent);
        m_marker.append(marker);
    }
    if (path.length() >= 1)
    {
        marker = new QwtPlotMarker();
        marker->setLinePen(QPen(Qt::green));
        marker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green),3),  QSize(7,7) ));
        QPainterPath::Element el = path.elementAt(1);
        marker->setXValue(el.x);
        marker->setYValue(el.y);
        marker->setVisible(true);
        marker->attach(m_pparent);
        m_marker.append(marker);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------

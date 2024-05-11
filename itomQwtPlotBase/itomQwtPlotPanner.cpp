/* ********************************************************************
itom measurement system
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2019, Institut für Technische Optik (ITO),
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

#include <itomQwtPlotPanner.h>
#include <QMouseEvent>

ItomQwtPlotPanner::ItomQwtPlotPanner(QWidget* parent) : QwtPlotPanner(parent),
m_leftClickEnabled(false)
{
}
//---------------------------------------------------------------------------
void ItomQwtPlotPanner::widgetMousePressEvent(QMouseEvent* mouseEvent)
{
    if (m_leftClickEnabled && mouseEvent->buttons() == Qt::LeftButton)
    {
        QMouseEvent newEvent(mouseEvent->type(), mouseEvent->localPos(), mouseEvent->windowPos(), mouseEvent->screenPos(), Qt::MiddleButton, Qt::MiddleButton, mouseEvent->modifiers(), mouseEvent->source());
        QwtPlotPanner::widgetMousePressEvent(&newEvent);
    }
    else
    {
        QwtPlotPanner::widgetMousePressEvent(mouseEvent);
    }
    return;
}

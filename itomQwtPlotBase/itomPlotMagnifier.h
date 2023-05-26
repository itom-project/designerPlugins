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

#ifndef ITOMPLOTMAGNIFIER_H
#define ITOMPLOTMAGNIFIER_H

#include "itomQwtPlotBase.h"

#include <qwt_plot_magnifier.h>
#include <qpoint.h>
#include <qpointer.h>
#include <qmap.h>

class QMouseEvent;
class ItomPlotZoomer;

class ITOMQWTPLOTBASE_EXPORT ItomPlotMagnifier : public QwtPlotMagnifier
{
public:
    explicit ItomPlotMagnifier(QWidget* parent, ItomPlotZoomer *zoomer = NULL);
    virtual ~ItomPlotMagnifier();

    void setAxesDisabledOnAdditionalModifier(const QList<int> &axes, Qt::KeyboardModifiers modifiers = Qt::NoModifier); //if Qt::NoModifier is selected and axes is empty, the axes will be removed from special handling

protected:
    void widgetMouseMoveEvent(QMouseEvent *mouseEvent);
    void widgetWheelEvent( QWheelEvent *wheelEvent );

    void rescale(double factor);
    void rescale(double factor, const QPointF &mouseCoords);

    bool isAxisEnabledSpecial(int axis);

    QPointer<ItomPlotZoomer> m_zoomer;
    const QList<int> *m_pActiveDisabledAxesSet;

private:
    QPoint m_mousePos;
    QMap<Qt::KeyboardModifiers, QList<int> > m_disabledAxesOnSpecialModifiers; //once a modifier listed in this map is pressed during a wheel action, only the indicated axes will be magnified
};

#endif //ITOMPLOTMAGNIFIER_H

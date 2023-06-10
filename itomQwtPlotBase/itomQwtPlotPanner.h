/* ********************************************************************
itom measurement system
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2019, Institut fuer Technische Optik (ITO),
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

#ifndef ITOMQWTPLOTPANNER_H
#define ITOMQWTPLOTPANNER_H

#include <qwt_plot_panner.h>

class ItomQwtPlotPanner : public QwtPlotPanner
{
public:
    explicit ItomQwtPlotPanner(QWidget *parent);
    inline void setLeftClickPanner(bool val) { m_leftClickEnabled = val; }
    inline bool leftClickPanner() { return m_leftClickEnabled; }
protected:
    virtual void widgetMousePressEvent(QMouseEvent* mouseEvent);
private:
    bool m_leftClickEnabled;
};

#endif

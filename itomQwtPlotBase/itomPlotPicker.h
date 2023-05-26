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

#ifndef ITOMPLOTPICKER_H
#define ITOMPLOTPICKER_H

#include "itomQwtPlotBase.h"

#include <qwt_plot_picker.h>

#include <qpoint.h>
#include <qpainter.h>

class ITOMQWTPLOTBASE_EXPORT ItomPlotPicker : public QwtPlotPicker
{
    Q_OBJECT

public:
    explicit ItomPlotPicker( QWidget *parent );
    explicit ItomPlotPicker( int xAxis, int yAxis, QWidget *parent );
    explicit ItomPlotPicker( int xAxis, int yAxis, RubberBand rubberBand, DisplayMode trackerMode, QWidget *parent );

    virtual ~ItomPlotPicker();

    void drawTracker( QPainter *painter ) const;
    void setBackgroundFillBrush( const QBrush &brush );

protected:

private:
    QBrush m_rectFillBrush;

signals:

public slots:

private slots:

};

#endif

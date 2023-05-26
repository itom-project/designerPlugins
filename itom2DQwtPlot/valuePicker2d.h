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

#ifndef VALUEPICKER2D_H
#define VALUEPICKER2D_H

#include <qwt_plot_picker.h>
#include <qwt_plot_canvas.h>

#include <qpoint.h>
#include "dataObjRasterData.h"

class ValuePicker2D : public QwtPlotPicker
{
    Q_OBJECT

public:
    explicit ValuePicker2D(int xAxis, int yAxis, QWidget* parent, const DataObjRasterData* valueData, const DataObjRasterData* overlayData);
    virtual ~ValuePicker2D();

    virtual QwtText trackerTextF( const QPointF &pos ) const;

    void drawTracker( QPainter *painter ) const;
    void setBackgroundFillBrush( const QBrush &brush );

    void enableOverlay(const bool enable) {m_showOverlayInfo = enable;}

protected:

private:
    const DataObjRasterData *m_valueData;
    const DataObjRasterData *m_overlayData;
    QBrush m_rectFillBrush;
    bool m_showOverlayInfo;

signals:

public slots:

private slots:

};

#endif

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

#ifndef QWTPLOTCURVEDATAOBJECT
#define QWTPLOTCURVEDATAOBJECT

#include "common/sharedStructures.h"

#include "DataObject/dataobj.h"

#include "dataObjectSeriesData.h"
#include <qwt_plot_curve.h>

using namespace ito;

class QwtPlotCurveDataObject : public QwtPlotCurve
{
public:

    explicit QwtPlotCurveDataObject( const QString &title = QString::null ) : QwtPlotCurve(title) {}
    explicit QwtPlotCurveDataObject( const QwtText &title ) : QwtPlotCurve(title) {}

    virtual void draw( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect ) const
    {
        DataObjectSeriesData *myData = NULL;
        myData = (DataObjectSeriesData *)data();

        if (myData && myData->isDobjInit())
        {
            myData->setRasterObj();
            drawSeries( painter, xMap, yMap, canvasRect, 0, -1 );
            myData->releaseRasterObj();
        }
    }

};

#endif
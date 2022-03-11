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

/* **********************************************************************

   The code form this file is based on / partially copied from / derived 
   from the code of qwt_plot_curve.cpp / qwt_plot_curve.h found 
   in the QWT-Framework

     * Qwt Widget Library
     * Copyright (C) 1997   Josef Wilgen
     * Copyright (C) 2002   Uwe Rathmann
     *
     * This library is free software; you can redistribute it and/or
     * modify it under the terms of the Qwt License, Version 1.0
************************************************************************ */

#ifndef QWTPLOTCURVEDATAOBJECT
#define QWTPLOTCURVEDATAOBJECT

#include <qwt_plot_curve.h>
#include <qbrush.h>
#include <qpolygon.h>

#include "itomQwtPlotEnums.h"
class QPainter;

class QwtPlotCurveDataObject : public QwtPlotCurve
{

public:

    explicit QwtPlotCurveDataObject(const QString &title = QString());
    explicit QwtPlotCurveDataObject(const QwtText &title);

    virtual void draw( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect ) const;

    void setBrush( const QBrush &brush ) { m_privBrush = brush; QwtPlotCurve::setBrush(brush); }
    void setCurveFitter( QwtCurveFitter *curveFitter ) { m_privCurveFitter = curveFitter; QwtPlotCurve::setCurveFitter(curveFitter); }
    void setCurveFilled(const ItomQwtPlotEnums::FillCurveStyle state) {m_curveFillState = state;}

protected:
    void drawSeries( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;
    void drawCurve( QPainter *painter, int style, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;
    void drawLines( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;

    void drawPolyline(QPainter *painter, QPolygonF &polyline, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect) const;

    void drawSymbols( QPainter *painter, const QwtSymbol & symbol,  const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;

    void drawSticks( QPainter *painter,  const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;
    void drawDots( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;
    void drawSteps( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;
    void drawCenteredSteps( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;
    void closePolyline( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, QPolygonF &polygon ) const;
    void fillCurve( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, QPolygonF &polygon ) const;

    QPolygonF reducePoints(const QPolygonF &polygon, const QwtScaleMap &xMap, bool doAlign) const;

private:
    //this is a hack in order to access PrivateData
    QBrush m_privBrush;
    QwtCurveFitter *m_privCurveFitter;
    ItomQwtPlotEnums::FillCurveStyle m_curveFillState;

};

#endif
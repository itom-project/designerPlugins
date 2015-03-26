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

#include <qwt_plot_curve.h>
#include <qpolygon.h>

class QPainter;

class QwtPlotCurveDataObject : public QwtPlotCurve
{
public:

    explicit QwtPlotCurveDataObject( const QString &title = QString::null ) : QwtPlotCurve(title), m_privCurveFitter(NULL) {}
    explicit QwtPlotCurveDataObject( const QwtText &title ) : QwtPlotCurve(title), m_privCurveFitter(NULL) {}

    virtual void draw( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect ) const;

    void setBrush( const QBrush &brush ) { m_privBrush = brush; QwtPlotCurve::setBrush(brush); }
    void setCurveFitter( QwtCurveFitter *curveFitter ) { m_privCurveFitter = curveFitter; QwtPlotCurve::setCurveFitter(curveFitter); }

protected:
    void drawSeries( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;
    void drawCurve( QPainter *painter, int style, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;
    void drawLines( QPainter *painter, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;

    void drawPolyline(QPainter *painter, QPolygonF &polyline, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect) const;

    void drawSymbols( QPainter *p, const QwtSymbol & symbol,  const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;

private:
    //this is a hack in order to access PrivateData
    QBrush m_privBrush;
    QwtCurveFitter *m_privCurveFitter;

};

#endif
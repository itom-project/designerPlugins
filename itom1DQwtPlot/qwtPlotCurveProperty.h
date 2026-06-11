/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#ifndef QwtPlotCurveProperty_H
#define QwtPlotCurveProperty_H


#include <qobject.h>
#include <qcolor.h>

#include "qwt_plot_curve.h"
#include "itom1DQwtPlot.h"

class QwtPlotCurveProperty : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool visible READ getVisible WRITE setVisible USER true)
    Q_PROPERTY(Qt::PenStyle lineStyle READ getLineStyle WRITE setLineStyle USER true)
    Q_PROPERTY(qreal lineWidth READ getLineWidth WRITE setLineWidth USER true)
    Q_PROPERTY(QColor lineColor READ getLineColor WRITE setLineColor USER true)
    Q_PROPERTY(Qt::PenJoinStyle lineJoinStyle READ getLineJoinStyle WRITE setLineJoinStyle USER true)
    Q_PROPERTY(Qt::PenCapStyle lineCapStyle READ getLineCapStyle WRITE setLineCapStyle USER true)
    Q_PROPERTY(int lineSymbolSize READ getLineSymbolSize WRITE setLineSymbolSize USER true)
    Q_PROPERTY(Itom1DQwtPlot::Symbol lineSymbolStyle READ getLineSymbolStyle WRITE setLineSymbolStyle USER true)
    Q_PROPERTY(bool legendVisible READ getLegendVisible WRITE setLegendVisible USER true)

public:
    QwtPlotCurveProperty(QwtPlotCurve *curve);
    ~QwtPlotCurveProperty() {};

    Qt::PenStyle getLineStyle() const;
    void setLineStyle(const Qt::PenStyle &style);

    qreal getLineWidth() const;
    void setLineWidth(const qreal &width);

    QColor getLineColor() const;
    void setLineColor(const QColor &color);

    Qt::PenJoinStyle getLineJoinStyle() const;
    void setLineJoinStyle(const Qt::PenJoinStyle &style);

    Qt::PenCapStyle getLineCapStyle() const;
    void setLineCapStyle(const Qt::PenCapStyle &style);

    int getLineSymbolSize() const;
    void setLineSymbolSize(int size);

    Itom1DQwtPlot::Symbol getLineSymbolStyle() const;
    void setLineSymbolStyle(const Itom1DQwtPlot::Symbol &symbol);

    bool getLegendVisible() const;
    void setLegendVisible(bool visible);

    bool getVisible() const;
    void setVisible(bool visible);

private:

    QwtPlotCurve *m_pCurve; //borrowed

private slots:

signals :
	void curveChanged();

};

#endif

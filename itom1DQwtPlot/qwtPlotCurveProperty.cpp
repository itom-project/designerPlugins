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

#include "qwtPlotCurveProperty.h"

#include "qwt_symbol.h"
#include "qwt_plot.h"
#include "qwt_legend.h"
#include "qwt_legend_label.h"

//-----------------------------------------------------------------------------------------------------
QwtPlotCurveProperty::QwtPlotCurveProperty(QwtPlotCurve *curve) : m_pCurve(curve)
{

}

//-----------------------------------------------------------------------------------------------------
Qt::PenStyle QwtPlotCurveProperty::getLineStyle() const
{
    if (m_pCurve)
    {
        return m_pCurve->pen().style();
    }
    return Qt::NoPen;
}

//-----------------------------------------------------------------------------------------------------
void QwtPlotCurveProperty::setLineStyle(const Qt::PenStyle &style)
{
    if (m_pCurve)
    {
        QPen pen = m_pCurve->pen();
        if (pen.style() != style)
        {
            pen.setStyle(style);
            m_pCurve->setPen(pen);
        }
		emit curveChanged();
    }
}

//-----------------------------------------------------------------------------------------------------
qreal QwtPlotCurveProperty::getLineWidth() const
{
    if (m_pCurve)
    {
        return m_pCurve->pen().widthF();
    }
    return 0.0;
}

//-----------------------------------------------------------------------------------------------------
void QwtPlotCurveProperty::setLineWidth(const qreal &width)
{
    if (m_pCurve)
    {
        QPen pen = m_pCurve->pen();
        if (pen.widthF() != width)
        {
            pen.setWidthF(width);
            m_pCurve->setPen(pen);
        }
		emit curveChanged();
    }
}

//-----------------------------------------------------------------------------------------------------
QColor QwtPlotCurveProperty::getLineColor() const
{
    if (m_pCurve)
    {
        return m_pCurve->pen().color();
    }
    return 0.0;
}

//-----------------------------------------------------------------------------------------------------
void QwtPlotCurveProperty::setLineColor(const QColor &color)
{
    if (m_pCurve)
    {
        QPen pen = m_pCurve->pen();
        if (pen.color() != color)
        {
            pen.setColor(color);
            m_pCurve->setPen(pen);
        }
		emit curveChanged();
    }
}

//-----------------------------------------------------------------------------------------------------
Qt::PenJoinStyle QwtPlotCurveProperty::getLineJoinStyle() const
{
    if (m_pCurve)
    {
        return m_pCurve->pen().joinStyle();
    }
    return Qt::BevelJoin;
}

//-----------------------------------------------------------------------------------------------------
void QwtPlotCurveProperty::setLineJoinStyle(const Qt::PenJoinStyle &style)
{
    if (m_pCurve)
    {
        QPen pen = m_pCurve->pen();
        if (pen.joinStyle() != style)
        {
            pen.setJoinStyle(style);
            m_pCurve->setPen(pen);
        }
		emit curveChanged();
    }
}

//-----------------------------------------------------------------------------------------------------
Qt::PenCapStyle QwtPlotCurveProperty::getLineCapStyle() const
{
    if (m_pCurve)
    {
        return m_pCurve->pen().capStyle();
    }
    return Qt::SquareCap;
}

//-----------------------------------------------------------------------------------------------------
void QwtPlotCurveProperty::setLineCapStyle(const Qt::PenCapStyle &style)
{
    if (m_pCurve)
    {
        QPen pen = m_pCurve->pen();
        if (pen.capStyle() != style)
        {
            pen.setCapStyle(style);
            m_pCurve->setPen(pen);
        }
		emit curveChanged();
    }
}

//-----------------------------------------------------------------------------------------------------
int QwtPlotCurveProperty::getLineSymbolSize() const
{
    if (m_pCurve && m_pCurve->symbol())
    {
        return m_pCurve->symbol()->size().width();
    }
    return 0;
}

//-----------------------------------------------------------------------------------------------------
void QwtPlotCurveProperty::setLineSymbolSize(int size)
{
    if (m_pCurve)
    {
        const QwtSymbol *s = m_pCurve->symbol();
        QSize newSize(size, size);
		if (!s)
		{
			m_pCurve->setSymbol(new QwtSymbol(QwtSymbol::NoSymbol, QBrush(Qt::white), QPen(m_pCurve->pen().color()), newSize));
		}
		else
		{
			m_pCurve->setSymbol(new QwtSymbol(s->style(), s->brush(), QPen(s->pen().color()), newSize));
		}
		emit curveChanged();
    }
}

//-----------------------------------------------------------------------------------------------------
Itom1DQwtPlot::Symbol QwtPlotCurveProperty::getLineSymbolStyle() const
{
    if (m_pCurve && m_pCurve->symbol())
    {
        QwtSymbol::Style s = m_pCurve->symbol()->style();
        return (s < QwtSymbol::Path ? (Itom1DQwtPlot::Symbol)(s + 1) : Itom1DQwtPlot::NoSymbol);
    }
    return Itom1DQwtPlot::NoSymbol;
}

//-----------------------------------------------------------------------------------------------------
void QwtPlotCurveProperty::setLineSymbolStyle(const Itom1DQwtPlot::Symbol &symbol)
{
	if (m_pCurve)
	{
		const QwtSymbol *s = m_pCurve->symbol();
		if (!s)
		{
			m_pCurve->setSymbol(new QwtSymbol((QwtSymbol::Style)(symbol - 1), QBrush(Qt::white), QPen(m_pCurve->pen().color()), QSize(1, 1)));
		}
		else
		{
			m_pCurve->setSymbol(new QwtSymbol((QwtSymbol::Style)(symbol - 1), QBrush(Qt::white), QPen(s->pen().color()), s->size()));
		}
		emit curveChanged();
	}
}

//-----------------------------------------------------------------------------------------------------
bool QwtPlotCurveProperty::getLegendVisible() const
{
    if (m_pCurve)
    {
        return m_pCurve->testItemAttribute(QwtPlotItem::Legend);
    }
    return true;
}

//-----------------------------------------------------------------------------------------------------
void QwtPlotCurveProperty::setLegendVisible(bool visible)
{
    //if the legend item becomes visible again and the legend is currently displayed, it will be appended to all legend items.
    // Once, the overall legend is moved to another location or becomes visible later, the original order is reset.
    // \Todo: change this!
    if (m_pCurve)
    {
        if (m_pCurve->testItemAttribute(QwtPlotItem::Legend) != visible)
        {
            m_pCurve->setItemAttribute(QwtPlotItem::Legend, visible);

            QwtPlot *plot = m_pCurve->plot();
            if (plot)
            {
                plot->updateLegend();

                QwtLegend *legend = qobject_cast<QwtLegend*>(plot->legend());
                if (legend)
                {
                    QwtLegendLabel *legendLabel = qobject_cast<QwtLegendLabel*>(legend->legendWidget(plot->itemToInfo(m_pCurve)));
                    if (legendLabel)
                    {
                        legendLabel->setChecked(m_pCurve->isVisible());
                    }
                }
            }
        }
		emit curveChanged();
    }

}


//-----------------------------------------------------------------------------------------------------
bool QwtPlotCurveProperty::getVisible() const
{
    if (m_pCurve)
    {
        return m_pCurve->isVisible();
    }
    return true;
}

//-----------------------------------------------------------------------------------------------------
void QwtPlotCurveProperty::setVisible(bool visible)
{
    if (m_pCurve)
    {
        m_pCurve->setVisible(visible);

        QwtPlot *plot = m_pCurve->plot();
        if (plot)
        {
            QwtLegend *legend = qobject_cast<QwtLegend*>(plot->legend());
            if (legend)
            {
                QwtLegendLabel *legendLabel = qobject_cast<QwtLegendLabel*>(legend->legendWidget(plot->itemToInfo(m_pCurve)));
                if (legendLabel)
                {
                    legendLabel->setChecked(m_pCurve->isVisible());
                }
            }
        }
		emit curveChanged();
    }

}

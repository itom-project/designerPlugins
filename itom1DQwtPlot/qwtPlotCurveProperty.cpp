/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2016, Institut fuer Technische Optik (ITO), 
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

#include "QwtPlotCurveProperty.h"

#include "qwt_symbol.h"

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
        if (s && size == 0)
        {
            m_pCurve->setSymbol(NULL);
        }
        else if (!s || s->size() != newSize)
        {
            m_pCurve->setSymbol(new QwtSymbol(s->style(), QBrush(Qt::white), QPen(s->pen().color()), newSize));
        }
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
        if (s && symbol == Itom1DQwtPlot::NoSymbol)
        {
            m_pCurve->setSymbol(NULL);
        }
        else if (!s || (s->style() != (QwtSymbol::Style)(symbol - 1)))
        {
            QSize size = s ? s->size() : QSize(1, 1);
            QColor c = s ? s->pen().color() : m_pCurve->pen().color();
            m_pCurve->setSymbol(new QwtSymbol((QwtSymbol::Style)(symbol - 1), QBrush(Qt::white), QPen(c), size));
        }
    }
}

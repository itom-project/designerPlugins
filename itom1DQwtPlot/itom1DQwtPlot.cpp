/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut fuer Technische Optik (ITO), 
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

#include "itom1DQwtPlot.h"
#include "dataObjectSeriesData.h"
#include "plot1DWidget.h"
#include "DataObject/dataobj.h"

#include "common/sharedStructuresPrimitives.h"

#include <qsharedpointer.h>

#include <qwt_plot.h>
#include <qgridlayout.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_text_label.h>
#include <qwt_scale_widget.h>
#include <qwt_symbol.h>

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::constructor()
{
    m_pInput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, tr("Points for line plots from 2d objects").toLatin1().data()));

    m_pData = new InternalData();
    m_pData->m_autoAxisLabel = true;
    m_pData->m_autoValueLabel = true;
    m_pData->m_valueScaleAuto = true;
    m_pData->m_dataType = ito::tFloat64;
    m_pData->m_valueMin = -127.0;
    m_pData->m_valueMax = 128.0;
    m_pData->m_axisScaleAuto = true;
    m_pData->m_forceValueParsing = false;

    m_pContent = new Plot1DWidget(m_pData, this);
    m_pBaseContent = m_pContent;
    m_pContent->setObjectName("canvasWidget");

    setFocus();
    setCentralWidget(m_pContent);
    m_pContent->setFocus();

    addToolbarsAndMenus();

    setPropertyObservedObject(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::Itom1DQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    ItomQwtDObjFigure(itomSettingsFile, windowMode, parent),
    m_pContent(NULL)
{
    constructor();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::Itom1DQwtPlot(QWidget *parent) :
    ItomQwtDObjFigure("", AbstractFigure::ModeStandaloneInUi, parent),
    m_pContent(NULL)
{
    constructor();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::~Itom1DQwtPlot()
{
    m_pContent->deleteLater();
//    m_pContent = NULL;
    m_pContent = NULL;
    m_pBaseContent = NULL;

    if (m_pData)
        delete m_pData;
    m_pData = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::init()
{ 
    return m_pContent->init(); 
} //called when api-pointers are transmitted, directly after construction

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtPlotEnums::MultiLineMode Itom1DQwtPlot::getRowPresentation(void) const
{
    return m_pData->m_multiLine;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setRowPresentation(const ItomQwtPlotEnums::MultiLineMode idx)
{
    if (m_pContent)
    {
        m_pContent->setRowPresentation(idx);  
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetRowPresentation() 
{
    setRowPresentation(ItomQwtPlotEnums::AutoRowCol);
}

//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getRGBPresentation(void) const
{
    return m_pData->m_colorLine;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setRGBPresentation(const ItomQwtPlotEnums::ColorHandling idx)
{
    if (m_pContent)
    {
        m_pContent->setRGBPresentation(idx);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetRGBPresentation() 
{
    setRGBPresentation(ItomQwtPlotEnums::AutoColor);
}

//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getPickerLimit(void) const
{
    if (m_pData) return m_pData->m_pickerLimit;
    return 2;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerLimit(const int idx)
{
    if (m_pData) m_pData->m_pickerLimit = std::max(0,idx);
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetPickerLimit()
{
    if (m_pData) m_pData->m_pickerLimit = 2;
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getPickerCount(void) const
{
    if (m_pContent)
    {
        return m_pContent->getPickerCount();
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer< ito::DataObject > Itom1DQwtPlot::getPicker() const
{
    if (m_pContent)
    {
        return m_pContent->getPlotPicker();
    }
    return QSharedPointer< ito::DataObject >(new ito::DataObject());
}



//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setUnitLabelStyle(const ito::AbstractFigure::UnitLabelStyle &style)
{
    if (m_pContent)
    {
        m_pContent->setUnitLabelStyle(style);
        QVector<QPointF> bounds = getBounds();
        m_pContent->refreshPlot(m_pInput["source"]->getVal<ito::DataObject*>(), bounds);

        //if y-axis is set to auto, it is rescaled here with respect to the new limits, else the manual range is kept unchanged.
        m_pContent->setInterval(Qt::YAxis, m_pData->m_valueScaleAuto, m_pData->m_valueMin, m_pData->m_valueMax); //replot is done here 
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::applyUpdate()
{
    QVector<QPointF> bounds = getBounds();

    if (m_pInput["source"]->getVal<ito::DataObject*>())
    {
        m_pOutput["displayed"]->copyValueFrom(m_pInput["source"]);
        // why "source" is used here and not "displayed" .... ck 05/15/2013
        
        ito::Channel* dataChannel = getInputChannel("source");
        m_pContent->m_hasParentForRescale = (dataChannel && dataChannel->getParent());

        m_pContent->refreshPlot(m_pInput["source"]->getVal<ito::DataObject*>(), bounds);
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setSource(QSharedPointer<ito::DataObject> source)
{
    m_pData->m_forceValueParsing = true; //recalculate boundaries since content of data object may have changed
    AbstractDObjFigure::setSource(source);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setBounds(QVector<QPointF> bounds) 
{ 
    double *pointArr = new double[2 * bounds.size()];
    for (int np = 0; np < bounds.size(); np++)
    {
        pointArr[np * 2] = bounds[np].x();
        pointArr[np * 2 + 1] = bounds[np].y();
    }
    m_pInput["bounds"]->setVal(pointArr, 2 * bounds.size());
    delete[] pointArr;
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<QPointF> Itom1DQwtPlot::getBounds(void) const
{ 
    int numPts = m_pInput["bounds"]->getLen();
    QVector<QPointF> boundsVec;

    if (numPts > 0)
    {
        double *ptsDblVec = m_pInput["bounds"]->getVal<double*>();
        boundsVec.reserve(numPts / 2);
        for (int n = 0; n < numPts / 2; n++)
        {
            boundsVec.append(QPointF(ptsDblVec[n * 2], ptsDblVec[n * 2 + 1]));
        }
    }
    return boundsVec;
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom1DQwtPlot::getTitle() const
{
    if (m_pData->m_autoTitle)
    {
        return "<auto>";
    }
    return m_pData->m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setTitle(const QString &title)
{
    if (title == "<auto>")
    {
        m_pData->m_autoTitle = true;
    }
    else
    {
        m_pData->m_autoTitle = false;
        m_pData->m_title = title;
    }

    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetTitle()
{
    m_pData->m_autoTitle = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom1DQwtPlot::getAxisLabel() const
{
    if (m_pData->m_autoAxisLabel)
    {
        return "<auto>";
    }
    return m_pData->m_axisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setAxisLabel(const QString &label)
{
    if (label == "<auto>")
    {
        m_pData->m_autoAxisLabel = true;
    }
    else
    {
        m_pData->m_autoAxisLabel = false;
        m_pData->m_axisLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetAxisLabel()
{
    m_pData->m_autoAxisLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom1DQwtPlot::getValueLabel() const
{
    if (m_pData->m_autoValueLabel)
    {
        return "<auto>";
    }
    return m_pData->m_valueLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setValueLabel(const QString &label)
{
    if (label == "<auto>")
    {
        m_pData->m_autoValueLabel = true;
    }
    else
    {
        m_pData->m_autoValueLabel = false;
        m_pData->m_valueLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetValueLabel()
{
    m_pData->m_autoValueLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtPlotEnums::ScaleEngine Itom1DQwtPlot::getValueScale() const
{
    if (m_pContent) 
    {
        return m_pContent->m_valueScale;
    }
    return ItomQwtPlotEnums::Linear;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setValueScale(const ItomQwtPlotEnums::ScaleEngine &scale)
{
    if (m_pContent)
    {
        m_pContent->setDefaultValueScaleEngine(scale);
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtPlotEnums::ScaleEngine Itom1DQwtPlot::getAxisScale() const
{
    if (m_pContent) 
    {
        return m_pContent->m_axisScale;
    }
    return ItomQwtPlotEnums::Linear;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setAxisScale(const ItomQwtPlotEnums::ScaleEngine &scale)
{
    if (m_pContent)
    {
        m_pContent->setDefaultAxisScaleEngine(scale);
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom1DQwtPlot::getTitleFont(void) const
{
    if (m_pContent)
    {
        return m_pContent->titleLabel()->font();
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setTitleFont(const QFont &font)
{
    if (m_pContent)
    {
        m_pContent->titleLabel()->setFont(font);
        //m_pContent->replot();
        updatePropertyDock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom1DQwtPlot::getLabelFont(void) const
{
    if (m_pContent)
    {
        QwtText t = m_pContent->axisWidget(QwtPlot::xBottom)->title();
        return t.font();
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLabelFont(const QFont &font)
{
    if (m_pContent)
    {
        QwtText title;
        title = m_pContent->axisWidget(QwtPlot::xBottom)->title();
        title.setFont(font);
        m_pContent->axisWidget(QwtPlot::xBottom)->setTitle(title);

        title = m_pContent->axisWidget(QwtPlot::yLeft)->title();
        title.setFont(font);
        m_pContent->axisWidget(QwtPlot::yLeft)->setTitle(title);

        updatePropertyDock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom1DQwtPlot::getAxisFont(void) const
{
    if (m_pContent)
    {
        return m_pContent->axisFont(QwtPlot::xBottom);
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setAxisFont(const QFont &font)
{
    if (m_pContent)
    {
        m_pContent->setAxisFont(QwtPlot::xBottom, font);
        m_pContent->setAxisFont(QwtPlot::yLeft, font);

        updatePropertyDock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom1DQwtPlot::getGrid(void) const
{
    if (m_pContent)
    {
        return m_pContent->m_gridEnabled;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setGrid(const bool &enabled)
{
    if (m_pContent)
    {
        m_pContent->setGridEnabled(enabled);
    }
    
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
qreal Itom1DQwtPlot::getLineWidth(void) const
{
    if (m_pContent)
    {
        return m_pContent->m_lineWidth;
    }
    return 1.0;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLineWidth(const qreal &width)
{
    if (m_pContent)
    {
        if (width >= 0.0)
        {
            m_pContent->setLineWidth(width);
        }
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
Qt::PenStyle Itom1DQwtPlot::getLineStyle(void) const
{
    if (m_pContent)
    {
        return m_pContent->m_lineStyle;
    }
    return Qt::SolidLine;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLineStyle(const Qt::PenStyle &style)
{
    if (m_pContent)
    {
        if (/*style != Qt::NoPen &&*/ style != Qt::CustomDashLine)
        {
            m_pContent->setLineStyle(style);
        }
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtPlotEnums::CurveStyle Itom1DQwtPlot::getCurveStyle(void) const
{
    if (m_pContent)
    {
        return m_pContent->m_qwtCurveStyle;
    }
    return ItomQwtPlotEnums::Lines;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setCurveStyle(const ItomQwtPlotEnums::CurveStyle state)
{
    if (m_pContent)
    {
        m_pContent->setQwtLineStyle(state);
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::Symbol Itom1DQwtPlot::getLineSymbol() const
{
    return (Symbol)(m_pData->m_lineSymbole);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLineSymbol(const Symbol symbol)
{
    if (m_pContent)
    {
        
        if (symbol < QwtSymbol::Path && symbol > -2 )
        {
            m_pContent->setSymbolStyle((QwtSymbol::Style)symbol, m_pData->m_lineSymboleSize);
        }
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetLineSymbol()
{
    setLineSymbol(NoSymbol);
}

//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getLineSymbolSize() const
{
    return m_pData->m_lineSymboleSize;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLineSymbolSize(const int size)
{
    if (size < 1 || size > 21)
        return;

    if (m_pContent)
    {
         m_pContent->setSymbolStyle(m_pData->m_lineSymbole, size);
    }
    else
    {
        m_pData->m_lineSymboleSize = size;
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetLineSymbolSize()
{
    if (m_pContent)
    {
        m_pContent->setSymbolStyle(m_pData->m_lineSymbole, 1);
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::LegendPos Itom1DQwtPlot::getLegendPosition() const
{
    if (m_pContent)
    {
        if (m_pContent->m_legendVisible)
        {
            switch (m_pContent->m_legendPosition)
            {
            case QwtPlot::BottomLegend:
                return Bottom;
            case QwtPlot::TopLegend:
                return Top;
            case QwtPlot::LeftLegend:
                return Left;
            case QwtPlot::RightLegend:
                return Right;
            }
        }
    }
    return Off;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLegendPosition(LegendPos legendPosition)
{
    if (m_pContent)
    {
        switch (legendPosition)
        {
        case Off:
            m_pContent->setLegendPosition(QwtPlot::BottomLegend, false);
            break;
        case Left:
            m_pContent->setLegendPosition(QwtPlot::LeftLegend, true);
            break;
        case Top:
            m_pContent->setLegendPosition(QwtPlot::TopLegend, true);
            break;
        case Right:
            m_pContent->setLegendPosition(QwtPlot::RightLegend, true);
            break;
        case Bottom:
            m_pContent->setLegendPosition(QwtPlot::BottomLegend, true);
            break;
        }
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QStringList Itom1DQwtPlot::getLegendTitles() const
{
    if (m_pContent)
    {
        return m_pContent->m_legendTitles;
    }
    return QStringList();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLegendTitles(const QStringList &legends)
{
    if (m_pContent)
    {
        return m_pContent->setLegendTitles(legends, m_pInput["source"]->getVal<ito::DataObject*>());
    }

    updatePropertyDock();
}



//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::plotMarkers(const ito::DataObject &coords, QString style, QString id /*= QString::Null()*/, int plane /*= -1*/)
{
    return m_pContent->plotMarkers(&coords, style, id, plane);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::deleteMarkers(int id)
{
    return m_pContent->deleteGeometricShape(id);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom1DQwtPlot::getYAxisInterval(void) const
{ 
    m_pContent->synchronizeCurrentScaleValues();
    ito::AutoInterval interval(m_pData->m_valueMin, m_pData->m_valueMax, m_pData->m_valueScaleAuto);
    return interval;
}

//----------------------------------------------------------------------------------------------------------------------------------        
void Itom1DQwtPlot::setYAxisInterval(ito::AutoInterval interval) 
{ 
    m_pData->m_valueMin = interval.minimum();
    m_pData->m_valueMax = interval.maximum();
    m_pData->m_valueScaleAuto = interval.isAuto();
    m_pContent->updateScaleValues( interval.isAuto() ); 
    updatePropertyDock();
}   

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom1DQwtPlot::getXAxisInterval(void) const
{ 
    m_pContent->synchronizeCurrentScaleValues();
    ito::AutoInterval interval(m_pData->m_axisMin, m_pData->m_axisMax, m_pData->m_axisScaleAuto);
    return interval;
}

//----------------------------------------------------------------------------------------------------------------------------------        
void Itom1DQwtPlot::setXAxisInterval(ito::AutoInterval interval) 
{ 
    m_pData->m_axisMin = interval.minimum();
    m_pData->m_axisMax = interval.maximum();
    m_pData->m_axisScaleAuto = interval.isAuto();
    m_pContent->updateScaleValues( interval.isAuto() );
    updatePropertyDock();
}  

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::enableObjectGUIElements(const int mode)
{ 
    m_pContent->enableObjectGUIElements(mode);
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> Itom1DQwtPlot::getDisplayed(void)
{
    if (!m_pContent)
    {
        return QSharedPointer<ito::DataObject>(); 
    }
    else
    {
        return m_pContent->getDisplayed();
    }
}



//----------------------------------------------------------------------------------------------------------------------------------
QColor Itom1DQwtPlot::getBackgroundColor(void) const
{
    if (m_pData) 
    {
        return m_pData->m_backgnd;
    }
    else
        return Qt::white;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setBackgroundColor(const QColor newVal)
{
    if (m_pData) 
    {
        InternalData* intData = m_pData;
        intData->m_backgnd = newVal.rgb() & 0x00FFFFFF;
    }
    if (m_pContent)
    {
        m_pContent->updateColors();
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor Itom1DQwtPlot::getAxisColor(void) const
{
    if (m_pData) 
    {
        return m_pData->m_axisColor;
    }
    else
        return Qt::black;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setAxisColor(const QColor newVal)
{
    if (m_pData) 
    {
        InternalData* intData = m_pData;
        intData->m_axisColor = newVal.rgb() & 0x00FFFFFF;
    }
    if (m_pContent)
    {
        m_pContent->updateColors();
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor Itom1DQwtPlot::getTextColor(void) const
{
    if (m_pData) 
    {
        return m_pData->m_textColor;
    }
    else
        return Qt::black;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setTextColor(const QColor newVal)
{
    if (m_pData) 
    {
        InternalData* intData = m_pData;
        intData->m_textColor = newVal.rgb() & 0x00FFFFFF;
    }
    if (m_pContent)
    {
        m_pContent->updateColors();
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::setPicker(const QVector<double> &coords, int curveIndex /*= 0*/, bool physicalCoordinates /*= true*/)
{
    if (!m_pContent)
    {
        return ito::RetVal(ito::retError, 0, tr("Set picker failed, since canvas not initilized").toLatin1().data());
    }
    return m_pContent->setPicker(coords, curveIndex, physicalCoordinates, false);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::appendPicker(const QVector<double> &coords, int curveIndex /*= 0*/, bool physicalCoordinates /*= true*/)
{
    if (!m_pContent)
    {
        return ito::RetVal(ito::retError, 0, tr("Set picker failed, since canvas not initilized").toLatin1().data());
    }
    return m_pContent->setPicker(coords, curveIndex, physicalCoordinates, true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::deletePicker(int id /*= -1*/)
{
    if (!m_pContent)
    {
        return ito::RetVal(ito::retError, 0, tr("Delete picker failed, since canvas not initilized").toLatin1().data());
    }
    return m_pContent->clearPicker(id);
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<int> Itom1DQwtPlot::getPickerPixel() const
{
    if (!m_pContent)
    {
        emit m_pContent->statusBarMessage(tr("Get picker failed, canvas handle not initilized."), 12000 );
    }
    return m_pContent->getPickerPixel();
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<float> Itom1DQwtPlot::getPickerPhys() const
{
    if (!m_pContent)
    {
        emit m_pContent->statusBarMessage(tr("Get picker failed, canvas handle not initilized."), 12000 );
    }
    return m_pContent->getPickerPhys();
}



//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerLabelVisible(const bool state)
{
    if (m_pData) 
    {
        m_pData->m_pickerLabelVisible = state;
    }
    if (m_pContent)
    {
        m_pContent->updatePickerStyle();
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom1DQwtPlot::getPickerLabelVisible() const
{
    if (!m_pData) 
    {
        return false;
    }
    return m_pData->m_pickerLabelVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerLabelOrientation(const Qt::Orientation val)
{
    if (m_pData) 
    {
        m_pData->m_pickerLabelOrientation = val;
    }
    if (m_pContent)
    {
        m_pContent->updatePickerStyle();
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
Qt::Orientation Itom1DQwtPlot::getPickerLabelOrientation() const
{
    if (!m_pData) 
    {
        return Qt::Horizontal;
    }
    return m_pData->m_pickerLabelOrientation;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerLabelAlignment(const Qt::Alignment val)
{
    if (m_pData) 
    {
        m_pData->m_pickerLabelAlignment = val;
    }
    if (m_pContent)
    {
        m_pContent->updatePickerStyle();
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
Qt::Alignment Itom1DQwtPlot::getPickerLabelAlignment()const
{
    if (!m_pData) 
    {
        return Qt::AlignRight;
    }
    return m_pData->m_pickerLabelAlignment;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerType(const ItomQwtPlotEnums::PlotPickerType val)
{
    if (m_pData) 
    {
        m_pData->m_pickerType = val;
    }
    if (m_pContent)
    {
        m_pContent->updatePickerStyle();
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtPlotEnums::PlotPickerType Itom1DQwtPlot::getPickerType() const
{
    if (!m_pData) 
    {
        return ItomQwtPlotEnums::DefaultMarker;
    }
    return m_pData->m_pickerType;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setBaseLine(const qreal val)
{
    if (m_pContent) 
    {
        return m_pContent->setBaseLine(val);
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
qreal Itom1DQwtPlot::getBaseLine() const
{
    if (m_pContent) 
    {
        return m_pContent->m_baseLine;
    }
    return 0.0;
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor Itom1DQwtPlot::getCurveFillColor() const
{
    if (m_pContent) 
    {
        return m_pContent->m_filledColor;
    }
    return QColor::Invalid;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setCurveFillColor(const QColor val)
{
    if (m_pContent) 
    {
        m_pContent->m_filledColor = val;
        m_pContent->m_filledColor.setAlpha(m_pContent->m_fillCurveAlpa);
        return m_pContent->setCurveFilled();
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetCurveFillColor()
{
    setCurveFillColor(QColor::Invalid);
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtPlotEnums::FillCurveStyle Itom1DQwtPlot::getCurveFilled() const
{
    if (m_pContent) 
    {

        return m_pContent->m_curveFilled;
    }
    return ItomQwtPlotEnums::NoCurveFill;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setCurveFilled(const ItomQwtPlotEnums::FillCurveStyle state)
{
    if (m_pContent) 
    {
        m_pContent->m_curveFilled = state;
        m_pContent->setCurveFilled();
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setCurveFillAlpha(const int val)
{
    if (m_pContent) 
    {
        m_pContent->m_fillCurveAlpa = cv::saturate_cast<ito::uint8>(val);
        m_pContent->m_filledColor.setAlpha(m_pContent->m_fillCurveAlpa);
        m_pContent->setCurveFilled();
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getCurveFillAlpha() const
{
    if (m_pContent) 
    {
        return m_pContent->m_fillCurveAlpa;
    }
    return 128;
}
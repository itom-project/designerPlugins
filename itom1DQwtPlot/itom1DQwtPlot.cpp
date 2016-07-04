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

#include <qsharedpointer.h>
#include <qshortcut.h>

#include <qwt_plot.h>
#include <qgridlayout.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_text_label.h>
#include <qwt_scale_widget.h>
#include <qwt_symbol.h>

using namespace ito;

//------------------------------------------------------------------------------------------------------------------------
class Itom1DQwtPlotPrivate 
{
public:
    Itom1DQwtPlotPrivate() : 
        m_pData(NULL),
        m_pLinePropertiesDock(NULL)
    {}
    
    InternalData *m_pData;
    QDockWidget *m_pLinePropertiesDock;
};

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::constructor()
{
    d = new Itom1DQwtPlotPrivate();

    m_pInput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, tr("Points for line plots from 2d objects").toLatin1().data()));

    d->m_pData = new InternalData();
    d->m_pData->m_autoAxisLabel = true;
    d->m_pData->m_autoValueLabel = true;
    d->m_pData->m_valueScaleAuto = true;
    d->m_pData->m_dataType = ito::tFloat64;
    d->m_pData->m_valueMin = -127.0;
    d->m_pData->m_valueMax = 128.0;
    d->m_pData->m_axisScaleAuto = true;
    d->m_pData->m_forceValueParsing = false;

    m_pContent = new Plot1DWidget(d->m_pData, this);
    m_pBaseContent = m_pContent;
    m_pContent->setObjectName("canvasWidget");

    setFocus();
    setCentralWidget(m_pContent);
    m_pContent->setFocus();

    addToolbarsAndMenus();

    //create dock widget for the line properties
    /*d->m_pLinePropertiesDock = new QDockWidget(this);
    d->m_pLinePropertiesDock->setObjectName(QStringLiteral("curveProperties"));
    d->m_pLinePropertiesDock->setWindowTitle("Curve Properties");
    d->m_pLinePropertiesDock->setVisible(false);
    d->m_pLinePropertiesDock->setWidget(new QLabel("this feature is coming soon"));
    addToolbox(d->m_pLinePropertiesDock, "curveProperties", Qt::BottomDockWidgetArea);*/

    registerShortcutActions();

    setPropertyObservedObject(this);

    init(); //here, the API is not ready, yet. Usually init() is called again once the event is raised signalling that the api is ready. However, in QtDesigner, the api is not available. Therefore, we want to init the canvas here.
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
    m_pContent = NULL;
    m_pBaseContent = NULL;

    if (d->m_pData)
        delete d->m_pData;
    d->m_pData = NULL;

    delete d;
    d = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::init()
{ 
    return m_pContent->init(); 
} //called when api-pointers are transmitted, directly after construction

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtPlotEnums::MultiLineMode Itom1DQwtPlot::getRowPresentation(void) const
{
    return d->m_pData->m_multiLine;
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
    return d->m_pData->m_colorLine;
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
    if (d->m_pData) return d->m_pData->m_pickerLimit;
    return 2;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerLimit(const int idx)
{
    if (m_pContent)
    {
        m_pContent->setPickerLimit(idx);
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetPickerLimit()
{
    if (d->m_pData) d->m_pData->m_pickerLimit = 2;
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
        m_pContent->setInterval(Qt::YAxis, d->m_pData->m_valueScaleAuto, d->m_pData->m_valueMin, d->m_pData->m_valueMax); //replot is done here 
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
    d->m_pData->m_forceValueParsing = true; //recalculate boundaries since content of data object may have changed
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
    if (d->m_pData->m_autoTitle)
    {
        return "<auto>";
    }
    return d->m_pData->m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setTitle(const QString &title)
{
    if (title == "<auto>")
    {
        d->m_pData->m_autoTitle = true;
    }
    else
    {
        d->m_pData->m_autoTitle = false;
        d->m_pData->m_title = title;
    }

    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetTitle()
{
    d->m_pData->m_autoTitle = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom1DQwtPlot::getAxisLabel() const
{
    if (d->m_pData->m_autoAxisLabel)
    {
        return "<auto>";
    }
    return d->m_pData->m_axisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setAxisLabel(const QString &label)
{
    if (label == "<auto>")
    {
        d->m_pData->m_autoAxisLabel = true;
    }
    else
    {
        d->m_pData->m_autoAxisLabel = false;
        d->m_pData->m_axisLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetAxisLabel()
{
    d->m_pData->m_autoAxisLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom1DQwtPlot::getValueLabel() const
{
    if (d->m_pData->m_autoValueLabel)
    {
        return "<auto>";
    }
    return d->m_pData->m_valueLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setValueLabel(const QString &label)
{
    if (label == "<auto>")
    {
        d->m_pData->m_autoValueLabel = true;
    }
    else
    {
        d->m_pData->m_autoValueLabel = false;
        d->m_pData->m_valueLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetValueLabel()
{
    d->m_pData->m_autoValueLabel = true;
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
Itom1DQwtPlot::GridStyle Itom1DQwtPlot::getGrid(void) const
{
    if (m_pContent)
    {
        return m_pContent->m_gridStyle;
    }
    return GridNo;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setGrid(const Itom1DQwtPlot::GridStyle &gridStyle)
{
    if (m_pContent)
    {
        m_pContent->setGridStyle(gridStyle);
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
    return (Symbol)(d->m_pData->m_lineSymbole + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLineSymbol(const Symbol symbol)
{
    if (m_pContent)
    {
        
        if (symbol <= QwtSymbol::Path && symbol >= 0 )
        {
            m_pContent->setSymbolStyle((QwtSymbol::Style)(symbol - 1), d->m_pData->m_lineSymboleSize);
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
    return d->m_pData->m_lineSymboleSize;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLineSymbolSize(const int size)
{
    if (size < 1)
        return;

    if (m_pContent)
    {
         m_pContent->setSymbolStyle(d->m_pData->m_lineSymbole, size);
    }
    else
    {
        d->m_pData->m_lineSymboleSize = size;
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetLineSymbolSize()
{
    if (m_pContent)
    {
        m_pContent->setSymbolStyle(d->m_pData->m_lineSymbole, 1);
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
ito::AutoInterval Itom1DQwtPlot::getYAxisInterval(void) const
{ 
    m_pContent->synchronizeCurrentScaleValues();
    ito::AutoInterval interval(d->m_pData->m_valueMin, d->m_pData->m_valueMax, d->m_pData->m_valueScaleAuto);
    return interval;
}

//----------------------------------------------------------------------------------------------------------------------------------        
void Itom1DQwtPlot::setYAxisInterval(ito::AutoInterval interval) 
{ 
    d->m_pData->m_valueMin = interval.minimum();
    d->m_pData->m_valueMax = interval.maximum();
    d->m_pData->m_valueScaleAuto = interval.isAuto();
    m_pContent->updateScaleValues( interval.isAuto() ); 
    updatePropertyDock();
}   

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom1DQwtPlot::getXAxisInterval(void) const
{ 
    m_pContent->synchronizeCurrentScaleValues();
    ito::AutoInterval interval(d->m_pData->m_axisMin, d->m_pData->m_axisMax, d->m_pData->m_axisScaleAuto);
    return interval;
}

//----------------------------------------------------------------------------------------------------------------------------------        
void Itom1DQwtPlot::setXAxisInterval(ito::AutoInterval interval) 
{ 
    d->m_pData->m_axisMin = interval.minimum();
    d->m_pData->m_axisMax = interval.maximum();
    d->m_pData->m_axisScaleAuto = interval.isAuto();
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
ito::RetVal Itom1DQwtPlot::setPicker(const QVector<double> &coordinates, int curveIndex /*= 0*/, bool physicalCoordinates /*= true*/)
{
    if (!m_pContent)
    {
        return ito::RetVal(ito::retError, 0, tr("Set picker failed, since canvas not initilized").toLatin1().data());
    }
    return m_pContent->setPicker(coordinates, curveIndex, physicalCoordinates, false);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::appendPicker(const QVector<double> &coordinates, int curveIndex /*= 0*/, bool physicalCoordinates /*= true*/)
{
    if (!m_pContent)
    {
        return ito::RetVal(ito::retError, 0, tr("Set picker failed, since canvas not initilized").toLatin1().data());
    }
    return m_pContent->setPicker(coordinates, curveIndex, physicalCoordinates, true);
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
    if (d->m_pData) 
    {
        d->m_pData->m_pickerLabelVisible = state;
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
    if (!d->m_pData) 
    {
        return false;
    }
    return d->m_pData->m_pickerLabelVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerLabelOrientation(const Qt::Orientation val)
{
    if (d->m_pData) 
    {
        d->m_pData->m_pickerLabelOrientation = val;
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
    if (!d->m_pData) 
    {
        return Qt::Horizontal;
    }
    return d->m_pData->m_pickerLabelOrientation;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerLabelAlignment(const Qt::Alignment val)
{
    if (d->m_pData) 
    {
        d->m_pData->m_pickerLabelAlignment = val;
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
    if (!d->m_pData) 
    {
        return Qt::AlignRight;
    }
    return d->m_pData->m_pickerLabelAlignment;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerType(const ItomQwtPlotEnums::PlotPickerType val)
{
    if (d->m_pData) 
    {
        d->m_pData->m_pickerType = val;
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
    if (!d->m_pData) 
    {
        return ItomQwtPlotEnums::DefaultMarker;
    }
    return d->m_pData->m_pickerType;
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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::setCurveProperty(int index, const QByteArray &property, const QVariant &value)
{
    if (m_pContent)
    {
        return m_pContent->setCurveProperty(index, property, value);
    }

    return ito::RetVal(ito::retError, 0, "canvas not available");
}

//----------------------------------------------------------------------------------------------------------------------------------
QVariant Itom1DQwtPlot::getCurveProperty(int index, const QByteArray &property)
{
    if (m_pContent)
    {
        return m_pContent->getCurveProperty(index, property);
    }

    return QVariant();
}
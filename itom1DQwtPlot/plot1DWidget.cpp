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

#include "itom1DQwtPlot.h"
#include "plot1DWidget.h"
#include "dataObjectSeriesData.h"
#include "qwtPlotCurveDataObject.h"
#include "common/sharedStructuresGraphics.h"
#include "common/apiFunctionsGraphInc.h"
#include "qnumeric.h"

#include "common/sharedStructuresPrimitives.h"
#include "../sharedFiles/multiPointPickerMachine.h"

#include <qwt_color_map.h>
#include <qwt_plot_layout.h>
#include <qwt_matrix_raster_data.h>
#include <qwt_plot_magnifier.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_renderer.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_canvas.h>
#include <qwt_legend.h>
#include <qwt_legend_label.h>
#include <qwt_symbol.h>
#include <qwt_picker.h>
#include <qwt_picker_machine.h>
#include <qwt_scale_widget.h>
#include <qwt_scale_engine.h>

#include <qimage.h>
#include <qpixmap.h>
#include <qdebug.h>
#include <qmessagebox.h>
#include <qnumeric.h>

//namespace ito {
//    extern void **ITOM_API_FUNCS_GRAPH;
//}

//----------------------------------------------------------------------------------------------------------------------------------
Plot1DWidget::Plot1DWidget(QMenu *contextMenu, InternalData *data, QWidget * parent) :
        QwtPlot(parent),
        m_contextMenu(contextMenu),
        m_pPlotGrid(NULL),
//        m_startScaledX(false),
//        m_startScaledY(false),
        m_xDirect(false),
        m_yDirect(false),
        //m_autoLineColIndex(0),
        m_lineCol(0),
        m_lineWidth(1.0),
        m_lineStyle(Qt::SolidLine),
        m_pParent(parent),
        m_actPickerIdx(-1),
        m_cmplxState(false),
        m_colorState(false),
        m_layerState(false),
        m_pData(data),
        m_activeDrawItem(1),
        m_pRescaler(NULL),
        m_ignoreNextMouseEvent(false),
        m_gridEnabled(false),
        m_legendPosition(BottomLegend),
        m_legendVisible(false),
        m_qwtCurveStyle(Itom1DQwt::Lines),
        m_baseLine(0.0),
        m_fillCurveAlpa(128),
        m_filledColor(QColor::Invalid),
        m_curveFilled(Itom1DQwt::NoCurveFill),
        m_pLegend(NULL),
        m_valueScale(Itom1DQwt::Linear),
        m_axisScale(Itom1DQwt::Linear)
{
    this->setMouseTracking(false);
    
    m_inverseColor0 = Qt::green;
    m_inverseColor1 = Qt::blue;
    

    //multi point picker for pick-point action (equivalent to matlabs ginput)
    m_pMultiPointPicker = new UserInteractionPlotPicker(QwtPlot::xBottom, QwtPlot::yLeft, QwtPicker::PolygonRubberBand, QwtPicker::AlwaysOn, canvas());
    m_pMultiPointPicker->setEnabled(false);
    m_pMultiPointPicker->setRubberBand(QwtPicker::UserRubberBand); //user is cross here
    //m_pMultiPointPicker->setStateMachine(new QwtPickerClickPointMachine);
    m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine);
    m_pMultiPointPicker->setRubberBandPen(QPen(QBrush(Qt::green, Qt::SolidPattern),2));
    connect(m_pMultiPointPicker, SIGNAL(activated(bool)), this, SLOT(multiPointActivated(bool)));

    //canvas() is the real plotting area, where the plot is printed (without axes...)
    //canvas()->setFrameShadow(QFrame::Plain); //deprecated in qwt 6.1.0
    //canvas()->setFrameShape(QFrame::NoFrame); //deprecated in qwt 6.1.0
    canvas()->setStyleSheet("border: 0px;");
    canvas()->setCursor(Qt::ArrowCursor);

    m_colorList.reserve(12);
    m_colorList.append("blue");
    m_colorList.append("green");
    m_colorList.append("red");
    m_colorList.append("magenta");
    m_colorList.append("cyan");
    m_colorList.append("yellow");
    m_colorList.append("darkBlue");
    m_colorList.append("darkGreen");
    m_colorList.append("darkRed");
    m_colorList.append("darkMagenta");
    m_colorList.append("darkCyan");
    m_colorList.append("darkYellow");

    m_pZoomer = new ItomPlotZoomer(QwtPlot::xBottom, QwtPlot::yLeft, canvas());
    m_pZoomer->setEnabled(false);
    m_pZoomer->setTrackerMode(QwtPicker::AlwaysOn);
    m_pZoomer->setMousePattern(QwtEventPattern::MouseSelect2, Qt::NoButton); //right click should open the context menu, not a zoom out to level 0 (Ctrl+0) if zoomer is enabled.
    //m_pZoomer->setFixedAspectRatio(true);
    //all others settings for zoomer are set in init (since they need access to the settings via api)

    m_pPanner = new QwtPlotPanner(canvas());
    m_pPanner->setAxisEnabled(QwtPlot::yRight,false);
    m_pPanner->setCursor(Qt::SizeAllCursor);
    m_pPanner->setEnabled(false);
    connect(m_pPanner, SIGNAL(panned(int,int)), m_pZoomer, SLOT(canvasPanned(int,int))); //if panner is moved, the new rect is added to the zoom stack for a synchronization of both tools

    m_pMagnifier = new ItomPlotMagnifier(canvas(), m_pZoomer);
    m_pMagnifier->setWheelModifiers(Qt::ControlModifier);
    m_pMagnifier->setZoomInKey(Qt::Key_Plus, Qt::KeypadModifier);
    m_pMagnifier->setZoomOutKey(Qt::Key_Minus, Qt::KeypadModifier);
    m_pMagnifier->setMouseFactor(-m_pMagnifier->mouseFactor());
    m_pMagnifier->setEnabled(true);
    m_pMagnifier->setAxisEnabled(QwtPlot::xTop, false);
    m_pMagnifier->setAxisEnabled(QwtPlot::yRight, false);
    m_pMagnifier->setAxisEnabled(QwtPlot::yLeft, true);
    m_pMagnifier->setAxisEnabled(QwtPlot::xBottom, true);

    //value picker
    m_pValuePicker = new ValuePicker1D(QwtPlot::xBottom, QwtPlot::yLeft, canvas());
    m_pValuePicker->setEnabled(false);
    m_pValuePicker->setTrackerMode(QwtPicker::AlwaysOn);
    //all others settings for tracker are set in init (since they need access to the settings via api)

    m_pPlotGrid = new QwtPlotGrid();
    m_pPlotGrid->attach(this);
    setGridEnabled(m_gridEnabled);
    m_pPlotGrid->setMajorPen(Qt::gray, 1);

    m_drawedIemsIndexes.clear();
    m_drawedIemsIndexes.reserve(10);

    setState((Plot1DWidget::tState)m_pData->m_state);
    updateColors();


    //Geometry of the plot:
    setContentsMargins(5,5,5,5); //this is the border between the canvas (including its axes and labels) and the overall mainwindow

    canvas()->setContentsMargins(0,0,0,0); //border of the canvas (border between canvas and axes or title)

    //left axis
    QwtScaleWidget *leftAxis = axisWidget(QwtPlot::yLeft);
    leftAxis->setMargin(0);                 //distance backbone <-> canvas
    leftAxis->setSpacing(6);                //distance tick labels <-> axis label
    leftAxis->scaleDraw()->setSpacing(4);   //distance tick labels <-> ticks
    leftAxis->setContentsMargins(0,0,0,0);  //left axis starts and ends at same level than canvas
    

    //bottom axis
    QwtScaleWidget *bottomAxis = axisWidget(QwtPlot::xBottom);
    bottomAxis->setMargin(0);                 //distance backbone <-> canvas
    bottomAxis->setSpacing(6);                //distance tick labels <-> axis label
    bottomAxis->scaleDraw()->setSpacing(4);   //distance tick labels <-> ticks
    bottomAxis->setContentsMargins(0,0,0,0);  //left axis starts and ends at same level than canvas
}

//----------------------------------------------------------------------------------------------------------------------------------
Plot1DWidget::~Plot1DWidget()
{
    foreach(QwtPlotCurve* c, m_plotCurveItems)
    {
        c->detach();
        delete c;
    }
    m_plotCurveItems.clear();

    foreach (Picker m, m_pickers)
    {
        m.item->detach();
        delete m.item;
    }
    m_pickers.clear();

    if (m_pPlotGrid)
    {
        m_pPlotGrid->detach();
        delete m_pPlotGrid;
        m_pPlotGrid = NULL;
    }

    if (m_pRescaler != NULL)
    {
        m_pRescaler->deleteLater();
        m_pRescaler = NULL;
    }

    if (m_pMultiPointPicker != NULL) 
    {
        m_pMultiPointPicker->deleteLater();
        m_pMultiPointPicker = NULL;
    }

    if (m_pLegend)
    {
        m_pLegend->deleteLater();
        m_pLegend = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::init()
{
    QPen rubberBandPen = QPen(QBrush(Qt::red),2,Qt::DashLine);
    QPen trackerPen = QPen(QBrush(Qt::red),2);
    QFont trackerFont = QFont("Verdana",10);
    QBrush trackerBg = QBrush(QColor(255,255,255,155), Qt::SolidPattern);

    QFont titleFont = QFont("Helvetica",12);
    QFont labelFont =  QFont("Helvetica",12);
    QFont axisFont = QFont("Helvetica",10);

    if(ito::ITOM_API_FUNCS_GRAPH)
    {
        rubberBandPen = apiGetFigureSetting(parent(), "zoomRubberBandPen", rubberBandPen, NULL).value<QPen>();
        trackerPen = apiGetFigureSetting(parent(), "trackerPen", trackerPen, NULL).value<QPen>();
        trackerFont = apiGetFigureSetting(parent(), "trackerFont", trackerFont, NULL).value<QFont>();
        trackerBg = apiGetFigureSetting(parent(), "trackerBackground", trackerBg, NULL).value<QBrush>();

        titleFont = apiGetFigureSetting(parent(), "titleFont", titleFont, NULL).value<QFont>();
        labelFont = apiGetFigureSetting(parent(), "labelFont", labelFont, NULL).value<QFont>();
        axisFont = apiGetFigureSetting(parent(), "axisFont", axisFont, NULL).value<QFont>();   

        m_lineStyle = (Qt::PenStyle)(apiGetFigureSetting(parent(), "lineStyle", (int)m_lineStyle, NULL).value<int>());
        m_lineWidth = apiGetFigureSetting(parent(), "lineWidth", m_lineWidth, NULL).value<qreal>();

        m_curveFilled = (Itom1DQwt::tFillCurveStyle)apiGetFigureSetting(parent(), "fillCurve", (int)m_curveFilled, NULL).value<int>();
        m_filledColor = apiGetFigureSetting(parent(), "curveFillColor", m_filledColor, NULL).value<QColor>();
        m_fillCurveAlpa = cv::saturate_cast<ito::uint8>(apiGetFigureSetting(parent(), "curveFillAlpha", m_fillCurveAlpa, NULL).value<int>());
    }

    
    m_pZoomer->setRubberBandPen(rubberBandPen);
    m_pZoomer->setTrackerFont(trackerFont);
    m_pZoomer->setTrackerPen(trackerPen);

    m_pValuePicker->setTrackerFont(trackerFont);
    m_pValuePicker->setTrackerPen(trackerPen);
    m_pValuePicker->setBackgroundFillBrush(trackerBg);

    title().setFont(titleFont);

    axisTitle(QwtPlot::xBottom).setFont(axisFont);
    axisTitle(QwtPlot::yLeft).setFont(axisFont);

    QwtText t = axisWidget(QwtPlot::xBottom)->title();
    t.setFont(labelFont);
    axisWidget(QwtPlot::xBottom)->setTitle(t);

    t = axisWidget(QwtPlot::yLeft)->title();
    t.setFont(labelFont);
    axisWidget(QwtPlot::yLeft)->setTitle(t);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setDefaultValueScaleEngine(const Itom1DQwt::ScaleEngine &scaleEngine)
{
    if (scaleEngine != m_valueScale)
    {
        if (scaleEngine == Itom1DQwt::Linear)
        {
        setAxisScaleEngine(QwtPlot::yLeft, new QwtLinearScaleEngine());
        }
        else
        {
            setAxisScaleEngine(QwtPlot::yLeft, new QwtLogScaleEngine((int)scaleEngine));
        }

        m_valueScale = scaleEngine;

        bool recalculateBoundaries = false;

        if (m_pData->m_valueScaleAuto == true || m_pData->m_axisScaleAuto == true)
        {
            recalculateBoundaries = true;
        }

        updateScaleValues(recalculateBoundaries);
    }
        
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setDefaultAxisScaleEngine(const Itom1DQwt::ScaleEngine &scaleEngine)
{
    if (scaleEngine != m_axisScale)
    {
        if (scaleEngine == Itom1DQwt::Linear)
        {
        setAxisScaleEngine(QwtPlot::xBottom, new QwtLinearScaleEngine());
        }
        else
        {
            setAxisScaleEngine(QwtPlot::xBottom, new QwtLogScaleEngine((int)scaleEngine));
        }

        m_axisScale = scaleEngine;

        bool recalculateBoundaries = false;

        if (m_pData->m_valueScaleAuto == true || m_pData->m_axisScaleAuto == true)
        {
            recalculateBoundaries = true;
        }

        updateScaleValues(recalculateBoundaries);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setLegendPosition(LegendPosition position, bool visible)
{
    if (m_pLegend)
    {
        m_pLegend->deleteLater();
        m_pLegend = NULL;
    }

    if (visible)
    {
        m_pLegend = new QwtLegend(this);
        m_pLegend->setDefaultItemMode(QwtLegendData::Checkable);
        connect(m_pLegend, SIGNAL(checked(QVariant,bool,int)), this, SLOT(legendItemChecked(QVariant,bool)));
        insertLegend(m_pLegend, position);

        if (m_pLegend)
        {
            QwtLegendLabel *legendLabel = NULL;
            foreach (QwtPlotCurve *item, m_plotCurveItems)
            {
                legendLabel = qobject_cast<QwtLegendLabel*>(m_pLegend->legendWidget( itemToInfo(item) ) );
                if (legendLabel)
                {
                    legendLabel->setChecked(item->isVisible());
                }
            }
        }
    }
    else
    {
        insertLegend(NULL);
    }

    m_legendVisible = visible;
    m_legendPosition = position;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setLegendTitles(const QStringList &legends)
{
    int index = 0;
    m_legendTitles = legends;

    QwtLegendLabel *legendLabel = NULL;
    foreach (QwtPlotCurve *item, m_plotCurveItems)
    {
        if (m_legendTitles.size() == 0)
        {
            item->setTitle(tr("curve %1").arg(index));
        }
        else if (m_legendTitles.size() > index)
        {
            item->setTitle(m_legendTitles[index]);
        }
        else
        {
            item->setTitle("");
        }
        index++;
    }
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setGridEnabled(const bool enabled)
{
    m_gridEnabled = enabled;
    m_pPlotGrid->enableX(enabled);
    m_pPlotGrid->enableY(enabled);
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setLineWidth(const qreal &width)
{
    m_lineWidth = width;

    foreach(QwtPlotCurve *c, m_plotCurveItems)
    {
        QPen pen = c->pen();
        pen.setWidthF(m_lineWidth);
        c->setPen(pen);
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setLineStyle(const Qt::PenStyle &style)
{
    m_lineStyle = style;

    foreach(QwtPlotCurve *c, m_plotCurveItems)
    {
        QPen pen = c->pen();
        pen.setStyle(m_lineStyle);
        c->setPen(pen);
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setSymbolStyle(const QwtSymbol::Style style, int size)
{
    m_pData->m_lineSymbole = style;
    m_pData->m_lineSymboleSize = size;
    foreach(QwtPlotCurve *c, m_plotCurveItems)
    {
        QPen pen = c->pen();
        c->setSymbol(new QwtSymbol(m_pData->m_lineSymbole, QBrush(Qt::white), QPen(pen.color()),  QSize(m_pData->m_lineSymboleSize,m_pData->m_lineSymboleSize)));
    }

    replot();
}


//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setLabels(const QString &title, const QString &valueLabel, const QString &axisLabel)
{
    QwtText t;
    t = axisTitle(QwtPlot::yLeft);
    if (m_pData->m_autoValueLabel)
    {
        t.setText(valueLabel);
    }
    else
    {
        t.setText(m_pData->m_valueLabel);
    }
    setAxisTitle(QwtPlot::yLeft, t);

    t = axisTitle(QwtPlot::xBottom);
    if (m_pData->m_autoAxisLabel)
    {
        t.setText(axisLabel);
    }
    else
    {
        t.setText(m_pData->m_axisLabel);
    }
    setAxisTitle(QwtPlot::xBottom, t);

    if (m_pData->m_autoTitle)
    {
        setTitle(title);
    }
    else
    {
        setTitle(m_pData->m_title);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::updateLabels()
{
    QwtText t;
    t = axisTitle(QwtPlot::yLeft);
    if (m_pData->m_autoValueLabel)
    {
        t.setText(m_pData->m_valueLabelDObj);
    }
    else
    {
        t.setText(m_pData->m_valueLabel);
    }
    setAxisTitle(QwtPlot::yLeft, t);

    t = axisTitle(QwtPlot::xBottom);
    if (m_pData->m_autoAxisLabel)
    {
        t.setText(m_pData->m_axisLabelDObj);
    }
    else
    {
        t.setText(m_pData->m_axisLabel);
    }
    setAxisTitle(QwtPlot::xBottom, t);

    if (m_pData->m_autoTitle)
    {
        setTitle(m_pData->m_titleDObj);
    }
    else
    {
        setTitle(m_pData->m_title);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::refreshPlot(const ito::DataObject* dataObj, QVector<QPointF> bounds)
{
    DataObjectSeriesData* seriesData = NULL;
    int colorIndex;
    int numCurves = 1;
    QwtPlotCurve *curve = NULL;
    QwtPlotCurveDataObject *dObjCurve = NULL;
    bool _unused;

    QwtLegendLabel *legendLabel = NULL;
    int index;
//    bool gotNewObject = false;
    //QString valueLabel, axisLabel, title;

    if (dataObj)
    {
//        gotNewObject = true;
        int dims = dataObj->getDims();
        int width = dims > 0 ? dataObj->getSize(dims - 1) : 0;
        int height = dims > 1 ? dataObj->getSize(dims - 2) : (width == 0) ? 0 : 1;

        if (dataObj->getType() == ito::tComplex128 || dataObj->getType() == ito::tComplex64)
        {
            if (!m_cmplxState) ((Itom1DQwtPlot*)m_pParent)->enableObjectGUIElements(2 | (dims > 1 ? 0x10 : 0x00));
            m_cmplxState = true;
            m_colorState = false;
        }
        else if(dataObj->getType() == ito::tRGBA32)
        {
            if (!m_colorState) ((Itom1DQwtPlot*)m_pParent)->enableObjectGUIElements(1);
            m_colorState = true;
            m_cmplxState = false;
        }
        else
        {
            if (m_cmplxState || m_colorState) ((Itom1DQwtPlot*)m_pParent)->enableObjectGUIElements(0 | (dims > 1 ? 0x10 : 0x00));
            m_cmplxState = false;  
            m_colorState = false;
        }

        Itom1DQwt::tMultiLineMode multiLineMode = m_pData->m_multiLine;

        if(dataObj->getType() == ito::tRGBA32)
        {
            m_layerState = false;
            if(bounds.size() == 0) m_pData->m_multiLine = width == 1 ? Itom1DQwt::FirstCol : Itom1DQwt::FirstRow;
            m_legendTitles.clear();
            switch(m_pData->m_colorLine)
            {
                case Itom1DQwt::Gray:
                    numCurves = 1;
                    m_legendTitles << "gray";
                break;
                case Itom1DQwt::RGB:
                    numCurves = 3;
                    m_legendTitles << "blue" << "green" << "red";
                break;
                case Itom1DQwt::RGBA:
                    numCurves = 4;
                    m_legendTitles << "blue" << "green" << "red" << "alpha";
                break;
                case Itom1DQwt::RGBGray:
                    numCurves = 4;
                    m_legendTitles << "blue" << "green" << "red" << "gray";
                break;
                default:
                    numCurves = 3;
                    m_pData->m_colorLine = Itom1DQwt::RGB;
                    m_legendTitles << "blue" << "green" << "red";
                break;
            }

        }
        else if (bounds.size() == 0)
        {
            m_pData->m_colorLine = Itom1DQwt::AutoColor;
            switch (m_pData->m_multiLine)
            {
                case Itom1DQwt::FirstRow:
                case Itom1DQwt::FirstCol:
                    m_layerState = false;
                    numCurves = 1;
                    break;
                case Itom1DQwt::MultiRows:
                    m_layerState = false;
                    numCurves = height;
                    break;
                case Itom1DQwt::MultiCols:
                    m_layerState = false;
                    numCurves = width;
                    break;
                case Itom1DQwt::MultiLayerAuto:
                    if(width == 1 && height == 1)
                    {
                        multiLineMode = Itom1DQwt::MultiLayerRows;
                    }
                    else if (width >= height)
                    {
                        multiLineMode = Itom1DQwt::MultiLayerRows;
                    }
                    else
                    {
                        multiLineMode = Itom1DQwt::MultiLayerCols;
                    }
                case Itom1DQwt::MultiLayerCols:
                case Itom1DQwt::MultiLayerRows:
                    numCurves = dims > 2 ? dataObj->getSize(dims - 3) : 1;
                    m_layerState = true;
                    break;
                default:
                {
                    m_layerState = false;
                    if(width == 1 && height == 1 && dims < 3)
                    {
                        multiLineMode = Itom1DQwt::MultiRows;
                        numCurves = height;
                    }
                    else if (width >= height)
                    {
                        numCurves = height;
                        multiLineMode = Itom1DQwt::MultiRows;
                    }
                    else
                    {
                        numCurves = width;
                        multiLineMode = Itom1DQwt::MultiCols;
                    }
                }
            }
        }
        else //if there are boundaries, only plot one curve from bounds[0] to bounds[1]
        {
            if(bounds.size() == 3)
            {
                if(m_pData->m_multiLine == Itom1DQwt::MultiLayerCols || 
                   m_pData->m_multiLine == Itom1DQwt::MultiLayerRows || 
                   m_pData->m_multiLine == Itom1DQwt::MultiLayerAuto)
                {
                    m_layerState = true;
                    numCurves = dims > 2 ? dataObj->getSize(dims - 3) : 1;
                }
                else
                {
                    m_layerState = false;
                    numCurves = 1;
                }
            }
            else
            {
                numCurves = 1;
                m_layerState = false;
            }
            m_pData->m_colorLine = Itom1DQwt::AutoColor;
        }

        //check if current number of curves does not correspond to height. If so, adjust the number of curves to the required number
        while (m_plotCurveItems.size() > numCurves)
        {
            curve = m_plotCurveItems.takeLast();
            curve->detach();
            delete curve;
        }

        while (m_plotCurveItems.size() < numCurves)
        {
            index = m_plotCurveItems.size();
            if (m_legendTitles.size() == 0)
            {
                dObjCurve = new QwtPlotCurveDataObject(tr("curve %1").arg(index));
            }
            else if (m_legendTitles.size() > index)
            {
                dObjCurve = new QwtPlotCurveDataObject(m_legendTitles[index]);
            }
            else
            {
                dObjCurve = new QwtPlotCurveDataObject("");
            }

    
            dObjCurve->setData(NULL);
            dObjCurve->attach(this);

            if (m_pLegend)
            {
                legendLabel = qobject_cast<QwtLegendLabel*>(m_pLegend->legendWidget( itemToInfo(dObjCurve) ) );
                if (legendLabel)
                {
                    legendLabel->setChecked(true);
                }
            }

            QPen plotPen;
            colorIndex = m_plotCurveItems.size() % m_colorList.size();
            plotPen.setColor(m_colorList[colorIndex]);
            plotPen.setStyle(m_lineStyle);
            plotPen.setWidth(m_lineWidth);
            dObjCurve->setPen(plotPen);

            // Add Symbol here
            if(m_pData->m_lineSymbole != QwtSymbol::NoSymbol)
            {
                dObjCurve->setSymbol(new QwtSymbol(m_pData->m_lineSymbole, QBrush(Qt::white), QPen(m_colorList[colorIndex]),  QSize(m_pData->m_lineSymboleSize,m_pData->m_lineSymboleSize)));
            }

            switch(m_qwtCurveStyle)
            {
                case Itom1DQwt::NoCurve:
                    dObjCurve->setStyle(QwtPlotCurve::NoCurve);
                break;
                default:
                case Itom1DQwt::Lines:
                    dObjCurve->setCurveAttribute(QwtPlotCurve::Fitted, false);
                    dObjCurve->setStyle(QwtPlotCurve::Lines);
                    break;
                case Itom1DQwt::FittedLines:
                    dObjCurve->setCurveAttribute(QwtPlotCurve::Fitted, true);
                    dObjCurve->setStyle(QwtPlotCurve::Lines);
                break;
                case Itom1DQwt::StepsLeft:
                    dObjCurve->setOrientation(Qt::Horizontal);
                    dObjCurve->setCurveAttribute(QwtPlotCurve::Inverted, false);
                    dObjCurve->setStyle(QwtPlotCurve::Steps);
                break;
                case Itom1DQwt::StepsRight:
                    dObjCurve->setOrientation(Qt::Horizontal);
                    dObjCurve->setCurveAttribute(QwtPlotCurve::Inverted, true);
                    dObjCurve->setStyle(QwtPlotCurve::Steps);
                break;
                case Itom1DQwt::Steps:
                    dObjCurve->setOrientation(Qt::Horizontal);
                    dObjCurve->setCurveAttribute(QwtPlotCurve::Inverted, false);
                    dObjCurve->setStyle(QwtPlotCurve::UserCurve );
                break;
                case Itom1DQwt::SticksHorizontal:
                    dObjCurve->setOrientation(Qt::Horizontal);
                    dObjCurve->setStyle(QwtPlotCurve::Sticks);
                break;

                case Itom1DQwt::Sticks:
                case Itom1DQwt::SticksVertical:
                    dObjCurve->setOrientation(Qt::Vertical);
                    dObjCurve->setStyle(QwtPlotCurve::Sticks);
                break;
                case Itom1DQwt::Dots:
                    dObjCurve->setStyle(QwtPlotCurve::Dots);
                break;
            }

            dObjCurve->setBaseline(m_baseLine);
            dObjCurve->setCurveFilled(m_curveFilled);
            if(m_curveFilled != Itom1DQwt::NoCurveFill)
            {
                if(m_filledColor.isValid())
                {
                    dObjCurve->setBrush(QBrush(m_filledColor));
                }
                else
                {
                    QColor fill = m_colorList[colorIndex];
                    fill.setAlpha(m_fillCurveAlpa);
                    dObjCurve->setBrush(QBrush(fill));
                }
            }
            else
            {
                dObjCurve->setBrush(Qt::NoBrush);
            }
            m_plotCurveItems.append(dObjCurve);
        }

        if (bounds.size() == 0)
        {
            QVector<QPointF> pts(2);

            switch(multiLineMode)
            {
            case Itom1DQwt::MultiLayerCols:
            case Itom1DQwt::FirstCol:
            case Itom1DQwt::MultiCols:
                if(m_layerState)
                {
                    pts.resize(3);
                    pts[1].setY(dataObj->getPixToPhys(dims-2, 0, _unused));
                    pts[2].setY(dataObj->getPixToPhys(dims-2, height-1, _unused)); 
                }
                else
                {
                    pts[0].setY(dataObj->getPixToPhys(dims-2, 0, _unused));
                    pts[1].setY(dataObj->getPixToPhys(dims-2, height-1, _unused)); 
                }

                for (int n = 0; n < numCurves; n++)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[n]->data());
                    if(m_layerState)
                    {
                        pts[0].setX(n);
                        pts[0].setY(n);
                        pts[1].setX(dataObj->getPixToPhys(dims-1, 0, _unused));
                        pts[2].setX(dataObj->getPixToPhys(dims-1, 0, _unused));
                    }
                    else
                    {
                        pts[0].setX(dataObj->getPixToPhys(dims-1, m_colorState ? 0 : n, _unused));
                        pts[1].setX(dataObj->getPixToPhys(dims-1, m_colorState ? 0 : n, _unused));                    
                    }

                    if (seriesData && seriesData->isDobjInit())
                    {
                        seriesData->updateDataObject(dataObj, pts);
                    }
                    else
                    {
                        seriesData = new DataObjectSeriesData(1);
                        seriesData->updateDataObject(dataObj, pts);
                        m_plotCurveItems[n]->setData(seriesData);
                    }
                    if(m_colorState)
                    {
                        if(m_pData->m_colorLine == Itom1DQwt::Gray) seriesData->setColorState(DataObjectSeriesData::grayColor);
                        else if(m_pData->m_colorLine == Itom1DQwt::RGB || m_pData->m_colorLine == Itom1DQwt::RGBA) seriesData->setColorState(n);
                        else if(m_pData->m_colorLine == Itom1DQwt::RGBGray) seriesData->setColorState(n == 3 ? 4 : n);
                    }
                }

                if (numCurves > 0)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
                    m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel();
                    m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel();
                }
                break;

            case Itom1DQwt::AutoRowCol:
            case Itom1DQwt::MultiLayerAuto:
            case Itom1DQwt::MultiLayerRows:
            case Itom1DQwt::FirstRow:
            case Itom1DQwt::MultiRows:

                if(m_layerState)
                {
                    pts.resize(3);
                    pts[1].setX(dataObj->getPixToPhys(dims-1, 0, _unused));
                    pts[2].setX(dataObj->getPixToPhys(dims-1, width-1, _unused));
                }
                else
                {
                    pts[0].setX(dataObj->getPixToPhys(dims-1, 0, _unused));
                    pts[1].setX(dataObj->getPixToPhys(dims-1, width-1, _unused));
                }


                for (int n = 0; n < numCurves; n++)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[n]->data());

                    if(m_layerState)
                    {
                        pts[0].setX(n);
                        pts[0].setY(n);
                        pts[1].setY(dataObj->getPixToPhys(dims-2, 0, _unused));
                        pts[2].setY(dataObj->getPixToPhys(dims-2, 0, _unused));
                    }
                    else
                    {
                        pts[0].setY(dataObj->getPixToPhys(dims-2, m_colorState ? 0 : n, _unused));
                        pts[1].setY(dataObj->getPixToPhys(dims-2, m_colorState ? 0 : n, _unused));               
                    }

                    if (seriesData && seriesData->isDobjInit())
                    {
                        seriesData->updateDataObject(dataObj, pts);
                    }
                    else
                    {
                        seriesData = new DataObjectSeriesData(1);
                        seriesData->updateDataObject(dataObj, pts);
                        m_plotCurveItems[n]->setData(seriesData);
                    }

                    if(m_colorState)
                    {
                        if(m_pData->m_colorLine == Itom1DQwt::Gray) seriesData->setColorState(DataObjectSeriesData::grayColor);
                        else if(m_pData->m_colorLine == Itom1DQwt::RGB || m_pData->m_colorLine == Itom1DQwt::RGBA) seriesData->setColorState(n);
                        else if(m_pData->m_colorLine == Itom1DQwt::RGBGray) seriesData->setColorState(n == 3 ? 4 : n);
                    }
                }

                if (numCurves > 0)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
                    m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel();
                    m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel();
                }
                break;
            } 
        }
        else if (bounds.size() > 1 && bounds.size() < 4) //boundaries given ->line plot
        {
            QVector<QPointF> tmpBounds = bounds;
            for (int n = 0; n < numCurves; n++)
            {
                seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[n]->data());
                if(tmpBounds.size() == 3 && m_layerState)
                {
                    tmpBounds[0].setX(n);
                    tmpBounds[0].setY(n);
                }
                if (seriesData && seriesData->isDobjInit())
                {
                    seriesData->updateDataObject(dataObj, tmpBounds);
                }
                else
                {
                    seriesData = new DataObjectSeriesData(1);
                    seriesData->updateDataObject(dataObj, tmpBounds);
                    m_plotCurveItems[n]->setData(seriesData);
                }
                if(m_colorState)
                {
                    if(m_pData->m_colorLine == Itom1DQwt::Gray) seriesData->setColorState(DataObjectSeriesData::grayColor);
                    else if(m_pData->m_colorLine == Itom1DQwt::RGB || m_pData->m_colorLine == Itom1DQwt::RGBA) seriesData->setColorState(n);
                    else if(m_pData->m_colorLine == Itom1DQwt::RGBGray) seriesData->setColorState(n == 3 ? 4 : n);
                }
            }

            if (numCurves > 0)
            {
                seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
                m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel();
                m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel();
            }
        }
        else if (bounds.size() == 1) //point in third dimension
        {
            for (int n = 0; n < numCurves; n++)
            {
                seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[n]->data());
                if (seriesData && seriesData->isDobjInit())
                {
                    seriesData->updateDataObject(dataObj, bounds);
                }
                else
                {
                    seriesData = new DataObjectSeriesData(1);
                    seriesData->updateDataObject(dataObj, bounds);
                    m_plotCurveItems[n]->setData(seriesData);
                }

                if(m_colorState)
                {
                    if(m_pData->m_colorLine == Itom1DQwt::Gray) seriesData->setColorState(DataObjectSeriesData::grayColor);
                    else if(m_pData->m_colorLine == Itom1DQwt::RGB || m_pData->m_colorLine == Itom1DQwt::RGBA) seriesData->setColorState(n);
                    else if(m_pData->m_colorLine == Itom1DQwt::RGBGray) seriesData->setColorState(n == 3 ? 4 : n);
                }
            }
            if (numCurves > 0)
            {
                seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
                m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel();
                m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel();
            }
        }

        bool valid;
        ito::DataObjectTagType tag;
        tag = dataObj->getTag("title", valid);
        m_pData->m_titleDObj = valid? QString::fromLatin1(tag.getVal_ToString().data()) : "";

    } 

    updateLabels();

    if (seriesData)
    {
        QByteArray hash = seriesData->getHash();

        if (hash != m_hash)
        {
            updatePickerPosition(true);

            QRectF rect = seriesData->boundingRect();
            if (m_pData->m_valueScaleAuto)
            {
                if (qIsFinite(rect.height()))
                {
                    m_pData->m_valueMin = rect.top();
                    m_pData->m_valueMax = rect.bottom();
                }
                else
                {
                    m_pData->m_valueMin = -0.01;
                    m_pData->m_valueMax = 0.01;
                }
            }

            if (m_pData->m_axisScaleAuto)
            {
                m_pData->m_axisMin = rect.left();
                m_pData->m_axisMax = rect.right();
            }

            updateScaleValues(); //replot is done here

            m_pZoomer->setZoomBase(true);
        }
        else if (m_pData->m_forceValueParsing)
        {
            updatePickerPosition(true);


            QRectF rect = seriesData->boundingRect();
            if (m_pData->m_valueScaleAuto)
            {
                m_pData->m_valueMin = rect.top();
                m_pData->m_valueMax = rect.bottom();
            }

            if (m_pData->m_axisScaleAuto)
            {
                m_pData->m_axisMin = rect.left();
                m_pData->m_axisMax = rect.right();
            }

            updateScaleValues(); //replot is done here

            m_pData->m_forceValueParsing = false;
        }
        else
        {
            updatePickerPosition(true,false);

            replot();
        }

        m_hash = hash;
    }
    else
    {
        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::keyPressEvent (QKeyEvent * event)
{
    event->ignore();

    Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
    Picker *m;
    int curves = m_plotCurveItems.size();

    if (m_pData->m_state == statePicker)
    {
        event->accept();

        switch(event->key())
        {
        case Qt::Key_Left:
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                m = &(m_pickers[i]);
                if (m->active)
                {
                    stickPickerToXPx(m, m->item->xValue(), -1);
                }
            }
            break;
        case Qt::Key_Right:
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                m = &(m_pickers[i]);
                if (m->active)
                {
                     stickPickerToXPx(m, m->item->xValue(), 1);
                }
            }
            break;
        case Qt::Key_Up:
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                m = &(m_pickers[i]);
                if (m->active)
                {
                    m->curveIdx++;
                    if (m->curveIdx >= curves) m->curveIdx = 0;
                    stickPickerToXPx(m, m->item->xValue(), 0);
                }
            }
            break;
        case Qt::Key_Down:
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                m = &(m_pickers[i]);
                if (m->active)
                {
                    m->curveIdx--;
                    if (m->curveIdx <= 0) m->curveIdx = curves-1;
                    stickPickerToXPx(m, m->item->xValue(), 0);
                }
            }
            break;
        case Qt::Key_Delete:
        {
            QList<Picker>::iterator it = m_pickers.begin();

            while (it != m_pickers.end())
            {
                if (it->active)
                {
                    it->item->detach();
                    delete it->item;
                    it = m_pickers.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            break;
        }
        default:
            event->ignore();
            break;
        }

        if (event->isAccepted())
        {
            updatePickerPosition(false,false);
        }
    }

    if(event->isAccepted() == false && event->matches(QKeySequence::Copy))
    {
        p->copyToClipBoard();
    }

    
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mousePressEvent (QMouseEvent * event)
{
    

    if (m_pData->m_state == stateIdle)
    {
        //int n;
        QHash<int, DrawItem*>::iterator it = m_pData->m_pDrawItems.begin();
        for (;it != m_pData->m_pDrawItems.end(); ++it)
//        for (n = 0; n < m_pData->m_pDrawItems.size(); n++)
        {
            if (it.value() == NULL)
            {
                continue;
            }

            int canxpos = event->x() - canvas()->x();
            int canypos = event->y() - canvas()->y();
//            double x = it.value()->x1;
//            double y = it.value()->y1;
            //double xx = transform(QwtPlot::xBottom, it.value()->x1);
            //double yy = transform(QwtPlot::xBottom, it.value()->y1);
            if (fabs(transform(QwtPlot::xBottom, it.value()->x1) - canxpos) < 10
                && fabs(transform(QwtPlot::yLeft, it.value()->y1) - canypos) < 10)
            {
                it.value()->m_active = 1;
                m_activeDrawItem = it.value()->m_idx;
                it.value()->setActive(1);
                it.value()->setSelected(true);
                ++it;
                break;
            }
            else if (fabs(transform(QwtPlot::xBottom, it.value()->x2) - canxpos) < 10
                && fabs(transform(QwtPlot::yLeft, it.value()->y2) - canypos) < 10)
            {
                it.value()->m_active = 2;
                m_activeDrawItem = it.value()->m_idx;
                it.value()->setActive(2);
                it.value()->setSelected(true);
                ++it;
                break;
            }
            else
            {
                it.value()->setSelected(false);
                it.value()->setActive(0);
            }
        }
//        for (n++; n < m_pData->m_pDrawItems.size(); n++)
        for (;it != m_pData->m_pDrawItems.end(); ++it)
        {
            if (it.value() == NULL)
            {
                continue;
            }
            it.value()->setSelected(false);
            it.value()->setActive(0);
        }
        replot();
    }
    else if (m_pData->m_state == statePicker)
    {
        int xPx = m_pValuePicker->trackerPosition().x();
        int yPx = m_pValuePicker->trackerPosition().y();
        double xScale = invTransform(xBottom, xPx);
//        double yScale = invTransform(yLeft, yPx);
        bool closeToPicker = false;

        if (event->button() == Qt::LeftButton)
        {
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                if (abs(transform(xBottom, m_pickers[i].item->xValue()) - xPx) < 20 && abs(transform(yLeft, m_pickers[i].item->yValue()) - yPx) < 20)
                {
                    closeToPicker = true;
                    m_pickers[i].active = true;
                    
                }
                else if ((event->modifiers() & Qt::ControlModifier) == false && m_pickers[i].active)
                {
                    m_pickers[i].active = false;
                    //m_pickers[i].item->setSymbol(new QwtSymbol(QwtSymbol::Diamond, m_pickers[i].color, QPen(m_pickers[i].color,1), QSize(6,6)));
                }
            }

            if (!closeToPicker && m_plotCurveItems.size() > 0 && m_pickers.size() < m_pData->m_pickerLimit)
            {
                Picker picker;
                picker.item = new ItomPlotMarker(m_pData->m_pickerLabelVisible, 
                                                 m_pData->m_pickerType, 
                                                 m_pData->m_pickerLabelAlignment, 
                                                 m_pData->m_pickerLabelOrientation );
                picker.item->attach(this);
                picker.active = true;
                //marker.color = Qt::darkGreen;
                //marker.item->setSymbol(new QwtSymbol(QwtSymbol::Diamond,QBrush(Qt::white), QPen(marker.color,1),  QSize(8,8)));
                
                picker.curveIdx = 0;
                stickPickerToXPx(&picker, xScale, 0);

                picker.item->setVisible(true);
                
                m_pickers.append(picker);
            }

            updatePickerPosition(false,false);

            replot();
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mouseMoveEvent (QMouseEvent * event)
{
//    Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());

    if(m_ignoreNextMouseEvent)
    {
        m_ignoreNextMouseEvent = false;
        return;
    }

    if (m_pData->m_state == statePicker)
    {
        int xPx = m_pValuePicker->trackerPosition().x();
//        int yPx = m_pValuePicker->trackerPosition().y();
        double xScale = invTransform(xBottom, xPx);
//        double yScale = invTransform(yLeft, yPx);

        if (event->buttons() & Qt::LeftButton)
        {
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                if (m_pickers[i].active == true)
                {
                    stickPickerToXPx(&m_pickers[i], xScale, 0);
                }
            }
            updatePickerPosition(false,false);

            replot();
        } 
    }
    else if (m_pData->m_state == stateIdle)
    {
        ito::float32 canxpos = invTransform(QwtPlot::xBottom, event->x() - canvas()->x());
        ito::float32 canypos = invTransform(QwtPlot::yLeft, event->y() - canvas()->y());

        QHash<int, DrawItem*>::Iterator it = m_pData->m_pDrawItems.begin();
        for (; it != m_pData->m_pDrawItems.end(); it++)
//        for (int n = 0; n < m_pData->m_pDrawItems.size(); n++)
        {
//            if (m_pData->m_pDrawItems[n]->m_active == 1)

            if (it.value() == NULL)
            {
                continue;
            }

            if (it.value()->m_active == 1)
            {
                ito::float32 dx, dy;

                QPainterPath path;
                switch (it.value()->m_type)
                {
                    case tPoint:
                        path.moveTo(canxpos, canypos);
                        path.lineTo(canxpos, canypos);
                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);
                        replot();
                    break;

                    case tLine:
                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                        {
                            dx = canxpos - it.value()->x2;
                            dy = it.value()->y2 - canypos;

                            dx = fabs(dx) <= std::numeric_limits<ito::float32>::epsilon() ? std::numeric_limits<ito::float32>::epsilon() : dx;
                            dy = fabs(dy) <= std::numeric_limits<ito::float32>::epsilon() ? - std::numeric_limits<ito::float32>::epsilon() : dy;

                            if (fabs(dx) > fabs(dy))
                            {
                                path.moveTo(canxpos, it.value()->y2);
                                path.lineTo(it.value()->x2, it.value()->y2);  
                            }
                            else
                            {
                                path.moveTo(it.value()->x2, canypos);
                                path.lineTo(it.value()->x2, it.value()->y2);  
                            }
                        }
                        else
                        {
                            path.moveTo(canxpos, canypos);
                            path.lineTo(it.value()->x2, it.value()->y2);                        
                        }
                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);
                        replot();
                    break;

                    case tRect:
                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                        {
                            dx = it.value()->x2 - canxpos;
                            dy = canypos - it.value()->y2;

                            dx = fabs(dx) <= std::numeric_limits<ito::float32>::epsilon() ? std::numeric_limits<ito::float32>::epsilon() : dx;
                            dy = fabs(dy) <= std::numeric_limits<ito::float32>::epsilon() ? - std::numeric_limits<ito::float32>::epsilon() : dy;

                            if (fabs(dx) < fabs(dy))
                            {
                                //canypos = it.value()->x2 - dx;
                                canypos = it.value()->y2 + dx * dx / fabs(dx) * dy / fabs(dy);
                            }
                            else
                            {
                                //canxpos = it.value()->x2 - dy;
                                canxpos = it.value()->x2 - dy * dx / fabs(dx) * dy / fabs(dy);
                            }

                            m_ignoreNextMouseEvent = true;
                        }

                        path.addRect(canxpos, canypos,
                            it.value()->x2 - canxpos,
                            it.value()->y2 - canypos);
                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);

                        if(m_ignoreNextMouseEvent)
                        {
                            ito::float32 destPosX = transform(QwtPlot::xBottom, canxpos);
                            ito::float32 destPosY = transform(QwtPlot::yLeft, canypos);
 
                            QPoint dst = canvas()->mapToGlobal(QPoint(destPosX, destPosY));

                            this->cursor().setPos(dst);
                        }
                        replot();
                    break;

                    case tEllipse:
                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                        {
                            dx = it.value()->x2 - canxpos;
                            dy = canypos - it.value()->y2;

                            dx = fabs(dx) <= std::numeric_limits<ito::float32>::epsilon() ? std::numeric_limits<ito::float32>::epsilon() : dx;
                            dy = fabs(dy) <= std::numeric_limits<ito::float32>::epsilon() ? - std::numeric_limits<ito::float32>::epsilon() : dy;

                            if (fabs(dx) < fabs(dy))
                            {
                                //canypos = it.value()->x2 - dx;
                                canypos = it.value()->y2 + dx * dx / fabs(dx) * dy / fabs(dy);
                            }
                            else
                            {
                                //canxpos = it.value()->x2 - dy;
                                canxpos = it.value()->x2 - dy * dx / fabs(dx) * dy / fabs(dy);
                            }
                            m_ignoreNextMouseEvent = true;
                        }
                        path.addEllipse(canxpos,
                            canypos,
                             it.value()->x2 - canxpos,
                             it.value()->y2 - canypos);
                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);

                        if(m_ignoreNextMouseEvent)
                        {
                            ito::float32 destPosX = transform(QwtPlot::xBottom, canxpos);
                            ito::float32 destPosY = transform(QwtPlot::yLeft, canypos);
 
                            QPoint dst = canvas()->mapToGlobal(QPoint(destPosX, destPosY));

                            this->cursor().setPos(dst);
                        }
                        replot();
                    break;
                }

                break;
            }
            else if (it.value()->m_active == 2)
            {
                ito::float32 dx, dy;

                QPainterPath path;
                switch (it.value()->m_type)
                {
                    case tLine:
                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                        {
                            dx = canxpos - it.value()->x1;
                            dy = it.value()->y1 - canypos;

                            if (fabs(dx) > fabs(dy))
                            {
                                path.moveTo(it.value()->x1, it.value()->y1);
                                path.lineTo(canxpos, it.value()->y1);
                            }
                            else
                            {
                                path.moveTo(it.value()->x1, it.value()->y1);
                                path.lineTo(it.value()->x1, canypos);
                            }
                        }
                        else
                        {
                            path.moveTo(it.value()->x1, it.value()->y1);
                            path.lineTo(canxpos, canypos);
                        }

                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);
                        //if (p) emit p->plotItemChanged(n);
                        replot();
                    break;

                    case tRect:
                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                        {
                            dx = canxpos - it.value()->x1;
                            dy = it.value()->y1 - canypos;

                            dx = fabs(dx) <= std::numeric_limits<ito::float32>::epsilon() ? std::numeric_limits<ito::float32>::epsilon() : dx;
                            dy = fabs(dy) <= std::numeric_limits<ito::float32>::epsilon() ? - std::numeric_limits<ito::float32>::epsilon() : dy;

                            if (fabs(dx) < fabs(dy))
                            {
                                //canypos = it.value()->y1 + dx;
                                canypos = it.value()->y1 - dx * dx / fabs(dx) * dy / fabs(dy);
                            }
                            else
                            {
                                //canypos = it.value()->x1 + dy;
                                canxpos = it.value()->x1 + dy * dx / fabs(dx) * dy / fabs(dy);
                            }
                            m_ignoreNextMouseEvent = true;
                        }
                        path.addRect(it.value()->x1, it.value()->y1,
                            canxpos - it.value()->x1,
                            canypos - it.value()->y1);
                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);
                        //if (p) emit p->plotItemChanged(n);
                        if(m_ignoreNextMouseEvent)
                        {
                            ito::float32 destPosX = transform(QwtPlot::xBottom, canxpos);
                            ito::float32 destPosY = transform(QwtPlot::yLeft, canypos);
 
                            QPoint dst = canvas()->mapToGlobal(QPoint(destPosX, destPosY));

                            this->cursor().setPos(dst);
                        }
                        replot();
                    break;

                    case tEllipse:
                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                        {
                            dx = canxpos - it.value()->x1;
                            dy = it.value()->y1 - canypos;

                            dx = fabs(dx) <= std::numeric_limits<ito::float32>::epsilon() ? std::numeric_limits<ito::float32>::epsilon() : dx;
                            dy = fabs(dy) <= std::numeric_limits<ito::float32>::epsilon() ? - std::numeric_limits<ito::float32>::epsilon() : dy;

                            if (fabs(dx) < fabs(dy))
                            {
                                //canypos = it.value()->y1 + dx;
                                canypos = it.value()->y1 - dx * dx / fabs(dx) * dy / fabs(dy);
                            }
                            else
                            {
                                //canypos = it.value()->x1 + dy;
                                canxpos = it.value()->x1 + dy * dx / fabs(dx) * dy / fabs(dy);
                            }
                            m_ignoreNextMouseEvent = true;
                            
                        }
                        path.addEllipse(it.value()->x1,
                            it.value()->y1,
                            canxpos - it.value()->x1,
                            canypos - it.value()->y1),
                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);
                        //if (p) emit p->plotItemChanged(n);
                        if(m_ignoreNextMouseEvent)
                        {
                            ito::float32 destPosX = transform(QwtPlot::xBottom, canxpos);
                            ito::float32 destPosY = transform(QwtPlot::yLeft, canypos);
 
                            QPoint dst = canvas()->mapToGlobal(QPoint(destPosX, destPosY));

                            this->cursor().setPos(dst);
                        }
                        replot();
                    break;
                }
                break;
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mouseReleaseEvent (QMouseEvent * event)
{
    Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
    if (m_pData->m_state == tEllipse || m_pData->m_state == tRect || m_pData->m_state == tLine
        || m_pData->m_state == tPoint || m_pData->m_state == stateIdle)
    {
        QHash<int, DrawItem*>::iterator it = m_pData->m_pDrawItems.begin();
        for (;it != m_pData->m_pDrawItems.end(); ++it)        
//        for (int n = 0; n < m_pData->m_pDrawItems.size(); n++)
        {
            if (it.value() != NULL && it.value()->m_active != 0 && p)
            {
                int type = 0;
                QVector<ito::float32> values;
                values.reserve(11);
                switch(it.value()->m_type)
                {
                    case tPoint:
                        type = ito::PrimitiveContainer::tPoint;
                        values.append(it.value()->x1);
                        values.append(it.value()->y1);
                        values.append(0.0);
                    break;

                    case tLine:
                        type = ito::PrimitiveContainer::tLine;
                        values.append(it.value()->x1);
                        values.append(it.value()->y1);
                        values.append(0.0);
                        values.append(it.value()->x2);
                        values.append(it.value()->y2);
                        values.append(0.0);
                    break;

                    // square is a rect
//                    case tSquare:
                    case tRect:
                        type = ito::PrimitiveContainer::tRectangle;
                        values.append(it.value()->x1);
                        values.append(it.value()->y1);
                        values.append(0.0);
                        values.append(it.value()->x2);
                        values.append(it.value()->y2);
                        values.append(0.0);
                    break;

                    // circle is an ellispe
//                    case tCircle:
                    case tEllipse:
                        type = ito::PrimitiveContainer::tEllipse;
                        values.append((it.value()->x1 + it.value()->x2)*0.5);
                        values.append((it.value()->y1 + it.value()->y2)*0.5);
                        values.append(0.0);
                        values.append(abs(it.value()->x1 - it.value()->x2)*0.5);
                        values.append(abs(it.value()->y1 - it.value()->y2)*0.5);
                        values.append(0.0);
                    break;

/*
                    case tCircle:
                        type = ito::PrimitiveContainer::tCircle;
                        values.append((it.value()->x1 + it.value()->x2)*0.5);
                        values.append((it.value()->y1 + it.value()->y2)*0.5);
                        values.append(0.0);
                        values.append(abs(it.value()->x1 - it.value()->x2)*0.5);
                        values.append(0.0);
                    break;
*/
/*
                    case tSquare:
                        type = ito::PrimitiveContainer::tSquare;
                        values.append((it.value()->x1 + it.value()->x2)*0.5);
                        values.append((it.value()->y1 + it.value()->y2)*0.5);
                        values.append(0.0);
                        values.append(abs(it.value()->x1 - it.value()->x2)*0.5);
                        values.append(0.0);
                    break;
*/
                }

                emit p->plotItemChanged(it.value()->m_idx, type, values);
            }
            if (it.value())
            {
                it.value()->m_active = 0;
                it.value()->setActive(0);
            }
        }
    }
    else if (m_pData->m_state == statePicker)
    {
        int xPx = m_pValuePicker->trackerPosition().x();
//        int yPx = m_pValuePicker->trackerPosition().y();
        double xScale = invTransform(xBottom, xPx);
//        double yScale = invTransform(yLeft, yPx);
//        bool closeToPicker = false;

        if (event->button() == Qt::LeftButton)
        {
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                if (m_pickers[i].active == true)
                {
                    stickPickerToXPx(&m_pickers[i], xScale, 0);
                }
            }

            updatePickerPosition(false,false);

            replot();
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setMainPickersToIndex(int idx1, int idx2, int curveIdx)
{
    while (m_pickers.size() < 2)
    {
        //prepend
        Picker picker;
        picker.item = new ItomPlotMarker(m_pData->m_pickerLabelVisible, 
                                                 m_pData->m_pickerType, 
                                                 m_pData->m_pickerLabelAlignment, 
                                                 m_pData->m_pickerLabelOrientation );
        picker.item->attach(this);
        picker.active = false;
                
        picker.curveIdx = curveIdx;
        picker.item->setVisible(true);
                
        m_pickers.prepend(picker);
    }

    for (int i = 0; i < m_pickers.size() ; ++i)
    {
        if (i == 0)
        {
            m_pickers[0].active = true;
            stickPickerToSampleIdx(&(m_pickers[0]), idx1, curveIdx, 0);
        }
        else if (i == 1)
        {
            m_pickers[1].active = false;
            stickPickerToSampleIdx(&(m_pickers[1]), idx2, curveIdx, 0);
        }
        else
        {
            m_pickers[i].active = false;
        }
    }

    updatePickerPosition(false,false);

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::stickPickerToXPx(Picker *m, double xScaleStart, int dir) //dir: 0: this point, -1: next valid to the left or this if not possible, 1: next valid to the right or this if not possible
{
    DataObjectSeriesData *data = (DataObjectSeriesData*)(m_plotCurveItems[m->curveIdx]->data());

    if (!qIsFinite(xScaleStart)) xScaleStart = m->item->xValue();

    int thisIdx = data->getPosToPix(xScaleStart);
    int s = (int)data->size();
    QPointF p;
    bool found = false;
    bool d = true;

    if (dir == 0)
    {
        if (thisIdx < 0)
        {
            thisIdx = 0;
        }
        else if (thisIdx >= s)
        {
            thisIdx = s-1;
        }

        while (!found)
        {
            if (thisIdx >= 0 && thisIdx < s)
            {
                p = data->sample(thisIdx);
                if (qIsFinite(p.ry()))
                {
                    m->item->setXValue(p.rx());
                    m->item->setYValue(p.ry());
                    found = true;
                }
            }
            else
            {
                break;
            }

            if (d) //iteratively search for the next valid point at the left or right of thisIdx
            {
                thisIdx = -thisIdx + 1;
                d = !d;
            }
            else
            {
                thisIdx = -thisIdx;
                d = !d;
            }
        }
    }
    else if (dir == -1)
    {
        if (thisIdx <= 0)
        {
            thisIdx = 1; //1 since it is decremented to 0 afterwards
        }
        else if (thisIdx > s)
        {
            thisIdx = s;
        }

        while (!found)
        {
            thisIdx -= 1;
            if (thisIdx >= 0)
            {
                p = data->sample(thisIdx);
                if (qIsFinite(p.ry()))
                {
                    m->item->setXValue(p.rx());
                    m->item->setYValue(p.ry());
                    found = true;
                }
                
            }
            else
            {
                break;
            }
        }
    }
    else //dir > 0
    {
        if (thisIdx <= -1)
        {
            thisIdx = -1; //-1 since it is incremented to 0 afterwards
        }
        else if (thisIdx > (s-2))
        {
            thisIdx = s-2;
        }

        while (!found)
        {
            thisIdx += 1;
            if (thisIdx >= 0 && thisIdx < s)
            {
                p = data->sample(thisIdx);
                if (qIsFinite(p.ry()))
                {
                    m->item->setXValue(p.rx());
                    m->item->setYValue(p.ry());
                    found = true;
                }
            }
            else
            {
                break;
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::stickPickerToSampleIdx(Picker *m, int idx, int curveIdx, int dir)
{
    DataObjectSeriesData *data = (DataObjectSeriesData*)(m_plotCurveItems[curveIdx]->data());

    int thisIdx = idx;
    int s = (int)data->size();
    QPointF p;
    bool found = false;
    bool d = true;

    if (dir == 0)
    {
        while (!found)
        {
            if (thisIdx >= 0 && thisIdx < s)
            {
                p = data->sample(thisIdx);
                if (qIsFinite(p.ry()))
                {
                    m->item->setXValue(p.rx());
                    m->item->setYValue(p.ry());
                    found = true;
                }
            }
            else
            {
                break;
            }

            if (d)
            {
                thisIdx = -thisIdx + 1;
                d = !d;
            }
            else
            {
                thisIdx = -thisIdx;
                d = !d;
            }
        }
    }
    else if (dir == -1)
    {
        while (!found)
        {
            thisIdx -= 1;
            if (thisIdx >= 0)
            {
                p = data->sample(thisIdx);
                if (qIsFinite(p.ry()))
                {
                    m->item->setXValue(p.rx());
                    m->item->setYValue(p.ry());
                    found = true;
                }
            }
            else
            {
                break;
            }
        }
    }
    else //dir > 0
    {
        while (!found)
        {
            thisIdx += 1;
            if (thisIdx >= 0 && thisIdx < s)
            {
                p = data->sample(thisIdx);
                if (qIsFinite(p.ry()))
                {
                    m->item->setXValue(p.rx());
                    m->item->setYValue(p.ry());
                    found = true;
                }
            }
            else
            {
                break;
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::contextMenuEvent(QContextMenuEvent * event)
{
    if (m_showContextMenu && m_pPanner->isEnabled() == false)
    {
        event->accept();
        m_contextMenu->exec(event->globalPos());
    }
    else
    {
        event->ignore();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue)
{
    bool recalculateBoundaries = false;
    switch(axis)
    {
        case Qt::YAxis:
            if (autoCalcLimits) 
            {
                m_pData->m_valueScaleAuto = true;
                recalculateBoundaries = true;
            }
            else
            {
                m_pData->m_valueScaleAuto = false;
                m_pData->m_valueMin = minValue;
                m_pData->m_valueMax = maxValue;
            }
        break;
        case Qt::XAxis:
            if (autoCalcLimits) 
            {
                m_pData->m_axisScaleAuto = true;
                recalculateBoundaries = true;
            }
            else
            {
                m_pData->m_axisScaleAuto = false;
                m_pData->m_axisMin = minValue;
                m_pData->m_axisMax = maxValue;                        
            }
        break;
    }

    updateScaleValues(recalculateBoundaries); //replot is done here

    return retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setZoomerEnable(const bool checked)
{
    if (checked)
    {
        setPickerEnable(false);
        setPannerEnable(false);

        m_pData->m_state = stateZoomer;

        m_pPanner->setEnabled(false);

        DataObjectSeriesData *data = NULL;

        foreach(QwtPlotCurve *curve, m_plotCurveItems)
        {
            data = (DataObjectSeriesData *)curve->data();
            m_pZoomer->setZoomBase(data->boundingRect());
        }

        m_pZoomer->setEnabled(true);
        canvas()->setCursor(Qt::CrossCursor);
    }
    else
    {
        m_pData->m_state = stateIdle;

        m_pZoomer->setEnabled(false);
        canvas()->setCursor(Qt::ArrowCursor);

        /*foreach(QwtPlotCurve *curve, m_plotCurveItems)
        {
            setAxisAutoScale(curve->xAxis(),true);
            setAxisAutoScale(curve->yAxis(),true);
        }*/
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setPickerEnable(const bool checked)
{
    if (checked)
    {   
        setZoomerEnable(false);
        setPannerEnable(false);

        m_pData->m_state = statePicker;
        m_pValuePicker->setEnabled(true);
        canvas()->setCursor(Qt::CrossCursor);
    }
    else
    {
        m_pData->m_state = stateIdle;
        m_pValuePicker->setEnabled(false);
        canvas()->setCursor(Qt::ArrowCursor);

        /*foreach(Pciker m, m_pickers)
        {
            m.item->detach();
            delete m.item;
        }
        m_pickers.clear();*/
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setPannerEnable(const bool checked)
{
    if (checked)
    {
        setZoomerEnable(false);
        setPickerEnable(false);
        m_pData->m_state = statePanner;
        canvas()->setCursor(Qt::OpenHandCursor);
    }
    else
    {
        m_pData->m_state = stateIdle;
        canvas()->setCursor(Qt::ArrowCursor);
    }
    m_pPanner->setEnabled(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::synchronizeCurrentScaleValues()
{
    QwtScaleDiv div = axisScaleDiv(QwtPlot::yLeft);
    m_pData->m_valueMin = div.interval().minValue();
    m_pData->m_valueMax = div.interval().maxValue();
    
    div = axisScaleDiv(QwtPlot::xBottom);
    m_pData->m_axisMin = div.interval().minValue();
    m_pData->m_axisMax = div.interval().maxValue();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::updateScaleValues(bool recalculateBoundaries /*= false*/)
{
    if (recalculateBoundaries)
    {
        QRectF rect;

        foreach(QwtPlotCurve *curve, m_plotCurveItems)
        {
            QRectF tmpRect = ((DataObjectSeriesData *)curve->data())->boundingRect();
            if (qIsFinite(tmpRect.height()))
            {
    #if QT_VERSION >= 0x050000
                rect = rect.united(((DataObjectSeriesData *)curve->data())->boundingRect());
    #else
                rect = rect.unite(((DataObjectSeriesData *)curve->data())->boundingRect());
    #endif
            }
        }

        if (m_pData->m_valueScaleAuto)
        {
            m_pData->m_valueMin = rect.top();
            m_pData->m_valueMax = rect.bottom();
        }

        if (m_pData->m_axisScaleAuto)
        {
            m_pData->m_axisMin = rect.left();
            m_pData->m_axisMax = rect.right();
        }
    }

    setAxisScale(QwtPlot::yLeft, m_pData->m_valueMin, m_pData->m_valueMax);
    setAxisScale(QwtPlot::xBottom, m_pData->m_axisMin, m_pData->m_axisMax);

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::updatePickerPosition(bool updatePositions, bool clear/* = false*/)
{
    if (clear)
    {
        foreach(Picker m, m_pickers)
        {
            m.item->detach();
            delete m.item;
        }
        m_pickers.clear();
    }

    QColor colors[3] = { Qt::red, Qt::darkGreen, Qt::darkGray };
    int cur = 0;
    Picker *m;
    QVector<QPointF> points;

    for (int i = 0 ; i < m_pickers.size() ; i++)
    {
        m = &(m_pickers[i]);
        if (updatePositions)
        {
            stickPickerToXPx(m, std::numeric_limits<double>::quiet_NaN() ,0);
        }

        if (m->active)
        {
            m_pickers[i].item->setSymbol(new QwtSymbol(QwtSymbol::Diamond, Qt::white, QPen(colors[cur],2), QSize(8,8)));
        }
        else
        {
            m_pickers[i].item->setSymbol(new QwtSymbol(QwtSymbol::Diamond, colors[cur], QPen(colors[cur],2), QSize(6,6)));
        }

        if (cur < 2) cur++;
        points << QPointF(m_pickers[i].item->xValue(), m_pickers[i].item->yValue());       
    }

    QString coords, offsets;
    if (points.size() > 1)
    {
        coords = QString("[%1; %2]\n [%3; %4]").arg(points[0].rx(),0,'g',4).arg(points[0].ry(),0,'g',4 ).arg(points[1].rx(),0,'g',4 ).arg(points[1].ry(),0,'g',4 );
        offsets = QString("dx = %1\n dy = %2").arg(points[1].rx() - points[0].rx(),0,'g',4).arg(points[1].ry() - points[0].ry(), 0, 'g', 4);
    }
    else if (points.size() == 1)
    {
        coords = QString("[%1; %2]\n      ").arg(points[0].rx(),0,'g',4).arg(points[0].ry(),0,'g',4 );
    }

    emit setPickerText(coords,offsets);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::configRescaler(void)
{
    //if (m_pData->m_keepAspect)
    //{
    //    //int height = plotLayout()->canvasRect().height();
    //    //int width = plotLayout()->canvasRect().width();

    //    int refAxis = plotLayout()->canvasRect().width() < plotLayout()->canvasRect().height() ? QwtPlot::xBottom : QwtPlot::yLeft;
    //    
    //    if (m_pRescaler == NULL)
    //    {
    //        QwtInterval curXInterVal = axisInterval(QwtPlot::xBottom);
    //        QwtInterval curYInterVal = axisInterval(QwtPlot::yLeft);

    //        m_pRescaler = new QwtPlotRescaler(canvas(), refAxis , QwtPlotRescaler::Fitting);
    //        m_pRescaler->setIntervalHint(QwtPlot::xBottom, curXInterVal);
    //        m_pRescaler->setIntervalHint(QwtPlot::yLeft, curYInterVal);
    //        m_pRescaler->setAspectRatio(1.0);
    //        m_pRescaler->setExpandingDirection(QwtPlot::xBottom, QwtPlotRescaler::ExpandUp);
    //        m_pRescaler->setExpandingDirection(QwtPlot::yLeft, QwtPlotRescaler::ExpandBoth);
    //        //m_pRescaler->setExpandingDirection(QwtPlot::yRight, QwtPlotRescaler::ExpandBoth);
    //    }
    //    else
    //    {

    //        m_pRescaler->setReferenceAxis(refAxis);
    //    }
    //    m_pRescaler->setEnabled(true);
    //}
    //else
    //{
    //    if (m_pRescaler != NULL)
    //    {
    //        m_pRescaler->setEnabled(false);
    //        //m_pRescaler->deleteLater();
    //        //m_pRescaler = NULL;
    //    }
    //    
    //}
    //if (m_pRescaler != NULL && m_pData->m_keepAspect)
    //{
    //    m_pRescaler->rescale();
    //}
    ////replot();
    m_pZoomer->setFixedAspectRatio(m_pData->m_keepAspect);

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::userInteractionStart(int type, bool start, int maxNrOfPoints)
{
    ito::RetVal retval;

    m_drawedIemsIndexes.clear();
    m_pMultiPointPicker->selection().clear();

    if (type == tPoint) //multiPointPick
    {
        if (start)
        {
            m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::CrossRubberBand);
            MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());

            if (m)
            {
                if (m_pData)
                {
                    m_pData->m_elementsToPick = 1;
                }
                m->setMaxNrItems(maxNrOfPoints);
                m_pMultiPointPicker->setEnabled(true);

                if (maxNrOfPoints > 0)
                {
                    emit statusBarMessage(tr("Please select %1 points or press Space to quit earlier. Esc aborts the selection.").arg(maxNrOfPoints));
                }
                else
                {
                    emit statusBarMessage(tr("Please select points and press Space to end the selection. Esc aborts the selection."));
                }

                //QKeyEvent evt(QEvent::KeyPress, Qt::Key_M, Qt::NoModifier);
                //m_pMultiPointPicker->eventFilter(m_pMultiPointPicker->parent(), &evt); //starts the process
            }
            setState((Plot1DWidget::tState)type);
        }
        else //start == false
        {
            m_pMultiPointPicker->setEnabled(false);

            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            if (m_pData)
            {
                m_pData->m_elementsToPick = 0;
            }

            Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
            if (p)
            {
                QPolygonF polygonScale;
                emit p->userInteractionDone(type, true, polygonScale);
            }
            setState(stateIdle);
        }
    }
    else if (type == tLine)
    {
        if (start)
        {
            m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::PolygonRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());

            if (m)
            {
                if (m_pData)
                {
                    m_pData->m_elementsToPick = (maxNrOfPoints / 2);
                }

                m->setMaxNrItems(2);
                m_pMultiPointPicker->setEnabled(true);

                if (m_pData->m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 lines. Esc aborts the selection.").arg(m_pData->m_elementsToPick));
                else emit statusBarMessage(tr("Please draw one line. Esc aborts the selection."));
            }
            setState(tLine);
        }
        else //start == false
        {
            m_pMultiPointPicker->setEnabled(false);

            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            if (m_pData)
            {
                m_pData->m_elementsToPick = 0;
            }

            Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
            if (p)
            {
                QPolygonF polygonScale;
                emit p->userInteractionDone(type, true, polygonScale);
            }
            setState(stateIdle);
        }
    }
    else if (type == tRect)
    {
        if (start)
        {
            //maxNrOfPoints = 2;
            m_pMultiPointPicker->setStateMachine(new QwtPickerClickRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::RectRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
//            MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());

//            if (m)
            {
                if (m_pData)
                {
                    m_pData->m_elementsToPick = (maxNrOfPoints / 2);
                }
//                m->setMaxNrItems(2);
                m_pMultiPointPicker->setEnabled(true);

                if (m_pData->m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 rectangles. Esc aborts the selection.").arg(m_pData->m_elementsToPick));
                else emit statusBarMessage(tr("Please draw one rectangle. Esc aborts the selection."));
            }
            setState(tRect);
        }
        else //start == false
        {
            m_pMultiPointPicker->setEnabled(false);

            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            if (m_pData)
            {
                m_pData->m_elementsToPick = 0;
            }

            Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
            if (p)
            {
                QPolygonF polygonScale;
                emit p->userInteractionDone(type, true, polygonScale);
            }
            setState(stateIdle);
        }
    }
    else if (type == tEllipse)
    {
        if (start)
        {
            //maxNrOfPoints = 2;
            m_pMultiPointPicker->setStateMachine(new QwtPickerClickRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::EllipseRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
//            MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());

//            if (m)
            {
                if (m_pData)
                {
                    m_pData->m_elementsToPick = (maxNrOfPoints / 2);
                }
//                m->setMaxNrItems(2);
                m_pMultiPointPicker->setEnabled(true);

                if (m_pData->m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 ellipses. Esc aborts the selection.").arg(m_pData->m_elementsToPick));
                else emit statusBarMessage(tr("Please draw one ellipse. Esc aborts the selection."));
            }
            setState(tEllipse);
        }
        else //start == false
        {
            m_pMultiPointPicker->setEnabled(false);

            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            if (m_pData)
            {
                m_pData->m_elementsToPick = 0;
            }

            Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
            if (p)
            {
                QPolygonF polygonScale;
                emit p->userInteractionDone(type, true, polygonScale);
            }
            setState(stateIdle);
        }
    }
    else
    {
        Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
        if (p)
        {
            QPolygonF polygonScale;
            emit p->userInteractionDone(type, true, polygonScale);
        }
        retval += ito::RetVal(ito::retError,0,"Unknown type for userInteractionStart");
        setState(stateIdle);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setState(tState state)
{
    Itom1DQwtPlot *p = (Itom1DQwtPlot*)(parent());

    if (m_pData->m_state != state)
    {
        if ((m_pData->m_state == tPoint || m_pData->m_state == tLine || 
             m_pData->m_state == tRect  || m_pData->m_state == tEllipse) 
             && state != stateIdle)
        {
            return; //drawFunction needs to go back to idle
        }

        if (m_pZoomer)
        {
            m_pZoomer->setEnabled(state == stateZoomer);
            this->setZoomerEnable(state == stateZoomer);
        }
        if (m_pPanner) m_pPanner->setEnabled(state == statePanner);
        if (m_pValuePicker) m_pValuePicker->setEnabled(state == statePicker);

        if (m_pData->m_state == tPoint || m_pData->m_state == tLine
            || m_pData->m_state == tRect || m_pData->m_state == tEllipse || state == stateIdle)
        {
            if (p)
            {
                p->m_pActZoomToRect->setEnabled(state == stateIdle);
                p->m_pActMarker->setEnabled(state == stateIdle);
                p->m_pActPan->setEnabled(state == stateIdle);
                this->setZoomerEnable(state != stateIdle);
            }
        }

        switch (state)
        {
            default:
            case stateIdle:
                canvas()->setCursor(Qt::ArrowCursor);
            break;

            case stateZoomer:
                canvas()->setCursor(Qt::CrossCursor);
            break;

            case statePanner:
                canvas()->setCursor(Qt::OpenHandCursor);
            break;

            case statePicker:
                canvas()->setCursor(Qt::CrossCursor);
            break;

            case tPoint:
            case tLine:
            case tRect:
            case tEllipse:
                canvas()->setCursor(Qt::CrossCursor);
            break;
        }

        m_pData->m_state = state;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::multiPointActivated (bool on)
{
	if (m_pData)
	{
		switch(m_pData->m_state)
		{
			case tPoint:
				if (!on)
				{
					QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
					bool aborted = false;

					if (polygonScale.size() == 0)
					{
						emit statusBarMessage(tr("Selection has been aborted."), 2000);
						aborted = true;
					}
					else
					{
						emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size()-1), 2000);
					}

					if (!aborted && polygonScale.size() > 1)
					{
						for (int i = 0; i < polygonScale.size() - 1; i++)
						{
							QPainterPath path;
							DrawItem *newItem = NULL;
							newItem = new DrawItem(this, tPoint);
							path.moveTo(polygonScale[i].x(), polygonScale[i].y());
							path.lineTo(polygonScale[i].x(), polygonScale[i].y());

							newItem->setShape(path, m_inverseColor0, m_inverseColor1);

							if (this->m_inverseColor0.isValid())
							{
								newItem->setPen(QPen(m_inverseColor0));
								//newItem->setBrush(QBrush(m_inverseColor0));
							}
							else newItem->setPen(QPen(Qt::green));

							newItem->setVisible(true);
							newItem->show();
							newItem->attach(this);
							newItem->setSelected(true);
							
		//                m_pData->m_pDrawItems.append(newItem);
							m_pData->m_pDrawItems.insert(newItem->m_idx, newItem);
						}
						replot();
					}

					Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
					if (p)
					{

						emit p->userInteractionDone(ito::PrimitiveContainer::tPoint, aborted, polygonScale);
						emit p->plotItemsFinished(ito::PrimitiveContainer::tPoint, aborted);
					}

					setState(stateIdle);
					m_pMultiPointPicker->setEnabled(false);
				}
			break;

			case tLine:
				if (!on)
				{
					QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
					bool aborted = false;

					if (polygonScale.size() == 0)
					{
						emit statusBarMessage(tr("Selection has been aborted."), 2000);
						aborted = true;
						m_drawedIemsIndexes.clear();
					}
					else
					{
						emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size()-1), 2000);

						QPainterPath path;
						DrawItem *newItem = NULL;
						newItem = new DrawItem(this, tLine);
						path.moveTo(polygonScale[0].x(), polygonScale[0].y());
						path.lineTo(polygonScale[1].x(), polygonScale[1].y());

						newItem->setShape(path, m_inverseColor0, m_inverseColor1);

						newItem->setVisible(true);
						newItem->show();
						newItem->attach(this);
						newItem->setSelected(true);
						replot();
						m_pData->m_pDrawItems.insert(newItem->m_idx, newItem);                
		//                m_pData->m_pDrawItems.append(newItem);

						m_drawedIemsIndexes.append(newItem->m_idx);
					}

					// if further elements are needed reset the plot engine and go ahead else finish editing
					if (m_pData->m_elementsToPick > 1)
					{
						m_pData->m_elementsToPick--;
						MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());
						if (m)
						{
							m->setMaxNrItems(2);
							m_pMultiPointPicker->setEnabled(true);

							if (!aborted)
							{
								if (m_pData->m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 lines. Esc aborts the selection.").arg(m_pData->m_elementsToPick));
								else emit statusBarMessage(tr("Please draw one line. Esc aborts the selection."));
							}
						}
						return;
					}
					else
					{
						m_pData->m_elementsToPick = 0;
						Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
						if (p)
						{
							QPolygonF destPolygon(0);//(m_drawedIemsIndexes.size() * 4);
							for (int i = 0; i < m_drawedIemsIndexes.size(); i++)
							{
								if (!m_pData->m_pDrawItems.contains(m_drawedIemsIndexes[i])) continue;
								destPolygon.append(QPointF(m_drawedIemsIndexes[i], ito::PrimitiveContainer::tLine));
								destPolygon.append(QPointF(m_pData->m_pDrawItems[m_drawedIemsIndexes[i]]->x1, m_pData->m_pDrawItems[m_drawedIemsIndexes[i]]->y1));
								destPolygon.append(QPointF(m_pData->m_pDrawItems[m_drawedIemsIndexes[i]]->x2, m_pData->m_pDrawItems[m_drawedIemsIndexes[i]]->y2));
								destPolygon.append(QPointF(0.0, 0.0));
							}
							m_drawedIemsIndexes.clear();
							emit p->userInteractionDone(ito::PrimitiveContainer::tLine, aborted, destPolygon);
							emit p->plotItemsFinished(ito::PrimitiveContainer::tLine, aborted);

							
						}
						setState(stateIdle);
						m_pMultiPointPicker->setEnabled(false);
					}
				}
			break;

			case tRect:
				if (!on)
				{
					QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
					bool aborted = false;

					if (polygonScale.size() == 0)
					{
						emit statusBarMessage(tr("Selection has been aborted."), 2000);
						aborted = true;
						m_drawedIemsIndexes.clear();
					}
					else
					{
						emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size()-1), 2000);

						QPainterPath path;
						DrawItem *newItem = NULL;
						newItem = new DrawItem(this, tRect);
						path.addRect(polygonScale[0].x(), polygonScale[0].y(), polygonScale[1].x() - polygonScale[0].x(),
									  polygonScale[1].y() - polygonScale[0].y());

						newItem->setShape(path, m_inverseColor0, m_inverseColor1);
					
						newItem->setVisible(true);
						newItem->show();
						newItem->attach(this);
						newItem->setSelected(true);
						replot();
						m_pData->m_pDrawItems.insert(newItem->m_idx, newItem);
		//                m_pData->m_pDrawItems.append(newItem);

						m_drawedIemsIndexes.append(newItem->m_idx);
					}

					// if further elements are needed reset the plot engine and go ahead else finish editing
					if (m_pData->m_elementsToPick > 1)
					{
						m_pData->m_elementsToPick--;
						MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());
						if (m)
						{
							//m->setMaxNrItems(2);
							m_pMultiPointPicker->setEnabled(true);

							if (!aborted)
							{
								if (m_pData->m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 rectangles. Esc aborts the selection.").arg(m_pData->m_elementsToPick));
								else emit statusBarMessage(tr("Please draw one rectangle. Esc aborts the selection."));
							}
						}
						return;
					}
					else
					{
						m_pData->m_elementsToPick = 0;
						Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
						if (p)
						{
							QPolygonF destPolygon(0);//(m_drawedIemsIndexes.size() * 4);
							for (int i = 0; i < m_drawedIemsIndexes.size(); i++)
							{
								if (!m_pData->m_pDrawItems.contains(m_drawedIemsIndexes[i])) continue;
								destPolygon.append(QPointF(m_drawedIemsIndexes[i], ito::PrimitiveContainer::tRectangle));
								destPolygon.append(QPointF(m_pData->m_pDrawItems[m_drawedIemsIndexes[i]]->x1, m_pData->m_pDrawItems[m_drawedIemsIndexes[i]]->y1));
								destPolygon.append(QPointF(m_pData->m_pDrawItems[m_drawedIemsIndexes[i]]->x2, m_pData->m_pDrawItems[m_drawedIemsIndexes[i]]->y2));
								destPolygon.append(QPointF(0.0, 0.0));
							}
							m_drawedIemsIndexes.clear();

							emit p->userInteractionDone(ito::PrimitiveContainer::tRectangle, aborted, destPolygon);
							emit p->plotItemsFinished(ito::PrimitiveContainer::tRectangle, aborted);
						}
						setState(stateIdle);
						m_pMultiPointPicker->setEnabled(false);
					}
				}
			break;

			case tEllipse:
				if (!on)
				{
					QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
					bool aborted = false;

					if (polygonScale.size() == 0)
					{
						emit statusBarMessage(tr("Selection has been aborted."), 2000);
						aborted = true;
						m_drawedIemsIndexes.clear();
					}
					else
					{
						emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size()-1), 2000);

						QPainterPath path;
						DrawItem *newItem = NULL;
						newItem = new DrawItem(this, tEllipse);
						path.addEllipse(polygonScale[0].x(), polygonScale[0].y(),
								(polygonScale[1].x() - polygonScale[0].x()), (polygonScale[1].y() - polygonScale[0].y()));

						newItem->setShape(path, m_inverseColor0, m_inverseColor1);
					
						if (this->m_inverseColor0.isValid())
						{
							newItem->setPen(QPen(m_inverseColor0));
							//newItem->setBrush(QBrush(m_inverseColor0));
						}
						else newItem->setPen(QPen(Qt::green));

						newItem->setVisible(true);
						newItem->show();
						newItem->attach(this);
						newItem->setSelected(true);
						replot();
						m_pData->m_pDrawItems.insert(newItem->m_idx, newItem);
		//                m_pData->m_pDrawItems.append(newItem);

						m_drawedIemsIndexes.append(newItem->m_idx);
					}

					// if further elements are needed reset the plot engine and go ahead else finish editing
					if (m_pData->m_elementsToPick > 1)
					{
						m_pData->m_elementsToPick--;
						MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());
						if (m)
						{
							//m->setMaxNrItems(2);
							m_pMultiPointPicker->setEnabled(true);

							if (!aborted)
							{
								if (m_pData->m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 ellipses. Esc aborts the selection.").arg(m_pData->m_elementsToPick));
								else emit statusBarMessage(tr("Please draw one ellipse. Esc aborts the selection."));
							}
						}
						return;
					}
					else
					{
						m_pData->m_elementsToPick = 0;
						Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
						if (p)
						{
							QPolygonF destPolygon(0);//(m_drawedIemsIndexes.size() * 4);
							for (int i = 0; i < m_drawedIemsIndexes.size(); i++)
							{
								if (!m_pData->m_pDrawItems.contains(m_drawedIemsIndexes[i])) continue;
								destPolygon.append(QPointF(m_drawedIemsIndexes[i], ito::PrimitiveContainer::tEllipse));
								destPolygon.append(QPointF(m_pData->m_pDrawItems[m_drawedIemsIndexes[i]]->x1, m_pData->m_pDrawItems[m_drawedIemsIndexes[i]]->y1));
								destPolygon.append(QPointF(m_pData->m_pDrawItems[m_drawedIemsIndexes[i]]->x2, m_pData->m_pDrawItems[m_drawedIemsIndexes[i]]->y2));
								destPolygon.append(QPointF(0.0, 0.0));
							}
							m_drawedIemsIndexes.clear();

							emit p->userInteractionDone(ito::PrimitiveContainer::tEllipse, aborted, destPolygon);
							emit p->plotItemsFinished(ito::PrimitiveContainer::tEllipse, aborted);
						}
						setState(stateIdle);
						m_pMultiPointPicker->setEnabled(false);
					}
				}
			break;
		}
	} // if m_pData
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::plotMarkers(const ito::DataObject *coords, QString style, QString id, int plane)
{
    ito::RetVal retval;
    int limits[] = {2,8,0,99999};

    if(!ito::ITOM_API_FUNCS_GRAPH)
        retval += ito::RetVal(ito::retError,0,"Could not plot markers because api is missing");

    ito::DataObject *dObj = apiCreateFromDataObject(coords, 2, ito::tFloat32, limits, &retval);

    QwtSymbol::Style symStyle = QwtSymbol::XCross;
    QSize symSize(5,5);
    QBrush symBrush(Qt::NoBrush);
    QPen symPen(Qt::red);

    QRegExp rgexp("^([b|g|r|c|m|y|k|w]?)([.|o|s|d|\\^|v|<|>|x|+|*|h]?)(\\d*)$");
    if (rgexp.indexIn(style) != -1)
    {
//        QString s = rgexp.cap(1);
        char s = rgexp.cap(1).toLatin1()[0];

        if (s == 'b') symPen.setColor(Qt::blue);
        else if (s == 'g') symPen.setColor(Qt::green);
        else if (s == 'r') symPen.setColor(Qt::red);
        else if (s == 'c') symPen.setColor(Qt::cyan);
        else if (s == 'm') symPen.setColor(Qt::magenta);
        else if (s == 'y') symPen.setColor(Qt::yellow);
        else if (s == 'k') symPen.setColor(Qt::black);
        else if (s == 'w') symPen.setColor(Qt::white);

        s = rgexp.cap(2).toLatin1()[0];
        bool ok;

        if (s == '.') symStyle = QwtSymbol::Ellipse;
        else if (s == 'o') symStyle = QwtSymbol::Ellipse;
        else if (s == 's') symStyle = QwtSymbol::Rect;
        else if (s == 'd') symStyle = QwtSymbol::Diamond;
        else if (s == '>') symStyle = QwtSymbol::RTriangle;
        else if (s == 'v') symStyle = QwtSymbol::DTriangle;
        else if (s == '^') symStyle = QwtSymbol::UTriangle;
        else if (s == '<') symStyle = QwtSymbol::LTriangle;
        else if (s == 'x') symStyle = QwtSymbol::XCross;
        else if (s == '*') symStyle = QwtSymbol::Star1;
        else if (s == '+') symStyle = QwtSymbol::Cross;
        else if (s == 'h') symStyle = QwtSymbol::Hexagon;

        //s = rgexp.cap(3);
        int size = rgexp.cap(3).toInt(&ok);
        if (ok)
        {
            symSize = QSize(size,size);
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError,0,"The style tag does not correspond to the required format");
    }

    if (!retval.containsError())
    {
        if (dObj->getSize(0) >= 8)
        {
//            ItomPlotMarker *marker = NULL;
            int nrOfMarkers = dObj->getSize(1);
            
            if (id == "") id = "unknown";

            ito::float32 *ids = (ito::float32*)dObj->rowPtr(0, 0);            
            ito::float32 *types = (ito::float32*)dObj->rowPtr(0, 1);
            ito::float32 *xCoords1 = (ito::float32*)dObj->rowPtr(0, 2);
            ito::float32 *yCoords1 = (ito::float32*)dObj->rowPtr(0, 3);
            ito::float32 *xCoords2 = (ito::float32*)dObj->rowPtr(0, 4);
            ito::float32 *yCoords2 = (ito::float32*)dObj->rowPtr(0, 5);
            
            for (int i = 0; i < nrOfMarkers; ++i)
            {
                QPainterPath path;
                DrawItem *newItem = NULL;                
                
                switch ((int)types[i])
                {
                    case tPoint:
                        path.moveTo(xCoords1[i], yCoords1[i]);
                        path.lineTo(xCoords1[i], yCoords1[i]);
                    break;
                        
                    case tLine:
                        path.moveTo(xCoords1[i], yCoords1[i]);
                        path.lineTo(xCoords2[i], yCoords2[i]);                        
                    break;
                        
                    case tRect:
                        path.addRect(xCoords1[i], yCoords1[i], xCoords2[i] -  xCoords1[i], yCoords2[i] - yCoords1[i]);
                    break;
                        
                    case tEllipse:
                        path.addEllipse(xCoords1[i], yCoords1[i], xCoords2[i] -  xCoords1[i], yCoords2[i] - yCoords1[i]);
                    break;
                    
                    default:
                        retval += ito::RetVal(ito::retError, 0, tr("invalid marker type").toLatin1().data());
                    break;
                }                    
                if (m_pData->m_pDrawItems.contains((int)ids[i]))
                {
                    m_pData->m_pDrawItems[(int)ids[i]]->setShape(path, m_inverseColor0, m_inverseColor1);
                }
                else
                {
                    switch ((int)types[i])
                    {
                        case tPoint:                    
                            newItem = new DrawItem(this, tPoint, (int)ids[i]);
                        break;
                        
                        case tLine:
                            newItem = new DrawItem(this, tLine, (int)ids[i]);
                        break;
                        
                        case tRect:
                            newItem = new DrawItem(this, tRect, (int)ids[i]);
                        break;
                        
                        case tEllipse:
                            newItem = new DrawItem(this, tEllipse, (int)ids[i]);
                        break;
                        
                        default:
                            retval += ito::RetVal(ito::retError, 0, tr("invalid marker type").toLatin1().data());
                        break;                        
                    }
                    if (newItem)
                    {
                        newItem->setShape(path, m_inverseColor0, m_inverseColor1);
                        
                        newItem->setVisible(true);
                        newItem->show();
                        newItem->attach(this);
                        newItem->setSelected(true);
                        replot();
                        //                m_pData->m_pDrawItems.append(newItem);
                        m_pData->m_pDrawItems.insert(newItem->m_idx, newItem);
                    }
                }                
            }
        }

        replot();
    }

    if (dObj)
    {
        delete dObj;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::deleteMarkers(const int id)
{
    ito::RetVal retval;
    bool found = false;
    
    if (m_pData->m_pDrawItems.contains(id))
    {
        //
        DrawItem *delItem = m_pData->m_pDrawItems[id];
        //delItem->setActive(0);
        //delItem->m_marker.detach();

        delItem->detach();
        m_pData->m_pDrawItems.remove(id);
        delete delItem; // ToDo check for memory leak
        found = true;
    }
    
    if (m_pData->m_pDrawItems.size() == 0)
    {
       m_pData->m_pDrawItems.clear(); 
    }

    if (!found)
    {
        retval += ito::RetVal::format(ito::retError,0,"No marker with id '%d' found.", id);
    }
    else
    {
        replot();
    }
    
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::home()
{
    QStack<QRectF> currentZoomStack = m_pZoomer->zoomStack();

    //get total bounding box
    QRectF boundingRect;
    foreach(QwtPlotCurve *curve, m_plotCurveItems)
    {
        boundingRect = boundingRect.united(((DataObjectSeriesData *)curve->data())->boundingRect());
    }

    if (currentZoomStack.empty())
    {
        currentZoomStack.push(boundingRect);
    }
    else
    {
        currentZoomStack.first() = boundingRect;
    }

    m_pZoomer->setZoomStack(currentZoomStack, 0);
    m_pZoomer->zoom(0);
}
//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer< ito::DataObject > Plot1DWidget::getPlotPicker() const
{
    int ysize = m_pickers.size();
    int xsize = 3;

    if(ysize == 0)
    {
        return QSharedPointer< ito::DataObject >(new ito::DataObject());
    }

    ito::DataObject tmp(ysize, xsize, ito::tFloat32);

    for(int idx = 0; idx < ysize; idx++)
    {
        ito::float64 xScaleStart = (m_pickers[idx]).item->xValue();
        ito::float64 yValue = (m_pickers[idx]).item->yValue();
        tmp.at<ito::float32>(idx, 1) = cv::saturate_cast<ito::float32>(xScaleStart);
        tmp.at<ito::float32>(idx, 2) = cv::saturate_cast<ito::float32>(yValue);

        if((m_pickers[idx]).curveIdx < 0 || (m_pickers[idx]).curveIdx > m_plotCurveItems.size() - 1)
            continue;

        DataObjectSeriesData *data = (DataObjectSeriesData*)(m_plotCurveItems[(m_pickers[idx]).curveIdx]->data());

        int thisIdx = data->getPosToPix(xScaleStart);
        tmp.at<ito::float32>(idx, 0) = cv::saturate_cast<ito::float32>(thisIdx);

    }

    QSharedPointer< ito::DataObject > exportItem(new ito::DataObject(tmp));

    return exportItem;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::setPicker(const QVector<int> &pxCords)
{
    int cnt = pxCords.size();
    if(cnt < m_pData->m_pickerLimit)
        cnt = m_pData->m_pickerLimit;

    for(int i = 0; i < cnt; i++)
    {
        if( i > m_pickers.size() - 1)
        {
                Picker picker;
                picker.item = new ItomPlotMarker(m_pData->m_pickerLabelVisible, 
                                                 m_pData->m_pickerType, 
                                                 m_pData->m_pickerLabelAlignment, 
                                                 m_pData->m_pickerLabelOrientation );
                picker.item->attach(this);
                picker.active = true;
                //marker.color = Qt::darkGreen;
                //marker.item->setSymbol(new QwtSymbol(QwtSymbol::Diamond,QBrush(Qt::white), QPen(marker.color,1),  QSize(8,8)));
                
                picker.curveIdx = 0;
                stickPickerToSampleIdx( &picker, pxCords[i] < 0 ? 0 : pxCords[i], 0, 0);
                picker.item->setVisible(true);
                
                m_pickers.append(picker);
        }
        else
        {
            stickPickerToSampleIdx( &(m_pickers[i]), pxCords[i] < 0 ? 0 : pxCords[i], 0, 0);
        }
        
    }
    updatePickerPosition(false, false);
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::setPicker(const QVector<float> &physCords)
{
    int cnt = physCords.size();
    if(cnt < m_pData->m_pickerLimit)
        cnt = m_pData->m_pickerLimit;

    for(int i = 0; i < cnt; i++)
    {
        if( i > m_pickers.size() - 1)
        {
                Picker picker;
                picker.item = new ItomPlotMarker(m_pData->m_pickerLabelVisible, 
                                                 m_pData->m_pickerType, 
                                                 m_pData->m_pickerLabelAlignment, 
                                                 m_pData->m_pickerLabelOrientation );
                picker.item->attach(this);
                picker.active = true;
                //picker.color = Qt::darkGreen;
                //picker.item->setSymbol(new QwtSymbol(QwtSymbol::Diamond,QBrush(Qt::white), QPen(marker.color,1),  QSize(8,8)));
                
                picker.curveIdx = 0;
                stickPickerToXPx(&picker, physCords[i], 0);

                picker.item->setVisible(true);
                
                m_pickers.append(picker);
        }
        else
        {
            stickPickerToXPx( &(m_pickers[i]), physCords[i], 0);
        }
    }
    updatePickerPosition(false, false);
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
QVector<int> Plot1DWidget::getPickerPixel() const
{
    int ysize = m_pickers.size();

    if(ysize == 0)
    {
        return QVector<int>();
    }

    QVector<int> exportItem(ysize, 0);

    for(int idx = 0; idx < ysize; idx++)
    {

        if((m_pickers[idx]).curveIdx < 0 || (m_pickers[idx]).curveIdx > m_plotCurveItems.size() - 1)
            continue;

        DataObjectSeriesData *data = (DataObjectSeriesData*)(m_plotCurveItems[(m_pickers[idx]).curveIdx]->data());

        int thisIdx = data->getPosToPix((m_pickers[idx]).item->xValue());
        exportItem[idx] = thisIdx;
    }

    return exportItem;
}
//----------------------------------------------------------------------------------------------------------------------------------
QVector<float> Plot1DWidget::getPickerPhys() const
{
    int ysize = m_pickers.size();

    if(ysize == 0)
    {
        return QVector<ito::float32>();
    }

    QVector<ito::float32> exportItem(ysize, 0);

    for(int idx = 0; idx < ysize; idx++)
    {
        exportItem[idx] = (m_pickers[idx]).item->xValue();
    }

    return exportItem;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::updateColors(void)
{
    if(m_pData)
    {
        QString styleSheet = this->styleSheet();

        styleSheet = QString("background-color: rgb(%1, %2, %3);").arg(QString::number(m_pData->m_backgnd.red()), QString::number(m_pData->m_backgnd.green()), QString::number(m_pData->m_backgnd.blue())) ;
        styleSheet.append(QString("color: rgb(%1, %2, %3);").arg(QString::number(m_pData->m_axisColor.red()), QString::number(m_pData->m_axisColor.green()), QString::number(m_pData->m_axisColor.blue())));
        this->setStyleSheet(styleSheet);

        QPalette newPalette(m_pData->m_backgnd);

        newPalette.setColor( QPalette::WindowText, m_pData->m_axisColor); // for ticks
        newPalette.setColor( QPalette::Text, m_pData->m_textColor); // for ticks' labels

        setAutoFillBackground( true );
        setPalette( newPalette );
        setCanvasBackground(m_pData->m_backgnd);
        
        
        axisWidget(QwtPlot::xBottom)->setAutoFillBackground( true );
        axisWidget(QwtPlot::xBottom)->setPalette(newPalette);

        axisWidget(QwtPlot::yLeft)->setAutoFillBackground( true );
        axisWidget(QwtPlot::yLeft)->setPalette(newPalette);

        axisWidget(QwtPlot::yRight)->setAutoFillBackground( true );
        axisWidget(QwtPlot::yRight)->setPalette(newPalette);

        axisWidget(QwtPlot::xTop)->setAutoFillBackground( true );
        axisWidget(QwtPlot::xTop)->setPalette(newPalette);
        
        replot();        
    }

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::legendItemChecked(const QVariant &itemInfo, bool on)
{
    QwtPlotItem *pi = infoToItem(itemInfo);
    if (pi)
    {
        pi->setVisible(on);
        replot();
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::updatePickerStyle(void)
{
    if(!m_pData) return;

    foreach (Picker m, m_pickers)
    {
        m.item->setLabelAlignment(m_pData->m_pickerLabelAlignment);
        m.item->setLabelOrientation(m_pData->m_pickerLabelOrientation);
        m.item->setLabelEnabled(m_pData->m_pickerLabelVisible);
        m.item->setPlotType(m_pData->m_pickerType);
    } 
    replot();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setQwtLineStyle(const Itom1DQwt::tCurveStyle &style)
{
    m_qwtCurveStyle = style;

    foreach(QwtPlotCurve *c, m_plotCurveItems)
    {
        switch(m_qwtCurveStyle)
        {
            case Itom1DQwt::NoCurve:
                c->setStyle(QwtPlotCurve::NoCurve);
            break;
            default:
            case Itom1DQwt::Lines:
                c->setCurveAttribute(QwtPlotCurve::Fitted, false);
                c->setStyle(QwtPlotCurve::Lines);
                break;
            case Itom1DQwt::FittedLines:
                c->setCurveAttribute(QwtPlotCurve::Fitted, true);
                c->setStyle(QwtPlotCurve::Lines);
            break;
            case Itom1DQwt::StepsLeft:
                c->setOrientation(Qt::Horizontal);
                c->setCurveAttribute(QwtPlotCurve::Inverted, false);
                c->setStyle(QwtPlotCurve::Steps);
            break;
            case Itom1DQwt::StepsRight:
                c->setOrientation(Qt::Horizontal);
                c->setCurveAttribute(QwtPlotCurve::Inverted, true);
                c->setStyle(QwtPlotCurve::Steps);
            break;
            case Itom1DQwt::Steps:
                c->setOrientation(Qt::Horizontal);
                c->setCurveAttribute(QwtPlotCurve::Inverted, false);
                c->setStyle(QwtPlotCurve::UserCurve );
            break;
            case Itom1DQwt::SticksHorizontal:
                c->setOrientation(Qt::Horizontal);
                c->setStyle(QwtPlotCurve::Sticks);
            break;
            case Itom1DQwt::Sticks:
            case Itom1DQwt::SticksVertical:
                c->setOrientation(Qt::Vertical);
                c->setStyle(QwtPlotCurve::Sticks);
            break;
            case Itom1DQwt::Dots:
                c->setStyle(QwtPlotCurve::Dots);
            break;
        }
    }

    replot();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setBaseLine(const qreal &line)
{
    m_baseLine = line;

    foreach(QwtPlotCurve *c, m_plotCurveItems)
    {
        c->setBaseline(m_baseLine);
    }

    replot();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setCurveFilled()
{
    int colorIndex = 0;
    foreach(QwtPlotCurve *c, m_plotCurveItems)
    {
        c->setBaseline(m_baseLine);
        ((QwtPlotCurveDataObject*)c)->setCurveFilled(m_curveFilled);
        if(m_curveFilled != Itom1DQwt::NoCurveFill)
        {
            if(m_filledColor.isValid())
            {
                ((QwtPlotCurveDataObject*)c)->setBrush(QBrush(m_filledColor));
            }
            else
            {
                QColor fill = m_colorList[colorIndex++];
                colorIndex = colorIndex % m_colorList.size();
                fill.setAlpha(m_fillCurveAlpa);
                ((QwtPlotCurveDataObject*)c)->setBrush(QBrush(fill));
            }
        }
        else
        {
            ((QwtPlotCurveDataObject*)c)->setBrush(Qt::NoBrush);
        }
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> Plot1DWidget::getDisplayed()
{
    ito::DataObject *displayed = NULL;

    if (m_plotCurveItems.size() > 0)
    {
        const DataObjectSeriesData* seriesData = static_cast<const DataObjectSeriesData*>(m_plotCurveItems[0]->data());
        ito::tDataType type = (seriesData->getDataObject() ? (ito::tDataType)seriesData->getDataObject()->getType() : ito::tUInt8);

        foreach (const QwtPlotCurve *curve, m_plotCurveItems)
        {
            const DataObjectSeriesData* temp = static_cast<const DataObjectSeriesData*>(curve->data());
            if (!temp->getDataObject() || temp->getDataObject()->getType() != type)
            {
                return QSharedPointer<ito::DataObject>();
            }
        }

        //complex64 will be mapped to float32 and complex128 to float64
        type = (type == ito::tComplex64) ? ito::tFloat32 : ((type == ito::tComplex128) ? ito::tFloat64 : type);

        //until now, rgba32 will be mapped to uint8
        type = (type == ito::tRGBA32) ? ito::tUInt8 : type;

        //create data object according to first series data and set its axis scales and axis offsets values accordingly
        QwtInterval ival = axisInterval(QwtPlot::xBottom);
        size_t firstIdx = std::numeric_limits<size_t>::max();
        size_t lastIdx = 0;

        //get start and end index of sample within interval
        for (size_t i = 0; i < seriesData->size(); ++i)
        {
            if (ival.contains(seriesData->sample(i).x()))
            {
                firstIdx = std::min(firstIdx, i);
                lastIdx = std::max(lastIdx, i);
            }
        }

        if (firstIdx == lastIdx)
        {
            return QSharedPointer<ito::DataObject>();
        }

        double lengthPhys = seriesData->sample(lastIdx).x() - seriesData->sample(firstIdx).x();
        size_t lengthPx = lastIdx - firstIdx;

        displayed = new ito::DataObject((int)m_plotCurveItems.size(), lastIdx - firstIdx + 1, type);

        std::string descr, unit;
        seriesData->getDObjValueDescriptionAndUnit(descr, unit);
        displayed->setValueUnit(unit);
        displayed->setValueDescription(descr);
        seriesData->getDObjAxisDescriptionAndUnit(descr, unit);
        displayed->setAxisUnit(1, unit);
        displayed->setAxisDescription(1, descr);

        if (lengthPx > 0)
        {
            displayed->setAxisScale(1, lengthPhys / lengthPx);
            displayed->setAxisOffset(1, -(seriesData->sample(firstIdx).x() / lengthPhys) * lengthPx );
        }
        else
        {
            displayed->setAxisScale(1, 1.0);
            displayed->setAxisOffset(1, -seriesData->sample(firstIdx).x());
        }
        

        //put data in
        for (int i = 0; i < m_plotCurveItems.size(); ++i)
        {
            seriesData = static_cast<const DataObjectSeriesData*>(m_plotCurveItems[i]->data());

            switch (type)
            {
            case ito::tUInt8:
                {
                ito::uint8 *rowPtr = (ito::uint8*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::uint8>(seriesData->sample(n).ry());
                }
                }
                break;
            case ito::tInt8:
                {
                ito::int8 *rowPtr = (ito::int8*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::int8>(seriesData->sample(n).ry());
                }
                }
                break;
            case ito::tUInt16:
                {
                ito::uint16 *rowPtr = (ito::uint16*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::uint16>(seriesData->sample(n).ry());
                }
                }
                break;
            case ito::tInt16:
                {
                ito::int16 *rowPtr = (ito::int16*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::int16>(seriesData->sample(n).ry());
                }
                }
                break;
            case ito::tUInt32:
                {
                ito::uint32 *rowPtr = (ito::uint32*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::uint32>(seriesData->sample(n).ry());
                }
                }
                break;
            case ito::tInt32:
                {
                ito::int32 *rowPtr = (ito::int32*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::int32>(seriesData->sample(n).ry());
                }
                }
                break;
            case ito::tFloat32:
                {
                ito::float32 *rowPtr = (ito::float32*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::float32>(seriesData->sample(n).ry());
                }
                }
                break;
            case ito::tFloat64:
                {
                ito::float64 *rowPtr = (ito::float64*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::float64>(seriesData->sample(n).ry());
                }
                }
                break;
            }
        }
    }

    return QSharedPointer<ito::DataObject>(displayed);
}
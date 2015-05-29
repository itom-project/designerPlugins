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

#include "plotCanvas.h"
#include "common/sharedStructuresGraphics.h"
#include "DataObject/dataObjectFuncs.h"
#include "common/apiFunctionsGraphInc.h"
#include "common/apiFunctionsInc.h"

#include "dataObjRasterData.h"
#include "itom2dqwtplot.h"
#include "valuePicker2d.h"
#include "../sharedFiles/multiPointPickerMachine.h"

#include <qwt_color_map.h>
#include <qwt_plot_layout.h>
#include <qwt_matrix_raster_data.h>
#include <qwt_scale_widget.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_renderer.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_canvas.h>
#include <qwt_symbol.h>
#include <qwt_picker.h>
#include <qwt_picker_machine.h>
#include <qwt_scale_engine.h>
#include <qwt_picker_machine.h>
#include <qwt_plot_shapeitem.h>

#include <qimage.h>
#include <qpixmap.h>
#include <qdebug.h>
#include <qmessagebox.h>


//----------------------------------------------------------------------------------------------------------------------------------
PlotCanvas::PlotCanvas(QMenu *contextMenu, InternalData *m_pData, QWidget * parent /*= NULL*/) :
        QwtPlot(parent),
        m_pZoomer(NULL),
        m_pPanner(NULL),
        m_pLineCutPicker(NULL),
        m_pCenterMarker(NULL),
//        m_pStackCut(NULL),
        m_dObjItem(NULL),
        m_rasterData(NULL),
        m_dOverlayItem(NULL),
        m_pData(m_pData),
        m_curOverlayColorMapIndex(0),
        m_curColorMapIndex(0),
        m_pValuePicker(NULL),
        m_dObjPtr(NULL),
        m_pStackPicker(NULL),
        m_zstackCutUID(0),
        m_lineCutUID(0),
        m_pLineCutLine(NULL),
        m_pMultiPointPicker(NULL),
        m_activeDrawItem(-1),
        m_ignoreNextMouseEvent(false),
        m_contextMenu(contextMenu),
        m_isRefreshingPlot(false),
        m_firstTimeVisible(false)
        
{
    setMouseTracking(false);

    //canvas() is the real plotting area, where the plot is printed (without axes...)
    //canvas()->setFrameShadow(QFrame::Plain);
    //canvas()->setFrameShape(QFrame::NoFrame);

    canvas()->setStyleSheet("border: 0px;");
    canvas()->setCursor(Qt::ArrowCursor);

    //main item on canvas -> the data object
    m_dObjItem = new DataObjItem("Data Object");
    m_dObjItem->setRenderThreadCount(0);
    //m_dObjItem->setColorMap(new QwtLinearColorMap(QColor::fromRgb(0,0,0), QColor::fromRgb(255,255,255), QwtColorMap::Indexed));
    m_rasterData = new DataObjRasterData(m_pData);
    m_dObjItem->setData(m_rasterData);
    m_dObjItem->attach(this);

    //overlayobject ttem on canvas -> the data object
    m_dOverlayItem = new DataObjItem("Overlay Object");
    m_dOverlayItem->setRenderThreadCount(0);
    m_rasterOverlayData = new DataObjRasterData(m_pData, true);
    m_dOverlayItem->setData(m_rasterOverlayData);
    m_dOverlayItem->attach(this);
    m_dOverlayItem->setAlpha(m_pData->m_alpha);
    m_dOverlayItem->setColorMap(new QwtLinearColorMap(Qt::black, Qt::white, QwtColorMap::Indexed));
    m_colorOverlayMapName = "gray";
    m_dOverlayItem->setVisible(false);
    //zoom tool
    m_pZoomer = new ItomPlotZoomer(QwtPlot::xBottom, QwtPlot::yLeft, canvas());
    m_pZoomer->setEnabled(false);
    m_pZoomer->setTrackerMode(QwtPicker::AlwaysOn);
    m_pZoomer->setMousePattern(QwtEventPattern::MouseSelect2, Qt::NoButton); //right click should open the context menu, not a zoom out to level 0 (Ctrl+0) if zoomer is enabled.

    //pan tool
    m_pPanner = new QwtPlotPanner(canvas());
    m_pPanner->setAxisEnabled(QwtPlot::yRight,false); //do not consider the right vertical axis
    m_pPanner->setCursor(Qt::SizeAllCursor);
    m_pPanner->setEnabled(false);
    connect(m_pPanner, SIGNAL(panned(int,int)), m_pZoomer, SLOT(canvasPanned(int,int))); //if panner is moved, the new rect is added to the zoom stack for a synchronization of both tools

    m_pMagnifier = new ItomPlotMagnifier(canvas(), m_pZoomer);
    m_pMagnifier->setEnabled(true);
    m_pMagnifier->setWheelModifiers(Qt::ControlModifier);
    m_pMagnifier->setAxisEnabled(QwtPlot::yLeft,true);
    m_pMagnifier->setAxisEnabled(QwtPlot::xBottom,true);
    m_pMagnifier->setAxisEnabled(QwtPlot::yRight,false); //do not consider the right vertical axis (color bar)

    //value picker
    m_pValuePicker = new ValuePicker2D(QwtPlot::xBottom, QwtPlot::yLeft, canvas(), m_rasterData, m_rasterOverlayData);
    m_pValuePicker->setEnabled(false);
    m_pValuePicker->setTrackerMode(QwtPicker::AlwaysOn);

    //zStack cut picker
    m_pStackPicker = new ItomPlotPicker(canvas());
    m_pStackPicker->setStateMachine(new QwtPickerClickPointMachine());
    m_pStackPicker->setTrackerMode(QwtPicker::AlwaysOn);
    m_pStackPicker->setRubberBand(QwtPicker::CrossRubberBand);
    m_pStackPicker->setEnabled(false);
    //disable key movements for the picker (the marker will be moved by the key-event of this widget)
    m_pStackPicker->setKeyPattern(QwtEventPattern::KeyLeft, 0);
    m_pStackPicker->setKeyPattern(QwtEventPattern::KeyRight, 0);
    m_pStackPicker->setKeyPattern(QwtEventPattern::KeyUp, 0);
    m_pStackPicker->setKeyPattern(QwtEventPattern::KeyDown, 0);
    connect(m_pStackPicker, SIGNAL(appended(const QPoint&)), this, SLOT(zStackCutTrackerMoved(const QPoint&)));
    connect(m_pStackPicker, SIGNAL(moved(const QPoint&)), this, SLOT(zStackCutTrackerMoved(const QPoint&)));

    //marker for zstack cut
    m_pStackCutMarker = new QwtPlotMarker();
    m_pStackCutMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green),3),  QSize(11,11)));
    m_pStackCutMarker->attach(this);
    m_pStackCutMarker->setVisible(false);

    //marker for the camera center

    m_pCenterMarker = new QwtPlotMarker();
    m_pCenterMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green), 1),  QSize(11,11)));
    m_pCenterMarker->attach(this);
    m_pCenterMarker->setVisible(m_pData->m_showCenterMarker);
    m_pCenterMarker->setValue(QPointF(0.0, 0.0));
    m_pCenterMarker->setAxes(QwtPlot::xBottom, QwtPlot::yLeft);
    m_pCenterMarker->setSpacing(0);

    //picker for line picking
    m_pLineCutPicker = new QwtPlotPicker(QwtPlot::xBottom, QwtPlot::yLeft, QwtPicker::CrossRubberBand, QwtPicker::AlwaysOn, canvas());
    m_pLineCutPicker->setEnabled(false);
    m_pLineCutPicker->setStateMachine(new QwtPickerDragPointMachine);
    m_pLineCutPicker->setRubberBandPen(QPen(Qt::green));
    m_pLineCutPicker->setTrackerPen(QPen(Qt::green));
    //disable key movements for the picker (the marker will be moved by the key-event of this widget)
    m_pLineCutPicker->setKeyPattern(QwtEventPattern::KeyLeft, 0);
    m_pLineCutPicker->setKeyPattern(QwtEventPattern::KeyRight, 0);
    m_pLineCutPicker->setKeyPattern(QwtEventPattern::KeyUp, 0);
    m_pLineCutPicker->setKeyPattern(QwtEventPattern::KeyDown, 0);
    connect(m_pLineCutPicker, SIGNAL(moved(const QPoint&)), SLOT(lineCutMoved(const QPoint&)));
    connect(m_pLineCutPicker, SIGNAL(appended(const QPoint&)), SLOT(lineCutAppended(const QPoint&)));

    //line for line picking
    m_pLineCutLine = new QwtPlotCurve();
    m_pLineCutLine->attach(this);
    m_pLineCutLine->setVisible(false);

    //multi point picker for pick-point action (equivalent to matlabs ginput)
    m_pMultiPointPicker = new UserInteractionPlotPicker(QwtPlot::xBottom, QwtPlot::yLeft, QwtPicker::PolygonRubberBand, QwtPicker::AlwaysOn, canvas());
    m_pMultiPointPicker->setEnabled(false);
    m_pMultiPointPicker->setRubberBand(QwtPicker::UserRubberBand); //user is cross here
    //m_pMultiPointPicker->setStateMachine(new QwtPickerClickPointMachine);
    m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine);
    m_pMultiPointPicker->setRubberBandPen(QPen(QBrush(Qt::green, Qt::SolidPattern),2));
    connect(m_pMultiPointPicker, SIGNAL(activated(bool)), this, SLOT(multiPointActivated(bool)));
    //connect(m_pMultiPointPicker, SIGNAL(selected(QPolygon)), this, SLOT(multiPointSelected (QPolygon)));
    //connect(m_pMultiPointPicker, SIGNAL(appended(QPoint)), this, SLOT(multiPointAppended (QPoint)));

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

    //right axis (color bar)
    QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
    rightAxis->setColorBarEnabled(true);
    rightAxis->setColorBarWidth(15);
    setAxisScale(QwtPlot::yRight, 0, 1.0);
    enableAxis(QwtPlot::yRight, m_pData->m_colorBarVisible);
    axisWidget(QwtPlot::yRight)->setLayoutFlag(QwtScaleWidget::TitleInverted, false); //let the label be in the same direction than on the left side

    //rightAxis->scaleDraw()->setLength(20);
    rightAxis->scaleDraw()->enableComponent(QwtAbstractScaleDraw::Backbone,false); //disable backbone
    rightAxis->setMargin(10);                       //distance colorbar <-> canvas
    rightAxis->setSpacing(8);                       //distance tick labels <-> axis label && color bar -> ticks
    rightAxis->scaleDraw()->setSpacing(6);          //distance tick labels <-> ticks
    rightAxis->setContentsMargins(0,0,0,0);         //top and bottom offset

    configRescaler();

    m_drawedIemsIndexes.clear();
    m_drawedIemsIndexes.reserve(10);
    updateColors();
}

//----------------------------------------------------------------------------------------------------------------------------------
PlotCanvas::~PlotCanvas()
{
    delete m_pData;
    m_pData = NULL;
    m_pLineCutLine->detach();
    delete m_pLineCutLine;
    m_pLineCutLine = NULL;

    m_pStackCutMarker->detach();
    delete m_pStackCutMarker;
    m_pStackCutMarker = NULL;

    m_pCenterMarker->detach();
    delete m_pCenterMarker;
    m_pCenterMarker = NULL;

    m_pMultiPointPicker = NULL;

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotCanvas::init()
{
    if (!setColorMap("__first__"))
    {
        refreshStyles();
    }
    else
    {
        //refreshStyles is implicitely called by setColorMap
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::refreshStyles()
{
    QPen rubberBandPen = QPen(QBrush(Qt::red), 1, Qt::DashLine);
    QPen trackerPen = QPen(QBrush(Qt::red), 2);
    QFont trackerFont = QFont("Verdana", 10);
    QBrush trackerBg = QBrush(QColor(255, 255, 255, 155), Qt::SolidPattern);
    QPen selectionPen = QPen(QBrush(Qt::gray), 2, Qt::SolidLine);

    QFont titleFont = QFont("Helvetica", 12);
    QFont labelFont = QFont("Helvetica", 12);
    labelFont.setItalic(false);
    QFont axisFont = QFont("Helvetica", 10);

    QSize centerMarkerSize = QSize(10, 10);
    QPen centerMarkerPen = QPen(QBrush(Qt::red), 1);

    if(ito::ITOM_API_FUNCS_GRAPH)
    {
        rubberBandPen = apiGetFigureSetting(parent(), "zoomRubberBandPen", QPen(QBrush(Qt::red), 1, Qt::DashLine),NULL).value<QPen>();
        trackerPen = apiGetFigureSetting(parent(), "trackerPen", QPen(QBrush(Qt::red), 2), NULL).value<QPen>();
        trackerFont = apiGetFigureSetting(parent(), "trackerFont", QFont("Verdana", 10), NULL).value<QFont>();
        trackerBg = apiGetFigureSetting(parent(), "trackerBackground", QBrush(QColor(255, 255, 255, 155), Qt::SolidPattern), NULL).value<QBrush>();
        selectionPen = apiGetFigureSetting(parent(), "selectionPen", QPen(QBrush(Qt::gray), 2, Qt::SolidLine), NULL).value<QPen>();

        titleFont = apiGetFigureSetting(parent(), "titleFont", QFont("Helvetica", 12), NULL).value<QFont>();
        labelFont = apiGetFigureSetting(parent(), "labelFont", QFont("Helvetica", 12), NULL).value<QFont>();
        labelFont.setItalic(false);
        axisFont = apiGetFigureSetting(parent(), "axisFont", QFont("Helvetica", 10), NULL).value<QFont>();

        centerMarkerSize = apiGetFigureSetting(parent(), "centerMarkerSize", QSize(10, 10), NULL).value<QSize>();
        centerMarkerPen = apiGetFigureSetting(parent(), "centerMarkerPen", QPen(QBrush(Qt::red), 1), NULL).value<QPen>();
    }



    if (m_inverseColor1.isValid())
    {
        rubberBandPen.setColor(m_inverseColor1);
    }

    if (m_inverseColor0.isValid())
    {
        selectionPen.setColor(m_inverseColor0);
        trackerPen.setColor(m_inverseColor0);
        centerMarkerPen.setColor(m_inverseColor0);
    }

    m_pZoomer->setRubberBandPen(rubberBandPen);
    m_pZoomer->setTrackerFont(trackerFont);
    m_pZoomer->setTrackerPen(trackerPen);

    m_pValuePicker->setTrackerFont(trackerFont);
    m_pValuePicker->setTrackerPen(trackerPen);
    m_pValuePicker->setBackgroundFillBrush(trackerBg);

    m_pMultiPointPicker->setTrackerFont(trackerFont);
    m_pMultiPointPicker->setTrackerPen(trackerPen);
    m_pMultiPointPicker->setBackgroundFillBrush(trackerBg);

    m_pLineCutPicker->setRubberBandPen(rubberBandPen);
    m_pLineCutPicker->setTrackerPen(trackerPen);
    m_pLineCutLine->setPen(selectionPen);

    m_pStackPicker->setTrackerFont(trackerFont);
    m_pStackPicker->setTrackerPen(trackerPen);
    m_pStackPicker->setBackgroundFillBrush(trackerBg);

    m_pStackCutMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(m_inverseColor1), QPen(QBrush(m_inverseColor1),3),  QSize(7,7)));

    m_pCenterMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(/*m_inverseColor0*/), centerMarkerPen,  centerMarkerSize));
    
    QHash<int, DrawItem*>::iterator it = m_pData->m_pDrawItems.begin();
    for (;it != m_pData->m_pDrawItems.end(); ++it)        
    {
        if (it.value() != NULL && it.value()->m_autoColor)
        {
            it.value()->setColor(m_inverseColor1, m_inverseColor0);
        }
    }
    //}
    //else
    //{
    //    m_pLineCutPicker->setRubberBandPen(QPen(Qt::gray));
    //    m_pLineCutPicker->setTrackerPen(QPen(Qt::gray));
    //    m_pLineCutLine->setPen(Qt::gray);

    title().setFont(titleFont);

    axisTitle(QwtPlot::xBottom).setFont(axisFont);
    axisTitle(QwtPlot::yLeft).setFont(axisFont);
    axisTitle(QwtPlot::yRight).setFont(axisFont);
    setAxisFont(QwtPlot::xBottom, axisFont);
    setAxisFont(QwtPlot::yLeft, axisFont);
    setAxisFont(QwtPlot::yRight, axisFont);

    QwtText t = axisWidget(QwtPlot::xBottom)->title();
    t.setFont(labelFont);
    axisWidget(QwtPlot::xBottom)->setTitle(t);

    t = axisWidget(QwtPlot::yLeft)->title();
    t.setFont(labelFont);
    axisWidget(QwtPlot::yLeft)->setTitle(t);

    t = axisWidget(QwtPlot::yRight)->title();
    t.setFont(labelFont);
    axisWidget(QwtPlot::yRight)->setTitle(t);

    //axisWidget(QwtPlot::yRight)->setLabelRotation(-90.0); //this rotates the tick values for the color bar ;)
    //axisScaleDraw(QwtPlot::yRight)->setLabelRotation(90); //this also ;)
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::refreshPlot(const ito::DataObject *dObj, int plane /*= -1*/)
{
    if (m_isRefreshingPlot)
    {
        return;
    }

    m_isRefreshingPlot = true;

    ito::uint8 updateState = 0; //changeNo (0): nothing changed, changeAppearance (1): appearance changed (yAxisFlipped, cmplxFlag, plane...), changeData (2): data changed (dimensions, sizes, other data object...)

    m_dObjPtr = dObj;

    //QString valueLabel, axisLabel, title;

    if (dObj)
    {
        int dims = dObj->getDims();
        int width = dims > 0 ? dObj->getSize(dims - 1) : 0;
        int height = dims > 1 ? dObj->getSize(dims - 2) : 1;

        updateState = m_rasterData->updateDataObject(dObj, plane);

        if (updateState & changeData)
        {
            bool valid;
            ito::DataObjectTagType tag;
            std::string descr, unit;
            tag = dObj->getTag("title", valid);
            m_pData->m_titleDObj = valid ? QString::fromLatin1(tag.getVal_ToString().data()) : "";
            m_pData->m_dataType = (ito::tDataType)dObj->getType();

            descr = dObj->getValueDescription();
            unit = dObj->getValueUnit();

            if (unit != "")
            {
                descr.append(" [" + unit + "]");
            }
            m_pData->m_valueLabelDObj = QString::fromLatin1(descr.data());

            if (dims >= 2)
            {
                descr = dObj->getAxisDescription(dims-1, valid);
                if (!valid) descr = "";
                unit = dObj->getAxisUnit(dims-1,valid);
                if (!valid) unit = "";

                if (unit != "")
                {
                    descr.append(" [" + unit + "]");
                }
                m_pData->m_xaxisLabelDObj = QString::fromLatin1(descr.data());

                descr = dObj->getAxisDescription(dims-2, valid);
                if (!valid) descr = "";
                unit = dObj->getAxisUnit(dims-2,valid);
                if (!valid) unit = "";

                if (unit != "")
                {
                    descr.append(" [" + unit + "]");
                }
                m_pData->m_yaxisLabelDObj = QString::fromLatin1(descr.data());
            }
            else
            {
                m_pData->m_xaxisLabelDObj = "";
                m_pData->m_yaxisLabelDObj = "";
            }

            m_pCenterMarker->setXValue(m_dObjPtr->getPixToPhys(dims-1, (width - 1)/ 2.0, valid));
            m_pCenterMarker->setYValue(m_dObjPtr->getPixToPhys(dims-2, (height - 1) / 2.0, valid));
        }
    }

    updateLabels();

    if (updateState != changeNo)
    {
        Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
        if (p)
        {
            p->setColorDataTypeRepresentation(dObj->getType() == ito::tRGBA32);

            if (dObj->getType() == ito::tRGBA32) //coloured objects have no color bar, therefore the value axis cropping is disabled
            {
                m_pData->m_valueScaleAuto = false;
                m_pData->m_valueMin = std::numeric_limits<ito::uint8>::min();
                m_pData->m_valueMax = std::numeric_limits<ito::uint8>::max();
            }

            if (dObj->getType() == ito::tComplex128 || dObj->getType() == ito::tComplex64)
            {
                p->setCmplxSwitch(m_pData->m_cmplxType, true);
            }
            else
            {
                p->setCmplxSwitch(m_pData->m_cmplxType, false);
            }

            int maxPlane = 0;
            if (dObj->getDims() > 2)
            {
                maxPlane = dObj->calcNumMats() - 1;
            }

            p->setPlaneRange(0, maxPlane);
        }

        updateScaleValues(false, updateState & changeData); //no replot here
        updateAxes();


        //set the base view for the zoomer (click on 'house' symbol) to the current representation (only if data changed)
        if (updateState & changeData)
        {
            m_pZoomer->setZoomBase(false); //do not replot in order to not destroy the recently set scale values, a rescale is executed at the end though
        }
        else
        {
            m_pZoomer->rescale(false);
        }
    }
    else
    {
        replot();
    }

    m_isRefreshingPlot = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::changePlane(int plane)
{
    refreshPlot(m_dObjPtr, plane);
}

//----------------------------------------------------------------------------------------------------------------------------------
int PlotCanvas::getCurrentPlane()
{
    if (m_rasterData)
    {
        return m_rasterData->getCurrentPlane();
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::internalDataUpdated()
{
    refreshPlot(m_dObjPtr, -1);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotCanvas::setColorMap(QString colormap /*= "__next__"*/)
{
    QwtLinearColorMap *colorMap = NULL;
    QwtLinearColorMap *colorBarMap = NULL;
    ito::ItomPalette newPalette;
    ito::RetVal retval(ito::retOk);
    int numPalettes = 1;

    if(!ito::ITOM_API_FUNCS_GRAPH)
    {
        emit statusBarMessage(tr("Could not change color bar, api is missing"), 4000);
        return false;
    }

    retval += apiPaletteGetNumberOfColorBars(numPalettes);

    if (numPalettes == 0 || retval.containsError())
    {
        emit statusBarMessage(tr("No color maps defined."), 4000);
        return false;
    }

    if (colormap == "__next__")
    {
        m_curColorMapIndex++;
        m_curColorMapIndex %= numPalettes; //map index to [0,numPalettes)
        retval += apiPaletteGetColorBarIdx(m_curColorMapIndex, newPalette);
    }
    else if (colormap == "__first__")
    {
        m_curColorMapIndex = 0;
        retval += apiPaletteGetColorBarIdx(m_curColorMapIndex, newPalette);
    }
    else
    {
        retval += apiPaletteGetColorBarName(colormap, newPalette);
    }

    if (retval.containsError() && retval.errorMessage() != NULL)
    {
        emit statusBarMessage(QString("%1").arg(retval.errorMessage()), 4000);
        return false;
    }
    else if (retval.containsError())
    {
        emit statusBarMessage("error when loading color map", 4000);
        return false;
    }

    int totalStops = newPalette.colorStops.size();

    if (totalStops < 2)
    {
        emit statusBarMessage(tr("Selected color map has less than two points."), 4000);
        return false;
    }

    m_colorMapName = newPalette.name;

/*
    if (newPalette.getPos(newPalette.getSize() - 1) == newPalette.getPos(newPalette.getSize() - 2))  // BuxFix - For Gray-Marked
    {
        colorMap = new QwtLinearColorMap(newPalette.getColorFirst(), newPalette.getColor(newPalette.getSize() - 2), QwtColorMap::Indexed);
        colorBarMap = new QwtLinearColorMap(newPalette.getColorFirst(), newPalette.getColor(newPalette.getSize() - 2), QwtColorMap::Indexed);
        if (newPalette.getSize() > 2)
        {
            for (int i = 1; i < newPalette.getSize() - 2; i++)
            {
                colorMap->addColorStop(newPalette.getPos(i), newPalette.getColor(i));
                colorBarMap->addColorStop(newPalette.getPos(i), newPalette.getColor(i));
            }
            colorMap->addColorStop(newPalette.getPos(newPalette.getSize() - 1), newPalette.getColor(newPalette.getSize() - 1));
            colorBarMap->addColorStop(newPalette.getPos(newPalette.getSize() - 1), newPalette.getColor(newPalette.getSize() - 1));
        }
    }
    else
    {
        colorMap = new QwtLinearColorMap(newPalette.getColorFirst(), newPalette.getColorLast(), QwtColorMap::Indexed);
        colorBarMap = new QwtLinearColorMap(newPalette.getColorFirst(), newPalette.getColorLast(), QwtColorMap::Indexed);
        if (newPalette.getSize() > 2)
        {
            for (int i = 1; i < newPalette.getSize() - 1; i++)
            {
                colorMap->addColorStop(newPalette.getPos(i), newPalette.getColor(i));
                colorBarMap->addColorStop(newPalette.getPos(i), newPalette.getColor(i));
            }
        }
    }
*/

    //if (newPalette.inverseColorOne.isValid() /* && newPalette.inverseColorTwo.isValid() */)
    //{
    //    m_pLineCutPicker->setRubberBandPen(QPen(newPalette.inverseColorOne));
    //    m_pLineCutPicker->setTrackerPen(QPen(newPalette.inverseColorOne));
    //    m_pLineCutLine->setPen(newPalette.inverseColorOne);
    //}
    //else
    //{
    //    m_pLineCutPicker->setRubberBandPen(QPen(Qt::gray));
    //    m_pLineCutPicker->setTrackerPen(QPen(Qt::gray));
    //    m_pLineCutLine->setPen(Qt::gray);
    //}
    m_inverseColor0 = newPalette.inverseColorOne;
    m_inverseColor1 = newPalette.inverseColorTwo;
    refreshStyles();

    if (newPalette.colorStops[totalStops - 1].first == newPalette.colorStops[totalStops - 2].first)  // BuxFix - For Gray-Marked
    {
        colorMap    = new QwtLinearColorMap(newPalette.colorStops[0].second, newPalette.colorStops[totalStops - 2].second, QwtColorMap::Indexed);
        colorBarMap = new QwtLinearColorMap(newPalette.colorStops[0].second, newPalette.colorStops[totalStops - 2].second, QwtColorMap::Indexed);
        if (totalStops > 2)
        {
            for (int i = 1; i < totalStops - 2; i++)
            {
                colorMap->addColorStop(newPalette.colorStops[i].first, newPalette.colorStops[i].second);
                colorBarMap->addColorStop(newPalette.colorStops[i].first, newPalette.colorStops[i].second);
            }
            colorMap->addColorStop(newPalette.colorStops[totalStops-1].first, newPalette.colorStops[totalStops-1].second);
            colorBarMap->addColorStop(newPalette.colorStops[totalStops-1].first, newPalette.colorStops[totalStops-1].second);
        }
    }
    else
    {
        colorMap    = new QwtLinearColorMap(newPalette.colorStops.first().second, newPalette.colorStops.last().second, QwtColorMap::Indexed);
        colorBarMap = new QwtLinearColorMap(newPalette.colorStops.first().second, newPalette.colorStops.last().second, QwtColorMap::Indexed);
        if (totalStops > 2)
        {
            for (int i = 1; i < totalStops - 1; i++)
            {
                colorMap->addColorStop(newPalette.colorStops[i].first, newPalette.colorStops[i].second);
                colorBarMap->addColorStop(newPalette.colorStops[i].first, newPalette.colorStops[i].second);
            }
        }
    }

    if (colorMap && colorBarMap)
    {
        if (m_rasterData)
        {
            QwtInterval interval = m_rasterData->interval(Qt::ZAxis);
            /*setAxisScale(QwtPlot::yRight, interval.minValue(), interval.maxValue());

            axisScale(QwtPlot::yRight, m_pData->m_valueMin, m_pData->m_valueMax);
*/
            axisWidget(QwtPlot::yRight)->setColorMap(interval, colorBarMap);
        }
        else
        {
            axisWidget(QwtPlot::yRight)->setColorMap(QwtInterval(0,1.0), colorBarMap);
        }

        m_dObjItem->setColorMap(colorMap);
    }
    else
    {
        delete colorMap;
        delete colorBarMap;
    }

    replot();
    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotCanvas::setOverlayColorMap(QString colormap /*= "__next__"*/)
{
    QwtLinearColorMap *colorMap = NULL;
    ito::ItomPalette newPalette;
    ito::RetVal retval(ito::retOk);
    int numPalettes = 1;

    if(!ito::ITOM_API_FUNCS_GRAPH)
    {
        emit statusBarMessage(tr("Could not change color bar, api is missing"), 4000);
        return false;
    }

    retval += apiPaletteGetNumberOfColorBars(numPalettes);

    if (numPalettes == 0 || retval.containsError())
    {
        emit statusBarMessage(tr("No color maps defined."), 4000);
        return false;
    }

    if (colormap == "__next__")
    {
        m_curOverlayColorMapIndex++;
        m_curOverlayColorMapIndex %= numPalettes; //map index to [0,numPalettes)
        retval += apiPaletteGetColorBarIdx(m_curOverlayColorMapIndex, newPalette);
    }
    else if (colormap == "__first__")
    {
        m_curOverlayColorMapIndex = 0;
        retval += apiPaletteGetColorBarIdx(m_curOverlayColorMapIndex, newPalette);
    }
    else
    {
        retval += apiPaletteGetColorBarName(colormap, newPalette);
    }

    if (retval.containsError() && retval.errorMessage() != NULL)
    {
        emit statusBarMessage(QString("%1").arg(retval.errorMessage()), 4000);
        return false;
    }
    else if (retval.containsError())
    {
        emit statusBarMessage("error when loading color map", 4000);
        return false;
    }

    int totalStops = newPalette.colorStops.size();

    if (totalStops < 2)
    {
        emit statusBarMessage(tr("Selected color map has less than two points."), 4000);
        return false;
    }

    m_colorOverlayMapName = newPalette.name;


    if (newPalette.colorStops[totalStops - 1].first == newPalette.colorStops[totalStops - 2].first)  // BuxFix - For Gray-Marked
    {
        colorMap    = new QwtLinearColorMap(newPalette.colorStops[0].second, newPalette.colorStops[totalStops - 2].second, QwtColorMap::Indexed);
        if (totalStops > 2)
        {
            for (int i = 1; i < totalStops - 2; i++)
            {
                colorMap->addColorStop(newPalette.colorStops[i].first, newPalette.colorStops[i].second);
            }
            colorMap->addColorStop(newPalette.colorStops[totalStops-1].first, newPalette.colorStops[totalStops-1].second);
        }
    }
    else
    {
        colorMap    = new QwtLinearColorMap(newPalette.colorStops.first().second, newPalette.colorStops.last().second, QwtColorMap::Indexed);
        if (totalStops > 2)
        {
            for (int i = 1; i < totalStops - 1; i++)
            {
                colorMap->addColorStop(newPalette.colorStops[i].first, newPalette.colorStops[i].second);
             }
        }
    }

    if (colorMap)
    {
        this->m_dOverlayItem->setColorMap(colorMap);
    }
    else
    {
        delete colorMap;
    }

    replot();
    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::keyPressEvent (QKeyEvent * event)
{
    event->ignore();

    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
    m_activeModifiers = event->modifiers();

    QPointF incr;

    if (event->modifiers() & Qt::ControlModifier)
    {
        //10-pixel increment
        incr.setX(invTransform(QwtPlot::xBottom, 10) - invTransform(QwtPlot::xBottom, 0));
        incr.setY(invTransform(QwtPlot::yLeft, 10) - invTransform(QwtPlot::yLeft, 0));
    }
    else
    {
        //1-pixel increment
        incr.setX(invTransform(QwtPlot::xBottom, 1) - invTransform(QwtPlot::xBottom, 0));
        incr.setY(invTransform(QwtPlot::yLeft, 1) - invTransform(QwtPlot::yLeft, 0));
    }

    if (m_pData->m_state == tStackCut)
    {
        QPointF markerPosScaleCoords = m_pStackCutMarker->value();

        event->accept();

        switch(event->key())
        {
            case Qt::Key_Left:
                markerPosScaleCoords.rx()-=incr.rx();
                break;
            case Qt::Key_Right:
                markerPosScaleCoords.rx()+=incr.rx();
                break;
            case Qt::Key_Up:
                markerPosScaleCoords.ry()-=incr.ry();
                break;
            case Qt::Key_Down:
                markerPosScaleCoords.ry()+=incr.ry();
                break;
            default:
                event->ignore();
            break;
        }

        if (event->isAccepted() && m_rasterData->pointValid(markerPosScaleCoords))
        {
            m_pStackCutMarker->setValue(markerPosScaleCoords);
            m_pStackCutMarker->setVisible(true);

            QVector<QPointF> pts;
            pts.append(markerPosScaleCoords);
            (p)->displayCut(pts, m_zstackCutUID,true);
            replot();
        }
    }
    else if (m_pData->m_state == tLineCut)
    {
        QVector<QPointF> pts;

        event->accept();

        if (event->key() == Qt::Key_H) //draw horizontal line in the middle of the plotted dataObject
        {
            QwtInterval hInterval = m_rasterData->interval(Qt::XAxis);
            QwtInterval vInterval = m_rasterData->interval(Qt::YAxis);

            pts.append(QPointF(hInterval.minValue(), (vInterval.minValue() + vInterval.maxValue())*0.5));
            pts.append(QPointF(hInterval.maxValue(), (vInterval.minValue() + vInterval.maxValue())*0.5));

            if (p)
            {
                p->setCoordinates(pts, true);
            }
        }
        else if (event->key() == Qt::Key_V) // draw vertical line in the middle of the plotted dataObject
        {
            QwtInterval hInterval = m_rasterData->interval(Qt::XAxis);
            QwtInterval vInterval = m_rasterData->interval(Qt::YAxis);

            pts.append(QPointF((hInterval.minValue() + hInterval.maxValue())*0.5, vInterval.minValue()));
            pts.append(QPointF((hInterval.minValue() + hInterval.maxValue())*0.5, vInterval.maxValue()));

            if (p)
            {
                p->setCoordinates(pts, true);
            }
        }
        else
        {
            pts.append(m_pLineCutLine->sample(0));
            pts.append(m_pLineCutLine->sample(1));

            switch(event->key())
            {
                case Qt::Key_Left:
                    pts[0].rx()-=incr.rx();
                    pts[1].rx()-=incr.rx();
                    break;
                case Qt::Key_Right:
                    pts[0].rx()+=incr.rx();
                    pts[1].rx()+=incr.rx();
                    break;
                case Qt::Key_Up:
                    pts[0].ry()-=incr.ry();
                    pts[1].ry()-=incr.ry();
                    break;
                case Qt::Key_Down:
                    pts[0].ry()+=incr.ry();
                    pts[1].ry()+=incr.ry();
                    break;
                default:
                    event->ignore();
                    break;
            }

            if (event->isAccepted() && p)
            {
                p->setCoordinates(pts, true);
            }
        }

        if (event->isAccepted() && m_rasterData->pointValid(pts[0]) && m_rasterData->pointValid(pts[1]))
        {
            m_pLineCutLine->setSamples(pts);
            m_pLineCutLine->setVisible(true);

            if(m_dObjPtr->getDims() > 2)
            {
                pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(),m_rasterData->getCurrentPlane()));
            }
            (p)->displayCut(pts, m_lineCutUID, false);
            
            replot();
        }
    }
    
    //check if the current event has not been accepted yet (special things to do)
    if(event->isAccepted() == false && event->matches(QKeySequence::Copy))
    {
        event->accept();
        p->copyToClipBoard();
    }
    else
    {
        event->ignore(); //Clearing the accept parameter indicates that the event receiver does not want the event. Unwanted events might be propagated to the parent widget.
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::keyReleaseEvent (QKeyEvent* /*event*/)
{
    m_activeModifiers = Qt::NoModifier;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setColorBarVisible(bool visible)
{
    m_pData->m_colorBarVisible = visible;
    enableAxis(QwtPlot::yRight, visible);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setLabels(const QString &title, const QString &valueLabel, const QString &xAxisLabel, const QString &yAxisLabel)
{
    if (m_pData->m_autoValueLabel)
    {
        setAxisTitle(QwtPlot::yRight, valueLabel);
    }
    else
    {
        setAxisTitle(QwtPlot::yRight, m_pData->m_valueLabel);
    }

    if (m_pData->m_autoxAxisLabel)
    {
        setAxisTitle(QwtPlot::xBottom, xAxisLabel);
    }
    else
    {
        setAxisTitle(QwtPlot::xBottom, m_pData->m_xaxisLabel);
    }

    if (m_pData->m_autoyAxisLabel)
    {
        setAxisTitle(QwtPlot::yLeft, yAxisLabel);
    }
    else
    {
        setAxisTitle(QwtPlot::yLeft, m_pData->m_yaxisLabel);
    }

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
void PlotCanvas::updateLabels()
{
    if (m_pData->m_autoValueLabel)
    {
        setAxisTitle(QwtPlot::yRight, m_pData->m_valueLabelDObj);
    }
    else
    {
        setAxisTitle(QwtPlot::yRight, m_pData->m_valueLabel);
    }

    if (m_pData->m_autoxAxisLabel)
    {
        setAxisTitle(QwtPlot::xBottom, m_pData->m_xaxisLabelDObj);
    }
    else
    {
        setAxisTitle(QwtPlot::xBottom, m_pData->m_xaxisLabel);
    }

    if (m_pData->m_autoyAxisLabel)
    {
        setAxisTitle(QwtPlot::yLeft, m_pData->m_yaxisLabelDObj);
    }
    else
    {
        setAxisTitle(QwtPlot::yLeft, m_pData->m_yaxisLabel);
    }

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
void PlotCanvas::synchronizeScaleValues()
{
    QwtInterval ival = m_rasterData->interval(Qt::ZAxis);
    m_pData->m_valueMin = ival.minValue();
    m_pData->m_valueMax = ival.maxValue();

    if (m_pData->m_xaxisScaleAuto)
    {
        ival = m_rasterData->interval(Qt::XAxis);
        
    }
    else
    {
        ival = axisScaleDiv(xBottom).interval();
    }
    m_pData->m_xaxisMin = ival.minValue();
    m_pData->m_xaxisMax = ival.maxValue();

    if (m_pData->m_yaxisScaleAuto)
    {
        ival = m_rasterData->interval(Qt::YAxis);
    }
    else
    {
        ival = axisScaleDiv(yLeft).interval();
    }
    m_pData->m_yaxisMin = ival.minValue();
    m_pData->m_yaxisMax = ival.maxValue();
}

//----------------------------------------------------------------------------------------------------------------------------------
/*
@param doReplot forces a replot of the content
@param doZoomBase if true, the x/y-zoom is reverted to the full x-y-area of the manually set ranges (the same holds for the value range)
*/
void PlotCanvas::updateScaleValues(bool doReplot /*= true*/, bool doZoomBase /*= true*/)
{
    QwtInterval ival;
    if (m_pData->m_valueScaleAuto)
    {
        internalDataUpdated();
        ival = m_rasterData->interval(Qt::ZAxis);
        m_pData->m_valueMin = ival.minValue();
        m_pData->m_valueMax = ival.maxValue();
    }
    else
    {
        m_rasterData->setInterval(Qt::ZAxis, QwtInterval(m_pData->m_valueMin, m_pData->m_valueMax));
    }

    if (m_pData->m_xaxisScaleAuto)
    {
        ival = m_rasterData->interval(Qt::XAxis);
        m_pData->m_xaxisMin = ival.minValue();
        m_pData->m_xaxisMax = ival.maxValue();
    }

    if (m_pData->m_yaxisScaleAuto)
    {
        ival = m_rasterData->interval(Qt::YAxis);
        m_pData->m_yaxisMin = ival.minValue();
        m_pData->m_yaxisMax = ival.maxValue();
    }

    if (m_pData->m_yaxisFlipped)
    {
        axisScaleEngine(QwtPlot::yLeft)->setAttribute(QwtScaleEngine::Inverted, true);
    }
    else
    {
        axisScaleEngine(QwtPlot::yLeft)->setAttribute(QwtScaleEngine::Inverted, false);
    }

    setAxisScale(QwtPlot::yRight, m_pData->m_valueMin, m_pData->m_valueMax);
    QwtScaleWidget *widget = axisWidget(QwtPlot::yRight);
    if (widget)
    {
        QwtInterval ival(m_pData->m_valueMin, m_pData->m_valueMax);
        axisWidget(QwtPlot::yRight)->setColorMap(ival, const_cast<QwtColorMap*>(widget->colorMap())); //the color map should be unchanged
    }

    if (doZoomBase)
    {
        // 10.02.15 ck we don't want to check if a zoomer exists, as it is always created in the constructor but if it is enabled
        if (m_pZoomer->isEnabled())
        {
            QRectF zoom(m_pData->m_xaxisMin, m_pData->m_yaxisMin, (m_pData->m_xaxisMax - m_pData->m_xaxisMin), (m_pData->m_yaxisMax - m_pData->m_yaxisMin));
            zoom = zoom.normalized();

            if (zoom == m_pZoomer->zoomRect())
            {
                m_pZoomer->zoom(zoom);
                m_pZoomer->rescale(false); //zoom of zoomer does not call rescale in this case, therefore we do it here
            }
            else
            {
                m_pZoomer->zoom(zoom);
            }
        }
        else
        {
            setAxisScale(QwtPlot::xBottom, m_pData->m_xaxisMin, m_pData->m_xaxisMax);

            if (m_pData->m_yaxisFlipped)
            {
                setAxisScale(QwtPlot::yLeft, m_pData->m_yaxisMax, m_pData->m_yaxisMin);
            }
            else
            {
                setAxisScale(QwtPlot::yLeft, m_pData->m_yaxisMin, m_pData->m_yaxisMax);
            }

            QRectF zoom(m_pData->m_xaxisMin, m_pData->m_yaxisMin, (m_pData->m_xaxisMax - m_pData->m_xaxisMin), (m_pData->m_yaxisMax - m_pData->m_yaxisMin));
            zoom = zoom.normalized();
            m_pZoomer->appendZoomStack(zoom);

        }
    }

    if (doReplot)
    {
        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setInterval(Qt::Axis axis, const ito::AutoInterval &interval)
{
    if (axis == Qt::XAxis)
    {
        m_pData->m_xaxisScaleAuto = interval.isAuto();

        if (m_pData->m_xaxisScaleAuto)
        {
            QwtInterval ival = m_rasterData->interval(Qt::XAxis);
            m_pData->m_valueMin = ival.minValue();
            m_pData->m_valueMax = ival.maxValue();
        }
        else
        {
            m_pData->m_xaxisMin = interval.minimum();
            m_pData->m_xaxisMax = interval.maximum();
        }
        setAxisScale(QwtPlot::xBottom, m_pData->m_xaxisMin, m_pData->m_xaxisMax);
    }
    else if (axis == Qt::YAxis)
    {
        m_pData->m_yaxisScaleAuto = interval.isAuto();

        if (m_pData->m_xaxisScaleAuto)
        {
            QwtInterval ival = m_rasterData->interval(Qt::YAxis);
            m_pData->m_yaxisMin = ival.minValue();
            m_pData->m_yaxisMax = ival.maxValue();
        }
        else
        {
            m_pData->m_yaxisMin = interval.minimum();
            m_pData->m_yaxisMax = interval.maximum();
        }

        QwtScaleEngine *scaleEngine = axisScaleEngine(QwtPlot::yLeft);

        if (m_pData->m_yaxisFlipped)
        {
            scaleEngine->setAttribute(QwtScaleEngine::Inverted, true);
            setAxisScale(QwtPlot::yLeft, m_pData->m_yaxisMax, m_pData->m_yaxisMin);
        }
        else
        {
            scaleEngine->setAttribute(QwtScaleEngine::Inverted, false);
            setAxisScale(QwtPlot::yLeft, m_pData->m_yaxisMin, m_pData->m_yaxisMax);
        }

    }
    else if (axis == Qt::ZAxis)
    {
        if (m_pData->m_dataType == ito::tRGBA32)
        {
            m_pData->m_valueScaleAuto = false;
            m_pData->m_valueMin = std::numeric_limits<ito::uint8>::min();
            m_pData->m_valueMax = std::numeric_limits<ito::uint8>::max();
        }
        else
        {
            m_pData->m_valueScaleAuto = interval.isAuto();

            if (m_pData->m_valueScaleAuto)
            {
                internalDataUpdated();
                QwtInterval ival = m_rasterData->interval(Qt::ZAxis);
                m_pData->m_valueMin = ival.minValue();
                m_pData->m_valueMax = ival.maxValue();
            }
            else
            {
                m_pData->m_valueMin = interval.minimum();
                m_pData->m_valueMax = interval.maximum();
            }
        }

        QwtScaleWidget *widget = axisWidget(QwtPlot::yRight);

        if (widget)
        {
            QwtInterval ival(interval.minimum(), interval.maximum());
            widget->setColorMap(ival, const_cast<QwtColorMap*>(widget->colorMap())); //the color map should be unchanged
        }

        setAxisScale(QwtPlot::yRight, m_pData->m_valueMin, m_pData->m_valueMax);

        m_rasterData->setInterval(Qt::ZAxis, QwtInterval(m_pData->m_valueMin, m_pData->m_valueMax));
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval PlotCanvas::getInterval(Qt::Axis axis) const
{
    QwtInterval i;
    bool autoScale;

    switch (axis)
    {
    case Qt::ZAxis:
        i = m_rasterData->interval(axis);
        autoScale = m_pData->m_valueScaleAuto;
        break;
    case Qt::XAxis:
        {
            QwtScaleDiv div =axisScaleDiv(QwtPlot::xBottom);
            i = div.interval();
            autoScale = m_pData->m_xaxisScaleAuto;
        }
        break;
    case Qt::YAxis:
        {
            QwtScaleDiv div =axisScaleDiv(QwtPlot::yLeft);
            i = div.interval();
            autoScale = m_pData->m_yaxisScaleAuto;
        }
    }

    return ito::AutoInterval(i.minValue(), i.maxValue(), autoScale);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setOverlayInterval(Qt::Axis axis, const ito::AutoInterval &interval)
{
    if (axis == Qt::ZAxis)
    {
        m_pData->m_overlayScaleAuto = false;
        m_pData->m_overlayMin = interval.minimum();
        m_pData->m_overlayMax = interval.maximum();

        m_rasterOverlayData->setInterval(Qt::ZAxis, QwtInterval(m_pData->m_overlayMin, m_pData->m_overlayMax));
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval PlotCanvas::getOverlayInterval(Qt::Axis axis) const
{
    QwtInterval i = m_rasterOverlayData->interval(axis);
    return ito::AutoInterval(i.minValue(), i.maxValue(), m_pData->m_overlayScaleAuto);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::int32 PlotCanvas::getCurrentPlane() const
{
    return m_rasterData->getCurrentPlane();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setState(tState state)
{
    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());

    m_pCenterMarker->setVisible(m_pData->m_showCenterMarker);
    if (m_pData->m_showCenterMarker && m_dObjPtr)
    {
        if (m_dObjPtr->getDims() > 1)
        {
            bool valid;
            m_pCenterMarker->setXValue(m_dObjPtr->getPixToPhys(m_dObjPtr->getDims()-1, (m_dObjPtr->getSize(m_dObjPtr->getDims()-1) - 1) / 2.0, valid));
            m_pCenterMarker->setYValue(m_dObjPtr->getPixToPhys(m_dObjPtr->getDims()-2, (m_dObjPtr->getSize(m_dObjPtr->getDims()-2) - 1) / 2.0, valid));
        }   
    }

    if (m_pData->m_state != state)
    {
        if ((m_pData->m_state == tMultiPointPick || m_pData->m_state == tPoint
            || m_pData->m_state == tLine || m_pData->m_state == tRect || m_pData->m_state == tEllipse) && state != tIdle)
        {
            return; //multiPointPick needs to go back to idle
        }

        if (m_pZoomer) m_pZoomer->setEnabled(state == tZoom);
        if (m_pPanner) m_pPanner->setEnabled(state == tPan);
        if (m_pValuePicker) m_pValuePicker->setEnabled(state == tValuePicker);
        if (m_pLineCutPicker) m_pLineCutPicker->setEnabled(state == tLineCut);
        if (m_pStackPicker) m_pStackPicker->setEnabled(state == tStackCut);
        //if (m_pMultiPointPicker) m_pMultiPointPicker->setEnabled(state == tMultiPointPick);

        if (state == tMultiPointPick || m_pData->m_state == tPoint || m_pData->m_state == tLine
            || m_pData->m_state == tRect || m_pData->m_state == tEllipse || state == tIdle)
        {
            if (p)
            {
                p->m_pActZoom->setEnabled(state == tIdle);
                p->m_pActPan->setEnabled(state == tIdle);
                p->m_pActLineCut->setEnabled(state == tIdle);
                p->m_pActStackCut->setEnabled(state == tIdle);
                p->m_pActValuePicker->setEnabled(state == tIdle);
            }
        }

        if (state == tZoom || state == tPan || state == tMultiPointPick || m_pData->m_state == tPoint || m_pData->m_state == tLine
            || m_pData->m_state == tRect || m_pData->m_state == tEllipse || state == tValuePicker || state == tIdle)
        {
            if (p)
            {
                p->setCoordinates(QVector<QPointF>(),false);
            }
        }

        switch (state)
        {
            default:
            case tIdle:
                canvas()->setCursor(Qt::ArrowCursor);
            break;

            case tZoom:
                canvas()->setCursor(Qt::CrossCursor);
            break;

            case tPan:
                canvas()->setCursor(Qt::OpenHandCursor);
            break;

            case tValuePicker:
                canvas()->setCursor(Qt::CrossCursor);
            break;

            case tStackCut:
                canvas()->setCursor(Qt::CrossCursor);
            break;

            case tMultiPointPick:
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
// a1 is line1 start, a2 is line1 end, b1 is line2 start, b2 is line2 end
bool PlotCanvas::lineIntersection(const QPointF &a1, const QPointF &a2, const QPointF &b1, const QPointF &b2, QPointF &intersection)
{
    intersection = QPointF(0,0);

    QPointF b = a2 - a1;
    QPointF d = b2 - b1;
    float bDotDPerp = b.x() * d.y() - b.y() * d.x();

    // if b dot d == 0, it means the lines are parallel so have infinite intersection points
    if (bDotDPerp == 0)
        return false;

    QPointF c = b1 - a1;
    float t = (c.x() * d.y() - c.y() * d.x()) / bDotDPerp;
    if (t < 0 || t > 1)
        return false;

    float u = (c.x() * b.y() - c.y() * b.x()) / bDotDPerp;
    if (u < 0 || u > 1)
        return false;

    intersection = a1 + t * b;

    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::zStackCutTrackerAppended(const QPoint &pt)
{
    QVector<QPointF> pts;

    if (m_pData->m_state == tStackCut)
    {
        
        QPointF ptScale;
        ptScale.setY(invTransform(QwtPlot::yLeft, pt.y()));
        ptScale.setX(invTransform(QwtPlot::xBottom, pt.x()));
        if (m_rasterData->pointValid(ptScale))
        {
            m_pStackCutMarker->setValue(ptScale);
            m_pStackCutMarker->setVisible(true);

            pts.append(ptScale);
            ((Itom2dQwtPlot*)parent())->setCoordinates(pts, true);
            ((Itom2dQwtPlot*)parent())->displayCut(pts, m_zstackCutUID,true);
            replot();
        }   
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::zStackCutTrackerMoved(const QPoint &pt)
{
    QVector<QPointF> pts;

    if (m_pData->m_state == tStackCut)
    {
        QPointF ptScale;
        ptScale.setY(invTransform(QwtPlot::yLeft, pt.y()));
        ptScale.setX(invTransform(QwtPlot::xBottom, pt.x()));
        if (m_rasterData->pointValid(ptScale))
        {
            if (m_activeModifiers.testFlag(Qt::ControlModifier))
            {
                if (abs(ptScale.x() - m_pStackCutMarker->xValue()) > abs(ptScale.y() - m_pStackCutMarker->yValue()))
                {
                    ptScale.setY(m_pStackCutMarker->yValue());
                }
                else
                {
                    ptScale.setX(m_pStackCutMarker->xValue());
                }
            }

            m_pStackCutMarker->setValue(ptScale);
            m_pStackCutMarker->setVisible(true);

            pts.append(ptScale);
            ((Itom2dQwtPlot*)parent())->setCoordinates(pts, true);
            ((Itom2dQwtPlot*)parent())->displayCut(pts, m_zstackCutUID,true);
            replot();
        }    
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::lineCutMoved(const QPoint &pt)
{
    QVector<QPointF> pts;
    pts.resize(2);

    if (m_pData->m_state == tLineCut)
    {
        QwtInterval xInterval = m_rasterData->interval(Qt::XAxis); 
        QwtInterval yInterval = m_rasterData->interval(Qt::YAxis);

        pts[0] = m_pLineCutLine->sample(0);

        if (m_lineCutValidStart)
        {
            pts[1].setY(invTransform(QwtPlot::yLeft, pt.y()));
            pts[1].setX(invTransform(QwtPlot::xBottom, pt.x()));

            if (!xInterval.contains(pts[1].x()) || !yInterval.contains(pts[1].y()))
            {
                QPointF intersection;
                QPointF rt(xInterval.maxValue(), yInterval.maxValue());
                QPointF lt(xInterval.minValue(), yInterval.maxValue());
                QPointF rb(xInterval.maxValue(), yInterval.minValue());
                QPointF lb(xInterval.minValue(), yInterval.minValue());
                
                if ((pts[1].x() < pts[0].x()) && lineIntersection(pts[1], pts[0], lt, lb, intersection))
                {
                    //try if pts[1] - pts[0] intersects with left border
                    pts[1] = intersection;
                }
                else if ((pts[1].x() > pts[0].x()) && lineIntersection(pts[1], pts[0], rt, rb, intersection))
                {
                    //try if pts[1] - pts[0] intersects with right border
                    pts[1] = intersection;
                }
                else if ((pts[1].y() > pts[0].y()) && lineIntersection(pts[1], pts[0], lt, rt, intersection))
                {
                    //try if pts[1] - pts[0] intersects with top border
                    pts[1] = intersection;
                }
                else if ((pts[1].y() < pts[0].y()) && lineIntersection(pts[1], pts[0], lb, rb, intersection))
                {
                    //try if pts[1] - pts[0] intersects with bottom border
                    pts[1] = intersection;
                }
            }

            if (m_activeModifiers.testFlag(Qt::ControlModifier))
            {
                if (abs(pts[1].x() - pts[0].x()) > abs(pts[1].y() - pts[0].y()))
                {
                    pts[1].setY(pts[0].y());
                }
                else
                {
                    pts[1].setX(pts[0].x());
                }
            }

            m_pLineCutLine->setSamples(pts);

            ((Itom2dQwtPlot*)parent())->setCoordinates(pts, true);

            if(m_dObjPtr->getDims() > 2)
            {
                pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(),m_rasterData->getCurrentPlane()));
            }
            ((Itom2dQwtPlot*)parent())->displayCut(pts, m_lineCutUID, false);

            replot();
        }
        else //first point is still not valid, try to find a valid first point
        {
            QPointF pts0 = pts[0];

            pts[0].setY(invTransform(QwtPlot::yLeft, pt.y()));
            pts[0].setX(invTransform(QwtPlot::xBottom, pt.x()));

            if (m_rasterData->pointValid(pts[0]))
            {
                //the mouse cursor is now in a valid area. Due to discretization limits, it might be that this point is not exactly at the border
                //towards the direction where the mouse has been pressed for the first time. Therefore get the intersection to the initial mouse press
                //position.
                QPointF intersection;
                QPointF rt(xInterval.maxValue(), yInterval.maxValue());
                QPointF lt(xInterval.minValue(), yInterval.maxValue());
                QPointF rb(xInterval.maxValue(), yInterval.minValue());
                QPointF lb(xInterval.minValue(), yInterval.minValue());
                
                if (lineIntersection(pts[0], pts0, lt, lb, intersection))
                {
                    //try if pts[1] - pts[0] intersects with left border
                    pts[0] = intersection;
                }
                else if (lineIntersection(pts[0], pts0, rt, rb, intersection))
                {
                    //try if pts[1] - pts[0] intersects with right border
                    pts[0] = intersection;
                }
                else if (lineIntersection(pts[0], pts0, lt, rt, intersection))
                {
                    //try if pts[1] - pts[0] intersects with top border
                    pts[0] = intersection;
                }
                else if (lineIntersection(pts[0], pts0, lb, rb, intersection))
                {
                    //try if pts[1] - pts[0] intersects with bottom border
                    pts[0] = intersection;
                }

                m_lineCutValidStart = true;
            }
            else
            {
                //first point not valid
                m_lineCutValidStart = false;
            }

            pts[1] = pts[0];

            m_pLineCutLine->setVisible(true);
            m_pLineCutLine->setSamples(pts);

            ((Itom2dQwtPlot*)parent())->setCoordinates(pts, true);

            // check for m_dObjPtr first otherwise crash
            if(m_dObjPtr && m_dObjPtr->getDims() > 2)
            {
                pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(),m_rasterData->getCurrentPlane()));
            }

            ((Itom2dQwtPlot*)parent())->displayCut(pts, m_lineCutUID, false);

            replot();
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::lineCutAppended(const QPoint &pt)
{
    QVector<QPointF> pts;
    pts.resize(2);

    if (m_pData->m_state == tLineCut)
    {
        pts[0].setY(invTransform(QwtPlot::yLeft, pt.y()));
        pts[0].setX(invTransform(QwtPlot::xBottom, pt.x()));

        if (m_rasterData->pointValid(pts[0]))
        {
            m_lineCutValidStart = true;
        }
        else
        {
            //first point not valid
            m_lineCutValidStart = false;
        }

        pts[1] = pts[0];

        m_pLineCutLine->setVisible(true);
        m_pLineCutLine->setSamples(pts);

        ((Itom2dQwtPlot*)parent())->setCoordinates(pts, true);

        // check for m_dObjPtr first otherwise crash
        if(m_dObjPtr && m_dObjPtr->getDims() > 2)
        {
            pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(),m_rasterData->getCurrentPlane()));
        }
        
        ((Itom2dQwtPlot*)parent())->displayCut(pts, m_lineCutUID, false);

        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::childFigureDestroyed(QObject* obj, ito::uint32 UID)
{
    if (UID > 0)
    {
        if (UID == m_zstackCutUID)
        {
            m_pStackCutMarker->setVisible(false);
        }
        else if (UID == m_lineCutUID)
        {
            m_pLineCutLine->setVisible(false);
            Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
            if (p)
            {
                p->setCoordinates(QVector<QPointF>(), false);
            }
        }
    }
    else
    {
        //hide all related markers (we have no further information)
        m_pStackCutMarker->setVisible(false);
        m_pLineCutLine->setVisible(false);
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotCanvas::plotMarkers(const ito::DataObject *coords, QString style, QString id, int plane)
{
    ito::RetVal retval;
    int limits[] = {2,8,0,99999};

    if(!ito::ITOM_API_FUNCS_GRAPH)
    {
        emit statusBarMessage(tr("Could not plot marker, api is missing"), 4000);
        return ito::RetVal(ito::retError, 0, tr("Could not plot marker, api is missing").toLatin1().data());
    }

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
        retval += ito::RetVal(ito::retError, 0, tr("The style tag does not correspond to the required format").toLatin1().data());
    }

    if (!retval.containsError())
    {
        if (dObj->getSize(0) <= 2)
        {
            //QMultiHash<QString, QPair<int, QwtPlotMarker*> > m_plotMarkers;
            QwtPlotMarker *marker = NULL;
            int nrOfMarkers = dObj->getSize(1);

            if (id == "") id = "unknown";

            ito::float32 *xCoords = (ito::float32*)dObj->rowPtr(0,0);
            ito::float32 *yCoords = (ito::float32*)dObj->rowPtr(0,1);

            for (int i = 0; i < nrOfMarkers; ++i)
            {
                marker = new QwtPlotMarker();
                marker->setSymbol(new QwtSymbol(symStyle,symBrush,symPen,symSize));
                marker->setValue(xCoords[i], yCoords[i]);
                marker->attach(this);
                if(m_pData->m_markerLabelVisible)
                {
                    QwtText label(QString(" %1").arg(id));
                    marker->setLabel(label);
                }
                
                
                m_plotMarkers.insert(id, QPair<int, QwtPlotMarker*>(plane, marker));
            }
        }
        else if (dObj->getSize(0) >= 8)
        {
//            QwtPlotMarker *marker = NULL;
            int nrOfMarkers = dObj->getSize(1);
            
            if (id == "") id = "unknown";

            ito::float32 *ids = (ito::float32*)dObj->rowPtr(0, 0);            
            ito::float32 *types = (ito::float32*)dObj->rowPtr(0, 1);
            ito::float32 *xCoords1 = (ito::float32*)dObj->rowPtr(0, 2);
            ito::float32 *yCoords1 = (ito::float32*)dObj->rowPtr(0, 3);

            ito::float32 *xCoords2 = (ito::float32*)dObj->rowPtr(0, 4);
            ito::float32 *yCoords2 = (ito::float32*)dObj->rowPtr(0, 5);
            
            // The definition do not correspond to the definetion of primitiv elements

            for (int i = 0; i < nrOfMarkers; ++i)
            {
                QPainterPath path;
                DrawItem *newItem = NULL;                
                unsigned short type = ((int)types[i]) & ito::PrimitiveContainer::tTypeMask;
                unsigned char flags = (((int)types[i]) & ito::PrimitiveContainer::tFlagMask) >> 16;
                switch (type)
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
                    m_pData->m_pDrawItems[(int)ids[i]]->m_flags = flags;
                }
                else
                {
                    switch (type)
                    {
                        case tMultiPointPick:
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
                        /*
                        if (this->m_inverseColor0.isValid())
                        {
                            newItem->setPen(QPen(m_inverseColor0, 1.0));
//                            newItem->setBrush(QBrush(m_inverseColor0));
                        }
                        else newItem->setPen(QPen(Qt::green, 1.0));
                        */

                        newItem->setShape(path, m_inverseColor0, m_inverseColor1);
                        
                        newItem->setVisible(true);
                        newItem->show();
                        newItem->attach(this);
                        newItem->m_flags = flags;
                        replot();
                        //                m_pData->m_pDrawItems.append(newItem);
                        m_pData->m_pDrawItems.insert(newItem->m_idx, newItem);
                    }
                }                
            }
        }

        replot();
    }

    if (dObj) delete dObj;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotCanvas::deleteMarkers(const QString &id)
{
    ito::RetVal retval;
    bool found = false;
    QMutableHashIterator<QString, QPair<int, QwtPlotMarker*> > i(m_plotMarkers);
    while (i.hasNext())
    {
        i.next();
        if (i.key() == id || id == "")
        {
            i.value().second->detach();
            delete i.value().second;
            found = true;
            i.remove();
        }
    }

    if (!found && id != "")
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("No marker with id '%1' found.").arg(id).toLatin1().data());
    }
    else
    {
        replot();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotCanvas::deleteMarkers(const int id)
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
        retval += ito::RetVal::format(ito::retError, 0, tr("No marker with id '%d' found.").toLatin1().data(), id);
    }
    else
    {
        replot();
    }
    
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotCanvas::userInteractionStart(int type, bool start, int maxNrOfPoints)
{
    ito::RetVal retval;

    m_drawedIemsIndexes.clear();
    m_pMultiPointPicker->selection().clear();

    if (type == tPoint || type == tMultiPointPick) //multiPointPick
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
            setState((PlotCanvas::tState)type);
        }
        else //start == false
        {

            m_pMultiPointPicker->setEnabled(false);

            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            if (m_pData)
            {
                m_pData->m_elementsToPick = 0;
            }

            Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
            if (p)
            {
                QPolygonF polygonScale;
                emit p->userInteractionDone(type, true, polygonScale);
            }
            setState(tIdle);
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

            Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
            if (p)
            {
                QPolygonF polygonScale;
                emit p->userInteractionDone(type, true, polygonScale);
            }
            setState(tIdle);
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

            Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
            if (p)
            {
                QPolygonF polygonScale;
                emit p->userInteractionDone(type, true, polygonScale);
            }
            setState(tIdle);
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

            Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
            if (p)
            {
                QPolygonF polygonScale;
                emit p->userInteractionDone(type, true, polygonScale);
            }
            setState(tIdle);
        }
    }
    else
    {
        m_pMultiPointPicker->setEnabled(false);
        Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
        if (p)
        {
            QPolygonF polygonScale;
            emit p->userInteractionDone(type, true, polygonScale);
        }
        setState(tIdle);
        retval += ito::RetVal(ito::retError, 0, tr("Unknown type for userInteractionStart").toLatin1().data());
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::multiPointActivated (bool on)
{
    if (m_pData)
    {
        switch(m_pData->m_state)
        {
            case tMultiPointPick:
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
                        emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size()), 2000);
                    }

                    if (m_pData->m_elementsToPick > 1)
                    {
                        m_pData->m_elementsToPick--;
                        MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());
                        if (m)
                        {
                            m->setMaxNrItems(2);
                            m_pMultiPointPicker->setEnabled(true);
                        }
                        return;
                    }
                    else
                    {
                        m_pData->m_elementsToPick = 0;
                    }

                    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
                    if (p)
                    {
                        if (polygonScale.size() > 0)
                        {
                            polygonScale.erase(polygonScale.end()-1); //remove last item, since the last one is always the current position of the mouse (without effective click)
                        }
                        emit p->userInteractionDone(ito::PrimitiveContainer::tMultiPointPick, aborted, polygonScale);
                    }

                    setState(tIdle);
                    m_pMultiPointPicker->setEnabled(false);
                }
            break;

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

                            newItem->setVisible(true);
                            newItem->show();
                            newItem->attach(this);
                            newItem->setSelected(true);
                            
                            m_pData->m_pDrawItems.insert(newItem->m_idx, newItem);
                        }
                        replot();
                    }

                    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
                    if (p)
                    {
                        emit p->userInteractionDone(ito::PrimitiveContainer::tPoint, aborted, polygonScale);
                        emit p->plotItemsFinished(ito::PrimitiveContainer::tPoint, aborted);
                    }

                    setState(tIdle);
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
                        Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
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
                        setState(tIdle);
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
                        Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
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
                        setState(tIdle);
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
                    
                        newItem->setVisible(true);
                        newItem->show();
                        newItem->attach(this);
                        newItem->setSelected(true);
                        replot();
                        m_pData->m_pDrawItems.insert(newItem->m_idx, newItem);
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
                        Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
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
                        setState(tIdle);
                        m_pMultiPointPicker->setEnabled(false);
                    }
                }
            break;
        }
    } // if m_pData
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mouseMoveEvent (QMouseEvent * event)
{
    if(m_ignoreNextMouseEvent)
    {
        m_ignoreNextMouseEvent = false;
        return;
    }

    //    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
    if (m_pData->m_state == tIdle)
    {
        ito::float32 canxpos = invTransform(QwtPlot::xBottom, event->x() - canvas()->x());
        ito::float32 canypos = invTransform(QwtPlot::yLeft, event->y() - canvas()->y());

        bool modificationDone = false;

        QHash<int, DrawItem*>::Iterator it = m_pData->m_pDrawItems.begin();
        for (; it != m_pData->m_pDrawItems.end(); it++)
        {   
            if (it.value() == NULL)
            {
                continue;
            }

            if(it.value()->m_active == 0)
            {
                continue;
            }

            ito::float32 dx, dy;
            QPainterPath path;

            switch(m_pData->m_modState)
            {
                default:
                    emit statusBarMessage(tr("Could not perform specific action on geomtric element, action not implemented."), 4000);
                    path = it.value()->shape();
                    break;
                case Itom2DQwt::tMoveGeometricElements:
                {

                    ito::float32 lenx = it.value()->x2 - it.value()->x1; 
                    ito::float32 leny = it.value()->y2 - it.value()->y1;

                    if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                    {
                        dx = invTransform(QwtPlot::xBottom, event->x()) - invTransform(QwtPlot::xBottom, m_initialMousePosition.x());
                        dy = invTransform(QwtPlot::yLeft, event->y()) - invTransform(QwtPlot::yLeft, m_initialMousePosition.y());
                        dx = fabs(dx) <= std::numeric_limits<ito::float32>::epsilon() ? std::numeric_limits<ito::float32>::epsilon() : dx;
                        dy = fabs(dy) <= std::numeric_limits<ito::float32>::epsilon() ? - std::numeric_limits<ito::float32>::epsilon() : dy;

                        if (it.value()->m_active == 1)
                        {
                            if (fabs(dx) > fabs(dy))
                            {
                                dx = canxpos - m_initialMarkerPosition.x();
                                dy = 0.0;
                            }
                            else
                            {
                                dy = canypos - m_initialMarkerPosition.y();
                                dx = 0.0;
                            }
                        
                        }
                        else
                        {
                            if (fabs(dx) > fabs(dy))
                            {
                                dx = canxpos - m_initialMarkerPosition.x() - lenx;
                                dy = 0.0;
                            }
                            else
                            {
                                dy = canypos - m_initialMarkerPosition.y() - leny;
                                dx = 0.0;
                            }
                        }


                    }
                    else
                    {
                        dx = 0.0;
                        dy = 0.0;
                        if (it.value()->m_active == 2)
                        {
                            canxpos -= lenx;
                            canypos -= leny;
                        }
                    }

                    switch (it.value()->m_type)
                    {
                        case tPoint:
                        {
                            if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                            {
                                path.moveTo(m_initialMarkerPosition.x() + dx, m_initialMarkerPosition.y() + dy);
                                path.lineTo(m_initialMarkerPosition.x() + dx, m_initialMarkerPosition.y() + dy); 
                            }
                            else
                            {
                                path.moveTo(canxpos, canypos);
                                path.lineTo(canxpos, canypos);                            
                            }
                        }
                        break;
                        case tLine:
                        {
                            if (QApplication::keyboardModifiers() == Qt::ControlModifier)      
                            {   
                                path.moveTo(m_initialMarkerPosition.x() + dx, m_initialMarkerPosition.y() + dy);
                                path.lineTo(m_initialMarkerPosition.x() + dx + lenx, m_initialMarkerPosition.y() + dy + leny);  
                            }
                            else
                            {
                                path.moveTo(canxpos, canypos);
                                path.lineTo(canxpos + lenx, canypos + leny);
                            }
                        }
                        break;

                        case tRect:
                        {
                            if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                            {
                                path.addRect(m_initialMarkerPosition.x() + dx, m_initialMarkerPosition.y() + dy, lenx, leny);                    
                            }
                            else
                            {
                                path.addRect(canxpos, canypos, lenx, leny);
                            }
                        }
                        break;

                        case tEllipse:
                        {
                            if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                            {
                                path.addEllipse(m_initialMarkerPosition.x() + dx, m_initialMarkerPosition.y() + dy, lenx, leny);
                            }
                            else
                            {
                                path.addEllipse(canxpos, canypos, lenx, leny);
                            }
                        }
                        break;
                    }
                    it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                    it.value()->setActive(it.value()->m_active);
                    replot();
                }

                break;
                case Itom2DQwt::tResizeGeometricElements:
                    emit statusBarMessage(tr("Could not perform specific action on geomtric element, resize action not implemented yet."), 4000);
                    break;
                case Itom2DQwt::tModifyPoints:
                {
                    if(it.value()->m_flags & 0x07)
                    {
                        emit statusBarMessage(tr("Could not change points of geomtric element, elemet is proteted."), 4000);
                        break;
                    }

                    if (it.value()->m_active == 1)
                    {

                        
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

                                if (QApplication::keyboardModifiers() == Qt::ControlModifier)       // draw line horizontal or vertical with second point fixed
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
                                else if(QApplication::keyboardModifiers() == Qt::ShiftModifier)    // move line without resize
                                {

                                    dx = it.value()->x2 - it.value()->x1;
                                    dy = it.value()->y2 - it.value()->y1;

                                    path.moveTo(canxpos, canypos);
                                    path.lineTo(canxpos + dx, canypos + dy);
                                }
                                else if(QApplication::keyboardModifiers() == Qt::AltModifier)      // keep linesize constant and second point fixed
                                {
                                    dx = it.value()->x2 - it.value()->x1;
                                    dy = it.value()->y2 - it.value()->y1;

                                    ito::float32 length = sqrt(dx * dx + dy * dy);

                                    dx = canxpos - it.value()->x2;
                                    dy = canypos - it.value()->y2;

                                    ito::float32 alpha = atan2(dy, dx);

                                    dx = cos(alpha) * length;
                                    dy = sin(alpha) * length;

                                    path.moveTo(it.value()->x2 + dx, it.value()->y2 + dy);
                                    path.lineTo(it.value()->x2, it.value()->y2); 
                                }
                                else                                                                // just draw line
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
                    }
                    else if (it.value()->m_active == 2)
                    {

                        if(it.value()->m_flags & 0x07)
                        {
                            emit statusBarMessage(tr("Could not change geomtric element, elemet is read only."), 4000);
                            break;
                        }

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
                                else if (QApplication::keyboardModifiers() == Qt::ShiftModifier)
                                {

                                    dx = it.value()->x2 - it.value()->x1;
                                    dy = it.value()->y2 - it.value()->y1;

                                    path.moveTo(canxpos - dx, canypos - dy);
                                    path.lineTo(canxpos , canypos );
                                }
                                else if (QApplication::keyboardModifiers() == Qt::AltModifier)
                                {
                                    dx = it.value()->x2 - it.value()->x1;
                                    dy = it.value()->y2 - it.value()->y1;

                                    ito::float32 length = sqrt(dx * dx + dy * dy);

                                    dx = it.value()->x1 - canxpos;
                                    dy = it.value()->y1 - canypos;

                                    ito::float32 alpha = atan2(dy, dx);

                                    dx = cos(alpha) * length;
                                    dy = sin(alpha) * length;

                                    path.moveTo(it.value()->x1, it.value()->y1);
                                    path.lineTo(it.value()->x1 - dx, it.value()->y1 - dy); 
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
                    }
                }
                break;
            }
            break;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mousePressEvent (QMouseEvent * event)
{
    if (m_pData->m_state == tIdle)
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
  //          double x = it.value()->x1;
//            double y = it.value()->y1;
            //double xx = transform(QwtPlot::xBottom, it.value()->x1);
            //double yy = transform(QwtPlot::yLeft, it.value()->y1);
            if (fabs(transform(QwtPlot::xBottom, it.value()->x1) - canxpos) < 10
                && fabs(transform(QwtPlot::yLeft, it.value()->y1) - canypos) < 10)
            {
                it.value()->m_active = 1;
                m_activeDrawItem = it.value()->m_idx;
                it.value()->setActive(1);
                it.value()->setSelected(true);
                
                m_initialMousePosition.setX( event->x() );
                m_initialMousePosition.setY( event->y() );
                m_initialMarkerPosition.setX( it.value()->x1 );
                m_initialMarkerPosition.setY( it.value()->y1 );
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
                m_initialMousePosition.setX( event->x() );
                m_initialMousePosition.setY( event->y() );
                m_initialMarkerPosition.setX( it.value()->x1 );
                m_initialMarkerPosition.setY( it.value()->y1 );
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
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mouseReleaseEvent (QMouseEvent * event)
{
    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
    if (m_pData->m_state == tEllipse || m_pData->m_state == tRect || m_pData->m_state == tLine
        || m_pData->m_state == tPoint || m_pData->m_state == tIdle)
    {
        QHash<int, DrawItem*>::iterator it = m_pData->m_pDrawItems.begin();
        for (;it != m_pData->m_pDrawItems.end(); ++it)        
//        for (int n = 0; n < m_pData->m_pDrawItems.size(); n++)
        {
            if (it.value() != NULL && it.value()->m_active > 0 != 0 && p)
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
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::configRescaler(void)
{
    m_pZoomer->setFixedAspectRatio(m_pData->m_keepAspect);
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> PlotCanvas::getDisplayed(void)
{
    if (!m_rasterData)
    {
        return QSharedPointer<ito::DataObject>(); 
    }

    return m_rasterData->rasterToObject(axisInterval(QwtPlot::xBottom), axisInterval(QwtPlot::yLeft));
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> PlotCanvas::getDisplayedOverlayObject()
{
    if (!m_rasterOverlayData)
    {
        return QSharedPointer<ito::DataObject>(); 
    }
    return m_rasterOverlayData->rasterToObject(axisInterval(QwtPlot::xBottom), axisInterval(QwtPlot::yLeft));
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> PlotCanvas::getOverlayObject()
{
    if (!m_rasterData)
    {
        return QSharedPointer<ito::DataObject>(); 
    }
    return m_rasterOverlayData->rasterToObject();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setOverlayObject(ito::DataObject* newOverlay)
{
    if(newOverlay)
    {
        m_dObjItem->setVisible(m_pData->m_alpha < 255);
        m_dOverlayItem->setAlpha(m_pData->m_alpha);
    }
    else
    {
        m_dObjItem->setVisible(true);
        m_dOverlayItem->setAlpha(0);
    }
    m_rasterOverlayData->updateDataObject(newOverlay);

    m_dOverlayItem->setVisible(m_rasterOverlayData->isInit() && m_pData->m_alpha > 0);
    if(m_pValuePicker) m_pValuePicker->enableOverlay(m_pData->m_alpha > 0);
    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
    p->enableOverlaySlider(m_rasterOverlayData->isInit());
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::alphaChanged()
{
    m_dOverlayItem->setVisible(m_pData->m_alpha > 0 && m_rasterOverlayData->isInit());
    m_dOverlayItem->setAlpha(m_pData->m_alpha);
    m_dObjItem->setVisible(m_pData->m_alpha < 255);
    if(m_pValuePicker) m_pValuePicker->enableOverlay(m_pData->m_alpha > 0);
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::contextMenuEvent(QContextMenuEvent * event)
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
void PlotCanvas::getMinMaxLoc(double &min, ito::uint32 *minLoc, double &max, ito::uint32 *maxLoc)
{
    if (!m_rasterData)
    {
        min = std::numeric_limits<double>::quiet_NaN();
        max = std::numeric_limits<double>::quiet_NaN();

        return;
    }   

    m_rasterData->getMinMaxLoc(min, minLoc, max, maxLoc);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::getMinMaxPhysLoc(double &min, double *minPhysLoc, double &max, double *maxPhysLoc)
{
    if (!m_rasterData || !m_dObjPtr || m_dObjPtr->getDims() < 2)
    {
        min = std::numeric_limits<double>::quiet_NaN();
        max = std::numeric_limits<double>::quiet_NaN();

        return;
    }   

    ito::uint32 minLoc[3];
    ito::uint32 maxLoc[3];
    m_rasterData->getMinMaxLoc(min, minLoc, max, maxLoc);

    bool check;

    minPhysLoc[0] = minLoc[0];
    minPhysLoc[1] = m_dObjPtr->getPixToPhys(m_dObjPtr->getDims()-2, minLoc[1], check); 
    minPhysLoc[2] = m_dObjPtr->getPixToPhys(m_dObjPtr->getDims()-1, minLoc[2], check); 

    maxPhysLoc[0] = maxLoc[0];
    maxPhysLoc[1] = m_dObjPtr->getPixToPhys(m_dObjPtr->getDims()-2, maxLoc[1], check); 
    maxPhysLoc[2] = m_dObjPtr->getPixToPhys(m_dObjPtr->getDims()-1, maxLoc[2], check); 

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::updateColors(void)
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
void PlotCanvas::setVisible(bool visible)
{    
    this->QwtPlot::setVisible(visible);
    if (visible)
    {
        if (!m_firstTimeVisible)
        {
            this->updateScaleValues(true, true);
        }
        else
        {
            this->updateScaleValues(true, false);
        }
        m_firstTimeVisible = true;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::updateLabelVisibility()
{    
    if(!m_pData) return;

    if(m_pData->m_markerLabelVisible)
    {
        QPair<int, QwtPlotMarker*> pair;
        QList<QString> keys = m_plotMarkers.keys();
        QString label;
        foreach(label, keys)
        {
            foreach(pair, m_plotMarkers.values(label))
            {
                if(pair.second) pair.second->setLabel(label);
            }
        }
    }
    else
    {
        QPair<int, QwtPlotMarker*> pair;

        foreach(pair, m_plotMarkers)
        {
            if(pair.second) pair.second->setLabel(QString(""));
        }
    }
}

////----------------------------------------------------------------------------------------------------------------------------------
//void PlotCanvas::multiPointSelected (const QPolygon &polygon)
//{
//    qDebug() << "pointSelected:" << polygon;
//};
//
//
////----------------------------------------------------------------------------------------------------------------------------------
//void PlotCanvas::multiPointAppended (const QPoint &pos)
//{
//    qDebug() << "pointAppended:" << pos;
//}
//----------------------------------------------------------------------------------------------------------------------------------
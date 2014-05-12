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
#include <qwt_plot_magnifier.h>
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
PlotCanvas::PlotCanvas(InternalData *m_pData, QWidget * parent /*= NULL*/) :
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
        m_curColorMapIndex(0),
        m_pValuePicker(NULL),
        m_dObjPtr(NULL),
        m_pStackPicker(NULL),
        m_zstackCutUID(0),
        m_lineCutUID(0),
        m_pLineCutLine(NULL),
        m_pMultiPointPicker(NULL),
        m_pRescaler(NULL),
        m_activeDrawItem(-1),
        m_ignoreNextMouseEvent(false)
        
{

    setMouseTracking(false);

    //this is the border between the canvas and the axes and the overall mainwindow
    setContentsMargins(5,5,5,5);

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
    m_rasterOverlayData = new DataObjRasterData(m_pData, false);
    m_dOverlayItem->setData(m_rasterOverlayData);
    m_dOverlayItem->attach(this);
    m_dOverlayItem->setAlpha(m_pData->m_alpha);
    m_dOverlayItem->setColorMap(new QwtLinearColorMap(Qt::black, Qt::white, QwtColorMap::Indexed));
    m_dOverlayItem->setVisible(false);
    //zoom tool
    m_pZoomer = new QwtPlotZoomer(QwtPlot::xBottom, QwtPlot::yLeft, canvas());
    m_pZoomer->setEnabled(false);
    m_pZoomer->setTrackerMode(QwtPicker::AlwaysOn);

    //pan tool
    m_pPanner = new QwtPlotPanner(canvas());
    m_pPanner->setAxisEnabled(QwtPlot::yRight,false); //do not consider the right vertical axis
    m_pPanner->setCursor(Qt::SizeAllCursor);
    m_pPanner->setEnabled(false);

    m_pMagnifier = new QwtPlotMagnifier(canvas());
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
    m_pStackPicker = new QwtPlotPicker(canvas());
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
    m_pStackCutMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green),3),  QSize(7,7)));
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

    //prepare color bar
    QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
    rightAxis->setColorBarEnabled(true);
    rightAxis->setColorBarWidth(15);

    //rightAxis->setColorMap(QwtInterval(0,1.0), new QwtLinearColorMap(QColor::fromRgb(0,0,0), QColor::fromRgb(255,255,255), QwtColorMap::Indexed));
    rightAxis->setFont(QFont("Verdana", 8, 1, true));

    rightAxis->setMargin(20); //margin to right border of window
    rightAxis->scaleDraw()->setLength(20);
    rightAxis->scaleDraw()->enableComponent(QwtAbstractScaleDraw::Backbone,false);

    setAxisScale(QwtPlot::yRight, 0, 1.0);
    enableAxis(QwtPlot::yRight, m_pData->m_colorBarVisible);
    axisWidget(QwtPlot::yRight)->setLayoutFlag(QwtScaleWidget::TitleInverted, false); //let the label be in the same direction than on the left side

    
    //m_pRescaler = new QwtPlotRescaler(canvas(), QwtPlot::xBottom, QwtPlotRescaler::Fixed);
    configRescaler();
    //m_pRescaler->setEnabled(true);

    m_drawedIemsIndexes.clear();
    m_drawedIemsIndexes.reserve(10);
}

//----------------------------------------------------------------------------------------------------------------------------------
PlotCanvas::~PlotCanvas()
{

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
    
    if (m_pRescaler != NULL)
    {
        m_pRescaler->deleteLater();
        m_pRescaler = NULL;
    }
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
    QPen rubberBandPen = apiGetFigureSetting(parent(), "zoomRubberBandPen", QPen(QBrush(Qt::red), 1, Qt::DashLine),NULL).value<QPen>();
    QPen trackerPen = apiGetFigureSetting(parent(), "trackerPen", QPen(QBrush(Qt::red), 2), NULL).value<QPen>();
    QFont trackerFont = apiGetFigureSetting(parent(), "trackerFont", QFont("Verdana", 10), NULL).value<QFont>();
    QBrush trackerBg = apiGetFigureSetting(parent(), "trackerBackground", QBrush(QColor(255, 255, 255, 155), Qt::SolidPattern), NULL).value<QBrush>();
    QPen selectionPen = apiGetFigureSetting(parent(), "selectionPen", QPen(QBrush(Qt::gray), 2, Qt::SolidLine), NULL).value<QPen>();

    QFont titleFont = apiGetFigureSetting(parent(), "titleFont", QFont("Helvetica", 12), NULL).value<QFont>();
    QFont labelFont = apiGetFigureSetting(parent(), "labelFont", QFont("Helvetica", 12), NULL).value<QFont>();
    labelFont.setItalic(false);
    QFont axisFont = apiGetFigureSetting(parent(), "axisFont", QFont("Helvetica", 10), NULL).value<QFont>();

    if (m_inverseColor1.isValid())
    {
        rubberBandPen.setColor(m_inverseColor1);
    }

    if (m_inverseColor0.isValid())
    {
        selectionPen.setColor(m_inverseColor0);
        trackerPen.setColor(m_inverseColor0);
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

    m_pStackCutMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(m_inverseColor1), QPen(QBrush(m_inverseColor1),3),  QSize(7,7)));
    m_pCenterMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(m_inverseColor0), QPen(QBrush(m_inverseColor0),1),  QSize(11,11)));
    
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
    bool needToUpdate = false;

    m_dObjPtr = dObj;

    //QString valueLabel, axisLabel, title;

    if (dObj)
    {
        int dims = dObj->getDims();
        int width = dims > 0 ? dObj->getSize(dims - 1) : 0;
        int height = dims > 1 ? dObj->getSize(dims - 2) : 1;

        needToUpdate = m_rasterData->updateDataObject(dObj, plane);

        if (needToUpdate)
        {
            bool valid;
            ito::DataObjectTagType tag;
            std::string descr, unit;
            tag = dObj->getTag("title", valid);
            m_pData->m_titleDObj = valid ? tag.getVal_ToString().data() : "";
            m_pData->m_dataType = (ito::tDataType)dObj->getType();

            descr = dObj->getValueDescription();
            unit = dObj->getValueUnit();

            if (unit != "")
            {
                descr.append(" [" + unit + "]");
            }
            m_pData->m_valueLabelDObj = QString::fromStdString(descr);

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
                m_pData->m_xaxisLabelDObj = QString::fromStdString(descr);

                descr = dObj->getAxisDescription(dims-2, valid);
                if (!valid) descr = "";
                unit = dObj->getAxisUnit(dims-2,valid);
                if (!valid) unit = "";

                if (unit != "")
                {
                    descr.append(" [" + unit + "]");
                }
                m_pData->m_yaxisLabelDObj = QString::fromStdString(descr);
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

    if (needToUpdate)
    {
        Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
        if (p)
        {
            p->setColorDataTypeRepresentation(dObj->getType() == ito::tRGBA32);

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

        //updateMarkerPosition(true);

        updateScaleValues(); //replot is done here

        if (m_pRescaler != NULL)
        {

            QwtInterval curXInterVal = m_rasterData->interval(Qt::XAxis);
            QwtInterval curYInterVal = m_rasterData->interval(Qt::YAxis);

            m_pRescaler->setIntervalHint(QwtPlot::xBottom, curXInterVal);
            m_pRescaler->setIntervalHint(QwtPlot::yLeft, curYInterVal);

            if (m_pData->m_keepAspect)m_pRescaler->rescale();
        }

        m_pZoomer->setZoomBase(true);
    }
    else
    {
        //updateMarkerPosition(true,false);

        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::changePlane(int plane)
{
    refreshPlot(m_dObjPtr, plane);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::internalDataUpdated()
{
    refreshPlot(m_dObjPtr, -1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::contextMenuEvent(QContextMenuEvent * event)
{
    /*if (m_showContextMenu)
    {
        event->accept();
        m_contextMenu->exec(event->globalPos());
    }
    else
    {
        event->ignore();
    }*/
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotCanvas::setColorMap(QString colormap /*= "__next__"*/)
{
    QwtLinearColorMap *colorMap = NULL;
    QwtLinearColorMap *colorBarMap = NULL;
    ito::ItomPalette newPalette;
    ito::RetVal retval(ito::retOk);
    int numPalettes = 1;

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
void PlotCanvas::keyPressEvent (QKeyEvent * event)
{
    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
    m_activeModifiers = event->modifiers();

    QPointF incr;
    incr.setX(invTransform(QwtPlot::xBottom, 1) - invTransform(QwtPlot::xBottom, 0));
    incr.setY(invTransform(QwtPlot::yLeft, 1) - invTransform(QwtPlot::yLeft, 0));

    if (m_pData->m_state == tStackCut)
    {
        QPointF markerPosScaleCoords = m_pStackCutMarker->value();

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
        }

        if (m_rasterData->pointValid(markerPosScaleCoords))
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

        QwtInterval hInterval = m_rasterData->interval(Qt::XAxis);
        QwtInterval vInterval = m_rasterData->interval(Qt::YAxis);

        if (event->key() == Qt::Key_H) //draw horizontal line in the middle of the plotted dataObject
        {
            pts.append(QPointF(hInterval.minValue(), (vInterval.minValue() + vInterval.maxValue())*0.5));
            pts.append(QPointF(hInterval.maxValue(), (vInterval.minValue() + vInterval.maxValue())*0.5));

            if (p)
            {
                p->setCoordinates(pts, true);
            }
        }
        else if (event->key() == Qt::Key_V) // draw vertical line in the middle of the plotted dataObject
        {
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
            }

            if (p)
            {
                p->setCoordinates(pts, true);
            }
        }

        if (m_rasterData->pointValid(pts[0]) && m_rasterData->pointValid(pts[1]))
        {
            m_pLineCutLine->setSamples(pts);

            (p)->displayCut(pts, m_lineCutUID, false);
            replot();
        }
    }
    else if(event->matches(QKeySequence::Copy))
    {
        p->copyToClipBoard();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::keyReleaseEvent (QKeyEvent * event)
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
void PlotCanvas::updateScaleValues()
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

    setAxisScale(QwtPlot::yRight, m_pData->m_valueMin, m_pData->m_valueMax);
    QwtScaleWidget *widget = axisWidget(QwtPlot::yRight);
    if (widget)
    {
        QwtInterval ival(m_pData->m_valueMin, m_pData->m_valueMax);
        axisWidget(QwtPlot::yRight)->setColorMap(ival, const_cast<QwtColorMap*>(widget->colorMap())); //the color map should be unchanged
    }

    setAxisScale(QwtPlot::xBottom, m_pData->m_xaxisMin, m_pData->m_xaxisMax);

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

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setInterval(Qt::Axis axis, const QPointF &interval)
{
    if (axis == Qt::XAxis)
    {
        m_pData->m_xaxisScaleAuto = false;
        m_pData->m_xaxisMin = interval.x();
        m_pData->m_xaxisMax = interval.y();
        setAxisScale(QwtPlot::xBottom, m_pData->m_xaxisMin, m_pData->m_xaxisMax);
    }
    else if (axis == Qt::YAxis)
    {
        m_pData->m_yaxisScaleAuto = false;

        QwtScaleEngine *scaleEngine = axisScaleEngine(QwtPlot::yLeft);
        m_pData->m_yaxisMin = interval.x();
        m_pData->m_yaxisMax = interval.y();

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
        m_pData->m_valueScaleAuto = false;
        m_pData->m_valueMin = interval.x();
        m_pData->m_valueMax = interval.y();

        QwtScaleWidget *widget = axisWidget(QwtPlot::yRight);

        if (widget)
        {
            QwtInterval ival(interval.x(), interval.y());
            widget->setColorMap(ival, const_cast<QwtColorMap*>(widget->colorMap())); //the color map should be unchanged
        }

        setAxisScale(QwtPlot::yRight, m_pData->m_valueMin, m_pData->m_valueMax);

        m_rasterData->setInterval(Qt::ZAxis, QwtInterval(m_pData->m_valueMin, m_pData->m_valueMax));
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF PlotCanvas::getInterval(Qt::Axis axis) const
{
    QwtInterval i = m_rasterData->interval(axis);
    return QPointF(i.minValue(), i.maxValue());
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setOverlayInterval(Qt::Axis axis, const QPointF &interval)
{
    if (axis == Qt::ZAxis)
    {
        m_pData->m_overlayScaleAuto = false;
        m_pData->m_overlayMin = interval.x();
        m_pData->m_overlayMax = interval.y();

        m_rasterOverlayData->setInterval(Qt::ZAxis, QwtInterval(m_pData->m_overlayMin, m_pData->m_overlayMax));
    }

    replot();
}
//----------------------------------------------------------------------------------------------------------------------------------
QPointF PlotCanvas::getOverlayInterval(Qt::Axis axis) const
{
    QwtInterval i = m_rasterOverlayData->interval(axis);
    return QPointF(i.minValue(), i.maxValue());
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
void PlotCanvas::zStackCutTrackerAppended(const QPoint &pt)
{
    QVector<QPointF> pts;

    if (m_pData->m_state == tStackCut)
    {
        QPointF ptScale;
        ptScale.setY(invTransform(QwtPlot::yLeft, pt.y()));
        ptScale.setX(invTransform(QwtPlot::xBottom, pt.x()));

        m_pStackCutMarker->setValue(ptScale);
        m_pStackCutMarker->setVisible(true);

        pts.append(ptScale);
        ((Itom2dQwtPlot*)parent())->setCoordinates(pts, true);
        ((Itom2dQwtPlot*)parent())->displayCut(pts, m_zstackCutUID,true);
    }

    replot();
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
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::lineCutMoved(const QPoint &pt)
{
    QVector<QPointF> pts;
    pts.resize(2);

    if (m_pData->m_state == tLineCut)
    {
        pts[0] = m_pLineCutLine->sample(0);
        pts[1].setY(invTransform(QwtPlot::yLeft, pt.y()));
        pts[1].setX(invTransform(QwtPlot::xBottom, pt.x()));

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
        ((Itom2dQwtPlot*)parent())->displayCut(pts, m_lineCutUID, false);
    }

    replot();
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

        pts[1] = pts[0];

        m_pLineCutLine->setVisible(true);
        m_pLineCutLine->setSamples(pts);

        ((Itom2dQwtPlot*)parent())->setCoordinates(pts, true);
        ((Itom2dQwtPlot*)parent())->displayCut(pts, m_lineCutUID, false);
    }

    replot();
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
					QPolygon polygon = m_pMultiPointPicker->selection();

					QPolygonF polygonScale;
					bool aborted = false;

					if (polygon.size() == 0)
					{
						emit statusBarMessage(tr("Selection has been aborted."), 2000);
						aborted = true;
					}
					else
					{
						QPointF pt;

						for (int i = 0; i < polygon.size() - 1; ++i)
						{
							pt.rx() = invTransform(QwtPlot::xBottom, polygon[i].rx());
							pt.ry() = invTransform(QwtPlot::yLeft, polygon[i].ry());
							polygonScale.append(pt);
						}

						emit statusBarMessage(tr("%1 points have been selected.").arg(polygon.size()-1), 2000);
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
						emit p->userInteractionDone(ito::PrimitiveContainer::tMultiPointPick, aborted, polygonScale);
					}

					setState(tIdle);
					m_pMultiPointPicker->setEnabled(false);
				}
			break;

			case tPoint:
				if (!on)
				{
					QPolygon polygon = m_pMultiPointPicker->selection();

					QPolygonF polygonScale;
					bool aborted = false;

					if (polygon.size() == 0)
					{
						emit statusBarMessage(tr("Selection has been aborted."), 2000);
						aborted = true;
					}
					else
					{
						QPointF pt;

						for (int i = 0; i < polygon.size() - 1; ++i)
						{
							pt.rx() = invTransform(QwtPlot::xBottom, polygon[i].rx());
							pt.ry() = invTransform(QwtPlot::yLeft, polygon[i].ry());
							polygonScale.append(pt);
						}

						emit statusBarMessage(tr("%1 points have been selected.").arg(polygon.size()-1), 2000);
					}

					if (!aborted && polygonScale.size() > 0)
					{
						for (int i = 0; i < polygonScale.size(); i++)
						{
							QPainterPath *path = new QPainterPath();
							DrawItem *newItem = NULL;
							newItem = new DrawItem(this, tPoint);
							path->moveTo(polygonScale[i].x(), polygonScale[i].y());
							path->lineTo(polygonScale[i].x(), polygonScale[i].y());

							newItem->setShape(*path, m_inverseColor0, m_inverseColor1);

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
					QPolygon polygon = m_pMultiPointPicker->selection();

					QPolygonF polygonScale;
					bool aborted = false;

					if (polygon.size() == 0)
					{
						emit statusBarMessage(tr("Selection has been aborted."), 2000);
						aborted = true;
						m_drawedIemsIndexes.clear();
					}
					else
					{
						QPointF pt;

						for (int i = 0; i < polygon.size(); ++i)
						{
							pt.rx() = invTransform(QwtPlot::xBottom, polygon[i].rx());
							pt.ry() = invTransform(QwtPlot::yLeft, polygon[i].ry());
							polygonScale.append(pt);
						}

						emit statusBarMessage(tr("%1 points have been selected.").arg(polygon.size()-1), 2000);

						QPainterPath *path = new QPainterPath();
						DrawItem *newItem = NULL;
						newItem = new DrawItem(this, tLine);
						path->moveTo(polygonScale[0].x(), polygonScale[0].y());
						path->lineTo(polygonScale[1].x(), polygonScale[1].y());

						newItem->setShape(*path, m_inverseColor0, m_inverseColor1);


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
					QPolygon polygon = m_pMultiPointPicker->selection();

					QPolygonF polygonScale;
					bool aborted = false;

					if (polygon.size() == 0)
					{
						emit statusBarMessage(tr("Selection has been aborted."), 2000);
						aborted = true;
						m_drawedIemsIndexes.clear();
					}
					else
					{
						QPointF pt;

						for (int i = 0; i < polygon.size(); ++i)
						{
							pt.rx() = invTransform(QwtPlot::xBottom, polygon[i].rx());
							pt.ry() = invTransform(QwtPlot::yLeft, polygon[i].ry());
							polygonScale.append(pt);
						}

						emit statusBarMessage(tr("%1 points have been selected.").arg(polygon.size()-1), 2000);

						QPainterPath *path = new QPainterPath();
						DrawItem *newItem = NULL;
						newItem = new DrawItem(this, tRect);
						path->addRect(polygonScale[0].x(), polygonScale[0].y(), polygonScale[1].x() - polygonScale[0].x(),
									  polygonScale[1].y() - polygonScale[0].y());

						newItem->setShape(*path, m_inverseColor0, m_inverseColor1);

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
					QPolygon polygon = m_pMultiPointPicker->selection();

					QPolygonF polygonScale;
					bool aborted = false;

					if (polygon.size() == 0)
					{
						emit statusBarMessage(tr("Selection has been aborted."), 2000);
						aborted = true;
						m_drawedIemsIndexes.clear();
					}
					else
					{
						QPointF pt;

						for (int i = 0; i < polygon.size(); ++i)
						{
							pt.rx() = invTransform(QwtPlot::xBottom, polygon[i].rx());
							pt.ry() = invTransform(QwtPlot::yLeft, polygon[i].ry());
							polygonScale.append(pt);
						}

						emit statusBarMessage(tr("%1 points have been selected.").arg(polygon.size()-1), 2000);

						QPainterPath *path = new QPainterPath();
						DrawItem *newItem = NULL;
						newItem = new DrawItem(this, tEllipse);
						path->addEllipse(polygonScale[0].x(), polygonScale[0].y(),
								(polygonScale[1].x() - polygonScale[0].x()), (polygonScale[1].y() - polygonScale[0].y()));

						newItem->setShape(*path, m_inverseColor0, m_inverseColor1);
					
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

        QHash<int, DrawItem*>::Iterator it = m_pData->m_pDrawItems.begin();
        for (; it != m_pData->m_pDrawItems.end(); it++)
        {
            if (it.value() == NULL)
            {
                continue;
            }

            if (it.value()->m_active == 1)
            {
                ito::float32 dx, dy;

                QPainterPath *path = new QPainterPath();
                switch (it.value()->m_type)
                {
                    case tPoint:

                        path->moveTo(canxpos, canypos);
                        path->lineTo(canxpos, canypos);                                              

                        it.value()->setShape(*path, m_inverseColor0, m_inverseColor1);
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
                                path->moveTo(canxpos, it.value()->y2);
                                path->lineTo(it.value()->x2, it.value()->y2);  
                            }
                            else
                            {
                                path->moveTo(it.value()->x2, canypos);
                                path->lineTo(it.value()->x2, it.value()->y2);  
                            }
                        }
                        else
                        {
                            path->moveTo(canxpos, canypos);
                            path->lineTo(it.value()->x2, it.value()->y2);                        
                        }
                        it.value()->setShape(*path, m_inverseColor0, m_inverseColor1);
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

                        path->addRect(canxpos, canypos,
                            it.value()->x2 - canxpos,
                            it.value()->y2 - canypos);
                        it.value()->setShape(*path, m_inverseColor0, m_inverseColor1);
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
                        path->addEllipse(canxpos,
                            canypos,
                             it.value()->x2 - canxpos,
                             it.value()->y2 - canypos);
                        it.value()->setShape(*path, m_inverseColor0, m_inverseColor1);
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

                QPainterPath *path = new QPainterPath();
                switch (it.value()->m_type)
                {
                    case tLine:
                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                        {
                            dx = canxpos - it.value()->x1;
                            dy = it.value()->y1 - canypos;

                            if (fabs(dx) > fabs(dy))
                            {
                                path->moveTo(it.value()->x1, it.value()->y1);
                                path->lineTo(canxpos, it.value()->y1);
                            }
                            else
                            {
                                path->moveTo(it.value()->x1, it.value()->y1);
                                path->lineTo(it.value()->x1, canypos);
                            }
                        }
                        else
                        {
                            path->moveTo(it.value()->x1, it.value()->y1);
                            path->lineTo(canxpos, canypos);
                        }
                        
                        it.value()->setShape(*path, m_inverseColor0, m_inverseColor1);
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
                        path->addRect(it.value()->x1, it.value()->y1,
                            canxpos - it.value()->x1,
                            canypos - it.value()->y1);
                        it.value()->setShape(*path, m_inverseColor0, m_inverseColor1);
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
                        path->addEllipse(it.value()->x1,
                            it.value()->y1,
                            canxpos - it.value()->x1,
                            canypos - it.value()->y1),
                        it.value()->setShape(*path, m_inverseColor0, m_inverseColor1);
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
    if (m_pData->m_keepAspect)
    {
        int refAxis = plotLayout()->canvasRect().width() < plotLayout()->canvasRect().height() ? QwtPlot::xBottom : QwtPlot::yLeft;
    
        /*
        QwtInterval curXInterVal = axisInterval(QwtPlot::xBottom);
        QwtInterval curYInterVal = axisInterval(QwtPlot::yLeft);

        QwtInterval userXInterVal = m_rasterData->interval(Qt::XAxis);
        QwtInterval userYInterVal = m_rasterData->interval(Qt::YAxis);

        if (curXInterVal.minValue() < userXInterVal.minValue())
        {
            curXInterVal.setMinValue(userXInterVal.minValue());
        }

        if (curXInterVal.maxValue() > userXInterVal.maxValue())
        {
            curXInterVal.setMaxValue(userXInterVal.maxValue());
        }

        if (curYInterVal.minValue() < userYInterVal.minValue())
        {
            curYInterVal.setMinValue(userYInterVal.minValue());
        }

        if (curYInterVal.maxValue() > userYInterVal.maxValue())
        {
            curYInterVal.setMaxValue(userYInterVal.maxValue());
        }
        */
        if (m_pRescaler == NULL)
        {
            QwtInterval curXInterVal = m_rasterData->interval(Qt::XAxis);
            QwtInterval curYInterVal = m_rasterData->interval(Qt::YAxis);

            m_pRescaler = new QwtPlotRescaler(canvas(), refAxis , QwtPlotRescaler::Fitting);
            m_pRescaler->setEnabled(false);
            m_pRescaler->setIntervalHint(QwtPlot::xBottom, curXInterVal);
            m_pRescaler->setIntervalHint(QwtPlot::yLeft, curYInterVal);
            m_pRescaler->setIntervalHint(QwtPlot::yRight, m_rasterData->interval(Qt::ZAxis));
            m_pRescaler->setAspectRatio(1.0);
            m_pRescaler->setAspectRatio(QwtPlot::yRight, 0.0);
            m_pRescaler->setExpandingDirection(QwtPlot::xBottom, QwtPlotRescaler::ExpandUp);
            m_pRescaler->setExpandingDirection(QwtPlot::yLeft, QwtPlotRescaler::ExpandBoth);
            m_pRescaler->setExpandingDirection(QwtPlot::yRight, QwtPlotRescaler::ExpandBoth);
        }
        else
        {
            m_pRescaler->setReferenceAxis(refAxis);
        }

        //m_pRescaler->setIntervalHint(QwtPlot::xBottom, curXInterVal);
        //m_pRescaler->setIntervalHint(QwtPlot::yLeft, curYInterVal);

        m_pRescaler->setEnabled(true);
    }
    else
    {
        if (m_pRescaler != NULL)
        {
            m_pRescaler->setEnabled(false);
            //m_pRescaler->deleteLater();
            //m_pRescaler = NULL;
        }
    }
    if (m_pRescaler != NULL && m_pData->m_keepAspect)
    {
        m_pRescaler->rescale();
    }
    //replot();
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


////----------------------------------------------------------------------------------------------------------------------------------
//void PlotCanvas::multiPointSelected (const QPolygon &polygon)
//{
//    qDebug() << "pointSelected:" << polygon;
//};
//
////----------------------------------------------------------------------------------------------------------------------------------
//void PlotCanvas::multiPointAppended (const QPoint &pos)
//{
//    qDebug() << "pointAppended:" << pos;
//};

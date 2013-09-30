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
#include "multiPointPickerMachine.h"
#include "userInteractionPlotPicker.h"

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
        m_pStackCutMarker(NULL),
		m_dObjItem(NULL),
        m_rasterData(NULL),
		m_pData(m_pData),
        m_curColorMapIndex(0),
        m_pValuePicker(NULL),
        m_dObjPtr(NULL),
		m_pStackPicker(NULL),
        m_zstackCutUID(0),
        m_lineCutUID(0),
        m_pLineCutLine(NULL),
        m_pMultiPointPicker(NULL)
{
    setMouseTracking(false);

	//this is the border between the canvas and the axes and the overall mainwindow
	setContentsMargins(5,5,5,5);
	
	//canvas() is the real plotting area, where the plot is printed (without axes...)
	//canvas()->setFrameShadow(QFrame::Plain);
	//canvas()->setFrameShape(QFrame::NoFrame);

	canvas()->setStyleSheet("border: 0px;");
	canvas()->setCursor( Qt::ArrowCursor );
	
	//main item on canvas -> the data object
    m_dObjItem = new DataObjItem("Data Object");
    m_dObjItem->setRenderThreadCount(0);
    //m_dObjItem->setColorMap( new QwtLinearColorMap(QColor::fromRgb(0,0,0), QColor::fromRgb(255,255,255), QwtColorMap::Indexed));

    m_rasterData = new DataObjRasterData(m_pData);
    m_dObjItem->setData(m_rasterData);
	m_dObjItem->attach(this);

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
    m_pMagnifier->setWheelModifiers( Qt::ControlModifier );
    m_pMagnifier->setAxisEnabled(QwtPlot::yLeft,true);
    m_pMagnifier->setAxisEnabled(QwtPlot::xBottom,true);
    m_pMagnifier->setAxisEnabled(QwtPlot::yRight,false); //do not consider the right vertical axis (color bar)

    //value picker
    m_pValuePicker = new ValuePicker2D(QwtPlot::xBottom, QwtPlot::yLeft, canvas(), m_rasterData);
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
    m_pStackCutMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green),3),  QSize(7,7) ));
    m_pStackCutMarker->attach(this);
    m_pStackCutMarker->setVisible(false);

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
    m_pMultiPointPicker->setRubberBand( QwtPicker::UserRubberBand ); //user is cross here
    //m_pMultiPointPicker->setStateMachine(new QwtPickerClickPointMachine); 
    m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine);
    m_pMultiPointPicker->setRubberBandPen( QPen(QBrush(Qt::green, Qt::SolidPattern),2) );
    connect(m_pMultiPointPicker, SIGNAL(activated(bool)), this, SLOT(multiPointActivated(bool)));
    //connect(m_pMultiPointPicker, SIGNAL(selected(QPolygon)), this, SLOT(multiPointSelected (QPolygon) ));
    //connect(m_pMultiPointPicker, SIGNAL(appended(QPoint)), this, SLOT(multiPointAppended (QPoint) ));

	//prepare color bar
	QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
    rightAxis->setColorBarEnabled(true);
    rightAxis->setColorBarWidth(30);

    //rightAxis->setColorMap(QwtInterval(0,1.0), new QwtLinearColorMap(QColor::fromRgb(0,0,0), QColor::fromRgb(255,255,255), QwtColorMap::Indexed));
    rightAxis->setFont(QFont("Verdana",8,1,true));

    rightAxis->setMargin(20); //margin to right border of window
    rightAxis->scaleDraw()->setLength(20);
    rightAxis->scaleDraw()->enableComponent(QwtAbstractScaleDraw::Backbone,false);

    setAxisScale(QwtPlot::yRight, 0, 1.0 );
    enableAxis(QwtPlot::yRight, m_pData->m_colorBarVisible );
    axisWidget(QwtPlot::yRight)->setLayoutFlag( QwtScaleWidget::TitleInverted, false ); //let the label be in the same direction than on the left side
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
    QPen rubberBandPen = apiGetFigureSetting(parent(), "zoomRubberBandPen", QPen(QBrush(Qt::red),1,Qt::DashLine),NULL).value<QPen>();
    QPen trackerPen = apiGetFigureSetting(parent(), "trackerPen", QPen(QBrush(Qt::red),2),NULL).value<QPen>();
    QFont trackerFont = apiGetFigureSetting(parent(), "trackerFont", QFont("Verdana",10),NULL).value<QFont>();
    QBrush trackerBg = apiGetFigureSetting(parent(), "trackerBackground", QBrush(QColor(255,255,255,155), Qt::SolidPattern),NULL).value<QBrush>();
    QPen selectionPen = apiGetFigureSetting(parent(), "selectionPen", QPen(QBrush(Qt::gray),2,Qt::SolidLine),NULL).value<QPen>();

    QFont titleFont = apiGetFigureSetting(parent(), "titleFont", QFont("Helvetica",12),NULL).value<QFont>();
    QFont labelFont = apiGetFigureSetting(parent(), "labelFont", QFont("Helvetica",12),NULL).value<QFont>();
    labelFont.setItalic(false);
    QFont axisFont = apiGetFigureSetting(parent(), "axisFont", QFont("Helvetica",10),NULL).value<QFont>();

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

        needToUpdate = m_rasterData->updateDataObject( dObj, plane);

        if (needToUpdate)
        {
            bool valid;
            ito::DataObjectTagType tag;
            std::string descr, unit;
            tag = dObj->getTag("title", valid);
            m_pData->m_titleDObj = valid? QString::fromStdString(tag.getVal_ToString()) : "";
		    m_pData->m_dataType = (ito::tDataType)dObj->getType();

            descr = dObj->getValueDescription();
            unit = dObj->getValueUnit();

            if (unit != "")
            {
                descr.append( " [" + unit + "]");
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
                    descr.append( " [" + unit + "]");
                }
                m_pData->m_xaxisLabelDObj = QString::fromStdString(descr);

                descr = dObj->getAxisDescription(dims-2, valid);
                if (!valid) descr = "";
                unit = dObj->getAxisUnit(dims-2,valid);
                if (!valid) unit = "";

                if (unit != "")
                {
                    descr.append( " [" + unit + "]");
                }
                m_pData->m_yaxisLabelDObj = QString::fromStdString(descr);
            }
            else
            {
                m_pData->m_xaxisLabelDObj = "";
                m_pData->m_yaxisLabelDObj = "";
            }
        }
    } 

    updateLabels();

    if (needToUpdate)
    {
        Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
        if (p)
        {
            if(dObj->getType() == ito::tComplex128 || dObj->getType() == ito::tComplex64)
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

        m_pZoomer->setZoomBase( true );
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
    /*if(m_showContextMenu)
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
        emit statusBarMessage( tr("No color maps defined."), 4000 );
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
        emit statusBarMessage( QString("%1").arg( retval.errorMessage() ), 4000 );
        return false;
    }
    else if (retval.containsError())
    {
        emit statusBarMessage( "error when loading color map", 4000 );
        return false;
    }

    int totalStops = newPalette.colorStops.size();

    if (totalStops < 2)
    {
        emit statusBarMessage( tr("Selected color map has less than two points."), 4000 );
        return false;
    }

    m_colorMapName = newPalette.name;

/*
    if(newPalette.getPos(newPalette.getSize() - 1) == newPalette.getPos(newPalette.getSize() - 2))  // BuxFix - For Gray-Marked
    {
        colorMap = new QwtLinearColorMap(newPalette.getColorFirst(), newPalette.getColor(newPalette.getSize() - 2), QwtColorMap::Indexed);
        colorBarMap = new QwtLinearColorMap(newPalette.getColorFirst(), newPalette.getColor(newPalette.getSize() - 2), QwtColorMap::Indexed);   
        if(newPalette.getSize() > 2)
        {
            for(int i = 1; i < newPalette.getSize() - 2; i++)
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
        if(newPalette.getSize() > 2)
        {
            for(int i = 1; i < newPalette.getSize() - 1; i++)
            {
                colorMap->addColorStop(newPalette.getPos(i), newPalette.getColor(i));
                colorBarMap->addColorStop(newPalette.getPos(i), newPalette.getColor(i));
            }
        }
    }
*/

    //if(newPalette.inverseColorOne.isValid() /* && newPalette.inverseColorTwo.isValid() */)
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


    if(newPalette.colorStops[totalStops - 1].first == newPalette.colorStops[totalStops - 2].first )  // BuxFix - For Gray-Marked
    {
        colorMap    = new QwtLinearColorMap(newPalette.colorStops[0].second, newPalette.colorStops[totalStops - 2].second, QwtColorMap::Indexed);
        colorBarMap = new QwtLinearColorMap(newPalette.colorStops[0].second, newPalette.colorStops[totalStops - 2].second, QwtColorMap::Indexed);   
        if(totalStops > 2)
        {
            for(int i = 1; i < totalStops - 2; i++)
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
        if(totalStops > 2)
        {
            for(int i = 1; i < totalStops - 1; i++)
            {
                colorMap->addColorStop(newPalette.colorStops[i].first, newPalette.colorStops[i].second);
                colorBarMap->addColorStop(newPalette.colorStops[i].first, newPalette.colorStops[i].second);
            }
        }
    }

    if(colorMap && colorBarMap)
    {
        if(m_rasterData)
        {
            QwtInterval interval = m_rasterData->interval(Qt::ZAxis);
            /*setAxisScale(QwtPlot::yRight, interval.minValue(), interval.maxValue());

            
            axisScale( QwtPlot::yRight, m_pData->m_valueMin, m_pData->m_valueMax);
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
void PlotCanvas::keyPressEvent ( QKeyEvent * event ) 
{
    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
    m_activeModifiers = event->modifiers();

    QPointF incr;
    incr.setX( invTransform(QwtPlot::xBottom, 1) - invTransform(QwtPlot::xBottom, 0) );
    incr.setY( invTransform(QwtPlot::yLeft, 1) - invTransform(QwtPlot::yLeft, 0) );

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

        if (m_rasterData->pointValid( markerPosScaleCoords ))
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
            pts.append( QPointF(hInterval.minValue(), (vInterval.minValue() + vInterval.maxValue())*0.5));
            pts.append( QPointF(hInterval.maxValue(), (vInterval.minValue() + vInterval.maxValue())*0.5));

            if (p)
            {
                p->setCoordinates(pts, true);
            }
        }
        else if(event->key() == Qt::Key_V) // draw vertical line in the middle of the plotted dataObject
        {
            pts.append( QPointF((hInterval.minValue() + hInterval.maxValue())*0.5, vInterval.minValue()));
            pts.append( QPointF((hInterval.minValue() + hInterval.maxValue())*0.5, vInterval.maxValue()));

            if (p)
            {
                p->setCoordinates(pts, true);
            }
        }
        else
        {
            pts.append( m_pLineCutLine->sample(0) );
            pts.append( m_pLineCutLine->sample(1) );

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

        if (m_rasterData->pointValid( pts[0] ) && m_rasterData->pointValid( pts[1] ))
        {
            m_pLineCutLine->setSamples(pts);

            (p)->displayCut(pts, m_lineCutUID, false);
            replot();
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::keyReleaseEvent ( QKeyEvent * event )
{
    m_activeModifiers = Qt::NoModifier;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setColorBarVisible(bool visible)
{
    m_pData->m_colorBarVisible = visible;
	enableAxis(QwtPlot::yRight, visible );
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setLabels(const QString &title, const QString &valueLabel, const QString &xAxisLabel, const QString &yAxisLabel)
{
    if(m_pData->m_autoValueLabel)
    {
        setAxisTitle(QwtPlot::yRight, valueLabel);
    }
    else
    {
        setAxisTitle(QwtPlot::yRight, m_pData->m_valueLabel);
    }

    if(m_pData->m_autoxAxisLabel)
    {
        setAxisTitle(QwtPlot::xBottom, xAxisLabel);
    }
    else
    {
        setAxisTitle(QwtPlot::xBottom, m_pData->m_xaxisLabel);
    }

    if(m_pData->m_autoyAxisLabel)
    {
        setAxisTitle(QwtPlot::yLeft, yAxisLabel);
    }
    else
    {
        setAxisTitle(QwtPlot::yLeft, m_pData->m_yaxisLabel);
    }

    if(m_pData->m_autoTitle)
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
    if(m_pData->m_autoValueLabel)
    {
        setAxisTitle(QwtPlot::yRight, m_pData->m_valueLabelDObj);
    }
    else
    {
        setAxisTitle(QwtPlot::yRight, m_pData->m_valueLabel);
    }

    if(m_pData->m_autoxAxisLabel)
    {
        setAxisTitle(QwtPlot::xBottom, m_pData->m_xaxisLabelDObj);
    }
    else
    {
        setAxisTitle(QwtPlot::xBottom, m_pData->m_xaxisLabel);
    }

    if(m_pData->m_autoyAxisLabel)
    {
        setAxisTitle(QwtPlot::yLeft, m_pData->m_yaxisLabelDObj);
    }
    else
    {
        setAxisTitle(QwtPlot::yLeft, m_pData->m_yaxisLabel);
    }

    if(m_pData->m_autoTitle)
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
    if(m_pData->m_valueScaleAuto)
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

    if(m_pData->m_xaxisScaleAuto)
    {
        ival = m_rasterData->interval(Qt::XAxis);
        m_pData->m_xaxisMin = ival.minValue();
        m_pData->m_xaxisMax = ival.maxValue();
    }

    if(m_pData->m_yaxisScaleAuto)
    {
        ival = m_rasterData->interval(Qt::YAxis);
        m_pData->m_yaxisMin = ival.minValue();
        m_pData->m_yaxisMax = ival.maxValue();
    }

    setAxisScale( QwtPlot::yRight, m_pData->m_valueMin, m_pData->m_valueMax);
    QwtScaleWidget *widget = axisWidget(QwtPlot::yRight);
    if (widget)
    {
        QwtInterval ival(m_pData->m_valueMin, m_pData->m_valueMax);
        axisWidget(QwtPlot::yRight)->setColorMap( ival, const_cast<QwtColorMap*>(widget->colorMap())); //the color map should be unchanged
    }
    
    setAxisScale( QwtPlot::xBottom, m_pData->m_xaxisMin, m_pData->m_xaxisMax);
    
    QwtScaleEngine *scaleEngine = axisScaleEngine(QwtPlot::yLeft);

    if (m_pData->m_yaxisFlipped)
    {
        scaleEngine->setAttribute( QwtScaleEngine::Inverted, true);
        setAxisScale( QwtPlot::yLeft, m_pData->m_yaxisMax, m_pData->m_yaxisMin);
    }
    else
    {
        scaleEngine->setAttribute( QwtScaleEngine::Inverted, false);
        setAxisScale( QwtPlot::yLeft, m_pData->m_yaxisMin, m_pData->m_yaxisMax);
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
        setAxisScale( QwtPlot::xBottom, m_pData->m_xaxisMin, m_pData->m_xaxisMax);
    }
    else if (axis == Qt::YAxis)
    {
        m_pData->m_yaxisScaleAuto = false;

        QwtScaleEngine *scaleEngine = axisScaleEngine(QwtPlot::yLeft);
        m_pData->m_yaxisMin = interval.x();
        m_pData->m_yaxisMax = interval.y();

        if (m_pData->m_yaxisFlipped)
        {
            scaleEngine->setAttribute( QwtScaleEngine::Inverted, true);
            setAxisScale( QwtPlot::yLeft, m_pData->m_yaxisMax, m_pData->m_yaxisMin);
        }
        else
        {
            scaleEngine->setAttribute( QwtScaleEngine::Inverted, false);
            setAxisScale( QwtPlot::yLeft, m_pData->m_yaxisMin, m_pData->m_yaxisMax);
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
            widget->setColorMap( ival, const_cast<QwtColorMap*>(widget->colorMap())); //the color map should be unchanged
        }

        setAxisScale( QwtPlot::yRight, m_pData->m_valueMin, m_pData->m_valueMax);
        
        m_rasterData->setInterval(Qt::ZAxis, QwtInterval(m_pData->m_valueMin, m_pData->m_valueMax));
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF PlotCanvas::getInterval(Qt::Axis axis) const
{
    QwtInterval i = m_rasterData->interval(axis);
    return QPointF( i.minValue(), i.maxValue() );
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setState( tState state)
{
    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());

    if (m_pData->m_state != state)
    {
        if (m_pData->m_state == tMultiPointPick && state != tIdle)
        {
            return; //multiPointPick needs to go back to idle
        }

        if (m_pZoomer) m_pZoomer->setEnabled( state == tZoom );
        if (m_pPanner) m_pPanner->setEnabled( state == tPan );
        if (m_pValuePicker) m_pValuePicker->setEnabled( state == tValuePicker );
        if (m_pLineCutPicker) m_pLineCutPicker->setEnabled( state == tLineCut );
        if (m_pStackPicker) m_pStackPicker->setEnabled( state == tStackCut );
        //if (m_pMultiPointPicker) m_pMultiPointPicker->setEnabled( state == tMultiPointPick );

        if (state == tMultiPointPick || state == tIdle)
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

        if (state == tZoom || state == tPan || state == tMultiPointPick ||state == tValuePicker ||state == tIdle)
        {
            if (p)
            {
                p->setCoordinates(QVector<QPointF>(),false);
            }
        }

        switch (state)
        {
        case tIdle:
            canvas()->setCursor( Qt::ArrowCursor );
            break;
		case tZoom:
			canvas()->setCursor( Qt::CrossCursor );
			break;
		case tPan:
			canvas()->setCursor( Qt::OpenHandCursor );
			break;
        case tValuePicker:
            canvas()->setCursor( Qt::CrossCursor );
            break;
		case tStackCut:
			canvas()->setCursor( Qt::CrossCursor );
			break;
        case tMultiPointPick:
            canvas()->setCursor( Qt::CrossCursor );
            break;
        default:
            canvas()->setCursor( Qt::ArrowCursor );
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

        if(m_activeModifiers.testFlag( Qt::ControlModifier ))
        {
            if(abs(ptScale.x() - m_pStackCutMarker->xValue()) > abs(ptScale.y() - m_pStackCutMarker->yValue()))
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

        if(m_activeModifiers.testFlag( Qt::ControlModifier ))
        {
            if(abs(pts[1].x() - pts[0].x()) > abs(pts[1].y() - pts[0].y()))
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
	size_t limits[] = {2,2,0,99999};
	ito::DataObject *dObj = apiCreateFromDataObject(coords, 2, ito::tFloat32, limits, &retval);

	QwtSymbol::Style symStyle = QwtSymbol::XCross;
	QSize symSize(5,5);
	QBrush symBrush(Qt::NoBrush);
	QPen symPen(Qt::red);


	QRegExp rgexp("^([b|g|r|c|m|y|k|w]?)([.|o|s|d|\\^|v|<|>|x|+|*|h]?)(\\d*)$");
	if (rgexp.indexIn(style) != -1)
	{
		QString s = rgexp.cap(1);

		if (s == "b") symPen.setColor( Qt::blue );
		else if (s == "g") symPen.setColor( Qt::green );
		else if (s == "r") symPen.setColor( Qt::red );
		else if (s == "c") symPen.setColor( Qt::cyan );
		else if (s == "m") symPen.setColor( Qt::magenta );
		else if (s == "y") symPen.setColor( Qt::yellow );
		else if (s == "k") symPen.setColor( Qt::black );
		else if (s == "w") symPen.setColor( Qt::white );

		s = rgexp.cap(2);
		bool ok;

		if (s == ".") symStyle = QwtSymbol::Ellipse;
		else if (s == "o") symStyle = QwtSymbol::Ellipse;
		else if (s == "s") symStyle = QwtSymbol::Rect;
		else if (s == "d") symStyle = QwtSymbol::Diamond;
		else if (s == ">") symStyle = QwtSymbol::RTriangle;
		else if (s == "v") symStyle = QwtSymbol::DTriangle;
		else if (s == "^") symStyle = QwtSymbol::UTriangle;
		else if (s == "<") symStyle = QwtSymbol::LTriangle;
		else if (s == "x") symStyle = QwtSymbol::XCross;
 		else if (s == "*") symStyle = QwtSymbol::Star1;
		else if (s == "+") symStyle = QwtSymbol::Cross;
		else if (s == "h") symStyle = QwtSymbol::Hexagon;

		s = rgexp.cap(3);
		int size = s.toInt(&ok);
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
		//QMultiHash<QString, QPair<int, QwtPlotMarker*> > m_plotMarkers;
		QwtPlotMarker *marker = NULL;
		int nrOfMarkers = dObj->getSize(1);

		if (id == "") id = "unknown";
		
		ito::float32 *xCoords = (ito::float32*)dObj->rowPtr(0,0);
		ito::float32 *yCoords = (ito::float32*)dObj->rowPtr(0,1);

		for (int i = 0; i < nrOfMarkers; ++i)
		{
			marker = new QwtPlotMarker();
			marker->setSymbol(new QwtSymbol(symStyle,symBrush,symPen,symSize) );
			marker->setValue(xCoords[i], yCoords[i]);
			marker->attach(this);

			m_plotMarkers.insert(id, QPair<int, QwtPlotMarker*>(plane, marker));
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
	while ( i.hasNext() )
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
		retval += ito::RetVal::format(ito::retError,0,"No marker with id '%s' found.", id.toAscii().data());
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

    if (type == 1) //multiPointPick
    {
        if (start)
        {
            setState(tMultiPointPick);
            MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());

            if (m)
            {
                m->setMaxNrItems( maxNrOfPoints );
                m_pMultiPointPicker->setEnabled(true);

                if (maxNrOfPoints > 0)
                {
                    emit statusBarMessage( tr("Please select %1 points or press Space to quit earlier. Esc aborts the selection.").arg( maxNrOfPoints ) );
                }
                else
                {
                    emit statusBarMessage( tr("Please select points and press Space to end the selection. Esc aborts the selection.") );
                }

                //QKeyEvent evt(QEvent::KeyPress, Qt::Key_M, Qt::NoModifier);
                //m_pMultiPointPicker->eventFilter( m_pMultiPointPicker->parent(), &evt); //starts the process
            }
        }
        else //start == false
        {
            setState(tIdle);
            m_pMultiPointPicker->setEnabled(false);
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError,0,"Unknown type for userInteractionStart");
    }

    return retval;


}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::multiPointActivated (bool on) 
{ 
    if (m_pData->m_state == tMultiPointPick)
    {
        if (!on)
        {
            QPolygon polygon = m_pMultiPointPicker->selection();

            QPolygonF polygonScale;
            bool aborted = false;

            if (polygon.size() == 0)
            {
                emit statusBarMessage( tr("Selection has been aborted."), 2000 );
                aborted = true;
            }
            else
            {
                QPointF pt;

                for (int i = 0; i < polygon.size() - 1; ++i)
                {
                    pt.rx() = invTransform(QwtPlot::xBottom, polygon[i].rx());
                    pt.ry() = invTransform(QwtPlot::yLeft, polygon[i].ry());
                    polygonScale.append( pt );
                }

                emit statusBarMessage( tr("%1 points have been selected.").arg(polygon.size()-1), 2000 );
            }

            Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
            if (p)
            {
                emit p->userInteractionDone(1, aborted, polygonScale);
            }

            userInteractionStart(1, false, -1);

        }
    }
};

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
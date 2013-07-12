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

#include "dataObjRasterData.h"
#include "itom2dqwtplot.h"
#include "valuePicker2d.h"
#include "multiPointPickerMachine.h"

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
	canvas()->setFrameShadow(QFrame::Plain);
	canvas()->setFrameShape(QFrame::NoFrame);
	canvas()->setCursor( Qt::ArrowCursor );
	
	//main item on canvas -> the data object
    m_dObjItem = new DataObjItem("Data Object");
    m_dObjItem->setRenderThreadCount(0);
    m_dObjItem->setColorMap( new QwtLinearColorMap(QColor::fromRgb(0,0,0), QColor::fromRgb(255,255,255), QwtColorMap::Indexed));

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
    m_pLineCutPicker = new QwtPlotPicker(QwtPicker::CrossRubberBand, QwtPicker::AlwaysOn, canvas());
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
    m_pMultiPointPicker = new QwtPlotPicker(QwtPicker::CrossRubberBand, QwtPicker::ActiveOnly, canvas());
    m_pMultiPointPicker->setEnabled(true);
    //m_pMultiPointPicker->setStateMachine(new QwtPickerClickPointMachine); 
    m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine);
    m_pMultiPointPicker->setTrackerPen( QPen(Qt::blue) );

	//prepare color bar
	QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
    rightAxis->setColorBarEnabled(true);
    rightAxis->setColorBarWidth(30);

    rightAxis->setColorMap(QwtInterval(0,1.0), new QwtLinearColorMap(QColor::fromRgb(0,0,0), QColor::fromRgb(255,255,255), QwtColorMap::Indexed));
    rightAxis->setFont(QFont("Verdana",8,1,true));

    rightAxis->setMargin(20); //margin to right border of window
    rightAxis->scaleDraw()->setLength(20);
    rightAxis->scaleDraw()->enableComponent(QwtAbstractScaleDraw::Backbone,false);

    setAxisScale(QwtPlot::yRight, 0, 1.0 );
    enableAxis(QwtPlot::yRight, m_pData->m_colorBarVisible );
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
    QPen rubberBandPen = apiGetFigureSetting(parent(), "zoomRubberBandPen", QPen(QBrush(Qt::red),2,Qt::DashLine),NULL).value<QPen>();
    QPen trackerPen = apiGetFigureSetting(parent(), "trackerPen", QPen(QBrush(Qt::red),2),NULL).value<QPen>();
    QFont trackerFont = apiGetFigureSetting(parent(), "trackerFont", QFont("Verdana",10),NULL).value<QFont>();
    QBrush trackerBg = apiGetFigureSetting(parent(), "trackerBackground", QBrush(QColor(255,255,255,155), Qt::SolidPattern),NULL).value<QBrush>();
    
    m_pZoomer->setRubberBandPen(rubberBandPen);
    m_pZoomer->setTrackerFont(trackerFont);
    m_pZoomer->setTrackerPen(trackerPen);

    m_pValuePicker->setTrackerFont(trackerFont);
    m_pValuePicker->setTrackerPen(trackerPen);
    m_pValuePicker->setBackgroundFillBrush(trackerBg);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::refreshPlot(const ito::DataObject *dObj, int plane /*= -1*/)
{
    int colorIndex;
    bool needToUpdate = false;

    m_dObjPtr = dObj;

    //QString valueLabel, axisLabel, title;

    if (dObj)
    {
        int dims = dObj->getDims();
        int width = dims > 0 ? dObj->getSize(dims - 1) : 0;
        int height = dims > 1 ? dObj->getSize(dims - 2) : 1;

        

        needToUpdate = m_rasterData->updateDataObject( dObj, plane);

        bool valid;
        ito::DataObjectTagType tag;
        tag = dObj->getTag("title", valid);
        m_pData->m_titleDObj = valid? QString::fromStdString(tag.getVal_ToString()) : "";
    } 

    updateLabels();

    if(needToUpdate)
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
void PlotCanvas::setColorMap(QString colormap /*= "__next__"*/)
{
    QwtLinearColorMap *colorMap = NULL;
    QwtLinearColorMap *colorBarMap = NULL;
    ito::ItomPalette newPalette;
    ito::RetVal retval(ito::retOk);
    int numPalettes = 1;

    if (colormap == "__next__")
    {
        retval = apiPaletteGetNumberOfColorBars(numPalettes);

        if (numPalettes == 0 || retval.containsError())
        {
            return;
        }

        m_curColorMapIndex++;
        m_curColorMapIndex %= numPalettes; //map index to [0,numPalettes)
        retval += apiPaletteGetColorBarIdx(m_curColorMapIndex, newPalette);
    }
    else
    {
        retval += apiPaletteGetColorBarName(colormap, newPalette);
    }

    if (retval.containsError() || newPalette.getSize() < 2)
    {
        return;
    }
   

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
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::keyPressEvent ( QKeyEvent * event ) 
{
    m_activeModifiers = event->modifiers();

    QPointF incr;
    incr.setX( transform(QwtPlot::xBottom, 1) - transform(QwtPlot::xBottom, 0) );
    incr.setY( transform(QwtPlot::yLeft, 1) - transform(QwtPlot::yLeft, 0) );

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
            ((Itom2dQwtPlot*)parent())->displayCut(pts, m_zstackCutUID,true);
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
        }
        else if(event->key() == Qt::Key_V) // draw vertical line in the middle of the plotted dataObject
        {
            pts.append( QPointF((hInterval.minValue() + hInterval.maxValue())*0.5, vInterval.minValue()));
            pts.append( QPointF((hInterval.minValue() + hInterval.maxValue())*0.5, vInterval.maxValue()));
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
        }

        if (m_rasterData->pointValid( pts[0] ) && m_rasterData->pointValid( pts[1] ))
        {
            m_pLineCutLine->setSamples(pts);

            ((Itom2dQwtPlot*)parent())->displayCut(pts, m_lineCutUID, false);
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
    if (m_pData->m_state != state)
    {
        if (m_pZoomer) m_pZoomer->setEnabled( state == tZoom );
        if (m_pPanner) m_pPanner->setEnabled( state == tPan );
        if (m_pValuePicker) m_pValuePicker->setEnabled( state == tValuePicker );
        if (m_pLineCutPicker) m_pLineCutPicker->setEnabled( state == tLineCut );
        if (m_pStackPicker) m_pStackPicker->setEnabled( state == tStackCut );
        //if (m_pMultiPointPicker) m_pMultiPointPicker->setEnabled( state == tMultiPointPick );

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
ito::RetVal PlotCanvas::pickPoints(ito::DataObject *coordsOut, int maxNrOfPoints)
{
    ito::RetVal retval;

    setState(tMultiPointPick);

    connect(m_pMultiPointPicker, SIGNAL(activated(bool)), this, SLOT(multiPointActivated(bool)));
    connect(m_pMultiPointPicker, SIGNAL(selected(QPolygon)), this, SLOT(multiPointSelected (QPolygon) ));
    connect(m_pMultiPointPicker, SIGNAL(appended(QPoint)), this, SLOT(multiPointAppended (QPoint) ));

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

        QKeyEvent evt(QEvent::KeyPress, Qt::Key_M, Qt::NoModifier);
        
        m_pMultiPointPicker->eventFilter( m_pMultiPointPicker->parent(), &evt); //starts the process

        while (m_pMultiPointPicker && m_pMultiPointPicker->isActive())
        {
            QCoreApplication::processEvents();
        }

        QPolygon polygon = m_pMultiPointPicker->selection();
        int dims = 2; //m_dObjPtr ? m_dObjPtr->getDims() : 2;
        ito::DataObject output(dims, polygon.size(), ito::tFloat64);

        if (polygon.size() == 0)
        {
            emit statusBarMessage( tr("Selection has been aborted."), 2000 );
            retval += ito::RetVal(ito::retError,0,"selection aborted");
        }
        else
        {
            ito::float64 *ptr = (ito::float64*)output.rowPtr(0,0);
            int stride = polygon.size();

            for (int i = 0; i < polygon.size(); ++i)
            {
                ptr[i] = invTransform(QwtPlot::xBottom, polygon[i].rx());
                ptr[i + stride] = invTransform(QwtPlot::yLeft, polygon[i].ry());
            }

            emit statusBarMessage( tr("%1 points have been selected.").arg(polygon.size()), 2000 );
        }

        *coordsOut = output;
    }

    disconnect(m_pMultiPointPicker, SIGNAL(activated(bool)), this, SLOT(multiPointActivated(bool)));
    disconnect(m_pMultiPointPicker, SIGNAL(selected(QPolygon)), this, SLOT(multiPointSelected (QPolygon) ));
    disconnect(m_pMultiPointPicker, SIGNAL(appended(QPoint)), this, SLOT(multiPointAppended (QPoint) ));

    setState(tIdle);

    return retval;
}
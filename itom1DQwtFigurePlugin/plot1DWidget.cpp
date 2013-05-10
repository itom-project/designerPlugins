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

#include "itom1DQwtFigure.h"
#include "plot1DWidget.h"
#include "dataObjectSeriesData.h"
#include "qwtPlotCurveDataObject.h"
#include "common/sharedStructuresGraphics.h"
#include "common/apiFunctionsGraphInc.h"

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
        m_startScaledX(false),
        m_startScaledY(false),
        m_xDirect(false),
        m_yDirect(false),
        m_multiLine(MultiRows),
        m_autoLineColIndex(0),
        m_lineCol(0),
        m_lineStyle(1),
        m_pParent(parent),
        m_actPickerIdx(-1),
        m_cmplxState(false),
        m_pData(data),
        m_state(stateIdle)
{
    this->setMouseTracking(false);

    //this is the border between the canvas and the axes and the overall mainwindow
	setContentsMargins(5,5,5,5);
	
	//canvas() is the real plotting area, where the plot is printed (without axes...)
	canvas()->setFrameShadow(QFrame::Plain);
	canvas()->setFrameShape(QFrame::NoFrame);
    canvas()->setCursor( Qt::ArrowCursor );

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

    m_pZoomer = new QwtPlotZoomer(QwtPlot::xBottom, QwtPlot::yLeft, canvas());
    m_pZoomer->setEnabled(false);
    m_pZoomer->setTrackerMode(QwtPicker::AlwaysOn);
    //all others settings for zoomer are set in init (since they need access to the settings via api)

    m_pPanner = new QwtPlotPanner(canvas());
    m_pPanner->setAxisEnabled(QwtPlot::yRight,false);
    m_pPanner->setCursor(Qt::SizeAllCursor);
    m_pPanner->setEnabled(false);;

    //value picker
    m_pValuePicker = new ValuePicker1D(QwtPlot::xBottom, QwtPlot::yLeft, canvas());
    m_pValuePicker->setEnabled(false);
    m_pValuePicker->setTrackerMode(QwtPicker::AlwaysOn);
    //all others settings for tracker are set in init (since they need access to the settings via api)
}

//----------------------------------------------------------------------------------------------------------------------------------
Plot1DWidget::~Plot1DWidget()
{
    foreach( QwtPlotCurve* c, m_plotCurveItems)
    {
        c->detach();
        delete c;
    }
    m_plotCurveItems.clear();

    foreach(Marker m, m_markers)
    {
        m.item->detach();
        delete m.item;
    }
    m_markers.clear();

    if(m_pPlotGrid)
    {
        m_pPlotGrid->detach();
        delete m_pPlotGrid;
        m_pPlotGrid = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::init()
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

////----------------------------------------------------------------------------------------------------------------------------------
//void Plot1DWidget::replot()
//{
//    DataObjectSeriesData *data = NULL;
//    if (m_plotCurveItems.size() > 0)
//    {
//        /*if(m_pCurserEnable && m_pContent[0])
//        {
//            QVector<QPointF> pts(3);
//
//            data = (DataObjectSeriesData *)m_pContent[0]->data();
//            data->setRasterObj();
//
//            int x1 = ((DataObjectSeriesData*)(m_pContent[0]->data()))->size()-1;
//
//            m_Curser[0] = m_Curser[0] > x1 ? x1: m_Curser[0];
//            m_Curser[1] = m_Curser[1] > x1 ? x1: m_Curser[1];
//
//            pts[0] = data->sample(m_Curser[0]);
//            pts[1] = data->sample(m_Curser[1]);
//            pts[2] = pts[1] - pts[0];
//
//            m_pCurser1->setValue(pts[0]);
//            m_pCurser2->setValue(pts[1]);
//            data->releaseRasterObj();
//            
//            ((Itom1DQwtFigure*) m_pParent)->setMarkerCoordinates(pts);
//        }*/
//
//        /*foreach(QwtPlotCurve* curve, m_plotCurveItems)
//        {
//            data = (DataObjectSeriesData *)m_pContent[n]->data();
//
//            if (data && data->isDobjInit())
//            {
//                data->beginSampling(
//                data->setRasterObj();*/
//    QwtPlot::replot();
//                /*data->releaseRasterObj();
//            }
//            }
//        }*/
//
//
//    }
//}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setLabels(const QString &title, const QString &valueLabel, const QString &axisLabel)
{
    if(m_pData->m_autoValueLabel)
    {
        setAxisTitle(QwtPlot::yLeft, valueLabel);
    }
    else
    {
        setAxisTitle(QwtPlot::yLeft, m_pData->m_valueLabel);
    }

    if(m_pData->m_autoAxisLabel)
    {
        setAxisTitle(QwtPlot::xBottom, axisLabel);
    }
    else
    {
        setAxisTitle(QwtPlot::xBottom, m_pData->m_axisLabel);
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
void Plot1DWidget::updateLabels()
{
    if(m_pData->m_autoValueLabel)
    {
        setAxisTitle(QwtPlot::yLeft, m_pData->m_valueLabelDObj);
    }
    else
    {
        setAxisTitle(QwtPlot::yLeft, m_pData->m_valueLabel);
    }

    if(m_pData->m_autoAxisLabel)
    {
        setAxisTitle(QwtPlot::xBottom, m_pData->m_axisLabelDObj);
    }
    else
    {
        setAxisTitle(QwtPlot::xBottom, m_pData->m_axisLabel);
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
void Plot1DWidget::refreshPlot(const ito::DataObject* dataObj, QVector<QPointF> bounds)
{
    DataObjectSeriesData* seriesData = NULL;
    int colorIndex;
    int numCurves = 1;
    QwtPlotCurve *curve = NULL;
    QwtPlotCurveDataObject *dObjCurve = NULL;
    bool _unused;

    //QString valueLabel, axisLabel, title;

    if (dataObj)
    {
        int dims = dataObj->getDims();
        int width = dims > 0 ? dataObj->getSize(dims - 1) : 0;
        int height = dims > 1 ? dataObj->getSize(dims - 2) : 1;

        if(dataObj->getType() == ito::tComplex128 || dataObj->getType() == ito::tComplex64)
        {
            if(!m_cmplxState) ((Itom1DQwtFigure*)m_pParent)->enableComplexGUI(true);
            m_cmplxState = true;                
        }
        else
        {
            if(m_cmplxState) ((Itom1DQwtFigure*)m_pParent)->enableComplexGUI(false);
            m_cmplxState = false;                
        }

        if(bounds.size() == 0)
        {
            switch (m_multiLine)
            {
            case FirstRow:
            case FirstCol:
                numCurves = 1;
                break;
            case MultiRows:
                numCurves = height;
                break;
            case MultiCols:
                numCurves = width;
                break;
            }
        }
        else //if there are boundaries, only plot one curve from bounds[0] to bounds[1]
        {
            numCurves = 1;
        }

        //check if current number of curves does not correspond to height. If so, adjust the number of curves to the required number
        while(m_plotCurveItems.size() > numCurves)
        {
            curve = m_plotCurveItems.takeLast();
            curve->detach();
            delete curve;
        }

        while(m_plotCurveItems.size() < numCurves)
        {
            dObjCurve = new QwtPlotCurveDataObject("");
            dObjCurve->setData(NULL);
            dObjCurve->attach(this);
            QPen plotPen;
            colorIndex = (m_autoLineColIndex++) % m_colorList.size();
            plotPen.setColor(m_colorList[colorIndex]);
            plotPen.setStyle((Qt::PenStyle)m_lineStyle);
            dObjCurve->setPen(plotPen);
            m_plotCurveItems.append(dObjCurve);
        }

        

        if( bounds.size() == 0)
        {
            QVector<QPointF> pts(2);

            switch(m_multiLine)
            {
            case FirstCol:
            case MultiCols:
                pts[0].setY( dataObj->getPixToPhys(dims-2, 0, _unused) );
                pts[1].setY( dataObj->getPixToPhys(dims-2, height-1, _unused) ); 

                for (int n = 0; n < numCurves; n++)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[n]->data());
                    pts[0].setX( dataObj->getPixToPhys(dims-1, n, _unused) );
                    pts[1].setX( dataObj->getPixToPhys(dims-1, n, _unused) );
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
                }

                if(numCurves > 0)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
                    m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel();
                    m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel();
                }
                break;

            case FirstRow:
            case MultiRows:
                pts[0].setX( dataObj->getPixToPhys(dims-1, 0, _unused) );
                pts[1].setX( dataObj->getPixToPhys(dims-1, width-1, _unused) );

                for (int n = 0; n < numCurves; n++)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[n]->data());
                    pts[0].setY( dataObj->getPixToPhys(dims-2, n, _unused) );
                    pts[1].setY( dataObj->getPixToPhys(dims-2, n, _unused) );
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
                }

                if(numCurves > 0)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
                    m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel();
                    m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel();
                }
                break;
            }
        }
        else if(bounds.size() == 2) //boundaries given ->line plot
        {
            seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
            if (seriesData && seriesData->isDobjInit())
            {
                seriesData->updateDataObject(dataObj, bounds);
            }
            else
            {
                seriesData = new DataObjectSeriesData(1);
                seriesData->updateDataObject(dataObj, bounds);
                m_plotCurveItems[0]->setData(seriesData);
            }

            m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel();
            m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel();
        }
        else if(bounds.size() == 1) //point in third dimension
        {
            seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
            if (seriesData && seriesData->isDobjInit())
            {
                seriesData->updateDataObject(dataObj, bounds);
            }
            else
            {
                seriesData = new DataObjectSeriesData(1);
                seriesData->updateDataObject(dataObj, bounds);
                m_plotCurveItems[0]->setData(seriesData);
            }

            m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel();
            m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel();
        }

        bool valid;
        ito::DataObjectTagType tag;
        tag = dataObj->getTag("title", valid);
        m_pData->m_titleDObj = valid? QString::fromStdString(tag.getVal_ToString()) : "";
    } 

    updateLabels();

    if(seriesData)
    {
        QByteArray hash = seriesData->getHash();

        if(hash != m_hash)
        {
            updateMarkerPosition(true);

            QRectF rect = seriesData->boundingRect();
            if(m_pData->m_valueScaleAuto)
            {
                m_pData->m_valueMin = rect.top();
                m_pData->m_valueMax = rect.bottom();
            }

            if(m_pData->m_axisScaleAuto)
            {
                m_pData->m_axisMin = rect.left();
                m_pData->m_axisMax = rect.right();
            }

            updateScaleValues(); //replot is done here

            m_pZoomer->setZoomBase( true );
        }
        else
        {
            updateMarkerPosition(true,false);

            replot();
        }

        m_hash = hash;
    }
    else
    {
        replot();
    }
   
    /*if(m_startScaledY && seriesData)
    {
        seriesData->setIntervalRange(Qt::YAxis, false, m_startRangeY.x(), m_startRangeY.y());
        m_startScaledY = false;
    }
    if(m_startScaledX && seriesData)
    {
        seriesData->setIntervalRange(Qt::XAxis, false, m_startRangeX.x(), m_startRangeX.y());
        m_startScaledY = false;
    }*/

    
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::keyPressEvent ( QKeyEvent * event )
{
    Marker *m;
    int curves = m_plotCurveItems.size();

    if(m_state == statePicker)
    {
        switch(event->key())
        {
        case Qt::Key_Left:
            for(int i = 0 ; i < m_markers.size() ; i++)
            {
                m = &(m_markers[i]);
                if(m->active)
                {
                    stickMarkerToXPx(m, m->item->xValue(), -1);
                }
            }
            break;
        case Qt::Key_Right:
            for(int i = 0 ; i < m_markers.size() ; i++)
            {
                m = &(m_markers[i]);
                if(m->active)
                {
                     stickMarkerToXPx(m, m->item->xValue(), 1);
                }
            }
            break;
        case Qt::Key_Up:
            for(int i = 0 ; i < m_markers.size() ; i++)
            {
                m = &(m_markers[i]);
                if(m->active)
                {
                    m->curveIdx++;
                    if(m->curveIdx >= curves) m->curveIdx = 0;
                    stickMarkerToXPx(m, m->item->xValue(), 0);
                }
            }
            break;
        case Qt::Key_Down:
            for(int i = 0 ; i < m_markers.size() ; i++)
            {
                m = &(m_markers[i]);
                if(m->active)
                {
                    m->curveIdx--;
                    if(m->curveIdx <= 0) m->curveIdx = curves-1;
                    stickMarkerToXPx(m, m->item->xValue(), 0);
                }
            }
            break;
        case Qt::Key_Delete:
        {
            QList<Marker>::iterator it = m_markers.begin();

            while(it != m_markers.end())
            {
                if(it->active)
                {
                    it->item->detach();
                    delete it->item;
                    it = m_markers.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            break;
        }
        }

        updateMarkerPosition(false,false);
    }

    /*int x1 = ((DataObjectSeriesData*)(m_plotCurveItems[0]->data()))->size()-1;

     switch(event->key())
    {
        case Qt::Key_1:
            m_curserFirstActive = true;
            break;
        case Qt::Key_2:
            m_curserFirstActive = false;
            break;
        case Qt::Key_Left:
            
            m_Curser[!m_curserFirstActive]--;
            m_Curser[!m_curserFirstActive] = m_Curser[!m_curserFirstActive] < 0 ? 0 : m_Curser[!m_curserFirstActive];

            break;
        case Qt::Key_Right:
            m_Curser[!m_curserFirstActive]++;
            m_Curser[!m_curserFirstActive] = m_Curser[!m_curserFirstActive] > x1 ? x1 : m_Curser[!m_curserFirstActive];
            break;
    }

    if(m_Curser[0] > m_Curser[1])
    {
        int temp = m_Curser[0];
        m_Curser[0] = m_Curser[1];
        m_Curser[1] = temp;
        m_curserFirstActive = !m_curserFirstActive;
    }*/

    event->accept();
    replot();
}


//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mouseReleaseEvent ( QMouseEvent * event )
{
    if(m_state == statePicker)
    {
        int xPx = m_pValuePicker->trackerPosition().x();
        int yPx = m_pValuePicker->trackerPosition().y();
        double xScale = invTransform( xBottom, xPx );
        double yScale = invTransform( yLeft, yPx );

        bool closeToMarker = false;

        if(event->button() == Qt::LeftButton)
        {
            for(int i = 0 ; i < m_markers.size() ; i++)
            {
                if( abs( transform(xBottom, m_markers[i].item->xValue()) - xPx) < 20 && abs(transform(yLeft, m_markers[i].item->yValue()) - yPx) < 20 )
                {
                    closeToMarker = true;
                    m_markers[i].active = true;
                    
                }
                else if( (event->modifiers() & Qt::ControlModifier) == false && m_markers[i].active)
                {
                    m_markers[i].active = false;
                    //m_markers[i].item->setSymbol( new QwtSymbol(QwtSymbol::Diamond, m_markers[i].color, QPen(m_markers[i].color,1), QSize(6,6) ));
                }
            }

            if(!closeToMarker && m_plotCurveItems.size() > 0)
            {
                Marker marker;
                marker.item = new QwtPlotMarker();
                marker.item->attach(this);
                marker.active = true;
                //marker.color = Qt::darkGreen;
                //marker.item->setSymbol(new QwtSymbol(QwtSymbol::Diamond,QBrush(Qt::white), QPen(marker.color,1),  QSize(8,8) ));
                
                marker.curveIdx = 0;
                stickMarkerToXPx(&marker, xScale, 0);

                marker.item->setVisible(true);
                
                m_markers.append(marker);

            }

            updateMarkerPosition(false,false);

            replot();

        }

    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::stickMarkerToXPx(Marker *m, double xScaleStart, int dir) //dir: 0: this point, -1: next valid to the left or this if not possible, 1: next valid to the right or this if not possible
{
    DataObjectSeriesData *data = (DataObjectSeriesData*)(m_plotCurveItems[m->curveIdx]->data());

    if(!qIsFinite(xScaleStart)) xScaleStart = m->item->xValue();

    int thisIdx = data->getPosToPix(xScaleStart);
    int s = data->size();
    QPointF p;
    bool found = false;
    bool d = true;

    if(dir == 0)
    {
        while(!found)
        {
            if(thisIdx >= 0 && thisIdx < s)
            {
                p = data->sample(thisIdx);
                if(qIsFinite(p.ry()))
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

            if(d)
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
    if(dir == -1)
    {
        while(!found)
        {
            thisIdx -= 1;
            if( thisIdx >= 0)
            {
                p = data->sample(thisIdx);
                if(qIsFinite(p.ry()))
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
        while(!found)
        {
            thisIdx += 1;
            if( (thisIdx) < s)
            {
                p = data->sample(thisIdx);
                if(qIsFinite(p.ry()))
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
    if(m_showContextMenu && m_pPanner->isEnabled() == false)
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
    DataObjectSeriesData* seriesData = m_plotCurveItems.size() > 0 ? static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data()) : NULL;
    if(seriesData)
    {
        seriesData->setIntervalRange(axis, autoCalcLimits, minValue, maxValue);

        replot();
        return retOk;
    }
    else
    {
        switch(axis)
        {
            case Qt::YAxis:
                m_startScaledY = true;
                m_startRangeY = QPointF(minValue, maxValue);
            break;
            case Qt::XAxis:
                m_startScaledX = true;
                m_startRangeX = QPointF(minValue, maxValue);
            break;
        }
    }
    return retError;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setZoomerEnable(const bool checked)
{
    if(checked)
    {
        setPickerEnable(false);
        setPannerEnable(false);

        m_state = stateZoomer;

        m_pPanner->setEnabled(false);

        DataObjectSeriesData *data = NULL;

        foreach( QwtPlotCurve *curve, m_plotCurveItems)
        {
            data = (DataObjectSeriesData *)curve->data();
            m_pZoomer->setZoomBase(data->boundingRect());
        }

        m_pZoomer->setEnabled(true);
        canvas()->setCursor( Qt::CrossCursor );
    }
    else
    {
        m_state = stateIdle;

        m_pZoomer->setEnabled(false);
        canvas()->setCursor( Qt::ArrowCursor );

        foreach( QwtPlotCurve *curve, m_plotCurveItems)
        {
            setAxisAutoScale(curve->xAxis(),true);
            setAxisAutoScale(curve->yAxis(),true);
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setPickerEnable(const bool checked)
{
    if(checked)
    {   
        setZoomerEnable(false);
        setPannerEnable(false);

        m_state = statePicker;
        m_pValuePicker->setEnabled(true);
        canvas()->setCursor( Qt::CrossCursor );
    }
    else
    {
        m_state = stateIdle;
        m_pValuePicker->setEnabled(false);
        canvas()->setCursor( Qt::ArrowCursor );

        /*foreach(Marker m, m_markers)
        {
            m.item->detach();
            delete m.item;
        }
        m_markers.clear();*/
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setPannerEnable(const bool checked)
{
    if(checked)
    {
        setZoomerEnable(false);
        setPickerEnable(false);
        m_state = statePanner;
        canvas()->setCursor( Qt::OpenHandCursor);
    }
    else
    {
        m_state = stateIdle;
        canvas()->setCursor( Qt::ArrowCursor );
    }
    m_pPanner->setEnabled(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::updateScaleValues()
{
    if(m_pData->m_valueScaleAuto)
    {
        setAxisAutoScale( QwtPlot::yLeft, true );
    }
    else
    {
        setAxisScale( QwtPlot::yLeft, m_pData->m_valueMin, m_pData->m_valueMax );
    }

    if(m_pData->m_axisScaleAuto)
    {
        setAxisAutoScale( QwtPlot::xBottom, true );
    }
    else
    {
        setAxisScale( QwtPlot::xBottom, m_pData->m_axisMin, m_pData->m_axisMax );
    }

    replot();
}

void Plot1DWidget::updateMarkerPosition(bool updatePositions, bool clear/* = false*/)
{
    if(clear)
    {
        foreach(Marker m, m_markers)
        {
            m.item->detach();
            delete m.item;
        }
        m_markers.clear();
    }

    QColor colors[3] = { Qt::red, Qt::darkGreen, Qt::darkGray };
    int cur = 0;
    Marker *m;
    QVector<QPointF> points;

    for(int i = 0 ; i < m_markers.size() ; i++)
    {
        m = &(m_markers[i]);
        if(updatePositions)
        {
            stickMarkerToXPx(m, std::numeric_limits<double>::signaling_NaN() ,0);
        }

        if(m->active)
        {
            m_markers[i].item->setSymbol( new QwtSymbol(QwtSymbol::Diamond, Qt::white, QPen(colors[cur],2), QSize(8,8) ));
        }
        else
        {
            m_markers[i].item->setSymbol( new QwtSymbol(QwtSymbol::Diamond, colors[cur], QPen(colors[cur],2), QSize(6,6) ));
        }

        if(cur < 2) cur++;
        points << QPointF(m_markers[i].item->xValue(), m_markers[i].item->yValue());
            
    }

    QString coords, offsets;
    if(points.size() > 1)
    {
        coords = QString("[%1; %2]\n [%3; %4]").arg( points[0].rx(),0,'g',4 ).arg( points[0].ry(),0,'g',4  ).arg( points[1].rx(),0,'g',4  ).arg( points[1].ry(),0,'g',4  );
        offsets = QString("dx = %1\n dy = %2").arg( points[1].rx() - points[0].rx(),0,'g',4).arg( points[1].ry() - points[0].ry(), 0, 'g', 4 );
    }
    else if(points.size() == 1)
    {
        coords = QString("[%1; %2]\n      ").arg( points[0].rx(),0,'g',4 ).arg( points[0].ry(),0,'g',4  );
    }

    emit setMarkerText(coords,offsets);
}
////----------------------------------------------------------------------------------------------------------------------------------
//void Itom1DQwtFigure::setMarkerCoordinates(const QVector<QPointF> pts)
//{
//    char buf[60] = {0};
//    if(pts.size() > 1)
//    {
//        sprintf(buf, " [%.4g; %.4g]\n [%.4g; %.4g]", pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
//    }
//
//    m_lblCoordinates->setText(buf);
//
//    if(pts.size() > 2)
//    {
//        sprintf(buf, " dx = %.4g\n dy = %.4g", pts[2].x(), pts[2].y());
//    }
//    m_CurCoordDelta->setText(buf);
//}
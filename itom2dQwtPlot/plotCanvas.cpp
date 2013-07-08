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
        m_pStackCutPicker(NULL),
        m_pStackCutMarker(NULL),
		m_dObjItem(NULL),
        m_rasterData(NULL),
		m_pData(m_pData),
        m_state(tIdle),
        m_curColorMapIndex(0),
        m_pValuePicker(NULL),
        m_dObjPtr(NULL)
{
    setMouseTracking(false);

	//this is the border between the canvas and the axes and the overall mainwindow
	setContentsMargins(5,5,5,5);
	
	//canvas() is the real plotting area, where the plot is printed (without axes...)
	canvas()->setFrameShadow(QFrame::Plain);
	canvas()->setFrameShape(QFrame::NoFrame);
	
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

	//prepare color bar
	QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
    rightAxis->setColorBarEnabled(true);
    rightAxis->setColorBarWidth(30);

    rightAxis->setColorMap(QwtInterval(0,1.0), new QwtLinearColorMap(Qt::black, Qt::white));
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
void PlotCanvas::refreshPlot(ito::DataObject *dObj, int plane /*= -1*/)
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
        QwtInterval ival;
        if(m_pData->m_valueScaleAuto)
        {
            ival = m_rasterData->interval(Qt::ZAxis);
            m_pData->m_valueMin = ival.minValue();
            m_pData->m_valueMax = ival.maxValue();
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

    if(colorMap != NULL)
    {
        if(m_rasterData)
        {
            QwtInterval interval = m_rasterData->interval(Qt::ZAxis);
            setAxisScale(QwtPlot::yRight, interval.minValue(), interval.maxValue());
            axisWidget(QwtPlot::yRight)->setColorMap(interval, colorBarMap);
        }
        else
        {
            axisWidget(QwtPlot::yRight)->setColorMap(QwtInterval(0,1.0), colorBarMap);
        }

        m_dObjItem->setColorMap(colorMap);
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::trackerAppended(const QPoint &pt)
{
    /*QVector<QPointF> pts;
    pts.resize(2);

    if (m_pContent && m_pContent->data())
    {
        pts[0].setY(invTransform(this->m_pContent->yAxis(), pt.y()));
        pts[0].setX(invTransform(this->m_pContent->xAxis(), pt.x()));

        pts[1] = pts[0];
    }
    m_lineCut.setSamples(pts);
    replot();

    ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);*/
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::trackerMoved(const QPoint &pt)
{
    /*QVector<QPointF> pts;
    pts.resize(2);

    pts[0] = m_lineCut.sample(0);
    if (m_pContent && m_pContent->data())
    {
        pts[1].setY(invTransform(this->m_pContent->yAxis(), pt.y()));
        pts[1].setX(invTransform(this->m_pContent->xAxis(), pt.x()));
    }

    if(m_stateMoveAligned)
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

    m_lineCut.setSamples(pts);

    replot();

    ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);*/
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::trackerAScanAppended(const QPoint &pt)
{
    //QVector<QPointF> pts;

    //QPointF scale;
    //pts.resize(1);

    //if (m_pContent && m_pContent->data())
    //{
    //    pts[0].setY(invTransform(this->m_pContent->yAxis(), pt.y()));
    //    pts[0].setX(invTransform(this->m_pContent->xAxis(), pt.x()));

    //    QwtInterval interv = m_pContent->data()->interval(Qt::ZAxis);
    //    //scale.setX(interv.minValue());
    //    //scale.setY(interv.maxValue());
    //}

    //m_pAScanMarker->setValue(pts[0]);
    //m_lineCut.setSamples(pts);

    //replot();
    //((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::trackerAScanMoved(const QPoint &pt)
{
    /*QVector<QPointF> pts;
    pts.resize(1);

    if (m_pContent && m_pContent->data())
    {
        pts[0].setY(invTransform(this->m_pContent->yAxis(), pt.y()));
        pts[0].setX(invTransform(this->m_pContent->xAxis(), pt.x()));
    }

    if(m_stateMoveAligned)
    {
        if(abs(pts[0].x() - m_pAScanMarker->xValue()) > abs(pts[0].y() - m_pAScanMarker->yValue()))
        {
            pts[0].setY(m_pAScanMarker->yValue());
        }
        else
        {
            pts[0].setX(m_pAScanMarker->xValue());
        }
    }

    m_pAScanMarker->setValue(pts[0]);
    m_lineCut.setSamples(pts);

    replot();
    ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);*/
}

//----------------------------------------------------------------------------------------------------------------------------------
//ito::RetVal Plot2DWidget::setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue)
//{
//    DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());
//    if(rasterData)
//    {
//        rasterData->setIntervalRange(axis, autoCalcLimits, minValue, maxValue);
//
//        replot();
//        return ito::retOk;
//    }
//    else
//    {
//        switch(axis)
//        {
//            case Qt::ZAxis:
//                m_startScaledZ = true;
//                m_startRangeZ = QPointF(minValue, maxValue);
//            break;
//            case Qt::YAxis:
//                m_startScaledY = true;
//                m_startRangeY = QPointF(minValue, maxValue);
//            break;
//            case Qt::XAxis:
//                m_startScaledX = true;
//                m_startRangeX = QPointF(minValue, maxValue);
//            break;
//        }
//    }
//    return ito::retError;
//}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::keyPressEvent ( QKeyEvent * event ) 
{

    //if(!m_pContent || !m_pContent->data())
    //    return;

    //if (!m_lineCut.isVisible())
    //    return;

    //switch(((const QKeyEvent *)event)->key())
    //{
    //    case Qt::Key_Up:
    //    {
    //        if (m_linePlotID == 0)
    //            return;
    //        QVector<QPointF> pts;
    //        DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());
    //        int dims = rasterData->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index
    //        int ySize = rasterData->getDataObject()->getSize(dims-2); // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index

    //        bool test;
    //        double upperBorder = rasterData->getDataObject()->getPixToPhys(dims-2, ySize - 1, test);
    //        double diff = rasterData->getDataObject()->getAxisScale(dims-2);
    // 
    //        if(!ito::dObjHelper::isFinite<double>(diff) || !ito::dObjHelper::isNotZero<double>(diff))
    //            diff = 1.0;

    //        if(m_pAScanPicker->isEnabled())   // doing the linecut
    //        {
    //            pts.resize(1);
    //            pts[0] = m_lineCut.sample(0);
    //            if (pts[0].y() < upperBorder) pts[0].setY(pts[0].y() + diff);
    //        }
    //        else
    //        {
    //            pts.resize(2);
    //            pts[0] = m_lineCut.sample(0);
    //            pts[1] = m_lineCut.sample(1);

    //            //if (pts[0].y() < upperBoarder) pts[0].setY(pts[0].y() + 1);
    //            //if (pts[1].y() < upperBoarder) pts[1].setY(pts[1].y() + 1);

    //            if (pts[0].y() < upperBorder) pts[0].setY(pts[0].y() + diff);
    //            if (pts[1].y() < upperBorder) pts[1].setY(pts[1].y() + diff);
    //        }

    //        m_lineCut.setSamples(pts);
    //        replot();
    //        ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);
    //    }
    //    return;

    //    case Qt::Key_Down:
    //    {
    //        if (m_linePlotID == 0)
    //            return;
    //        QVector<QPointF> pts;
    //        DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());

    //        int dims = rasterData->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index
    //                
    //        bool test;
    //        double lowerBorder = rasterData->getDataObject()->getPixToPhys(dims-2, 0, test);
    //        double diff = rasterData->getDataObject()->getAxisScale(dims-2);

    //        if(!ito::dObjHelper::isFinite<double>(diff) || !ito::dObjHelper::isNotZero<double>(diff))
    //            diff = 1.0;

    //        //if (pts[0].y() > 0) pts[0].setY(pts[0].y() - 1);
    //        //if (pts[1].y() > 0) pts[1].setY(pts[1].y() - 1);
    //        if(m_pAScanPicker->isEnabled())   // doing the linecut
    //        {
    //            pts.resize(1);
    //            pts[0] = m_lineCut.sample(0);
    //            if (pts[0].y() > lowerBorder) pts[0].setY(pts[0].y() - diff);
    //        }
    //        else
    //        {
    //            pts.resize(2);
    //            pts[0] = m_lineCut.sample(0);
    //            pts[1] = m_lineCut.sample(1);
    //            if (pts[0].y() > lowerBorder) pts[0].setY(pts[0].y() - diff);
    //            if (pts[1].y() > lowerBorder) pts[1].setY(pts[1].y() - diff);                    
    //        }

    //        m_lineCut.setSamples(pts);
    //        replot();
    //        ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);
    //    }
    //    return;

    //    case Qt::Key_Right:
    //    {
    //        if (m_linePlotID == 0)
    //            return;
    //        QVector<QPointF> pts;
    //        DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());
    //                
    //        int dims = rasterData->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index

    //        int xSize = rasterData->getDataObject()->getSize(dims-1);
    //        bool test;
    //        double rightBorder = rasterData->getDataObject()->getPixToPhys(dims-1, xSize - 1, test);
    //        double diff = rasterData->getDataObject()->getAxisScale(dims-1);

    //        if(!ito::dObjHelper::isFinite<double>(diff) || !ito::dObjHelper::isNotZero<double>(diff))
    //            diff = 1.0;

    //        //if (pts[0].x() < rasterData->getDataObject()->getSize(dims-1) - 1) pts[0].setX(pts[0].x() + 1);
    //        //if (pts[1].x() < rasterData->getDataObject()->getSize(dims-1) - 1) pts[1].setX(pts[1].x() + 1);

    //        if(m_pAScanPicker->isEnabled())   // doing the linecut
    //        {
    //            pts.resize(1);
    //            pts[0] = m_lineCut.sample(0);
    //            if (pts[0].x() < rightBorder) pts[0].setX(pts[0].x() + diff);
    //        }
    //        else
    //        {
    //            pts.resize(2);
    //            pts[0] = m_lineCut.sample(0);
    //            pts[1] = m_lineCut.sample(1);

    //            if (pts[0].x() < rightBorder) pts[0].setX(pts[0].x() + diff);
    //            if (pts[1].x() < rightBorder) pts[1].setX(pts[1].x() + diff);                    
    //        }

    //        m_lineCut.setSamples(pts);
    //        replot();
    //        ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);
    //    }
    //    return;

    //    case Qt::Key_Left:
    //    {
    //        if (m_linePlotID == 0)
    //            return;
    //        QVector<QPointF> pts;
    //        DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());

    //        bool test;

    //        int dims = rasterData->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index

    //        double leftBorder = rasterData->getDataObject()->getPixToPhys(dims-1, 0.0, test);
    //        double diff = rasterData->getDataObject()->getAxisScale(dims-1);

    //        if(!ito::dObjHelper::isFinite<double>(diff) || !ito::dObjHelper::isNotZero<double>(diff))
    //            diff = 1.0;

    //        if(m_pAScanPicker->isEnabled())   // doing the linecut
    //        {
    //            pts.resize(1);
    //            pts[0] = m_lineCut.sample(0);
    //            if (pts[0].x() > leftBorder) pts[0].setX(pts[0].x() - diff);
    //        }
    //        else
    //        {
    //            pts.resize(2);
    //            pts[0] = m_lineCut.sample(0);
    //            pts[1] = m_lineCut.sample(1);

    //            //if (pts[0].x() > 0) pts[0].setX(pts[0].x() - 1);
    //            //if (pts[1].x() > 0) pts[1].setX(pts[1].x() - 1);

    //            if (pts[0].x() > leftBorder) pts[0].setX(pts[0].x() - diff);
    //            if (pts[1].x() > leftBorder) pts[1].setX(pts[1].x() - diff);
    //        }
    //        m_lineCut.setSamples(pts);
    //        replot();
    //        ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);
    //    }
    //    return;

    //    // The following keys represent a direction, they are
    //    // organized on the keyboard.

    //    case Qt::Key_H:
    //    {
    //        if (m_linePlotID == 0 || m_pAScanPicker->isEnabled())
    //            return;
    //        QVector<QPointF> pts;
    //        pts.resize(2);
    //        DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());

    //        int dims = rasterData->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index
    //                
    //        int sizey = rasterData->getDataObject()->getSize(dims-2);
    //        int sizex = rasterData->getDataObject()->getSize(dims-1);
    //        //pts[0].setX(0);
    //        //pts[0].setY(sizey / 2);
    //        //pts[1].setX(sizex);
    //        //pts[1].setY(pts[0].y());
    //        bool test = true;
    //        double yCenter = rasterData->getDataObject()->getPixToPhys( dims-2, sizey / 2.0, test);
    //        double xMin = rasterData->getDataObject()->getPixToPhys( dims-1, 0, test);
    //        double xMax = rasterData->getDataObject()->getPixToPhys( dims-1, sizex, test);

    //        pts[0].setX(xMin);
    //        pts[0].setY(yCenter);
    //        pts[1].setX(xMax);
    //        pts[1].setY(yCenter);

    //        m_lineCut.setSamples(pts);
    //        replot();
    //        ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);
    //    }
    //    break;

    //    case Qt::Key_V:
    //    {
    //        if (m_linePlotID == 0 || m_pAScanPicker->isEnabled())
    //            return;
    //        QVector<QPointF> pts;
    //        pts.resize(2);
    //        DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());

    //        int dims = rasterData->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index
    //                
    //        int sizey = rasterData->getDataObject()->getSize(dims-2);
    //        int sizex = rasterData->getDataObject()->getSize(dims-1);
    //        //pts[0].setX(sizex / 2);
    //        //pts[0].setY(0);
    //        //pts[1].setX(pts[0].x());
    //        //pts[1].setY(sizey);

    //        bool test = true;
    //        double xCenter = rasterData->getDataObject()->getPixToPhys( dims-1, sizex / 2.0, test);
    //        double yMin = rasterData->getDataObject()->getPixToPhys( dims-2, 0, test);
    //        double yMax = rasterData->getDataObject()->getPixToPhys( dims-2, sizey, test);

    //        pts[0].setX(xCenter);
    //        pts[0].setY(yMin);
    //        pts[1].setX(xCenter);
    //        pts[1].setY(yMax);

    //        m_lineCut.setSamples(pts);
    //        replot();
    //        ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);
    //    }
    //    break;

    //    case Qt::Key_Control:
    //        m_stateMoveAligned = true;
    //        break;

    //    default:
    //    break;
    //}
    //return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::keyReleaseEvent ( QKeyEvent * event )
{
/*
    switch(((const QKeyEvent *)event)->key())
    {
        case Qt::Key_Control:
            m_stateMoveAligned = false;
            break;

        default:
        break;
    }
    return;*/
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
    setAxisScale( QwtPlot::yRight, m_pData->m_valueMin, m_pData->m_valueMax);
    
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
void PlotCanvas::setState( tState state)
{
    if (m_state != state)
    {
        if (m_pZoomer) m_pZoomer->setEnabled( state == tZoom );
        if (m_pPanner) m_pPanner->setEnabled( state == tPan );
        if (m_pValuePicker) m_pValuePicker->setEnabled( state == tValuePicker );
        if (m_pLineCutPicker) m_pLineCutPicker->setEnabled( state == tLineCut );
        if (m_pStackCutPicker) m_pStackCutPicker->setEnabled( state == tStackCut );
        if (m_pStackCutMarker) m_pStackCutMarker->setVisible( state == tStackCut );

        switch (state)
        {
        case tIdle:
            canvas()->setCursor( Qt::ArrowCursor );
            break;
        case tValuePicker:
            canvas()->setCursor( Qt::CrossCursor );
            break;
        default:
            canvas()->setCursor( Qt::ArrowCursor );
            break;
        }

        m_state = state;
    }
}
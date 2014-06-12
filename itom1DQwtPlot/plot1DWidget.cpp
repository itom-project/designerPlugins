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
#include <qwt_scale_widget.h>
#include <qwt_plot_magnifier.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_renderer.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_canvas.h>
#include <qwt_symbol.h>
#include <qwt_picker.h>
#include <qwt_picker_machine.h>
#include <qwt_scale_widget.h>

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
        m_multiLine(Auto),
        m_autoLineColIndex(0),
        m_lineCol(0),
        m_lineStyle(1),
        m_pParent(parent),
        m_actPickerIdx(-1),
        m_cmplxState(false),
        m_pData(data),
        m_activeDrawItem(1),
        m_pRescaler(NULL),
        m_ignoreNextMouseEvent(false)
{
    this->setMouseTracking(false);

    //this is the border between the canvas and the axes and the overall mainwindow
    setContentsMargins(5,5,5,5);
    
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

    m_drawedIemsIndexes.clear();
    m_drawedIemsIndexes.reserve(10);

    setState((Plot1DWidget::tState)m_pData->m_state);
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

    foreach (Marker m, m_markers)
    {
        m.item->detach();
        delete m.item;
    }
    m_markers.clear();

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

    if (m_pMultiPointPicker != NULL) m_pMultiPointPicker->deleteLater();
    m_pMultiPointPicker = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::init()
{
    QPen rubberBandPen = apiGetFigureSetting(parent(), "zoomRubberBandPen", QPen(QBrush(Qt::red),2,Qt::DashLine),NULL).value<QPen>();
    QPen trackerPen = apiGetFigureSetting(parent(), "trackerPen", QPen(QBrush(Qt::red),2),NULL).value<QPen>();
    QFont trackerFont = apiGetFigureSetting(parent(), "trackerFont", QFont("Verdana",10),NULL).value<QFont>();
    QBrush trackerBg = apiGetFigureSetting(parent(), "trackerBackground", QBrush(QColor(255,255,255,155), Qt::SolidPattern),NULL).value<QBrush>();

    QFont titleFont = apiGetFigureSetting(parent(), "titleFont", QFont("Helvetica",12),NULL).value<QFont>();
    QFont labelFont = apiGetFigureSetting(parent(), "labelFont", QFont("Helvetica",12),NULL).value<QFont>();
    QFont axisFont = apiGetFigureSetting(parent(), "axisFont", QFont("Helvetica",10),NULL).value<QFont>();
    
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

////----------------------------------------------------------------------------------------------------------------------------------
//void Plot1DWidget::replot()
//{
//    DataObjectSeriesData *data = NULL;
//    if (m_plotCurveItems.size() > 0)
//    {
//        /*if (m_pCurserEnable && m_pContent[0])
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
//    bool gotNewObject = false;
    //QString valueLabel, axisLabel, title;

    if (dataObj)
    {
//        gotNewObject = true;
        int dims = dataObj->getDims();
        int width = dims > 0 ? dataObj->getSize(dims - 1) : 0;
        int height = dims > 1 ? dataObj->getSize(dims - 2) : 1;

        if (dataObj->getType() == ito::tComplex128 || dataObj->getType() == ito::tComplex64)
        {
            if (!m_cmplxState) ((Itom1DQwtPlot*)m_pParent)->enableComplexGUI(true);
            m_cmplxState = true;                
        }
        else
        {
            if (m_cmplxState) ((Itom1DQwtPlot*)m_pParent)->enableComplexGUI(false);
            m_cmplxState = false;                
        }

        if (bounds.size() == 0)
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
                default:
                {
                    if(width == 1 && height == 1 && dims < 3)
                    {
                        m_multiLine = Auto;
                        numCurves = height;
                    }
                    else if (width >= height)
                    {
                        numCurves = height;
                        m_multiLine = MultiRows;
                    }
                    else
                    {
                        numCurves = width;
                        m_multiLine = MultiCols;
                    }
                }
            }
        }
        else //if there are boundaries, only plot one curve from bounds[0] to bounds[1]
        {
            numCurves = 1;
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

        if (bounds.size() == 0)
        {
            QVector<QPointF> pts(2);

            switch(m_multiLine)
            {
            case FirstCol:
            case MultiCols:
                pts[0].setY(dataObj->getPixToPhys(dims-2, 0, _unused));
                pts[1].setY(dataObj->getPixToPhys(dims-2, height-1, _unused)); 

                for (int n = 0; n < numCurves; n++)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[n]->data());
                    pts[0].setX(dataObj->getPixToPhys(dims-1, n, _unused));
                    pts[1].setX(dataObj->getPixToPhys(dims-1, n, _unused));
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

                if (numCurves > 0)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
                    m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel();
                    m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel();
                }
                break;

            case Auto:
            case FirstRow:
            case MultiRows:
                pts[0].setX(dataObj->getPixToPhys(dims-1, 0, _unused));
                pts[1].setX(dataObj->getPixToPhys(dims-1, width-1, _unused));

                for (int n = 0; n < numCurves; n++)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[n]->data());
                    pts[0].setY(dataObj->getPixToPhys(dims-2, n, _unused));
                    pts[1].setY(dataObj->getPixToPhys(dims-2, n, _unused));
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

                if (numCurves > 0)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
                    m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel();
                    m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel();
                }
                break;
            }
        }
        else if (bounds.size() == 2) //boundaries given ->line plot
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
        else if (bounds.size() == 1) //point in third dimension
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
        m_pData->m_titleDObj = valid? tag.getVal_ToString().data() : "";

    } 

    updateLabels();

    if (seriesData)
    {
        QByteArray hash = seriesData->getHash();

        if (hash != m_hash)
        {
            updateMarkerPosition(true);

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
            updateMarkerPosition(true);


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
            updateMarkerPosition(true,false);

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
    Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
    Marker *m;
    int curves = m_plotCurveItems.size();

    if (m_pData->m_state == statePicker)
    {
        switch(event->key())
        {
        case Qt::Key_Left:
            for (int i = 0 ; i < m_markers.size() ; i++)
            {
                m = &(m_markers[i]);
                if (m->active)
                {
                    stickMarkerToXPx(m, m->item->xValue(), -1);
                }
            }
            break;
        case Qt::Key_Right:
            for (int i = 0 ; i < m_markers.size() ; i++)
            {
                m = &(m_markers[i]);
                if (m->active)
                {
                     stickMarkerToXPx(m, m->item->xValue(), 1);
                }
            }
            break;
        case Qt::Key_Up:
            for (int i = 0 ; i < m_markers.size() ; i++)
            {
                m = &(m_markers[i]);
                if (m->active)
                {
                    m->curveIdx++;
                    if (m->curveIdx >= curves) m->curveIdx = 0;
                    stickMarkerToXPx(m, m->item->xValue(), 0);
                }
            }
            break;
        case Qt::Key_Down:
            for (int i = 0 ; i < m_markers.size() ; i++)
            {
                m = &(m_markers[i]);
                if (m->active)
                {
                    m->curveIdx--;
                    if (m->curveIdx <= 0) m->curveIdx = curves-1;
                    stickMarkerToXPx(m, m->item->xValue(), 0);
                }
            }
            break;
        case Qt::Key_Delete:
        {
            QList<Marker>::iterator it = m_markers.begin();

            while (it != m_markers.end())
            {
                if (it->active)
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
    else if(event->matches(QKeySequence::Copy))
    {
        p->copyToClipBoard();
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

    if (m_Curser[0] > m_Curser[1])
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
        bool closeToMarker = false;

        if (event->button() == Qt::LeftButton)
        {
            for (int i = 0 ; i < m_markers.size() ; i++)
            {
                if (abs(transform(xBottom, m_markers[i].item->xValue()) - xPx) < 20 && abs(transform(yLeft, m_markers[i].item->yValue()) - yPx) < 20)
                {
                    closeToMarker = true;
                    m_markers[i].active = true;
                    
                }
                else if ((event->modifiers() & Qt::ControlModifier) == false && m_markers[i].active)
                {
                    m_markers[i].active = false;
                    //m_markers[i].item->setSymbol(new QwtSymbol(QwtSymbol::Diamond, m_markers[i].color, QPen(m_markers[i].color,1), QSize(6,6)));
                }
            }

            if (!closeToMarker && m_plotCurveItems.size() > 0)
            {
                Marker marker;
                marker.item = new QwtPlotMarker();
                marker.item->attach(this);
                marker.active = true;
                //marker.color = Qt::darkGreen;
                //marker.item->setSymbol(new QwtSymbol(QwtSymbol::Diamond,QBrush(Qt::white), QPen(marker.color,1),  QSize(8,8)));
                
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
            for (int i = 0 ; i < m_markers.size() ; i++)
            {
                if (m_markers[i].active == true)
                {
                    stickMarkerToXPx(&m_markers[i], xScale, 0);
                }
            }
            updateMarkerPosition(false,false);

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
//        bool closeToMarker = false;

        if (event->button() == Qt::LeftButton)
        {
            for (int i = 0 ; i < m_markers.size() ; i++)
            {
                if (m_markers[i].active == true)
                {
                    stickMarkerToXPx(&m_markers[i], xScale, 0);
                }
            }

            updateMarkerPosition(false,false);

            replot();
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setMainMarkersToIndex(int idx1, int idx2, int curveIdx)
{
    while (m_markers.size() < 2)
    {
        //prepend
        Marker marker;
        marker.item = new QwtPlotMarker();
        marker.item->attach(this);
        marker.active = false;
                
        marker.curveIdx = curveIdx;
        marker.item->setVisible(true);
                
        m_markers.prepend(marker);
    }

    for (int i = 0; i < m_markers.size() ; ++i)
    {
        if (i == 0)
        {
            m_markers[0].active = true;
            stickMarkerToSampleIdx(&(m_markers[0]), idx1, curveIdx, 0);
        }
        else if (i == 1)
        {
            m_markers[1].active = false;
            stickMarkerToSampleIdx(&(m_markers[1]), idx2, curveIdx, 0);
        }
        else
        {
            m_markers[i].active = false;
        }
    }

    updateMarkerPosition(false,false);

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::stickMarkerToXPx(Marker *m, double xScaleStart, int dir) //dir: 0: this point, -1: next valid to the left or this if not possible, 1: next valid to the right or this if not possible
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
    if (dir == -1)
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
void Plot1DWidget::stickMarkerToSampleIdx(Marker *m, int idx, int curveIdx, int dir)
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
    if (dir == -1)
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

        foreach(QwtPlotCurve *curve, m_plotCurveItems)
        {
            setAxisAutoScale(curve->xAxis(),true);
            setAxisAutoScale(curve->yAxis(),true);
        }
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
void Plot1DWidget::updateMarkerPosition(bool updatePositions, bool clear/* = false*/)
{
    if (clear)
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

    for (int i = 0 ; i < m_markers.size() ; i++)
    {
        m = &(m_markers[i]);
        if (updatePositions)
        {
            stickMarkerToXPx(m, std::numeric_limits<double>::quiet_NaN() ,0);
        }

        if (m->active)
        {
            m_markers[i].item->setSymbol(new QwtSymbol(QwtSymbol::Diamond, Qt::white, QPen(colors[cur],2), QSize(8,8)));
        }
        else
        {
            m_markers[i].item->setSymbol(new QwtSymbol(QwtSymbol::Diamond, colors[cur], QPen(colors[cur],2), QSize(6,6)));
        }

        if (cur < 2) cur++;
        points << QPointF(m_markers[i].item->xValue(), m_markers[i].item->yValue());       
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

    emit setMarkerText(coords,offsets);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::configRescaler(void)
{
    if (m_pData->m_keepAspect)
    {
        //int height = plotLayout()->canvasRect().height();
        //int width = plotLayout()->canvasRect().width();

        int refAxis = plotLayout()->canvasRect().width() < plotLayout()->canvasRect().height() ? QwtPlot::xBottom : QwtPlot::yLeft;
        
        if (m_pRescaler == NULL)
        {
            QwtInterval curXInterVal = axisInterval(QwtPlot::xBottom);
            QwtInterval curYInterVal = axisInterval(QwtPlot::yLeft);

            m_pRescaler = new QwtPlotRescaler(canvas(), refAxis , QwtPlotRescaler::Fitting);
            m_pRescaler->setIntervalHint(QwtPlot::xBottom, curXInterVal);
            m_pRescaler->setIntervalHint(QwtPlot::yLeft, curYInterVal);
            m_pRescaler->setAspectRatio(1.0);
            m_pRescaler->setExpandingDirection(QwtPlot::xBottom, QwtPlotRescaler::ExpandUp);
            m_pRescaler->setExpandingDirection(QwtPlot::yLeft, QwtPlotRescaler::ExpandBoth);
            //m_pRescaler->setExpandingDirection(QwtPlot::yRight, QwtPlotRescaler::ExpandBoth);
        }
        else
        {

            m_pRescaler->setReferenceAxis(refAxis);
        }
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
//            QwtPlotMarker *marker = NULL;
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

////----------------------------------------------------------------------------------------------------------------------------------
//void Itom1DQwtFigure::setMarkerCoordinates(const QVector<QPointF> pts)
//{
//    char buf[60] = {0};
//    if (pts.size() > 1)
//    {
//        sprintf(buf, " [%.4g; %.4g]\n [%.4g; %.4g]", pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
//    }
//
//    m_lblCoordinates->setText(buf);
//
//    if (pts.size() > 2)
//    {
//        sprintf(buf, " dx = %.4g\n dy = %.4g", pts[2].x(), pts[2].y());
//    }
//    m_CurCoordDelta->setText(buf);
//}
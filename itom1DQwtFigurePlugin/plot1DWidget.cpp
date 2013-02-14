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
//#include "common/apiFunctionsGraphInc.h"

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

//namespace ito {
//    extern void **ITOM_API_FUNCS_GRAPH;
//}

//----------------------------------------------------------------------------------------------------------------------------------
Plot1DWidget::Plot1DWidget(QMenu *contextMenu, QWidget * parent) :
        QwtPlot(parent),
        m_contextMenu(contextMenu),
        m_startScaledX(false),
        m_startScaledY(false),
        m_xDirect(false),
        m_yDirect(false),
        m_numElements(0),
        m_multiLine(0),
        m_autoLineColIndex(0),
        m_lineCol(0),
        m_lineStyle(1),
        m_pParent(parent),
        m_pCurser1(NULL),
        m_pCurser2(NULL),
        m_pCurserEnable(false),
        m_cmplxState(false)
{
    this->setMouseTracking(false); //(mouse tracking is controled by action in WinMatplotlib)

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

    m_pContent = static_cast<QwtPlotCurve**>(calloc(1, sizeof(QwtPlotCurve*)));
    m_pContent[0] = new QwtPlotCurveDataObject("itomQwt1DPlot");
    m_numElements = 1;
    m_pContent[0]->setData(NULL);

    m_pContent[0]->attach(this);

    m_pZoomer = new QwtPlotZoomer(QwtPlot::xBottom, QwtPlot::yLeft, canvas());
    m_pZoomer->setEnabled(false);
    m_pZoomer->setRubberBandPen(QPen(QBrush(Qt::red),3,Qt::DashLine));
    m_pZoomer->setTrackerMode(QwtPicker::AlwaysOn);
    m_pZoomer->setTrackerFont(QFont("Verdana",10));
    m_pZoomer->setTrackerPen(QPen(QBrush(Qt::green),2));

    m_pPanner = new QwtPlotPanner(canvas());
    m_pPanner->setAxisEnabled(QwtPlot::yRight,false);
    m_pPanner->setCursor(Qt::SizeAllCursor);
    m_pPanner->setEnabled(false);

    // This will be point tracker!
    m_pCurser1 = new QwtPlotMarker();
    m_pCurser1->setSymbol(new QwtSymbol(QwtSymbol::Diamond,QBrush(Qt::red), QPen(QBrush(Qt::red),1),  QSize(6,6) ));
    m_pCurser1->attach(this);
    m_pCurser1->setVisible(false);
    m_pCurser2 = new QwtPlotMarker();
    m_pCurser2->setSymbol(new QwtSymbol(QwtSymbol::Diamond,QBrush(Qt::darkGreen), QPen(QBrush(Qt::darkGreen),1),  QSize(6,6) ));
    m_pCurser2->attach(this);
    m_pCurser2->setVisible(false);


    m_pValuePicker = new ValuePicker1D(QwtPlot::xBottom, QwtPlot::yLeft, canvas());
    m_pValuePicker->setEnabled(false);
    m_pValuePicker->setTrackerMode(QwtPicker::AlwaysOn);
    m_pValuePicker->setTrackerFont(QFont("Verdana",10));
    m_pValuePicker->setTrackerPen(QPen(QBrush(Qt::red),2));
    m_pValuePicker->setBackgroundFillBrush( QBrush(QColor(255,255,255,155), Qt::SolidPattern) );


    QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
    rightAxis->setColorBarEnabled(true);
    rightAxis->setColorBarWidth(30);

    rightAxis->setColorMap(QwtInterval(0,1.0), new QwtLinearColorMap(Qt::black, Qt::white));
    rightAxis->setFont(QFont("Comic Sans",8,1,true));

    rightAxis->setMargin(20); //margin to right border of window
    rightAxis->scaleDraw()->setLength(20);
    rightAxis->scaleDraw()->enableComponent(QwtAbstractScaleDraw::Backbone,false);

    setAxisScale(QwtPlot::yRight, 0, 1.0 );
    enableAxis(QwtPlot::yRight,false);

    ((QwtPlotCurve**)m_pContent)[0]->attach(this);

    m_Curser[0] = 0;
    m_Curser[1] = 0;
    m_curserFirstActive = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
Plot1DWidget::~Plot1DWidget()
{
    if(m_pContent)
    {
        for(int i=m_numElements-1;i>=0;i--)
        {
            if(m_pContent[i])
            {
                m_pContent[i]->detach();
                delete(m_pContent[i]);
                m_pContent[i] = NULL;
            }
        }

        free(m_pContent);
		m_pContent = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::replot()
{
    DataObjectSeriesData *data = NULL;
    if (m_pContent)
    {
        if(m_pCurserEnable && m_pContent[0])
        {
            QVector<QPointF> pts(3);

            data = (DataObjectSeriesData *)m_pContent[0]->data();
            data->setRasterObj();

            int x1 = ((DataObjectSeriesData*)(m_pContent[0]->data()))->size()-1;

            m_Curser[0] = m_Curser[0] > x1 ? x1: m_Curser[0];
            m_Curser[1] = m_Curser[1] > x1 ? x1: m_Curser[1];

            pts[0] = data->sample(m_Curser[0]);
            pts[1] = data->sample(m_Curser[1]);
            pts[2] = pts[1] - pts[0];

            m_pCurser1->setValue(pts[0]);
            m_pCurser2->setValue(pts[1]);
            data->releaseRasterObj();
            
            ((itom1DQwtFigure*) m_pParent)->setMarkerCoordinates(pts);
        }

        for (int n = 0; n < m_numElements; n++)
        {
            if (m_pContent[n])
            {
                data = (DataObjectSeriesData *)m_pContent[n]->data();

                if (data && data->isDobjInit())
                {
                    data->setRasterObj();
                    QwtPlot::replot();
                    data->releaseRasterObj();
                }
            }
        }


    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::refreshPlot(QSharedPointer<ito::DataObject> dataObj, QVector<QPointF> pts)
{
    DataObjectSeriesData* seriesData = NULL;
    int colorIndex;

    if (dataObj.data() != NULL)
    {

        if(dataObj->getType() == ito::tComplex128 || dataObj->getType() == ito::tComplex64)
        {
            if(!m_cmplxState) ((itom1DQwtFigure*)m_pParent)->enableComplexGUI(true);
            m_cmplxState = true;                
        }
        else
        {
            if(m_cmplxState) ((itom1DQwtFigure*)m_pParent)->enableComplexGUI(false);
            m_cmplxState = false;                
        }

        if (m_pContent)
        {
            if (dataObj->getSize(dataObj->getDims() - 1) != 1 && dataObj->getSize(dataObj->getDims() - 2) != 1 && m_multiLine)
            {
                if (m_multiLine == 1)
                {
                    int numLines = dataObj->getSize(dataObj->getDims() - 2);
                    if (m_numElements != numLines)
                    {
                        m_pContent = static_cast<QwtPlotCurve**>(realloc(m_pContent, numLines * sizeof(QwtPlotCurve*)));
                        for (int m = numLines; m < m_numElements; m++)
                        {
                            m_pContent[m]->detach();
                            delete m_pContent[m];
                        }
                        for (int m = m_numElements; m < numLines; m++)
                        {
                            m_pContent[m] = new QwtPlotCurveDataObject("");
                            m_pContent[m]->setData(NULL);
                            m_pContent[m]->attach(this);
                            QPen plotPen;
                            colorIndex = (m_autoLineColIndex++) % m_colorList.size();
                            plotPen.setColor(m_colorList[colorIndex]);
                            plotPen.setStyle((Qt::PenStyle)m_lineStyle);
                            m_pContent[m]->setPen(plotPen);
                        }
                        m_numElements = numLines;
                    }

                    QVector<QPointF> pts;
                    pts.resize(2);
                    pts[0].setX(0);
                    pts[1].setX(dataObj->getSize(dataObj->getDims() - 1));
                    if ((pts[1].x() - pts[0].x()) == 0)
                        return; //cannot plot line with 0 zero points -> end

                    for (int n = 0; n < numLines; n++)
                    {
                        seriesData = static_cast<DataObjectSeriesData*>(m_pContent[n]->data());
                        if (seriesData && seriesData->isDobjInit())
                        {
                            pts[0].setY(n);
                            pts[1].setY(n);
                            seriesData->updateDataObject(dataObj, pts);
                        }
                        else
                        {
                            pts[0].setY(n);
                            pts[1].setY(n);
                            m_pContent[n]->setData(new DataObjectSeriesData(dataObj, pts, 0));
                        }
                    }
                }
                else
                {
                    int numLines = dataObj->getSize(dataObj->getDims() - 1);
                    if (m_numElements != numLines)
                    {
                        m_pContent = static_cast<QwtPlotCurve**>(realloc(m_pContent, numLines * sizeof(QwtPlotCurve*)));
                        for (int m = numLines; m < m_numElements; m++)
                        {
                            m_pContent[m]->detach();
                            delete m_pContent[m];
                        }
                        for (int m = m_numElements; m < numLines; m++)
                        {
                            m_pContent[m] = new QwtPlotCurveDataObject("");
                            m_pContent[m]->setData(NULL);
                            m_pContent[m]->attach(this);
                        }
                        m_numElements = numLines;
                    }

                    QVector<QPointF> pts;
                    pts.resize(2);
                    pts[0].setY(0);
                    pts[1].setY(dataObj->getSize(dataObj->getDims() - 2));
                    if ((pts[0].y() - pts[1].y()) == 0)
                        return; //cannot plot line with 0 zero points -> end

                    for (int n = 0; n < numLines; n++)
                    {
                        seriesData = static_cast<DataObjectSeriesData*>(m_pContent[n]->data());
                        if (seriesData && seriesData->isDobjInit())
                        {
                            pts[0].setX(n);
                            pts[1].setX(n);
                            seriesData->updateDataObject(dataObj, pts);
                        }
                        else
                        {
                            pts[0].setX(n);
                            pts[1].setX(n);
                            m_pContent[n]->setData(new DataObjectSeriesData(dataObj, pts, 0));
                        }
                    }
                }
            }
            else
            {
                if (m_pContent[0])
                    seriesData = static_cast<DataObjectSeriesData*>(m_pContent[0]->data());

                if (seriesData && seriesData->isDobjInit())
                {
                    int startPoint = 0;
                    if (dataObj->getSize(dataObj->getDims() - 1) == 1)
                    {
                        seriesData->updateDataObject(dataObj, startPoint, dataObj->getDims() - 2, dataObj->getSize(dataObj->getDims() - 2));
                    }
                    else  if (dataObj->getSize(dataObj->getDims() - 2) == 1)
                    {
                        seriesData->updateDataObject(dataObj, startPoint, dataObj->getDims() - 1, dataObj->getSize(dataObj->getDims() - 1));
                    }
                    else if (pts.size() >= 2)
                    {
                        seriesData->updateDataObject(dataObj, pts);
                    }
                    else if (pts.size() == 1)
                    {
                        seriesData->updateDataObject(dataObj, pts);
                    }
                    else
                        return;
                }
                else
                {
                    int startPoint = 0;
                    if (dataObj->getSize(dataObj->getDims() - 1) == 1 && (dataObj->getDims() > 1))
                    {
                        if (m_numElements != 1)
                        {
                            for (int m = 1; m < m_numElements; m++)
                                delete m_pContent[m];
                            m_pContent = static_cast<QwtPlotCurve**>(realloc(m_pContent, sizeof(QwtPlotCurve*)));
                            m_numElements = 1;
                        }
                        m_pContent[0]->setData(new DataObjectSeriesData(dataObj, startPoint, dataObj->getDims() - 2, dataObj->getSize(dataObj->getDims() - 2), 1));


                        bool test;
                        QString qtBuf("");
                        std::string tDescription = dataObj->getAxisDescription(dataObj->getDims() - 2, test, true);
                        std::string tUnit = dataObj->getAxisUnit(dataObj->getDims() - 2, test, true);

                        if(!tDescription.empty())
                        {
                            qtBuf.append(tDescription.data());
                        }
                        if (!tUnit.empty())
                        {
                            if(qtBuf.size()) qtBuf.append(" in ");
                            qtBuf.append(tUnit.data());
                        }
                        if(qtBuf.size()) setAxisTitle(QwtPlot::xBottom, qtBuf);

                    }
                    else if (dataObj->getSize(dataObj->getDims() - 2) == 1)
                    {
                        if (m_numElements != 1)
                        {
                            for (int m = 1; m < m_numElements; m++)
                                delete m_pContent[m];
                            m_pContent = static_cast<QwtPlotCurve**>(realloc(m_pContent, sizeof(QwtPlotCurve*)));
                            m_numElements = 1;
                        }
                        m_pContent[0]->setData(new DataObjectSeriesData(dataObj, startPoint, dataObj->getDims() - 1, dataObj->getSize(dataObj->getDims() - 1), 1));

                        bool test;
                        QString qtBuf("");
                        std::string tDescription = dataObj->getAxisDescription(dataObj->getDims() - 1, test, true);
                        std::string tUnit = dataObj->getAxisUnit(dataObj->getDims() - 1, test, true);

                        if(!tDescription.empty())
                        {
                            qtBuf.append(tDescription.data());
                        }
                        if (!tUnit.empty())
                        {
                            if(qtBuf.size()) qtBuf.append(" in ");
                            qtBuf.append(tUnit.data());
                        }
                        if(qtBuf.size()) setAxisTitle(QwtPlot::xBottom, qtBuf);
                    }
                    else if (pts.size() == 2)
                    {
                        if (m_numElements != 1)
                        {
                            m_pContent = static_cast<QwtPlotCurve**>(realloc(m_pContent, sizeof(QwtPlotCurve*)));
                            m_numElements = 1;
                        }
                        m_pContent[0]->setData(new DataObjectSeriesData(dataObj, pts, 0));

                    }
                    else if (pts.size() == 1)
                    {
                        // This is for the depth cut!!!
                        if (m_numElements != 1)
                        {
                            m_pContent = static_cast<QwtPlotCurve**>(realloc(m_pContent, sizeof(QwtPlotCurve*)));
                            m_numElements = 1;
                        }
                        m_pContent[0]->setData(new DataObjectSeriesData(dataObj, pts, 0));
                    }
                }
            }

            if(dataObj)
            {
                bool test;

                // Copy Titel and axis scale to widget
                QString qtBuf("");
                std::string tDescription(dataObj->getValueDescription());
                std::string tUnit(dataObj->getValueUnit());

                int objDims = dataObj->getDims();

                if(!tDescription.empty())
                {
                    qtBuf.append(tDescription.data());
                }
                if (!tUnit.empty())
                {
                    if(qtBuf.size()) qtBuf.append(" in ");
                    qtBuf.append(tUnit.data());
                }
                if(qtBuf.size()) setAxisTitle(QwtPlot::yLeft, qtBuf);

                std::string tTitle(dataObj->getTag("title", test).getVal_ToString());
                qtBuf.clear();
                if(test)
                {
                    qtBuf.append(tTitle.data());
                }
                setTitle(qtBuf);


                if (dataObj->getSize(dataObj->getDims() - 1) == 1 && (dataObj->getDims() > 1))
                {
                    qtBuf.clear();
                    tDescription = dataObj->getAxisDescription(dataObj->getDims() - 2, test, true);
                    tUnit = dataObj->getAxisUnit(dataObj->getDims() - 2, test, true);

                    if(!tDescription.empty())
                    {
                        qtBuf.append(tDescription.data());
                    }
                    if (!tUnit.empty())
                    {
                        if(qtBuf.size()) qtBuf.append(" in ");
                        qtBuf.append(tUnit.data());
                    }
                    if(qtBuf.size()) setAxisTitle(QwtPlot::xBottom, qtBuf);

                }
                else if (dataObj->getSize(dataObj->getDims() - 2) == 1)
                {
                    qtBuf.clear();

                    tDescription = dataObj->getAxisDescription(dataObj->getDims() - 1, test, true);
                    tUnit = dataObj->getAxisUnit(dataObj->getDims() - 1, test, true);

                    if(!tDescription.empty())
                    {
                        qtBuf.append(tDescription.data());
                    }
                    if (!tUnit.empty())
                    {
                        if(qtBuf.size()) qtBuf.append(" in ");
                        qtBuf.append(tUnit.data());
                    }
                    if(qtBuf.size()) setAxisTitle(QwtPlot::xBottom, qtBuf);
                }
                else if (pts.size() == 2)
                {
                    bool xdirect = false;
                    bool ydirect = false;

                    if(fabs((double)pts[1].x() - (double)pts[0].x()) > std::numeric_limits<double>::epsilon())
                    {
                       xdirect = true;
                    }

                    if(fabs((double)pts[1].y() - (double)pts[0].y()) > std::numeric_limits<double>::epsilon())
                    {
                       ydirect = true;
                    }

                    if((xdirect != m_xDirect) || (ydirect != m_yDirect))   // Check if the oriantation of the plot has changed (x || y || xy)
                    {
                        qtBuf.clear();
                        m_xDirect = xdirect;
                        m_yDirect = ydirect;
                        if(xdirect)
                        {
                           tUnit = dataObj->getAxisUnit(objDims - 1, test, true);
                           tDescription = dataObj->getAxisDescription(objDims - 1, test, true);
                            if(tDescription.size())
                            {
                                qtBuf.append(tDescription.data());
                            }
                            else
                            {
                                qtBuf.append("x-Axis");
                            }
                            if(tUnit.size())
                            {
                                qtBuf.append(" in ");
                                qtBuf.append(tUnit.data());
                            }
                        }
                        if (ydirect)
                        {
                            if(qtBuf.size()) qtBuf.append(" & ");
                            tUnit.clear();
                            tDescription.clear();
                            tUnit = dataObj->getAxisUnit(objDims - 2, test, true);
                            tDescription = dataObj->getAxisDescription(objDims - 1, test, true);
                            if(tDescription.size())
                            {
                                qtBuf.append(tDescription.data());
                            }
                            else
                            {
                                qtBuf.append("y-Axis");
                            }
                            if(tUnit.size())
                            {
                                qtBuf.append(" in ");
                                qtBuf.append(tUnit.data());
                            }
                        }
                        if(qtBuf.size()) setAxisTitle(QwtPlot::xBottom, qtBuf);
                    }
                }
                else if (pts.size() == 1 && (objDims > 2))
                {

                    qtBuf.clear();

                    tUnit = dataObj->getAxisUnit(objDims - 3, test, true);
                    tDescription = dataObj->getAxisDescription(objDims - 3, test, true);
                    if(tDescription.size())
                    {
                        qtBuf.append(tDescription.data());
                    }
                    else
                    {
                        qtBuf.append("z-Direction");
                    }
                    if(tUnit.size())
                    {
                        qtBuf.append(" in ");
                        qtBuf.append(tUnit.data());
                    }

                    if(qtBuf.size()) setAxisTitle(QwtPlot::xBottom, qtBuf);

                }    
            }
        }
    }
    else if(seriesData)
    {
        for (int n = 0; n < m_numElements; n++)
        {
            seriesData = static_cast<DataObjectSeriesData*>(m_pContent[n]->data());
            seriesData->updateDataObject(QSharedPointer<ito::DataObject>());
        }

    }
    else
    {

    }

    if(m_startScaledY && seriesData)
    {
        seriesData->setIntervalRange(Qt::YAxis, false, m_startRangeY.x(), m_startRangeY.y());
        m_startScaledY = false;
    }
    if(m_startScaledX && seriesData)
    {
        seriesData->setIntervalRange(Qt::XAxis, false, m_startRangeX.x(), m_startRangeX.y());
        m_startScaledY = false;
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::keyPressEvent ( QKeyEvent * event )
{
   // if (!hasFocus())
   //     return;

    if(!m_pCurserEnable && !m_pContent && !m_pContent[0])
    {
        return;      
    }

    int x1 = ((DataObjectSeriesData*)(m_pContent[0]->data()))->size()-1;

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
    }

    event->accept();
    replot();
}


//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mouseReleaseEvent ( QMouseEvent * event )
{
    Qt::MouseButton btn = event->button();
    Qt::MouseButtons btns = event->buttons();
    int button = 0;

    QPointF scenePos;
    
    switch(btn)
    {
        case Qt::LeftButton: button = 1; 
            if(m_pCurserEnable && m_pContent && m_pContent[0])
            {
                int xpos = m_pValuePicker->trackerPosition().x();
                double d_xpos = invTransform(this->m_pContent[0]->xAxis(), xpos);
                xpos = ((DataObjectSeriesData*)(m_pContent[0]->data()))->getPosToPix(d_xpos);

                if(m_curserFirstActive && xpos > m_Curser[1])
                {
                    m_Curser[0] = m_Curser[1];
                    m_Curser[1] = xpos;
                }
                else if(!m_curserFirstActive && xpos < m_Curser[0])
                {
                    m_Curser[1] = m_Curser[0];
                    m_Curser[0] = xpos;                        
                }
                else if(m_curserFirstActive)
                {
                    m_Curser[0] = xpos;
                }
                else
                {
                    m_Curser[1] = xpos;
                }
                replot();
            }
                
            break;
        case Qt::RightButton: button = 3; break;
        case Qt::MiddleButton: button = 2; break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::contextMenuEvent(QContextMenuEvent * event)
{
    if(m_showContextMenu)
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
    DataObjectSeriesData* seriesData = static_cast<DataObjectSeriesData*>(m_pContent[0]->data());
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
        m_pPanner->setEnabled(false);

        DataObjectSeriesData *data = NULL;
        if (m_pContent)
        {
            for (int n = 0; n < m_numElements; n++)
            {
                if (m_pContent[n])
                {
                    data = (DataObjectSeriesData *)m_pContent[n]->data();
                    m_pZoomer->setZoomBase(data->boundingRect());
                }
            }
        }

        m_pZoomer->setEnabled(true);
    }
    else
    {
        m_pZoomer->setEnabled(false);
        if (m_pContent)
        {
            for (int n = 0; n < m_numElements; n++)
            {
                if (m_pContent[n])
                {
                    setAxisAutoScale(m_pContent[n]->xAxis(),true);
                    setAxisAutoScale(m_pContent[n]->yAxis(),true);
                }
            }
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setPickerEnable(const bool checked)
{
    if(checked)
    {      
        m_pValuePicker->setEnabled(true);
        m_pCurser1->setVisible(true);
        m_pCurser2->setVisible(true);
        m_pCurserEnable = true;
        //m_pValuePicker->setVisible(true);
    }
    else
    {
        m_pValuePicker->setEnabled(false);
        m_pCurser1->setVisible(false);
        m_pCurser2->setVisible(false);
        m_pCurserEnable = false;
        //m_pValuePicker->setVisible(false);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------

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

#include "itom2DQwtFigure.h"
#include "common/sharedStructuresGraphics.h"
#include "DataObject/dataObjectFuncs.h"

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

using namespace ito;


//----------------------------------------------------------------------------------------------------------------------------------
Plot2DWidget::Plot2DWidget(QMenu *contextMenu, QWidget * parent) :
        QwtPlot(parent),
        m_lineplotUID(0),
        m_contextMenu(contextMenu),
        m_pContent(NULL),
        m_startScaledZ(false),
        m_startScaledY(false),
        m_startScaledX(false),
        m_paletteNum(0),
        m_pParent(parent),
        m_pZoomer(NULL),
        m_pPanner(NULL),
        m_pLinePicker(NULL),
        m_pAScanPicker(NULL),
        m_pAScanMarker(NULL),
        m_pValuePicker(NULL),
        m_pCmplxMenu(NULL),
        m_cmplxState(false),
        m_stackState(false),
        m_stateMoveAligned(false)

{
    this->setMouseTracking(false); //(mouse tracking is controled by action in WinMatplotlib)

    m_pContent = new ItoPlotSpectrogram("test");
    m_pContent->setRenderThreadCount(0);
    m_pContent->setColorMap( new QwtLinearColorMap(QColor::fromRgb(0,0,0), QColor::fromRgb(255,255,255), QwtColorMap::Indexed));

    m_pZoomer = new QwtPlotZoomer(QwtPlot::xBottom, QwtPlot::yLeft, canvas());
    m_pZoomer->setEnabled(false);
    m_pZoomer->setRubberBandPen(QPen(QBrush(Qt::red),3,Qt::DashLine));
    m_pZoomer->setTrackerMode(QwtPicker::AlwaysOn);
    m_pZoomer->setTrackerFont(QFont("Verdana",10));
    m_pZoomer->setTrackerPen(QPen(QBrush(Qt::green),2));

	m_orgImageSize = m_pZoomer->zoomBase();

    m_pPanner = new QwtPlotPanner(canvas());
    m_pPanner->setAxisEnabled(QwtPlot::yRight,false);
    m_pPanner->setCursor(Qt::SizeAllCursor);
    m_pPanner->setEnabled(false);

    m_pValuePicker = new ValuePicker2D(m_pContent, QwtPlot::xBottom, QwtPlot::yLeft, canvas());
    m_pValuePicker->setEnabled(false);
    m_pValuePicker->setTrackerMode(QwtPicker::AlwaysOn);
    m_pValuePicker->setTrackerFont(QFont("Verdana",10));
    m_pValuePicker->setTrackerPen(QPen(QBrush(Qt::red),2));
    m_pValuePicker->setBackgroundFillBrush( QBrush(QColor(255,255,255,155), Qt::SolidPattern) );

    m_pAScanPicker = new QwtPicker(QwtPicker::CrossRubberBand , QwtPicker::AlwaysOn, canvas());
    m_pAScanPicker->setEnabled(false);
    m_pAScanPicker->setStateMachine(new QwtPickerDragPointMachine);
    m_pAScanPicker->setRubberBandPen(QPen(QBrush(Qt::red),1));
    m_pAScanPicker->setTrackerPen(QPen(QBrush(Qt::red),1));
    connect(m_pAScanPicker, SIGNAL(moved(const QPoint&)), SLOT(trackerAScanMoved(const QPoint&)));
    connect(m_pAScanPicker, SIGNAL(appended(const QPoint&)), SLOT(trackerAScanAppended(const QPoint&)));

    m_pAScanMarker = new QwtPlotMarker();
    m_pAScanMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green),3),  QSize(7,7) ));
    m_pAScanMarker->attach(this);
    m_pAScanMarker->setVisible(false);
    
    m_pLinePicker = new QwtPicker(QwtPicker::CrossRubberBand, QwtPicker::AlwaysOn, canvas());
    m_pLinePicker->setEnabled(false);
    m_pLinePicker->setStateMachine(new QwtPickerDragPointMachine);
    m_pLinePicker->setRubberBandPen(QPen(Qt::green));
    m_pLinePicker->setTrackerPen(QPen(Qt::green));
    connect(m_pLinePicker, SIGNAL(moved(const QPoint&)), SLOT(trackerMoved(const QPoint&)));
    connect(m_pLinePicker, SIGNAL(appended(const QPoint&)), SLOT(trackerAppended(const QPoint&)));

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
    m_lineCut.attach(this);

    m_pContent->attach(this);

//    m_pEventFilter = new Plot2DEFilter(this, m_pthisNode, m_pContent);
//    installEventFilter( m_pEventFilter );
}

//----------------------------------------------------------------------------------------------------------------------------------
Plot2DWidget::~Plot2DWidget()
{
    if (m_pZoomer)
    {
        delete m_pZoomer;
    }
    if (m_pPanner)
    {
        delete m_pPanner;
    }
    if (m_pLinePicker)
    {
        delete m_pLinePicker;
    }
    if (m_pAScanPicker)
    {
        delete m_pAScanPicker;
    }
    if (m_pAScanMarker)
    {
        delete m_pAScanMarker;
    }
    if (m_pValuePicker)
    {
        delete m_pValuePicker;
    }
    if (m_pCmplxMenu)
    {
        m_pCmplxMenu->clear();
        delete m_pCmplxMenu;
    }
    if (m_pContent)
    {
        delete m_pContent;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot2DWidget::refreshPlot(ito::ParamBase *param)
{
    DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());
    bool newRasterDataCreated = false;

    if ((param != NULL) && (param->getType() == (ito::Param::DObjPtr & ito::paramTypeMask)))
    {
        //check dataObj
        ito::DataObject *dataObj = (ito::DataObject*)param->getVal<char*>();
        int dims = dataObj->getDims();
        if(dims > 1)
        {
            QList<unsigned int> startPoint;
            startPoint.append(0);
            startPoint.append(0);
            
            if(dataObj->calcNumMats() > 1)
            { 
                int* limits = new int[2 * dims];
                memset(limits, 0, sizeof(int) * 2 * dims);

                // set roi to first matrix
                for(int i = 0; i < (dims - 2); i++)
                {
                    limits[2 * i + 1] = -1 * dataObj->getSize(i) + 1;
                }
                dataObj->adjustROI(dims, limits);
                delete limits;
                
                if(!m_stackState) ((itom2DQwtFigure*)m_pParent)->enableZStackGUI(true);
                m_stackState = true;
            }
            else
            {
                if(m_stackState) ((itom2DQwtFigure*)m_pParent)->enableZStackGUI(false);
                m_stackState = false;
            }

            if(dataObj->getType() == ito::tComplex128 || dataObj->getType() == ito::tComplex64)
            {
                if(!m_cmplxState) ((itom2DQwtFigure*)m_pParent)->enableComplexGUI(true);
                m_cmplxState = true;                
            }
            else
            {
                if(!m_cmplxState) ((itom2DQwtFigure*)m_pParent)->enableComplexGUI(false);
                m_cmplxState = false;                
            }

			bool test = true;
			int width = dataObj->getSize(dims - 1);
			int height = dataObj->getSize(dims - 2);

            double x0 = dataObj->getPixToPhys(dims - 1, 0, test);
            double y0 = dataObj->getPixToPhys(dims - 2, 0, test);
            double realWidth = dataObj->getPixToPhys(dims - 1, width, test) - x0;
            double realHeight = dataObj->getPixToPhys(dims - 2, height, test) - y0;

			QRectF newImageSize = QRectF( x0, y0, realWidth, realHeight);
			if(newImageSize != m_orgImageSize)
			{
				m_orgImageSize = newImageSize;

				setAxisScale( QwtPlot::xBottom, newImageSize.left(), newImageSize.width() );
                setAxisScale( QwtPlot::yLeft, newImageSize.bottom(), newImageSize.height() );
				m_pZoomer->setZoomBase(m_orgImageSize);
				m_pZoomer->zoom(0);
			}

            if(rasterData)
            {
                rasterData->updateDataObject(QSharedPointer<ito::DataObject>(new ito::DataObject(*dataObj)), startPoint, dims - 1, dataObj->getSize(dims - 1), dims - 2, dataObj->getSize(dims - 2));
            }
            else
            {
                m_pContent->setData(new DataObjectRasterData(QSharedPointer<ito::DataObject>(new ito::DataObject(*dataObj)), startPoint, dims - 1, dataObj->getSize(dims - 1), dims - 2, dataObj->getSize(dims - 2), 1));
                newRasterDataCreated = true;
                if(dataObj)
                {
                    bool test;
                    // Copy Object scales to rastobject
                    //DataObjectRasterData * rastaData = static_cast<DataObjectRasterData*>(m_pContent->data());
                    //rastaData->setObjOffsetsAndScales(dataObj->getAxisScales(dataObj->getDims() - 1, true), dataObj->getAxisOffset(dataObj->getDims() - 1, true),dataObj->getAxisScales(dataObj->getDims() - 2, true), dataObj->getAxisOffset(dataObj->getDims() - 2, true));

                    // Copy Titel and axis scale to widget
                    QString qtBuf("");

                    std::string tDescription(dataObj->getValueDescription());
                    std::string tUnit(dataObj->getValueUnit());
                    if(!tDescription.empty())
                    {
                        qtBuf.append(tDescription.data());
                    }
                    if (!tUnit.empty())
                    {
                        if(qtBuf.size()) qtBuf.append(" in ");
                        qtBuf.append(tUnit.data()); 
                    }
                    if(qtBuf.size()) setAxisTitle(QwtPlot::yRight, qtBuf);
                    qtBuf.clear();

                    tDescription = dataObj->getAxisDescription(dims - 1, test);
                    tUnit = dataObj->getAxisUnit(dims - 1, test);
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
                    qtBuf.clear();

                    tDescription = dataObj->getAxisDescription(dims- 2, test);
                    tUnit = dataObj->getAxisUnit(dims - 2, test);
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
                    qtBuf.clear();

                    std::string tTitle(dataObj->getTag("title", test).getVal_ToString());
                    qtBuf.clear();
                    if(test)
                    {
                        qtBuf.append(tTitle.data());   
                    }
                    setTitle(qtBuf);

                    rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());
                    QwtInterval interval = rasterData->interval(Qt::ZAxis);
                    setAxisScale(QwtPlot::yRight, interval.minValue(), interval.maxValue());
                    axisWidget(QwtPlot::yRight)->setColorMap(interval, new QwtLinearColorMap(Qt::black, Qt::white));

                    if(m_startScaledZ)
                    {
                        rasterData->setIntervalRange(Qt::ZAxis, false, m_startRangeZ.x(), m_startRangeZ.y());
                    }
                    if(m_startScaledY)
                    {
                        rasterData->setIntervalRange(Qt::YAxis, false, m_startRangeY.x(), m_startRangeY.y());
                    }
                    if(m_startScaledX)
                    {
                        rasterData->setIntervalRange(Qt::XAxis, false, m_startRangeX.x(), m_startRangeX.y());
                    }
                }
            }
        }
        else
        {
            //error
        }
    }
    else if(rasterData)
    {
        rasterData->updateDataObject(QSharedPointer<ito::DataObject>());
        //if(m_pReplotPending) *m_pReplotPending = false;
    }
    else
    {
        //if(m_pReplotPending) *m_pReplotPending = false;
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot2DWidget::contextMenuEvent(QContextMenuEvent * event)
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
void Plot2DWidget::refreshColorMap(QString palette)
{
    QwtLinearColorMap *colorMap = NULL;
    QwtLinearColorMap *colorBarMap = NULL;
    ito::ItomPalette newPalette;
    ito::RetVal retval(ito::retOk);
    int numPalettes = 1;


    if (palette.isEmpty())
    {
        retval = apiPaletteGetNumberOfColorBars(numPalettes);
        m_paletteNum %= numPalettes;
        if (m_paletteNum == 0)
            return;

        retval += apiPaletteGetColorBarIdx(m_paletteNum, newPalette);
    }
    else
    {
        retval += apiPaletteGetColorBarName(palette, newPalette);
        retval += apiPaletteGetColorBarIdxFromName(palette, m_paletteNum);
    }

    int totalStops = newPalette.colorStops.size();

    if (totalStops < 2)
    {
        return;
    }

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

    if(colorMap != NULL)
    {
        m_pContent->setColorMap(colorMap);
        DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());
        if(rasterData)
        {
            QwtInterval interval = rasterData->interval(Qt::ZAxis);
            setAxisScale(QwtPlot::yRight, interval.minValue(), interval.maxValue());
            axisWidget(QwtPlot::yRight)->setColorMap(interval, colorBarMap);
        }
        else
        {
            axisWidget(QwtPlot::yRight)->setColorMap(QwtInterval(0,1.0), colorBarMap);
        }
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot2DWidget::trackerAppended(const QPoint &pt)
{
    QVector<QPointF> pts;
    pts.resize(2);

    if (m_pContent && m_pContent->data())
    {
        pts[0].setY(invTransform(this->m_pContent->yAxis(), pt.y()));
        pts[0].setX(invTransform(this->m_pContent->xAxis(), pt.x()));

        pts[1] = pts[0];
    }
    m_lineCut.setSamples(pts);
    replot();

    ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID, 0);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot2DWidget::trackerMoved(const QPoint &pt)
{
    QVector<QPointF> pts;
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

    ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID, 0);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot2DWidget::trackerAScanAppended(const QPoint &pt)
{
    QVector<QPointF> pts;

    QPointF scale;
    pts.resize(1);

    if (m_pContent && m_pContent->data())
    {
        pts[0].setY(invTransform(this->m_pContent->yAxis(), pt.y()));
        pts[0].setX(invTransform(this->m_pContent->xAxis(), pt.x()));

        QwtInterval interv = m_pContent->data()->interval(Qt::ZAxis);
        //scale.setX(interv.minValue());
        //scale.setY(interv.maxValue());
    }

    m_pAScanMarker->setValue(pts[0]);
    m_lineCut.setSamples(pts);

    replot();
    ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID, 2);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot2DWidget::trackerAScanMoved(const QPoint &pt)
{
    QVector<QPointF> pts;
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
    ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID, 2);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot2DWidget::setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue)
{
    DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());
    if(rasterData)
    {
        rasterData->setIntervalRange(axis, autoCalcLimits, minValue, maxValue);

        replot();
        return ito::retOk;
    }
    else
    {
        switch(axis)
        {
            case Qt::ZAxis:
                m_startScaledZ = true;
                m_startRangeZ = QPointF(minValue, maxValue);
            break;
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
    return ito::retError;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot2DWidget::keyPressEvent ( QKeyEvent * event ) 
{
    char dir = 0;
    if(!m_pContent || !m_pContent->data())
        return;

    if (!m_lineCut.isVisible())
        return;

    switch(((const QKeyEvent *)event)->key())
    {
        case Qt::Key_Up:
        {
            if (m_lineplotUID == 0 || m_lineCut.data()->size() == 0)
                return;
            QVector<QPointF> pts;
            DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());
            int dims = rasterData->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index
            int ySize = rasterData->getDataObject()->getSize(dims-2); // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index

            bool test;
            double upperBorder = rasterData->getDataObject()->getPixToPhys(dims-2, ySize - 1, test);
            double diff = rasterData->getDataObject()->getAxisScale(dims-2);
     
            if(!ito::dObjHelper::isFinite<double>(diff) || !ito::dObjHelper::isNotZero<double>(diff))
                diff = 1.0;

            if(m_pAScanPicker->isEnabled())   // doing the linecut
            {
                pts.resize(1);
                pts[0] = m_lineCut.sample(0);
                if (pts[0].y() < upperBorder) pts[0].setY(pts[0].y() + diff);

                // setting linecut direction to z
                dir = 2;
            }
            else
            {
                pts.resize(2);
                pts[0] = m_lineCut.sample(0);
                pts[1] = m_lineCut.sample(1);

                //if (pts[0].y() < upperBoarder) pts[0].setY(pts[0].y() + 1);
                //if (pts[1].y() < upperBoarder) pts[1].setY(pts[1].y() + 1);

                if (pts[0].y() < upperBorder) pts[0].setY(pts[0].y() + diff);
                if (pts[1].y() < upperBorder) pts[1].setY(pts[1].y() + diff);

                // setting linecut direction to x / y
                dir = 0;
            }

            m_lineCut.setSamples(pts);
            replot();
            ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID, dir);
        }
        return;

        case Qt::Key_Down:
        {
            if (m_lineplotUID == 0 || m_lineCut.data()->size() == 0)
                return;
            QVector<QPointF> pts;
            DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());

            int dims = rasterData->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index
                    
            bool test;
            double lowerBorder = rasterData->getDataObject()->getPixToPhys(dims-2, 0, test);
            double diff = rasterData->getDataObject()->getAxisScale(dims-2);

            if(!ito::dObjHelper::isFinite<double>(diff) || !ito::dObjHelper::isNotZero<double>(diff))
                diff = 1.0;

            //if (pts[0].y() > 0) pts[0].setY(pts[0].y() - 1);
            //if (pts[1].y() > 0) pts[1].setY(pts[1].y() - 1);
            if(m_pAScanPicker->isEnabled())   // doing the linecut
            {
                pts.resize(1);
                pts[0] = m_lineCut.sample(0);
                if (pts[0].y() > lowerBorder) pts[0].setY(pts[0].y() - diff);

                // setting linecut direction to z
                dir = 2;
            }
            else
            {
                pts.resize(2);
                pts[0] = m_lineCut.sample(0);
                pts[1] = m_lineCut.sample(1);
                if (pts[0].y() > lowerBorder) pts[0].setY(pts[0].y() - diff);
                if (pts[1].y() > lowerBorder) pts[1].setY(pts[1].y() - diff);

                // setting linecut direction to x / y
                dir = 0;
            }

            m_lineCut.setSamples(pts);
            replot();
            ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID, dir);
        }
        return;

        case Qt::Key_Right:
        {
            if (m_lineplotUID == 0 || m_lineCut.data()->size() == 0)
                return;
            QVector<QPointF> pts;
            DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());
                    
            int dims = rasterData->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index

            int xSize = rasterData->getDataObject()->getSize(dims-1);
            bool test;
            double rightBorder = rasterData->getDataObject()->getPixToPhys(dims-1, xSize - 1, test);
            double diff = rasterData->getDataObject()->getAxisScale(dims-1);

            if(!ito::dObjHelper::isFinite<double>(diff) || !ito::dObjHelper::isNotZero<double>(diff))
                diff = 1.0;

            //if (pts[0].x() < rasterData->getDataObject()->getSize(dims-1) - 1) pts[0].setX(pts[0].x() + 1);
            //if (pts[1].x() < rasterData->getDataObject()->getSize(dims-1) - 1) pts[1].setX(pts[1].x() + 1);

            if(m_pAScanPicker->isEnabled())   // doing the linecut
            {
                pts.resize(1);
                pts[0] = m_lineCut.sample(0);
                if (pts[0].x() < rightBorder) pts[0].setX(pts[0].x() + diff);

                // setting linecut direction to z
                dir = 2;
            }
            else
            {
                pts.resize(2);
                pts[0] = m_lineCut.sample(0);
                pts[1] = m_lineCut.sample(1);

                if (pts[0].x() < rightBorder) pts[0].setX(pts[0].x() + diff);
                if (pts[1].x() < rightBorder) pts[1].setX(pts[1].x() + diff);

                // setting linecut direction to x / y
                dir = 0;
            }

            m_lineCut.setSamples(pts);
            replot();
            ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID, dir);
        }
        return;

        case Qt::Key_Left:
        {
            if (m_lineplotUID == 0 || m_lineCut.data()->size() == 0)
                return;
            QVector<QPointF> pts;
            DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());

            bool test;

            int dims = rasterData->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index

            double leftBorder = rasterData->getDataObject()->getPixToPhys(dims-1, 0.0, test);
            double diff = rasterData->getDataObject()->getAxisScale(dims-1);

            if(!ito::dObjHelper::isFinite<double>(diff) || !ito::dObjHelper::isNotZero<double>(diff))
                diff = 1.0;

            if(m_pAScanPicker->isEnabled())   // doing the linecut
            {
                pts.resize(1);
                pts[0] = m_lineCut.sample(0);
                if (pts[0].x() > leftBorder) pts[0].setX(pts[0].x() - diff);

                // setting linecut direction to z
                dir = 2;
            }
            else
            {
                pts.resize(2);
                pts[0] = m_lineCut.sample(0);
                pts[1] = m_lineCut.sample(1);

                //if (pts[0].x() > 0) pts[0].setX(pts[0].x() - 1);
                //if (pts[1].x() > 0) pts[1].setX(pts[1].x() - 1);

                if (pts[0].x() > leftBorder) pts[0].setX(pts[0].x() - diff);
                if (pts[1].x() > leftBorder) pts[1].setX(pts[1].x() - diff);

                // setting linecut direction to x / y
                dir = 0;
            }
            m_lineCut.setSamples(pts);
            replot();
            ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID, dir);
        }
        return;

        // The following keys represent a direction, they are
        // organized on the keyboard.

        case Qt::Key_H:
        {
            if (m_lineplotUID == 0 || m_pAScanPicker->isEnabled() || m_lineCut.data()->size() == 0)
                return;
            QVector<QPointF> pts;
            pts.resize(2);
            DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());

            int dims = rasterData->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index
                    
            int sizey = rasterData->getDataObject()->getSize(dims-2);
            int sizex = rasterData->getDataObject()->getSize(dims-1);
            //pts[0].setX(0);
            //pts[0].setY(sizey / 2);
            //pts[1].setX(sizex);
            //pts[1].setY(pts[0].y());
            bool test = true;
            double yCenter = rasterData->getDataObject()->getPixToPhys( dims-2, sizey / 2.0, test);
            double xMin = rasterData->getDataObject()->getPixToPhys( dims-1, 0, test);
            double xMax = rasterData->getDataObject()->getPixToPhys( dims-1, sizex, test);

            pts[0].setX(xMin);
            pts[0].setY(yCenter);
            pts[1].setX(xMax);
            pts[1].setY(yCenter);

            m_lineCut.setSamples(pts);
            replot();
            ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID, 0);
        }
        break;

        case Qt::Key_V:
        {
            if (m_lineplotUID == 0 || m_pAScanPicker->isEnabled() || m_lineCut.data()->size() == 0)
                return;
            QVector<QPointF> pts;
            pts.resize(2);
            DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());

            int dims = rasterData->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index
                    
            int sizey = rasterData->getDataObject()->getSize(dims-2);
            int sizex = rasterData->getDataObject()->getSize(dims-1);
            //pts[0].setX(sizex / 2);
            //pts[0].setY(0);
            //pts[1].setX(pts[0].x());
            //pts[1].setY(sizey);

            bool test = true;
            double xCenter = rasterData->getDataObject()->getPixToPhys( dims-1, sizex / 2.0, test);
            double yMin = rasterData->getDataObject()->getPixToPhys( dims-2, 0, test);
            double yMax = rasterData->getDataObject()->getPixToPhys( dims-2, sizey, test);

            pts[0].setX(xCenter);
            pts[0].setY(yMin);
            pts[1].setX(xCenter);
            pts[1].setY(yMax);

            m_lineCut.setSamples(pts);
            replot();
            ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID, 0);
        }
        break;

        case Qt::Key_Control:
            m_stateMoveAligned = true;
        break;

        default:
        break;
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot2DWidget::keyReleaseEvent ( QKeyEvent * event )
{

    switch(((const QKeyEvent *)event)->key())
    {
        case Qt::Key_Control:
            m_stateMoveAligned = false;
        break;

        default:
        break;
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot2DWidget::stackForward()
{
    // TODO: this implementation doesn't respect the originally set roi of the source, neither is the displayed object updated. 
    // This must be fixed soon
    DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());

    if(rasterData)
    {
        QSharedPointer<ito::DataObject> dObj = rasterData->getDataObject();
        QList<unsigned int> startPoint;
        startPoint.append(0);
        startPoint.append(0);

        int dims, *wholeSizes = NULL, *offsets = NULL;

        dims = dObj->getDims();
        wholeSizes = new int[2 * dims];
        memset(wholeSizes, 0, sizeof(int) * dims);
        offsets = new int[2 * dims];
        memset(offsets, 0, sizeof(int) * dims);
        dObj->locateROI(wholeSizes, offsets);

        int* limits = new int[2 * dims];
        memset(limits, 0, sizeof(int) * 2 * dims);

        if (offsets[dims - 3] < wholeSizes[dims - 3] - 1)
        {
            limits[(dims - 3) * 2] = -1;
            limits[(dims - 3) * 2 + 1] = 1;
        }
        else
        {
            limits[(dims - 3) * 2] = wholeSizes[(dims - 3)] - 1;
            limits[(dims - 3) * 2 + 1] = -wholeSizes[(dims - 3)] + 1;
        }

        dObj->adjustROI(dims, limits);
        delete offsets;
        delete wholeSizes;
        delete limits;

        rasterData->updateDataObject(QSharedPointer<ito::DataObject>(new ito::DataObject(*dObj)), startPoint, dims - 1, dObj->getSize(dims - 1), dims - 2, dObj->getSize(dims - 2));
        rasterData->setIntervalRange(Qt::ZAxis, true, 0, 0);
        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot2DWidget::stackBack()
{
    // TODO: this implementation doesn't respect the originally set roi of the source, neither is the displayed object updated. 
    // This must be fixed soon
    DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->data());

    if(rasterData)
    {
        QSharedPointer<ito::DataObject> dObj = rasterData->getDataObject();
        QList<unsigned int> startPoint;
        startPoint.append(0);
        startPoint.append(0);

        int dims, *wholeSizes = NULL, *offsets = NULL;

        dims = dObj->getDims();
        wholeSizes = new int[2 * dims];
        memset(wholeSizes, 0, sizeof(int) * dims);
        offsets = new int[2 * dims];
        memset(offsets, 0, sizeof(int) * dims);
        dObj->locateROI(wholeSizes, offsets);

        int* limits = new int[2 * dims];
        memset(limits, 0, sizeof(int) * 2 * dims);

        if (offsets[dims - 3] > 0)
        {
            limits[(dims - 3) * 2] = 1;
            limits[(dims - 3) * 2 + 1] = -1;
        }
        else
        {
            limits[(dims - 3) * 2] = -(wholeSizes[(dims - 3)] - 1);
            limits[(dims - 3) * 2 + 1] = wholeSizes[(dims - 3)] - 1;
        }

        dObj->adjustROI(dims, limits);
        delete offsets;
        delete wholeSizes;
        delete limits;

        rasterData->updateDataObject(QSharedPointer<ito::DataObject>(new ito::DataObject(*dObj)), startPoint, dims - 1, dObj->getSize(dims - 1), dims - 2, dObj->getSize(dims - 2));
        rasterData->setIntervalRange(Qt::ZAxis, true, 0, 0);
        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------

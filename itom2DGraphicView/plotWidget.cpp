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

#include "graphicViewPlot.h"

#include "DataObject/dataObjectFuncs.h"

#include <qimage.h>

#include <qpixmap.h>
#include <qdebug.h>
#include <qmessagebox.h>


using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
PlotWidget::PlotWidget(InternalData* pData, QMenu *contextMenu, QWidget * parent) :
        QGraphicsView(parent),
        m_pData(pData),
        m_contextMenu(contextMenu),
        m_pParent(parent),
        m_lineplotUID(0),
        m_pItem(NULL),
        m_pLineCut(NULL),
        //m_pTracker(NULL),
        m_pContent(NULL),
        m_ObjectContainer(NULL),
        m_lineIsSampling(false),
        m_trackerIsSampling(false),
        m_stateMoveAligned(false),
        m_pValuePicker(NULL)
{
    m_pContent = new QGraphicsScene(this);
    setScene(m_pContent);
    m_pContent->clear();
    m_pixMap.fromImage(QImage(30, 30, QImage::Format_Indexed8));

    m_pItem = new QGraphicsPixmapItem(m_pixMap);
    m_pContent->clear();
    m_pContent->addItem((QGraphicsItem*)m_pItem);
    fitInView(m_pItem, Qt::KeepAspectRatio);
}

//----------------------------------------------------------------------------------------------------------------------------------
PlotWidget::~PlotWidget()
{
    if(m_pContent)
    {
        m_pContent->deleteLater();
        m_pContent = NULL;
    }

    if(m_ObjectContainer)
    {
        delete m_ObjectContainer;
        m_ObjectContainer = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::handleMouseEvent(int type, QMouseEvent *event)
{
    Qt::MouseButton btn = event->button();
    Qt::MouseButtons btns = event->buttons();
    int button = 0;
    QPointF scenePos = mapToScene(event->pos());
    
    if (!m_pContent || !m_pItem || !m_pLineCut || !m_pValuePicker)
        return;

    switch(type)
    {
        case 0:
            switch(btn)
            {
                case Qt::LeftButton:
                {
                    if (m_pData->m_state == tLineCut)
                    {
                        trackerAppended(scenePos);
                        m_lineIsSampling = true;
                    }
                    else if (m_pData->m_state == tValuePicker)
                    {
                        m_pValuePicker->setPos(scenePos);
                        updatePointTracker();
                        m_trackerIsSampling  = true;
                    }
                }
                break;
            }

        break;
        case 1:
            if (m_pData->m_state == tLineCut)
            {
                m_lineIsSampling = false;
            }
            switch(btn)
            {
                case Qt::LeftButton:
                {
                
                }
                break;
            }
        break;
        case 2:
        {
            if (m_lineIsSampling)
            {

                if (m_stateMoveAligned)
                {
                    if (abs(scenePos.x() - m_pLineCut->line().x1()) > abs(scenePos.y() - m_pLineCut->line().y1()))
                    {
                        scenePos.setY(m_pLineCut->line().y1());
                    }
                    else
                    {
                        scenePos.setX(m_pLineCut->line().x1());
                    }
                }

                trackerMoved(scenePos);
            }
            else if (m_trackerIsSampling)
            {
                if (m_stateMoveAligned)
                {
                    double x = m_pValuePicker->x();
                    double y = m_pValuePicker->y();
                    if (abs(scenePos.x() - x) > abs(scenePos.y() - y))
                    {
                        scenePos.setY(y + 1.0);
                    }
                    else
                    {
                        scenePos.setX(x + 1.0);
                    }
                }

                m_pValuePicker->setPos(scenePos);
                updatePointTracker();          
            }
        }
        break;
        case 3:
            if (m_pData->m_state == tLineCut)
            {
                m_lineIsSampling = false;
            }
            if (m_pData->m_state == tValuePicker)
            {
                m_trackerIsSampling = false;        
            }

        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::refreshPlot(ito::ParamBase *param)
{
    bool newObjectContainer = false;

    if ((param != NULL) && (param->getType() == (ito::Param::DObjPtr & ito::paramTypeMask)))
    {
        //check dataObj
        ito::DataObject *dataObj = (ito::DataObject*)param->getVal<char*>();
        int dims = dataObj->getDims();
        if (dims > 1)
        {            
            //if (m_pItem)
            //    delete m_pItem;
            
            if (dataObj->getType() == ito::tComplex128 || dataObj->getType() == ito::tComplex64)
            {
                if (m_pData->m_dataType != ito::tComplex128 && m_pData->m_dataType != ito::tComplex64)
                {
                    ((GraphicViewPlot*)m_pParent)->enableComplexGUI(true);
                }             
            }
            else
            {
                if (m_pData->m_dataType == ito::tComplex128 && m_pData->m_dataType == ito::tComplex64)
                {
                    ((GraphicViewPlot*)m_pParent)->enableComplexGUI(false);
                }            
            }

            m_pData->m_dataType = (ito::tDataType)(dataObj->getType());

            if (m_ObjectContainer == NULL)
            {
                m_ObjectContainer = new RasterToQImageObj(QSharedPointer<ito::DataObject>(new ito::DataObject(*dataObj)), 1); 
                newObjectContainer = true;
            }
            else
            {
                m_ObjectContainer->updateDataObject(QSharedPointer<ito::DataObject>(new ito::DataObject(*dataObj)));
            }

            m_pixMap.convertFromImage(m_ObjectContainer->convert2QImage(m_pData));               
            m_pItem->setPixmap(m_pixMap);

            if (m_pValuePicker &&  m_pValuePicker->isVisible())
            {
                updatePointTracker();
            }

            repaint();
        }
    }
    else if (m_ObjectContainer != NULL)
    {
        m_pixMap.convertFromImage(m_ObjectContainer->convert2QImage(m_pData));              

        if (!m_pItem)
        {
            m_pItem = new QGraphicsPixmapItem(m_pixMap);
            m_pContent->clear();
            m_pContent->addItem((QGraphicsItem*)m_pItem);
            fitInView(m_pItem, Qt::KeepAspectRatio);
        }
        else
            m_pItem->setPixmap(m_pixMap);

        repaint();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::keyReleaseEvent (QKeyEvent * event)
{
    if (!hasFocus())
    {
        return;
    }

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
void PlotWidget::keyPressEvent (QKeyEvent * event) 
{
    if (!hasFocus())
    {
        return;
    }

    if (!m_pContent || !m_pItem || !m_pLineCut || !m_pValuePicker)
    {
        return;
    }

    if (!m_ObjectContainer)
    {
        return;
    }    

    switch(((const QKeyEvent *)event)->key())
    {
        case Qt::Key_Up:
        { 
            if (m_pData->m_state == tValuePicker)   // doing the linecut
            {
                QPointF scenePos = m_pValuePicker->pos() - QPointF(0.0, 1.0);
                if (scenePos.y() < -1.0) scenePos.setY(-1.0);

                m_pValuePicker->setPos(scenePos);
                updatePointTracker();
            }
            else if (m_pData->m_state == tLineCut)
            {
                QPointF pt1 = m_pLineCut->line().p1() - QPointF(0.0, 1.0);
                QPointF pt2 = m_pLineCut->line().p2() - QPointF(0.0, 1.0);

                if (pt1.y() < 0.0) pt1.setY(0.0);
                trackerAppended(pt1);

                if (pt2.y() < 0.0) pt2.setY(0.0);
                trackerMoved(pt2);
            }
        }
        return;

        case Qt::Key_Down:
        {
            int dims = m_ObjectContainer->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index
            int y1 = m_ObjectContainer->getDataObject()->getSize(dims-2) - 1; // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index
       
            if (m_pData->m_state == tValuePicker)   // doing the linecut
            {
                QPointF scenePos = m_pValuePicker->pos() + QPointF(0.0, 1.0);
                if (scenePos.y() > (y1 - 1)) scenePos.setY(y1 - 1);

                m_pValuePicker->setPos(scenePos);
                updatePointTracker();
            }
            else if (m_pData->m_state == tLineCut)
            {
                QPointF pt1 = m_pLineCut->line().p1() + QPointF(0.0, 1.0);
                QPointF pt2 = m_pLineCut->line().p2() + QPointF(0.0, 1.0);

                if (pt1.y() > y1) pt1.setY(y1);
                trackerAppended(pt1);

                if (pt2.y() > y1) pt2.setY(y1);
                trackerMoved(pt2);
            }
        }
        return;

        case Qt::Key_Right:
        {
            int dims = m_ObjectContainer->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index
            int x1 = m_ObjectContainer->getDataObject()->getSize(dims-1) - 1; // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index
 
            if (m_pData->m_state == tValuePicker)   // doing the linecut
            {
                QPointF scenePos = m_pValuePicker->pos() + QPointF(1.0, 0.0);
                if (scenePos.x() < x1) scenePos.setX(scenePos.x() - 1.0);

                m_pValuePicker->setPos(scenePos);
                updatePointTracker();
            }
            else if (m_pData->m_state == tLineCut)
            {
                QPointF pt1 = m_pLineCut->line().p1() + QPointF(1.0, 0.0);
                QPointF pt2 = m_pLineCut->line().p2() + QPointF(1.0, 0.0);

                if (pt1.x() > x1) pt1.setX(x1);
                trackerAppended(pt1);

                if (pt2.x() > x1) pt2.setX(x1);
                trackerMoved(pt2);
            }
        }
        return;

        case Qt::Key_Left:
        {
            if (m_pData->m_state == tValuePicker)   // doing the linecut
            {
                QPointF scenePos = m_pValuePicker->pos() - QPointF(1.0, 0.0);
                if (scenePos.x() < 0.0) scenePos.setX(0.0);

                m_pValuePicker->setPos(scenePos);
                updatePointTracker();
            }
            else if (m_pData->m_state == tLineCut)
            {
                QPointF pt1 = m_pLineCut->line().p1() - QPointF(1.0, 0.0);
                QPointF pt2 = m_pLineCut->line().p2() - QPointF(1.0, 0.0);
                if (pt1.x() < 0.0) pt1.setX(0.0);
                trackerAppended(pt1);

                if (pt2.x() < 0.0) pt2.setX(0.0);
                trackerMoved(pt2);
            }
        }
        return ;

        // The following keys represent a direction, they are
        // organized on the keyboard.

        case Qt::Key_H:
        {
            int dims = m_ObjectContainer->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index

            bool test = true;
            double yCenter = m_ObjectContainer->getDataObject()->getSize(dims-2) / 2.0;
            double xMin = 0.0;
            double xMax = m_ObjectContainer->getDataObject()->getSize(dims-1) - 1;

            if (m_pData->m_state == tValuePicker)   // doing the linecut
            {
                QPointF scenePos = m_pValuePicker->pos();
                scenePos.setY(yCenter);
                if (scenePos.y() < 0.0) scenePos.setY(0.0);

                m_pValuePicker->setPos(scenePos);
                updatePointTracker();
            }
            else if (m_pData->m_state == tLineCut)
            {
                QPointF pt(xMin, yCenter);
                trackerAppended(pt);

                pt = QPointF(xMax, yCenter);
                trackerMoved(pt);
            }
        }
        break;

        case Qt::Key_V:
        {
            int dims = m_ObjectContainer->getDataObject()->getDims();  // Be careful -> 3D Objects are orders in z y x so y-Dims changes its index

            bool test = true;
            double xCenter = m_ObjectContainer->getDataObject()->getSize(dims-1) / 2.0;
            double yMin = 0.0;
            double yMax = m_ObjectContainer->getDataObject()->getSize(dims-2) - 1;

            if (m_pData->m_state == tValuePicker)   // doing the linecut
            {
                QPointF scenePos = m_pValuePicker->pos();
                scenePos.setX(xCenter);
                if (scenePos.x() < 0.0) scenePos.setX(0.0);

                m_pValuePicker->setPos(scenePos);
                updatePointTracker();
            }
            else if (m_pData->m_state == tLineCut)
            {
                QPointF pt(xCenter, yMin);
                trackerAppended(pt);

                pt = QPointF(xCenter, yMax);
                trackerMoved(pt);
            }
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
void PlotWidget::mouseMoveEvent (QMouseEvent * event)
{
    if (!hasFocus())
    {
        return;
    }
    handleMouseEvent(2, event);
    event->accept();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::mousePressEvent (QMouseEvent * event)
{
    if (!hasFocus())
    {
        return;
    }
    handleMouseEvent(0, event);
    event->accept();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::mouseReleaseEvent (QMouseEvent * event)
{
    if (!hasFocus())
        return;
    QApplication::restoreOverrideCursor();
    handleMouseEvent(3, event);
    event->accept();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::contextMenuEvent(QContextMenuEvent * event)
{
    if (m_showContextMenu)
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
void PlotWidget::resizeEvent(QResizeEvent * event)
{
    if (m_pItem)
    {
        if (m_pData->m_zoomLevel == RatioOff)
        {
            fitInView(m_pItem, Qt::KeepAspectRatio);
        }
        else
        {
            if (m_ObjectContainer)
            {
                int ysize = m_ObjectContainer->getDataObjHeight();
                int xsize = m_ObjectContainer->getDataObjWidth();
                setSceneRect (0.0, 0.0, xsize, ysize);
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::trackerAppended(const QPointF &pt)
{
    if (!m_pLineCut) return;

    int ymax = m_ObjectContainer->getDataObjHeight() - 1;
    int xmax = m_ObjectContainer->getDataObjWidth() - 1;

    QVector<QPointF> pts(2);
    
    bool check;

    double x0 = pt.x();
    double y0 = pt.y();

    if (x0 < 0) x0 = 0.0;
    else if (x0 > xmax) x0 = xmax;

    if (y0 < 0) y0 = 0.0;
    else if (y0 > ymax) y0 = ymax;

    double x1 = x0 + 1;
    double y1 = y0 + 1;

    m_pLineCut->setLine(x0, y0, x1, y1);

    int dim = m_ObjectContainer->getDataObject()->getDims()-1;
    pts[0].setX(m_ObjectContainer->getDataObject()->getPixToPhys(dim, x0, check));
    pts[0].setY(m_ObjectContainer->getDataObject()->getPixToPhys(dim, y0, check));

    dim = m_ObjectContainer->getDataObject()->getDims()-2;
    pts[1].setX(m_ObjectContainer->getDataObject()->getPixToPhys(dim, x1, check));
    pts[1].setY(m_ObjectContainer->getDataObject()->getPixToPhys(dim, y1, check));

    repaint();

    ((GraphicViewPlot*)m_pParent)->displayLineCut(pts, m_lineplotUID);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::trackerMoved(const QPointF &pt)
{
    if (!m_pLineCut) return;

    int ymax = m_ObjectContainer->getDataObjHeight() - 1;
    int xmax = m_ObjectContainer->getDataObjWidth() - 1;

    QVector<QPointF> pts(2);
    
    bool check;

    double x0 = m_pLineCut->line().x1();
    double y0 = m_pLineCut->line().y1();
    double x1 = pt.x();
    double y1 = pt.y();

    pts[1] = pt;

    if (x1 < 0) x1 = 0.0;
    else if (x1 > xmax) x1 = xmax;

    if (y1 < 0) y1 = 0.0;
    else if (y1 > ymax) y1 = ymax;

    m_pLineCut->setLine(x0, y0, x1, y1);

    int dim = m_ObjectContainer->getDataObject()->getDims()-1;
    pts[0].setX(m_ObjectContainer->getDataObject()->getPixToPhys(dim, x0, check));
    pts[0].setY(m_ObjectContainer->getDataObject()->getPixToPhys(dim, y0, check));

    dim = m_ObjectContainer->getDataObject()->getDims()-2;
    pts[1].setX(m_ObjectContainer->getDataObject()->getPixToPhys(dim, x1, check));
    pts[1].setY(m_ObjectContainer->getDataObject()->getPixToPhys(dim, y1, check));

    repaint();

    ((GraphicViewPlot*)m_pParent)->displayLineCut(pts, m_lineplotUID);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::trackerAScanAppended(const QPoint &pt)
{
/*
    QVector<QPointF> pts;

    QPointF scale;
    pts.resize(1);

    if (m_pContent && m_pContent->data())
    {
        pts[0].setY(invTransform(this->m_pContent->yAxis(), pt.y()));
        pts[0].setX(invTransform(this->m_pContent->xAxis(), pt.x()));

        QwtInterval interv = m_pContent->data()->interval(Qt::ZAxis);
        scale.setX(interv.minValue());
        scale.setY(interv.maxValue());
    }

    m_pAScanMarker->setValue(pts[0]);
    m_lineCut.setSamples(pts);

    ((itom2DQwtFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);
*/
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::trackerAScanMoved(const QPoint &pt)
{
/*
    QVector<QPointF> pts;
    pts.resize(1);

    if (m_pContent && m_pContent->data())
    {
        pts[0].setY(invTransform(this->m_pContent->yAxis(), pt.y()));
        pts[0].setX(invTransform(this->m_pContent->xAxis(), pt.x()));
    }
	m_pAScanMarker->setValue(pts[0]);
    m_lineCut.setSamples(pts);

    replot();
*/
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotWidget::setCanvasZoom(const int zoolLevel)
{
    int xsize = 1;
    int ysize = 1;

    if (m_ObjectContainer == NULL || m_pItem == NULL)
    {
        return ito::retError;
    }
    else
    {
        ysize = m_ObjectContainer->getDataObjHeight();
        xsize = m_ObjectContainer->getDataObjWidth();
    }
    switch(zoolLevel)
    {
        case PlotWidget::RatioOff:
            m_pData->m_zoomLevel = PlotWidget::RatioOff;
            //m_pItem->setScale(1.0);
            setMatrix(QMatrix(1, 0, 0, 1, 1, 1), false);
            fitInView(m_pItem, Qt::KeepAspectRatio);
            break;
        case PlotWidget::Ratio1_1:
            m_pData->m_zoomLevel = PlotWidget::Ratio1_1;
            setSceneRect (0.0, 0.0, xsize, ysize);
            setMatrix(QMatrix(1, 0, 0, 1, 1, 1), false);
            m_pItem->setScale(1.0);
            centerOn(m_pItem);
            break;
        case PlotWidget::Ratio1_2:
            m_pData->m_zoomLevel = PlotWidget::Ratio1_2;
            setSceneRect (0.0, 0.0, xsize, ysize);
            setMatrix(QMatrix(2, 0, 0, 2, 1, 1), false);
            m_pItem->setScale(1.0);
            centerOn(m_pItem);
            break;
        case PlotWidget::Ratio1_4:
            m_pData->m_zoomLevel = PlotWidget::Ratio1_4;
            setSceneRect (0.0, 0.0, xsize, ysize);
            setMatrix(QMatrix(4, 0, 0, 4, 1, 1), false);
            m_pItem->setScale(1.0);
            centerOn(m_pItem);
            break;
        case PlotWidget::Ratio2_1:
            m_pData->m_zoomLevel = PlotWidget::Ratio2_1;
            setSceneRect (0.0, 0.0, xsize, ysize);
            setMatrix(QMatrix(0.5, 0, 0, 0.5, 1, 1), false);
            m_pItem->setScale(1.0);
            centerOn(m_pItem);
            break;
        case PlotWidget::Ratio4_1:
            m_pData->m_zoomLevel = PlotWidget::Ratio4_1;
            setSceneRect (0.0, 0.0, xsize, ysize);
            setMatrix(QMatrix(0.25, 0, 0, 0.25, 1, 1), false);
            m_pItem->setScale(1.0);
            centerOn(m_pItem);
            break;
    }

    repaint();
    return ito::retError;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::updatePointTracker()
{
    if (!m_ObjectContainer || !m_pValuePicker)
        return;

    char buf[60] = {0};

    int y1 = m_ObjectContainer->getDataObjHeight() - 1;
    int x1 = m_ObjectContainer->getDataObjWidth() - 1;

    if (x1 < 0 || y1 < 0)
        return;

    QPointF cursorPos = m_pValuePicker->pos();
    

    if (cursorPos.x() < 0)
    {
        cursorPos.setX(0);
    }
    else if (cursorPos.x() > x1)
    {
        cursorPos.setX(x1);
    }

    if (cursorPos.y() < 0)
    {
        cursorPos.setY(0);
    }
    else if (cursorPos.y() > y1)
    {
        cursorPos.setY(y1);
    }

    m_pValuePicker->setPos(cursorPos);

    bool isInt = false;

    if ((m_pData->m_colorMode & (RasterToQImageObj::ColorRGB24 | RasterToQImageObj::ColorRGB32)))
    {
        unsigned char A = 0;
        unsigned char R = 0;
        unsigned char G = 0;
        unsigned char B = 0;

        ito::float64 value = m_ObjectContainer->getPixelARGB(cursorPos, A, R, G, B);

        if (m_pData->m_colorMode & RasterToQImageObj::ColorRGB24)
        {
            sprintf(buf, "[%i; %i]\n %i, %i, %i", (int)cursorPos.x(), (int)cursorPos.y(), R, G, B);
        }
        else
        {
            sprintf(buf, "[%i; %i]\n %i, %i, %i, %i", (int)cursorPos.x(), (int)cursorPos.y(), A, R, G, B);
        }
        //m_pointMarker->setPen(QPen(QColor(255-R, 255-G, 255-B)));
    }
    else
    {
        ito::float64 value = m_ObjectContainer->getPixel(cursorPos, isInt, m_pData->m_cmplxType);
        if (isInt)
        {
            sprintf(buf, "[%i; %i]\n %i", (int)cursorPos.x(), (int)cursorPos.y(), (int)value);
    
        }
        else
        {
            sprintf(buf, "[%i; %i]\n %.4g", (int)cursorPos.x(), (int)cursorPos.y(), value);
        }
    }
    //m_pointTracker->setHtml("<div style=\"background:#ff8800;\">html item</p>");
    m_pValuePicker->setText(buf);
    m_pValuePicker->setPos(cursorPos);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::enableMarker(const bool enabled)
{
    m_pValuePicker->setVisible(enabled);

    if (!enabled) m_trackerIsSampling = false;

    updatePointTracker();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::enableLinePointer(const bool enabled)
{
    m_pLineCut->setVisible(enabled);
    if (!enabled) m_lineIsSampling = false;
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotWidget::init()
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
bool PlotWidget::setColorMap(QString colormap /*= "__next__"*/)
{
    ito::ItomPalette newPalette;
    ito::RetVal retval(ito::retOk);
    int numPalettes = 1;

    retval += apiPaletteGetNumberOfColorBars(numPalettes);

    if (numPalettes == 0 || retval.containsError())
    {
        emit statusBarMessage( tr("No color maps defined."), 4000 );
        return false;
    }

    if(m_pData->m_colorMode == RasterToQImageObj::ColorRGB24 || m_pData->m_colorMode == RasterToQImageObj::ColorRGB32)
    {
        emit statusBarMessage( tr("Can not toogle colorbar while using RGB-Colors."), 4000 );
        return false;        
    }

    if (colormap == "__next__")
    {
        m_pData->m_paletteNum++;
        m_pData->m_paletteNum %= numPalettes; //map index to [0,numPalettes)
        retval += apiPaletteGetColorBarIdx(m_pData->m_paletteNum, newPalette);
    }
    else if (colormap == "__first__")
    {
        m_pData->m_paletteNum = 0;
        retval += apiPaletteGetColorBarIdx(m_pData->m_paletteNum, newPalette);
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

    if(newPalette.type == ito::tPaletteNoType || newPalette.colorVector256.size() < 256)
    {
        emit statusBarMessage( "Selected color bar invalid", 4000 );
        return false;
    }
    else
    {
        m_pData->m_colorTable.clear();
        m_pData->m_colorTable.resize(256);
        for(int i = 0; i < 256; i++)
        {
            m_pData->m_colorTable[i] = newPalette.colorVector256[i] | 0xFF000000;
        }
        m_pData->m_inverseColor0 = newPalette.inverseColorOne;
        m_pData->m_inverseColor1 = newPalette.inverseColorTwo;
        ((GraphicViewPlot*)m_pParent)->setPaletteText(newPalette.name);
    }

    refreshStyles();


    refreshPlot(NULL);
    return true;
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::refreshStyles()
{
    //QPen rubberBandPen = apiGetFigureSetting(m_pParent, "zoomRubberBandPen", QPen(QBrush(Qt::red),1,Qt::DashLine),NULL).value<QPen>();
    QPen trackerPen = apiGetFigureSetting(parent(), "trackerPen", QPen(QBrush(Qt::red),2),NULL).value<QPen>();
    QFont trackerFont = apiGetFigureSetting(parent(), "trackerFont", QFont("Verdana",10),NULL).value<QFont>();
    //QBrush trackerBg = apiGetFigureSetting(parent(), "trackerBackground", QBrush(QColor(255,255,255,155), Qt::SolidPattern),NULL).value<QBrush>();
    //QPen selectionPen = apiGetFigureSetting(parent(), "selectionPen", QPen(QBrush(Qt::gray),2,Qt::SolidLine),NULL).value<QPen>();

    //QFont titleFont = apiGetFigureSetting(parent(), "titleFont", QFont("Helvetica",12),NULL).value<QFont>();
    //QFont labelFont = apiGetFigureSetting(parent(), "labelFont", QFont("Helvetica",12),NULL).value<QFont>();
    //labelFont.setItalic(false);
    //QFont axisFont = apiGetFigureSetting(parent(), "axisFont", QFont("Helvetica",10),NULL).value<QFont>();

    //rubberBandPen.setColor(m_pData->m_inverseColor1);

    //selectionPen.setColor(m_pData->m_inverseColor0);
    trackerPen.setColor(m_pData->m_inverseColor0);

    /*
    m_pZoomer->setRubberBandPen(rubberBandPen);
    m_pZoomer->setTrackerFont(trackerFont);
    m_pZoomer->setTrackerPen(trackerPen);
    */

    if (!m_pLineCut)
    {
        m_pLineCut = new QGraphicsLineItem(NULL, m_pContent);
        m_pLineCut->setVisible(false);
        m_lineIsSampling = false;

        m_pLineCut->setPen(trackerPen);
    }
    else
    {
        m_pLineCut->setPen(trackerPen);    
    }

    if (!m_pValuePicker)
    {
        m_pValuePicker = new QGraphicsViewValuePicker("[0.0; 0.0]\n 0.0", m_pContent);
        m_pValuePicker->setColor(m_pData->m_inverseColor0);
        m_pValuePicker->setVisible(false);
    }
    else
    {
        m_pValuePicker->setColor(m_pData->m_inverseColor0);
    }

    //m_pStackCutMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(m_inverseColor1), QPen(QBrush(m_inverseColor1),3),  QSize(7,7) ));
    
    //title().setFont(titleFont);

//    axisTitle(QwtPlot::xBottom).setFont(axisFont);
//    axisTitle(QwtPlot::yLeft).setFont(axisFont);
//    axisTitle(QwtPlot::yRight).setFont(axisFont);

//    QwtText t = axisWidget(QwtPlot::xBottom)->title();
//    t.setFont(labelFont);
//    axisWidget(QwtPlot::xBottom)->setTitle(t);

//    t = axisWidget(QwtPlot::yLeft)->title();
//    t.setFont(labelFont);
//    axisWidget(QwtPlot::yLeft)->setTitle(t);

//    t = axisWidget(QwtPlot::yRight)->title();
//    t.setFont(labelFont);
//    axisWidget(QwtPlot::yRight)->setTitle(t);
    repaint();
}
void PlotWidget::updateLabels()
{

}
void PlotWidget::enableAxis(const int axis, const bool value)
{

}

QPointF PlotWidget::calcInterval(const int axis) const
{

    if (!m_ObjectContainer || m_ObjectContainer->getDataObject().isNull())
        return QPointF(0.0, 1.0);

    switch(axis)
    {
        case Qt::XAxis:
        {
            int dim = m_ObjectContainer->getDataObject()->getDims() - 1;
            bool check;
            double x0 = m_ObjectContainer->getDataObject()->getPixToPhys(dim, 0.0, check);
            double x1 = m_ObjectContainer->getDataObject()->getPixToPhys(dim, m_ObjectContainer->getDataObjWidth(), check);
            return QPointF(x0, x1);
        }
        case Qt::YAxis:
        {
            int dim = m_ObjectContainer->getDataObject()->getDims() - 2;
            bool check;
            double y0 = m_ObjectContainer->getDataObject()->getPixToPhys(dim, 0.0, check);
            double y1 = m_ObjectContainer->getDataObject()->getPixToPhys(dim, m_ObjectContainer->getDataObjHeight(), check);
            return QPointF(y0, y1);
        }
        case Qt::ZAxis:
        {
            ito::float64 z0 = 0.0;
            ito::float64 z1 = 1.0;
            ito::uint32 loc0[3], loc1[3];
            ito::dObjHelper::minMaxValue(m_ObjectContainer->getDataObject().data(), z0, loc0, z1, loc1, true, m_pData->m_cmplxType);
            return QPointF(z0, z1);
        }
    }
    return QPointF(0.0, 1.0);
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotWidget::setState( tState state)
{
    GraphicViewPlot *p = (GraphicViewPlot*)(this->parent());

    /*
    m_pCenterMarker->setVisible(m_pData->m_showCenterMarker);
    if(m_pData->m_showCenterMarker && m_dObjPtr)
    {
        if(m_dObjPtr->getDims() > 1)
        {
            bool valid;
            m_pCenterMarker->setXValue(m_dObjPtr->getPixToPhys( m_dObjPtr->getDims()-1, (m_dObjPtr->getSize(m_dObjPtr->getDims()-1) - 1) / 2.0, valid));
            m_pCenterMarker->setYValue(m_dObjPtr->getPixToPhys( m_dObjPtr->getDims()-2, (m_dObjPtr->getSize(m_dObjPtr->getDims()-2) - 1) / 2.0, valid));
        }
        
    }
    */
    if (m_pData->m_state != state)
    {

        /*
        if ((m_pData->m_state == tMultiPointPick || m_pData->m_state == tPoint
            || m_pData->m_state == tLine || m_pData->m_state == tRect || m_pData->m_state == tEllipse) && state != tIdle)
        {
            return; //multiPointPick needs to go back to idle
        }
        */

        //if (m_pZoomer) m_pZoomer->setEnabled( state == tZoom );
        //if (m_pPanner) m_pPanner->setEnabled( state == tPan );

        if (m_pValuePicker) enableMarker( state == tValuePicker );
        if (m_pLineCut) enableLinePointer( state == tLineCut );

        //if (m_pStackPicker) m_pStackPicker->setEnabled( state == tStackCut );
        
        //if (m_pMultiPointPicker) m_pMultiPointPicker->setEnabled( state == tMultiPointPick );

        
        if (/*state == tMultiPointPick || m_pData->m_state == tPoint || m_pData->m_state == tLine
            || m_pData->m_state == tRect || m_pData->m_state == tEllipse || */ state == tIdle)
        {
            if (p)
            {
                //p->m_pActZoom->setEnabled(state == tIdle);
                p->m_pActPan->setEnabled(state == tIdle);
                p->m_pActLineCut->setEnabled(state == tIdle);
                //p->m_pActStackCut->setEnabled(state == tIdle);
                p->m_pActValuePicker->setEnabled(state == tIdle);
            }
        }

        if (/*state == tZoom || state == tPan || state == tMultiPointPick || m_pData->m_state == tPoint || m_pData->m_state == tLine
            || m_pData->m_state == tRect || m_pData->m_state == tEllipse ||*/ state == tValuePicker || state == tIdle)
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
                setCursor( Qt::ArrowCursor );
            break;
            /*
            case tZoom:
                setCursor( Qt::CrossCursor );
            break;

            case tPan:
                setCursor( Qt::OpenHandCursor );
            break;
            */
            case tValuePicker:
                setCursor( Qt::CrossCursor );
            break;

            case tStackCut:
                setCursor( Qt::CrossCursor );
            break;
            /*
            case tMultiPointPick:
                setCursor( Qt::CrossCursor );
            break;
            
            case tPoint:
            case tLine:
            case tRect:
            case tEllipse:
                setCursor( Qt::CrossCursor );
            break;
            */
        }

        m_pData->m_state = state;
    }
    repaint();
}

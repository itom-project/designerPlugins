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

#include "itom2DGVFigure.h"
#include "common/sharedStructuresGraphics.h"

#include "DataObject/dataObjectFuncs.h"

#include <qimage.h>

#include <qpixmap.h>
#include <qdebug.h>
#include <qmessagebox.h>


using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
plot2DWidget::plot2DWidget(QMenu *contextMenu, QWidget * parent) :
        QGraphicsView(parent),
        m_contextMenu(contextMenu),
        m_pParent(parent),
        m_paletteNum(0),
        m_lineplotUID(0),
        m_startScaledX(false),
        m_startScaledY(false),
        m_startScaledZ(false),
        m_pItem(NULL),
        m_pLineCut(NULL),
        //m_pTracker(NULL),
        m_pContent(NULL),
        m_ObjectContainer(NULL),
        m_showColored(RasterToQImageObj::ColorIndex8Bitshift),
        m_fixedZoom(false),
        m_lineIsSampling(false),
        m_trackerIsSampling(false),
        m_cmplxState(false),
        m_stackState(false),
        m_stateMoveAligned(false),
        m_pointTracker(NULL),
        m_pointMarker(NULL)
{
    this->setMouseTracking(false); //(mouse tracking is controled by action in WinMatplotlib)

    m_pContent = new QGraphicsScene(this);
    setScene(m_pContent);
    //refreshColorMap();
}

//----------------------------------------------------------------------------------------------------------------------------------
plot2DWidget::~plot2DWidget()
{
    delete m_ObjectContainer;
    m_ObjectContainer = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void plot2DWidget::handleMouseEvent(int type, QMouseEvent *event)
{
    Qt::MouseButton btn = event->button();
    Qt::MouseButtons btns = event->buttons();
//    int button = 0;
    QPointF scenePos = mapToScene(event->pos());
    
    if (!m_pContent || !m_pItem || !m_pLineCut || !m_pointMarker)
        return;

    switch(type)
    {
        case 0:
            switch(btn)
            {
                case Qt::LeftButton:
                {
                    if (m_pLineCut->isVisible())
                    {
                        trackerAppended(scenePos);
                        m_lineIsSampling = true;
                    }
                    else if (m_pointMarker->isVisible())
                    {
                        m_pointMarker->setPos(scenePos - QPointF(1.0, 1.0));
                        updatePointTracker();
                        m_trackerIsSampling  = true;
                    }
                }
                break;
            }

        break;
        case 1:
            if (m_pLineCut->isVisible())
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
                    if (abs(scenePos.x() - m_pointMarker->pos().x()) > abs(scenePos.y() - m_pointMarker->pos().y()))
                    {
                        scenePos.setY(m_pointMarker->pos().y() + 1.0);
                    }
                    else
                    {
                        scenePos.setX(m_pointMarker->pos().x() + 1.0);
                    }
                }

                m_pointMarker->setPos(scenePos - QPointF(1.0, 1.0));
                updatePointTracker();          
            }
        }
        break;
        case 3:
            if (m_pLineCut->isVisible())
            {
                m_lineIsSampling = false;
            }
            else if (m_pointMarker->isVisible())
            {
                m_trackerIsSampling = false;        
            }

        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void plot2DWidget::refreshPlot(ito::ParamBase *param)
{
//    bool newObjectContainer = false;

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
                if (!m_cmplxState) ((itom2DGVFigure*)m_pParent)->enableComplexGUI(true);
                m_cmplxState = true;                
            }
            else
            {
                if (!m_cmplxState) ((itom2DGVFigure*)m_pParent)->enableComplexGUI(false);
                m_cmplxState = false;                
            }

            if (m_ObjectContainer == NULL)
            {
                QRect ROI(0, 0, dataObj->getSize(dims - 1), dataObj->getSize(dims - 2));
                m_ObjectContainer = new RasterToQImageObj(QSharedPointer<ito::DataObject>(new ito::DataObject(*dataObj)), ROI, /*dims -1, dims-2,*/ 1); 
//                newObjectContainer = true;

                if (m_startScaledZ) m_ObjectContainer->setIntervalRange(Qt::ZAxis, false, m_startRangeZ.x(), m_startRangeZ.y());
                if (m_startScaledY) m_ObjectContainer->setIntervalRange(Qt::YAxis, false, m_startRangeY.x(), m_startRangeY.y());
                if (m_startScaledX) m_ObjectContainer->setIntervalRange(Qt::XAxis, false, m_startRangeX.x(), m_startRangeX.y());
                
                ito::ItomPalette newPalette;
                apiPaletteGetColorBarIdx(m_paletteNum, newPalette);

                if (newPalette.type != tPaletteNoType) m_ObjectContainer->setColorTable(newPalette.colorVector256);
                //refreshColorMap();
            }
            else
            {
                m_ObjectContainer->updateDataObject(QSharedPointer<ito::DataObject>(new ito::DataObject(*dataObj)));
            }

            m_pixMap.convertFromImage(m_ObjectContainer->getRastersImage());               

            if (!m_pItem)
            {
                m_pItem = new QGraphicsPixmapItem(m_pixMap);
                m_pContent->clear();
                m_pContent->addItem((QGraphicsItem*)m_pItem);
                fitInView(m_pItem, Qt::KeepAspectRatio);
            }
            else
                m_pItem->setPixmap(m_pixMap);

            if (!m_pLineCut)
            {
                m_pLineCut = new QGraphicsLineItem(NULL, m_pContent);
                m_pLineCut->setVisible(false);
                m_lineIsSampling = false;

                ito::ItomPalette newPalette;
                apiPaletteGetColorBarIdx(m_paletteNum, newPalette);

                m_pLineCut->setPen(QPen(QColor(newPalette.inverseColorOne)));
            }

            if (!m_pointMarker)
            {
                m_pointMarker = new QGraphicsEllipseItem(0.0, 0.0, 2.0, 2.0, NULL, m_pContent);
                m_pointMarker->setVisible(false);

                ito::ItomPalette newPalette;
                apiPaletteGetColorBarIdx(m_paletteNum, newPalette);


                m_pointMarker->setPen(QPen(QColor(newPalette.inverseColorOne)));
            }

            if (!m_pointTracker)
            {
                m_pointTracker = new QGraphicsTextItem("[0.0; 0.0]\n 0.0", NULL, m_pContent);

                ito::ItomPalette newPalette;
                apiPaletteGetColorBarIdx(m_paletteNum, newPalette);

                m_pointTracker->setDefaultTextColor(newPalette.inverseColorOne);
                m_pointTracker->setVisible(false);
            }
            else
            {
                if (m_pointMarker->isVisible())
                {
                    updatePointTracker();
                }
            }

            repaint();
        }
    }
    else if (m_ObjectContainer != NULL)
    {
        m_pixMap.convertFromImage(m_ObjectContainer->getRastersImage());               

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
void plot2DWidget::keyReleaseEvent (QKeyEvent * event)
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
void plot2DWidget::keyPressEvent (QKeyEvent * event) 
{
    if (!hasFocus())
    {
        return;
    }

    if (!m_pContent || !m_pItem || !m_pLineCut || !m_pointMarker)
    {
        return;
    }

    if (!m_ObjectContainer || !(m_pLineCut->isVisible() || m_pointMarker->isVisible()))
    {
        return;
    }

    switch(((const QKeyEvent *)event)->key())
    {
        case Qt::Key_Up:
        { 
            if (m_pointMarker->isVisible())   // doing the linecut
            {
                QPointF scenePos = m_pointMarker->pos() - QPointF(0.0, 1.0);
                if (scenePos.y() < -1.0) scenePos.setY(-1.0);

                m_pointMarker->setPos(scenePos);
                updatePointTracker();
            }
            else if (m_pLineCut->isVisible())
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
       
            if (m_pointMarker->isVisible())   // doing the linecut
            {
                QPointF scenePos = m_pointMarker->pos() + QPointF(0.0, 1.0);
                if (scenePos.y() > (y1 - 1)) scenePos.setY(y1 - 1);

                m_pointMarker->setPos(scenePos);
                updatePointTracker();
            }
            else if (m_pLineCut->isVisible())
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
 
            if (m_pointMarker->isVisible())   // doing the linecut
            {
                QPointF scenePos = m_pointMarker->pos() + QPointF(1.0, 0.0);
                if (scenePos.x() > (x1-1.0)) scenePos.setX(x1-1.0);

                m_pointMarker->setPos(scenePos);
                updatePointTracker();
            }
            else if (m_pLineCut->isVisible())
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
            if (m_pointMarker->isVisible())   // doing the linecut
            {
                QPointF scenePos = m_pointMarker->pos() - QPointF(1.0, 0.0);
                if (scenePos.x() < -1.0) scenePos.setX(-1.0);

                m_pointMarker->setPos(scenePos);
                updatePointTracker();
            }
            else if (m_pLineCut->isVisible())
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

//            bool test = true;
            double yCenter = m_ObjectContainer->getDataObject()->getSize(dims-2) / 2.0;
            double xMin = 0.0;
            double xMax = m_ObjectContainer->getDataObject()->getSize(dims-1) - 1;

            if (m_pointMarker->isVisible())   // doing the linecut
            {
                QPointF scenePos = m_pointMarker->pos();
                scenePos.setY(yCenter - 1.0);
                if (scenePos.y() < -1.0) scenePos.setY(-1.0);

                m_pointMarker->setPos(scenePos);
                updatePointTracker();
            }
            else if (m_pLineCut->isVisible())
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

//            bool test = true;
            double xCenter = m_ObjectContainer->getDataObject()->getSize(dims-1) / 2.0;
            double yMin = 0.0;
            double yMax = m_ObjectContainer->getDataObject()->getSize(dims-2) - 1;

            if (m_pointMarker->isVisible())   // doing the linecut
            {
                QPointF scenePos = m_pointMarker->pos();
                scenePos.setX(xCenter - 1.0);
                if (scenePos.x() < -1.0) scenePos.setX(-1.0);

                m_pointMarker->setPos(scenePos);
                updatePointTracker();
            }
            else if (m_pLineCut->isVisible())
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
void plot2DWidget::mouseMoveEvent (QMouseEvent * event)
{
    if (!hasFocus())
    {
        return;
    }
    handleMouseEvent(2, event);
    event->accept();
}

//----------------------------------------------------------------------------------------------------------------------------------
void plot2DWidget::mousePressEvent (QMouseEvent * event)
{
    if (!hasFocus())
    {
        return;
    }
    handleMouseEvent(0, event);
    event->accept();
}

//----------------------------------------------------------------------------------------------------------------------------------
void plot2DWidget::mouseReleaseEvent (QMouseEvent * event)
{
    if (!hasFocus())
        return;
    QApplication::restoreOverrideCursor();
    handleMouseEvent(3, event);
    event->accept();
}

//----------------------------------------------------------------------------------------------------------------------------------
void plot2DWidget::contextMenuEvent(QContextMenuEvent * event)
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
void plot2DWidget::resizeEvent(QResizeEvent * event)
{
    if (m_pItem)
    {
        if (!m_fixedZoom) fitInView(m_pItem, Qt::KeepAspectRatio);
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
void plot2DWidget::refreshColorMap(QString palette)
{
    ito::ItomPalette newPalette;
    ito::RetVal retval(ito::retOk);
    int numPalettes = 1;
    bool isFalseColor = true;

	if (ITOM_API_FUNCS_GRAPH == NULL)
    {
		return;
    }

    if (palette.isEmpty())
    {
        retval = apiPaletteGetNumberOfColorBars(numPalettes);
        m_paletteNum %= numPalettes;
        //if (m_paletteNum == 0)
        //    return;

        retval += apiPaletteGetColorBarIdx(m_paletteNum, newPalette);
        //((itom2DGVFigure*)m_pParent)->setPaletteText(newPalette.getName());
        ((itom2DGVFigure*)m_pParent)->setPaletteText(newPalette.name);
    }
    else if (!palette.compare("RGB24", Qt::CaseInsensitive))
    {
        if (m_ObjectContainer != NULL)
        {
            m_ObjectContainer->setColorMode(RasterToQImageObj::ColorRGB24);
        }
        m_showColored = RasterToQImageObj::ColorRGB24;
        ((itom2DGVFigure*)m_pParent)->setPaletteText("RGB24");
        isFalseColor = false;
    }
    else if (!palette.compare("RGB32", Qt::CaseInsensitive))
    {
        if (m_ObjectContainer != NULL)
        {
            m_ObjectContainer->setColorMode(RasterToQImageObj::ColorRGB32);
        }
        m_showColored = RasterToQImageObj::ColorRGB32;
        ((itom2DGVFigure*)m_pParent)->setPaletteText("RGB32");
        isFalseColor = false;
    }
    else
    {
        retval += apiPaletteGetColorBarName(palette, newPalette);
        retval += apiPaletteGetColorBarIdxFromName(palette, m_paletteNum);
        //((itom2DGVFigure*)m_pParent)->setPaletteText(newPalette.getName());
        ((itom2DGVFigure*)m_pParent)->setPaletteText(newPalette.name);
    }
    
    if (newPalette.colorVector256.isEmpty())
    {
        return;
    }
    if (newPalette.inverseColorOne.isValid() && m_pLineCut)
    {
        m_pLineCut->setPen(QPen(newPalette.inverseColorOne));
        //m_pLineCut->set
    }
    if (newPalette.inverseColorOne.isValid() && m_pointMarker)
    {
        m_pointMarker->setPen(QPen(QColor(newPalette.inverseColorOne)));
        //m_pLineCut->set
    }
    if (newPalette.inverseColorOne.isValid() && m_pointTracker)
    {
        m_pointTracker->setDefaultTextColor(QColor(newPalette.inverseColorOne));
        //m_pLineCut->set
    }    
    if (isFalseColor)
    {
        //QVector<ito::uint32> colors(newPalette.get256Colors());
        if (m_ObjectContainer != NULL)
        {
            //m_ObjectContainer->setColorTable(colors);
            m_ObjectContainer->setColorTable(newPalette.colorVector256);
        }
    }
    ito::ParamBase temp("newPalette", ito::ParamBase::Int, 1);
    refreshPlot(&temp);
}

//----------------------------------------------------------------------------------------------------------------------------------
void plot2DWidget::trackerAppended(const QPointF &pt)
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

    ((itom2DGVFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);
}

//----------------------------------------------------------------------------------------------------------------------------------
void plot2DWidget::trackerMoved(const QPointF &pt)
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

    ((itom2DGVFigure*)m_pParent)->displayLineCut(pts, m_lineplotUID);
}

//----------------------------------------------------------------------------------------------------------------------------------
void plot2DWidget::trackerAScanAppended(const QPoint &pt)
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
void plot2DWidget::trackerAScanMoved(const QPoint &pt)
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
ito::RetVal plot2DWidget::setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue)
{
    if (m_ObjectContainer)
    {
        m_ObjectContainer->setIntervalRange(axis, autoCalcLimits, minValue, maxValue);
        ito::ParamBase temp("newPalette", ito::ParamBase::Int, 1);
        refreshPlot(&temp);
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

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------

ito::RetVal plot2DWidget::setCanvasZoom(const int zoolLevel)
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
        case plot2DWidget::RatioOff:
            m_fixedZoom = false;
            //m_pItem->setScale(1.0);
            setMatrix(QMatrix(1, 0, 0, 1, 1, 1), false);
            fitInView(m_pItem, Qt::KeepAspectRatio);
            break;
        case plot2DWidget::Ratio1_1:
            m_fixedZoom = true;
            setSceneRect (0.0, 0.0, xsize, ysize);
            setMatrix(QMatrix(1, 0, 0, 1, 1, 1), false);
            m_pItem->setScale(1.0);
            centerOn(m_pItem);
            break;
        case plot2DWidget::Ratio1_2:
            m_fixedZoom = true;
            setSceneRect (0.0, 0.0, xsize, ysize);
            setMatrix(QMatrix(2, 0, 0, 2, 1, 1), false);
            m_pItem->setScale(1.0);
            centerOn(m_pItem);
            break;
        case plot2DWidget::Ratio1_4:
            m_fixedZoom = true;
            setSceneRect (0.0, 0.0, xsize, ysize);
            setMatrix(QMatrix(4, 0, 0, 4, 1, 1), false);
            m_pItem->setScale(1.0);
            centerOn(m_pItem);
            break;
        case plot2DWidget::Ratio2_1:
            m_fixedZoom = true;
            setSceneRect (0.0, 0.0, xsize, ysize);
            setMatrix(QMatrix(0.5, 0, 0, 0.5, 1, 1), false);
            m_pItem->setScale(1.0);
            centerOn(m_pItem);
            break;
        case plot2DWidget::Ratio4_1:
            m_fixedZoom = true;
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
void plot2DWidget::updatePointTracker()
{
    if (!m_ObjectContainer || !m_pointTracker || !m_pointMarker)
        return;

    char buf[50] = {0};

    int y1 = m_ObjectContainer->getDataObjHeight() - 1;
    int x1 = m_ObjectContainer->getDataObjWidth() - 1;

    if (x1 < 0 || y1 < 0)
        return;

    QPointF cursorPos = m_pointMarker->pos() + QPointF(1.0, 1.0);
    

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

    m_pointMarker->setPos(cursorPos - QPointF(1.0, 1.0));

    bool isInt = false;
    int mode = m_ObjectContainer->getColorMode();

    if ((mode & (m_ObjectContainer->ColorRGB24 | m_ObjectContainer->ColorRGB32)))
    {
        unsigned char A = 0;
        unsigned char R = 0;
        unsigned char G = 0;
        unsigned char B = 0;

        ito::float64 value = m_ObjectContainer->getPixelARGB(cursorPos, A, R, G, B);

        if (mode & m_ObjectContainer->ColorRGB24)
        {
            sprintf(buf, "[%i; %i]\n %i, %i, %i", (int)cursorPos.x(), (int)cursorPos.y(), R, G, B);
        }
        else
        {
            sprintf(buf, "[%i; %i]\n %i, %i, %i, %i", (int)cursorPos.x(), (int)cursorPos.y(), A, R, G, B);
        }
        m_pointMarker->setPen(QPen(QColor(255-R, 255-G, 255-B)));
    }
    else
    {
        ito::float64 value = m_ObjectContainer->getPixel(cursorPos, isInt);
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
    m_pointTracker->setPlainText(buf);
    m_pointTracker->setPos(cursorPos);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void plot2DWidget::enableMarker(const bool enabled)
{
    m_pointTracker->setVisible(enabled);
    m_pointMarker->setVisible(enabled);

    if (!enabled) m_trackerIsSampling = false;

    updatePointTracker();
}

//----------------------------------------------------------------------------------------------------------------------------------
void plot2DWidget::enableLinePointer(const bool enabled)
{
    m_pLineCut->setVisible(enabled);
    if (!enabled) m_lineIsSampling = false;
    return;
}
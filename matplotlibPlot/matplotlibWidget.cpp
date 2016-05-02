/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2016, Institut fuer Technische Optik (ITO), 
   Universitaet Stuttgart, Germany 
 
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

#include "matplotlibWidget.h"

#include <qimage.h>
#include <qpixmap.h>
#include <qdebug.h>
#include "matplotlibplot.h"

//-------------------------------------------------------------------------------------
MatplotlibWidget::MatplotlibWidget(QMenu *contextMenu, QWidget * parent) :
        QGraphicsView(parent),
        m_trackerActive(false),
        m_internalResize(false),
        m_keepSizeFixed(false),
        m_scene(NULL),
        m_rectItem(NULL),
        m_pixmapItem(NULL),
        m_contextMenu(contextMenu)/*,
        m_externalSizeHint(0,0)*/
{
    //setBackgroundBrush(QBrush(Qt::red));
    m_scene = new QGraphicsScene(this);
    setScene(m_scene);

    this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    this->setMouseTracking(false); //only react on mouse-move events between a mouse-click and a subsequent release

    //create empty pixmap
    m_pixmap = QPixmap(20,20);
    m_pixmap.fill(Qt::white);

    //create pixmap item on scene
    m_pixmapItem = m_scene->addPixmap(m_pixmap);
    m_pixmapItem->setOffset(0,0);
    m_pixmapItem->setVisible(true);

    //create rectangle item on scene
    m_rectItem = m_scene->addRect(0,0,100,200,QPen(Qt::black, 1, Qt::DotLine));
    m_rectItem->setVisible(false);

    m_timer.setSingleShot(true);
    QObject::connect(&m_timer, SIGNAL(timeout()), this, SLOT(paintTimeout()));
};

//-------------------------------------------------------------------------------------
MatplotlibWidget::~MatplotlibWidget()
{
}

//QSize MatplotlibWidget::sizeHint() const
//{
//    /*if(!m_externalSizeHint.isNull())
//    {
//        return m_externalSizeHint;
//    }*/
//    return QGraphicsView::sizeHint();
//}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::externalResize(int width, int height)
{
    //QSize oldSize = this->size();
    //m_externalSizeHint = QSize(width,height);

    //QSizePolicy::Policy hPol = QSizePolicy::Fixed; //oldSize.width() < sizeHint().width() ? QSizePolicy::Minimum : QSizePolicy::Maximum;
    //QSizePolicy::Policy vPol = QSizePolicy::Fixed; //oldSize.height() < sizeHint().height() ? QSizePolicy::Minimum : QSizePolicy::Maximum;

    //setSizePolicy(hPol, vPol);    
    //updateGeometry();

    MatplotlibPlot *parent = qobject_cast<MatplotlibPlot*>(this->parent());
    if(parent)
    {
        m_internalResize = true;
        parent->resizeCanvas(width,height);
        m_internalResize = false;
    }

    //resize(width,height);
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::paintResult(QSharedPointer<char> imageString, int x, int y, int w, int h, bool blit )
{
    int imgHeight = 0;
    int imgWidth = 0;

    m_timer.stop();
    
    if(blit == false)
    {
        //qDebug() << "size: " << w << ", " << h << ", imageString-size:" << imageString.length() << " win: " << width() << ", " << height();
        QImage image = QImage((uchar*)imageString.data(),w,h,QImage::Format_ARGB32);

        m_pixmap = QPixmap::fromImage(image);
        m_pixmapItem->setPixmap(m_pixmap);
        m_pixmapItem->setOffset(x,y);
        m_pixmapItem->update();
    }
    else
    {
        //check sizes
        
        imgHeight = m_pixmap.height();
        imgWidth = m_pixmap.width();

        if(x>=0 && y>=0 && imgHeight >= (y+h) && imgWidth >= (x+w))
        {
            QPainter painter(&m_pixmap);
            QImage image = QImage((uchar*)imageString.data(),w,h,QImage::Format_ARGB32);
            //painter.fillRect(x,y,w,h,QBrush(Qt::red));
            painter.drawImage(QPoint(x,y),image);
            painter.end();
            m_pixmapItem->setPixmap(m_pixmap);
            //m_pixmapItem->setOffset(x,y);
            m_pixmapItem->update();
        }

    }

    paintRect(false);
    
    QSize s = size();

    if ((abs(m_pixmap.width() - s.width()) < 6 && abs(m_pixmap.height() - s.height()) < 6))
    {
        setTransform( QTransform(1,0,0,1,0,0), false );
        centerOn( m_pixmapItem->boundingRect().center() );
        //qDebug() << "centerOn" << m_pixmapItem->boundingRect() << s << w << h;
    }
    else
    {
        fitInView(m_pixmapItem,Qt::IgnoreAspectRatio);
        //qDebug() << "fitInView" << m_pixmapItem->boundingRect() << s << w << h;

        if (m_keepSizeFixed && (abs(m_pixmap.width() - s.width()) != 0 || abs(m_pixmap.height() - s.height()) >> 0))
        {
            m_pendingEvent = PendingEvent(s.height(), s.width());
        }
    }
    
    //handle possible further update requests
    paintTimeout();

    emit eventIdle();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::paintResultDeprecated(QByteArray imageString, int x, int y, int w, int h, bool blit )
{
    int imgHeight = 0;
    int imgWidth = 0;

    m_timer.stop();
    
    if(blit == false)
    {
        //qDebug() << "size: " << w << ", " << h << ", imageString-size:" << imageString.length() << " win: " << width() << ", " << height();
        QImage image = QImage((uchar*)imageString.data(),w,h,QImage::Format_ARGB32);
        m_pixmap = QPixmap::fromImage(image);
        m_pixmapItem->setPixmap(m_pixmap);
        m_pixmapItem->setOffset(x,y);
        m_pixmapItem->update();
    }
    else
    {
        //check sizes
        
        imgHeight = m_pixmap.height();
        imgWidth = m_pixmap.width();

        if(x>=0 && y>=0 && imgHeight >= (y+h) && imgWidth >= (x+w))
        {
            QPainter painter(&m_pixmap);
            QImage image = QImage((uchar*)imageString.data(),w,h,QImage::Format_ARGB32);
            //painter.fillRect(x,y,w,h,QBrush(Qt::red));
            painter.drawImage(QPoint(x,y),image);
            painter.end();
            m_pixmapItem->setPixmap(m_pixmap);
            //m_pixmapItem->setOffset(x,y);
            m_pixmapItem->update();
        }

    }

    paintRect(false);
    
    QSize s = size();

    if (m_keepSizeFixed || (abs(m_pixmap.width() - s.width()) < 6 && abs(m_pixmap.height() - s.height()) < 6))
    {
        setTransform( QTransform(1,0,0,1,0,0), false );
        centerOn( m_pixmapItem->boundingRect().center() );
    }
    else
    {
        fitInView(m_pixmapItem,Qt::IgnoreAspectRatio);
    }
    
    //handle possible further update requests
    paintTimeout();

    emit eventIdle();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::paintRect(bool drawRect, int x, int y, int w, int h)
{
    if(drawRect == false)
    {
        if (m_rectItem->isVisible())
        {
            m_rectItem->setVisible(false);
        }
    }
    else
    {
        
        m_rectItem->setRect(x, y, w, h);
        //qDebug() << "Rect: " << x << y << w << h;
        m_rectItem->setVisible(true);
        m_rectItem->update();

        m_timer.stop();

        //handle possible further update requests
        paintTimeout();
        //qDebug() << "End Rect";
    }
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::resizeEvent ( QResizeEvent * event )
{
    if(m_internalResize == false)
    {
        if(m_pixmapItem)
        {            
            fitInView(m_pixmapItem,Qt::IgnoreAspectRatio);

            //qDebug() << "fitInView (resizeEvent)" << m_pixmapItem->boundingRect() << event->size();
        }

        m_pendingEvent = PendingEvent(event->size().height(), event->size().width());
        m_timer.start(60);
    }
    m_internalResize = false;
    
    QGraphicsView::resizeEvent(event);
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::paintTimeout()
{
    if(m_pendingEvent.isValid())
    {
        
        switch(m_pendingEvent.m_type)
        {
        case PendingEvent::typeResize:
            //qDebug() << "request resize event " << m_pendingEvent.m_w << " - " << m_pendingEvent.m_h;
            m_timer.start(2000); //if further update is required, it will be requested if the recent update has been transmitted or the timer runs into its timeout
            emit eventResize(m_pendingEvent.m_w, m_pendingEvent.m_h);
            m_pendingEvent.clear();
            break;
        case PendingEvent::typeMouseMove:
            //qDebug() << "timer active?" << m_timer.isActive();
            m_timer.start(2000); //if further update is required, it will be requested if the recent update has been transmitted or the timer runs into its timeout
            //qDebug() << "mouseMoveEvent" << m_pendingEvent.m_x << m_pendingEvent.m_y << m_pendingEvent.m_button;
            emit eventMouse(2, m_pendingEvent.m_x,m_pendingEvent.m_y, m_pendingEvent.m_button);
            m_pendingEvent.clear();
            break;
        }
    }
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::handleMouseEvent( int type, QMouseEvent *event)
{
    Qt::MouseButton btn = event->button();
    Qt::MouseButtons btns = event->buttons();
    int button = 0;
    QPointF scenePos = mapToScene( event->pos().x(), event->pos().y() );
    
    if(type == 2 /*&& button != 0*/) //move, handle by timer in order to not overload the repaint process in python (if no button is pressed, send immediately, since no repaint or rect-paint is pending)
    {
        
        if(btns & Qt::LeftButton)
        {
            button = 1;
        }
        else if(btns & Qt::RightButton)
        {
            button = 3;
        }
        else if(btns & Qt::MidButton)
        {
            button = 2;
        }

        if(button == 0) //no mouse button pressed, then handle mouse move event with lowest priority
        {
            if(!m_timer.isActive())
            {
                //m_pendingEvent = PendingEvent(event->pos().x(), event->pos().y(), button);
                m_pendingEvent = PendingEvent(qRound(scenePos.x()), qRound(scenePos.y()), button);
                paintTimeout();
            }
        }
        else
        {
            //m_pendingEvent = PendingEvent(qRound(event->pos().x()), qRound(event->pos().y()), button);
            m_pendingEvent = PendingEvent(qRound(scenePos.x()), qRound(scenePos.y()), button);
            if(!m_timer.isActive())
            {
                paintTimeout();
            }
        }
    }
    /*else if(type == 2 && button == 0)
    {
    }*/
    else
    {
        //TODO: NoButton, ... not handled
        switch(btn)
        {
            case Qt::LeftButton: button = 1; break;
            case Qt::RightButton: button = 3; break;
            case Qt::MiddleButton: button = 2; break;
        }
        //emit eventMouse(type, event->pos().x(), event->pos().y(), button);
        emit eventMouse(type, qRound(scenePos.x()), qRound(scenePos.y()), button);
    }
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::keyPressEvent ( QKeyEvent * event )
{
    if (!hasFocus())
        return;

    emit eventKey(0, event->key(), event->text(), event->isAutoRepeat());
    event->accept();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::keyReleaseEvent ( QKeyEvent * event )
{
    if (!hasFocus())
        return;
    emit eventKey(1, event->key(), event->text(), event->isAutoRepeat());
    event->accept();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::leaveEvent ( QEvent * /*event*/ )
{
    if (!hasFocus())
        return;
    emit eventLeaveEnter(0);
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::enterEvent ( QEvent * /*event*/ )
{
    if (!hasFocus())
        return;
    emit eventLeaveEnter(1);
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::wheelEvent( QWheelEvent * event )
{
    if (!hasFocus())
        return;
    QPointF scenePos = mapToScene( event->pos().x(), event->pos().y() );
    if(event->orientation() == Qt::Vertical)
    {
        emit eventWheel(qRound(scenePos.x()), qRound(scenePos.y()), event->delta(), 1);
    }
    else
    {
        emit eventWheel(qRound(scenePos.x()), qRound(scenePos.y()), event->delta(), 0);
    }
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::mouseDoubleClickEvent ( QMouseEvent * event )
{
    if (!hasFocus())
        return;
    Qt::MouseButton btn = event->button();
    int button;
    switch(btn)
    {
    case Qt::LeftButton: button = 1; break;
    case Qt::RightButton: button = 3; break;
    case Qt::MiddleButton: button = 2; break;
    }
    QPointF scenePos = mapToScene( event->pos().x(), event->pos().y() );
    //emit eventMouse(1, event->pos().x(), event->pos().y(), button);
    emit eventMouse(1, qRound(scenePos.x()), qRound(scenePos.y()), button);
    event->accept();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::mouseMoveEvent ( QMouseEvent * event )
{
    if (!hasFocus())
        return;
    handleMouseEvent(2, event);
    event->accept();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::mousePressEvent ( QMouseEvent * event )
{
    if (!hasFocus())
        return;

    setMouseTracking(true);
    m_pendingEvent.clear(); //clear possible move events which are still in queue
    handleMouseEvent(0, event);
    event->accept();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::mouseReleaseEvent ( QMouseEvent * event )
{
    if (!hasFocus())
        return;
    if(m_trackerActive == false)
    {
        setMouseTracking(false);
    }
    m_pendingEvent.clear(); //clear possible move events which are still in queue
    handleMouseEvent(3, event);
    event->accept();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::showEvent ( QShowEvent * event ) //widget is shown, now the view can be fitted to size
{
    QGraphicsView::showEvent( event );
    fitInView(m_pixmapItem,Qt::IgnoreAspectRatio);
    //qDebug() << "fitInView (showEvent)" << m_pixmapItem->boundingRect();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::contextMenuEvent(QContextMenuEvent * event)
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



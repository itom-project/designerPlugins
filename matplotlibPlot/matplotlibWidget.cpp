/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2021, Institut fuer Technische Optik (ITO), 
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
#include <qclipboard.h>
#include <qstatusbar.h>
#include "matplotlibplot.h"

//-------------------------------------------------------------------------------------
MatplotlibWidget::MatplotlibWidget(QMenu *contextMenu, QWidget * parent) :
    QGraphicsView(parent),
    m_trackerActive(false),
    m_internalResize(false),
    m_keepSizeFixed(false),
    m_updatePlot(false),
    m_scene(nullptr),
    m_rectItem(nullptr),
    m_pixmapItem(nullptr),
#ifdef _DEBUG
    m_debugOutput(false), //set this to true in order to get qDebug() outputs in _DEBUG mode
#endif
    m_contextMenu(contextMenu),
    m_mouseTrackingState(false)
{
    m_scene = new QGraphicsScene(this);
    setScene(m_scene);

    this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    this->setMouseTracking(false); //only react on mouse-move events between a mouse-click and a subsequent release
    m_mouseTrackingState = hasMouseTracking();

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

    if (qobject_cast<QMainWindow*>(parent))
    {
        QStatusBar *statusBar = qobject_cast<QMainWindow*>(parent)->statusBar();
        connect(this, SIGNAL(statusBarClear()), statusBar, SLOT(clearMessage()));
        connect(this, SIGNAL(statusBarMessage(QString)), statusBar, SLOT(showMessage(QString)));
        connect(this, SIGNAL(statusBarMessage(QString, int)), statusBar, SLOT(showMessage(QString, int)));
    }
};

//-------------------------------------------------------------------------------------
MatplotlibWidget::~MatplotlibWidget()
{
#ifdef _DEBUG
    if (m_debugOutput)
    {
        qDebug() << "MatplotlibWidet::destructor";
    }
#endif
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
#ifdef _DEBUG
    if (m_debugOutput)
    {
        qDebug() << "MatplotlibWidet::externalResize:" << width << height;
    }
#endif

    MatplotlibPlot *parent = qobject_cast<MatplotlibPlot*>(this->parent());
    if(parent)
    {
        m_internalResize = true;
        parent->resizeCanvas(width,height);
        m_internalResize = false;
    }
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::paintResult(QSharedPointer<char> imageString, int x, int y, int w, int h, bool blit )
{
#ifdef _DEBUG
    if (m_debugOutput)
    {
        qDebug() << "MatplotlibWidet::paintResult:" << x << y << w << h << blit;
    }
#endif

    int imgHeight = 0;
    int imgWidth = 0;

    m_timer.stop();
    
    if(blit == false)
    {
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
            //in case of blit, y is the distance from the bottom border, convert it into the
            //distance from top:
            y = imgHeight - h - y;

            QPainter painter(&m_pixmap);
            QImage image = QImage((uchar*)imageString.data(),w,h,QImage::Format_ARGB32);
            painter.drawImage(QPoint(x,y),image);
            painter.end();
            m_pixmapItem->setPixmap(m_pixmap);
            m_pixmapItem->update();
        }

    }

    paintRect(false);
    
    QSize s = size();

	//it seems that screens with scaling factor cannot render pixmaps with all sizes,
	//therefore it can occur that the real image size is not returned, which will lead to
	//an inifinte regression. To terminate this, we allow a size difference of up to 1px.
    if (qAbs(m_pixmap.width() - s.width()) < 2 && qAbs(m_pixmap.height() - s.height()) < 2)
    {
        setTransform( QTransform(1,0,0,1,0,0), false );
        centerOn( m_pixmapItem->boundingRect().center() );
    }
    else
    {
#ifdef _DEBUG
        if (m_debugOutput)
        {
            qDebug() << "MatplotlibWidet::paintResult: create PendingEvent" << s.height() << s.width();
        }
#endif
        fitInView(m_pixmapItem,Qt::IgnoreAspectRatio);
        m_pendingEvent = PendingEvent(s.height(), s.width());
    }
    
    //handle possible further update requests
    paintTimeout();

    emit eventIdle();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::paintResultDeprecated(QByteArray imageString, int x, int y, int w, int h, bool blit )
{
#ifdef _DEBUG
    if (m_debugOutput)
    {
        qDebug() << "MatplotlibWidet::paintResultDeprecated:" << x << y << w << h << blit;
    }
#endif

    int imgHeight = 0;
    int imgWidth = 0;

    m_timer.stop();
    
    if(blit == false)
    {
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
            //in case of blit, y is the distance from the bottom border, convert it into the
            //distance from top:
            y = imgHeight - h - y;
            QPainter painter(&m_pixmap);
            QImage image = QImage((uchar*)imageString.data(),w,h,QImage::Format_ARGB32);
            painter.drawImage(QPoint(x,y),image);
            painter.end();
            m_pixmapItem->setPixmap(m_pixmap);
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
void MatplotlibWidget::copyToClipboardResult(QSharedPointer<char> imageString, int x, int y, int w, int h)
{
#ifdef _DEBUG
    if (m_debugOutput)
    {
        qDebug() << "MatplotlibWidet::copyToClipboardResult:" << x << y << w << h;
    }
#endif

    QImage image  = QImage((uchar*)imageString.data(), w, h, QImage::Format_ARGB32); //shallow copy of imageString buffer data, imageString must be alive during livetime of image (therefore, copy below)

    QClipboard *clipboard = QApplication::clipboard();
    if (clipboard)
    {
        clipboard->setImage(image.copy());
        emit statusBarMessage(tr("Copy current view to clipboard ... Done."), 1000);
    }

}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::paintRect(bool drawRect, int x /*= 0*/, int y /*= 0*/, int w /*= 0*/, int h /*= 0*/)
{
#ifdef _DEBUG
    if (m_debugOutput)
    {
        qDebug() << "MatplotlibWidet::paintRect:" << x << y << w << h << drawRect;
    }
#endif

    if(drawRect == false)
    {
        if (m_rectItem->isVisible())
        {
            m_rectItem->setVisible(false);
        }
    }
    else
    {
        if (w < 0)
        {
            x += w;
            w *= -1;
        }

        if (h < 0)
        {
            y += h;
            h *= -1;
        }
        
        m_rectItem->setRect(x, y, w, h);
        m_rectItem->setVisible(true);
        m_rectItem->update();

        m_timer.stop();

        //handle possible further update requests
        paintTimeout();
    }
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::resizeEvent ( QResizeEvent * event )
{
    //It seems that event->size() is not exactly the same than QWidget::size(), although the documentation
    //pretends this. Therefore, we only refer to QWidget::size() to be consistent.
#ifdef _DEBUG
    if (m_debugOutput)
    {
        qDebug() << "MatplotlibWidet::resizeEvent:" << event->size() << size() << m_internalResize;
    }
#endif

    if(m_internalResize == false)
    {
        if(m_pixmapItem)
        {            
            fitInView(m_pixmapItem,Qt::IgnoreAspectRatio);
        }

#ifdef _DEBUG
        if (m_debugOutput)
        {
            qDebug() << "MatplotlibWidet::resizeEvent:2:" << size();
        }
#endif

        if (m_updatePlot)
        {
            m_pendingEvent = PendingEvent(size().height(), size().width());
            m_timer.start(60);
        }
    }
    m_internalResize = false;
    
    QGraphicsView::resizeEvent(event);
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::replot()
{
#ifdef _DEBUG
    if (m_debugOutput)
    {
        qDebug() << "MatplotlibWidet::replot: create PendingEvent" << height() << width();
    }
#endif
    m_pendingEvent = PendingEvent(height(), width());
    m_timer.start(1);
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::paintTimeout()
{
#ifdef _DEBUG
    if (m_debugOutput)
    {
        qDebug() << "MatplotlibWidet::paintTimeout";
    }
#endif

    if(m_pendingEvent.isValid())
    {
        
        switch(m_pendingEvent.m_type)
        {
        case PendingEvent::typeResize:
            m_timer.start(2000); //if further update is required, it will be requested if the recent update has been transmitted or the timer runs into its timeout
            emit eventResize(m_pendingEvent.m_w, m_pendingEvent.m_h);
            m_pendingEvent.clear();
            break;
        case PendingEvent::typeMouseMove:
            m_timer.start(100); //if further update is required, it will be requested if the recent update has been transmitted or the timer runs into its timeout
            emit eventMouse(2, m_pendingEvent.m_x,m_pendingEvent.m_y, m_pendingEvent.m_button);
            m_pendingEvent.clear();
            break;
        }
    }
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::copyToClipboard(int dpi /*= 200*/)
{
    emit statusBarMessage(tr("Copy current view to clipboard ..."), 5000);
    emit eventCopyToClipboard(dpi);
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
        else if(btns & Qt::MiddleButton)
        {
            button = 2;
        }

        if(0 && button == 0) //no mouse button pressed, then handle mouse move event with lowest priority
        {
            if(!m_timer.isActive())
            {
                m_pendingEvent = PendingEvent(qRound(scenePos.x()), qRound(scenePos.y()), button);
                paintTimeout();
            }
        }
        else
        {
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
        emit eventMouse(type, qRound(scenePos.x()), qRound(scenePos.y()), button);
    }
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::keyPressEvent ( QKeyEvent * event )
{
    if (!hasFocus())
        return;

    emit eventKey(0, event->key(), event->modifiers(), event->isAutoRepeat());
    event->accept();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::keyReleaseEvent ( QKeyEvent * event )
{
    if (!hasFocus())
        return;
    emit eventKey(1, event->key(), event->modifiers(), event->isAutoRepeat());
    event->accept();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::leaveEvent ( QEvent * event )
{
    if (!hasFocus())
        return;

    emit eventLeave();
    emit eventLeaveEnter(false); //deprecated
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::enterEvent ( QEvent * event )
{
    if (!hasFocus())
        return;

    QEnterEvent *e = static_cast<QEnterEvent*>(event);
    if (e)
    {
        emit eventEnter(e->pos().x(), e->pos().y());
    }
    else
    {
        emit eventEnter(0, 0);
    }

    emit eventLeaveEnter(true); //deprecated
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::wheelEvent( QWheelEvent * event )
{
    if (!hasFocus())
        return;
#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
    QPointF scenePos = mapToScene(event->position().x(), event->position().y());
    if (std::abs(event->angleDelta().y()) > 0)
#else
    QPointF scenePos = mapToScene(event->pos().x(), event->pos().y());
    if (event->orientation() == Qt::Vertical)
#endif    
    {
#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
        emit eventWheel(qRound(scenePos.x()), qRound(scenePos.y()), event->angleDelta().y(), 1);
#else
        emit eventWheel(qRound(scenePos.x()), qRound(scenePos.y()), event->delta(), 1);
#endif   
    }
    else
    {
#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
        emit eventWheel(qRound(scenePos.x()), qRound(scenePos.y()), event->angleDelta().x(), 0);
#else
        emit eventWheel(qRound(scenePos.x()), qRound(scenePos.y()), event->delta(), 0);
#endif 
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

    m_mouseTrackingState = hasMouseTracking();
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

    setMouseTracking(m_mouseTrackingState);
    m_pendingEvent.clear(); //clear possible move events which are still in queue
    handleMouseEvent(3, event);
    event->accept();
}

//-------------------------------------------------------------------------------------
void MatplotlibWidget::showEvent ( QShowEvent * event ) //widget is shown, now the view can be fitted to size
{
    QGraphicsView::showEvent( event );
    fitInView(m_pixmapItem,Qt::IgnoreAspectRatio);
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



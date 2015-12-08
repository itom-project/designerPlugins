/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2015, Institut fuer Technische Optik (ITO), 
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

#include "itomQwtPlot.h"

#include <qmenu.h>
#include <qwidget.h>
#include <qpen.h>
#include <qfont.h>
#include <qevent.h>
#include <qmainwindow.h>
#include <qstatusbar.h>

#include "itomPlotZoomer.h"
#include "itomPlotMagnifier.h"

#include "common/apiFunctionsGraphInc.h"
#include "common/retVal.h"

#include <qwt_plot_picker.h>
#include <qwt_plot_panner.h>
#include <qwt_scale_widget.h>

//---------------------------------------------------------------------------
ItomQwtPlot::ItomQwtPlot(QMenu *contextMenu, QWidget * parent /*= NULL*/) :
    QwtPlot(parent),
    m_pContextMenu(contextMenu),
    m_showContextMenu(true),
    m_keepAspectRatio(false),
    m_firstTimeVisible(false)
{
    if (qobject_cast<QMainWindow*>(parent))
    {
        QStatusBar *statusBar = qobject_cast<QMainWindow*>(parent)->statusBar();
        connect(this, SIGNAL(statusBarClear()), statusBar, SLOT(clearMessage()));
        connect(this, SIGNAL(statusBarMessage(QString)), statusBar, SLOT(showMessage(QString)));
        connect(this, SIGNAL(statusBarMessage(QString, int)), statusBar, SLOT(showMessage(QString, int)));
    }

    setMouseTracking(false);

    //zoom tool
    m_pZoomer = new ItomPlotZoomer(QwtPlot::xBottom, QwtPlot::yLeft, canvas());
    m_pZoomer->setEnabled(false);
    m_pZoomer->setTrackerMode(QwtPicker::AlwaysOn);
    m_pZoomer->setMousePattern(QwtEventPattern::MouseSelect2, Qt::NoButton); //right click should open the context menu, not a zoom out to level 0 (Ctrl+0) if zoomer is enabled.
    //all others settings for zoomer are set in init (since they need access to the settings via api)

    //panner tool
    m_pPanner = new QwtPlotPanner(canvas());
    m_pPanner->setAxisEnabled(QwtPlot::yRight, false);
    m_pPanner->setCursor(Qt::SizeAllCursor);
    m_pPanner->setEnabled(false);
    connect(m_pPanner, SIGNAL(panned(int, int)), m_pZoomer, SLOT(canvasPanned(int, int))); //if panner is moved, the new rect is added to the zoom stack for a synchronization of both tools

    //magnifier tool
    m_pMagnifier = new ItomPlotMagnifier(canvas(), m_pZoomer);
    m_pMagnifier->setWheelModifiers(Qt::ControlModifier);
    m_pMagnifier->setZoomInKey(Qt::Key_Plus, Qt::KeypadModifier);
    m_pMagnifier->setZoomOutKey(Qt::Key_Minus, Qt::KeypadModifier);
    m_pMagnifier->setMouseFactor(-m_pMagnifier->mouseFactor()); //todo: not done in 2d plot, only in 1d plot. what is right?
    m_pMagnifier->setEnabled(true);
    m_pMagnifier->setAxisEnabled(QwtPlot::xTop, false);
    m_pMagnifier->setAxisEnabled(QwtPlot::yRight, false);
    m_pMagnifier->setAxisEnabled(QwtPlot::yLeft, true);
    m_pMagnifier->setAxisEnabled(QwtPlot::xBottom, true);

    //Geometry of the plot:
    setContentsMargins(5, 5, 5, 5); //this is the border between the canvas (including its axes and labels) and the overall mainwindow
    canvas()->setContentsMargins(0, 0, 0, 0); //border of the canvas (border between canvas and axes or title)

    //left axis
    QwtScaleWidget *leftAxis = axisWidget(QwtPlot::yLeft);
    leftAxis->setMargin(0);                 //distance backbone <-> canvas
    leftAxis->setSpacing(6);                //distance tick labels <-> axis label
    leftAxis->scaleDraw()->setSpacing(4);   //distance tick labels <-> ticks
    leftAxis->setContentsMargins(0, 0, 0, 0);  //left axis starts and ends at same level than canvas

    //bottom axis
    QwtScaleWidget *bottomAxis = axisWidget(QwtPlot::xBottom);
    bottomAxis->setMargin(0);                 //distance backbone <-> canvas
    bottomAxis->setSpacing(6);                //distance tick labels <-> axis label
    bottomAxis->scaleDraw()->setSpacing(4);   //distance tick labels <-> ticks
    bottomAxis->setContentsMargins(0, 0, 0, 0);  //left axis starts and ends at same level than canvas
}

//---------------------------------------------------------------------------
ItomQwtPlot::~ItomQwtPlot()
{
}

//---------------------------------------------------------------------------
void ItomQwtPlot::loadStyles()
{
    QPen rubberBandPen = QPen(QBrush(Qt::red), 2, Qt::DashLine);
    QPen trackerPen = QPen(QBrush(Qt::red), 2);
    QFont trackerFont = QFont("Verdana", 10);

    if (ito::ITOM_API_FUNCS_GRAPH)
    {
        rubberBandPen = apiGetFigureSetting(parent(), "zoomRubberBandPen", rubberBandPen, NULL).value<QPen>();
        trackerPen = apiGetFigureSetting(parent(), "trackerPen", trackerPen, NULL).value<QPen>();
        trackerFont = apiGetFigureSetting(parent(), "trackerFont", trackerFont, NULL).value<QFont>();
    }

    m_pZoomer->setRubberBandPen(rubberBandPen);
    m_pZoomer->setTrackerFont(trackerFont);
    m_pZoomer->setTrackerPen(trackerPen);
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomPlotZoomer *ItomQwtPlot::zoomer() const
{
    return m_pZoomer;
}

//----------------------------------------------------------------------------------------------------------------------------------
QwtPlotPanner *ItomQwtPlot::panner() const
{
    return m_pPanner;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::configRescaler(void)
{
    m_pZoomer->setFixedAspectRatio(m_keepAspectRatio);
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setKeepAspectRatio(bool keep)
{ 
    m_keepAspectRatio = keep; 
    configRescaler(); 
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setVisible(bool visible)
{
    this->QwtPlot::setVisible(visible);
    if (visible)
    {
        if (!m_firstTimeVisible)
        {
            this->updateScaleValues(true, true);
        }
        else
        {
            this->updateScaleValues(true, false);
        }
        m_firstTimeVisible = true;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::contextMenuEvent(QContextMenuEvent * event)
{
    if (m_showContextMenu && m_pPanner->isEnabled() == false)
    {
        event->accept();
        m_pContextMenu->exec(event->globalPos());
    }
    else
    {
        event->ignore();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::resizeEvent(QResizeEvent * event)
{
    configRescaler();
    QwtPlot::resizeEvent(event);
}
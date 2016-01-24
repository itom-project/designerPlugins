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
#include <qhash.h>
#include <qaction.h>
#include <qapplication.h>
#include <qclipboard.h>
#include <qlineedit.h>
#include <qinputdialog.h>
#include <qmessagebox.h>
#include <qfiledialog.h>

#include "itomPlotZoomer.h"
#include "itomPlotMagnifier.h"
#include "drawItem.h"
#include "itomQwtDObjFigure.h"
#include "userInteractionPlotPicker.h"
#include "multiPointPickerMachine.h"
#include "dialogExportProperties.h"

#include "plotLegends/infoWidgetShapes.h"
#include "plotLegends/infoWidgetMarkers.h"

#include "common/apiFunctionsGraphInc.h"
#include "common/retVal.h"

#include <qwt_plot_picker.h>
#include <qwt_plot_panner.h>
#include <qwt_scale_widget.h>
#include <qwt_plot_renderer.h>
#include <qwt_symbol.h>

//---------------------------------------------------------------------------
ItomQwtPlot::ItomQwtPlot(ItomQwtDObjFigure * parent /*= NULL*/) :
    QwtPlot(parent),
    m_pContextMenu(NULL),
    m_showContextMenu(true),
    m_keepAspectRatio(false),
    m_firstTimeVisible(false),
    m_state(-1),
    m_stateIsChanging(false),
    m_selectedShape(NULL),
    m_ignoreNextMouseEvent(false),
    m_shapeModificationModes(ItomQwtPlotEnums::Move | ItomQwtPlotEnums::Rotate | ItomQwtPlotEnums::Resize),
    m_inverseColor0(Qt::green),
    m_inverseColor1(Qt::blue),
    m_elementsToPick(1),
    m_currentShapeType(ito::Shape::Point),
    m_buttonStyle(0),
    m_plottingEnabled(true),
    m_markerLabelVisible(false),
    m_shapesLabelVisible(false),
    m_unitLabelStyle(ito::AbstractFigure::UnitLabelSlash),
    m_pActSave(NULL),
    m_pActHome(NULL),
    m_pActPan(NULL),
    m_pActZoom(NULL),
    m_pActAspectRatio(NULL),
    m_pActSendCurrentToWorkspace(NULL),
    m_pActCopyClipboard(NULL),
    m_pMenuShapeType(NULL),
    m_pActClearShapes(NULL),
    m_pActProperties(NULL),
    m_pActShapeType(NULL)
{
    if (qobject_cast<QMainWindow*>(parent))
    {
        QStatusBar *statusBar = qobject_cast<QMainWindow*>(parent)->statusBar();
        connect(this, SIGNAL(statusBarClear()), statusBar, SLOT(clearMessage()));
        connect(this, SIGNAL(statusBarMessage(QString)), statusBar, SLOT(showMessage(QString)));
        connect(this, SIGNAL(statusBarMessage(QString, int)), statusBar, SLOT(showMessage(QString, int)));
    }

    setMouseTracking(false);

    //load actions and icons
    createBaseActions();

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
    //m_pMagnifier->setMouseFactor(-m_pMagnifier->mouseFactor()); //todo: not done in 2d plot, only in 1d plot. what is right?
    m_pMagnifier->setEnabled(true);
    m_pMagnifier->setAxisEnabled(QwtPlot::xTop, false);
    m_pMagnifier->setAxisEnabled(QwtPlot::yRight, false);
    m_pMagnifier->setAxisEnabled(QwtPlot::yLeft, true);
    m_pMagnifier->setAxisEnabled(QwtPlot::xBottom, true);

    //multi point picker for pick-point action (equivalent to matlabs ginput)
    m_pMultiPointPicker = new UserInteractionPlotPicker(QwtPlot::xBottom, QwtPlot::yLeft, QwtPicker::PolygonRubberBand, QwtPicker::AlwaysOn, canvas());
    m_pMultiPointPicker->setEnabled(false);
    m_pMultiPointPicker->setRubberBand(QwtPicker::UserRubberBand); //user is cross here
    //m_pMultiPointPicker->setStateMachine(new QwtPickerClickPointMachine);
    //m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine);
    m_pMultiPointPicker->setRubberBandPen(QPen(QBrush(Qt::green, Qt::SolidPattern), 2));
    connect(m_pMultiPointPicker, SIGNAL(activated(bool)), this, SLOT(multiPointActivated(bool)));

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

    setState(stateIdle);

    if (parent)
    {
        m_pActProperties = parent->getPropertyDockWidget()->toggleViewAction();
        connect(m_pActProperties, SIGNAL(triggered(bool)), parent, SLOT(mnuShowProperties(bool)));
    }
}

//---------------------------------------------------------------------------
ItomQwtPlot::~ItomQwtPlot()
{
    clearAllGeometricShapes();

    if (m_pMultiPointPicker != NULL)
    {
        m_pMultiPointPicker->deleteLater();
        m_pMultiPointPicker = NULL;
    }
}

//---------------------------------------------------------------------------
void ItomQwtPlot::createBaseActions()
{
    QAction *a = NULL;
    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(parent());

    //m_actHome
    m_pActHome = a = new QAction(tr("Home"), p);
    a->setObjectName("actHome");
    a->setToolTip(tr("Reset original view"));
    a->setShortcut(Qt::CTRL + Qt::Key_0);
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActHome()));

    //m_actSave
    m_pActSave = a = new QAction(tr("Save..."), p);
    a->setShortcut(QKeySequence::Save);
    a->setObjectName("actSave");
    a->setToolTip(tr("Export current view..."));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActSave()));

    //m_actCopyClipboard
    m_pActCopyClipboard = a = new QAction(tr("Copy to clipboard"), p);
    a->setShortcut(QKeySequence::Copy);
    a->setObjectName("actCopyClipboard");
    a->setToolTip(tr("Copies the current view to the clipboard"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuCopyToClipboard()));

    //m_pActSendCurrentToWorkspace
    m_pActSendCurrentToWorkspace = a = new QAction(tr("Send current view to workspace..."), p);
    a->setObjectName("actSendCurrentToWorkspace");
    connect(a, SIGNAL(triggered()), this, SLOT(mnuSendCurrentToWorkspace()));

    //m_actPan
    m_pActPan = a = new QAction(tr("Move"), p);
    a->setObjectName("actPan");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Pan axes with left mouse, zoom with right"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActPan(bool)));

    //m_actZoom
    m_pActZoom = a = new QAction(tr("Zoom to rectangle"), p);
    a->setObjectName("actZoom");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Zoom to rectangle"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActZoom(bool)));

    //m_pActClearDrawings
    m_pActClearShapes = a = new QAction(tr("Clear geometric shapes"), p);
    a->setObjectName("actClearGeometrics");
    a->setCheckable(false);
    a->setChecked(false);
    a->setToolTip(tr("Clear all existing geometric shapes"));
    connect(a, SIGNAL(triggered()), this, SLOT(clearAllGeometricShapes()));

    //m_actApectRatio
    m_pActAspectRatio = a = new QAction(tr("Lock aspect ratio"), p);
    a->setObjectName("actRatio");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Toggle fixed / variable aspect ration between axis x and y"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActRatio(bool)));

    //m_pActShapeType
    m_pActShapeType = new QAction(tr("Draw geometric shape"), p);
    m_pMenuShapeType = new QMenu(tr("Draw geometric shape"), p);
    m_pActShapeType->setMenu(m_pMenuShapeType);

    a = m_pMenuShapeType->addAction(tr("Point"));
    a->setData(ito::Shape::Point);
    m_pMenuShapeType->addAction(a);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Line"));
    a->setData(ito::Shape::Line);
    m_pMenuShapeType->addAction(a);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Rectangle"));
    a->setData(ito::Shape::Rectangle);
    m_pMenuShapeType->addAction(a);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Square"));
    a->setData(ito::Shape::Square);
    m_pMenuShapeType->addAction(a);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Ellipse"));
    a->setData(ito::Shape::Ellipse);
    m_pMenuShapeType->addAction(a);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Circle"));
    a->setData(ito::Shape::Circle);
    m_pMenuShapeType->addAction(a);
    a->setCheckable(true);

    m_pActShapeType->setData(ito::Shape::Rectangle);
    m_pActShapeType->setVisible(true);
    m_pActShapeType->setCheckable(true);
    connect(m_pMenuShapeType, SIGNAL(triggered(QAction*)), this, SLOT(mnuGroupShapeTypes(QAction*)));
    connect(m_pActShapeType, SIGNAL(triggered(bool)), this, SLOT(mnuShapeType(bool)));
}

//---------------------------------------------------------------------------
void ItomQwtPlot::loadStyles()
{
    QPen rubberBandPen = QPen(QBrush(Qt::red), 2, Qt::DashLine);
    QPen trackerPen = QPen(QBrush(Qt::red), 2);
    QFont trackerFont = QFont("Verdana", 10);
    QBrush trackerBg = QBrush(QColor(255, 255, 255, 155), Qt::SolidPattern);

    m_unitLabelStyle = ito::AbstractFigure::UnitLabelSlash;

    if (ito::ITOM_API_FUNCS_GRAPH)
    {
        rubberBandPen = apiGetFigureSetting(parent(), "zoomRubberBandPen", rubberBandPen, NULL).value<QPen>();
        trackerPen = apiGetFigureSetting(parent(), "trackerPen", trackerPen, NULL).value<QPen>();
        trackerBg = apiGetFigureSetting(parent(), "trackerBackground", QBrush(QColor(255, 255, 255, 155), Qt::SolidPattern), NULL).value<QBrush>();
        trackerFont = apiGetFigureSetting(parent(), "trackerFont", trackerFont, NULL).value<QFont>();

        m_unitLabelStyle = (ito::AbstractFigure::UnitLabelStyle)(apiGetFigureSetting(parent(), "unitLabelStyle", m_unitLabelStyle, NULL).value<int>());
    }

    m_pZoomer->setRubberBandPen(rubberBandPen);
    m_pZoomer->setTrackerFont(trackerFont);
    m_pZoomer->setTrackerPen(trackerPen);

    m_pMultiPointPicker->setTrackerFont(trackerFont);
    m_pMultiPointPicker->setTrackerPen(trackerPen);
    m_pMultiPointPicker->setBackgroundFillBrush(trackerBg);

    foreach(DrawItem *item, m_pShapes)
    {
        if (item && item->getAutoColor())
        {
            item->setColor(m_inverseColor0, m_inverseColor1);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setButtonStyle(int style)
{
    if (style == 0)
    {
        m_pActSave->setIcon(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"));
        m_pActCopyClipboard->setIcon(QIcon(":/itomDesignerPlugins/general/icons/clipboard.png"));
        m_pActHome->setIcon(QIcon(":/itomDesignerPlugins/general/icons/home.png"));
        m_pActPan->setIcon(QIcon(":/itomDesignerPlugins/general/icons/move.png"));
        m_pActClearShapes->setIcon(QIcon(":/itomDesignerPlugins/general/icons/editDelete.png"));
        m_pActAspectRatio->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio11.png"));
        m_pActZoom->setIcon(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"));
        m_pActSendCurrentToWorkspace->setIcon(QIcon(":/plugins/icons/sendToPython.png"));

        switch (m_currentShapeType)
        {
            default:
            case ito::Shape::Point:
                m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/point.png"));
                break;
            case ito::Shape::Line:
                m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"));
                break;
            case ito::Shape::Rectangle:
                m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/rectangle.png"));
                break;
            case ito::Shape::Ellipse:
                m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/ellipse.png"));
                break;
            case ito::Shape::Circle:
                m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/circle.png"));
                break;
            case ito::Shape::Square:
                m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/square.png"));
                break;
        }
    }
    else
    {
        m_pActSave->setIcon(QIcon(":/itomDesignerPlugins/general_lt/icons/filesave_lt.png"));
        m_pActCopyClipboard->setIcon(QIcon(":/itomDesignerPlugins/general_lt/icons/clipboard_lt.png"));
        m_pActHome->setIcon(QIcon(":/itomDesignerPlugins/general_lt/icons/home_lt.png"));
        m_pActPan->setIcon(QIcon(":/itomDesignerPlugins/general_lt/icons/move_lt.png"));
        m_pActClearShapes->setIcon(QIcon(":/itomDesignerPlugins/general_lt/icons/editDelete_lt.png"));
        m_pActAspectRatio->setIcon(QIcon(":/itomDesignerPlugins/aspect_lt/icons/AspRatio11_lt.png"));
        m_pActZoom->setIcon(QIcon(":/itomDesignerPlugins/general_lt/icons/zoom_to_rect_lt.png"));
        m_pActSendCurrentToWorkspace->setIcon(QIcon(":/plugins/icons/sendToPython_lt.png"));

        switch (m_currentShapeType)
        {
        default:
        case ito::Shape::Point:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/point_lt.png"));
            break;
        case ito::Shape::Line:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/pntline_lt.png"));
            break;
        case ito::Shape::Rectangle:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/rectangle_lt.png"));
            break;
        case ito::Shape::Ellipse:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/ellipse_lt.png"));
            break;
        case ito::Shape::Circle:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/circle_lt.png"));
            break;
        case ito::Shape::Square:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/square_lt.png"));
            break;
        }
    }

    
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setInverseColors(const QColor &color0, const QColor &color1)
{
    m_inverseColor0 = color0;
    m_inverseColor1 = color1;

    foreach(DrawItem *item, m_pShapes)
    {
        if (item && item->getAutoColor())
        {
            item->setColor(m_inverseColor0, m_inverseColor1);
        }
    }
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
    m_pActAspectRatio->setChecked(keep);
    configRescaler(); 
    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());
    if (p)
    {
        p->updatePropertyDock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuActPan(bool checked)
{
    if (checked)
    {
        setState(statePan);
    }
    else if (m_state == statePan)
    {
        setState(stateIdle);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuActZoom(bool checked)
{
    if (checked)
    {
        setState(stateZoom);
    }
    else if (m_state == stateZoom)
    {
        setState(stateIdle);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setShapeModificationModes(const ItomQwtPlotEnums::ModificationModes &modes)
{
    m_shapeModificationModes = modes;

    foreach(DrawItem *d, this->m_pShapes)
    {
        d->setModificationModes(modes);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuGroupShapeTypes(QAction *action)
{
    switch (action->data().toInt())
    {
    case ito::Shape::Point:
        m_currentShapeType = ito::Shape::Point;
        break;
    case ito::Shape::Line:
        m_currentShapeType = ito::Shape::Line;
        break;
    case ito::Shape::Rectangle:
        m_currentShapeType = ito::Shape::Rectangle;
        break;
    case ito::Shape::Ellipse:
        m_currentShapeType = ito::Shape::Ellipse;
        break;
    case ito::Shape::Circle:
        m_currentShapeType = ito::Shape::Circle;
        break;
    case ito::Shape::Square:
        m_currentShapeType = ito::Shape::Square;
        break;
    default:
        m_currentShapeType = ito::Shape::Invalid;
        break;
    }

    if (m_state == stateDrawShape) //if an interaction is already running and the user desires another one, goto idle first.
    {
        setState(stateIdle);
    }
    setState(stateDrawShape);
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuShapeType(bool checked)
{
    if (checked)
    {
        setState(stateDrawShape); 
    }
    else if (state() == stateDrawShape)
    {
        setState(stateIdle);
    }
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

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setState(int state)
{
    if (!m_stateIsChanging)
    {
        m_stateIsChanging = true;

        if (state != stateDrawShape)
        {
            m_elementsToPick = 1;
        }

        m_pActZoom->setChecked(state == stateZoom);
        m_pZoomer->setEnabled(state == stateZoom);
        m_pActPan->setChecked(state == statePan);
        m_pPanner->setEnabled(state == statePan);
        m_pActShapeType->setChecked(state == stateDrawShape);
        m_pMenuShapeType->setEnabled(m_plottingEnabled);
        m_pActClearShapes->setEnabled(m_plottingEnabled && countGeometricShapes() > 0);

        if (m_pActShapeType->isChecked() && !m_plottingEnabled)
        {
            m_pActShapeType->setChecked(false);
        }

        if (state != stateDrawShape && m_pMultiPointPicker->isEnabled()) //there is currently a shape selection running, stop it!
        {
            m_pMultiPointPicker->setStateMachine(NULL);
        }

        switch (state)
        {
        case stateIdle:
            canvas()->setCursor(Qt::ArrowCursor);
            break;

        case stateZoom:
            canvas()->setCursor(Qt::CrossCursor);
            break;

        case statePan:
            canvas()->setCursor(Qt::OpenHandCursor);
            break;

        case stateDrawShape:
        {
            if (m_state != stateDrawShape) //start draw shape action
            {
                switch (m_currentShapeType)
                {
                default:
                case ito::Shape::Point:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/point.png" : ":/itomDesignerPlugins/plot/icons/point_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(1);
                    break;

                case ito::Shape::Line:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/pntline.png" : ":/itomDesignerPlugins/plot/icons/pntline_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(1);
                    break;

                case ito::Shape::Rectangle:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/rectangle.png" : ":/itomDesignerPlugins/plot/icons/rectangle_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(1);
                    break;

                case ito::Shape::Ellipse:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/ellipse.png" : ":/itomDesignerPlugins/plot/icons/ellipse_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(1);
                    break;

                case ito::Shape::Circle:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/circle.png" : ":/itomDesignerPlugins/plot/icons/circle_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(1);
                    break;

                case ito::Shape::Square:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/square.png" : ":/itomDesignerPlugins/plot/icons/square_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(1);
                    break;
                }
            }

            foreach(QAction *act, m_pMenuShapeType->actions())
            {
                act->setChecked(act->data() == m_currentShapeType);
            }

            canvas()->setCursor(Qt::CrossCursor);
            break;
        }
        }

        m_state = state;
        stateChanged(state);

        m_stateIsChanging = false;
    }
}



//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::keyPressEvent(QKeyEvent * event)
{
    if (event->isAccepted() == false && event->matches(QKeySequence::Copy))
    {
        ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(parent());
        if (p) p->copyToClipBoard();
        replot();
        event->accept();
    }

    if (!event->isAccepted())
    {
        QwtPlot::keyPressEvent(event);
    }
    
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mousePressEvent(QMouseEvent * event)
{
    //tries to select the currently active item
    if (m_state == stateIdle)
    {
        event->accept();

        int canxpos = event->x() - canvas()->x();
        int canypos = event->y() - canvas()->y();
        QPointF scalePos(invTransform(QwtPlot::xBottom, canxpos), invTransform(QwtPlot::yLeft, canypos));
        double tol_x = std::abs(invTransform(QwtPlot::xBottom, 15) - invTransform(QwtPlot::xBottom, 0)); //tolerance in pixel for snapping to a geometric shape in x-direction
        double tol_y = std::abs(invTransform(QwtPlot::yLeft, 15) - invTransform(QwtPlot::yLeft, 0)); //tolerance in pixel for snapping to a geometric shape in y-direction
        char hitType;

        m_selectedShape = NULL;
        m_selectedShapeHitType = -1;

        QHash<int, DrawItem*>::iterator it;
        for (it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
        {
            if (it.value() == NULL)
            {
                continue;
            }

            hitType = it.value()->hitEdge(scalePos, tol_x, tol_y);
            if (hitType >= 0)
            {
                it.value()->setSelected(true);
                m_selectedShape = it.value();
                m_selectedShapeHitType = hitType;
                m_startMouseScale = scalePos;
                m_startMousePx = event->pos();
                m_mouseDragReplotCounter = 0;

                if (hitType == 0)
                {
                    m_startMouseScaleDiff = m_startMouseScale - m_selectedShape->getMarkerPosScale(0);
                }
                else
                {
                    m_startMouseScaleDiff = m_startMouseScale - m_selectedShape->getMarkerPosScale(hitType - 1);
                }
                ++it;
                break;
            }
            else
            {
                it.value()->setSelected(false);
            }
        }

        for (; it != m_pShapes.end(); ++it)
        {
            if (it.value() == NULL)
            {
                continue;
            }
            it.value()->setSelected(false);
        }
        replot();
    }

    if (!event->isAccepted())
    {
        QwtPlot::mousePressEvent(event);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mouseMoveEvent(QMouseEvent * event)
{
    if (m_ignoreNextMouseEvent)
    {
        m_ignoreNextMouseEvent = false;
        return;
    }

    if (m_state == stateIdle && m_selectedShape != NULL)
    {
        QPointF mousePosScale(invTransform(QwtPlot::xBottom, event->x() - canvas()->x()), invTransform(QwtPlot::yLeft, event->y() - canvas()->y()));

        if (m_selectedShapeHitType == 0) //mouse move
        {
            if (event->modifiers() == Qt::ControlModifier)
            {
                //only move horizontally or vertically
                QPointF diff = event->pos() - m_startMousePx;
                if (std::abs(diff.x()) > std::abs(diff.y()))
                {
                    mousePosScale = QPointF(mousePosScale.x(), m_startMouseScale.y());
                }
                else
                {
                    mousePosScale = QPointF(m_startMouseScale.x(), mousePosScale.y());
                }

            }

            if (m_selectedShape->shapeMoveTo(mousePosScale - m_startMouseScaleDiff))
            {
                event->accept();
            }
            else
            {
                event->ignore();
            }
        }
        else if (m_selectedShapeHitType > 0) //rescale
        {
            if (event->modifiers() == Qt::ControlModifier && \
                (m_selectedShape->getShape().type() == ito::Shape::Point || \
                m_selectedShape->getShape().type() == ito::Shape::MultiPointPick))
            {
                //only move horizontally or vertically
                QPointF diff = event->pos() - m_startMousePx;
                if (std::abs(diff.x()) > std::abs(diff.y()))
                {
                    mousePosScale = QPointF(mousePosScale.x(), m_startMouseScale.y());
                }
                else
                {
                    mousePosScale = QPointF(m_startMouseScale.x(), mousePosScale.y());
                }

            }

            //todo: line + alt: leep line size constant
            if (m_selectedShape->shapeResize(m_selectedShapeHitType, mousePosScale - m_startMouseScaleDiff, event->modifiers()))
            {
                event->accept();
            }
            else
            {
                event->ignore();
            }
        }

        if (event->isAccepted())
        {
            m_mouseDragReplotCounter++;
            if (m_mouseDragReplotCounter % 2 == 0)
            {
                replot();
                m_mouseDragReplotCounter = 0;
            }
        }
    }

    if (!event->isAccepted())
    {
        QwtPlot::mouseMoveEvent(event);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mouseReleaseEvent(QMouseEvent * event)
{
    if (m_state == stateIdle)
    {
        //modification of shape finished
        event->accept();
        QHash<int, DrawItem*>::iterator it = m_pShapes.begin();
        bool found = false;

        for (; it != m_pShapes.end(); ++it)
        {
            if (it.value() != NULL && it.value()->getSelected())
            {
                found = true;
                ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(parent());
                if (p)
                {
					if (p->ShapesWidget())
					{
						((PlotInfoShapes*)p->ShapesWidget())->updateShape(it.value()->getShape());
					}
                    emit p->geometricShapeChanged(it.value()->getIndex(), it.value()->getShape());
                }
            }
        }

        if (found)
        {
            replot();
        }
    }

    if (!event->isAccepted())
    {
        QwtPlot::mouseReleaseEvent(event);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::multiPointActivated(bool on)
{
    ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());
    QVector<ito::Shape> shapes;

    switch (m_currentShapeType)
    {
    case ito::Shape::MultiPointPick:
        if (!on)
        {
            QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
            bool aborted = false;

            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);
            }

            if (!aborted && polygonScale.size() > 1)
            {
                polygonScale.pop_back(); //the last item is only the current position of the cursor, erase it.
                shapes.push_back(ito::Shape::fromMultipoint(polygonScale));
                replot();
            }

            if (p)
            {

                emit p->userInteractionDone(ito::Shape::MultiPointPick, aborted, shapes);
                emit p->geometricShapeFinished(ito::Shape::MultiPointPick, aborted);
				if (((PlotInfoMarker*)((ItomQwtDObjFigure*)(this->parent()))->MarkerWidget()))
				{
					((PlotInfoMarker*)((ItomQwtDObjFigure*)(this->parent()))->MarkerWidget())->updateMarkers(shapes);
				}
            }

            m_pMultiPointPicker->setEnabled(false);
            setState(stateIdle);
        }
        break;

    case ito::Shape::Point:
        if (!on)
        {
            QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
            bool aborted = false;

            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
                m_currentShapeIndices.clear();
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);

                ito::Shape shape = ito::Shape::fromPoint(polygonScale[0]);
                DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
                newItem->setColor(m_inverseColor0, m_inverseColor1);
                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else newItem->setPen(QPen(Qt::green));

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_pShapes.insert(newItem->getIndex(), newItem);
                m_currentShapeIndices.append(newItem->getIndex());
                replot();
            }

            // if further elements are needed reset the plot engine and go ahead else finish editing
            if (m_elementsToPick > 1)
            {
                m_elementsToPick--;
                MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());
                if (m)
                {
                    m->setMaxNrItems(1);
                    m_pMultiPointPicker->setEnabled(true);

                    if (!aborted)
                    {
                        if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 points. Esc aborts the selection.").arg(m_elementsToPick));
                        else emit statusBarMessage(tr("Please draw one point. Esc aborts the selection."));
                    }
                }
                return;
            }
            else
            {
                m_elementsToPick = 0;
                if (p)
                {
                    QPolygonF destPolygon(0);
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
                    }
                    m_currentShapeIndices.clear();
                    emit p->userInteractionDone(ito::Shape::Point, aborted, shapes);
                    emit p->geometricShapeFinished(ito::Shape::Point, aborted);
					if (((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget()))
					{
						((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget())->updateShapes(shapes);
					}
					
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        }
        break;

    case ito::Shape::Line:
        if (!on)
        {
            QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
            bool aborted = false;

            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
                m_currentShapeIndices.clear();
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);

                ito::Shape shape = ito::Shape::fromLine(polygonScale[0], polygonScale[1]);
                DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
                newItem->setColor(m_inverseColor0, m_inverseColor1);
                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else newItem->setPen(QPen(Qt::green));

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_pShapes.insert(newItem->getIndex(), newItem);
                m_currentShapeIndices.append(newItem->getIndex());
                replot();
            }

            // if further elements are needed reset the plot engine and go ahead else finish editing
            if (m_elementsToPick > 1)
            {
                m_elementsToPick--;
                MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());
                if (m)
                {
                    m->setMaxNrItems(2);
                    m_pMultiPointPicker->setEnabled(true);

                    if (!aborted)
                    {
                        if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 lines. Esc aborts the selection.").arg(m_elementsToPick));
                        else emit statusBarMessage(tr("Please draw one line. Esc aborts the selection."));
                    }
                }
                return;
            }
            else
            {
                m_elementsToPick = 0;
                if (p)
                {
                    QPolygonF destPolygon(0);
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
                    }
                    m_currentShapeIndices.clear();
                    emit p->userInteractionDone(ito::Shape::Line, aborted, shapes);
                    emit p->geometricShapeFinished(ito::Shape::Line, aborted);
					if (((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget()))
					{
						((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget())->updateShapes(shapes);
					}
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        }
        break;

    case ito::Shape::Rectangle:
        if (!on)
        {
            QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
            bool aborted = false;

            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
                m_currentShapeIndices.clear();
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);

                ito::Shape shape = ito::Shape::fromRectangle(QRectF(polygonScale[0], polygonScale[1]));
                DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
                newItem->setColor(m_inverseColor0, m_inverseColor1);
                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else newItem->setPen(QPen(Qt::green));

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_pShapes.insert(newItem->getIndex(), newItem);
                m_currentShapeIndices.append(newItem->getIndex());
                replot();
            }

            // if further elements are needed reset the plot engine and go ahead else finish editing
            if (m_elementsToPick > 1)
            {
                m_elementsToPick--;
                MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());
                if (m)
                {
                    m_pMultiPointPicker->setEnabled(true);

                    if (!aborted)
                    {
                        if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 rectangles. Esc aborts the selection.").arg(m_elementsToPick));
                        else emit statusBarMessage(tr("Please draw one rectangle. Esc aborts the selection."));
                    }
                }
                return;
            }
            else
            {
                m_elementsToPick = 0;
                if (p)
                {
                    QPolygonF destPolygon(0);
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
                    }
                    m_currentShapeIndices.clear();
                    emit p->userInteractionDone(ito::Shape::Rectangle, aborted, shapes);
                    emit p->geometricShapeFinished(ito::Shape::Rectangle, aborted);
					if (((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget()))
					{
						((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget())->updateShapes(shapes);
					}
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        }
        break;

    case ito::Shape::Square:
        if (!on)
        {
            QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
            bool aborted = false;

            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
                m_currentShapeIndices.clear();
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);

                ito::Shape shape = ito::Shape::fromSquare(0.5 * (polygonScale[1] + polygonScale[0]), std::abs((polygonScale[1] - polygonScale[0]).x()));
                DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
                newItem->setColor(m_inverseColor0, m_inverseColor1);
                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else newItem->setPen(QPen(Qt::green));

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_pShapes.insert(newItem->getIndex(), newItem);
                m_currentShapeIndices.append(newItem->getIndex());
                replot();
            }

            // if further elements are needed reset the plot engine and go ahead else finish editing
            if (m_elementsToPick > 1)
            {
                m_elementsToPick--;
                MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());
                if (m)
                {
                    m_pMultiPointPicker->setEnabled(true);

                    if (!aborted)
                    {
                        if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 rectangles. Esc aborts the selection.").arg(m_elementsToPick));
                        else emit statusBarMessage(tr("Please draw one rectangle. Esc aborts the selection."));
                    }
                }
                return;
            }
            else
            {
                m_elementsToPick = 0;
                if (p)
                {
                    QPolygonF destPolygon(0);
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
                    }
                    m_currentShapeIndices.clear();
                    emit p->userInteractionDone(ito::Shape::Square, aborted, shapes);
                    emit p->geometricShapeFinished(ito::Shape::Square, aborted);
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        }
        break;

    case ito::Shape::Ellipse:
        if (!on)
        {
            QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
            bool aborted = false;

            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
                m_currentShapeIndices.clear();
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);

                ito::Shape shape = ito::Shape::fromEllipse(QRectF(polygonScale[0], polygonScale[1]));
                DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
                newItem->setColor(m_inverseColor0, m_inverseColor1);
                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else newItem->setPen(QPen(Qt::green));

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_pShapes.insert(newItem->getIndex(), newItem);
                m_currentShapeIndices.append(newItem->getIndex());
                replot();
            }

            // if further elements are needed reset the plot engine and go ahead else finish editing
            if (m_elementsToPick > 1)
            {
                m_elementsToPick--;
                MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());
                if (m)
                {
                    m_pMultiPointPicker->setEnabled(true);

                    if (!aborted)
                    {
                        if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 ellipses. Esc aborts the selection.").arg(m_elementsToPick));
                        else emit statusBarMessage(tr("Please draw one ellipse. Esc aborts the selection."));
                    }
                }
                return;
            }
            else
            {
                m_elementsToPick = 0;
                if (p)
                {
                    QPolygonF destPolygon(0);
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
                    }
                    m_currentShapeIndices.clear();
                    emit p->userInteractionDone(ito::Shape::Ellipse, aborted, shapes);
                    emit p->geometricShapeFinished(ito::Shape::Ellipse, aborted);
					if (((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget()))
					{
						((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget())->updateShapes(shapes);
					}
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        }
        break;

    case ito::Shape::Circle:
        if (!on)
        {
            QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
            bool aborted = false;

            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
                m_currentShapeIndices.clear();
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);

                ito::Shape shape = ito::Shape::fromCircle(0.5 * (polygonScale[1] + polygonScale[0]), std::abs((polygonScale[1] - polygonScale[0]).x()) * 0.5);
                DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
                newItem->setColor(m_inverseColor0, m_inverseColor1);
                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else newItem->setPen(QPen(Qt::green));

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_pShapes.insert(newItem->getIndex(), newItem);
                m_currentShapeIndices.append(newItem->getIndex());
                replot();
            }

            // if further elements are needed reset the plot engine and go ahead else finish editing
            if (m_elementsToPick > 1)
            {
                m_elementsToPick--;
                MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());
                if (m)
                {
                    m_pMultiPointPicker->setEnabled(true);

                    if (!aborted)
                    {
                        if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 ellipses. Esc aborts the selection.").arg(m_elementsToPick));
                        else emit statusBarMessage(tr("Please draw one ellipse. Esc aborts the selection."));
                    }
                }
                return;
            }
            else
            {
                m_elementsToPick = 0;
                if (p)
                {
                    QPolygonF destPolygon(0);
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
                    }
                    m_currentShapeIndices.clear();
                    emit p->userInteractionDone(ito::Shape::Circle, aborted, shapes);
                    emit p->geometricShapeFinished(ito::Shape::Circle, aborted);
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        }
        break;
    }
}



//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::userInteractionStart(int type, bool start, int maxNrOfPoints)
{
    ito::RetVal retVal;

    if (start)
    {
        if (type == ito::Shape::MultiPointPick || \
            type == ito::Shape::Point || \
            type == ito::Shape::Line || \
            type == ito::Shape::Rectangle || \
            type == ito::Shape::Square || \
            type == ito::Shape::Circle || \
            type == ito::Shape::Ellipse)
        {
            m_currentShapeType = (ito::Shape::ShapeType)type;
            m_elementsToPick = maxNrOfPoints;
            setState(stateDrawShape); //this calls startOrStopDrawGeometricShape if everything is ok
        }
        else
        {
            retVal += ito::RetVal(ito::retError, 0, "invalid type for userInteractionStart");
        }
    }
    else
    {
        startOrStopDrawGeometricShape(false);
    }

    return retVal;

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::startOrStopDrawGeometricShape(bool start)
{
    ito::RetVal retval;
    ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());

    m_currentShapeIndices.clear();
    m_pMultiPointPicker->selection().clear();

    if (m_currentShapeType == ito::Shape::MultiPointPick) //multiPointPick
    {
        if (start)
        {
            m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::CrossRubberBand);
            m_pMultiPointPicker->setKeepAspectRatio(false);
            MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());
            if (m)
            {
                m->setMaxNrItems(m_elementsToPick);
                m_elementsToPick = 1;
                m_pMultiPointPicker->setEnabled(true);

                if (m->maxNrItems() > 0)
                {
                    emit statusBarMessage(tr("Please select %1 points or press Space to quit earlier. Esc aborts the selection.").arg(m->maxNrItems()));
                }
                else
                {
                    emit statusBarMessage(tr("Please select points and press Space to end the selection. Esc aborts the selection."));
                }
            }
        }
        else //start == false
        {

            m_pMultiPointPicker->setEnabled(false);

            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            m_elementsToPick = 1;

            if (p)
            {
                QVector<ito::Shape> shapes;
                emit p->userInteractionDone(m_currentShapeType, true, shapes);
            }
            setState(stateIdle);
        }
    }
    else if (m_currentShapeType == ito::Shape::Point)
    {
        if (start)
        {
            m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::CrossRubberBand);
            m_pMultiPointPicker->setKeepAspectRatio(false);
            MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());

            if (m)
            {
                m->setMaxNrItems(1);
                m_pMultiPointPicker->setEnabled(true);

                if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 points. Esc aborts the selection.").arg(m_elementsToPick));
                else emit statusBarMessage(tr("Please draw one point. Esc aborts the selection."));
            }
        }
        else //start == false
        {
            m_pMultiPointPicker->setEnabled(false);
            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            m_elementsToPick = 1;

            if (p)
            {
                QVector<ito::Shape> shapes;
                emit p->userInteractionDone(m_currentShapeType, true, shapes);
            }
            setState(stateIdle);
        }
    }
    else if (m_currentShapeType == ito::Shape::Line)
    {
        if (start)
        {
            m_pMultiPointPicker->setStateMachine(new QwtPickerDragLineMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::PolygonRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            m_pMultiPointPicker->setKeepAspectRatio(false);
            m_pMultiPointPicker->setEnabled(true);

            if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 lines. Esc aborts the selection.").arg(m_elementsToPick));
            else emit statusBarMessage(tr("Please draw one line. Esc aborts the selection."));
        }
        else //start == false
        {
            m_pMultiPointPicker->setEnabled(false);
            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            m_elementsToPick = 1;

            if (p)
            {
                QVector<ito::Shape> shapes;
                emit p->userInteractionDone(m_currentShapeType, true, shapes);
            }
            setState(stateIdle);
        }
    }
    else if (m_currentShapeType == ito::Shape::Rectangle)
    {
        if (start)
        {
            m_pMultiPointPicker->setStateMachine(new QwtPickerDragRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::RectRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            m_pMultiPointPicker->setKeepAspectRatio(false);
            m_pMultiPointPicker->setEnabled(true);

            if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 rectangles. Esc aborts the selection.").arg(m_elementsToPick));
            else emit statusBarMessage(tr("Please draw one rectangle. Esc aborts the selection."));

        }
        else //start == false
        {
            m_pMultiPointPicker->setEnabled(false);

            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            m_elementsToPick = 1;

            if (p)
            {
                QVector<ito::Shape> shapes;
                emit p->userInteractionDone(m_currentShapeType, true, shapes);
            }
            setState(stateIdle);
        }
    }
    else if (m_currentShapeType == ito::Shape::Square)
    {
        if (start)
        {
            m_pMultiPointPicker->setStateMachine(new QwtPickerDragRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::RectRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            m_pMultiPointPicker->setKeepAspectRatio(true);
            m_pMultiPointPicker->setEnabled(true);

            if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 squares. Esc aborts the selection.").arg(m_elementsToPick));
            else emit statusBarMessage(tr("Please draw one square. Esc aborts the selection."));

        }
        else //start == false
        {
            m_pMultiPointPicker->setEnabled(false);

            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            m_elementsToPick = 1;

            if (p)
            {
                QVector<ito::Shape> shapes;
                emit p->userInteractionDone(m_currentShapeType, true, shapes);
            }
            setState(stateIdle);
        }
    }
    else if (m_currentShapeType == ito::Shape::Ellipse)
    {
        if (start)
        {

            m_pMultiPointPicker->setStateMachine(new QwtPickerDragRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::EllipseRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            m_pMultiPointPicker->setKeepAspectRatio(false);
            m_pMultiPointPicker->setEnabled(true);

            if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 ellipses. Esc aborts the selection.").arg(m_elementsToPick));
            else emit statusBarMessage(tr("Please draw one ellipse. Esc aborts the selection."));
        }
        else //start == false
        {
            m_pMultiPointPicker->setEnabled(false);

            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            m_elementsToPick = 1;

            if (p)
            {
                QVector<ito::Shape> shapes;
                emit p->userInteractionDone(m_currentShapeType, true, shapes);
            }
            setState(stateIdle);
        }
    }
    else if (m_currentShapeType == ito::Shape::Circle)
    {
        if (start)
        {

            m_pMultiPointPicker->setStateMachine(new QwtPickerDragRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::EllipseRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            m_pMultiPointPicker->setKeepAspectRatio(true);
            m_pMultiPointPicker->setEnabled(true);

            if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 circles. Esc aborts the selection.").arg(m_elementsToPick));
            else emit statusBarMessage(tr("Please draw one circle. Esc aborts the selection."));
        }
        else //start == false
        {
            m_pMultiPointPicker->setEnabled(false);

            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            m_elementsToPick = 1;

            if (p)
            {
                QVector<ito::Shape> shapes;
                emit p->userInteractionDone(m_currentShapeType, true, shapes);
            }
            setState(stateIdle);
        }
    }
    else
    {
        m_pMultiPointPicker->setEnabled(false);
        
        if (p)
        {
            QVector<ito::Shape> shapes;
            emit p->userInteractionDone(m_currentShapeType, true, shapes);
        }
        setState(stateIdle);
        retval += ito::RetVal(ito::retError, 0, tr("Unknown type for userInteractionStart").toLatin1().data());
    }

    if (start)
    {
        //if spinbox for multiple planes has the focus, a possible ESC is not correctly caught.
        //therefore set the focus to the canvas.
        canvas()->setFocus();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::clearAllGeometricShapes()
{
    bool thingsToDo = m_pShapes.size() > 0;
	if (thingsToDo)
	{
		if (((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget()))
		{
			((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget())->removeShapes();
		}
	}
    //delete all geometric shapes and marker sets
    QHashIterator<int, DrawItem *> i(m_pShapes);
    while (i.hasNext())
    {
        i.next();
        delete i.value();
    }
    m_pShapes.clear();

    m_pActClearShapes->setEnabled(false);
    
    if (thingsToDo)
    {
        replot();
    }

    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());
    if (p && thingsToDo)
    {
        emit p->geometricShapesDeleted();
        p->updatePropertyDock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::deleteGeometricShape(const int id)
{
    ito::RetVal retVal;
    bool found = false;

    if (m_pShapes.contains(id))
    {
        //
        DrawItem *delItem = m_pShapes[id];
        delItem->detach();
        m_pShapes.remove(id);
        delete delItem; // ToDo check for memory leak
        found = true;
    }

    if (!found)
    {
        retVal += ito::RetVal::format(ito::retError, 0, tr("No geometric shape with id '%d' found.").toLatin1().data(), id);
    }
    else
    {
        replot();

        ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());
        if (p) emit  p->geometricShapeDeleted(id);
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
int ItomQwtPlot::getSelectedGeometricShapeIdx() const
{
    QHash<int, DrawItem*>::const_iterator it;
    for (it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
    {
        if (it.value() != NULL && it.value()->getSelected() != 0)
        {
            return it.value()->getIndex();
        }
    }

    return -1;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setSelectedGeometricShapeIdx(int idx)
{
    bool do_replot = false;
    bool failed = idx == -1 ? false : true;
    QHash<int, DrawItem*>::const_iterator it = m_pShapes.begin();
    for (; it != m_pShapes.end(); ++it)
    {
        if (it.value() != NULL && it.value()->getIndex() == idx)
        {
            it.value()->setSelected(true);
            failed = false;
            do_replot = true;
            continue;
        }
        if (it.value() != NULL && (it.value()->getSelected()))
        {
            do_replot = true;
            it.value()->setSelected(false);
        }
    }

    if (do_replot) replot();
    if (failed) emit statusBarMessage(tr("Could not set active element, index out of range."), 12000);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::setGeometricShapes(const QVector<ito::Shape> &geometricShapes)
{
    ito::RetVal retVal;
    clearAllGeometricShapes();
    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());


    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Could not plot marker, api is missing").toLatin1().data());
    }
    else
    {
        int nrOfShapes = geometricShapes.size();
            
        DrawItem *newItem = NULL;
        // The definition do not correspond to the definetion of primitiv elements

        foreach(ito::Shape shape, geometricShapes)
        {
            if (m_pShapes.contains(shape.index()))
            {
                m_pShapes[shape.index()]->setShape(shape, m_inverseColor0, m_inverseColor1);
				if (((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget()))
				{
					((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget())->updateShape(shape);
				}
            }
            else
            {
                switch (shape.type())
                {
                case ito::Shape::MultiPointPick:
                case ito::Shape::Point:
                case ito::Shape::Line:
                case ito::Shape::Rectangle:
                case ito::Shape::Square:
                case ito::Shape::Ellipse:
                case ito::Shape::Circle:
                    newItem = new DrawItem(shape, m_shapeModificationModes, this, &retVal, m_shapesLabelVisible);
                    if (!retVal.containsError())
                    {
                        newItem->setColor(m_inverseColor0, m_inverseColor1);
                        newItem->setVisible(true);
                        newItem->show();
                        newItem->attach(this);
                        m_pShapes.insert(newItem->getIndex(), newItem);
						shape.setIndex(newItem->getIndex());
						if (((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget()))
						{
							((PlotInfoShapes*)((ItomQwtDObjFigure*)(this->parent()))->ShapesWidget())->updateShape(shape);
						}
                    }
                    break;

                default:
                    retVal += ito::RetVal(ito::retError, 0, tr("invalid marker type").toLatin1().data());
                    break;
                }

                if (newItem)
                {
                    
                }
				
            }

            replot();
        }
    }

    m_pActClearShapes->setEnabled(m_plottingEnabled && countGeometricShapes() > 0);
	
	if (retVal.hasErrorMessage())
    {
        emit statusBarMessage(retVal.errorMessage(), 12000);
    }
    
    if (p) emit  p->geometricShapeFinished(0, retVal.containsError());

    return retVal;
}


//----------------------------------------------------------------------------------------------------------------------------------
QVector<ito::Shape> ItomQwtPlot::getGeometricShapes()
{
    QVector<ito::Shape> shapes;
    shapes.reserve(m_pShapes.size());
    QHash<int, DrawItem*>::Iterator it = m_pShapes.begin();

    for (; it != m_pShapes.end(); it++)
    {
        if (it.value() == NULL)
        {
            shapes.append(ito::Shape());

        }
        else
        {
            shapes.append(it.value()->getShape());
        }
    }

    return shapes;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::setGeometricShapeLabel(int id, const QString &label)
{
    if (!m_pShapes.contains(id))
    {
        return ito::RetVal(ito::retError, 0, tr("Geometric element not found").toLatin1().data());
    }

    m_pShapes[id]->setLabel(label);
    replot();

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::setGeometricShapeLabelVisible(int id, bool setVisible)
{
    if (!m_pShapes.contains(id))
    {
        return ito::RetVal(ito::retError, 0, tr("Geometric element not found").toLatin1().data());
    }

    m_pShapes[id]->setLabelVisible(setVisible);
    replot();

    return ito::retOk;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::exportCanvas(const bool copyToClipboardNotFile, const QString &fileName, QSizeF curSize /*= QSizeF(0.0,0.0)*/, const int resolution /*= 300*/)
{
    if (curSize.height() == 0 || curSize.width() == 0)
    {
        curSize = size();
    }
	ItomQwtDObjFigure* hMyParent = (ItomQwtDObjFigure*)parent();

    QBrush curBrush = canvasBackground();
    QPalette curPalette = palette();

    setAutoFillBackground(true);
    setPalette(Qt::white);
    setCanvasBackground(Qt::white);

    replot();

    QwtPlotRenderer renderer;

	QList< QRectF > plotContentWidgetCoords;
	plotContentWidgetCoords << QRectF(QPointF(hMyParent->x(), hMyParent->y()), hMyParent->size());
	qDebug("x %i, y %i, sizeX %i, sizeY %i", hMyParent->x(), hMyParent->y(), hMyParent->size().width(), hMyParent->size().height());
    // flags to make the document look like the widget
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    //renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true); //deprecated in qwt 6.1.0

    if (copyToClipboardNotFile)
    {
		bool plotInfoVisible = false;
		qreal resFaktor = std::max(qRound(resolution / 72.0), 1);
		QSize myRect(curSize.width() * resFaktor, curSize.height() * resFaktor);

		QClipboard *clipboard = QApplication::clipboard();

		if ((hMyParent->MarkerWidget()  && ((QWidget*)hMyParent->MarkerWidget())->isVisible()) ||
			(hMyParent->PickerWidget()  && ((QWidget*)hMyParent->PickerWidget())->isVisible()) ||
			(hMyParent->DObjectWidget() && ((QWidget*)hMyParent->DObjectWidget())->isVisible()) ||
			(hMyParent->ShapesWidget()  && ((QWidget*)hMyParent->ShapesWidget())->isVisible()))
		{
			plotInfoVisible = true;
			emit statusBarMessage(tr("copy current view to clipboard including infoWidgets ..."));
			
			for each (ito::AbstractFigure::ToolboxItem item in hMyParent->getToolboxes())
			{
				if (item.toolbox && item.toolbox->isVisible() /*&& item.toolbox->widget()->isVisible()*/)
				{
					plotContentWidgetCoords << QRectF(QPointF(item.toolbox->x(), item.toolbox->y()), item.toolbox->size());
					qDebug("x %i, y %i, sizeX %i, sizeY %i", item.toolbox->x(), item.toolbox->y(), item.toolbox->size().width(), item.toolbox->size().height());
				}
			}


		}
		else
		{
			emit statusBarMessage(tr("copy current view to clipboard ..."));
		}
        
        QImage img(myRect, QImage::Format_ARGB32);
        QPainter painter(&img);
        painter.scale(resFaktor, resFaktor);
		


		if (plotInfoVisible)
		{

		}
		else
		{
			renderer.render(this, &painter, rect());
		}

        img.setDotsPerMeterX(img.dotsPerMeterX() * resFaktor); //setDotsPerMeterXY must be set after rendering
        img.setDotsPerMeterY(img.dotsPerMeterY() * resFaktor);
        clipboard->setImage(img);
		if (plotInfoVisible)
		{
			emit statusBarMessage(tr("copy current view to clipboard including infoWidgets. Done."), 1000);
		}
		else
		{
			emit statusBarMessage(tr("copy current view to clipboard. Done."), 1000);
		}
    }
    else
    {
        renderer.renderDocument(this, fileName, curSize, resolution);
    }

    setPalette(curPalette);
    setCanvasBackground(curBrush);

    replot();
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuActSave()
{
    //first get the output format information, then the filename (in order to let the user see what can be adjusted before defining a filename)
    bool abort = true;

    QSizeF curSize = size();
    int resolution = 300;

    DialogExportProperties *dlg = new DialogExportProperties("", curSize, qobject_cast<ItomQwtDObjFigure*>(parent()));
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData(curSize, resolution);

        abort = false;
    }

    delete dlg;
    dlg = NULL;

    if (abort)
    {
        return;
    }

    static QString saveDefaultPath;

#ifndef QT_NO_PRINTER
    QString fileName = "plot.pdf";
#else
    QString fileName = "plot.png";
#endif

    if (saveDefaultPath == "")
    {
        saveDefaultPath = QDir::currentPath();
    }

#ifndef QT_NO_FILEDIALOG
    const QList<QByteArray> imageFormats =
        QImageWriter::supportedImageFormats();

    QStringList filter;
    filter += tr("PDF Documents (*.pdf)");
#ifndef QWT_NO_SVG
#ifdef QT_SVG_LIB
    filter += tr("SVG Documents (*.svg)");
#endif
#endif
    filter += tr("Postscript Documents (*.ps)");

    if (imageFormats.size() > 0)
    {
        QString imageFilter(tr("Images ("));
        for (int i = 0; i < imageFormats.size(); i++)
        {
            if (i > 0)
                imageFilter += " ";
            imageFilter += "*.";
            imageFilter += imageFormats[i];
        }
        imageFilter += ")";

        filter += imageFilter;
    }

    QDir file(saveDefaultPath);
    fileName = QFileDialog::getSaveFileName(
        this, tr("Export File Name"), file.absoluteFilePath(fileName),
        filter.join(";;"), NULL, QFileDialog::DontConfirmOverwrite);
#endif

    if (!fileName.isEmpty())
    {
        QFileInfo fi(fileName);
        saveDefaultPath = fi.path();

        exportCanvas(false, fileName, curSize, resolution);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuActHome()
{
    home();
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuActRatio(bool checked)
{
    setKeepAspectRatio(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setPlottingEnabled(bool enabled)
{
    m_plottingEnabled = enabled;
    setState(m_state); //modifies actions... corresponding to plotting enabled
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuCopyToClipboard()
{
    exportCanvas(true, "");
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuSendCurrentToWorkspace()
{
    bool ok;
    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(parent());

    QString varname = QInputDialog::getText(p, tr("Current to workspace"), tr("Indicate the python variable name for the currently visible object"), QLineEdit::Normal, "zoom_object", &ok);
    if (ok && varname != "")
    {
        QSharedPointer<ito::DataObject> obj = p->getDisplayed();
        const ito::DataObject *dobj = &(*obj);
        QSharedPointer<ito::ParamBase> obj_(new ito::ParamBase("displayed", ito::ParamBase::DObjPtr, (const char*)dobj));

        QApplication::setOverrideCursor(Qt::WaitCursor);

        ito::RetVal retval = apiSendParamToPyWorkspace(varname, obj_);

        QApplication::restoreOverrideCursor();

        if (retval.containsError())
        {
            QMessageBox msgBox;
            msgBox.setText(tr("Error sending data object to workspace").toLatin1().data());
            if (retval.errorMessage())
            {
                msgBox.setInformativeText(QLatin1String(retval.errorMessage()));
            }
            msgBox.setIcon(QMessageBox::Critical);
            msgBox.exec();
        }
        else if (retval.containsWarning())
        {
            QMessageBox msgBox;
            msgBox.setText(tr("Error sending data object to workspace").toLatin1().data());
            if (retval.errorMessage())
            {
                msgBox.setInformativeText(QLatin1String(retval.errorMessage()));
            }
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.exec();
        }
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::plotMarkers(const QSharedPointer<ito::DataObject> coordinates, const QString &style, const QString &id, int plane)
{
    ito::RetVal retval;
    int limits[] = { 2, 2, 0, std::numeric_limits<int>::max() };

	QString tmpID = id;
	if (tmpID == "")
	{
		tmpID = "undef";
		int cnt = 0;
		while (m_plotMarkers.contains(tmpID))
		{
			tmpID = QString("undef%1").arg(cnt);
			cnt++;
		}
	}
    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        emit statusBarMessage(tr("Could not plot marker, api is missing"), 4000);
        return ito::RetVal(ito::retError, 0, tr("Could not plot marker, api is missing").toLatin1().data());
    }

    ito::DataObject *dObj = apiCreateFromDataObject(coordinates.data(), 2, ito::tFloat32, limits, &retval);

    if (!retval.containsError())
    {
        QwtSymbol::Style symStyle = QwtSymbol::XCross;
        QSize symSize(5, 5);
        QBrush symBrush(Qt::NoBrush);
        QPen symPen(Qt::red);

        QRegExp rgexp("^([b|g|r|c|m|y|k|w]?)([.|o|s|d|\\^|v|<|>|x|+|*|h]?)(\\d*)(;(\\d*))?$");
        if (rgexp.indexIn(style) != -1)
        {
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
                symSize = QSize(size, size);
            }

            size = rgexp.cap(5).toInt(&ok);
            if (ok)
            {
                symPen.setWidth(size);
            }
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, tr("The style tag does not correspond to the required format").toLatin1().data());
        }

        QwtPlotMarker *marker = NULL;
        int nrOfMarkers = dObj->getSize(1);

        const cv::Mat *mat = dObj->getCvPlaneMat(0);

        const ito::float32 *xRow = mat->ptr<const ito::float32>(0);
        const ito::float32 *yRow = mat->ptr<const ito::float32>(1);

		QPolygonF markerPolygon;
		markerPolygon.clear();

        for (int i = 0; i < nrOfMarkers; ++i)
        {
            marker = new QwtPlotMarker();
            marker->setSymbol(new QwtSymbol(symStyle, symBrush, symPen, symSize));
            marker->setValue(xRow[i], yRow[i]);
            marker->attach(this);
			
			markerPolygon.append(QPointF(xRow[i], yRow[i]));

            if (m_markerLabelVisible)
            {
                QwtText label(QString(" %1").arg(tmpID));
                marker->setLabel(label);
            }

			m_plotMarkers.insert(tmpID, QPair<int, QwtPlotMarker*>(plane, marker));
        }

		if (((PlotInfoMarker*)((ItomQwtDObjFigure*)(this->parent()))->MarkerWidget()))
		{
			ito::Shape shapes = ito::Shape::fromMultipoint(markerPolygon, m_plotMarkers.size(), tmpID);
			((PlotInfoMarker*)((ItomQwtDObjFigure*)(this->parent()))->MarkerWidget())->updateMarker(shapes);
		}

        replot();
    }

    if (dObj) delete dObj;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::deleteMarkers(const QString &id)
{
    ito::RetVal retval;
    bool found = false;
    QMutableHashIterator<QString, QPair<int, QwtPlotMarker*> > i(m_plotMarkers);
    while (i.hasNext())
    {
        i.next();
        if (i.key() == id || id == "")
        {
            i.value().second->detach();
            delete i.value().second;
            found = true;
            i.remove();
        }
    }

    if (!found && id != "")
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("No marker with id '%1' found.").arg(id).toLatin1().data());
    }
    else
    {
        replot();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setMarkerLabelVisible(bool visible)
{
    if (m_markerLabelVisible != visible)
    {
        if (visible)
        {
            QPair<int, QwtPlotMarker*> pair;
            QList<QString> keys = m_plotMarkers.keys();
            QString label;
            foreach(label, keys)
            {
                foreach(pair, m_plotMarkers.values(label))
                {
                    if (pair.second) pair.second->setLabel(label);
                }
            }
        }
        else
        {
            QPair<int, QwtPlotMarker*> pair;

            foreach(pair, m_plotMarkers)
            {
                if (pair.second) pair.second->setLabel(QwtText());
            }
        }
    }

    m_markerLabelVisible = visible;

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setShapesLabelVisible(bool visible)
{
    if (m_shapesLabelVisible != visible)
    {
        foreach(DrawItem* d, m_pShapes)
        {
            d->setLabelVisible(visible);
        }

        replot();
    }

    m_shapesLabelVisible = visible;
}
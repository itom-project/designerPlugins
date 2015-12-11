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
    m_activeShapeIdx(-1),
    m_ignoreNextMouseEvent(false),
    m_shapeModificationType(ItomQwtPlotEnums::ModifyPoints),
    m_inverseColor0(Qt::green),
    m_inverseColor1(Qt::blue),
    m_elementsToPick(1),
    m_currentShapeType(ito::PrimitiveContainer::tPoint),
    m_buttonStyle(0),
    m_plottingEnabled(true),
    m_markerLabelVisible(false),
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
    m_pMenuShapeModifyType(NULL),
    m_pActProperties(NULL),
    m_pActShapeType(NULL),
    m_pActShapeModifyType(NULL)
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
    a->setData(ito::PrimitiveContainer::tPoint);
    m_pMenuShapeType->addAction(a);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Line"));
    a->setData(ito::PrimitiveContainer::tLine);
    m_pMenuShapeType->addAction(a);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Rectangle"));
    a->setData(ito::PrimitiveContainer::tRectangle);
    m_pMenuShapeType->addAction(a);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Ellipse"));
    a->setData(ito::PrimitiveContainer::tEllipse);
    m_pMenuShapeType->addAction(a);
    a->setCheckable(true);

    m_pActShapeType->setData(ito::PrimitiveContainer::tPoint);
    m_pActShapeType->setVisible(true);
    m_pActShapeType->setCheckable(true);
    connect(m_pMenuShapeType, SIGNAL(triggered(QAction*)), this, SLOT(mnuGroupShapeTypes(QAction*)));
    connect(m_pActShapeType, SIGNAL(triggered(bool)), this, SLOT(mnuShapeType(bool)));


    //m_pMenuShapeModifyType
    m_pActShapeModifyType = new QAction(tr("Geometric shape modification mode"), p);
    m_pMenuShapeModifyType = new QMenu(tr("Geometric shape modification mode"), p);
    m_pActShapeModifyType->setMenu(m_pMenuShapeModifyType);

    a = m_pMenuShapeModifyType->addAction(tr("Move elements"));
    a->setData(ItomQwtPlotEnums::MoveGeometricElements);
    m_pMenuShapeModifyType->addAction(a);
    a->setCheckable(false);

    a = m_pMenuShapeModifyType->addAction(tr("Resize elements"));
    a->setData(ItomQwtPlotEnums::RotateGeometricElements);
    m_pMenuShapeModifyType->addAction(a);
    a->setCheckable(false);
    a->setEnabled(false);

    a = m_pMenuShapeModifyType->addAction(tr("Rotate elements"));
    a->setData(ItomQwtPlotEnums::ResizeGeometricElements);
    m_pMenuShapeModifyType->addAction(a);
    a->setCheckable(false);
    a->setEnabled(false);

    a = m_pMenuShapeModifyType->addAction(tr("Modify points"));
    a->setData(ItomQwtPlotEnums::ModifyPoints);
    m_pMenuShapeModifyType->addAction(a);
    m_pMenuShapeModifyType->setDefaultAction(a);
    a->setCheckable(false);

    m_pActShapeModifyType->setVisible(true);
    m_pActShapeModifyType->setCheckable(true);
    connect(m_pMenuShapeModifyType, SIGNAL(triggered(QAction*)), this, SLOT(mnuGroupShapeModifyTypes(QAction*)));
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

    QHash<int, DrawItem*>::iterator it = m_pShapes.begin();
    for (; it != m_pShapes.end(); ++it)
    {
        if (it.value() != NULL && it.value()->m_autoColor)
        {
            it.value()->setColor(m_inverseColor1, m_inverseColor0);
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
        m_pMenuShapeModifyType->actions()[0]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/geosMove.png"));
        m_pMenuShapeModifyType->actions()[1]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/geosResize.png"));
        m_pMenuShapeModifyType->actions()[2]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/geosRotate.png"));
        m_pMenuShapeModifyType->actions()[3]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/geosPoints.png"));
        m_pActShapeModifyType->setIcon(m_pMenuShapeModifyType->defaultAction()->icon());

        switch (m_currentShapeType)
        {
            default:
            case ito::PrimitiveContainer::tPoint:
                m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/marker.png"));
                break;
            case ito::PrimitiveContainer::tLine:
                m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"));
                break;
            case ito::PrimitiveContainer::tRectangle:
                m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/rectangle.png"));
                break;
            case ito::PrimitiveContainer::tEllipse:
                m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/ellipse.png"));
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
        m_pMenuShapeModifyType->actions()[0]->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/geosMove_lt.png"));
        m_pMenuShapeModifyType->actions()[1]->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/geosResize_lt.png"));
        m_pMenuShapeModifyType->actions()[2]->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/geosRotate_lt.png"));
        m_pMenuShapeModifyType->actions()[3]->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/geosPoints_lt.png"));
        m_pActShapeModifyType->setIcon(m_pMenuShapeModifyType->defaultAction()->icon());

        switch (m_currentShapeType)
        {
        default:
        case ito::PrimitiveContainer::tPoint:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/marker_lt.png"));
            break;
        case ito::PrimitiveContainer::tLine:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/pntline_lt.png"));
            break;
        case ito::PrimitiveContainer::tRectangle:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/rectangle_lt.png"));
            break;
        case ito::PrimitiveContainer::tEllipse:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/ellipse_lt.png"));
            break;
        }
    }

    
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setInverseColors(const QColor &color0, const QColor &color1)
{
    m_inverseColor0 = color0;
    m_inverseColor1 = color1;
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
void ItomQwtPlot::setShapeModificationType(const ItomQwtPlotEnums::ModificationState &type)
{
    switch (type)
    {
    default:
    case ItomQwtPlotEnums::MoveGeometricElements:
        m_shapeModificationType = ItomQwtPlotEnums::MoveGeometricElements;
        m_pMenuShapeModifyType->setDefaultAction(m_pMenuShapeModifyType->actions()[0]);
        m_pMenuShapeModifyType->setIcon(m_pMenuShapeModifyType->defaultAction()->icon());
        break;
    case ItomQwtPlotEnums::ResizeGeometricElements:
        m_shapeModificationType = ItomQwtPlotEnums::ResizeGeometricElements;
        m_pMenuShapeModifyType->setDefaultAction(m_pMenuShapeModifyType->actions()[1]);
        m_pMenuShapeModifyType->setIcon(m_pMenuShapeModifyType->defaultAction()->icon());
        break;
    case ItomQwtPlotEnums::RotateGeometricElements:
        m_shapeModificationType = ItomQwtPlotEnums::RotateGeometricElements;
        m_pMenuShapeModifyType->setDefaultAction(m_pMenuShapeModifyType->actions()[2]);
        m_pMenuShapeModifyType->setIcon(m_pMenuShapeModifyType->defaultAction()->icon());
        break;
    case ItomQwtPlotEnums::ModifyPoints:
        m_shapeModificationType = ItomQwtPlotEnums::ModifyPoints;
        m_pMenuShapeModifyType->setDefaultAction(m_pMenuShapeModifyType->actions()[3]);
        m_pMenuShapeModifyType->setIcon(m_pMenuShapeModifyType->defaultAction()->icon());
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuGroupShapeModifyTypes(QAction *action)
{
    switch (action->data().toInt())
    {
    default:
    case ItomQwtPlotEnums::MoveGeometricElements:
        setShapeModificationType(ItomQwtPlotEnums::MoveGeometricElements);
        break;
    case ItomQwtPlotEnums::ResizeGeometricElements:
        setShapeModificationType(ItomQwtPlotEnums::ResizeGeometricElements);
        break;
    case ItomQwtPlotEnums::RotateGeometricElements:
        setShapeModificationType(ItomQwtPlotEnums::RotateGeometricElements);
        break;
    case ItomQwtPlotEnums::ModifyPoints:
        setShapeModificationType(ItomQwtPlotEnums::ModifyPoints);
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuGroupShapeTypes(QAction *action)
{
    switch (action->data().toInt())
    {
    case ito::PrimitiveContainer::tPoint:
        m_currentShapeType = ito::PrimitiveContainer::tPoint;
        break;
    case ito::PrimitiveContainer::tLine:
        m_currentShapeType = ito::PrimitiveContainer::tLine;
        break;
    case ito::PrimitiveContainer::tRectangle:
        m_currentShapeType = ito::PrimitiveContainer::tRectangle;
        break;
    case ito::PrimitiveContainer::tEllipse:
        m_currentShapeType = ito::PrimitiveContainer::tEllipse;
        break;
    default:
        m_currentShapeType = ito::PrimitiveContainer::tNoType;
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
                case ito::PrimitiveContainer::tPoint:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/marker.png" : ":/itomDesignerPlugins/plot/icons/marker_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(1);
                    break;

                case ito::PrimitiveContainer::tLine:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/pntline.png" : ":/itomDesignerPlugins/plot/icons/pntline_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(1);
                    break;

                case ito::PrimitiveContainer::tRectangle:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/rectangle.png" : ":/itomDesignerPlugins/plot/icons/rectangle_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(1);
                    break;

                case ito::PrimitiveContainer::tEllipse:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/ellipse.png" : ":/itomDesignerPlugins/plot/icons/ellipse_lt.png"));
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

        QHash<int, DrawItem*>::iterator it;
        for (it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
        {
            if (it.value() == NULL)
            {
                continue;
            }

            if (fabs(transform(QwtPlot::xBottom, it.value()->x1) - canxpos) < 10
                && fabs(transform(QwtPlot::yLeft, it.value()->y1) - canypos) < 10)
            {
                it.value()->m_active = 1;
                m_activeShapeIdx = it.value()->m_idx;
                it.value()->setActive(1);
                it.value()->setSelected(true);

                m_initialMousePosition.setX(event->x());
                m_initialMousePosition.setY(event->y());
                m_initialMarkerPosition.setX(it.value()->x1);
                m_initialMarkerPosition.setY(it.value()->y1);
                ++it;
                break;
            }
            else if (fabs(transform(QwtPlot::xBottom, it.value()->x2) - canxpos) < 10
                && fabs(transform(QwtPlot::yLeft, it.value()->y2) - canypos) < 10)
            {
                it.value()->m_active = 2;
                m_activeShapeIdx = it.value()->m_idx;
                it.value()->setActive(2);
                it.value()->setSelected(true);
                m_initialMousePosition.setX(event->x());
                m_initialMousePosition.setY(event->y());
                m_initialMarkerPosition.setX(it.value()->x1);
                m_initialMarkerPosition.setY(it.value()->y1);
                ++it;
                break;
            }
            else
            {
                it.value()->setSelected(false);
                it.value()->setActive(0);
            }
        }

        for (; it != m_pShapes.end(); ++it)
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

    //    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
    if (m_state == stateIdle)
    {
        event->accept();

        ito::float32 canxpos = invTransform(QwtPlot::xBottom, event->x() - canvas()->x());
        ito::float32 canypos = invTransform(QwtPlot::yLeft, event->y() - canvas()->y());

        bool modificationDone = false;

        QHash<int, DrawItem*>::Iterator it = m_pShapes.begin();
        for (; it != m_pShapes.end(); it++)
        {
            if (it.value() == NULL)
            {
                continue;
            }

            if (it.value()->m_active == 0)
            {
                continue;
            }

            ito::float32 dx, dy;
            QPainterPath path;

            switch (m_shapeModificationType)
            {
            default:
                emit statusBarMessage(tr("Could not perform specific action on geomtric element, action not implemented."), 4000);
                path = it.value()->shape();
                break;
            case ItomQwtPlotEnums::MoveGeometricElements:
            {
                ito::float32 lenx = it.value()->x2 - it.value()->x1;
                ito::float32 leny = it.value()->y2 - it.value()->y1;

                if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                {
                    dx = invTransform(QwtPlot::xBottom, event->x()) - invTransform(QwtPlot::xBottom, m_initialMousePosition.x());
                    dy = invTransform(QwtPlot::yLeft, event->y()) - invTransform(QwtPlot::yLeft, m_initialMousePosition.y());
                    dx = fabs(dx) <= std::numeric_limits<ito::float32>::epsilon() ? std::numeric_limits<ito::float32>::epsilon() : dx;
                    dy = fabs(dy) <= std::numeric_limits<ito::float32>::epsilon() ? -std::numeric_limits<ito::float32>::epsilon() : dy;

                    if (it.value()->m_active == 1)
                    {
                        if (fabs(dx) > fabs(dy))
                        {
                            dx = canxpos - m_initialMarkerPosition.x();
                            dy = 0.0;
                        }
                        else
                        {
                            dy = canypos - m_initialMarkerPosition.y();
                            dx = 0.0;
                        }

                    }
                    else
                    {
                        if (fabs(dx) > fabs(dy))
                        {
                            dx = canxpos - m_initialMarkerPosition.x() - lenx;
                            dy = 0.0;
                        }
                        else
                        {
                            dy = canypos - m_initialMarkerPosition.y() - leny;
                            dx = 0.0;
                        }
                    }
                }
                else
                {
                    dx = 0.0;
                    dy = 0.0;
                    if (it.value()->m_active == 2)
                    {
                        canxpos -= lenx;
                        canypos -= leny;
                    }
                }

                switch (it.value()->m_type)
                {
                case ito::PrimitiveContainer::tPoint:
                {
                    if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                    {
                        path.moveTo(m_initialMarkerPosition.x() + dx, m_initialMarkerPosition.y() + dy);
                        path.lineTo(m_initialMarkerPosition.x() + dx, m_initialMarkerPosition.y() + dy);
                    }
                    else
                    {
                        path.moveTo(canxpos, canypos);
                        path.lineTo(canxpos, canypos);
                    }
                }
                break;
                case ito::PrimitiveContainer::tLine:
                {
                    if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                    {
                        path.moveTo(m_initialMarkerPosition.x() + dx, m_initialMarkerPosition.y() + dy);
                        path.lineTo(m_initialMarkerPosition.x() + dx + lenx, m_initialMarkerPosition.y() + dy + leny);
                    }
                    else
                    {
                        path.moveTo(canxpos, canypos);
                        path.lineTo(canxpos + lenx, canypos + leny);
                    }
                }
                break;

                case ito::PrimitiveContainer::tRectangle:
                {
                    if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                    {
                        path.addRect(m_initialMarkerPosition.x() + dx, m_initialMarkerPosition.y() + dy, lenx, leny);
                    }
                    else
                    {
                        path.addRect(canxpos, canypos, lenx, leny);
                    }
                }
                break;

                case ito::PrimitiveContainer::tEllipse:
                {
                    if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                    {
                        path.addEllipse(m_initialMarkerPosition.x() + dx, m_initialMarkerPosition.y() + dy, lenx, leny);
                    }
                    else
                    {
                        path.addEllipse(canxpos, canypos, lenx, leny);
                    }
                }
                break;
                }
                it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                it.value()->setActive(it.value()->m_active);
                replot();
            }

            break;
            case ItomQwtPlotEnums::ResizeGeometricElements:
                emit statusBarMessage(tr("Could not perform specific action on geomtric element, resize action not implemented yet."), 4000);
                break;
            case ItomQwtPlotEnums::ModifyPoints:
            {
                if (it.value()->m_flags & 0x07)
                {
                    emit statusBarMessage(tr("Could not change points of geomtric element, elemet is proteted."), 4000);
                    break;
                }

                if (it.value()->m_active == 1)
                {


                    switch (it.value()->m_type)
                    {
                    case ito::PrimitiveContainer::tPoint:

                        path.moveTo(canxpos, canypos);
                        path.lineTo(canxpos, canypos);

                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);
                        replot();
                        break;

                    case ito::PrimitiveContainer::tLine:

                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)       // draw line horizontal or vertical with second point fixed
                        {

                            dx = canxpos - it.value()->x2;
                            dy = it.value()->y2 - canypos;

                            dx = fabs(dx) <= std::numeric_limits<ito::float32>::epsilon() ? std::numeric_limits<ito::float32>::epsilon() : dx;
                            dy = fabs(dy) <= std::numeric_limits<ito::float32>::epsilon() ? -std::numeric_limits<ito::float32>::epsilon() : dy;

                            if (fabs(dx) > fabs(dy))
                            {
                                path.moveTo(canxpos, it.value()->y2);
                                path.lineTo(it.value()->x2, it.value()->y2);
                            }
                            else
                            {
                                path.moveTo(it.value()->x2, canypos);
                                path.lineTo(it.value()->x2, it.value()->y2);
                            }
                        }
                        else if (QApplication::keyboardModifiers() == Qt::ShiftModifier)    // move line without resize
                        {

                            dx = it.value()->x2 - it.value()->x1;
                            dy = it.value()->y2 - it.value()->y1;

                            path.moveTo(canxpos, canypos);
                            path.lineTo(canxpos + dx, canypos + dy);
                        }
                        else if (QApplication::keyboardModifiers() == Qt::AltModifier)      // keep linesize constant and second point fixed
                        {
                            dx = it.value()->x2 - it.value()->x1;
                            dy = it.value()->y2 - it.value()->y1;

                            ito::float32 length = sqrt(dx * dx + dy * dy);

                            dx = canxpos - it.value()->x2;
                            dy = canypos - it.value()->y2;

                            ito::float32 alpha = atan2(dy, dx);

                            dx = cos(alpha) * length;
                            dy = sin(alpha) * length;

                            path.moveTo(it.value()->x2 + dx, it.value()->y2 + dy);
                            path.lineTo(it.value()->x2, it.value()->y2);
                        }
                        else                                                                // just draw line
                        {
                            path.moveTo(canxpos, canypos);
                            path.lineTo(it.value()->x2, it.value()->y2);
                        }
                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);
                        replot();
                        break;

                    case ito::PrimitiveContainer::tRectangle:
                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                        {
                            dx = it.value()->x2 - canxpos;
                            dy = canypos - it.value()->y2;

                            dx = fabs(dx) <= std::numeric_limits<ito::float32>::epsilon() ? std::numeric_limits<ito::float32>::epsilon() : dx;
                            dy = fabs(dy) <= std::numeric_limits<ito::float32>::epsilon() ? -std::numeric_limits<ito::float32>::epsilon() : dy;

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

                        path.addRect(canxpos, canypos,
                            it.value()->x2 - canxpos,
                            it.value()->y2 - canypos);
                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);

                        if (m_ignoreNextMouseEvent)
                        {
                            ito::float32 destPosX = transform(QwtPlot::xBottom, canxpos);
                            ito::float32 destPosY = transform(QwtPlot::yLeft, canypos);

                            QPoint dst = canvas()->mapToGlobal(QPoint(destPosX, destPosY));

                            this->cursor().setPos(dst);
                        }
                        replot();
                        break;

                    case ito::PrimitiveContainer::tEllipse:
                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                        {
                            dx = it.value()->x2 - canxpos;
                            dy = canypos - it.value()->y2;

                            dx = fabs(dx) <= std::numeric_limits<ito::float32>::epsilon() ? std::numeric_limits<ito::float32>::epsilon() : dx;
                            dy = fabs(dy) <= std::numeric_limits<ito::float32>::epsilon() ? -std::numeric_limits<ito::float32>::epsilon() : dy;

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
                        path.addEllipse(canxpos,
                            canypos,
                            it.value()->x2 - canxpos,
                            it.value()->y2 - canypos);
                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);

                        if (m_ignoreNextMouseEvent)
                        {
                            ito::float32 destPosX = transform(QwtPlot::xBottom, canxpos);
                            ito::float32 destPosY = transform(QwtPlot::yLeft, canypos);

                            QPoint dst = canvas()->mapToGlobal(QPoint(destPosX, destPosY));

                            this->cursor().setPos(dst);
                        }
                        replot();
                        break;
                    }
                }
                else if (it.value()->m_active == 2)
                {

                    if (it.value()->m_flags & 0x07)
                    {
                        emit statusBarMessage(tr("Could not change geomtric element, elemet is read only."), 4000);
                        break;
                    }

                    ito::float32 dx, dy;

                    QPainterPath path;
                    switch (it.value()->m_type)
                    {
                    case ito::PrimitiveContainer::tLine:
                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                        {
                            dx = canxpos - it.value()->x1;
                            dy = it.value()->y1 - canypos;

                            if (fabs(dx) > fabs(dy))
                            {
                                path.moveTo(it.value()->x1, it.value()->y1);
                                path.lineTo(canxpos, it.value()->y1);
                            }
                            else
                            {
                                path.moveTo(it.value()->x1, it.value()->y1);
                                path.lineTo(it.value()->x1, canypos);
                            }
                        }
                        else if (QApplication::keyboardModifiers() == Qt::ShiftModifier)
                        {

                            dx = it.value()->x2 - it.value()->x1;
                            dy = it.value()->y2 - it.value()->y1;

                            path.moveTo(canxpos - dx, canypos - dy);
                            path.lineTo(canxpos, canypos);
                        }
                        else if (QApplication::keyboardModifiers() == Qt::AltModifier)
                        {
                            dx = it.value()->x2 - it.value()->x1;
                            dy = it.value()->y2 - it.value()->y1;

                            ito::float32 length = sqrt(dx * dx + dy * dy);

                            dx = it.value()->x1 - canxpos;
                            dy = it.value()->y1 - canypos;

                            ito::float32 alpha = atan2(dy, dx);

                            dx = cos(alpha) * length;
                            dy = sin(alpha) * length;

                            path.moveTo(it.value()->x1, it.value()->y1);
                            path.lineTo(it.value()->x1 - dx, it.value()->y1 - dy);
                        }
                        else
                        {
                            path.moveTo(it.value()->x1, it.value()->y1);
                            path.lineTo(canxpos, canypos);
                        }

                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);
                        //if (p) emit p->plotItemChanged(n);
                        replot();
                        break;

                    case ito::PrimitiveContainer::tRectangle:
                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                        {
                            dx = canxpos - it.value()->x1;
                            dy = it.value()->y1 - canypos;

                            dx = fabs(dx) <= std::numeric_limits<ito::float32>::epsilon() ? std::numeric_limits<ito::float32>::epsilon() : dx;
                            dy = fabs(dy) <= std::numeric_limits<ito::float32>::epsilon() ? -std::numeric_limits<ito::float32>::epsilon() : dy;

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
                        path.addRect(it.value()->x1, it.value()->y1,
                            canxpos - it.value()->x1,
                            canypos - it.value()->y1);
                        it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);
                        //if (p) emit p->plotItemChanged(n);
                        if (m_ignoreNextMouseEvent)
                        {
                            ito::float32 destPosX = transform(QwtPlot::xBottom, canxpos);
                            ito::float32 destPosY = transform(QwtPlot::yLeft, canypos);

                            QPoint dst = canvas()->mapToGlobal(QPoint(destPosX, destPosY));

                            this->cursor().setPos(dst);
                        }
                        replot();
                        break;

                    case ito::PrimitiveContainer::tEllipse:
                        if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                        {
                            dx = canxpos - it.value()->x1;
                            dy = it.value()->y1 - canypos;

                            dx = fabs(dx) <= std::numeric_limits<ito::float32>::epsilon() ? std::numeric_limits<ito::float32>::epsilon() : dx;
                            dy = fabs(dy) <= std::numeric_limits<ito::float32>::epsilon() ? -std::numeric_limits<ito::float32>::epsilon() : dy;

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
                        path.addEllipse(it.value()->x1,
                            it.value()->y1,
                            canxpos - it.value()->x1,
                            canypos - it.value()->y1),
                            it.value()->setShape(path, m_inverseColor0, m_inverseColor1);
                        it.value()->setActive(it.value()->m_active);
                        //if (p) emit p->plotItemChanged(n);
                        if (m_ignoreNextMouseEvent)
                        {
                            ito::float32 destPosX = transform(QwtPlot::xBottom, canxpos);
                            ito::float32 destPosY = transform(QwtPlot::yLeft, canypos);

                            QPoint dst = canvas()->mapToGlobal(QPoint(destPosX, destPosY));

                            this->cursor().setPos(dst);
                        }
                        replot();
                        break;
                    }
                }
            }
            break;
            }
            break;
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
        for (; it != m_pShapes.end(); ++it)
        {
            if (it.value() != NULL && it.value()->m_active != 0)
            {
                int type = 0;
                QVector<ito::float32> values;
                values.reserve(11);
                switch (it.value()->m_type)
                {
                case ito::PrimitiveContainer::tPoint:
                    type = ito::PrimitiveContainer::tPoint;
                    values.append(it.value()->x1);
                    values.append(it.value()->y1);
                    values.append(0.0);
                    break;

                case ito::PrimitiveContainer::tLine:
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
                case ito::PrimitiveContainer::tRectangle:
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
                case ito::PrimitiveContainer::tEllipse:
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

                ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(parent());
                if (p)
                {
                    emit p->plotItemChanged(it.value()->m_idx, type, values);
                }
            }
            if (it.value())
            {
                it.value()->m_active = 0;
                it.value()->setActive(0);
            }
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

    switch (m_currentShapeType)
    {
    case ito::PrimitiveContainer::tMultiPointPick:
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
                for (int i = 0; i < polygonScale.size(); i++)
                {
                    QPainterPath path;
                    DrawItem *newItem = NULL;
                    newItem = new DrawItem(this, ito::PrimitiveContainer::tPoint);
                    path.moveTo(polygonScale[i].x(), polygonScale[i].y());
                    path.lineTo(polygonScale[i].x(), polygonScale[i].y());

                    newItem->setShape(path, m_inverseColor0, m_inverseColor1);

                    if (this->m_inverseColor0.isValid())
                    {
                        newItem->setPen(QPen(m_inverseColor0));
                    }
                    else newItem->setPen(QPen(Qt::green));

                    newItem->setVisible(true);
                    newItem->show();
                    newItem->attach(this);
                    newItem->setSelected(true);
                    m_pShapes.insert(newItem->m_idx, newItem);
                }
                replot();
            }

            if (p)
            {

                emit p->userInteractionDone(ito::PrimitiveContainer::tMultiPointPick, aborted, polygonScale);
                emit p->plotItemsFinished(ito::PrimitiveContainer::tMultiPointPick, aborted);
            }

            m_pMultiPointPicker->setEnabled(false);
            setState(stateIdle);
        }
        break;

    case ito::PrimitiveContainer::tPoint:
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

                QPainterPath path;
                DrawItem *newItem = NULL;
                newItem = new DrawItem(this, ito::PrimitiveContainer::tPoint);
                path.moveTo(polygonScale[0].x(), polygonScale[0].y());
                path.lineTo(polygonScale[0].x(), polygonScale[0].y());

                newItem->setShape(path, m_inverseColor0, m_inverseColor1);

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_pShapes.insert(newItem->m_idx, newItem);
                m_currentShapeIndices.append(newItem->m_idx);
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
                        destPolygon.append(QPointF(m_currentShapeIndices[i], ito::PrimitiveContainer::tPoint));
                        destPolygon.append(QPointF(m_pShapes[m_currentShapeIndices[i]]->x1, m_pShapes[m_currentShapeIndices[i]]->y1));
                        destPolygon.append(QPointF(m_pShapes[m_currentShapeIndices[i]]->x2, m_pShapes[m_currentShapeIndices[i]]->y2));
                        destPolygon.append(QPointF(0.0, 0.0));
                    }
                    m_currentShapeIndices.clear();
                    emit p->userInteractionDone(ito::PrimitiveContainer::tPoint, aborted, destPolygon);
                    emit p->plotItemsFinished(ito::PrimitiveContainer::tPoint, aborted);
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        }
        break;

    case ito::PrimitiveContainer::tLine:
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

                QPainterPath path;
                DrawItem *newItem = NULL;
                newItem = new DrawItem(this, ito::PrimitiveContainer::tLine);
                path.moveTo(polygonScale[0].x(), polygonScale[0].y());
                path.lineTo(polygonScale[1].x(), polygonScale[1].y());

                newItem->setShape(path, m_inverseColor0, m_inverseColor1);

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_pShapes.insert(newItem->m_idx, newItem);
                m_currentShapeIndices.append(newItem->m_idx);
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
                        destPolygon.append(QPointF(m_currentShapeIndices[i], ito::PrimitiveContainer::tLine));
                        destPolygon.append(QPointF(m_pShapes[m_currentShapeIndices[i]]->x1, m_pShapes[m_currentShapeIndices[i]]->y1));
                        destPolygon.append(QPointF(m_pShapes[m_currentShapeIndices[i]]->x2, m_pShapes[m_currentShapeIndices[i]]->y2));
                        destPolygon.append(QPointF(0.0, 0.0));
                    }
                    m_currentShapeIndices.clear();
                    emit p->userInteractionDone(ito::PrimitiveContainer::tLine, aborted, destPolygon);
                    emit p->plotItemsFinished(ito::PrimitiveContainer::tLine, aborted);


                }
                
                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        }
        break;

    case ito::PrimitiveContainer::tRectangle:
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

                QPainterPath path;
                DrawItem *newItem = NULL;
                newItem = new DrawItem(this, ito::PrimitiveContainer::tRectangle);
                path.addRect(polygonScale[0].x(), polygonScale[0].y(), polygonScale[1].x() - polygonScale[0].x(), polygonScale[1].y() - polygonScale[0].y());
                newItem->setShape(path, m_inverseColor0, m_inverseColor1);
                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_pShapes.insert(newItem->m_idx, newItem);
                m_currentShapeIndices.append(newItem->m_idx);
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
                    QPolygonF destPolygon;
                    destPolygon.reserve(4);
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        destPolygon.append(QPointF(m_currentShapeIndices[i], ito::PrimitiveContainer::tRectangle));
                        destPolygon.append(QPointF(m_pShapes[m_currentShapeIndices[i]]->x1, m_pShapes[m_currentShapeIndices[i]]->y1));
                        destPolygon.append(QPointF(m_pShapes[m_currentShapeIndices[i]]->x2, m_pShapes[m_currentShapeIndices[i]]->y2));
                        destPolygon.append(QPointF(0.0, 0.0));
                    }
                    m_currentShapeIndices.clear();

                    emit p->userInteractionDone(ito::PrimitiveContainer::tRectangle, aborted, destPolygon);
                    emit p->plotItemsFinished(ito::PrimitiveContainer::tRectangle, aborted);
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        }
        break;

    case ito::PrimitiveContainer::tEllipse:
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

                QPainterPath path;
                DrawItem *newItem = NULL;
                newItem = new DrawItem(this, ito::PrimitiveContainer::tEllipse);
                path.addEllipse(polygonScale[0].x(), polygonScale[0].y(),
                    (polygonScale[1].x() - polygonScale[0].x()), (polygonScale[1].y() - polygonScale[0].y()));

                newItem->setShape(path, m_inverseColor0, m_inverseColor1);

                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else newItem->setPen(QPen(Qt::green));

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_pShapes.insert(newItem->m_idx, newItem);
                m_currentShapeIndices.append(newItem->m_idx);
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
                    QPolygonF destPolygon;
                    destPolygon.reserve(4);
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        destPolygon.append(QPointF(m_currentShapeIndices[i], ito::PrimitiveContainer::tEllipse));
                        destPolygon.append(QPointF(m_pShapes[m_currentShapeIndices[i]]->x1, m_pShapes[m_currentShapeIndices[i]]->y1));
                        destPolygon.append(QPointF(m_pShapes[m_currentShapeIndices[i]]->x2, m_pShapes[m_currentShapeIndices[i]]->y2));
                        destPolygon.append(QPointF(0.0, 0.0));
                    }
                    m_currentShapeIndices.clear();

                    emit p->userInteractionDone(ito::PrimitiveContainer::tEllipse, aborted, destPolygon);
                    emit p->plotItemsFinished(ito::PrimitiveContainer::tEllipse, aborted);
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
        if (type == ito::PrimitiveContainer::tMultiPointPick || \
            type == ito::PrimitiveContainer::tPoint || \
            type == ito::PrimitiveContainer::tLine || \
            type == ito::PrimitiveContainer::tRectangle || \
            type == ito::PrimitiveContainer::tEllipse)
        {
            m_currentShapeType = (ito::PrimitiveContainer::tPrimitive)type;
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

    if (m_currentShapeType == ito::PrimitiveContainer::tMultiPointPick) //multiPointPick
    {
        if (start)
        {
            m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::CrossRubberBand);
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
                QPolygonF polygonScale;
                emit p->userInteractionDone(m_currentShapeType, true, polygonScale);
            }
            setState(stateIdle);
        }
    }
    else if (m_currentShapeType == ito::PrimitiveContainer::tPoint)
    {
        if (start)
        {
            m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::CrossRubberBand);
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
                QPolygonF polygonScale;
                emit p->userInteractionDone(m_currentShapeType, true, polygonScale);
            }
            setState(stateIdle);
        }
    }
    else if (m_currentShapeType == ito::PrimitiveContainer::tLine)
    {
        if (start)
        {
            m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::PolygonRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());

            if (m)
            {
                m->setMaxNrItems(2);
                m_pMultiPointPicker->setEnabled(true);

                if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 lines. Esc aborts the selection.").arg(m_elementsToPick));
                else emit statusBarMessage(tr("Please draw one line. Esc aborts the selection."));
            }
        }
        else //start == false
        {
            m_pMultiPointPicker->setEnabled(false);
            emit statusBarMessage(tr("Selection has been interrupted."), 2000);

            m_elementsToPick = 1;

            if (p)
            {
                QPolygonF polygonScale;
                emit p->userInteractionDone(m_currentShapeType, true, polygonScale);
            }
            setState(stateIdle);
        }
    }
    else if (m_currentShapeType == ito::PrimitiveContainer::tRectangle)
    {
        if (start)
        {
            m_pMultiPointPicker->setStateMachine(new QwtPickerClickRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::RectRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
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
                QPolygonF polygonScale;
                emit p->userInteractionDone(m_currentShapeType, true, polygonScale);
            }
            setState(stateIdle);
        }
    }
    else if (m_currentShapeType == ito::PrimitiveContainer::tEllipse)
    {
        if (start)
        {

            m_pMultiPointPicker->setStateMachine(new QwtPickerClickRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::EllipseRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
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
                QPolygonF polygonScale;
                emit p->userInteractionDone(m_currentShapeType, true, polygonScale);
            }
            setState(stateIdle);
        }
    }
    else
    {
        m_pMultiPointPicker->setEnabled(false);
        
        if (p)
        {
            QPolygonF polygonScale;
            emit p->userInteractionDone(m_currentShapeType, true, polygonScale);
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

    //delete all geometric shapes and marker sets
    QHashIterator<int, DrawItem *> i(m_pShapes);
    DrawItem *shape;
    while (i.hasNext())
    {
        i.next();
        shape = i.value();
        shape->detach();
        delete shape;
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
        emit p->plotItemsDeleted();
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
        if (p) emit  p->plotItemDeleted(id);
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
int ItomQwtPlot::getSelectedGeometricShapeIdx() const
{
    QHash<int, DrawItem*>::const_iterator it;
    for (it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
    {
        if (it.value() != NULL && it.value()->selected() != 0)
        {
            return it.value()->m_idx;
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
        if (it.value() != NULL && it.value()->m_idx == idx)
        {
            it.value()->setSelected(true);
            failed = false;
            do_replot = true;
            continue;
        }
        if (it.value() != NULL && (it.value()->m_active != 0 || it.value()->selected()))
        {
            do_replot = true;
            it.value()->m_active = 0;
            it.value()->setActive(0);
            it.value()->setSelected(false);
        }
    }

    if (do_replot) replot();
    if (failed) emit statusBarMessage(tr("Could not set active element, index out of range."), 12000);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::setGeometricShapes(const QSharedPointer<ito::DataObject> geometricShapes)
{
    ito::RetVal retVal;
    clearAllGeometricShapes();
    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());


    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Could not plot marker, api is missing").toLatin1().data());
    }
    else if (geometricShapes.isNull())
    {
        retVal += ito::RetVal(ito::retError, 0, tr("no geometric shapes given").toLatin1().data());
    }
    else
    {
        int limits[] = { 0, std::numeric_limits<int>::max(), 6, 11 };

        ito::DataObject *dObj = apiCreateFromDataObject(geometricShapes.data(), 2, ito::tFloat32, limits, &retVal);

        if (!retVal.containsError())
        {
            int nrOfShapes = dObj->getSize(0);
            
            DrawItem *newItem = NULL;
            unsigned short type;
            unsigned char flags;
            // The definition do not correspond to the definetion of primitiv elements

            for (int i = 0; i < nrOfShapes; ++i)
            {
                QPainterPath path; //this must be inside of for loop, else it is not cleared for every new shape
                newItem = NULL;
                const ito::float32 *row = dObj->rowPtr<const ito::float32>(0, i);
                int idx = (int)row[0];
                int t = (int)row[1];

                type = t & ito::PrimitiveContainer::tTypeMask;
                flags = (t & ito::PrimitiveContainer::tFlagMask) >> 16;

                switch (type)
                {
                case ito::PrimitiveContainer::tPoint:
                    path.moveTo(row[2], row[3]);
                    path.lineTo(row[2], row[3]);
                    break;

                case ito::PrimitiveContainer::tLine:
                    path.moveTo(row[2], row[3]);
                    path.lineTo(row[4], row[5]);
                    break;

                case ito::PrimitiveContainer::tRectangle:
                    path.addRect(row[2], row[3], row[4] - row[2], row[5] - row[3]);
                    break;

                case ito::PrimitiveContainer::tEllipse:
                    path.addEllipse(row[2], row[3], row[4] - row[2], row[5] - row[3]);
                    break;

                default:
                    retVal += ito::RetVal(ito::retError, 0, tr("invalid geometric shape type").toLatin1().data());
                    break;
                }

                if (m_pShapes.contains(idx))
                {
                    m_pShapes[idx]->setShape(path, m_inverseColor0, m_inverseColor1);
                    m_pShapes[idx]->m_flags = flags;
                }
                else
                {
                    switch (type)
                    {
                    case ito::PrimitiveContainer::tMultiPointPick:
                    case ito::PrimitiveContainer::tPoint:
                        newItem = new DrawItem(this, ito::PrimitiveContainer::tPoint, idx);
                        break;

                    case ito::PrimitiveContainer::tLine:
                        newItem = new DrawItem(this, ito::PrimitiveContainer::tLine, idx);
                        break;

                    case ito::PrimitiveContainer::tRectangle:
                        newItem = new DrawItem(this, ito::PrimitiveContainer::tRectangle, idx);
                        break;

                    case ito::PrimitiveContainer::tEllipse:
                        newItem = new DrawItem(this, ito::PrimitiveContainer::tEllipse, idx);
                        break;

                    default:
                        retVal += ito::RetVal(ito::retError, 0, tr("invalid marker type").toLatin1().data());
                        break;
                    }

                    if (newItem)
                    {
                        newItem->setShape(path, m_inverseColor0, m_inverseColor1);

                        newItem->setVisible(true);
                        newItem->show();
                        newItem->attach(this);
                        newItem->m_flags = flags;
                        m_pShapes.insert(newItem->m_idx, newItem);
                    }
                }
            }

            replot();
        }

        if (dObj) delete dObj;
    }

    m_pActClearShapes->setEnabled(m_plottingEnabled && countGeometricShapes() > 0);

    if (retVal.hasErrorMessage())
    {
        emit statusBarMessage(retVal.errorMessage(), 12000);
        
    }
    
    if (p) emit  p->plotItemsFinished(0, retVal.containsError());

    return retVal;
}


//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> ItomQwtPlot::geometricShapes2DataObject()
{
    QSharedPointer<ito::DataObject> obj(new ito::DataObject());
    obj->zeros(m_pShapes.size(), PRIM_ELEMENTLENGTH, ito::tFloat32);

    QHash<int, DrawItem*>::Iterator it = m_pShapes.begin();
    ito::float32 *rowPtr;

    int idx = 0;
    for (; it != m_pShapes.end(); it++)
    {
        if (it.value() == NULL)
        {
            continue;
        }

        rowPtr = obj->rowPtr<ito::float32>(0, idx);
        rowPtr[0] = (ito::float32) (it.value()->m_idx);

        switch (it.value()->m_type)
        {
        case ito::PrimitiveContainer::tPoint:
            rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tPoint;
            rowPtr[2] = (ito::float32) (it.value()->x1);
            rowPtr[3] = (ito::float32) (it.value()->y1);
            break;

        case ito::PrimitiveContainer::tLine:
            rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tLine;
            rowPtr[2] = (ito::float32) (it.value()->x1);
            rowPtr[3] = (ito::float32) (it.value()->y1);
            rowPtr[5] = (ito::float32) (it.value()->x2);
            rowPtr[6] = (ito::float32) (it.value()->y2);
            break;

        case ito::PrimitiveContainer::tRectangle:
            rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tRectangle;
            rowPtr[2] = (ito::float32) (it.value()->x1);
            rowPtr[3] = (ito::float32) (it.value()->y1);
            rowPtr[5] = (ito::float32) (it.value()->x2);
            rowPtr[6] = (ito::float32) (it.value()->y2);
            break;

        case ito::PrimitiveContainer::tEllipse:
            rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tEllipse;
            rowPtr[2] = (((ito::float32)it.value()->x1 + (ito::float32)it.value()->x2) / 2.0);
            rowPtr[3] = (((ito::float32)it.value()->y1 + (ito::float32)it.value()->y2) / 2.0);
            rowPtr[5] = (abs((ito::float32)it.value()->x1 - (ito::float32)it.value()->x2) / 2.0);
            rowPtr[6] = (abs((ito::float32)it.value()->y1 - (ito::float32)it.value()->y2) / 2.0);
            break;
            /*
            case Plot1DWidget::tCircle:
            rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tCircle;
            rowPtr[2] = (((ito::float32)it.value()->x1 + (ito::float32)it.value()->x2) / 2.0);
            rowPtr[3] = (((ito::float32)it.value()->y1 + (ito::float32)it.value()->y2) / 2.0);
            rowPtr[5] = (abs((ito::float32)it.value()->x1 - (ito::float32)it.value()->x2) / 4.0) + (abs((ito::float32)it.value()->y1 - (ito::float32)it.value()->y2) / 4.0);
            break;

            case Plot1DWidget::tSquare:
            rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tSquare;
            rowPtr[2] = (((ito::float32)it.value()->x1 + (ito::float32)it.value()->x2) / 2.0);
            rowPtr[3] = (((ito::float32)it.value()->y1 + (ito::float32)it.value()->y2) / 2.0);
            rowPtr[5] = (abs((ito::float32)it.value()->x1 - (ito::float32)it.value()->x2) / 4.0) + (abs((ito::float32)it.value()->y1 - (ito::float32)it.value()->y2) / 4.0);
            break;
            */
        }
        idx++;
    }

    return obj;
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

    QBrush curBrush = canvasBackground();
    QPalette curPalette = palette();

    setAutoFillBackground(true);
    setPalette(Qt::white);
    setCanvasBackground(Qt::white);

    replot();

    QwtPlotRenderer renderer;

    // flags to make the document look like the widget
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    //renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true); //deprecated in qwt 6.1.0

    if (copyToClipboardNotFile)
    {
        emit statusBarMessage(tr("copy current view to clipboard..."));

        qreal resFaktor = std::max(qRound(resolution / 72.0), 1);

        QSize myRect(curSize.width() * resFaktor, curSize.height() * resFaktor);
        QClipboard *clipboard = QApplication::clipboard();
        QImage img(myRect, QImage::Format_ARGB32);
        QPainter painter(&img);
        painter.scale(resFaktor, resFaktor);
        renderer.render(this, &painter, rect());
        img.setDotsPerMeterX(img.dotsPerMeterX() * resFaktor); //setDotsPerMeterXY must be set after rendering
        img.setDotsPerMeterY(img.dotsPerMeterY() * resFaktor);
        clipboard->setImage(img);

        emit statusBarMessage(tr("copy current view to clipboard. done."), 1000);
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
    int limits[] = { 0, std::numeric_limits<int>::max(), 2, 2 };

    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        emit statusBarMessage(tr("Could not plot marker, api is missing"), 4000);
        return ito::RetVal(ito::retError, 0, tr("Could not plot marker, api is missing").toLatin1().data());
    }

    ito::DataObject *dObj = NULL;

    if (coordinates->getDims() == 2 && coordinates->getSize(0) == 2 && coordinates->getSize(1) != 2)
    {
        QString err = tr("Coordinates of markers must be a Nx2 data object where each line is the (x,y) coordinate of the marker. This behaviour changed since version 2.0 of itom1dqwtplot and itom2dqwtplot in order to unify all markers and geometric shapes.");
        retval += ito::RetVal(ito::retWarning, 0, err.toLatin1().data());
        ito::DataObject trans = coordinates.data()->trans();
        dObj = apiCreateFromDataObject(&trans, 2, ito::tFloat32, limits, &retval);
    }
    else
    {
        dObj = apiCreateFromDataObject(coordinates.data(), 2, ito::tFloat32, limits, &retval);
    }

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
        int nrOfMarkers = dObj->getSize(0);

        const cv::Mat *mat = dObj->getCvPlaneMat(0);

        const ito::float32 *row;

        for (int i = 0; i < nrOfMarkers; ++i)
        {
            row = mat->ptr<const ito::float32>(i);
            marker = new QwtPlotMarker();
            marker->setSymbol(new QwtSymbol(symStyle, symBrush, symPen, symSize));
            marker->setValue(row[0], row[1]);
            marker->attach(this);
            if (m_markerLabelVisible)
            {
                QwtText label(QString(" %1").arg(id == "" ? "unknown" : id));
                marker->setLabel(label);
            }

            m_plotMarkers.insert(id == "" ? "unknown" : id, QPair<int, QwtPlotMarker*>(plane, marker));
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
}
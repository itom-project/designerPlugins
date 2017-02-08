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
#include <qprintpreviewdialog.h>

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
#include <qwt_plot_layout.h>
#include <qwt_plot_canvas.h>

//---------------------------------------------------------------------------
ItomQwtPlot::ItomQwtPlot(ItomQwtDObjFigure * parent /*= NULL*/) :
    QwtPlot(parent),
    m_pContextMenu(NULL),
    m_showContextMenu(true),
    m_keepAspectRatio(false),
    m_firstTimeVisible(false),
    m_state(-1),
    m_isUserInteraction(false),
    m_stateIsChanging(false),
    m_selectedShape(NULL),
    m_ignoreNextMouseEvent(false),
    m_shapeModificationModes(ItomQwtPlotEnums::Move | ItomQwtPlotEnums::Rotate | ItomQwtPlotEnums::Resize),
    m_inverseColor0(Qt::green),
    m_inverseColor1(Qt::blue),
    m_elementsToPick(1),
    m_currentShapeType(ito::Shape::Point),
    m_allowedShapeTypes(~ItomQwtPlotEnums::ShapeTypes()),
    m_buttonStyle(0),
    m_boxFrame(true),
    m_plottingEnabled(true),
    m_markerLabelVisible(false),
    m_shapesLabelVisible(false),
    m_unitLabelStyle(ito::AbstractFigure::UnitLabelSlash),
    m_pActSave(NULL),
    m_pActPrint(NULL),
    m_pActHome(NULL),
    m_pActPan(NULL),
    m_pActZoom(NULL),
    m_pActAspectRatio(NULL),
    m_pActSendCurrentToWorkspace(NULL),
    m_pActCopyClipboard(NULL),
    m_pMenuShapeType(NULL),
    m_pActClearShapes(NULL),
    m_pActProperties(NULL),
    m_pActShapeType(NULL),
    m_currentPlane(0),
    m_axisColor(Qt::black),
    m_textColor(Qt::black),
    m_backgroundColor(Qt::white),
    m_canvasColor(Qt::white),
    m_styledBackground(false),
    m_pPrinter(NULL),
    m_shapeModifiedByMouseMove(false),
    m_geometricShapeOpacity(0),
    m_geometricShapeOpacitySelected(0)
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

    updateColors();

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
    m_pMagnifier->setAxesDisabledOnAdditionalModifier(QList<int>() << QwtPlot::yLeft, Qt::AltModifier);
    m_pMagnifier->setAxesDisabledOnAdditionalModifier(QList<int>() << QwtPlot::xBottom, Qt::ShiftModifier);
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
    QwtPlotCanvas* c = dynamic_cast<QwtPlotCanvas*>(canvas());
    c->setFrameStyle(QFrame::Box);
    c->setLineWidth(1);
    c->setMidLineWidth(0);
    //canvas()->setStyleSheet("border: 0px;");
    //plotLayout()->setAlignCanvasToScales(true); //directly connects the bottom-left-corners of the y-left and x-bottom axis.
    plotLayout()->setCanvasMargin(2,-1);

    //left axis
    QwtScaleWidget *leftAxis = axisWidget(QwtPlot::yLeft);
    leftAxis->setMargin(0);                 //distance backbone <-> canvas
    leftAxis->setSpacing(6);                //distance tick labels <-> axis label
    leftAxis->scaleDraw()->setSpacing(4);   //distance tick labels <-> ticks
    leftAxis->setContentsMargins(0, 0, 0, 0);  //left axis starts and ends at same level than canvas
    axisScaleDraw(QwtPlot::yLeft)->enableComponent(QwtScaleDraw::Backbone, !m_boxFrame);

    //bottom axis
    QwtScaleWidget *bottomAxis = axisWidget(QwtPlot::xBottom);
    bottomAxis->setMargin(0);                 //distance backbone <-> canvas
    bottomAxis->setSpacing(6);                //distance tick labels <-> axis label
    bottomAxis->scaleDraw()->setSpacing(4);   //distance tick labels <-> ticks
    bottomAxis->setContentsMargins(0, 0, 0, 0);  //left axis starts and ends at same level than canvas
    axisScaleDraw(QwtPlot::xBottom)->enableComponent(QwtScaleDraw::Backbone, !m_boxFrame);

    ////top axis
    //QwtScaleWidget *topAxis = axisWidget(QwtPlot::xTop);
    //enableAxis(QwtPlot::xTop, true);
    //topAxis->setMargin(0);                 //distance backbone <-> canvas
    //topAxis->setSpacing(6);                //distance tick labels <-> axis label
    //topAxis->scaleDraw()->setSpacing(4);   //distance tick labels <-> ticks
    //topAxis->setContentsMargins(0, 0, 0, 0);  //left axis starts and ends at same level than canvas

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

    if (m_pPrinter)
    {
        delete m_pPrinter;
        m_pPrinter = NULL;
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
    
    //m_pActPrint
    m_pActPrint = a = new QAction(tr("Print..."), p);
    a->setShortcut(QKeySequence::Print);
    a->setObjectName("actPrint");
    a->setToolTip(tr("Print preview..."));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActPrint()));

    //m_actCopyClipboard
    m_pActCopyClipboard = a = new QAction(tr("Copy To Clipboard"), p);
    a->setShortcut(QKeySequence::Copy);
    a->setObjectName("actCopyClipboard");
    a->setToolTip(tr("Copies the current view to the clipboard"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuCopyToClipboard()));

    //m_pActSendCurrentToWorkspace
    m_pActSendCurrentToWorkspace = a = new QAction(tr("Send Current View To Workspace..."), p);
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
    m_pActZoom = a = new QAction(tr("Zoom To Rectangle"), p);
    a->setObjectName("actZoom");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Zoom to rectangle"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActZoom(bool)));

    //m_pActClearDrawings
    m_pActClearShapes = a = new QAction(tr("Clear Geometric Shapes"), p);
    a->setObjectName("actClearGeometrics");
    a->setCheckable(false);
    a->setChecked(false);
    a->setToolTip(tr("Clear All Existing Geometric Shapes"));
    connect(a, SIGNAL(triggered()), this, SLOT(clearAllGeometricShapes()));

    //m_actApectRatio
    m_pActAspectRatio = a = new QAction(tr("Lock Aspect Ratio"), p);
    a->setObjectName("actRatio");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Toggle fixed / variable aspect ration between axis x and y"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActRatio(bool)));

    //m_pActShapeType
    m_pActShapeType = new QAction(tr("Draw Geometric Shape"), p);
    m_pMenuShapeType = new QMenu(tr("Draw Geometric Shape"), p);
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
    QBrush trackerBg = QBrush(Qt::white, Qt::SolidPattern);

	QPen shapeRubberBandPen = QPen(QBrush(Qt::red), 2, Qt::DashLine);
	QPen shapePen = QPen(QBrush(Qt::red), 2);
	QFont shapeLabelFont = QFont("Verdana", 10);
	QBrush shapeLabelBg = QBrush(QColor(255, 255, 255, 155), Qt::SolidPattern);

    if (ito::ITOM_API_FUNCS_GRAPH)
    {
        rubberBandPen = apiGetFigureSetting(parent(), "zoomRubberBandPen", rubberBandPen, NULL).value<QPen>();
        trackerPen = apiGetFigureSetting(parent(), "trackerPen", trackerPen, NULL).value<QPen>();
		trackerBg = apiGetFigureSetting(parent(), "trackerBackground", trackerBg, NULL).value<QBrush>();
        trackerFont = apiGetFigureSetting(parent(), "trackerFont", trackerFont, NULL).value<QFont>();

		shapeRubberBandPen = apiGetFigureSetting(parent(), "shapeRubberBandPen", shapeRubberBandPen, NULL).value<QPen>();
		shapePen = apiGetFigureSetting(parent(), "shapePen", shapePen, NULL).value<QPen>();
		shapeLabelBg = apiGetFigureSetting(parent(), "shapeLabelTrackerBackground", shapeLabelBg, NULL).value<QBrush>();
		shapeLabelFont = apiGetFigureSetting(parent(), "shapeLabelTrackerFont", shapeLabelFont, NULL).value<QFont>();

        m_unitLabelStyle = (ito::AbstractFigure::UnitLabelStyle)(apiGetFigureSetting(parent(), "unitLabelStyle", m_unitLabelStyle, NULL).value<int>());
        bool enableBoxFrame = (ito::AbstractFigure::UnitLabelStyle)(apiGetFigureSetting(parent(), "enableBoxFrame", m_boxFrame, NULL).value<bool>());
        setBoxFrame(enableBoxFrame);
    }

    rubberBandPen.setColor(m_inverseColor0);
    trackerPen.setColor(m_inverseColor0);

    m_pZoomer->setRubberBandPen(rubberBandPen);
    m_pZoomer->setTrackerFont(trackerFont);
    m_pZoomer->setTrackerPen(trackerPen);

	m_pMultiPointPicker->setTrackerFont(shapeLabelFont);
	m_pMultiPointPicker->setTrackerPen(shapePen);
	m_pMultiPointPicker->setBackgroundFillBrush(shapeLabelBg);
	m_pMultiPointPicker->setRubberBandPen(shapeRubberBandPen);

    foreach(DrawItem *item, m_pShapes)
    {
        if (item && item->getAutoColor())
        {
            item->setColor(m_inverseColor0, m_inverseColor1);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::updateColors(void)
{
    QwtPlotCanvas* c = dynamic_cast<QwtPlotCanvas*>(canvas());

    //at first, it was possible to let the windows be styled by os-dependent tools. However, the QFrame::Box 
    //created undesired corner forms when printing the canvas. Therefore, stylesheets are always used.

    /*if (testAttribute(Qt::WA_StyledBackground))
    {*/
        m_styledBackground = true;

        //we have to apply all styles using style sheets
        QString styleSheet = QString("background-color: %1;").arg(m_backgroundColor.name());
        styleSheet.append(QString("color: %1;").arg(m_axisColor.name()));
        setStyleSheet(styleSheet);

        if (m_boxFrame)
        {
            c->setStyleSheet(QString("border: 1px solid %1; background-color: %2;").arg(m_axisColor.name()).arg(m_canvasColor.name()));
        }
        else
        {
            c->setStyleSheet(QString("background-color: %2;").arg(m_canvasColor.name()));
        }
    //}
    //else
    //{
    //    m_styledBackground = false;

    //    //no style sheets are applied, therefore the default OS dependet style methods can be used
    //    c->setFrameStyle(QFrame::Box);
    //    c->setLineWidth(1);
    //    c->setMidLineWidth(0);
    //}

    QPalette newPalette(m_backgroundColor);

    newPalette.setColor(QPalette::WindowText, m_axisColor); // for ticks
    newPalette.setColor(QPalette::Text, m_textColor); // for ticks' labels

    setAutoFillBackground(true);
    setPalette(newPalette);
    setCanvasBackground(m_canvasColor);


    axisWidget(QwtPlot::xBottom)->setAutoFillBackground(true);
    axisWidget(QwtPlot::xBottom)->setPalette(newPalette);

    axisWidget(QwtPlot::yLeft)->setAutoFillBackground(true);
    axisWidget(QwtPlot::yLeft)->setPalette(newPalette);

    axisWidget(QwtPlot::yRight)->setAutoFillBackground(true);
    axisWidget(QwtPlot::yRight)->setPalette(newPalette);

    axisWidget(QwtPlot::xTop)->setAutoFillBackground(true);
    axisWidget(QwtPlot::xTop)->setPalette(newPalette);

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setBoxFrame(bool boxFrame)
{
    if (boxFrame != m_boxFrame)
    {
        axisScaleDraw(QwtPlot::yLeft)->enableComponent(QwtScaleDraw::Backbone, !boxFrame);
        axisScaleDraw(QwtPlot::xBottom)->enableComponent(QwtScaleDraw::Backbone, !boxFrame);
        m_boxFrame = boxFrame;
        updateColors();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setBackgroundColor(const QColor &color)
{
    m_backgroundColor = color;
    updateColors();
}


//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setAxisColor(const QColor &color)
{
    m_axisColor = color;
    updateColors();
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setTextColor(const QColor &color)
{
    m_textColor = color;
    updateColors();
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setCanvasColor(const QColor &color)
{
    m_canvasColor = color;
    updateColors();
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setButtonStyle(int style)
{
    if (style == 0)
    {
        m_pActSave->setIcon(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"));
        m_pActPrint->setIcon(QIcon(":/itomDesignerPlugins/general/icons/print.png"));
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
        m_pActPrint->setIcon(QIcon(":/itomDesignerPlugins/general_lt/icons/print_lt.png"));
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
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/point_lt.png"));
            break;
        case ito::Shape::Line:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/pntline_lt.png"));
            break;
        case ito::Shape::Rectangle:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/rectangle_lt.png"));
            break;
        case ito::Shape::Ellipse:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/ellipse_lt.png"));
            break;
        case ito::Shape::Circle:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/circle_lt.png"));
            break;
        case ito::Shape::Square:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/square_lt.png"));
            break;
        }
    }

    m_buttonStyle = style;
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
void ItomQwtPlot::setShapeModificationModes(const ItomQwtPlotEnums::ModificationModes &modes)
{
    m_shapeModificationModes = modes;

    foreach(DrawItem *d, this->m_pShapes)
    {
        d->setModificationModes(modes);
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setAllowedGeometricShapes(const ItomQwtPlotEnums::ShapeTypes &allowedTypes)
{
    m_allowedShapeTypes = allowedTypes;
    ito::Shape::ShapeType potentialCurrentShape = ito::Shape::Invalid;

    foreach(QAction *a, m_pMenuShapeType->actions())
    {
        if (m_allowedShapeTypes.testFlag((ItomQwtPlotEnums::ShapeType)(a->data().toInt())))
        {
            a->setEnabled(true);
            potentialCurrentShape = (ito::Shape::ShapeType)(a->data().toInt());
        }
        else
        {
            a->setEnabled(false);
        }
    }

    if (m_allowedShapeTypes.testFlag((ItomQwtPlotEnums::ShapeType)m_currentShapeType) == false)
    {
        if (potentialCurrentShape != ito::Shape::Invalid)
        {
            if (m_state == stateDrawShape) //if an interaction is already running and the user desires another one, goto idle first.
            {
                setState(stateIdle);
            }

            m_currentShapeType = potentialCurrentShape;
            m_pActShapeType->setData(potentialCurrentShape);
            setButtonStyle(m_buttonStyle); //to set the current icon
        }
    }

    m_pMenuShapeType->setEnabled(m_plottingEnabled && (m_allowedShapeTypes ^ ItomQwtPlotEnums::MultiPointPick) != 0);
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

    if (m_allowedShapeTypes.testFlag((ItomQwtPlotEnums::ShapeType)m_currentShapeType))
    {
        if (m_state == stateDrawShape) //if an interaction is already running and the user desires another one, goto idle first.
        {
            setState(stateIdle);
        }
        setState(stateDrawShape);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuShapeType(bool checked)
{
    if (checked && m_allowedShapeTypes.testFlag((ItomQwtPlotEnums::ShapeType)m_currentShapeType))
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
        m_pMenuShapeType->setEnabled(m_plottingEnabled && (m_allowedShapeTypes ^ ItomQwtPlotEnums::MultiPointPick) != 0);
        m_pActClearShapes->setEnabled(m_plottingEnabled && countGeometricShapes());
        m_pActShapeType->setVisible(m_plottingEnabled);
        m_pActClearShapes->setVisible(m_plottingEnabled);

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
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/point.png" : ":/itomDesignerPlugins/plot_lt/icons/point_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(true);
                    break;

                case ito::Shape::Line:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/pntline.png" : ":/itomDesignerPlugins/plot_lt/icons/pntline_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(true);
                    break;

                case ito::Shape::Rectangle:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/rectangle.png" : ":/itomDesignerPlugins/plot_lt/icons/rectangle_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(true);
                    break;

                case ito::Shape::Ellipse:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/ellipse.png" : ":/itomDesignerPlugins/plot_lt/icons/ellipse_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(true);
                    break;

                case ito::Shape::Circle:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/circle.png" : ":/itomDesignerPlugins/plot_lt/icons/circle_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(true);
                    break;

                case ito::Shape::Square:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/square.png" : ":/itomDesignerPlugins/plot_lt/icons/square_lt.png"));
                    m_elementsToPick = std::max(m_elementsToPick, 1);
                    startOrStopDrawGeometricShape(true);
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
bool ItomQwtPlot::event(QEvent * event)
{
    if (event->type() == QEvent::StyleChange)
    {
        if (testAttribute(Qt::WA_StyledBackground) != m_styledBackground)
        {
            QTimer::singleShot(0, this, SLOT(updateColors()));
        }
    }

    return QwtPlot::event(event);
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
    else if (m_plottingEnabled && event->matches(QKeySequence::Delete))
    {
        //delete the currently selected geometric shape (but only, if the property 'geometricShapesDrawingEnabled' is true)
        QVector<int> indices_to_delete;

        for (QMap<int, DrawItem*>::iterator it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
        {
            if (it.value() != NULL && it.value()->getSelected())
            {
                indices_to_delete << it.value()->getIndex();
            }
        }

        foreach(int index, indices_to_delete)
        {
            deleteGeometricShape(index);
        }
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
        ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(parent());
        char hitType = -1; //not hit
        bool currentShapeFound = false;

        //at first check if the currently selected item is hit: If so, take this; else take any other item
        QMap<int, DrawItem*>::iterator it = m_pShapes.begin();
        if (m_selectedShape)
        {
            for (it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
            {
                if (it.value() == m_selectedShape)
                {
                    hitType = it.value()->hitEdge(scalePos, tol_x, tol_y);
                    break;
                }
            }
        }

        if (hitType == -1) //current shape is not hit, reset it to begin()
        {
            it = m_pShapes.begin();
        }
        
        for (; it != m_pShapes.end(); ++it)
        {
            if (it.value() == NULL)
            {
                continue;
            }

            hitType = it.value()->hitEdge(scalePos, tol_x, tol_y);
            if (hitType >= 0)
            {
                it.value()->setSelected(true);

                if (m_selectedShape != it.value())
                {
                    m_selectedShape = it.value();
                    if (p)
                    {
                        emit p->geometricShapeCurrentChanged(m_selectedShape->getShape());
                    }
                }
                m_selectedShapeHitType = hitType;
                m_startMouseScale = scalePos;
                m_startMousePx = event->pos();
                m_mouseDragReplotCounter = 0;
                currentShapeFound = true;

                if (hitType == 0)
                {
                    m_startMouseScaleDiff = m_startMouseScale - m_selectedShape->getMarkerPosScale(0);
                }
                else
                {
                    m_startMouseScaleDiff = m_startMouseScale - m_selectedShape->getMarkerPosScale(hitType - 1);
                }
                ++it; //such that the following for loop does not affect the newly selected item.
                break;
            }
            else
            {
                it.value()->setSelected(false);
            }
        }

        if (!currentShapeFound)
        {
            if (m_selectedShape && p)
            {
                emit p->geometricShapeCurrentChanged(ito::Shape());
            }

            m_selectedShape = NULL;
            m_selectedShapeHitType = -1;
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

        m_shapeModifiedByMouseMove = true;
    }

    if (!event->isAccepted())
    {
        QwtPlot::mouseMoveEvent(event);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mouseReleaseEvent(QMouseEvent * event)
{
    if (m_state == stateIdle && m_shapeModifiedByMouseMove)
    {
        //modification of shape finished
        event->accept();
        QMap<int, DrawItem*>::iterator it = m_pShapes.begin();
        bool found = false;
        QVector<ito::Shape> shapes;

        for (; it != m_pShapes.end(); ++it)
        {
            if (it.value() != NULL && it.value()->getSelected())
            {
                found = true;
                shapes << it.value()->getShape();

                ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(parent());
                if (p)
                {
                    if (p->shapesWidget())
                    {
                        p->shapesWidget()->updateShape(shapes.last());
                    }
                    emit p->geometricShapeChanged(it.value()->getIndex(), shapes.last());
                    emit p->geometricShapeFinished(shapes, false);
                }
            }
        }

        if (found)
        {
            replot();
        }

        m_shapeModifiedByMouseMove = false;
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

    if (!on)
    {
        QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
        bool aborted = false;

        switch (m_currentShapeType)
        {
        case ito::Shape::MultiPointPick:
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
                if (m_isUserInteraction)
                {
                    emit p->userInteractionDone(ito::Shape::MultiPointPick, aborted, shapes);
                    m_isUserInteraction = false;
                }
                emit p->geometricShapeFinished(shapes, aborted);

                PlotInfoMarker *pim = ((ItomQwtDObjFigure*)parent())->markerWidget();
                if (pim)
                {
                    pim->updateMarkers(shapes);
                }
            }

            m_pMultiPointPicker->setEnabled(false);
            setState(stateIdle);
        break;

        case ito::Shape::Point:
            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);

                ito::Shape shape = ito::Shape::fromPoint(polygonScale[0]);
                DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
                newItem->setColor(m_inverseColor0, m_inverseColor1);
                newItem->setFillOpacity(m_geometricShapeOpacity, m_geometricShapeOpacitySelected);
                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else
                {
                    newItem->setPen(QPen(Qt::green));
                }

                /*unselect all existing shapes before adding the new one*/
                for (QMap<int, DrawItem*>::iterator it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
                {
                    if (it.value() == NULL)
                    {
                        continue;
                    }
                    it.value()->setSelected(false);
                }

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_selectedShape = newItem;
                m_selectedShapeHitType = 0;
                m_pShapes.insert(newItem->getIndex(), newItem);
                m_currentShapeIndices.append(newItem->getIndex());
                emit p->geometricShapeAdded(newItem->getIndex(), newItem->getShape());
                emit p->geometricShapeCurrentChanged(newItem->getShape());
                replot();
            }

            // if further elements are needed reset the plot engine and go ahead else finish editing
            if (!aborted && m_elementsToPick > 1)
            {
                m_elementsToPick--;
                MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());
                if (m)
                {
                    m->setMaxNrItems(1);
                    m_pMultiPointPicker->setEnabled(true);
                }

                if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 points. Esc aborts the selection.").arg(m_elementsToPick));
                else emit statusBarMessage(tr("Please draw one point. Esc aborts the selection."));
                return;
            }
            else
            {
                m_elementsToPick = 0;
                if (p)
                {
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
                    }
                    m_currentShapeIndices.clear();
                    if (m_isUserInteraction)
                    {
                        emit p->userInteractionDone(ito::Shape::Point, aborted, shapes);
                        m_isUserInteraction = false;
                    }
                    emit p->geometricShapeFinished(shapes, aborted);
                    if (p->shapesWidget())
                    {
                        p->shapesWidget()->updateShapes(shapes);
                    }
                    
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        break;

        case ito::Shape::Line:
            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);

                ito::Shape shape = ito::Shape::fromLine(polygonScale[0], polygonScale[1]);
                DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
                newItem->setColor(m_inverseColor0, m_inverseColor1);
                newItem->setFillOpacity(m_geometricShapeOpacity, m_geometricShapeOpacitySelected);
                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else
                {
                    newItem->setPen(QPen(Qt::green));
                }

                /*unselect all existing shapes before adding the new one*/
                for (QMap<int, DrawItem*>::iterator it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
                {
                    if (it.value() == NULL)
                    {
                        continue;
                    }
                    it.value()->setSelected(false);
                }

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_selectedShape = newItem;
                m_selectedShapeHitType = 0;
                m_pShapes.insert(newItem->getIndex(), newItem);
                m_currentShapeIndices.append(newItem->getIndex());
                emit p->geometricShapeAdded(newItem->getIndex(), newItem->getShape());
                emit p->geometricShapeCurrentChanged(newItem->getShape());
                replot();
            }

            // if further elements are needed reset the plot engine and go ahead else finish editing
            if (!aborted && m_elementsToPick > 1)
            {
                m_elementsToPick--;
                m_pMultiPointPicker->setEnabled(true);

                if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 lines. Esc aborts the selection.").arg(m_elementsToPick));
                else emit statusBarMessage(tr("Please draw one line. Esc aborts the selection."));
                return;
            }
            else
            {
                m_elementsToPick = 0;
                if (p)
                {
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
                    }
                    m_currentShapeIndices.clear();
                    if (m_isUserInteraction)
                    {
                        emit p->userInteractionDone(ito::Shape::Line, aborted, shapes);
                        m_isUserInteraction = false;
                    }
                    emit p->geometricShapeFinished(shapes, aborted);
                    if (p->shapesWidget())
                    {
                        p->shapesWidget()->updateShapes(shapes);
                    }
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        break;

        case ito::Shape::Rectangle:
            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);

                ito::Shape shape = ito::Shape::fromRectangle(QRectF(polygonScale[0], polygonScale[1]));
                DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
                newItem->setColor(m_inverseColor0, m_inverseColor1);
                newItem->setFillOpacity(m_geometricShapeOpacity, m_geometricShapeOpacitySelected);
                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else
                {
                    newItem->setPen(QPen(Qt::green));
                }

                /*unselect all existing shapes before adding the new one*/
                for (QMap<int, DrawItem*>::iterator it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
                {
                    if (it.value() == NULL)
                    {
                        continue;
                    }
                    it.value()->setSelected(false);
                }

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_selectedShape = newItem;
                m_selectedShapeHitType = 0;
                m_pShapes.insert(newItem->getIndex(), newItem);
                m_currentShapeIndices.append(newItem->getIndex());
                emit p->geometricShapeAdded(newItem->getIndex(), newItem->getShape());
                emit p->geometricShapeCurrentChanged(newItem->getShape());
                replot();
            }

            // if further elements are needed reset the plot engine and go ahead else finish editing
            if (!aborted && m_elementsToPick > 1)
            {
                m_elementsToPick--;
                m_pMultiPointPicker->setEnabled(true);
                    
                if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 rectangles. Esc aborts the selection.").arg(m_elementsToPick));
                else emit statusBarMessage(tr("Please draw one rectangle. Esc aborts the selection."));
                return;
            }
            else
            {
                m_elementsToPick = 0;
                if (p)
                {
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
                    }
                    m_currentShapeIndices.clear();
                    if (m_isUserInteraction)
                    {
                        emit p->userInteractionDone(ito::Shape::Rectangle, aborted, shapes);
                        m_isUserInteraction = false;
                    }
                    emit p->geometricShapeFinished(shapes, aborted);
                    if (p->shapesWidget())
                    {
                        p->shapesWidget()->updateShapes(shapes);
                    }
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        break;

        case ito::Shape::Square:
            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);

                ito::Shape shape = ito::Shape::fromSquare(0.5 * (polygonScale[1] + polygonScale[0]), std::abs((polygonScale[1] - polygonScale[0]).x()));
                DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
                newItem->setColor(m_inverseColor0, m_inverseColor1);
                newItem->setFillOpacity(m_geometricShapeOpacity, m_geometricShapeOpacitySelected);
                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else
                {
                    newItem->setPen(QPen(Qt::green));
                }

                /*unselect all existing shapes before adding the new one*/
                for (QMap<int, DrawItem*>::iterator it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
                {
                    if (it.value() == NULL)
                    {
                        continue;
                    }
                    it.value()->setSelected(false);
                }

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_selectedShape = newItem;
                m_selectedShapeHitType = 0;
                m_pShapes.insert(newItem->getIndex(), newItem);
                m_currentShapeIndices.append(newItem->getIndex());
                emit p->geometricShapeAdded(newItem->getIndex(), newItem->getShape());
                emit p->geometricShapeCurrentChanged(newItem->getShape());
                replot();
            }

            // if further elements are needed reset the plot engine and go ahead else finish editing
            if (!aborted && m_elementsToPick > 1)
            {
                m_elementsToPick--;
                m_pMultiPointPicker->setEnabled(true);

                if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 rectangles. Esc aborts the selection.").arg(m_elementsToPick));
                else emit statusBarMessage(tr("Please draw one rectangle. Esc aborts the selection."));
                return;
            }
            else
            {
                m_elementsToPick = 0;
                if (p)
                {
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
                    }
                    m_currentShapeIndices.clear();
                    if (m_isUserInteraction)
                    {
                        emit p->userInteractionDone(ito::Shape::Square, aborted, shapes);
                        m_isUserInteraction = false;
                    }
                    emit p->geometricShapeFinished(shapes, aborted);
                    if (p->shapesWidget())
                    {
                        p->shapesWidget()->updateShapes(shapes);
                    }
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        break;

        case ito::Shape::Ellipse:
            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);

                ito::Shape shape = ito::Shape::fromEllipse(QRectF(polygonScale[0], polygonScale[1]));
                DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
                newItem->setColor(m_inverseColor0, m_inverseColor1);
                newItem->setFillOpacity(m_geometricShapeOpacity, m_geometricShapeOpacitySelected);
                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else
                {
                    newItem->setPen(QPen(Qt::green));
                }

                /*unselect all existing shapes before adding the new one*/
                for (QMap<int, DrawItem*>::iterator it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
                {
                    if (it.value() == NULL)
                    {
                        continue;
                    }
                    it.value()->setSelected(false);
                }

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_selectedShape = newItem;
                m_selectedShapeHitType = 0;
                m_pShapes.insert(newItem->getIndex(), newItem);
                m_currentShapeIndices.append(newItem->getIndex());
                emit p->geometricShapeAdded(newItem->getIndex(), newItem->getShape());
                emit p->geometricShapeCurrentChanged(newItem->getShape());
                replot();
            }

            // if further elements are needed reset the plot engine and go ahead else finish editing
            if (!aborted && m_elementsToPick > 1)
            {
                m_elementsToPick--;
                m_pMultiPointPicker->setEnabled(true);

                if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 ellipses. Esc aborts the selection.").arg(m_elementsToPick));
                else emit statusBarMessage(tr("Please draw one ellipse. Esc aborts the selection."));
                return;
            }
            else
            {
                m_elementsToPick = 0;
                if (p)
                {
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
                    }
                    m_currentShapeIndices.clear();
                    if (m_isUserInteraction)
                    {
                        emit p->userInteractionDone(ito::Shape::Ellipse, aborted, shapes);
                        m_isUserInteraction = false;
                    }
                    emit p->geometricShapeFinished(shapes, aborted);
                    if (p->shapesWidget())
                    {
                        p->shapesWidget()->updateShapes(shapes);
                    }
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        break;

        case ito::Shape::Circle:
            if (polygonScale.size() == 0)
            {
                emit statusBarMessage(tr("Selection has been aborted."), 2000);
                aborted = true;
            }
            else
            {
                emit statusBarMessage(tr("%1 points have been selected.").arg(polygonScale.size() - 1), 2000);

                ito::Shape shape = ito::Shape::fromCircle(0.5 * (polygonScale[1] + polygonScale[0]), std::abs((polygonScale[1] - polygonScale[0]).x()) * 0.5);
                DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
                newItem->setColor(m_inverseColor0, m_inverseColor1);
                newItem->setFillOpacity(m_geometricShapeOpacity, m_geometricShapeOpacitySelected);
                if (this->m_inverseColor0.isValid())
                {
                    newItem->setPen(QPen(m_inverseColor0));
                }
                else
                {
                    newItem->setPen(QPen(Qt::green));
                }

                /*unselect all existing shapes before adding the new one*/
                for (QMap<int, DrawItem*>::iterator it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
                {
                    if (it.value() == NULL)
                    {
                        continue;
                    }
                    it.value()->setSelected(false);
                }

                newItem->setVisible(true);
                newItem->show();
                newItem->attach(this);
                newItem->setSelected(true);
                m_selectedShape = newItem;
                m_selectedShapeHitType = 0;
                m_pShapes.insert(newItem->getIndex(), newItem);
                m_currentShapeIndices.append(newItem->getIndex());
                emit p->geometricShapeAdded(newItem->getIndex(), newItem->getShape());
                emit p->geometricShapeCurrentChanged(newItem->getShape());
                replot();
            }

            // if further elements are needed reset the plot engine and go ahead else finish editing
            if (!aborted && m_elementsToPick > 1)
            {
                m_elementsToPick--;
                m_pMultiPointPicker->setEnabled(true);

                if (m_elementsToPick > 1) emit statusBarMessage(tr("Please draw %1 ellipses. Esc aborts the selection.").arg(m_elementsToPick));
                else emit statusBarMessage(tr("Please draw one ellipse. Esc aborts the selection."));
                return;
            }
            else
            {
                m_elementsToPick = 0;
                if (p)
                {
                    for (int i = 0; i < m_currentShapeIndices.size(); i++)
                    {
                        if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
                        shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
                    }
                    m_currentShapeIndices.clear();
                    if (m_isUserInteraction)
                    {
                        emit p->userInteractionDone(ito::Shape::Circle, aborted, shapes);
                        m_isUserInteraction = false;
                    }
                    emit p->geometricShapeFinished(shapes, aborted);
                    if (p->shapesWidget())
                    {
                        p->shapesWidget()->updateShapes(shapes);
                    }
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
        break;
        }
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
            m_isUserInteraction = true; // setting userinteraction to true, so we emit the counter part signal only if started with userinteractionstart
            setState(stateDrawShape); //this calls startOrStopDrawGeometricShape if everything is ok
            m_isUserInteraction = true; // setting userinteraction to true, so we emit the counter part signal only if started with userinteractionstart
        }
        else
        {
            retVal += ito::RetVal(ito::retError, 0, tr("Invalid type for userInteractionStart").toLatin1().data());
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

    if (start)
    {
        m_currentShapeIndices.clear();
        m_pMultiPointPicker->selection().clear();

        if (m_currentShapeType == ito::Shape::MultiPointPick) //multiPointPick
        {
            if (p)
            {
                emit p->geometricShapeStartUserInput(m_currentShapeType, m_isUserInteraction);
            }

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
        else if (m_currentShapeType == ito::Shape::Point)
        {
            if (p)
            {
                emit p->geometricShapeStartUserInput(m_currentShapeType, m_isUserInteraction);
            }

            m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::CrossRubberBand);
            m_pMultiPointPicker->setKeepAspectRatio(false);
            MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());

            if (m)
            {
                m->setMaxNrItems(1);
                m_pMultiPointPicker->setEnabled(true);

                if (m_elementsToPick > 1) 
                    emit statusBarMessage(tr("Please draw %1 points. Esc aborts the selection.").arg(m_elementsToPick));
                else 
                    emit statusBarMessage(tr("Please draw one point. Esc aborts the selection."));
            }
        }
        else if (m_currentShapeType == ito::Shape::Line)
        {
            if (p)
            {
                emit p->geometricShapeStartUserInput(m_currentShapeType, m_isUserInteraction);
            }

            m_pMultiPointPicker->setStateMachine(new QwtPickerDragLineMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::PolygonRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            m_pMultiPointPicker->setKeepAspectRatio(false);
            m_pMultiPointPicker->setEnabled(true);

            if (m_elementsToPick > 1) 
                emit statusBarMessage(tr("Please draw %1 lines. Esc aborts the selection.").arg(m_elementsToPick));
            else 
                emit statusBarMessage(tr("Please draw one line. Esc aborts the selection."));
        }
        else if (m_currentShapeType == ito::Shape::Rectangle)
        {
            if (p)
            {
                emit p->geometricShapeStartUserInput(m_currentShapeType, m_isUserInteraction);
            }

            m_pMultiPointPicker->setStateMachine(new QwtPickerDragRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::RectRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            m_pMultiPointPicker->setKeepAspectRatio(false);
            m_pMultiPointPicker->setEnabled(true);

            if (m_elementsToPick > 1) 
                emit statusBarMessage(tr("Please draw %1 rectangles. Esc aborts the selection.").arg(m_elementsToPick));
            else 
                emit statusBarMessage(tr("Please draw one rectangle. Esc aborts the selection."));
        }
        else if (m_currentShapeType == ito::Shape::Square)
        {
            if (p)
            {
                emit p->geometricShapeStartUserInput(m_currentShapeType, m_isUserInteraction);
            }

            m_pMultiPointPicker->setStateMachine(new QwtPickerDragRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::RectRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            m_pMultiPointPicker->setKeepAspectRatio(true);
            m_pMultiPointPicker->setEnabled(true);

            if (m_elementsToPick > 1) 
                emit statusBarMessage(tr("Please draw %1 squares. Esc aborts the selection.").arg(m_elementsToPick));
            else 
                emit statusBarMessage(tr("Please draw one square. Esc aborts the selection."));
        }
        else if (m_currentShapeType == ito::Shape::Ellipse)
        {
            if (p)
            {
                emit p->geometricShapeStartUserInput(m_currentShapeType, m_isUserInteraction);
            }

            m_pMultiPointPicker->setStateMachine(new QwtPickerDragRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::EllipseRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            m_pMultiPointPicker->setKeepAspectRatio(false);
            m_pMultiPointPicker->setEnabled(true);

            if (m_elementsToPick > 1) 
                emit statusBarMessage(tr("Please draw %1 ellipses. Esc aborts the selection.").arg(m_elementsToPick));
            else 
                emit statusBarMessage(tr("Please draw one ellipse. Esc aborts the selection."));
        }
        else if (m_currentShapeType == ito::Shape::Circle)
        {
            if (p)
            {
                emit p->geometricShapeStartUserInput(m_currentShapeType, m_isUserInteraction);
            }

            m_pMultiPointPicker->setStateMachine(new QwtPickerDragRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::EllipseRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            m_pMultiPointPicker->setKeepAspectRatio(true);
            m_pMultiPointPicker->setEnabled(true);

            if (m_elementsToPick > 1) 
                emit statusBarMessage(tr("Please draw %1 circles. Esc aborts the selection.").arg(m_elementsToPick));
            else 
                emit statusBarMessage(tr("Please draw one circle. Esc aborts the selection."));
        }
        else
        {
            m_pMultiPointPicker->setEnabled(false);

            if (p && m_isUserInteraction)
            {
                QVector<ito::Shape> shapes;
                emit p->geometricShapeStartUserInput(m_currentShapeType, m_isUserInteraction);
                m_isUserInteraction = false;
            }
            setState(stateIdle);
            retval += ito::RetVal(ito::retError, 0, tr("Unknown type for userInteractionStart").toLatin1().data());
        }

        //if spinbox for multiple planes has the focus, a possible ESC is not correctly caught.
        //therefore set the focus to the canvas.
        canvas()->setFocus();
    } 
    else //start = false
    {
        m_pMultiPointPicker->setEnabled(false);
        emit statusBarMessage(tr("Selection has been interrupted."), 2000);
        m_elementsToPick = 1;

        QVector<ito::Shape> shapes;
        for (int i = 0; i < m_currentShapeIndices.size(); i++)
        {
            if (!m_pShapes.contains(m_currentShapeIndices[i])) continue;
            shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
        }
        m_currentShapeIndices.clear();
        m_pMultiPointPicker->selection().clear();

        if (m_isUserInteraction)
        {
            if (p)
            {
                emit p->userInteractionDone(ito::Shape::Point, true, shapes);
            }
            m_isUserInteraction = false;
        }

        if (p)
        {
            emit p->geometricShapeFinished(shapes, true);

            if (p->shapesWidget())
            {
                p->shapesWidget()->updateShapes(shapes);
            }
            m_isUserInteraction = false;
        }

        setState(stateIdle);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::clearAllGeometricShapes()
{
    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());
    bool thingsToDo = m_pShapes.size() > 0;

    if (thingsToDo)
    {
        if (p && p->shapesWidget())
        {
            p->shapesWidget()->removeShapes();
        }
    }

    //delete all geometric shapes and marker sets
    QMapIterator<int, DrawItem *> i(m_pShapes);
    while (i.hasNext())
    {
        i.next();
        if (p)
        {
            emit p->geometricShapeDeleted(i.value()->getIndex());
        }
        delete i.value();
    }
    m_pShapes.clear();

    m_pActClearShapes->setEnabled(false);
    
    if (thingsToDo)
    {
        replot();
        if (p)
        {
            emit p->geometricShapesDeleted();
            p->updatePropertyDock();
        }
    }

    if (m_selectedShape != NULL)
    {
        if (p)
        {
            emit p->geometricShapeCurrentChanged(ito::Shape());
        }
        m_selectedShape = NULL;
        m_selectedShapeHitType = -1;
    }

    m_pActClearShapes->setEnabled(m_plottingEnabled && countGeometricShapes() > 0);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::deleteGeometricShape(const int idx)
{
    ito::RetVal retVal;
    bool found = false;

    if (m_pShapes.contains(idx))
    {
        //
        DrawItem *delItem = m_pShapes[idx];

        if (m_selectedShape == delItem)
        {
            ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());
            if (p)
            {
                emit p->geometricShapeCurrentChanged(ito::Shape());
            }
            m_selectedShape = NULL;
            m_selectedShapeHitType = -1;
        }

        delItem->detach();
        m_pShapes.remove(idx);
        delete delItem; // ToDo check for memory leak
        found = true;
    }

    if (!found)
    {
        retVal += ito::RetVal::format(ito::retError, 0, tr("No geometric shape with index '%d' found.").toLatin1().data(), idx);
    }
    else
    {
        replot();

        ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());
        if (p) emit  p->geometricShapeDeleted(idx);

        if (m_pShapes.count() == 0) //all deleted now
        {
            emit p->geometricShapesDeleted();
        }

        if (p->shapesWidget())
        {
            p->shapesWidget()->removeShape(idx);
        }
    }

    m_pActClearShapes->setEnabled(m_plottingEnabled && countGeometricShapes() > 0);

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setGeometricShapesFillOpacity(const int &opacity)
{
    if (opacity != m_geometricShapeOpacity)
    {
        for (QMap<int, DrawItem*>::const_iterator it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
        {
            if (it.value() != NULL)
            {
                it.value()->setFillOpacity(opacity, m_geometricShapeOpacitySelected);
            }
        }

        m_geometricShapeOpacity = opacity;
        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setGeometricShapesFillOpacitySelected(const int &opacity)
{
    if (opacity != m_geometricShapeOpacitySelected)
    {
        for (QMap<int, DrawItem*>::const_iterator it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
        {
            if (it.value() != NULL)
            {
                it.value()->setFillOpacity(m_geometricShapeOpacity, opacity);
            }
        }

        m_geometricShapeOpacitySelected = opacity;
        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
int ItomQwtPlot::getSelectedGeometricShapeIdx() const
{
    QMap<int, DrawItem*>::const_iterator it;
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
    QMap<int, DrawItem*>::const_iterator it = m_pShapes.begin();
    for (; it != m_pShapes.end(); ++it)
    {
        if (it.value() != NULL && it.value()->getIndex() == idx)
        {
            if (m_selectedShape != it.value())
            {
                ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());
                if (p)
                {
                    emit p->geometricShapeCurrentChanged(it.value()->getShape());
                }
            }
            m_selectedShape = it.value();
            m_selectedShapeHitType = 0;
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
    if (failed)
    {
        if (m_selectedShape != NULL)
        {
            ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());
            if (p)
            {
                emit p->geometricShapeCurrentChanged(ito::Shape());
            }
            m_selectedShape = NULL;
        }
        m_selectedShapeHitType = -1;
        emit statusBarMessage(tr("Could not set active element, index out of range."), 12000);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::setGeometricShapes(const QVector<ito::Shape> &geometricShapes)
{
    ito::RetVal retVal;
    clearAllGeometricShapes();
    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());
    QVector<ito::Shape> updatedShapes;


    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Could not set geometric shapes, api is missing").toLatin1().data());
    }
    else
    {
        int nrOfShapes = geometricShapes.size();
            
        DrawItem *newItem = NULL;
        // The definition do not correspond to the definetion of primitiv elements

        foreach(ito::Shape shape, geometricShapes)
        {
            if (!m_allowedShapeTypes.testFlag((ItomQwtPlotEnums::ShapeType)shape.type()))
            {
                retVal += ito::RetVal(ito::retWarning, 0, tr("The shapes contain at least one shape type, that is not allowed for this plot, and is therefore ignored.").toLatin1().data());
            }
            else
            {
                if (m_pShapes.contains(shape.index()))
                {
                    m_pShapes[shape.index()]->setShape(shape, m_inverseColor0, m_inverseColor1);
                    if (p->shapesWidget())
                    {
                        p->shapesWidget()->updateShape(shape);
                    }
                    emit p->geometricShapeChanged(shape.index(), shape);
                    updatedShapes << shape;
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
                            newItem->setFillOpacity(m_geometricShapeOpacity, m_geometricShapeOpacitySelected);
                            newItem->setVisible(true);
                            newItem->show();
                            newItem->attach(this);
                            m_pShapes.insert(newItem->getIndex(), newItem);
                            shape.setIndex(newItem->getIndex());
                            if (p)
                            {
                                if (p->shapesWidget())
                                {
                                    p->shapesWidget()->updateShape(shape);
                                }
                                emit p->geometricShapeAdded(newItem->getIndex(), newItem->getShape());
                            }
                            updatedShapes << shape;
                        }
                        break;

                    default:
                        retVal += ito::RetVal(ito::retError, 0, tr("Invalid or unsupported shape type").toLatin1().data());
                        break;
                    }

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
    
    if (p && updatedShapes.size() > 0) 
    {
        emit  p->geometricShapeFinished(updatedShapes, retVal.containsError());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::addGeometricShape(const ito::Shape &geometricShape, int *newIndex /*= NULL*/)
{
    ito::RetVal retVal;
    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());
    QVector<ito::Shape> updatedShapes;


    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Could not add a geometric shape, api is missing").toLatin1().data());
    }
    else
    {
        if (m_pShapes.contains(geometricShape.index()))
        {
            retVal += ito::RetVal(ito::retError, 0, tr("Could not add the geometric shape since a shape with the same index already exists").toLatin1().data());
        }
        else if (!m_allowedShapeTypes.testFlag((ItomQwtPlotEnums::ShapeType)geometricShape.type()))
        {
            retVal += ito::RetVal(ito::retError, 0, tr("The type of the shape is not allowed for this plot.").toLatin1().data());
        }
        else
        {
            
            DrawItem *newItem = NULL;
            switch (geometricShape.type())
            {
            case ito::Shape::MultiPointPick:
            case ito::Shape::Point:
            case ito::Shape::Line:
            case ito::Shape::Rectangle:
            case ito::Shape::Square:
            case ito::Shape::Ellipse:
            case ito::Shape::Circle:
                newItem = new DrawItem(geometricShape, m_shapeModificationModes, this, &retVal, m_shapesLabelVisible);
                if (!retVal.containsError())
                {
                    newItem->setColor(m_inverseColor0, m_inverseColor1);
                    newItem->setFillOpacity(m_geometricShapeOpacity, m_geometricShapeOpacitySelected);
                    newItem->setVisible(true);
                    newItem->show();
                    newItem->attach(this);
                    m_pShapes.insert(newItem->getIndex(), newItem);
                    if (newIndex)
                    {
                        *newIndex = newItem->getIndex();
                    }
                    if (p->shapesWidget())
                    {
                        p->shapesWidget()->updateShape(newItem->getShape());
                    }

                    emit p->geometricShapeAdded(newItem->getIndex(), newItem->getShape());
                    updatedShapes << geometricShape;
                }
                break;

            default:
                retVal += ito::RetVal(ito::retError, 0, tr("Invalid or unsupported shape type").toLatin1().data());
                break;                
            }

            replot();
        }
    }

    m_pActClearShapes->setEnabled(m_plottingEnabled && countGeometricShapes() > 0);
    
    if (retVal.hasErrorMessage())
    {
        emit statusBarMessage(retVal.errorMessage(), 12000);
    }
    
    if (p && updatedShapes.size() > 0) 
    {
        emit  p->geometricShapeFinished(updatedShapes, retVal.containsError());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::updateGeometricShape(const ito::Shape &geometricShape, int *newIndex /*= NULL*/)
{
    ito::RetVal retVal;
    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());
    QVector<ito::Shape> updatedShapes;


    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Could not modify a geometric shape, api is missing").toLatin1().data());
    }
    else if (!m_allowedShapeTypes.testFlag((ItomQwtPlotEnums::ShapeType)geometricShape.type()))
    {
        retVal += ito::RetVal(ito::retError, 0, tr("The type of the shape is not allowed for this plot.").toLatin1().data());
    }
    else
    {            
        DrawItem *newItem = NULL;
        // The definition do not correspond to the definetion of primitiv elements

        if (m_pShapes.contains(geometricShape.index()))
        {
            m_pShapes[geometricShape.index()]->setShape(geometricShape, m_inverseColor0, m_inverseColor1);
            if (p->shapesWidget())
            {
                p->shapesWidget()->updateShape(geometricShape);
            }

            emit p->geometricShapeChanged(geometricShape.index(), geometricShape);
            updatedShapes << geometricShape;

            if (newIndex)
            {
                *newIndex = geometricShape.index();
            }
        }
        else
        {
            switch (geometricShape.type())
            {
            case ito::Shape::MultiPointPick:
            case ito::Shape::Point:
            case ito::Shape::Line:
            case ito::Shape::Rectangle:
            case ito::Shape::Square:
            case ito::Shape::Ellipse:
            case ito::Shape::Circle:
                newItem = new DrawItem(geometricShape, m_shapeModificationModes, this, &retVal, m_shapesLabelVisible);
                if (!retVal.containsError())
                {
                    newItem->setColor(m_inverseColor0, m_inverseColor1);
                    newItem->setFillOpacity(m_geometricShapeOpacity, m_geometricShapeOpacitySelected);
                    newItem->setVisible(true);
                    newItem->show();
                    newItem->attach(this);
                    m_pShapes.insert(newItem->getIndex(), newItem);
                    if (newIndex)
                    {
                        *newIndex = newItem->getIndex();
                    }
                    if (p->shapesWidget())
                    {
                        p->shapesWidget()->updateShape(newItem->getShape());
                    }
                    emit p->geometricShapeAdded(newItem->getIndex(), newItem->getShape());
                    updatedShapes << geometricShape;
                }
                break;

            default:
                retVal += ito::RetVal(ito::retError, 0, tr("Invalid marker type").toLatin1().data());
                break;
            }
        }

        replot();
    }

    m_pActClearShapes->setEnabled(m_plottingEnabled && countGeometricShapes() > 0);
    
    if (retVal.hasErrorMessage())
    {
        emit statusBarMessage(retVal.errorMessage(), 12000);
    }
    
    if (p && updatedShapes.size() > 0) 
    {
        emit  p->geometricShapeFinished(updatedShapes, retVal.containsError());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<ito::Shape> ItomQwtPlot::getGeometricShapes()
{
    QVector<ito::Shape> shapes;
    shapes.reserve(m_pShapes.size());
    QMap<int, DrawItem*>::Iterator it = m_pShapes.begin();

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
ito::RetVal ItomQwtPlot::setGeometricShapeLabel(int idx, const QString &label)
{
    if (!m_pShapes.contains(idx))
    {
        return ito::RetVal(ito::retError, 0, tr("Geometric shape not found").toLatin1().data());
    }

    m_pShapes[idx]->setLabel(label);
    replot();

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::setGeometricShapeLabelVisible(int idx, bool setVisible)
{
    if (!m_pShapes.contains(idx))
    {
        return ito::RetVal(ito::retError, 0, tr("Geometric shape not found").toLatin1().data());
    }

    m_pShapes[idx]->setLabelVisible(setVisible);
    replot();

    return ito::retOk;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::exportCanvas(const bool copyToClipboardNotFile, const QString &fileName, const QSizeF &curSizeMm /*= QSizeF(0.0,0.0)*/, const int resolution /*= 300*/)
{
    if (!copyToClipboardNotFile)
    {
        QFileInfo fileInfo(fileName);
        if (fileInfo.exists())
        {
            if (!fileInfo.isWritable())
            {
                return ito::RetVal::format(ito::retError, 0, tr("The file '%s' already exists but cannot be overwritten.").toLatin1().data(), fileName.toLatin1().data());
            }
            else
            {
                QFile file(fileName);
                if (!file.open(QIODevice::WriteOnly))
                {
                    return ito::RetVal::format(ito::retError, 0, tr("The file '%s' already exists but cannot be overwritten (Maybe it is opened in another application).").toLatin1().data(), 
                        fileName.toLatin1().data());
                }
                file.close();
            }
        }
    }

    ItomQwtDObjFigure* hMyParent = (ItomQwtDObjFigure*)parent();

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
        bool plotInfoVisible = false;
        qreal resFactor = std::max(resolution / 92.0, 0.5); //todo: consider dpi of real screen (92.0 is only a default value)
        QSize newSizePx;

        if (curSizeMm.isEmpty())
        {
            newSizePx = size() * resFactor;
        }
        else
        {
            newSizePx = (curSizeMm * (resolution / 25.4)).toSize();
        }

        QClipboard *clipboard = QApplication::clipboard();


        if ((hMyParent->markerWidget()  && hMyParent->markerWidget()->isVisible()) ||
            (hMyParent->pickerWidget()  && hMyParent->pickerWidget()->isVisible()) ||
            (hMyParent->dObjectWidget() && hMyParent->dObjectWidget()->isVisible()) ||
            (hMyParent->shapesWidget()  && hMyParent->shapesWidget()->isVisible()))
        {
#if QT_VERSION >= 0x050000
            plotInfoVisible = true;
            emit statusBarMessage(tr("Copy current view to clipboard including meta information widgets ..."));
#else
            emit statusBarMessage(tr("Copy current view to clipboard without meta information widgets (requires Qt5) ..."));
#endif
        }
        else
        {
            emit statusBarMessage(tr("Copy current view to clipboard ..."));
        }

        
        QImage img(newSizePx, QImage::Format_ARGB32);
        QPainter painter(&img);
        painter.scale(resFactor, resFactor);
        renderer.render(this, &painter, rect());
        img.setDotsPerMeterX(img.dotsPerMeterX() * resFactor); //setDotsPerMeterXY must be set after rendering
        img.setDotsPerMeterY(img.dotsPerMeterY() * resFactor);

#if QT_VERSION >= 0x050000
        if (plotInfoVisible)
        {
            QList<QWidget*> widgets;
            widgets << hMyParent->dObjectWidget() << hMyParent->pickerWidget() << \
                hMyParent->markerWidget()  << hMyParent->shapesWidget();
            QList<QPixmap> pixmaps;
            int height = 0;
            int width = 0;
            int x0 = img.width();

            foreach(QWidget* widget, widgets)
            {
                if (widget && widget->isVisible())
                {
                    QWidget *parent = qobject_cast<QWidget*>(widget->parent());
                    widget = parent ? parent : widget;
                    QPixmap pixmap = widget->grab();
                    height += pixmap.height();
                    width = qMax(width, pixmap.width());
                    pixmaps << pixmap;
                }
            }

            QImage imgTotal(img.width() + width, qMax(img.height(), height), QImage::Format_ARGB32);
            QPainter painter(&imgTotal);
            painter.drawImage(0, 0, img);
            int y0 = 0;
            foreach(QPixmap pixmap, pixmaps)
            {
                painter.drawPixmap(x0, y0, pixmap);
                y0 += pixmap.height();
            }

            img = imgTotal;
        }
#endif

        clipboard->setImage(img);

        if (plotInfoVisible)
        {
            emit statusBarMessage(tr("Copy current view to clipboard including infoWidgets. Done."), 1000);
        }
        else
        {
            emit statusBarMessage(tr("Copy current view to clipboard. Done."), 1000);
        }
    }
    else
    {
        QSizeF newSizeMm = curSizeMm;

        if (curSizeMm.isEmpty())
        {
            newSizeMm = size() * (25.4 / 92.0); //todo: consider dpi of real screen (92.0 is only a default value)
        }

        renderer.renderDocument(this, fileName, newSizeMm, resolution);
    }

    setPalette(curPalette);
    setCanvasBackground(curBrush);

    replot();
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::printCanvas()
{
    if (!m_pPrinter)
    {
        m_pPrinter = new QPrinter();
        m_pPrinter->setPageMargins(15, 15, 15, 15, QPrinter::Millimeter);
    }

    QPrintPreviewDialog printPreviewDialog(m_pPrinter, this, Qt::Window);
    connect(&printPreviewDialog, SIGNAL(paintRequested(QPrinter*)), this, SLOT(printPreviewRequested(QPrinter*)));
    printPreviewDialog.exec();
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::printPreviewRequested(QPrinter* printer)
{
    QwtPlotRenderer renderer;
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    renderer.renderTo(this, *printer);
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuActSave()
{
    //first get the output format information, then the filename (in order to let the user see what can be adjusted before defining a filename)
    bool abort = true;

    QSizeF curSizePx = size();
    QSizeF curSizeMm;
    int resolution = 300;

    DialogExportProperties *dlg = new DialogExportProperties(curSizePx, qobject_cast<ItomQwtDObjFigure*>(parent()));
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData(curSizePx, curSizeMm, resolution);
        abort = false;
    }

    DELETE_AND_SET_NULL(dlg);

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
        filter.join(";;"), NULL);
#endif

    ito::RetVal retval;

    if (!fileName.isEmpty())
    {
        QFileInfo fi(fileName);
        saveDefaultPath = fi.path();

        retval += exportCanvas(false, fileName, curSizeMm, resolution);
    }

    if (retval.containsError())
    {
        QMessageBox msgBox;
        msgBox.setText(tr("Error while saving the plot").toLatin1().data());
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
        msgBox.setText(tr("Warning while saving the plot").toLatin1().data());
        if (retval.errorMessage())
        {
            msgBox.setInformativeText(QLatin1String(retval.errorMessage()));
        }
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.exec();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuActPrint()
{
    printCanvas();
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
    QSizeF size(0.0, 0.0);
    int dpi = 300;

    if (ito::ITOM_API_FUNCS_GRAPH)
    {
        dpi = qBound(48, apiGetFigureSetting(parent(), "copyClipboardResolutionDpi", 300, NULL).value<int>(), 2000);
    }

    exportCanvas(true, "", size, dpi);
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuSendCurrentToWorkspace()
{
    bool ok;
    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(parent());

    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        emit statusBarMessage(tr("Could not send object to workspace, api is missing."), 4000);
    }

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
            msgBox.setText(tr("Warning sending data object to workspace").toLatin1().data());
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

            switch (s)
            {
            case 'b':
                symPen.setColor(Qt::blue);
                break;
            case 'g':
                symPen.setColor(Qt::green);
                break;
            case 'r':
                symPen.setColor(Qt::red);
                break;
            case 'c':
                symPen.setColor(Qt::cyan);
                break;
            case 'm':
                symPen.setColor(Qt::magenta);
                break;
            case 'y':
                symPen.setColor(Qt::yellow);
                break;
            case 'k':
                symPen.setColor(Qt::black);
                break;
            case 'w':
                symPen.setColor(Qt::white);
                break;
            }

            s = rgexp.cap(2).toLatin1()[0];
            bool ok;

            switch (s)
            {
            case '.':
            case 'o':
                symStyle = QwtSymbol::Ellipse;
                break;
            case 's':
                symStyle = QwtSymbol::Rect;
                break;
            case 'd':
                symStyle = QwtSymbol::Diamond;
                break;
            case '>':
                symStyle = QwtSymbol::RTriangle;
                break;
            case 'v':
                symStyle = QwtSymbol::DTriangle;
                break;
            case '^':
                symStyle = QwtSymbol::UTriangle;
                break;
            case '<':
                symStyle = QwtSymbol::LTriangle;
                break;
            case 'x':
                symStyle = QwtSymbol::XCross;
                break;
            case '*':
                symStyle = QwtSymbol::Star1;
                break;
            case '+':
                symStyle = QwtSymbol::Cross;
                break;
            case 'h':
                symStyle = QwtSymbol::Hexagon;
                break;
            }

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
            retval += ito::RetVal(ito::retError, 0, tr("The style tag does not correspond to the required format: ColorStyleSize[;Linewidth] (Color = b,g,r,c,m,y,k,w; Style = o,s,d,>,v,^,<,x,*,+,h)").toLatin1().data());
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

            if (plane == -1 || plane == m_currentPlane)
            {
                marker->setVisible(true);
            }
            else
            {
                marker->setVisible(false);
            }

            m_plotMarkers.insert(tmpID, QPair<int, QwtPlotMarker*>(plane, marker));
        }

        PlotInfoMarker *pim = ((ItomQwtDObjFigure*)parent())->markerWidget();
        if (pim)
        {
            ito::Shape shapes = ito::Shape::fromMultipoint(markerPolygon, m_plotMarkers.size(), tmpID);
            pim->updateMarker(shapes);
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

    PlotInfoMarker *pim = ((ItomQwtDObjFigure*)parent())->markerWidget();
    if (pim)
    {
        pim->removeMarker(id);
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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::changeVisibleMarkers(int currentPlane)
{
    m_currentPlane = currentPlane;

    QMutableHashIterator<QString, QPair<int, QwtPlotMarker*> > i(m_plotMarkers);

    while (i.hasNext())
    {
        i.next();
        i.value().second->setVisible(i.value().first == -1 || i.value().first == currentPlane);
    }

    return ito::retOk;
}
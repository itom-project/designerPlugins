/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2020, Institut fuer Technische Optik (ITO), 
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
#include <qdialogbuttonbox.h>
#include <qcheckbox.h>
#include <qregularexpression.h>

#include "itomPlotZoomer.h"
#include "itomPlotMagnifier.h"
#include "drawItem.h"
#include "itomQwtDObjFigure.h"
#include "userInteractionPlotPicker.h"
#include "multiPointPickerMachine.h"
#include "dialogExportProperties.h"
#include "itomQwtPlotPanner.h"
#include "markerWidget.h"

#include "../DataObject/dataObjectFuncs.h"

#include "common/apiFunctionsGraphInc.h"
#include "common/retVal.h"
#include "plot/designerPluginInterfaceVersion.h"

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
    m_numShapesToPick(1),
    m_currentShapeType(ito::Shape::Point),
    m_allowedShapeTypes(~ItomQwtPlotEnums::ShapeTypes()),
    m_buttonStyle(0),
    m_boxFrame(true),
    m_plottingEnabled(true),
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
    m_pActShapesToolbox(nullptr),
    m_pActMarkerToolbox(nullptr),
    m_pActDObjInfoToolbox(nullptr),
    m_pActPickerToolbox(nullptr),
    m_pMenuToolboxes(nullptr),
    m_pMenuShapeType(NULL),
    m_pActClearShapes(NULL),
    m_pActProperties(NULL),
    m_pActCamParameters(NULL),
    m_pActShapeType(NULL),
    m_axisColor(Qt::black),
    m_textColor(Qt::black),
    m_backgroundColor(Qt::white),
    m_canvasColor(Qt::white),
    m_styledBackground(false),
    m_pPrinter(NULL),
    m_shapeModifiedByMouseMove(false),
    m_geometricShapeOpacity(0),
    m_geometricShapeOpacitySelected(0),
	m_mouseCatchTolerancePx(8),
    m_markerModel(new MarkerModel(false, this))
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
    m_pPanner = new ItomQwtPlotPanner(canvas());
    m_pPanner->setAxisEnabled(QwtPlot::yRight, false);
    m_pPanner->setCursor(Qt::SizeAllCursor);
    m_pPanner->setEnabled(true);
    m_pPanner->setMouseButton(Qt::MiddleButton);
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

        m_pActCamParameters = parent->cameraParamEditorDockWidget()->toggleViewAction();
        m_pActCamParameters->setVisible(false);

        MarkerWidget* markerInfoWidget = parent->markerInfoWidget();
        markerInfoWidget->setModel(m_markerModel.data());
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
    a->setToolTip(tr("Clear all existing geometric shapes"));
    connect(a, SIGNAL(triggered()), this, SLOT(clearAllGeometricShapes()));

    //m_actApectRatio
    m_pActAspectRatio = a = new QAction(tr("Lock Aspect Ratio"), p);
    a->setObjectName("actRatio");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Toggle fixed / variable aspect ration between axis x and y"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActRatio(bool)));

    m_pActShapesToolbox = p->shapesDockWidget()->toggleViewAction();
    m_pActMarkerToolbox = p->markerDockWidget()->toggleViewAction();
    m_pActDObjInfoToolbox = p->dObjectDockWidget()->toggleViewAction();
    m_pActPickerToolbox = p->pickerDockWidget()->toggleViewAction();
    m_pMenuToolboxes = new QMenu(tr("Toolboxes"), p);
    m_pMenuToolboxes->addAction(m_pActShapesToolbox);
    m_pMenuToolboxes->addAction(m_pActMarkerToolbox);
    m_pMenuToolboxes->addAction(m_pActPickerToolbox);
    m_pMenuToolboxes->addAction(m_pActDObjInfoToolbox);

    //m_pActShapeType
    m_pActShapeType = new QAction(tr("Draw Geometric Shape"), p);
    m_pMenuShapeType = new QMenu(tr("Draw Geometric Shape"), p);
    m_pActShapeType->setMenu(m_pMenuShapeType);

    a = m_pMenuShapeType->addAction(tr("Point"));
    a->setData(ito::Shape::Point);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Line"));
    a->setData(ito::Shape::Line);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Rectangle"));
    a->setData(ito::Shape::Rectangle);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Square"));
    a->setData(ito::Shape::Square);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Ellipse"));
    a->setData(ito::Shape::Ellipse);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Circle"));
    a->setData(ito::Shape::Circle);
    a->setCheckable(true);

    a = m_pMenuShapeType->addAction(tr("Polygon"));
    a->setData(ito::Shape::Polygon);
    a->setCheckable(true);

    m_pMenuShapeType->addSeparator();
    m_pMenuShapeType->addAction(m_pActShapesToolbox);

    m_pActShapeType->setData(ito::Shape::Rectangle);
    m_pActShapeType->setVisible(true);
    m_pActShapeType->setCheckable(true);
    connect(m_pMenuShapeType, SIGNAL(triggered(QAction*)), this, SLOT(mnuGroupShapeTypes(QAction*)));
    connect(m_pActShapeType, SIGNAL(triggered(bool)), this, SLOT(mnuShapeType(bool)));
}

//---------------------------------------------------------------------------
void ItomQwtPlot::loadStyles(bool overwriteDesignableProperties)
{
    //overwriteDesignableProperties: if the plot widget is configured for the first time
    //(e.g. in the constructor), all styles and properties should be set to their default
    //value, hence overwriteDesignableProperties is set to true. If the plot is
    //displayed as standalone-plot, the styles have to be updated once the APIs are available
    //and the styles can be read from the itom settings. In this case, overwriteDesignableProperties
    //is also set to true. Only in the case, that the plot is integrated in a ui-file, that
    //has been configured in the QtDesigner, overwriteDesignableProperties has to be set
    //to false if not called during the construction, such that the properties from
    //the QtDesigner are not overwritten again by the itom settings. Properties, that
    //are not designable by the QtDesigner should nevertheless be obtained by the itom settings.

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

        if (overwriteDesignableProperties)
        {
            //the following settings are designable via properties in QtDesigner
            m_unitLabelStyle = (ito::AbstractFigure::UnitLabelStyle)(apiGetFigureSetting(parent(), "unitLabelStyle", m_unitLabelStyle, NULL).value<int>());
            bool enableBoxFrame = (ito::AbstractFigure::UnitLabelStyle)(apiGetFigureSetting(parent(), "enableBoxFrame", m_boxFrame, NULL).value<bool>());
            setBoxFrame(enableBoxFrame);
        }
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
        /*if (item && item->getAutoColor())
        {*/
            item->setColor(m_inverseColor0, m_inverseColor1, m_inverseColor1);
        //}
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
    m_pMenuToolboxes->setIcon(QIcon(":/application/icons/list.png"));

    if (style == 0)
    {
        m_pActSave->setIcon(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"));
        m_pActPrint->setIcon(QIcon(":/itomDesignerPlugins/general/icons/print.png"));
        m_pActCopyClipboard->setIcon(QIcon(":/itomDesignerPlugins/general/icons/clipboard.png"));
        m_pActHome->setIcon(QIcon(":/itomDesignerPlugins/general/icons/home.png"));
        m_pActPan->setIcon(QIcon(":/itomDesignerPlugins/general/icons/move.png"));
        m_pActClearShapes->setIcon(QIcon(":/itomDesignerPlugins/general/icons/editDelete.png"));
        m_pActAspectRatio->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio11.png"));
        
        if (m_pActProperties)
        {
            m_pActProperties->setIcon(QIcon(":/itomDesignerPlugins/general/icons/settings.png"));
        }

        if (m_pActCamParameters)
        {
            m_pActCamParameters->setIcon(QIcon(":/itomDesignerPlugins/general/icons/camParams.png"));
        }
        
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
            case ito::Shape::Polygon:
                m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/polygon.png"));
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
        case ito::Shape::Polygon:
            m_pActShapeType->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/polygon_lt.png"));
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
ItomQwtPlotPanner *ItomQwtPlot::panner() const
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

    bool ok; // only modify actions of m_pMenuShapeType, that have an 
             // integer data(). Do not touch the separator or dock widget toggle action

    foreach(QAction *a, m_pMenuShapeType->actions())
    {
        a->data().toInt(&ok);

        if (ok)
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
    bool ok; // false if action has no integer data --> no shape type clicked but any other action
    int shapeType = action->data().toInt(&ok);

    if (ok)
    {
        switch (shapeType)
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
        case ito::Shape::Polygon:
            m_currentShapeType = ito::Shape::Polygon;
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
            this->updateScaleValues(true, true, true);
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
    if (m_showContextMenu && m_pPanner->leftClickPanner() == false)
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
            m_numShapesToPick = 1;
        }

        m_pActZoom->setChecked(state == stateZoom);
        m_pZoomer->setEnabled(state == stateZoom);
        m_pActPan->setChecked(state == statePan);
        m_pPanner->setLeftClickPanner(state == statePan);
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
                    m_numShapesToPick = std::max(m_numShapesToPick, -1);
                    startOrStopDrawGeometricShape(true);
                    break;

                case ito::Shape::Line:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/pntline.png" : ":/itomDesignerPlugins/plot_lt/icons/pntline_lt.png"));
                    m_numShapesToPick = std::max(m_numShapesToPick, -1);
                    startOrStopDrawGeometricShape(true);
                    break;

                case ito::Shape::Rectangle:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/rectangle.png" : ":/itomDesignerPlugins/plot_lt/icons/rectangle_lt.png"));
                    m_numShapesToPick = std::max(m_numShapesToPick, -1);
                    startOrStopDrawGeometricShape(true);
                    break;

                case ito::Shape::Ellipse:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/ellipse.png" : ":/itomDesignerPlugins/plot_lt/icons/ellipse_lt.png"));
                    m_numShapesToPick = std::max(m_numShapesToPick, -1);
                    startOrStopDrawGeometricShape(true);
                    break;

                case ito::Shape::Circle:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/circle.png" : ":/itomDesignerPlugins/plot_lt/icons/circle_lt.png"));
                    m_numShapesToPick = std::max(m_numShapesToPick, -1);
                    startOrStopDrawGeometricShape(true);
                    break;

                case ito::Shape::Square:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/square.png" : ":/itomDesignerPlugins/plot_lt/icons/square_lt.png"));
                    m_numShapesToPick = std::max(m_numShapesToPick, -1);
                    startOrStopDrawGeometricShape(true);
                    break;

                case ito::Shape::Polygon:
                    m_pActShapeType->setIcon(QIcon(m_buttonStyle == 0 ? ":/itomDesignerPlugins/plot/icons/polygon.png" : ":/itomDesignerPlugins/plot_lt/icons/polygon_lt.png"));
                    m_numShapesToPick = std::max(m_numShapesToPick, -1);
                    startOrStopDrawGeometricShape(true);
                    break;
                }
            }

            bool ok; // only modify actions of m_pMenuShapeType, that have an 
                     // integer data(). Do not touch the separator or dock widget toggle action
            foreach(QAction *act, m_pMenuShapeType->actions())
            {
                act->data().toInt(&ok);

                if (ok)
                {
                    act->setChecked(act->data() == m_currentShapeType);
                }
            }

            canvas()->setCursor(Qt::CrossCursor);
            break;
        }

        default:
            canvas()->setCursor(Qt::ArrowCursor);
            break;
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

        if (indices_to_delete.size() == 1 && 
            m_pShapes[indices_to_delete[0]]->getShape().type() == ito::Shape::Polygon)
        {
            const ito::Shape &shape = m_pShapes[indices_to_delete[0]]->getShape();
			QPolygonF newBasePoints = shape.basePoints();
			int marker_index = m_pShapes[indices_to_delete[0]]->getSelectedMarker();

			if (marker_index >= 0 && 
                marker_index < newBasePoints.size() &&
                newBasePoints.size() > 1)
			{
				//delete single point
				newBasePoints.remove(marker_index);
				ito::Shape newShape(ito::Shape::Polygon, shape.flags(), newBasePoints, shape.index(), shape.transform());
                newShape.setUnclosed(shape.unclosed() || newBasePoints.size() <= 2);
				m_pShapes[indices_to_delete[0]]->setShape(newShape);
                int new_marker_index = marker_index - 1;

                if (new_marker_index < 0)
                {
                    new_marker_index = newBasePoints.size() - 1;
                }

                m_pShapes[indices_to_delete[0]]->setSelected(true, new_marker_index);

                ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(this->parent());

                if (p)
                {
                    emit p->geometricShapeCurrentChanged(newShape);

                    if (p->shapesWidget())
                    {
                        QVector<ito::Shape> shapes;
                        shapes << newShape;
                        p->shapesWidget()->updateShapes(shapes);
                    }
                }

				replot();
			}
			else
			{
				//delete entire polygon, also because there is no point left any more
				deleteGeometricShape(indices_to_delete[0]);
			}
        }
        else
        {
            foreach(int index, indices_to_delete)
            {
                deleteGeometricShape(index);
            }
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
        double tol_x_scale = std::abs(invTransform(QwtPlot::xBottom, m_mouseCatchTolerancePx) - invTransform(QwtPlot::xBottom, 0)); //tolerance in pixel for snapping to a geometric shape in x-direction
        double tol_y_scale = std::abs(invTransform(QwtPlot::yLeft, m_mouseCatchTolerancePx) - invTransform(QwtPlot::yLeft, 0)); //tolerance in pixel for snapping to a geometric shape in y-direction
        ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(parent());
        int hitType = DrawItem::hitNone; //not hit
        bool currentShapeFound = false;

        //at first check if the currently selected item is hit: If so, take this; else take any other item
        QMap<int, DrawItem*>::iterator it = m_pShapes.begin();
        if (m_selectedShape)
        {
            for (it = m_pShapes.begin(); it != m_pShapes.end(); ++it)
            {
                if (it.value() == m_selectedShape)
                {
                    hitType = it.value()->hitEdge(scalePos, tol_x_scale, tol_y_scale);
                    break;
                }
            }
        }

        if (hitType == DrawItem::hitNone) //current shape is not hit, reset it to begin()
        {
            it = m_pShapes.begin();
        }
        
        for (; it != m_pShapes.end(); ++it)
        {
            if (it.value() == NULL)
            {
                continue;
            }

            if (hitType == DrawItem::hitNone)
            {
                hitType = it.value()->hitEdge(scalePos, tol_x_scale, tol_y_scale);
            }

            if (hitType != DrawItem::hitNone)
            {
                // hit a line of a polygon with shift key pressed, then we add a new point to the line
                if (hitType == DrawItem::hitMove && event->modifiers() == Qt::ShiftModifier && it.value()->getShape().type() == ito::Shape::Polygon)
                {
                    DrawItem *ditem = it.value();
                    const ito::Shape &shape = ditem->getShape();
                    int closed = shape.unclosed();
                    int hitLine;
                    for (int i = 0; i < shape.basePoints().size() - closed; i++)
                    {
                        if (ditem->hitLine(scalePos, QLineF(shape.basePoints()[i], shape.basePoints()[(i + 1) % shape.basePoints().size()]), tol_x_scale, tol_y_scale))
                        {
                            hitLine = i;
                            break;
                        }
                    }
                    QPolygonF newBasePoints = shape.basePoints();
                    newBasePoints.insert(hitLine + 1, scalePos);
                    ditem->setShape(ito::Shape(ito::Shape::Polygon, shape.flags(), newBasePoints, shape.index(), shape.transform()));
                }

                if (hitType == DrawItem::hitMove)
                {
                    it.value()->setSelected(true, -1);
                }
                else if (hitType >= DrawItem::hitResize)
                {
                    it.value()->setSelected(true, hitType - DrawItem::hitResize);
                }
                else if (hitType <= DrawItem::hitRotation)
                {
                    it.value()->setSelected(true, DrawItem::hitRotation - hitType);
                }

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

                if (hitType == DrawItem::hitMove)
                {
                    m_startMouseScaleDiff = m_startMouseScale - m_selectedShape->getMarkerPosScale(0);
                }
                else if (hitType >= DrawItem::hitResize)
                {
                    m_startMouseScaleDiff = m_startMouseScale - m_selectedShape->getMarkerPosScale(hitType - DrawItem::hitResize);
                }
                else if (hitType <= DrawItem::hitRotation)
                {
                    m_startMouseScaleDiff = m_startMouseScale - m_selectedShape->getMarkerPosScale(DrawItem::hitRotation - hitType);
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
            m_selectedShapeHitType = DrawItem::hitNone;
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
    else if (m_state == stateDrawShape && event->button() == Qt::MiddleButton)
    {
        if (m_pShapes.size() > 0 && m_currentShapeIndices.size() > 0
            && m_pShapes[m_currentShapeIndices[0]]->getShape().type() == ito::Shape::Polygon)
        {
            closePolygon(0);
        }
    }

    if (!event->isAccepted())
    {
        QwtPlot::mousePressEvent(event);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mouseMoveEvent(QMouseEvent * event)
{
    // mouse tracking should be only used when drawing polygon, so make a short cut here,
    // avoiding to run too much of code 
    if (event->buttons() == Qt::NoButton)
    {
        if (m_state == stateDrawShape)
        {
            if (m_pShapes.size() > 0 && 
                m_currentShapeIndices.size() > 0 && 
                m_pShapes[m_currentShapeIndices.last()]->getShape().type() == ito::Shape::Polygon)
            {
                const ito::Shape &thisShape = m_pShapes[m_currentShapeIndices.last()]->getShape();
                const QPolygonF &poly = thisShape.basePoints();
                int canxpos = event->x() - canvas()->x();
                int canypos = event->y() - canvas()->y();

                QPointF scalePos(invTransform(QwtPlot::xBottom, canxpos), invTransform(QwtPlot::yLeft, canypos));
                double tol_x = std::abs(
                    invTransform(QwtPlot::xBottom, m_mouseCatchTolerancePx) -
                    invTransform(QwtPlot::xBottom, 0)); //tolerance in pixel for snapping to a geometric shape in x-direction

                double tol_y = std::abs(
                    invTransform(QwtPlot::yLeft, m_mouseCatchTolerancePx) - 
                    invTransform(QwtPlot::yLeft, 0)); //tolerance in pixel for snapping to a geometric shape in y-direction

                QLineF line(poly[0], scalePos);
                
                if ((std::abs(line.dx()) <= tol_x) && (std::abs(line.dy()) <= tol_y))
                {
                    if (!QApplication::overrideCursor())
                    {
                        QApplication::setOverrideCursor(Qt::PointingHandCursor);
                    }
                }
                else
                {
                    QApplication::restoreOverrideCursor();
                }
            }            
        }
        return;
    }

    if (m_ignoreNextMouseEvent)
    {
        m_ignoreNextMouseEvent = false;
        return;
    }

    if (m_state == stateIdle && m_selectedShape != NULL)
    {
        QPointF mousePosScale(invTransform(QwtPlot::xBottom, event->x() - canvas()->x()), invTransform(QwtPlot::yLeft, event->y() - canvas()->y()));

        if (m_selectedShapeHitType == DrawItem::hitMove) //mouse move
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
        else if ((m_selectedShapeHitType >= DrawItem::hitResize) || (m_selectedShapeHitType <= DrawItem::hitRotation)) //rescale or rotation
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

            //todo: line + alt: keep line size constant
            if (m_selectedShape->shapeResizeOrRotate(m_selectedShapeHitType, mousePosScale - m_startMouseScaleDiff, event->modifiers()))
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
void ItomQwtPlot::closePolygon(bool aborted)
{
    QVector<ito::Shape> shapes;
    ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());

    // disable mouse tracking
    setMouseTracking(false);
    QApplication::restoreOverrideCursor();

    int shapeIdx = m_currentShapeIndices.last();
    ito::Shape thisShape = m_pShapes[shapeIdx]->getShape();
    thisShape.setUnclosed(false);
    m_pShapes[shapeIdx]->setShape(thisShape);

    for (int i = 0; i < m_currentShapeIndices.size(); i++)
    {
        if (m_pShapes.contains(m_currentShapeIndices[i]))
        {
            shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
        }
    }

    m_currentShapeIndices.clear();

    if (m_isUserInteraction)
    {
        emit p->userInteractionDone(ito::Shape::Polygon, false, shapes);
        m_isUserInteraction = false;
    }

    emit p->geometricShapeFinished(shapes, aborted);

    if (p->shapesWidget())
    {
        p->shapesWidget()->updateShapes(shapes);
    }

    m_pMultiPointPicker->setEnabled(false);
    setState(stateIdle);

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::multiPointActivated(bool on)
{
    if (!on)
    {
        switch (m_currentShapeType)
        {
            case ito::Shape::MultiPointPick:
            {
                QVector<ito::Shape> shapes;
                ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());

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
                    if (m_isUserInteraction)
                    {
                        emit p->userInteractionDone(ito::Shape::MultiPointPick, aborted, shapes);
                        m_isUserInteraction = false;
                    }

                    emit p->geometricShapeFinished(shapes, aborted);

                    /*PlotInfoMarker *pim = ((ItomQwtDObjFigure*)parent())->markerWidget();
                    if (pim)
                    {
                        pim->updateMarkers(shapes);
                    }*/
                }

                m_pMultiPointPicker->setEnabled(false);
                setState(stateIdle);
            }
            break;

            case ito::Shape::Point:
                pointPickingFinished();
            break;

            case ito::Shape::Line:
                linePickingFinished();
                break;

            case ito::Shape::Rectangle:
                rectanglePickingFinished();
            break;

            case ito::Shape::Square:
                squarePickingFinished();
            break;

            case ito::Shape::Ellipse:
                ellipsePickingFinished();
            break;

            case ito::Shape::Circle:
                circlePickingFinished();
            break;

            case ito::Shape::Polygon:
                polygonSinglePointPickingFinished();
            break;
        }
    }
}

//-------------------------------------------------------------------------------------
void ItomQwtPlot::attachAndSelectNewShape(const ito::Shape &shape)
{
    ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());

    DrawItem *newItem = new DrawItem(shape, m_shapeModificationModes, this, NULL, m_shapesLabelVisible);
    newItem->setColor(m_inverseColor0, m_inverseColor1, m_inverseColor1);
    newItem->setFillOpacity(m_geometricShapeOpacity, m_geometricShapeOpacitySelected);

    if (m_inverseColor0.isValid())
    {
        newItem->setPen(QPen(m_inverseColor0));
    }
    else
    {
        newItem->setPen(QPen(Qt::green));
    }

    /* unselect all existing shapes before adding the new one */
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
    m_selectedShapeHitType = DrawItem::hitMove;
    m_pShapes.insert(newItem->getIndex(), newItem);
    m_currentShapeIndices.append(newItem->getIndex());

    emit p->geometricShapeAdded(newItem->getIndex(), newItem->getShape());
    emit p->geometricShapeCurrentChanged(newItem->getShape());

    replot();
}

//-------------------------------------------------------------------------------------
void ItomQwtPlot::pointPickingFinished()
{
    ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());

    QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();

    bool aborted = false;
    if (polygonScale.size() == 0)
    {
        emit statusBarMessage(tr("Selection has been aborted."), 2000);
        aborted = true;
    }
    else
    {
        ito::Shape shape = ito::Shape::fromPoint(polygonScale[0]);
        attachAndSelectNewShape(shape);
    }

    // if further elements are needed reset the plot engine and go ahead else finish editing
    if (!aborted && m_numShapesToPick > 1)
    {
        m_numShapesToPick--;
        MultiPointPickerMachine *m = static_cast<MultiPointPickerMachine*>(m_pMultiPointPicker->stateMachine());

        if (m)
        {
            m->setMaxNrItems(1);
            m_pMultiPointPicker->setEnabled(true);
        }

        if (m_numShapesToPick > 1) 
        { 
            emit statusBarMessage(tr("Please draw %1 points. Esc aborts the selection.").arg(m_numShapesToPick)); 
        }
        else 
        { 
            emit statusBarMessage(tr("Please draw one point. Esc aborts the selection.")); 
        }
    }
    else
    {
        m_numShapesToPick = 0;

        QVector<ito::Shape> shapes = getAndUpdateAllCurrentShapes(true);

        m_currentShapeIndices.clear();

        if (m_isUserInteraction)
        {
            emit p->userInteractionDone(ito::Shape::Point, aborted, shapes);
            m_isUserInteraction = false;
        }

        emit statusBarMessage(tr("%1 points have been selected.").arg(shapes.size()), 2000);
        emit p->geometricShapeFinished(shapes, aborted);

        m_pMultiPointPicker->setEnabled(false);
        setState(stateIdle);
    }
}

//-------------------------------------------------------------------------------------
void ItomQwtPlot::linePickingFinished()
{
    ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());

    QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
    bool aborted = false;

    if (polygonScale.size() == 0)
    {
        emit statusBarMessage(tr("Selection has been aborted."), 2000);
        aborted = true;
    }
    else
    {
        ito::Shape shape = ito::Shape::fromLine(polygonScale[0], polygonScale[1]);
        attachAndSelectNewShape(shape);
    }

    // if further elements are needed reset the plot engine and go ahead else finish editing
    if (!aborted && m_numShapesToPick > 1)
    {
        m_pMultiPointPicker->setEnabled(true);

        m_numShapesToPick--;

        if (m_numShapesToPick > 1)
        {
            emit statusBarMessage(tr("Please draw %1 lines. Esc aborts the selection.").arg(m_numShapesToPick));
        }
        else
        {
            emit statusBarMessage(tr("Please draw one line. Esc aborts the selection."));
        }
    }
    else
    {
        m_numShapesToPick = 0;

        QVector<ito::Shape> shapes = getAndUpdateAllCurrentShapes(true);

        m_currentShapeIndices.clear();

        if (m_isUserInteraction)
        {
            emit p->userInteractionDone(ito::Shape::Point, aborted, shapes);
            m_isUserInteraction = false;
        }

        emit statusBarMessage(tr("%1 lines have been selected.").arg(shapes.size()), 2000);
        emit p->geometricShapeFinished(shapes, aborted);

        m_pMultiPointPicker->setEnabled(false);
        setState(stateIdle);
    }
}

//-------------------------------------------------------------------------------------
void ItomQwtPlot::squarePickingFinished()
{
    ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());

    QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
    bool aborted = false;

    if (polygonScale.size() == 0)
    {
        emit statusBarMessage(tr("Selection has been aborted."), 2000);
        aborted = true;
    }
    else
    {
        ito::Shape shape = ito::Shape::fromSquare(0.5 * (polygonScale[1] + polygonScale[0]), std::abs((polygonScale[1] - polygonScale[0]).x()));
        attachAndSelectNewShape(shape);
    }

    // if further elements are needed reset the plot engine and go ahead else finish editing
    if (!aborted && m_numShapesToPick > 1)
    {
        m_numShapesToPick--;
        m_pMultiPointPicker->setEnabled(true);

        if (m_numShapesToPick > 1) 
        { 
            emit statusBarMessage(tr("Please draw %1 squares. Esc aborts the selection.").arg(m_numShapesToPick)); 
        }
        else
        {
            emit statusBarMessage(tr("Please draw one square. Esc aborts the selection."));
        }
    }
    else
    {
        m_numShapesToPick = 0;

        QVector<ito::Shape> shapes = getAndUpdateAllCurrentShapes(true);

        m_currentShapeIndices.clear();

        if (m_isUserInteraction)
        {
            emit p->userInteractionDone(ito::Shape::Point, aborted, shapes);
            m_isUserInteraction = false;
        }

        emit statusBarMessage(tr("%1 squares have been selected.").arg(shapes.size()), 2000);
        emit p->geometricShapeFinished(shapes, aborted);

        m_pMultiPointPicker->setEnabled(false);
        setState(stateIdle);
    }
}

//-------------------------------------------------------------------------------------
void ItomQwtPlot::rectanglePickingFinished()
{
    ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());

    QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
    bool aborted = false;

    if (polygonScale.size() == 0)
    {
        emit statusBarMessage(tr("Selection has been aborted."), 2000);
        aborted = true;
    }
    else
    {
        ito::Shape shape = ito::Shape::fromRectangle(QRectF(polygonScale[0], polygonScale[1]));
        attachAndSelectNewShape(shape);
    }

    // if further elements are needed reset the plot engine and go ahead else finish editing
    if (!aborted && m_numShapesToPick > 1)
    {
        m_numShapesToPick--;
        m_pMultiPointPicker->setEnabled(true);

        if (m_numShapesToPick > 1) 
        { 
            emit statusBarMessage(tr("Please draw %1 rectangles. Esc aborts the selection.").arg(m_numShapesToPick)); 
        }
        else
        {
            emit statusBarMessage(tr("Please draw one rectangle. Esc aborts the selection."));
        }
    }
    else
    {
        m_numShapesToPick = 0;

        QVector<ito::Shape> shapes = getAndUpdateAllCurrentShapes(true);

        m_currentShapeIndices.clear();

        if (m_isUserInteraction)
        {
            emit p->userInteractionDone(ito::Shape::Point, aborted, shapes);
            m_isUserInteraction = false;
        }

        emit statusBarMessage(tr("%1 rectangles have been selected.").arg(shapes.size()), 2000);
        emit p->geometricShapeFinished(shapes, aborted);

        m_pMultiPointPicker->setEnabled(false);
        setState(stateIdle);
    }
}

//-------------------------------------------------------------------------------------
void ItomQwtPlot::circlePickingFinished()
{
    ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());

    QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
    bool aborted = false;

    if (polygonScale.size() == 0)
    {
        emit statusBarMessage(tr("Selection has been aborted."), 2000);
        aborted = true;
    }
    else
    {
        ito::Shape shape = ito::Shape::fromCircle(0.5 * (polygonScale[1] + polygonScale[0]), std::abs((polygonScale[1] - polygonScale[0]).x()) * 0.5);
        attachAndSelectNewShape(shape);
    }

    // if further elements are needed reset the plot engine and go ahead else finish editing
    if (!aborted && m_numShapesToPick > 1)
    {
        m_numShapesToPick--;
        m_pMultiPointPicker->setEnabled(true);

        if (m_numShapesToPick > 1)
        {
            emit statusBarMessage(tr("Please draw %1 circles. Esc aborts the selection.").arg(m_numShapesToPick));
        }
        else
        {
            emit statusBarMessage(tr("Please draw one circle. Esc aborts the selection."));
        }
    }
    else
    {
        m_numShapesToPick = 0;

        QVector<ito::Shape> shapes = getAndUpdateAllCurrentShapes(true);

        m_currentShapeIndices.clear();

        if (m_isUserInteraction)
        {
            emit p->userInteractionDone(ito::Shape::Point, aborted, shapes);
            m_isUserInteraction = false;
        }

        emit statusBarMessage(tr("%1 circles have been selected.").arg(shapes.size()), 2000);
        emit p->geometricShapeFinished(shapes, aborted);

        m_pMultiPointPicker->setEnabled(false);
        setState(stateIdle);
    }
}

//-------------------------------------------------------------------------------------
void ItomQwtPlot::ellipsePickingFinished()
{
    ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());

    QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
    bool aborted = false;

    if (polygonScale.size() == 0)
    {
        emit statusBarMessage(tr("Selection has been aborted."), 2000);
        aborted = true;
    }
    else
    {
        ito::Shape shape = ito::Shape::fromEllipse(QRectF(polygonScale[0], polygonScale[1]));
        attachAndSelectNewShape(shape);
    }

    // if further elements are needed reset the plot engine and go ahead else finish editing
    if (!aborted && m_numShapesToPick > 1)
    {
        m_numShapesToPick--;
        m_pMultiPointPicker->setEnabled(true);

        if (m_numShapesToPick > 1)
        {
            emit statusBarMessage(tr("Please draw %1 ellipses. Esc aborts the selection.").arg(m_numShapesToPick));
        }
        else
        {
            emit statusBarMessage(tr("Please draw one ellipse. Esc aborts the selection."));
        }
    }
    else
    {
        m_numShapesToPick = 0;

        QVector<ito::Shape> shapes = getAndUpdateAllCurrentShapes(true);

        m_currentShapeIndices.clear();

        if (m_isUserInteraction)
        {
            emit p->userInteractionDone(ito::Shape::Point, aborted, shapes);
            m_isUserInteraction = false;
        }

        emit statusBarMessage(tr("%1 ellipses have been selected.").arg(shapes.size()), 2000);
        emit p->geometricShapeFinished(shapes, aborted);

        m_pMultiPointPicker->setEnabled(false);
        setState(stateIdle);
    }
}

//-------------------------------------------------------------------------------------
void ItomQwtPlot::polygonSinglePointPickingFinished()
{
    QVector<ito::Shape> shapes;
    ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());

    QPolygonF polygonScale = m_pMultiPointPicker->selectionInPlotCoordinates();
    bool aborted = false;
    bool lastPolygonClosed = false;

    if (polygonScale.size() == 0)
    {
        emit statusBarMessage(tr("Selection has been aborted."), 2000);
        aborted = true;
    }
    else
    {
        int lastModifiedShapeIdx = m_currentShapeIndices.size() > 0 ? m_currentShapeIndices.last() : -1;

        if (m_pShapes.contains(lastModifiedShapeIdx) && 
            m_pShapes[lastModifiedShapeIdx]->getShape().unclosed())
        {
            const ito::Shape &thisShape = m_pShapes[lastModifiedShapeIdx]->getShape();
            QPolygonF poly = thisShape.basePoints();

            // tolerance in pixel for snapping to a geometric shape in x-direction
            double tol_x_scale = std::abs(
                invTransform(QwtPlot::xBottom, m_mouseCatchTolerancePx) 
                - invTransform(QwtPlot::xBottom, 0));

            // tolerance in pixel for snapping to a geometric shape in y-direction
            double tol_y_scale = std::abs(
                invTransform(QwtPlot::yLeft, m_mouseCatchTolerancePx) 
                - invTransform(QwtPlot::yLeft, 0)); 

            if (abs(poly[0].x() - polygonScale.back().x()) < tol_x_scale && 
                abs(poly[0].y() - polygonScale.back().y()) < tol_y_scale)
            {
                // close the current shape
                emit statusBarMessage(tr("Polygon with %1 points created.").arg(poly.size()), 2000);

                // disable mouse tracking
                setMouseTracking(false);
                QApplication::restoreOverrideCursor();

                ito::Shape thisShape = m_pShapes[lastModifiedShapeIdx]->getShape();
                thisShape.setUnclosed(false);
                m_pShapes[lastModifiedShapeIdx]->setShape(thisShape);
                lastPolygonClosed = true;
            }
            else
            {
                // modify the current unclosed shape
                poly.append(polygonScale.back());

                emit statusBarMessage(tr("%1 points in polygon currently selected. Click the start point again to finish the polygon or press Esc to abort.").arg(poly.size()), 2000);

                ito::Shape newShape = ito::Shape::fromPolygon(poly, thisShape.index());
                newShape.setUnclosed(true);
                m_pShapes[lastModifiedShapeIdx]->setShape(newShape);
                emit p->geometricShapeCurrentChanged(m_pShapes[lastModifiedShapeIdx]->getShape());
            }

            replot();
        }
        else
        {
            // enable mouse tracking to change curser when passing over starting / end point
            setMouseTracking(true);

            emit statusBarMessage(tr("1 point in polygon currently selected. Click the start point again to finish the polygon or press Esc to abort."), 2000);

            polygonScale.pop_back(); // remove duplicated point

            ito::Shape shape = ito::Shape::fromPolygon(polygonScale);
            shape.setUnclosed(true);

            attachAndSelectNewShape(shape);
        }
    }

    if (aborted || lastPolygonClosed)
    {
        // if further elements are needed reset the plot engine and go ahead else finish editing
        if (!aborted)
        {
            if (m_numShapesToPick > 1)
            {
                // there are further polygons to be added
                m_numShapesToPick--;
                m_pMultiPointPicker->setEnabled(true);

                if (m_numShapesToPick > 1)
                {
                    emit statusBarMessage(tr("Please draw %1 more polygons. Esc aborts the selection.").arg(m_numShapesToPick));
                }
                else
                {
                    emit statusBarMessage(tr("Please draw one further polygon. Esc aborts the selection."));
                }

                getAndUpdateAllCurrentShapes(true);
            }
            else
            {
                // this was the last polygon. Success.
                m_numShapesToPick = 0;

                QVector<ito::Shape> shapes = getAndUpdateAllCurrentShapes(true);

                m_currentShapeIndices.clear();

                if (m_isUserInteraction)
                {
                    emit p->userInteractionDone(ito::Shape::Polygon, aborted, shapes);
                    m_isUserInteraction = false;
                }

                emit p->geometricShapeFinished(shapes, aborted);

                QApplication::restoreOverrideCursor();
                setMouseTracking(false);

                m_pMultiPointPicker->setEnabled(false);

                replot();

                setState(stateIdle);
            }
        }
        else
        {
            // abort: remove the last unclosed polygon
            m_numShapesToPick = 0;

            int lastShapeIdx = m_currentShapeIndices.size() > 0 ? m_currentShapeIndices.last() : -1;

            if (lastShapeIdx >= 0 && p)
            {
                DrawItem *delItem = m_pShapes[lastShapeIdx];

                if (delItem->getShape().unclosed())
                {
                    delItem->setSelected(false);

                    if (p)
                    {
                        emit p->geometricShapeCurrentChanged(ito::Shape());
                    }

                    delItem->detach();
                    m_pShapes.remove(lastShapeIdx);
                }
            }

            QVector<ito::Shape> shapes = getAndUpdateAllCurrentShapes(true);

            m_currentShapeIndices.clear();

            if (m_isUserInteraction)
            {
                emit p->userInteractionDone(ito::Shape::Polygon, aborted, shapes);
                m_isUserInteraction = false;
            }

            emit p->geometricShapeFinished(shapes, aborted);

            QApplication::restoreOverrideCursor();
            setMouseTracking(false);

            m_pMultiPointPicker->setEnabled(false);

            replot();

            setState(stateIdle);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
// during an interactive shape session, this method collect all current shapes and returns its vector. 
// additionally, the shape toolbox is updated and (if desired) the geometricShapeChanged signal
// is emitted for the last added shape in the m_currentShapeIndices vector.
QVector<ito::Shape> ItomQwtPlot::getAndUpdateAllCurrentShapes(bool emitUpdateForLastModifiedShape)
{
    QVector<ito::Shape> shapes;
    ItomQwtDObjFigure *p = (ItomQwtDObjFigure*)(this->parent());

    if (p)
    {
        int shapeIdx;

        for (int i = 0; i < m_currentShapeIndices.size(); i++)
        {
            shapeIdx = m_currentShapeIndices[i];

            if (m_pShapes.contains(shapeIdx))
            {
                shapes.append(m_pShapes[shapeIdx]->getShape());
            }
        }

        if (emitUpdateForLastModifiedShape && shapes.size() > 0)
        {
            const ito::Shape &last = shapes.last();

            emit p->geometricShapeChanged(last.index(), last);
        }

        if (p->shapesWidget())
        {
            p->shapesWidget()->updateShapes(shapes);
        }
    }

    return shapes;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::userInteractionStart(int type, bool start, int maxNrOfPoints)
{
    ito::RetVal retVal;

    if (start)
    {
        if (maxNrOfPoints < -1 || maxNrOfPoints == 0)
        {
            retVal += ito::RetVal(ito::retError, 0, tr("The maximum number of points must be -1 (infinite) or >= 1.").toLatin1().data());
        }
        else
        {
            if (type == ito::Shape::MultiPointPick || \
                type == ito::Shape::Point || \
                type == ito::Shape::Line || \
                type == ito::Shape::Rectangle || \
                type == ito::Shape::Square || \
                type == ito::Shape::Circle || \
                type == ito::Shape::Ellipse || \
                type == ito::Shape::Polygon)
            {
                m_currentShapeType = (ito::Shape::ShapeType)type;
                m_numShapesToPick = maxNrOfPoints;
                m_isUserInteraction = true; // setting userinteraction to true, so we emit the counter part signal only if started with userinteractionstart
                setState(stateDrawShape); //this calls startOrStopDrawGeometricShape if everything is ok
                m_isUserInteraction = true; // setting userinteraction to true, so we emit the counter part signal only if started with userinteractionstart
            }
            else
            {
                retVal += ito::RetVal(ito::retError, 0, tr("Invalid type for userInteractionStart").toLatin1().data());
            }
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
                m->setMaxNrItems(m_numShapesToPick);
                m_numShapesToPick = 1;
                m_pMultiPointPicker->setEnabled(true);

                if (m->maxNrItems() > 1)
                {
                    emit statusBarMessage(tr("Please select %1 points or press Space to quit earlier. Esc aborts the selection.").arg(m->maxNrItems()));
                }
                else if (m->maxNrItems() == 1)
                {
                    emit statusBarMessage(tr("Please select 1 point or press Space to quit earlier. Esc aborts the selection."));
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

                if (m_numShapesToPick > 1)
                {
                    emit statusBarMessage(tr("Please draw %1 points. Esc aborts the selection.").arg(m_numShapesToPick));
                }
                else
                {
                    emit statusBarMessage(tr("Please draw one point. Esc aborts the selection."));
                }
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

            if (m_numShapesToPick > 1)
            {
                emit statusBarMessage(tr("Please draw %1 lines. Esc aborts the selection.").arg(m_numShapesToPick));
            }
            else
            {
                emit statusBarMessage(tr("Please draw one line. Esc aborts the selection."));
            }
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

            if (m_numShapesToPick > 1)
            {
                emit statusBarMessage(tr("Please draw %1 rectangles. Esc aborts the selection.").arg(m_numShapesToPick));
            }
            else
            {
                emit statusBarMessage(tr("Please draw one rectangle. Esc aborts the selection."));
            }
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

            if (m_numShapesToPick > 1)
            {
                emit statusBarMessage(tr("Please draw %1 squares. Esc aborts the selection.").arg(m_numShapesToPick));
            }
            else
            {
                emit statusBarMessage(tr("Please draw one square. Esc aborts the selection."));
            }
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

            if (m_numShapesToPick > 1)
            {
                emit statusBarMessage(tr("Please draw %1 ellipses. Esc aborts the selection.").arg(m_numShapesToPick));
            }
            else
            {
                emit statusBarMessage(tr("Please draw one ellipse. Esc aborts the selection."));
            }
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

            if (m_numShapesToPick > 1)
            {
                emit statusBarMessage(tr("Please draw %1 circles. Esc aborts the selection.").arg(m_numShapesToPick));
            }
            else
            {
                emit statusBarMessage(tr("Please draw one circle. Esc aborts the selection."));
            }
        }
        else if (m_currentShapeType == ito::Shape::Polygon)
        {
            if (p)
            {
                emit p->geometricShapeStartUserInput(m_currentShapeType, m_isUserInteraction);
            }

            m_pMultiPointPicker->setStateMachine(new QwtPickerDragRectMachine());
            m_pMultiPointPicker->setRubberBand(QwtPicker::PolygonRubberBand);
            m_pMultiPointPicker->setTrackerMode(QwtPicker::AlwaysOn);
            m_pMultiPointPicker->setKeepAspectRatio(false);
            m_pMultiPointPicker->setEnabled(true);

            if (m_numShapesToPick > 1)
            {
                emit statusBarMessage(tr("Please draw %1 polygon. Esc aborts the selection.").arg(m_numShapesToPick));
            }
            else
            {
                emit statusBarMessage(tr("Please draw one polygon. Esc aborts the selection."));
            }
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
        m_numShapesToPick = 1;

        QVector<ito::Shape> shapes;

        for (int i = 0; i < m_currentShapeIndices.size(); i++)
        {
            if (m_pShapes.contains(m_currentShapeIndices[i]))
            {
                shapes.append(m_pShapes[m_currentShapeIndices[i]]->getShape());
            }
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
        m_selectedShapeHitType = DrawItem::hitNone;
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
            m_selectedShapeHitType = DrawItem::hitNone;
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
            m_selectedShapeHitType = DrawItem::hitMove;
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
        m_selectedShapeHitType = DrawItem::hitNone;
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
                        case ito::Shape::Polygon:
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
                                newItem->setColor(m_inverseColor0, m_inverseColor1, m_inverseColor1);
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
                case ito::Shape::Polygon:
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
                        newItem->setColor(m_inverseColor0, m_inverseColor1, m_inverseColor1);
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
                    newItem->setColor(m_inverseColor0, m_inverseColor1, m_inverseColor1);
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
ito::RetVal ItomQwtPlot::exportCanvas(const bool copyToClipboardNotFile, const QString &fileName, const QSizeF &curSizeMm /*= QSizeF(0.0,0.0)*/, const int resolution /*= 200*/)
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
        else
        {
            //check if the file can be created and written by creating a "dummy"-file:
            QFile file(fileName);
            if (!file.open(QIODevice::WriteOnly))
            {
                return ito::RetVal::format(ito::retError, 0, tr("The file '%s' cannot be created. Check the filename and the required permissions.").toLatin1().data(),
                    fileName.toLatin1().data());
            }
            file.close();
            file.remove();
            
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


        if ((hMyParent->markerInfoWidget()  && hMyParent->markerInfoWidget()->isVisible()) ||
            (hMyParent->pickerWidget()  && hMyParent->pickerWidget()->isVisible()) ||
            (hMyParent->dObjectWidget() && hMyParent->dObjectWidget()->isVisible()) ||
            (hMyParent->shapesWidget()  && hMyParent->shapesWidget()->isVisible()))
        {
            plotInfoVisible = true;
            emit statusBarMessage(tr("Copy current view to clipboard including meta information widgets ..."));
        }
        else
        {
            emit statusBarMessage(tr("Copy current view to clipboard ..."));
        }

        
        QImage img(newSizePx, QImage::Format_ARGB32);
        QPainter painter(&img);
        painter.scale(resFactor, resFactor);
        renderer.render(this, &painter, rect());
        painter.end();
        img.setDotsPerMeterX(img.dotsPerMeterX() * resFactor); //setDotsPerMeterXY must be set after rendering
        img.setDotsPerMeterY(img.dotsPerMeterY() * resFactor);

        if (plotInfoVisible)
        {
            QList<QWidget*> widgets;
            widgets << hMyParent->dObjectWidget() << hMyParent->pickerWidget() << \
                hMyParent->markerInfoWidget()  << hMyParent->shapesWidget();
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
			imgTotal.fill(QColor(255, 255, 255, 0));
            QPainter painter2(&imgTotal);
            painter2.drawImage(0, 0, img);
            int y0 = 0;

            foreach(QPixmap pixmap, pixmaps)
            {
                painter2.drawPixmap(x0, y0, pixmap);
                y0 += pixmap.height();
            }

            painter2.end();

            img = imgTotal;
        }

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
        m_pPrinter->setPageMargins(QMarginsF(15, 15, 15, 15), QPageLayout::Unit::Millimeter);
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

    if (saveDefaultPath == "")
    {
        saveDefaultPath = QDir::currentPath();
    }

#ifndef QT_NO_FILEDIALOG
    const QList<QByteArray> imageFormats =
        QImageWriter::supportedImageFormats();

    QStringList filter;
    QString allSupportedFormats = tr("All Supported Files (");

    if (imageFormats.size() > 0)
    {
        QString imageFilter(tr("Images ("));

        for (int i = 0; i < imageFormats.size(); i++)
        {
            if (i > 0)
            {
                imageFilter += " ";
                allSupportedFormats += " ";
            }

            imageFilter += "*.";
            imageFilter += imageFormats[i];

            allSupportedFormats += "*.";
            allSupportedFormats += imageFormats[i];
        }

        imageFilter += ")";
        filter << imageFilter;
    }

    filter << tr("PDF Documents (*.pdf)");
    allSupportedFormats += " *.pdf";
#ifndef QWT_NO_SVG
#ifdef QT_SVG_LIB
    filter << tr("SVG Documents (*.svg)");
    allSupportedFormats += " *.svg";
#endif
#endif
    filter << tr("Postscript Documents (*.ps)");
    allSupportedFormats += " *.ps)";

    filter.insert(0, allSupportedFormats);

    // default fileName is the first of the all supported files list
    QString fileName = "plot.";
    fileName += imageFormats[0];


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
    int dpi = 200;

    if (ito::ITOM_API_FUNCS_GRAPH)
    {
        dpi = qBound(48, apiGetFigureSetting(parent(), "copyClipboardResolutionDpi", 200, NULL).value<int>(), 2000);
    }

    exportCanvas(true, "", size, dpi);
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::mnuSendCurrentToWorkspace()
{
    bool ok = false;
    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(parent());
    QString varname = "zoom_object";
    m_copyDisplayedAsComplex = false;
    const QString dialogTitle = "Current to workspace";
    const QString userText = "Indicate the python varible name for the currently visible object";

    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        emit statusBarMessage(tr("Could not send object to workspace, api is missing."), 4000);
    }
    
    if (p->getSource()->getType() == ito::tComplex128 || p->getSource()->getType() == ito::tComplex64) //checkBox if dataObject is complex to copy complex/not-displayed dataObject real and imaginary part. 
    {
        QDialog *dialog = new QDialog(this);
        dialog->setWindowTitle(dialogTitle);

        QVBoxLayout *layout = new QVBoxLayout(dialog);
        
        QLabel *label = new QLabel(dialog);
        label->setText(userText);
        layout->addWidget(label);
        
        QLineEdit *lineEdit = new QLineEdit(dialog);
        lineEdit->setText(varname);
        lineEdit->setFocus();
        connect(lineEdit, SIGNAL(returnPressed()), dialog, SLOT(accept()));
        layout->addWidget(lineEdit);
        
        QCheckBox *checkBox = new QCheckBox("copy as complex dataType", dialog);
        checkBox->setCheckable(true);
        checkBox->setCheckState(Qt::Checked); 
        layout->addWidget(checkBox);
        
        QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        layout->addWidget(buttonBox);
        buttonBox->button(QDialogButtonBox::Ok)->setDefault(true);
        buttonBox->button(QDialogButtonBox::Cancel)->setDefault(true);
        connect(buttonBox->button(QDialogButtonBox::Ok), SIGNAL(clicked()), dialog, SLOT(accept()));
        connect(buttonBox->button(QDialogButtonBox::Cancel), SIGNAL(clicked()), dialog, SLOT(reject()));
        
        dialog->setLayout(layout);
        dialog->setModal(true);

        int dialogCode = dialog->exec();

        if (dialogCode == QDialog::Accepted)
        {
            varname = lineEdit->text();
            m_copyDisplayedAsComplex = checkBox->isChecked();
            ok = true;
        }
        else if (dialogCode == QDialog::Rejected)
        {
            ok = false;
        }

        delete dialog;
        dialog = NULL;
    }
    else
    {
        varname = QInputDialog::getText(p, tr(dialogTitle.toLatin1().data()), tr(userText.toLatin1().data()), QLineEdit::Normal, "zoom_object", &ok);
    }    
    
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

    QString setname = id;

    if (setname == "")
    {
        setname = m_markerModel->getNextDefaultSetname();
    }

    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        emit statusBarMessage(tr("Could not plot marker, api is missing"), 4000);
        return ito::RetVal(ito::retError, 0, tr("Could not plot marker, api is missing").toLatin1().data());
    }

    ito::DataObject dObj = ito::dObjHelper::squeezeConvertCheck2DDataObject(coordinates.data(), "coordinates", ito::Range(2, 2), ito::Range::all(), retval, \
        ito::tFloat32, 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt32, ito::tInt32, ito::tFloat32, ito::tFloat64);

    if (!retval.containsError())
    {
        QwtSymbol::Style symStyle = QwtSymbol::XCross;
        QSize symSize(5, 5);
        QBrush symBrush(Qt::NoBrush);
        QPen symPen(Qt::red);

        QRegularExpression rgexp("^([b|g|r|c|m|y|k|w]?)([.|o|s|d|\\^|v|<|>|x|+|*|h]?)(\\d*)(;(\\d*))?$");

        auto match = rgexp.match(style);

        if (match.hasMatch())
        {
            char s = match.captured(1).toLatin1()[0];

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

            s = match.captured(2).toLatin1()[0];
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
            int size = match.captured(3).toInt(&ok);
            if (ok)
            {
                symSize = QSize(size, size);
            }

            size = match.captured(5).toInt(&ok);
            if (ok)
            {
                symPen.setWidth(size);
            }
        }
        else
        {
            retval += ito::RetVal(
                ito::retError, 
                0, 
                tr("The style tag does not correspond to the required format: "
                    "ColorStyleSize[;Linewidth] (Color = b,g,r,c,m,y,k,w; "
                    "Style = o,s,d,>,v,^,<,x,*,+,h)").toLatin1().data());
        }

        QwtPlotMarker *marker = NULL;
        int nrOfMarkers = dObj.getSize(1);

        const cv::Mat *mat = dObj.getCvPlaneMat(0);
        const ito::float32 *xRow = mat->ptr<const ito::float32>(0);
        const ito::float32 *yRow = mat->ptr<const ito::float32>(1);

        QPolygonF markerPolygon;
        QList<MarkerItem> markers;

        for (int i = 0; i < nrOfMarkers; ++i)
        {
			//always add coordinates to markerPolygon, such that all points, even NaN points, are displayed in the marker info toolbox
			markerPolygon.append(QPointF(xRow[i], yRow[i]));

			if (qIsFinite(xRow[i]) && qIsFinite(yRow[i]))
			{
				//... however, NaN points should not be visible on the canvas, since they are displayed in the left, upper corner (independent on zoom...)
				marker = new QwtPlotMarker();
				marker->setSymbol(new QwtSymbol(symStyle, symBrush, symPen, symSize));
				marker->setValue(xRow[i], yRow[i]);
				marker->attach(this);
                markers.append(MarkerItem(marker, plane, true));
			}
        }

        m_markerModel->addMarkers(setname, markers);

        replot();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::deleteMarkers(const QString &setname)
{
    ito::RetVal retval;

    if (setname == "")
    {
        m_markerModel->removeAllMarkers();
        replot();
    }
    else
    {
        if (!m_markerModel->removeAllMarkersFromSet(setname))
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("No marker with id '%1' found.").arg(setname).toLatin1().data());
        }
        else
        {
            replot();
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtPlot::showHideMarkers(const QString &setname, bool show)
{
    ito::RetVal retval;

    if (!m_markerModel->setVisibility(setname, show))
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("No marker with id '%1' found.").arg(setname).toLatin1().data());
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtPlot::setMarkerLabelVisible(bool visible)
{
    m_markerModel->setMarkerLabelsVisible(visible);
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
    m_markerModel->changeCurrentPlane(currentPlane);
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void** ItomQwtPlot::getItomApiFuncsPtr() const { return ito::ITOM_API_FUNCS; }
void** ItomQwtPlot::getItomApiFuncsPtrGraph() const { return ito::ITOM_API_FUNCS_GRAPH; }
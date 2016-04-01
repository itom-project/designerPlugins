/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut fuer Technische Optik (ITO),
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

#include "plotCanvas.h"
#include "common/sharedStructuresGraphics.h"
#include "DataObject/dataObjectFuncs.h"
#include "common/apiFunctionsGraphInc.h"
#include "common/apiFunctionsInc.h"

#include "dataObjRasterData.h"
#include "itom2dqwtplot.h"
#include "valuePicker2d.h"
#include "multiPointPickerMachine.h"
#include "itomPlotZoomer.h"
#include "dialog2DScale.h"

#include "plotLegends/infoWidgetDObject.h"
#include "plotLegends/infoWidgetPickers.h"

#include <qwt_color_map.h>
#include <qwt_plot_layout.h>
#include <qwt_matrix_raster_data.h>
#include <qwt_scale_widget.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_renderer.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_canvas.h>
#include <qwt_picker.h>
#include <qwt_picker_machine.h>
#include <qwt_scale_engine.h>
#include <qwt_picker_machine.h>
#include <qwt_plot_shapeitem.h>
#include <qwt_symbol.h>
#include <qwt_text_label.h>

#include <qimage.h>
#include <qpixmap.h>
#include <qdebug.h>
#include <qmessagebox.h>
#include <qwidgetaction.h>
#include <qmenu.h>


//----------------------------------------------------------------------------------------------------------------------------------
PlotCanvas::PlotCanvas(InternalData *m_pData, ItomQwtDObjFigure * parent /*= NULL*/) :
        ItomQwtPlot(parent),
        m_pLineCutPicker(NULL),
        m_pCenterMarker(NULL),
//        m_pStackCut(NULL),
        m_dObjItem(NULL),
        m_rasterData(NULL),
        m_dOverlayItem(NULL),
        m_pData(m_pData),
        m_curOverlayColorMapIndex(0),
        m_curColorMapIndex(0),
        m_pValuePicker(NULL),
        m_dObjPtr(NULL),
        m_pStackPicker(NULL),
        m_zstackCutUID(0),
        m_lineCutUID(0),
        m_pLineCutLine(NULL),
        m_isRefreshingPlot(false),
        m_unitLabelChanged(false),
        m_pActScaleSettings(NULL),
        m_pActColorPalette(NULL),
        m_pActToggleColorBar(NULL),
        m_pActValuePicker(NULL),
        m_pActLineCut(NULL),
        m_showCenterMarker(false),
        m_pMnuLineCutMode(NULL),
        m_pActStackCut(NULL),
        m_pActPlaneSelector(NULL),
        m_pCoordinates(NULL),
        m_pActCoordinates(NULL),
        m_pActCmplxSwitch(NULL),
        m_pMnuCmplxSwitch(NULL),
        m_pActCntrMarker(NULL),
        m_pOverlaySlider(NULL),
        m_pActOverlaySlider(NULL)
{
    createActions();
    setButtonStyle(buttonStyle());

    canvas()->setCursor(Qt::ArrowCursor);

    //main item on canvas -> the data object
    m_dObjItem = new DataObjItem("Data Object");
    m_dObjItem->setRenderThreadCount(0);
    //m_dObjItem->setColorMap(new QwtLinearColorMap(QColor::fromRgb(0,0,0), QColor::fromRgb(255,255,255), QwtColorMap::Indexed));
    m_rasterData = new DataObjRasterData(m_pData);
    m_dObjItem->setData(m_rasterData);
    m_dObjItem->attach(this);

    //overlayobject ttem on canvas -> the data object
    m_dOverlayItem = new DataObjItem("Overlay Object");
    m_dOverlayItem->setRenderThreadCount(0);
    m_rasterOverlayData = new DataObjRasterData(m_pData, true);
    m_dOverlayItem->setData(m_rasterOverlayData);
    m_dOverlayItem->attach(this);
    m_dOverlayItem->setAlpha(m_pData->m_alpha);
    m_dOverlayItem->setColorMap(new QwtLinearColorMap(Qt::black, Qt::white, QwtColorMap::Indexed));
    m_colorOverlayMapName = "gray";
    m_dOverlayItem->setVisible(false);

    //value picker
    m_pValuePicker = new ValuePicker2D(QwtPlot::xBottom, QwtPlot::yLeft, canvas(), m_rasterData, m_rasterOverlayData);
    m_pValuePicker->setEnabled(false);
    m_pValuePicker->setTrackerMode(QwtPicker::AlwaysOn);

    //zStack cut picker
    m_pStackPicker = new ItomPlotPicker(canvas());
    m_pStackPicker->setStateMachine(new QwtPickerClickPointMachine());
    m_pStackPicker->setTrackerMode(QwtPicker::AlwaysOn);
    m_pStackPicker->setRubberBand(QwtPicker::CrossRubberBand);
    m_pStackPicker->setEnabled(false);

    //disable key movements for the picker (the marker will be moved by the key-event of this widget)
    m_pStackPicker->setKeyPattern(QwtEventPattern::KeyLeft, 0);
    m_pStackPicker->setKeyPattern(QwtEventPattern::KeyRight, 0);
    m_pStackPicker->setKeyPattern(QwtEventPattern::KeyUp, 0);
    m_pStackPicker->setKeyPattern(QwtEventPattern::KeyDown, 0);
    connect(m_pStackPicker, SIGNAL(appended(const QPoint&)), this, SLOT(zStackCutTrackerMoved(const QPoint&)));
    connect(m_pStackPicker, SIGNAL(moved(const QPoint&)), this, SLOT(zStackCutTrackerMoved(const QPoint&)));

    //marker for zstack cut
    m_pStackCutMarker = new QwtPlotMarker();
    m_pStackCutMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green),3),  QSize(11,11)));
    m_pStackCutMarker->attach(this);
    m_pStackCutMarker->setVisible(false);

    //marker for the camera center
    m_pCenterMarker = new QwtPlotMarker();
    m_pCenterMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(Qt::green), QPen(QBrush(Qt::green), 1),  QSize(11,11)));
    m_pCenterMarker->attach(this);
    m_pCenterMarker->setVisible(m_showCenterMarker);
    m_pCenterMarker->setValue(QPointF(0.0, 0.0));
    m_pCenterMarker->setAxes(QwtPlot::xBottom, QwtPlot::yLeft);
    m_pCenterMarker->setSpacing(0);

    //picker for line picking
    m_pLineCutPicker = new QwtPlotPicker(QwtPlot::xBottom, QwtPlot::yLeft, QwtPicker::CrossRubberBand, QwtPicker::AlwaysOn, canvas());
    m_pLineCutPicker->setEnabled(false);
    m_pLineCutPicker->setStateMachine(new QwtPickerDragPointMachine);
    m_pLineCutPicker->setRubberBandPen(QPen(Qt::green));
    m_pLineCutPicker->setTrackerPen(QPen(Qt::green));
    //disable key movements for the picker (the marker will be moved by the key-event of this widget)
    m_pLineCutPicker->setKeyPattern(QwtEventPattern::KeyLeft, 0);
    m_pLineCutPicker->setKeyPattern(QwtEventPattern::KeyRight, 0);
    m_pLineCutPicker->setKeyPattern(QwtEventPattern::KeyUp, 0);
    m_pLineCutPicker->setKeyPattern(QwtEventPattern::KeyDown, 0);
    connect(m_pLineCutPicker, SIGNAL(moved(const QPoint&)), SLOT(lineCutMoved(const QPoint&)));
    connect(m_pLineCutPicker, SIGNAL(appended(const QPoint&)), SLOT(lineCutAppended(const QPoint&)));

    //line for line picking
    m_pLineCutLine = new QwtPlotCurve();
    m_pLineCutLine->attach(this);
    m_pLineCutLine->setVisible(false);

    //right axis (color bar)
    QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
    rightAxis->setColorBarEnabled(true);
    rightAxis->setColorBarWidth(15);
    setAxisScale(QwtPlot::yRight, 0, 1.0);
    enableAxis(QwtPlot::yRight, m_pData->m_colorBarVisible);
    axisWidget(QwtPlot::yRight)->setLayoutFlag(QwtScaleWidget::TitleInverted, false); //let the label be in the same direction than on the left side

    //rightAxis->scaleDraw()->setLength(20);
    rightAxis->scaleDraw()->enableComponent(QwtAbstractScaleDraw::Backbone,false); //disable backbone
    rightAxis->setMargin(10);                       //distance colorbar <-> canvas
    rightAxis->setSpacing(8);                       //distance tick labels <-> axis label && color bar -> ticks
    rightAxis->scaleDraw()->setSpacing(6);          //distance tick labels <-> ticks
    rightAxis->setContentsMargins(0,0,0,0);         //top and bottom offset

    configRescaler();

    QWidget *guiParent = parent;
    if (!guiParent) guiParent = this;


    //initialize actions
    QToolBar *mainTb = new QToolBar(tr("plotting tools"), guiParent);
    mainTb->setObjectName("mainToolBar");
    m_toolbars.append(mainTb);

    mainTb->addAction(m_pActSave);
    mainTb->addSeparator();
    mainTb->addAction(m_pActHome);
    mainTb->addAction(m_pActPan);
    mainTb->addAction(m_pActZoom);
    mainTb->addAction(m_pActAspectRatio);
    mainTb->addAction(m_pActOverlaySlider);
    mainTb->addSeparator();
    mainTb->addAction(m_pActScaleSettings);
    mainTb->addAction(m_pActToggleColorBar);
    mainTb->addAction(m_pActColorPalette);
    mainTb->addSeparator();
    mainTb->addAction(m_pActValuePicker);
    mainTb->addAction(m_pActCntrMarker);
    mainTb->addAction(m_pActLineCut);
    mainTb->addAction(m_pActStackCut);
    mainTb->addSeparator();
    mainTb->addAction(m_pActShapeType);
    mainTb->addAction(m_pActClearShapes);
    mainTb->addSeparator();
    mainTb->addAction(m_pActPlaneSelector);
    mainTb->addAction(m_pActCmplxSwitch);
    mainTb->addAction(m_pActCoordinates);

    QMenu *menuFile = new QMenu(tr("File"), guiParent);
    menuFile->addAction(m_pActSave);
    menuFile->addSeparator();
    menuFile->addAction(m_pActCopyClipboard);
    menuFile->addAction(m_pActSendCurrentToWorkspace);
    m_menus.append(menuFile);

    QMenu *menuView = new QMenu(tr("View"), guiParent);
    menuView->addAction(m_pActHome);
    menuView->addAction(m_pActPan);
    menuView->addAction(m_pActZoom);
    menuView->addAction(m_pActAspectRatio);
    menuView->addSeparator();
    menuView->addAction(m_pActToggleColorBar);
    menuView->addAction(m_pActColorPalette);
    menuView->addSeparator();
    menuView->addAction(m_pActScaleSettings);
    menuView->addSeparator();
    menuView->addAction(m_pActCmplxSwitch);
    menuView->addSeparator();
    menuView->addAction(m_pActProperties);
    m_menus.append(menuView);

    QMenu *menuTools = new QMenu(tr("Tools"), guiParent);
    menuTools->addAction(m_pActValuePicker);
    menuTools->addAction(m_pActCntrMarker);
    menuTools->addAction(m_pActLineCut);
    menuTools->addAction(m_pActStackCut);
    menuTools->addSeparator();
    menuTools->addMenu(m_pMenuShapeType);
    menuTools->addAction(m_pActClearShapes);
    m_menus.append(menuTools);

    m_pContextMenu = new QMenu(QObject::tr("plot2D"), guiParent);
    m_pContextMenu->addAction(m_pActSave);
    m_pContextMenu->addAction(m_pActCopyClipboard);
    m_pContextMenu->addAction(m_pActSendCurrentToWorkspace);
    m_pContextMenu->addSeparator();
    m_pContextMenu->addAction(m_pActHome);
    m_pContextMenu->addAction(m_pActPan);
    m_pContextMenu->addAction(m_pActZoom);
    m_pContextMenu->addAction(m_pActAspectRatio);
    m_pContextMenu->addSeparator();
    m_pContextMenu->addAction(m_pActValuePicker);
    m_pContextMenu->addAction(m_pActCntrMarker);
    m_pContextMenu->addSeparator();
    m_pContextMenu->addAction(mainTb->toggleViewAction());
}

//----------------------------------------------------------------------------------------------------------------------------------
PlotCanvas::~PlotCanvas()
{
    m_pLineCutLine->detach();
    delete m_pLineCutLine;
    m_pLineCutLine = NULL;

    m_pStackCutMarker->detach();
    delete m_pStackCutMarker;
    m_pStackCutMarker = NULL;

    m_pCenterMarker->detach();
    delete m_pCenterMarker;
    m_pCenterMarker = NULL;

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotCanvas::init()
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
void PlotCanvas::refreshStyles()
{
    ItomQwtPlot::loadStyles();

    QPen rubberBandPen = QPen(QBrush(Qt::red), 1, Qt::DashLine);
    QPen trackerPen = QPen(QBrush(Qt::red), 2);
    QFont trackerFont = QFont("Verdana", 10);
    QBrush trackerBg = QBrush(QColor(255, 255, 255, 155), Qt::SolidPattern);
    QPen selectionPen = QPen(QBrush(Qt::gray), 2, Qt::SolidLine);

    QFont titleFont = QFont("Helvetica", 12);
    QFont labelFont = QFont("Helvetica", 12);
    labelFont.setItalic(false);
    QFont axisFont = QFont("Helvetica", 10);

    QSize centerMarkerSize = QSize(10, 10);
    QPen centerMarkerPen = QPen(QBrush(Qt::red), 1);
    int buttonSet = 0;

    if(ito::ITOM_API_FUNCS_GRAPH)
    {
        rubberBandPen = apiGetFigureSetting(parent(), "zoomRubberBandPen", rubberBandPen, NULL).value<QPen>();
        trackerPen = apiGetFigureSetting(parent(), "trackerPen", trackerPen, NULL).value<QPen>();
        trackerFont = apiGetFigureSetting(parent(), "trackerFont", trackerFont, NULL).value<QFont>();
        trackerBg = apiGetFigureSetting(parent(), "trackerBackground", trackerBg, NULL).value<QBrush>();
        selectionPen = apiGetFigureSetting(parent(), "selectionPen", selectionPen, NULL).value<QPen>();

        buttonSet = apiGetFigureSetting(parent(), "buttonSet", buttonSet, NULL).value<int>();

        titleFont = apiGetFigureSetting(parent(), "titleFont", titleFont, NULL).value<QFont>();
        labelFont = apiGetFigureSetting(parent(), "labelFont", labelFont, NULL).value<QFont>();
        labelFont.setItalic(false);
        axisFont = apiGetFigureSetting(parent(), "axisFont", axisFont, NULL).value<QFont>();

        centerMarkerSize = apiGetFigureSetting(parent(), "centerMarkerSize", centerMarkerSize, NULL).value<QSize>();
        centerMarkerPen = apiGetFigureSetting(parent(), "centerMarkerPen", centerMarkerPen, NULL).value<QPen>();
    }

    if (inverseColor1().isValid())
    {
        rubberBandPen.setColor(inverseColor1());
    }

    if (inverseColor0().isValid())
    {
        selectionPen.setColor(inverseColor0());
        trackerPen.setColor(inverseColor0());
        centerMarkerPen.setColor(inverseColor0());
    }

    m_pValuePicker->setTrackerFont(trackerFont);
    m_pValuePicker->setTrackerPen(trackerPen);
    m_pValuePicker->setBackgroundFillBrush(trackerBg);

    m_pLineCutPicker->setRubberBandPen(rubberBandPen);
    m_pLineCutPicker->setTrackerPen(trackerPen);
    m_pLineCutLine->setPen(selectionPen);

    m_pStackPicker->setTrackerFont(trackerFont);
    m_pStackPicker->setTrackerPen(trackerPen);
    m_pStackPicker->setBackgroundFillBrush(trackerBg);

    m_pStackCutMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross, QBrush(inverseColor0()), QPen(QBrush(inverseColor1()), 3), QSize(7, 7)));

    m_pCenterMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(/*m_inverseColor0*/), centerMarkerPen,  centerMarkerSize));
    
    title().setFont(titleFont);
    titleLabel()->setFont(titleFont);

    axisTitle(QwtPlot::xBottom).setFont(axisFont);
    axisTitle(QwtPlot::yLeft).setFont(axisFont);
    axisTitle(QwtPlot::yRight).setFont(axisFont);
    setAxisFont(QwtPlot::xBottom, axisFont);
    setAxisFont(QwtPlot::yLeft, axisFont);
    setAxisFont(QwtPlot::yRight, axisFont);

    QwtText t = axisWidget(QwtPlot::xBottom)->title();
    t.setFont(labelFont);
    axisWidget(QwtPlot::xBottom)->setTitle(t);

    t = axisWidget(QwtPlot::yLeft)->title();
    t.setFont(labelFont);
    axisWidget(QwtPlot::yLeft)->setTitle(t);

    t = axisWidget(QwtPlot::yRight)->title();
    t.setFont(labelFont);
    axisWidget(QwtPlot::yRight)->setTitle(t);

    //axisWidget(QwtPlot::yRight)->setLabelRotation(-90.0); //this rotates the tick values for the color bar ;)
    //axisScaleDraw(QwtPlot::yRight)->setLabelRotation(90); //this also ;)

    setButtonStyle(buttonSet);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::createActions()
{
    QAction *a = NULL;
    QWidget *p = qobject_cast<QWidget*>(parent());

    //m_actScaleSetting
    m_pActScaleSettings = a = new QAction(tr("Scale Settings..."), p);
    a->setObjectName("actScaleSetting");
    a->setToolTip(tr("Set the ranges and offsets of this view"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuScaleSettings()));

    //m_actPalette
    m_pActColorPalette = a = new QAction(tr("Palette"), p);
    a->setObjectName("actColorPalette");
    a->setToolTip(tr("Switch between color palettes"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuColorPalette()));

    //m_actCmplxSwitch
    m_pActCmplxSwitch = new QAction(tr("Complex Switch"), p);
    m_pMnuCmplxSwitch = new QMenu(tr("Complex Switch"), p);
    m_pActCmplxSwitch->setMenu(m_pMnuCmplxSwitch);
    a = m_pMnuCmplxSwitch->addAction(tr("Imag"));
    a->setData(ItomQwtPlotEnums::CmplxImag);
    a = m_pMnuCmplxSwitch->addAction(tr("Real"));
    a->setData(ItomQwtPlotEnums::CmplxReal);
    a = m_pMnuCmplxSwitch->addAction(tr("Abs"));
    a->setData(ItomQwtPlotEnums::CmplxAbs);
    m_pMnuCmplxSwitch->setDefaultAction(a);
    a = m_pMnuCmplxSwitch->addAction(tr("Pha"));
    a->setData(ItomQwtPlotEnums::CmplxArg);
    m_pActCmplxSwitch->setVisible(false);
    connect(m_pMnuCmplxSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuCmplxSwitch(QAction*)));

    //m_actToggleColorBar
    m_pActToggleColorBar = a = new QAction(tr("Show Colorbar"), p);
    a->setCheckable(true);
    a->setObjectName("actShowColorBar");
    a->setToolTip(tr("Toggle visibility of the color bar on the right side"));
    connect(a, SIGNAL(toggled(bool)), this, SLOT(mnuToggleColorBar(bool)));

    //m_actMarker
    m_pActValuePicker = a = new QAction(tr("Marker"), p);
    a->setToolTip(tr("Show current coordinates at mouse cursor"));
    a->setObjectName("actValuePicker");
    a->setCheckable(true);
    a->setChecked(false);
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuValuePicker(bool)));

    //m_actLineCut
    m_pActLineCut = a = new QAction(tr("Linecut"), p);
    a->setCheckable(true);
    a->setObjectName("actLineCut");
    a->setToolTip(tr("Show a 1D line cut (in-plane)"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuLineCut(bool)));

    //m_pActLineCutMode
    m_pMnuLineCutMode = new QMenu(tr("Linecut Mode"), p);
    m_pActLineCut->setMenu(m_pMnuLineCutMode);
    a = m_pMnuLineCutMode->addAction(tr("min & max"));
    a->setToolTip(tr("line cut through global minimum and maximum value"));
    a->setData(0);
    a->setCheckable(true);

    a = m_pMnuLineCutMode->addAction(tr("- & min"));
    a->setToolTip(tr("horizontal line cut through global minimum value"));
    a->setData(1);
    a->setCheckable(true);

    a = m_pMnuLineCutMode->addAction(tr("- & max"));
    a->setToolTip(tr("horizontal line cut through global maximum value"));
    a->setData(2);
    a->setCheckable(true);

    a = m_pMnuLineCutMode->addAction(tr("| & min"));
    a->setToolTip(tr("vertical line cut through global minimum value"));
    a->setData(3);
    a->setCheckable(true);

    a = m_pMnuLineCutMode->addAction(tr("| & max"));
    a->setToolTip(tr("vertical line cut through global maximum value"));
    a->setData(4);
    a->setCheckable(true);
    connect(m_pMnuLineCutMode, SIGNAL(triggered(QAction*)), this, SLOT(mnuLineCutMode(QAction*)));

    //m_pOverlaySlider
    m_pOverlaySlider = new QSlider(Qt::Horizontal, p);
    m_pOverlaySlider->setMinimum(0);
    m_pOverlaySlider->setMaximum(255);
    m_pOverlaySlider->setValue(0);
    m_pOverlaySlider->setWhatsThis(tr("Control alpha-value of overlay image"));
    m_pOverlaySlider->setToolTip(tr("Set alpha for overlay"));
    m_pActOverlaySlider = new QWidgetAction(p);
    m_pActOverlaySlider->setDefaultWidget(m_pOverlaySlider);
    m_pActOverlaySlider->setObjectName("overlaySlider");
    m_pActOverlaySlider->setVisible(false);
    connect(m_pOverlaySlider, SIGNAL(valueChanged(int)), this, SLOT(mnuOverlaySliderChanged(int)));

    //m_actStackCut
    m_pActStackCut = a = new QAction(tr("Slice in z-direction"), p);
    a->setObjectName("actStackCut");
    a->setToolTip(tr("Show a slice through z-stack"));
    a->setCheckable(true);
    a->setVisible(false);
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuStackCut(bool)));

    //m_pActCntrMarker
    m_pActCntrMarker = a = new QAction(tr("Center marker"), p);
    a->setObjectName("actCenterMarker");
    a->setToolTip(tr("Display a cross at data object's center"));
    a->setCheckable(true);
    a->setVisible(true);
    a->setChecked(false);
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuCenterMarker(bool)));

    m_pCoordinates = new QLabel("[0.0; 0.0]\n[0.0; 0.0]", p);
    m_pCoordinates->setAlignment(Qt::AlignRight | Qt::AlignTop);
    m_pCoordinates->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    m_pCoordinates->setObjectName("lblCoordinates");

    m_pActCoordinates = new QWidgetAction(p);
    m_pActCoordinates->setDefaultWidget(m_pCoordinates);
    m_pActCoordinates->setVisible(false);

    QSpinBox *planeSelector = new QSpinBox(p);
    planeSelector->setMinimum(0);
    planeSelector->setMaximum(0);
    planeSelector->setValue(0);
    planeSelector->setKeyboardTracking(false);
    planeSelector->setToolTip(tr("Select image plane"));
    m_pActPlaneSelector = new QWidgetAction(this);
    m_pActPlaneSelector->setDefaultWidget(planeSelector);
    m_pActPlaneSelector->setObjectName("planeSelector");
    m_pActPlaneSelector->setVisible(false);
    connect(planeSelector, SIGNAL(valueChanged(int)), this, SLOT(mnuPlaneSelector(int)));
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setButtonStyle(int style)
{
    ItomQwtPlot::setButtonStyle(style);

    if (style == 0)
    {
        m_pActScaleSettings->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/autoscal.png"));
        m_pActColorPalette->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorPalette.png"));
        m_pActCntrMarker->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/markerCntr.png"));
        m_pActStackCut->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/zStack.png"));
        m_pActLineCut->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"));
        m_pActToggleColorBar->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorbar.png"));
        m_pActValuePicker->setIcon(QIcon(":/itomDesignerPlugins/general/icons/crosshairs.png"));

        int cmplxIdx = m_pMnuCmplxSwitch->defaultAction()->data().toInt();
        if (cmplxIdx == ItomQwtPlotEnums::CmplxImag)
        {
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReImag.png"));
        }
        else if (cmplxIdx == ItomQwtPlotEnums::CmplxReal)
        {
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReReal.png"));
        }
        else if (cmplxIdx == ItomQwtPlotEnums::CmplxArg)
        {
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImRePhase.png"));
        }
        else
        {
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReAbs.png"));
        }
    }
    else
    {
        m_pActScaleSettings->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/autoscal_lt.png"));
        m_pActColorPalette->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/colorPalette_lt.png"));
        m_pActCntrMarker->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/markerCntr_lt.png"));
        m_pActStackCut->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/zStack_lt.png"));
        m_pActLineCut->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/pntline_lt.png"));
        m_pActToggleColorBar->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/colorbar_lt.png"));
        m_pActValuePicker->setIcon(QIcon(":/itomDesignerPlugins/general_lt/icons/marker_lt.png"));

        int cmplxIdx = m_pMnuCmplxSwitch->defaultAction()->data().toInt();
        if (cmplxIdx == ItomQwtPlotEnums::CmplxImag)
        {
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex_lt/icons/ImReImag_lt.png"));
        }
        else if (cmplxIdx == ItomQwtPlotEnums::CmplxReal)
        {
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex_lt/icons/ImReReal_lt.png"));
        }
        else if (cmplxIdx == ItomQwtPlotEnums::CmplxArg)
        {
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex_lt/icons/ImRePhase_lt.png"));
        }
        else
        {
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex_lt/icons/ImReAbs_lt.png"));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::refreshPlot(const ito::DataObject *dObj, int plane /*= -1*/)
{
    if (m_isRefreshingPlot)
    {
        return;
    }

    m_isRefreshingPlot = true;

    ito::uint8 updateState = 0; //changeNo (0): nothing changed, changeAppearance (1): appearance changed (yAxisFlipped, cmplxFlag, plane...), changeData (2): data changed (dimensions, sizes, other data object...)

    m_dObjPtr = dObj;

    //QString valueLabel, axisLabel, title;
    if (dObj)
    {
        int dims = dObj->getDims();
        int width = dims > 0 ? dObj->getSize(dims - 1) : 0;
        int height = dims > 1 ? dObj->getSize(dims - 2) : 1;

        updateState = m_rasterData->updateDataObject(dObj, plane);

        if (updateState & changeData || m_unitLabelChanged)
        {
            m_unitLabelChanged = false;
            bool valid;
            ito::DataObjectTagType tag;
            std::string descr, unit;
            tag = dObj->getTag("title", valid);
            m_pData->m_titleDObj = valid ? QString::fromLatin1(tag.getVal_ToString().data()) : "";
            m_pData->m_dataType = (ito::tDataType)dObj->getType();

            descr = dObj->getValueDescription();
            unit = dObj->getValueUnit();

            if (unit != "")
            {
                switch (unitLabelStyle())
                {
                    case ito::AbstractFigure::UnitLabelSlash:
                        descr.append(descr != "" ? " / " + unit : unit);
                        break;
                    case ito::AbstractFigure::UnitLabelKeywordIn:
                        descr.append(descr != "" ? " in " + unit : unit);
                        break;
                    case ito::AbstractFigure::UnitLabelSquareBrackets:
                        descr.append(" [" + unit + "]");
                        break;
                }
                
            }
            m_pData->m_valueLabelDObj = QString::fromLatin1(descr.data());

            if (dims >= 2)
            {
                descr = dObj->getAxisDescription(dims-1, valid);
                if (!valid) descr = "";
                unit = dObj->getAxisUnit(dims-1,valid);
                if (!valid) unit = "";

                if (unit != "")
                {
                    switch (unitLabelStyle())
                    {
                        case ito::AbstractFigure::UnitLabelSlash:
                            descr.append(descr != "" ? " / " + unit : unit);
                            break;
                        case ito::AbstractFigure::UnitLabelKeywordIn:
                            descr.append(descr != "" ? " in " + unit : unit);
                            break;
                        case ito::AbstractFigure::UnitLabelSquareBrackets:
                            descr.append(" [" + unit + "]");
                            break;
                    }
                }

                m_pData->m_xaxisLabelDObj = QString::fromLatin1(descr.data());

                descr = dObj->getAxisDescription(dims-2, valid);
                if (!valid) descr = "";
                unit = dObj->getAxisUnit(dims-2,valid);
                if (!valid) unit = "";

                if (unit != "")
                {
                    switch (unitLabelStyle())
                    {
                        case ito::AbstractFigure::UnitLabelSlash:
                            descr.append(descr != "" ? " / " + unit : unit);
                            break;
                        case ito::AbstractFigure::UnitLabelKeywordIn:
                            descr.append(descr != "" ? " in " + unit : unit);
                            break;
                        case ito::AbstractFigure::UnitLabelSquareBrackets:
                            descr.append(" [" + unit + "]");
                            break;
                    }
                }

                m_pData->m_yaxisLabelDObj = QString::fromLatin1(descr.data());
            }
            else
            {
                m_pData->m_xaxisLabelDObj = "";
                m_pData->m_yaxisLabelDObj = "";
            }

            m_pCenterMarker->setXValue(m_dObjPtr->getPixToPhys(dims-1, (width - 1)/ 2.0, valid));
            m_pCenterMarker->setYValue(m_dObjPtr->getPixToPhys(dims-2, (height - 1) / 2.0, valid));
        }
    }

	if (m_dObjPtr && ((Itom2dQwtPlot*)(this->parent()))->dObjectWidget())
	{
		PlotInfoDObject* infoWidget = (((Itom2dQwtPlot*)(this->parent()))->dObjectWidget());
		infoWidget->updateInfoHeader(
			QString("2D Object Plot"),
			m_dObjPtr->getType(),
			m_dObjPtr->getDims(),
			m_dObjPtr->getSize());

		if (infoWidget->useDetailInfo())
		{
			double min = 0.0, max = 0.0, mean = 0.0, dev = 0.0;
			ito::uint32 minLoc[3];
			ito::uint32 maxLoc[3];
			ito::dObjHelper::minMaxValue(m_dObjPtr, min, minLoc, max, maxLoc, true);
			ito::dObjHelper::devValue(m_dObjPtr, 0, mean, dev, true);
			infoWidget->updateInfoDetail(min, max, mean, dev);
		}
	}

    updateLabels();

    if (updateState != changeNo)
    {
        Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
        if (p)
        {
            setColorDataTypeRepresentation(dObj->getType() == ito::tRGBA32);

            if (dObj->getType() == ito::tRGBA32) //coloured objects have no color bar, therefore the value axis cropping is disabled
            {
                m_pData->m_valueScaleAuto = false;
                m_pData->m_valueMin = std::numeric_limits<ito::uint8>::min();
                m_pData->m_valueMax = std::numeric_limits<ito::uint8>::max();
            }

            if (dObj->getType() == ito::tComplex128 || dObj->getType() == ito::tComplex64)
            {
                m_pActCmplxSwitch->setVisible(true);
            }
            else
            {
                m_pActCmplxSwitch->setVisible(false);
            }

            int maxPlane = 0;
            if (dObj->getDims() > 2)
            {
                maxPlane = dObj->calcNumMats() - 1;
            }

            p->setPlaneRange(0, maxPlane);
        }

        updateScaleValues(false, updateState & changeData); //no replot here
        updateAxes();


        //set the base view for the zoomer (click on 'house' symbol) to the current representation (only if data changed)
        if (updateState & changeData)
        {
            zoomer()->setZoomBase(false); //do not replot in order to not destroy the recently set scale values, a rescale is executed at the end though
        }
        else
        {
            zoomer()->rescale(false);
        }

    }
    else
    {
        replot();
    }

    m_isRefreshingPlot = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setColorDataTypeRepresentation(bool colorOn)
{
    if (colorOn)
    {
        setColorBarVisible(false);
        m_pActColorPalette->setVisible(false);
        m_pActToggleColorBar->setVisible(false);
    }
    else
    {
        m_pActColorPalette->setVisible(true);
        m_pActToggleColorBar->setVisible(true);
        setColorBarVisible(m_pActToggleColorBar->isChecked());
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::changePlane(int plane)
{
    refreshPlot(m_dObjPtr, plane);
    changeVisibleMarkers(plane);
}

//----------------------------------------------------------------------------------------------------------------------------------
int PlotCanvas::getCurrentPlane()
{
    if (m_rasterData)
    {
        return m_rasterData->getCurrentPlane();
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::internalDataUpdated()
{
    refreshPlot(m_dObjPtr, -1);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotCanvas::setColorMap(QString colormap /*= "__next__"*/)
{
    QwtLinearColorMap *colorMap = NULL;
    QwtLinearColorMap *colorBarMap = NULL;
    ito::ItomPalette newPalette;
    ito::RetVal retval(ito::retOk);
    int numPalettes = 1;

    if(!ito::ITOM_API_FUNCS_GRAPH)
    {
        emit statusBarMessage(tr("Could not change color bar, api is missing"), 4000);
        return false;
    }

    retval += apiPaletteGetNumberOfColorBars(numPalettes);

    if (numPalettes == 0 || retval.containsError())
    {
        emit statusBarMessage(tr("No color maps defined."), 4000);
        return false;
    }

    if (colormap == "__next__")
    {
        m_curColorMapIndex++;
        m_curColorMapIndex %= numPalettes; //map index to [0,numPalettes)
        retval += apiPaletteGetColorBarIdx(m_curColorMapIndex, newPalette);
    }
    else if (colormap == "__first__")
    {
        m_curColorMapIndex = 0;
        retval += apiPaletteGetColorBarIdx(m_curColorMapIndex, newPalette);
    }
    else
    {
        retval += apiPaletteGetColorBarName(colormap, newPalette);
    }

    if (retval.containsError() && retval.errorMessage() != NULL)
    {
        emit statusBarMessage(QString("%1").arg(QLatin1String(retval.errorMessage())), 4000);
        return false;
    }
    else if (retval.containsError())
    {
        emit statusBarMessage("error when loading color map", 4000);
        return false;
    }

    int totalStops = newPalette.colorStops.size();

    if (totalStops < 2)
    {
        emit statusBarMessage(tr("Selected color map has less than two points."), 4000);
        return false;
    }

    m_colorMapName = newPalette.name;

/*
    if (newPalette.getPos(newPalette.getSize() - 1) == newPalette.getPos(newPalette.getSize() - 2))  // BuxFix - For Gray-Marked
    {
        colorMap = new QwtLinearColorMap(newPalette.getColorFirst(), newPalette.getColor(newPalette.getSize() - 2), QwtColorMap::Indexed);
        colorBarMap = new QwtLinearColorMap(newPalette.getColorFirst(), newPalette.getColor(newPalette.getSize() - 2), QwtColorMap::Indexed);
        if (newPalette.getSize() > 2)
        {
            for (int i = 1; i < newPalette.getSize() - 2; i++)
            {
                colorMap->addColorStop(newPalette.getPos(i), newPalette.getColor(i));
                colorBarMap->addColorStop(newPalette.getPos(i), newPalette.getColor(i));
            }
            colorMap->addColorStop(newPalette.getPos(newPalette.getSize() - 1), newPalette.getColor(newPalette.getSize() - 1));
            colorBarMap->addColorStop(newPalette.getPos(newPalette.getSize() - 1), newPalette.getColor(newPalette.getSize() - 1));
        }
    }
    else
    {
        colorMap = new QwtLinearColorMap(newPalette.getColorFirst(), newPalette.getColorLast(), QwtColorMap::Indexed);
        colorBarMap = new QwtLinearColorMap(newPalette.getColorFirst(), newPalette.getColorLast(), QwtColorMap::Indexed);
        if (newPalette.getSize() > 2)
        {
            for (int i = 1; i < newPalette.getSize() - 1; i++)
            {
                colorMap->addColorStop(newPalette.getPos(i), newPalette.getColor(i));
                colorBarMap->addColorStop(newPalette.getPos(i), newPalette.getColor(i));
            }
        }
    }
*/

    //if (newPalette.inverseColorOne.isValid() /* && newPalette.inverseColorTwo.isValid() */)
    //{
    //    m_pLineCutPicker->setRubberBandPen(QPen(newPalette.inverseColorOne));
    //    m_pLineCutPicker->setTrackerPen(QPen(newPalette.inverseColorOne));
    //    m_pLineCutLine->setPen(newPalette.inverseColorOne);
    //}
    //else
    //{
    //    m_pLineCutPicker->setRubberBandPen(QPen(Qt::gray));
    //    m_pLineCutPicker->setTrackerPen(QPen(Qt::gray));
    //    m_pLineCutLine->setPen(Qt::gray);
    //}

    setInverseColors(newPalette.inverseColorOne, newPalette.inverseColorTwo);
    refreshStyles();

    if (newPalette.colorStops[totalStops - 1].first == newPalette.colorStops[totalStops - 2].first)  // BuxFix - For Gray-Marked
    {
        colorMap    = new QwtLinearColorMap(newPalette.colorStops[0].second, newPalette.colorStops[totalStops - 2].second, QwtColorMap::Indexed);
        colorBarMap = new QwtLinearColorMap(newPalette.colorStops[0].second, newPalette.colorStops[totalStops - 2].second, QwtColorMap::Indexed);
        if (totalStops > 2)
        {
            for (int i = 1; i < totalStops - 2; i++)
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
        if (totalStops > 2)
        {
            for (int i = 1; i < totalStops - 1; i++)
            {
                colorMap->addColorStop(newPalette.colorStops[i].first, newPalette.colorStops[i].second);
                colorBarMap->addColorStop(newPalette.colorStops[i].first, newPalette.colorStops[i].second);
            }
        }
    }

    if (colorMap && colorBarMap)
    {
        if (m_rasterData)
        {
            QwtInterval interval = m_rasterData->interval(Qt::ZAxis);
            /*setAxisScale(QwtPlot::yRight, interval.minValue(), interval.maxValue());

            axisScale(QwtPlot::yRight, m_pData->m_valueMin, m_pData->m_valueMax);
*/
            axisWidget(QwtPlot::yRight)->setColorMap(interval, colorBarMap);
        }
        else
        {
            axisWidget(QwtPlot::yRight)->setColorMap(QwtInterval(0,1.0), colorBarMap);
        }

        m_dObjItem->setColorMap(colorMap);
    }
    else
    {
        delete colorMap;
        delete colorBarMap;
    }

    replot();
    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotCanvas::setOverlayColorMap(QString colormap /*= "__next__"*/)
{
    QwtLinearColorMap *colorMap = NULL;
    ito::ItomPalette newPalette;
    ito::RetVal retval(ito::retOk);
    int numPalettes = 1;

    if(!ito::ITOM_API_FUNCS_GRAPH)
    {
        emit statusBarMessage(tr("Could not change color bar, api is missing"), 4000);
        return false;
    }

    retval += apiPaletteGetNumberOfColorBars(numPalettes);

    if (numPalettes == 0 || retval.containsError())
    {
        emit statusBarMessage(tr("No color maps defined."), 4000);
        return false;
    }

    if (colormap == "__next__")
    {
        m_curOverlayColorMapIndex++;
        m_curOverlayColorMapIndex %= numPalettes; //map index to [0,numPalettes)
        retval += apiPaletteGetColorBarIdx(m_curOverlayColorMapIndex, newPalette);
    }
    else if (colormap == "__first__")
    {
        m_curOverlayColorMapIndex = 0;
        retval += apiPaletteGetColorBarIdx(m_curOverlayColorMapIndex, newPalette);
    }
    else
    {
        retval += apiPaletteGetColorBarName(colormap, newPalette);
    }

    if (retval.containsError() && retval.errorMessage() != NULL)
    {
        emit statusBarMessage(QString("%1").arg(QLatin1String(retval.errorMessage())), 4000);
        return false;
    }
    else if (retval.containsError())
    {
        emit statusBarMessage("error when loading color map", 4000);
        return false;
    }

    int totalStops = newPalette.colorStops.size();

    if (totalStops < 2)
    {
        emit statusBarMessage(tr("Selected color map has less than two points."), 4000);
        return false;
    }

    m_colorOverlayMapName = newPalette.name;


    if (newPalette.colorStops[totalStops - 1].first == newPalette.colorStops[totalStops - 2].first)  // BuxFix - For Gray-Marked
    {
        colorMap    = new QwtLinearColorMap(newPalette.colorStops[0].second, newPalette.colorStops[totalStops - 2].second, QwtColorMap::Indexed);
        if (totalStops > 2)
        {
            for (int i = 1; i < totalStops - 2; i++)
            {
                colorMap->addColorStop(newPalette.colorStops[i].first, newPalette.colorStops[i].second);
            }
            colorMap->addColorStop(newPalette.colorStops[totalStops-1].first, newPalette.colorStops[totalStops-1].second);
        }
    }
    else
    {
        colorMap    = new QwtLinearColorMap(newPalette.colorStops.first().second, newPalette.colorStops.last().second, QwtColorMap::Indexed);
        if (totalStops > 2)
        {
            for (int i = 1; i < totalStops - 1; i++)
            {
                colorMap->addColorStop(newPalette.colorStops[i].first, newPalette.colorStops[i].second);
             }
        }
    }

    if (colorMap)
    {
        this->m_dOverlayItem->setColorMap(colorMap);
    }
    else
    {
        delete colorMap;
    }

    replot();
    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setShowCenterMarker(bool show)
{
    m_showCenterMarker = show;
    setState(state());
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::keyPressEvent (QKeyEvent * event)
{
    event->ignore();

    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
    m_activeModifiers = event->modifiers();

    QPointF incr;

    if (event->modifiers() & Qt::ControlModifier)
    {
        //10-pixel increment
        incr.setX(invTransform(QwtPlot::xBottom, 10) - invTransform(QwtPlot::xBottom, 0));
        incr.setY(invTransform(QwtPlot::yLeft, 10) - invTransform(QwtPlot::yLeft, 0));
    }
    else
    {
        //1-pixel increment
        incr.setX(invTransform(QwtPlot::xBottom, 1) - invTransform(QwtPlot::xBottom, 0));
        incr.setY(invTransform(QwtPlot::yLeft, 1) - invTransform(QwtPlot::yLeft, 0));
    }

    if (state() == stateStackCut)
    {
        QPointF markerPosScaleCoords = m_pStackCutMarker->value();

        event->accept();

        switch(event->key())
        {
            case Qt::Key_Left:
                markerPosScaleCoords.rx()-=incr.rx();
                break;
            case Qt::Key_Right:
                markerPosScaleCoords.rx()+=incr.rx();
                break;
            case Qt::Key_Up:
                markerPosScaleCoords.ry()-=incr.ry();
                break;
            case Qt::Key_Down:
                markerPosScaleCoords.ry()+=incr.ry();
                break;
            default:
                event->ignore();
            break;
        }

        if (event->isAccepted() && m_rasterData->pointValid(markerPosScaleCoords))
        {
            m_pStackCutMarker->setValue(markerPosScaleCoords);
            m_pStackCutMarker->setVisible(true);

            QVector<QPointF> pts;
            pts.append(markerPosScaleCoords);
            p->displayCut(pts, m_zstackCutUID,true);
			if (((Itom2dQwtPlot*)this->parent())->pickerWidget())
			{
				(((Itom2dQwtPlot*)this->parent())->pickerWidget())->updateChildPlot(m_zstackCutUID, ito::Shape::Point, QVector4D(pts[0]));
			}

            replot();
        }
    }
    else if (state() == stateLineCut)
    {
        QVector<QPointF> pts;

        event->accept();

        if (event->key() == Qt::Key_H) //draw horizontal line in the middle of the plotted dataObject
        {
            QwtInterval hInterval = m_rasterData->interval(Qt::XAxis);
            QwtInterval vInterval = m_rasterData->interval(Qt::YAxis);

            pts.append(QPointF(hInterval.minValue(), (vInterval.minValue() + vInterval.maxValue())*0.5));
            pts.append(QPointF(hInterval.maxValue(), (vInterval.minValue() + vInterval.maxValue())*0.5));

            setCoordinates(pts, true);
        }
        else if (event->key() == Qt::Key_V) // draw vertical line in the middle of the plotted dataObject
        {
            QwtInterval hInterval = m_rasterData->interval(Qt::XAxis);
            QwtInterval vInterval = m_rasterData->interval(Qt::YAxis);

            pts.append(QPointF((hInterval.minValue() + hInterval.maxValue())*0.5, vInterval.minValue()));
            pts.append(QPointF((hInterval.minValue() + hInterval.maxValue())*0.5, vInterval.maxValue()));

            setCoordinates(pts, true);
        }
        else
        {
            pts.append(m_pLineCutLine->sample(0));
            pts.append(m_pLineCutLine->sample(1));

            switch(event->key())
            {
                case Qt::Key_Left:
                    pts[0].rx()-=incr.rx();
                    pts[1].rx()-=incr.rx();
                    break;
                case Qt::Key_Right:
                    pts[0].rx()+=incr.rx();
                    pts[1].rx()+=incr.rx();
                    break;
                case Qt::Key_Up:
                    pts[0].ry()-=incr.ry();
                    pts[1].ry()-=incr.ry();
                    break;
                case Qt::Key_Down:
                    pts[0].ry()+=incr.ry();
                    pts[1].ry()+=incr.ry();
                    break;
                default:
                    event->ignore();
                    break;
            }

            if (event->isAccepted())
            {
                setCoordinates(pts, true);
            }
        }

        if (event->isAccepted() && m_rasterData->pointValid(pts[0]) && m_rasterData->pointValid(pts[1]))
        {
            m_pLineCutLine->setSamples(pts);
            m_pLineCutLine->setVisible(true);

            if(m_dObjPtr && m_dObjPtr->getDims() > 2)
            {
                pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(),m_rasterData->getCurrentPlane()));
            }
            p->displayCut(pts, m_lineCutUID, false);
			if (((Itom2dQwtPlot*)this->parent())->pickerWidget())
			{
				QVector4D vec;
				if (pts.size() == 3) vec = QVector4D(pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
				else vec = QVector4D(pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
				(((Itom2dQwtPlot*)this->parent())->pickerWidget())->updateChildPlot(m_lineCutUID, ito::Shape::Line, vec);
			}

            replot();
        }
    }
    
    if (!event->isAccepted())
    {
        ItomQwtPlot::keyPressEvent(event);
    }

}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::keyReleaseEvent (QKeyEvent* event)
{
    m_activeModifiers = Qt::NoModifier;

    ItomQwtPlot::keyReleaseEvent(event);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setColorBarVisible(bool visible)
{
    m_pData->m_colorBarVisible = visible;
    enableAxis(QwtPlot::yRight, visible);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setLabels(const QString &title, const QString &valueLabel, const QString &xAxisLabel, const QString &yAxisLabel)
{
    if (m_pData->m_autoValueLabel)
    {
        setAxisTitle(QwtPlot::yRight, valueLabel);
    }
    else
    {
        setAxisTitle(QwtPlot::yRight, m_pData->m_valueLabel);
    }

    if (m_pData->m_autoxAxisLabel)
    {
        setAxisTitle(QwtPlot::xBottom, xAxisLabel);
    }
    else
    {
        setAxisTitle(QwtPlot::xBottom, m_pData->m_xaxisLabel);
    }

    if (m_pData->m_autoyAxisLabel)
    {
        setAxisTitle(QwtPlot::yLeft, yAxisLabel);
    }
    else
    {
        setAxisTitle(QwtPlot::yLeft, m_pData->m_yaxisLabel);
    }

    if (m_pData->m_autoTitle)
    {
        setTitle(title);
    }
    else
    {
        setTitle(m_pData->m_title);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::updateLabels()
{
    if (m_pData->m_autoValueLabel)
    {
        setAxisTitle(QwtPlot::yRight, m_pData->m_valueLabelDObj);
    }
    else
    {
        setAxisTitle(QwtPlot::yRight, m_pData->m_valueLabel);
    }

    if (m_pData->m_autoxAxisLabel)
    {
        setAxisTitle(QwtPlot::xBottom, m_pData->m_xaxisLabelDObj);
    }
    else
    {
        setAxisTitle(QwtPlot::xBottom, m_pData->m_xaxisLabel);
    }

    if (m_pData->m_autoyAxisLabel)
    {
        setAxisTitle(QwtPlot::yLeft, m_pData->m_yaxisLabelDObj);
    }
    else
    {
        setAxisTitle(QwtPlot::yLeft, m_pData->m_yaxisLabel);
    }

    if (m_pData->m_autoTitle)
    {
        setTitle(m_pData->m_titleDObj);
    }
    else
    {
        setTitle(m_pData->m_title);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::synchronizeScaleValues()
{
    QwtInterval ival = m_rasterData->interval(Qt::ZAxis);
    m_pData->m_valueMin = ival.minValue();
    m_pData->m_valueMax = ival.maxValue();

    if (m_pData->m_xaxisScaleAuto)
    {
        ival = m_rasterData->interval(Qt::XAxis);
        
    }
    else
    {
        ival = axisScaleDiv(xBottom).interval();
    }
    m_pData->m_xaxisMin = std::min(ival.minValue(), ival.maxValue());
    m_pData->m_xaxisMax = std::max(ival.minValue(), ival.maxValue());

    if (m_pData->m_yaxisScaleAuto)
    {
        ival = m_rasterData->interval(Qt::YAxis);
    }
    else
    {
        ival = axisScaleDiv(yLeft).interval();
    }
    m_pData->m_yaxisMin = std::min(ival.minValue(), ival.maxValue());
    m_pData->m_yaxisMax = std::max(ival.minValue(), ival.maxValue());
}

//----------------------------------------------------------------------------------------------------------------------------------
/*
@param doReplot forces a replot of the content
@param doZoomBase if true, the x/y-zoom is reverted to the full x-y-area of the manually set ranges (the same holds for the value range)
*/
void PlotCanvas::updateScaleValues(bool doReplot /*= true*/, bool doZoomBase /*= true*/)
{
    QwtInterval ival;
    if (m_pData->m_valueScaleAuto)
    {
        internalDataUpdated();
        ival = m_rasterData->interval(Qt::ZAxis);
        m_pData->m_valueMin = ival.minValue();
        m_pData->m_valueMax = ival.maxValue();
    }
    else
    {
        m_rasterData->setInterval(Qt::ZAxis, QwtInterval(m_pData->m_valueMin, m_pData->m_valueMax));
    }

    if (m_pData->m_xaxisScaleAuto)
    {
        ival = m_rasterData->interval(Qt::XAxis);
        m_pData->m_xaxisMin = ival.minValue();
        m_pData->m_xaxisMax = ival.maxValue();
    }

    if (m_pData->m_yaxisScaleAuto)
    {
        ival = m_rasterData->interval(Qt::YAxis);
        m_pData->m_yaxisMin = ival.minValue();
        m_pData->m_yaxisMax = ival.maxValue();
    }

    if (m_pData->m_yaxisFlipped)
    {
        axisScaleEngine(QwtPlot::yLeft)->setAttribute(QwtScaleEngine::Inverted, true);
    }
    else
    {
        axisScaleEngine(QwtPlot::yLeft)->setAttribute(QwtScaleEngine::Inverted, false);
    }

    setAxisScale(QwtPlot::yRight, m_pData->m_valueMin, m_pData->m_valueMax);
    QwtScaleWidget *widget = axisWidget(QwtPlot::yRight);
    if (widget)
    {
        QwtInterval ival(m_pData->m_valueMin, m_pData->m_valueMax);
        axisWidget(QwtPlot::yRight)->setColorMap(ival, const_cast<QwtColorMap*>(widget->colorMap())); //the color map should be unchanged
    }

    if (doZoomBase)
    {
        // 10.02.15 ck we don't want to check if a zoomer exists, as it is always created in the constructor but if it is enabled
        if (zoomer()->isEnabled())
        {
            QRectF zoom(m_pData->m_xaxisMin, m_pData->m_yaxisMin, (m_pData->m_xaxisMax - m_pData->m_xaxisMin), (m_pData->m_yaxisMax - m_pData->m_yaxisMin));
            zoom = zoom.normalized();

            if (zoom == zoomer()->zoomRect())
            {
                zoomer()->zoom(zoom);
                zoomer()->rescale(false); //zoom of zoomer does not call rescale in this case, therefore we do it here
            }
            else
            {
                zoomer()->zoom(zoom);
            }
        }
        else
        {
            setAxisScale(QwtPlot::xBottom, m_pData->m_xaxisMin, m_pData->m_xaxisMax);

            if (m_pData->m_yaxisFlipped)
            {
                setAxisScale(QwtPlot::yLeft, m_pData->m_yaxisMax, m_pData->m_yaxisMin);
            }
            else
            {
                setAxisScale(QwtPlot::yLeft, m_pData->m_yaxisMin, m_pData->m_yaxisMax);
            }

            QRectF zoom(m_pData->m_xaxisMin, m_pData->m_yaxisMin, (m_pData->m_xaxisMax - m_pData->m_xaxisMin), (m_pData->m_yaxisMax - m_pData->m_yaxisMin));
            zoom = zoom.normalized();
            
            if (zoom == zoomer()->zoomRect())
            {
                zoomer()->zoom(zoom);
                zoomer()->rescale(false); //zoom of zoomer does not call rescale in this case, therefore we do it here
            }
            else
            {
                zoomer()->appendZoomStack(zoom);
            }

        }
    }

    if (doReplot)
    {
        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::home()
{
    zoomer()->zoom(0);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setInterval(Qt::Axis axis, const ito::AutoInterval &interval)
{
    if (axis == Qt::XAxis)
    {
        m_pData->m_xaxisScaleAuto = interval.isAuto();

        if (m_pData->m_xaxisScaleAuto)
        {
            QwtInterval ival = m_rasterData->interval(Qt::XAxis);
            m_pData->m_valueMin = ival.minValue();
            m_pData->m_valueMax = ival.maxValue();
        }
        else
        {
            m_pData->m_xaxisMin = interval.minimum();
            m_pData->m_xaxisMax = interval.maximum();
        }
        setAxisScale(QwtPlot::xBottom, m_pData->m_xaxisMin, m_pData->m_xaxisMax);
    }
    else if (axis == Qt::YAxis)
    {
        m_pData->m_yaxisScaleAuto = interval.isAuto();

        if (m_pData->m_yaxisScaleAuto)
        {
            QwtInterval ival = m_rasterData->interval(Qt::YAxis);
            m_pData->m_yaxisMin = ival.minValue();
            m_pData->m_yaxisMax = ival.maxValue();
        }
        else
        {
            m_pData->m_yaxisMin = interval.minimum();
            m_pData->m_yaxisMax = interval.maximum();
        }

        QwtScaleEngine *scaleEngine = axisScaleEngine(QwtPlot::yLeft);

        if (m_pData->m_yaxisFlipped)
        {
            scaleEngine->setAttribute(QwtScaleEngine::Inverted, true);
            setAxisScale(QwtPlot::yLeft, m_pData->m_yaxisMax, m_pData->m_yaxisMin);
        }
        else
        {
            scaleEngine->setAttribute(QwtScaleEngine::Inverted, false);
            setAxisScale(QwtPlot::yLeft, m_pData->m_yaxisMin, m_pData->m_yaxisMax);
        }

    }
    else if (axis == Qt::ZAxis)
    {
        if (m_pData->m_dataType == ito::tRGBA32)
        {
            m_pData->m_valueScaleAuto = false;
            m_pData->m_valueMin = std::numeric_limits<ito::uint8>::min();
            m_pData->m_valueMax = std::numeric_limits<ito::uint8>::max();
        }
        else
        {
            m_pData->m_valueScaleAuto = interval.isAuto();

            if (m_pData->m_valueScaleAuto)
            {
                internalDataUpdated();
                QwtInterval ival = m_rasterData->interval(Qt::ZAxis);
                m_pData->m_valueMin = ival.minValue();
                m_pData->m_valueMax = ival.maxValue();
            }
            else
            {
                m_pData->m_valueMin = interval.minimum();
                m_pData->m_valueMax = interval.maximum();
            }
        }

        QwtScaleWidget *widget = axisWidget(QwtPlot::yRight);

        if (widget)
        {
            QwtInterval ival(interval.minimum(), interval.maximum());
            widget->setColorMap(ival, const_cast<QwtColorMap*>(widget->colorMap())); //the color map should be unchanged
        }

        setAxisScale(QwtPlot::yRight, m_pData->m_valueMin, m_pData->m_valueMax);

        m_rasterData->setInterval(Qt::ZAxis, QwtInterval(m_pData->m_valueMin, m_pData->m_valueMax));
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval PlotCanvas::getInterval(Qt::Axis axis) const
{
    QwtInterval i;
    bool autoScale;

    switch (axis)
    {
    case Qt::ZAxis:
        i = m_rasterData->interval(axis);
        autoScale = m_pData->m_valueScaleAuto;
        break;
    case Qt::XAxis:
        {
            QwtScaleDiv div =axisScaleDiv(QwtPlot::xBottom);
            i = div.interval();
            autoScale = m_pData->m_xaxisScaleAuto;
        }
        break;
    case Qt::YAxis:
        {
            QwtScaleDiv div =axisScaleDiv(QwtPlot::yLeft);
            i = div.interval();
            autoScale = m_pData->m_yaxisScaleAuto;
        }
    }

    double minimum = i.minValue();
    double maximum = i.maxValue();
    if (maximum < minimum) std::swap(minimum, maximum);
    return ito::AutoInterval(minimum, maximum, autoScale);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setOverlayInterval(Qt::Axis axis, const ito::AutoInterval &interval)
{
    if (axis == Qt::ZAxis)
    {
        m_pData->m_overlayScaleAuto = false;
        m_pData->m_overlayMin = interval.minimum();
        m_pData->m_overlayMax = interval.maximum();

        m_rasterOverlayData->setInterval(Qt::ZAxis, QwtInterval(m_pData->m_overlayMin, m_pData->m_overlayMax));
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval PlotCanvas::getOverlayInterval(Qt::Axis axis) const
{
    QwtInterval i = m_rasterOverlayData->interval(axis);
    double minimum = i.minValue();
    double maximum = i.maxValue();
    if (maximum < minimum) std::swap(minimum, maximum);
    return ito::AutoInterval(minimum, maximum, m_pData->m_overlayScaleAuto);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::int32 PlotCanvas::getCurrentPlane() const
{
    return m_rasterData->getCurrentPlane();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::stateChanged(int state)
{
    m_pCenterMarker->setVisible(m_showCenterMarker);
    m_pActCntrMarker->setChecked(m_showCenterMarker);
    if (m_showCenterMarker && m_dObjPtr)
    {
        if (m_dObjPtr->getDims() > 1)
        {
            bool valid;
            m_pCenterMarker->setXValue(m_dObjPtr->getPixToPhys(m_dObjPtr->getDims() - 1, (m_dObjPtr->getSize(m_dObjPtr->getDims() - 1) - 1) / 2.0, valid));
            m_pCenterMarker->setYValue(m_dObjPtr->getPixToPhys(m_dObjPtr->getDims() - 2, (m_dObjPtr->getSize(m_dObjPtr->getDims() - 2) - 1) / 2.0, valid));
        }
    }

    if (m_pValuePicker) m_pValuePicker->setEnabled(state == stateValuePicker);
    if (m_pLineCutPicker) m_pLineCutPicker->setEnabled(state == stateLineCut);
    if (m_pStackPicker) m_pStackPicker->setEnabled(state == stateStackCut);
    m_pActLineCut->setChecked(state == stateLineCut);
    m_pActValuePicker->setChecked(state == stateValuePicker);
    m_pActStackCut->setChecked(state == stateStackCut);

    if (state != stateLineCut && state != stateStackCut)
    {
        setCoordinates(QVector<QPointF>(), false);
    }

    switch (state)
    {
    case stateValuePicker:
        canvas()->setCursor(Qt::CrossCursor);
        break;
    case stateStackCut:
        canvas()->setCursor(Qt::CrossCursor);
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
// a1 is line1 start, a2 is line1 end, b1 is line2 start, b2 is line2 end
bool PlotCanvas::lineIntersection(const QPointF &a1, const QPointF &a2, const QPointF &b1, const QPointF &b2, QPointF &intersection)
{
    intersection = QPointF(0,0);

    QPointF b = a2 - a1;
    QPointF d = b2 - b1;
    float bDotDPerp = b.x() * d.y() - b.y() * d.x();

    // if b dot d == 0, it means the lines are parallel so have infinite intersection points
    if (bDotDPerp == 0)
        return false;

    QPointF c = b1 - a1;
    float t = (c.x() * d.y() - c.y() * d.x()) / bDotDPerp;
    if (t < 0 || t > 1)
        return false;

    float u = (c.x() * b.y() - c.y() * b.x()) / bDotDPerp;
    if (u < 0 || u > 1)
        return false;

    intersection = a1 + t * b;

    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::zStackCutTrackerAppended(const QPoint &pt)
{
    QVector<QPointF> pts;

    if (state() == stateStackCut)
    {
        
        QPointF ptScale;
        ptScale.setY(invTransform(QwtPlot::yLeft, pt.y()));
        ptScale.setX(invTransform(QwtPlot::xBottom, pt.x()));
        if (m_rasterData->pointValid(ptScale))
        {
            m_pStackCutMarker->setValue(ptScale);
            m_pStackCutMarker->setVisible(true);

            pts.append(ptScale);
            setCoordinates(pts, true);
            ((Itom2dQwtPlot*)parent())->displayCut(pts, m_zstackCutUID,true);

			if (((Itom2dQwtPlot*)this->parent())->pickerWidget())
			{
				(((Itom2dQwtPlot*)this->parent())->pickerWidget())->updateChildPlot(m_zstackCutUID, ito::Shape::Point, QVector4D(pt));
			}

            replot();
        }   
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::zStackCutTrackerMoved(const QPoint &pt)
{
    QVector<QPointF> pts;

    if (state() == stateStackCut)
    {
        QPointF ptScale;
        ptScale.setY(invTransform(QwtPlot::yLeft, pt.y()));
        ptScale.setX(invTransform(QwtPlot::xBottom, pt.x()));
        if (m_rasterData->pointValid(ptScale))
        {
            if (m_activeModifiers.testFlag(Qt::ControlModifier))
            {
                if (abs(ptScale.x() - m_pStackCutMarker->xValue()) > abs(ptScale.y() - m_pStackCutMarker->yValue()))
                {
                    ptScale.setY(m_pStackCutMarker->yValue());
                }
                else
                {
                    ptScale.setX(m_pStackCutMarker->xValue());
                }
            }

            m_pStackCutMarker->setValue(ptScale);
            m_pStackCutMarker->setVisible(true);

            pts.append(ptScale);
            setCoordinates(pts, true);

            ((Itom2dQwtPlot*)parent())->displayCut(pts, m_zstackCutUID,true);
			if (((Itom2dQwtPlot*)this->parent())->pickerWidget())
			{
				(((Itom2dQwtPlot*)this->parent())->pickerWidget())->updateChildPlot(m_zstackCutUID, ito::Shape::Point, QVector4D(pts[0]));
			}
            replot();
        }    
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::lineCutMoved(const QPoint &pt)
{
    QVector<QPointF> pts;
    pts.resize(2);

    if (state() == stateLineCut)
    {
        QwtInterval xInterval = m_rasterData->interval(Qt::XAxis); 
        QwtInterval yInterval = m_rasterData->interval(Qt::YAxis);

        if (m_pLineCutLine->dataSize() > 0)
            pts[0] = m_pLineCutLine->sample(0);
        else
            return;

        if (m_lineCutValidStart)
        {
            pts[1].setY(invTransform(QwtPlot::yLeft, pt.y()));
            pts[1].setX(invTransform(QwtPlot::xBottom, pt.x()));

            if (!xInterval.contains(pts[1].x()) || !yInterval.contains(pts[1].y()))
            {
                QPointF intersection;
                QPointF rt(xInterval.maxValue(), yInterval.maxValue());
                QPointF lt(xInterval.minValue(), yInterval.maxValue());
                QPointF rb(xInterval.maxValue(), yInterval.minValue());
                QPointF lb(xInterval.minValue(), yInterval.minValue());
                
                if ((pts[1].x() < pts[0].x()) && lineIntersection(pts[1], pts[0], lt, lb, intersection))
                {
                    //try if pts[1] - pts[0] intersects with left border
                    pts[1] = intersection;
                }
                else if ((pts[1].x() > pts[0].x()) && lineIntersection(pts[1], pts[0], rt, rb, intersection))
                {
                    //try if pts[1] - pts[0] intersects with right border
                    pts[1] = intersection;
                }
                else if ((pts[1].y() > pts[0].y()) && lineIntersection(pts[1], pts[0], lt, rt, intersection))
                {
                    //try if pts[1] - pts[0] intersects with top border
                    pts[1] = intersection;
                }
                else if ((pts[1].y() < pts[0].y()) && lineIntersection(pts[1], pts[0], lb, rb, intersection))
                {
                    //try if pts[1] - pts[0] intersects with bottom border
                    pts[1] = intersection;
                }
            }

            if (m_activeModifiers.testFlag(Qt::ControlModifier))
            {
                if (abs(pts[1].x() - pts[0].x()) > abs(pts[1].y() - pts[0].y()))
                {
                    pts[1].setY(pts[0].y());
                }
                else
                {
                    pts[1].setX(pts[0].x());
                }
            }

            m_pLineCutLine->setSamples(pts);

            setCoordinates(pts, true);

            if(m_dObjPtr && m_dObjPtr->getDims() > 2)
            {
                pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(),m_rasterData->getCurrentPlane()));
            }
            ((Itom2dQwtPlot*)parent())->displayCut(pts, m_lineCutUID, false);
			if (((Itom2dQwtPlot*)this->parent())->pickerWidget())
			{
				QVector4D vec;
				if (pts.size() == 3) vec = QVector4D(pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
				else vec = QVector4D(pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
				(((Itom2dQwtPlot*)this->parent())->pickerWidget())->updateChildPlot(m_lineCutUID, ito::Shape::Line, vec);
			}
            replot();
        }
        else //first point is still not valid, try to find a valid first point
        {
            QPointF pts0 = pts[0];

            pts[0].setY(invTransform(QwtPlot::yLeft, pt.y()));
            pts[0].setX(invTransform(QwtPlot::xBottom, pt.x()));

            if (m_rasterData->pointValid(pts[0]))
            {
                //the mouse cursor is now in a valid area. Due to discretization limits, it might be that this point is not exactly at the border
                //towards the direction where the mouse has been pressed for the first time. Therefore get the intersection to the initial mouse press
                //position.
                QPointF intersection;
                QPointF rt(xInterval.maxValue(), yInterval.maxValue());
                QPointF lt(xInterval.minValue(), yInterval.maxValue());
                QPointF rb(xInterval.maxValue(), yInterval.minValue());
                QPointF lb(xInterval.minValue(), yInterval.minValue());
                
                if (lineIntersection(pts[0], pts0, lt, lb, intersection))
                {
                    //try if pts[1] - pts[0] intersects with left border
                    pts[0] = intersection;
                }
                else if (lineIntersection(pts[0], pts0, rt, rb, intersection))
                {
                    //try if pts[1] - pts[0] intersects with right border
                    pts[0] = intersection;
                }
                else if (lineIntersection(pts[0], pts0, lt, rt, intersection))
                {
                    //try if pts[1] - pts[0] intersects with top border
                    pts[0] = intersection;
                }
                else if (lineIntersection(pts[0], pts0, lb, rb, intersection))
                {
                    //try if pts[1] - pts[0] intersects with bottom border
                    pts[0] = intersection;
                }

                m_lineCutValidStart = true;
            }
            else
            {
                //first point not valid
                m_lineCutValidStart = false;
            }

            pts[1] = pts[0];

            m_pLineCutLine->setVisible(true);
            m_pLineCutLine->setSamples(pts);

            setCoordinates(pts, true);

            // check for m_dObjPtr first otherwise crash
            if(m_dObjPtr && m_dObjPtr->getDims() > 2)
            {
                pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(),m_rasterData->getCurrentPlane()));
            }

            ((Itom2dQwtPlot*)parent())->displayCut(pts, m_lineCutUID, false);
			if (((Itom2dQwtPlot*)this->parent())->pickerWidget())
			{
				QVector4D vec;
				if (pts.size() == 3) vec = QVector4D(pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
				else vec = QVector4D(pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
				(((Itom2dQwtPlot*)this->parent())->pickerWidget())->updateChildPlot(m_lineCutUID, ito::Shape::Line, vec);
			}
            replot();
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::lineCutAppended(const QPoint &pt)
{
    QVector<QPointF> pts;
    pts.resize(2);

    if (state() == stateLineCut && m_dObjPtr)
    {
        pts[0].setY(invTransform(QwtPlot::yLeft, pt.y()));
        pts[0].setX(invTransform(QwtPlot::xBottom, pt.x()));

        if (m_rasterData->pointValid(pts[0]))
        {
            m_lineCutValidStart = true;
        }
        else
        {
            //first point not valid
            m_lineCutValidStart = false;
        }

        pts[1] = pts[0];

        m_pLineCutLine->setVisible(true);
        m_pLineCutLine->setSamples(pts);

        setCoordinates(pts, true);

        // check for m_dObjPtr first otherwise crash
        if(m_dObjPtr && m_dObjPtr->getDims() > 2)
        {
            pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(),m_rasterData->getCurrentPlane()));
        }
        
        ((Itom2dQwtPlot*)parent())->displayCut(pts, m_lineCutUID, false);
		if (((Itom2dQwtPlot*)this->parent())->pickerWidget())
		{
			QVector4D vec;
			if (pts.size() == 3) vec = QVector4D(pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
			else vec = QVector4D(pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
			(((Itom2dQwtPlot*)this->parent())->pickerWidget())->updateChildPlot(m_lineCutUID, ito::Shape::Line, vec);
		}

        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::childFigureDestroyed(QObject* /*obj*/, ito::uint32 UID)
{
    if (UID > 0)
    {
        if (UID == m_zstackCutUID)
        {
            m_pStackCutMarker->setVisible(false);
        }
        else if (UID == m_lineCutUID)
        {
            m_pLineCutLine->setVisible(false);
            setCoordinates(QVector<QPointF>(), false);
        }
    }
    else
    {
        //hide all related markers (we have no further information)
        m_pStackCutMarker->setVisible(false);
        m_pLineCutLine->setVisible(false);
    }

    replot();
}



//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> PlotCanvas::getDisplayed(void)
{
    if (!m_rasterData)
    {
        return QSharedPointer<ito::DataObject>(); 
    }

    return m_rasterData->rasterToObject(axisInterval(QwtPlot::xBottom), axisInterval(QwtPlot::yLeft));
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> PlotCanvas::getDisplayedOverlayObject()
{
    if (!m_rasterOverlayData)
    {
        return QSharedPointer<ito::DataObject>(); 
    }
    return m_rasterOverlayData->rasterToObject(axisInterval(QwtPlot::xBottom), axisInterval(QwtPlot::yLeft));
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> PlotCanvas::getOverlayObject()
{
    if (!m_rasterData)
    {
        return QSharedPointer<ito::DataObject>(); 
    }
    return m_rasterOverlayData->rasterToObject();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setOverlayObject(ito::DataObject* newOverlay)
{
    if(newOverlay)
    {
        m_dObjItem->setVisible(m_pData->m_alpha < 255);
        m_dOverlayItem->setAlpha(m_pData->m_alpha);
    }
    else
    {
        m_dObjItem->setVisible(true);
        m_dOverlayItem->setAlpha(0);
    }
    m_rasterOverlayData->updateDataObject(newOverlay);

    m_dOverlayItem->setVisible(m_rasterOverlayData->isInit() && m_pData->m_alpha > 0);
    if(m_pValuePicker) m_pValuePicker->enableOverlay(m_pData->m_alpha > 0);
    Itom2dQwtPlot *p = (Itom2dQwtPlot*)(this->parent());
    p->enableOverlaySlider(m_rasterOverlayData->isInit());
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::alphaChanged()
{
    m_dOverlayItem->setVisible(m_pData->m_alpha > 0 && m_rasterOverlayData->isInit());
    m_dOverlayItem->setAlpha(m_pData->m_alpha);
    m_dObjItem->setVisible(m_pData->m_alpha < 255);
    if(m_pValuePicker) m_pValuePicker->enableOverlay(m_pData->m_alpha > 0);
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::getMinMaxLoc(double &min, ito::uint32 *minLoc, double &max, ito::uint32 *maxLoc)
{
    if (!m_rasterData)
    {
        min = std::numeric_limits<double>::quiet_NaN();
        max = std::numeric_limits<double>::quiet_NaN();

        return;
    }   

    m_rasterData->getMinMaxLoc(min, minLoc, max, maxLoc);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::getMinMaxPhysLoc(double &min, double *minPhysLoc, double &max, double *maxPhysLoc)
{
    if (!m_rasterData || !m_dObjPtr || m_dObjPtr->getDims() < 2)
    {
        min = std::numeric_limits<double>::quiet_NaN();
        max = std::numeric_limits<double>::quiet_NaN();

        return;
    }   

    ito::uint32 minLoc[3];
    ito::uint32 maxLoc[3];
    m_rasterData->getMinMaxLoc(min, minLoc, max, maxLoc);

    bool check;

    minPhysLoc[0] = minLoc[0];
    minPhysLoc[1] = m_dObjPtr->getPixToPhys(m_dObjPtr->getDims()-2, minLoc[1], check); 
    minPhysLoc[2] = m_dObjPtr->getPixToPhys(m_dObjPtr->getDims()-1, minLoc[2], check); 

    maxPhysLoc[0] = maxLoc[0];
    maxPhysLoc[1] = m_dObjPtr->getPixToPhys(m_dObjPtr->getDims()-2, maxLoc[1], check); 
    maxPhysLoc[2] = m_dObjPtr->getPixToPhys(m_dObjPtr->getDims()-1, maxLoc[2], check); 

    return;
}


//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setCoordinates(const QVector<QPointF> &pts, bool visible)
{
    m_pActCoordinates->setVisible(visible);

    if (visible)
    {
        char buf[60] = { 0 };
        if (pts.size() > 1)
        {
            sprintf(buf, "[%.4g; %.4g]\n[%.4g; %.4g]", pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
        }
        else if (pts.size() == 1)
        {
            sprintf(buf, "[%.4g; %.4g]\n[ - ; - ]", pts[0].x(), pts[0].y());
        }
        else
        {
            sprintf(buf, "[ - ; - ]\n[ - ; - ]");
        }
        m_pCoordinates->setText(buf);
    }

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotCanvas::setLinePlot(const double x0, const double y0, const double x1, const double y1)
{
    if (m_pActLineCut->isCheckable() && m_pActLineCut->isEnabled())
    {
        m_pActLineCut->setChecked(true);
        mnuLineCut(true);
    }
    else
    {
        return ito::RetVal(ito::retError, 0, tr("Set lineCut coordinates failed. Could not activate lineCut.").toLatin1().data());
    }

    QPoint first(transform(QwtPlot::xBottom, x0), transform(QwtPlot::yLeft, y0));
    QPoint second(transform(QwtPlot::xBottom, x1), transform(QwtPlot::yLeft, y1));
    lineCutAppended(first);
    lineCutMoved(second);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuScaleSettings()
{
    synchronizeScaleValues();

    Dialog2DScale *dlg = new Dialog2DScale(*m_pData, qobject_cast<QWidget*>(parent()));
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData(*m_pData);
        updateScaleValues();
    }

    delete dlg;
    dlg = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuCmplxSwitch(QAction *action)
{
    m_pMnuCmplxSwitch->setDefaultAction(action);
    setButtonStyle(buttonStyle()); //to change icon of menu

    int idx = action->data().toInt();
    m_pData->m_cmplxType = (ItomQwtPlotEnums::ComplexType)idx;
    internalDataUpdated();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuColorPalette()
{
    setColorMap("__next__");
}



//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuToggleColorBar(bool checked)
{
    setColorBarVisible(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuValuePicker(bool checked)
{
    if (checked)
    {
        setState(stateValuePicker);
    }
    else if (state() == stateValuePicker)
    {
        setState(stateIdle);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuLineCut(bool checked)
{
    if (checked)
    {
        setState(stateLineCut);
    }
    else if (state() == stateLineCut)
    {
        setState(stateIdle);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuLineCutMode(QAction *action)
{
    if (!m_pActLineCut->isChecked())
    {
        m_pActLineCut->setChecked(true);
    }

    ito::AutoInterval x = getInterval(Qt::XAxis);
    ito::AutoInterval y = getInterval(Qt::YAxis);

    double min = 0;
    double max = 0;

    double minLoc[3];
    double maxLoc[3];
    getMinMaxPhysLoc(min, minLoc, max, maxLoc);

    switch (action->data().toInt())
    {
    default:
    case 0:
        if (ito::dObjHelper::isFinite(min) && ito::dObjHelper::isFinite(max))
        {

            double dy = maxLoc[1] - minLoc[1];
            double dx = maxLoc[2] - minLoc[2];

            if (!ito::dObjHelper::isNotZero(dy) && !ito::dObjHelper::isNotZero(dx))
            {
                y.setMinimum((y.rmin() + y.rmax()) / 2.0);
                y.setMaximum(y.rmin());
            }
            else if (fabs(dx) < std::numeric_limits<double>::epsilon() * 100)
            {
                y.setMinimum(minLoc[1]);
                y.setMaximum(maxLoc[1]);
            }
            else if (fabs(dy) < std::numeric_limits<double>::epsilon() * 100)
            {
                x.setMinimum(minLoc[2]);
                x.setMaximum(maxLoc[2]);
            }
            else
            {
                double b = minLoc[1] - dy / dx * minLoc[2];

                double xbmin = std::min(x.rmin(), x.rmax());
                double xbmax = std::max(x.rmin(), x.rmax());
                double ybmin = std::min(y.rmin(), y.rmax());
                double ybmax = std::max(y.rmin(), y.rmax());

                double xmin = (ybmin - b) / (dy / dx);
                double xmax = (ybmax - b) / (dy / dx);
                double ymin = xbmin * (dy / dx) + b;
                double ymax = xbmax * (dy / dx) + b;

                if (dx / dy > 0)
                {
                    if (xmin < xbmin)
                    {
                        x.setMinimum(xbmin);
                        y.setMinimum(ymin);
                    }
                    else
                    {
                        x.setMinimum(xmin);
                        y.setMinimum(ybmin);
                    }

                    if (xmax > xbmax)
                    {
                        x.setMaximum(xbmax);
                        y.setMaximum(ymax);
                    }
                    else
                    {
                        x.setMaximum(xmax);
                        y.setMaximum(ybmax);
                    }
                }
                else
                {
                    if (xmin > xbmax)
                    {
                        x.setMinimum(xbmin);
                        y.setMinimum(ymin);
                    }
                    else
                    {
                        x.setMinimum(xmin);
                        y.setMinimum(ybmin);
                    }

                    if (xmax < xbmin)
                    {
                        x.setMaximum(xbmax);
                        y.setMaximum(ymax);
                    }
                    else
                    {
                        x.setMaximum(xmax);
                        y.setMaximum(ybmax);
                    }
                }

            }
            /*
            x.setX(minLoc[2]);
            x.setY(maxLoc[2]);
            y.setX(minLoc[1]);
            y.setY(maxLoc[1]);
            */
        }
        break;
    case 1:
        if (ito::dObjHelper::isFinite(min) && ito::dObjHelper::isFinite(max))
        {
            y.setMinimum(minLoc[1]);
            y.setMaximum(minLoc[1]);
        }
        break;
    case 2:
        if (ito::dObjHelper::isFinite(min) && ito::dObjHelper::isFinite(max))
        {
            y.setMinimum(maxLoc[1]);
            y.setMaximum(maxLoc[1]);
        }
        break;

    case 3:
        if (ito::dObjHelper::isFinite(min) && ito::dObjHelper::isFinite(max))
        {
            x.setMinimum(minLoc[2]);
            x.setMaximum(minLoc[2]);
        }
        break;

    case 4:
        if (ito::dObjHelper::isFinite(min) && ito::dObjHelper::isFinite(max))
        {
            x.setMinimum(maxLoc[2]);
            x.setMaximum(maxLoc[2]);
        }
        break;
    }

    setLinePlot(x.rmin(), y.rmin(), x.rmax(), y.rmax());
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuStackCut(bool checked)
{
    if (checked)
    {
        setState(stateStackCut);
    }
    else if (state() == stateStackCut)
    {
        setState(stateIdle);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuPlaneSelector(int plane)
{
    Itom2dQwtPlot *p = qobject_cast<Itom2dQwtPlot*>(this->parent());
    p->setPlaneIndex(plane);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuOverlaySliderChanged(int value)
{
    if (m_pData == NULL)
    {
        return;
    }

    if (value != m_pData->m_alpha)
    {
        m_pData->m_alpha = value;
        alphaChanged();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuCenterMarker(bool checked)
{
    setShowCenterMarker(checked);
    Itom2dQwtPlot *p = qobject_cast<Itom2dQwtPlot*>(this->parent());
    if (p)
        p->updatePropertyDock();
}

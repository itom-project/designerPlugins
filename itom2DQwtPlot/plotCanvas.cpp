/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2025, Institut für Technische Optik (ITO),
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

#include "plotCanvas.h"
#include "common/sharedStructuresGraphics.h"
#include "common/numeric.h"
#include "common/apiFunctionsGraphInc.h"
#include "common/apiFunctionsInc.h"
#include "DataObject/dataObjectFuncs.h"

#include "dataObjRasterData.h"
#include "valuePicker2d.h"
#include "multiPointPickerMachine.h"
#include "itomPlotZoomer.h"
#include "dialog2DScale.h"
#include "itomLogLogScaleEngine.h"
#include "itomColorMap.h"

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
PlotCanvas::PlotCanvas(PlotCanvas::InternalData *m_pData, ItomQwtDObjFigure * parent /*= NULL*/) :
	ItomQwtPlot(parent),
	m_pLineCutPicker(NULL),
	m_pCenterMarker(NULL),
	m_dObjItem(NULL),
	m_rasterData(NULL),
	m_dOverlayItem(NULL),
	m_pData(m_pData),
	m_curOverlayColorMapIndex(0),
	m_curColorMapIndex(0),
    m_curContourColorMapIndex(-1),
	m_pValuePicker(NULL),
	m_dObjPtr(NULL),
	m_pStackPicker(NULL),
	m_pLineCutLine(NULL),
    m_pVolumeCutLine(NULL),
	m_isRefreshingPlot(false),
	m_unitLabelChanged(false),
	m_pPaletteIsChanging(false),
	m_pActScaleSettings(NULL),
	m_pActColorPalette(NULL),
	m_pMenuColorPalette(NULL),
    m_pActToggleColorBar(NULL),
    m_pActValuePicker(NULL),
    m_pActLineCut(NULL),
    m_pActVolumeCut(NULL),
    m_showCenterMarker(false),
    m_pMnuLineCutMode(NULL),
    m_pActStackCut(NULL),
    m_pActPlaneSelector(NULL),
    m_pCoordinates(NULL),
    m_pActCoordinates(NULL),
    m_pActCmplxSwitch(NULL),
    m_pMnuCmplxSwitch(NULL),
    m_pActDataChannel(NULL),
    m_pMnuDataChannel(NULL),
    m_pActCntrMarker(NULL),
    m_pOverlaySlider(NULL),
    m_pActOverlaySlider(NULL),
    m_currentDataType(-1),
    m_valueScale(ItomQwtPlotEnums::Linear),
    m_dObjVolumeCut(),
    m_dir(inPlane),
    m_gridStyle(Itom2dQwtPlot::GridNo),
    m_pPlotGrid(NULL)
{
    createActions();
    setButtonStyle(buttonStyle());

    canvas()->setCursor(Qt::ArrowCursor);

    m_pPlotGrid = new QwtPlotGrid();
    m_pPlotGrid->setZ(11.0);
    m_pPlotGrid->attach(this);
    setGridStyle(m_gridStyle);
    m_pPlotGrid->setMajorPen(Qt::gray, 1);
    m_pPlotGrid->setMinorPen(Qt::gray, 1, Qt::DashLine);

    //main item on canvas -> the data object
    m_dObjItem = new DataObjItem("Data Object");
    m_dObjItem->setZ(10.0);
    m_dObjItem->setRenderThreadCount(0); //uses ideal thread count
    //m_dObjItem->setColorMap(new QwtLinearColorMap(QColor::fromRgb(0,0,0), QColor::fromRgb(255,255,255), QwtColorMap::Indexed));
    m_rasterData = new DataObjRasterData(m_pData);
    m_dObjItem->setData(m_rasterData);
    m_dObjItem->attach(this);
    m_colorContourMapName = "";

    //overlayobject item on canvas -> the data object
    m_dOverlayItem = new DataObjItem("Overlay Object");
    m_dOverlayItem->setZ(12.0);
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

    //picker for volume cut
    m_pVolumeCutPicker = new QwtPlotPicker(QwtPlot::xBottom, QwtPlot::yLeft, QwtPicker::CrossRubberBand, QwtPicker::AlwaysOn, canvas());
    m_pVolumeCutPicker->setEnabled(false);
    m_pVolumeCutPicker->setStateMachine(new QwtPickerDragPointMachine);
    m_pVolumeCutPicker->setRubberBandPen(QPen(Qt::green));
    m_pVolumeCutPicker->setTrackerPen(QPen(Qt::green));
    //disable key movements for the picker (the marker will be moved by the key-event of this widget)
    m_pVolumeCutPicker->setKeyPattern(QwtEventPattern::KeyLeft, 0);
    m_pVolumeCutPicker->setKeyPattern(QwtEventPattern::KeyRight, 0);
    m_pVolumeCutPicker->setKeyPattern(QwtEventPattern::KeyUp, 0);
    m_pVolumeCutPicker->setKeyPattern(QwtEventPattern::KeyDown, 0);
    connect(m_pVolumeCutPicker, SIGNAL(moved(const QPoint&)), SLOT(volumeCutMovedPx(const QPoint&)));
    connect(m_pVolumeCutPicker, SIGNAL(appended(const QPoint&)), SLOT(volumeCutAppendedPx(const QPoint&)));


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
    connect(m_pLineCutPicker, SIGNAL(moved(const QPoint&)), SLOT(lineCutMovedPx(const QPoint&)));
    connect(m_pLineCutPicker, SIGNAL(appended(const QPoint&)), SLOT(lineCutAppendedPx(const QPoint&)));

    //line for line picking
    m_pLineCutLine = new QwtPlotCurve();
    m_pLineCutLine->attach(this);
    m_pLineCutLine->setVisible(false);

    //line for volume picking
    m_pVolumeCutLine = new QwtPlotCurve();
    m_pVolumeCutLine->attach(this);
    m_pVolumeCutLine->setVisible(false);

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
    rightAxis->setSpacing(4);                       //distance tick labels <-> axis label && color bar -> ticks
    rightAxis->scaleDraw()->setSpacing(5);          //distance tick labels <-> ticks
    rightAxis->setContentsMargins(0,0,0,0);         //top and bottom offset

    configRescaler();

    QWidget *guiParent = parent;
    if (!guiParent) guiParent = this;


    //initialize actions
    QToolBar *mainTb = new QToolBar(tr("plotting tools"), guiParent);
    mainTb->setObjectName("mainToolBar");
    m_toolbars.append(mainTb);

    mainTb->addAction(m_pActSave);
    mainTb->addAction(m_pActPrint);
    mainTb->addSeparator();
    mainTb->addAction(m_pActHome);
    mainTb->addAction(m_pActProperties);
    mainTb->addAction(m_pActCamParameters);
    mainTb->addAction(m_pActPan);
    mainTb->addAction(m_pActZoom);
    mainTb->addAction(m_pActAspectRatio);
    mainTb->addAction(m_pActOverlaySlider);
    mainTb->addSeparator();
    mainTb->addAction(m_pActScaleSettings);
    mainTb->addAction(m_pActDataChannel);
    mainTb->addAction(m_pActToggleColorBar);
    mainTb->addAction(m_pActColorPalette);
    mainTb->addSeparator();
    mainTb->addAction(m_pActValuePicker);
    mainTb->addAction(m_pActCntrMarker);
    mainTb->addAction(m_pActLineCut);
    mainTb->addAction(m_pActVolumeCut);
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
    menuFile->addAction(m_pActPrint);
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
    menuView->addAction(m_pActDataChannel);
    menuView->addSeparator();
    menuView->addAction(m_pActScaleSettings);
    menuView->addSeparator();
    menuView->addAction(m_pActCmplxSwitch);
    menuView->addAction(m_pActGrid);
    menuView->addSeparator();
    menuView->addMenu(m_pMenuToolboxes);
    menuView->addSeparator();
    menuView->addAction(m_pActCamParameters);
    menuView->addAction(m_pActProperties);
    m_menus.append(menuView);

    QMenu *menuTools = new QMenu(tr("Tools"), guiParent);
    menuTools->addAction(m_pActValuePicker);
    menuTools->addAction(m_pActCntrMarker);
    menuTools->addAction(m_pActLineCut);
    menuTools->addAction(m_pActVolumeCut);
    menuTools->addAction(m_pActStackCut);
    menuTools->addSeparator();
    menuTools->addMenu(m_pMenuShapeType);
    menuTools->addAction(m_pActClearShapes);
    m_menus.append(menuTools);

    m_pContextMenu = new QMenu(QObject::tr("plot2D"), guiParent);
    m_pContextMenu->addAction(m_pActSave);
    m_pContextMenu->addAction(m_pActPrint);
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

	setAxisScaleEngine(QwtPlot::yRight, new QwtLinearScaleEngine());

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
ito::RetVal PlotCanvas::init(bool overwriteDesignableProperties)
{
    //since itom2dqwtplot is based on itomqwtplotbase, which also uses the concept of ITOM_API_FUNCS and ITOM_API_FUNCS_GRAPH,
    //itom only writes the global ITOM_API_FUNCS(_GRAPH) pointer in the base library 'itomqwtplotbase'. Therefore the following
    //lines are used to get the pointer from the base library and set it to the global variable of the same name, defined in this library.
    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        ito::ITOM_API_FUNCS_GRAPH = getItomApiFuncsPtrGraph();
    }

    if (!ito::ITOM_API_FUNCS)
    {
        ito::ITOM_API_FUNCS = getItomApiFuncsPtr();
    }

	if (ito::ITOM_API_FUNCS_GRAPH && m_pMenuColorPalette && (m_pMenuColorPalette->actions().size() == 0))
	{
		int numPalettes;
		ito::RetVal retval = apiPaletteGetNumberOfColorBars(numPalettes);

		if (!retval.containsError())
		{
			ito::ItomPalette palette;
			QAction *a;

			for (int i = 0; i < numPalettes; ++i)
			{
				if (!apiPaletteGetColorBarIdx(i, palette).containsError())
				{
					a = m_pMenuColorPalette->addAction(palette.name);
					a->setData(palette.name);
					a->setCheckable(true);
				}
			}
		}
	}

    if (!setColorMap("__first__"))
    {
        refreshStyles(overwriteDesignableProperties);
    }
    else
    {
        refreshStyles(overwriteDesignableProperties);
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::refreshStyles(bool overwriteDesignableProperties)
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

    ItomQwtPlot::loadStyles(overwriteDesignableProperties);

    QPen rubberBandPen = QPen(QBrush(Qt::red), 1, Qt::DashLine);
    QPen trackerPen = QPen(QBrush(Qt::red), 2);
    QFont trackerFont = QFont("Verdana", 10);
    QBrush trackerBg = QBrush(Qt::white, Qt::SolidPattern);
    QPen selectionPen = QPen(QBrush(Qt::gray), 2, Qt::SolidLine);
    QPen zStackMarkerPen = QPen(QBrush(inverseColor1()), 3);
    QSize zStackMarkerSize = QSize(7, 7);

    QFont titleFont = QFont("Verdana", 12);
    QFont labelFont = QFont("Verdana", 10);
    QFont axisFont = QFont("Verdana", 10);

    QSize centerMarkerSize = QSize(25, 25);
    QPen centerMarkerPen = QPen(QBrush(Qt::red), 1);
    QPen contourPen = QPen(Qt::black, 1.0, Qt::SolidLine);
    int buttonSet = buttonStyle();
    QString colorMap = "__first__";

    if(ito::ITOM_API_FUNCS_GRAPH)
    {
        rubberBandPen = apiGetFigureSetting(parent(), "zoomRubberBandPen", rubberBandPen, nullptr).value<QPen>();
        trackerPen = apiGetFigureSetting(parent(), "trackerPen", trackerPen, nullptr).value<QPen>(); //defines the color of the tracker
        trackerFont = apiGetFigureSetting(parent(), "trackerFont", trackerFont, nullptr).value<QFont>();
        trackerBg = apiGetFigureSetting(parent(), "trackerBackground", trackerBg, nullptr).value<QBrush>();
        selectionPen = apiGetFigureSetting(parent(), "selectionPen", selectionPen, nullptr).value<QPen>();

        centerMarkerSize = apiGetFigureSetting(parent(), "centerMarkerSize", centerMarkerSize, nullptr).value<QSize>();
        centerMarkerPen = apiGetFigureSetting(parent(), "centerMarkerPen", centerMarkerPen, nullptr).value<QPen>();
        zStackMarkerPen = apiGetFigureSetting(parent(), "zStackMarkerPen", zStackMarkerPen, nullptr).value<QPen>();
        zStackMarkerSize = apiGetFigureSetting(parent(), "zStackMarkerSize", zStackMarkerSize, nullptr).value<QSize>();

        if (overwriteDesignableProperties)
        {
            buttonSet = apiGetFigureSetting(parent(), "buttonSet", buttonSet, nullptr).value<int>(); //usually this property is only asked to inherit the buttonSet from the parent plot. //designable

            titleFont = apiGetFigureSetting(parent(), "titleFont", titleFont, nullptr).value<QFont>(); //designable
            labelFont = apiGetFigureSetting(parent(), "labelFont", labelFont, nullptr).value<QFont>(); //designable
            axisFont = apiGetFigureSetting(parent(), "axisFont", axisFont, nullptr).value<QFont>(); //designable

            colorMap = apiGetFigureSetting(parent(), "defaultColorMap", colorMap, nullptr).value<QString>();

            if (colorMap == "")
            {
                colorMap = "__first__";
            }

            setColorMap(colorMap);

            setKeepAspectRatio(apiGetFigureSetting(parent(), "keepAspectRatio", false, nullptr).value<bool>());
            m_pData->m_yaxisFlipped = apiGetFigureSetting(parent(), "yAxisFlipped", false, nullptr).value<bool>();
        }

    }

    if (inverseColor1().isValid())
    {
        rubberBandPen.setColor(inverseColor1());
    }

    if (inverseColor0().isValid())
    {
        selectionPen.setColor(inverseColor0());
        //trackerPen.setColor(inverseColor0());
        centerMarkerPen.setColor(inverseColor0());
        zStackMarkerPen.setColor(inverseColor0());
        m_pPlotGrid->setMajorPen(inverseColor0(), 1);
        m_pPlotGrid->setMinorPen(inverseColor0(), 1, Qt::DashLine);
        if (m_dObjItem)
        {
           contourPen = m_dObjItem->defaultContourPen();//hold setting from default pen
        }
        if (m_curContourColorMapIndex == -1)
        {
            contourPen.setColor(inverseColor1());
        }
    }


    m_pValuePicker->setTrackerFont(trackerFont);
    m_pValuePicker->setTrackerPen(trackerPen);
    m_pValuePicker->setBackgroundFillBrush(trackerBg);

    m_pLineCutPicker->setRubberBandPen(rubberBandPen);
    m_pLineCutPicker->setTrackerPen(trackerPen);
    m_pLineCutLine->setPen(selectionPen);
    m_pVolumeCutLine->setPen(selectionPen);
    m_pVolumeCutPicker->setRubberBandPen(rubberBandPen);
    m_pVolumeCutPicker->setTrackerPen(trackerPen);

    m_pStackPicker->setTrackerFont(trackerFont);
    m_pStackPicker->setTrackerPen(trackerPen);
    m_pStackPicker->setBackgroundFillBrush(trackerBg);

    m_pStackCutMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross, QBrush(inverseColor0()), zStackMarkerPen, zStackMarkerSize));

    m_pCenterMarker->setSymbol(new QwtSymbol(QwtSymbol::Cross,QBrush(/*m_inverseColor0*/), centerMarkerPen,  centerMarkerSize));
    if (m_dObjItem)
    {
        if (m_curContourColorMapIndex == -1)
        {
            m_dObjItem->setDefaultContourPen(contourPen);
        }
        else
        {
            contourPen.setStyle(Qt::NoPen);
            m_dObjItem->setDefaultContourPen(contourPen);
        }
    }
	if (!m_pPaletteIsChanging && overwriteDesignableProperties)
	{
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
	}


    //axisWidget(QwtPlot::yRight)->setLabelRotation(-90.0); //this rotates the tick values for the color bar ;)
    //axisScaleDraw(QwtPlot::yRight)->setLabelRotation(90); //this also ;)

    if ((buttonSet != buttonStyle()) && overwriteDesignableProperties)
    {
        setButtonStyle(buttonSet);
    }
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
    a->setToolTip(tr("Click to switch to next color palette"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuColorPalette()));

	m_pMenuColorPalette = new QMenu(tr("Color palettes"), p);
	m_pActColorPalette->setMenu(m_pMenuColorPalette);
	//items of m_pMenuColorPalette will be added in ::init()
	connect(m_pMenuColorPalette, SIGNAL(triggered(QAction*)), this, SLOT(mnuGroupColorPalette(QAction*)));

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

    //m_pActGrid
    m_pActGrid = a = new QAction(tr("Grid"), p);
    a->setObjectName("actGrid");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Shows/hides a grid"));

    m_pMnuGrid = new QMenu(tr("Grid"), p);
    m_pActGrid->setMenu(m_pMnuGrid);

    m_pMnuGrid = new QMenu(tr("Grid"), p);
    m_pActGrid->setMenu(m_pMnuGrid);

    a = m_pMnuGrid->addAction(tr("No Grid"));
    a->setData(Itom2dQwtPlot::GridNo);
    m_pMnuGrid->setDefaultAction(a);

    a = m_pMnuGrid->addAction(tr("Major XY"));
    a->setData(Itom2dQwtPlot::GridMajorXY);

    a = m_pMnuGrid->addAction(tr("Major X"));
    a->setData(Itom2dQwtPlot::GridMajorX);

    a = m_pMnuGrid->addAction(tr("Major Y"));
    a->setData(Itom2dQwtPlot::GridMajorY);

    a = m_pMnuGrid->addAction(tr("Minor XY"));
    a->setData(Itom2dQwtPlot::GridMinorXY);

    a = m_pMnuGrid->addAction(tr("Minor X"));
    a->setData(Itom2dQwtPlot::GridMinorX);

    a = m_pMnuGrid->addAction(tr("Minor Y"));
    a->setData(Itom2dQwtPlot::GridMinorY);

    connect(m_pMnuGrid, SIGNAL(triggered(QAction*)), this, SLOT(mnuSetGrid(QAction*)));

    //m_actToggleColorBar
    m_pActToggleColorBar = a = new QAction(tr("Show Colorbar"), p);
    a->setCheckable(true);
    a->setObjectName("actShowColorBar");
    a->setToolTip(tr("Toggle visibility of the color bar on the right side"));
    connect(a, SIGNAL(toggled(bool)), this, SLOT(mnuToggleColorBar(bool)));

    //m_pActDataChannel
    m_pActDataChannel = new QAction(tr("Data Channel"), p);
    m_pMnuDataChannel = new QMenu(tr("Data Channel"), p);
    m_pActDataChannel->setMenu(m_pMnuDataChannel);
    a = m_pMnuDataChannel->addAction(tr("Auto"));
    a->setData(ItomQwtPlotEnums::ChannelAuto);
    m_pMnuDataChannel->setDefaultAction(a);
    a = m_pMnuDataChannel->addAction(tr("Color"));
    a->setData(ItomQwtPlotEnums::ChannelRGBA);
    a = m_pMnuDataChannel->addAction(tr("Gray"));
    a->setData(ItomQwtPlotEnums::ChannelGray);
    a = m_pMnuDataChannel->addAction(tr("Red"));
    a->setData(ItomQwtPlotEnums::ChannelRed);
    a = m_pMnuDataChannel->addAction(tr("Green"));
    a->setData(ItomQwtPlotEnums::ChannelGreen);
    a = m_pMnuDataChannel->addAction(tr("Blue"));
    a->setData(ItomQwtPlotEnums::ChannelBlue);
    a = m_pMnuDataChannel->addAction(tr("Alpha"));
    a->setData(ItomQwtPlotEnums::ChannelAlpha);
    m_pActDataChannel->setVisible(false);
    connect(m_pMnuDataChannel, SIGNAL(triggered(QAction*)), this, SLOT(mnuDataChannel(QAction*)));

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

    //m_actVolumeCut
    m_pActVolumeCut = a = new QAction(tr("Volumecut"), p);
    a->setCheckable(true);
    a->setVisible(false);
    a->setObjectName("actVolumeCut");
    a->setToolTip(tr("Show a 2D volume cut"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuVolumeCut(bool)));

    //m_pActLineCutMode
    m_pMnuLineCutMode = new QMenu(tr("Linecut Mode"), p);
    m_pActLineCut->setMenu(m_pMnuLineCutMode);
    a = m_pMnuLineCutMode->addAction(tr("min && max"));
    a->setToolTip(tr("line cut through global minimum and maximum value"));
    a->setData(0);
//    a->setCheckable(true);

    a = m_pMnuLineCutMode->addAction(tr("- && min"));
    a->setToolTip(tr("horizontal line cut through global minimum value"));
    a->setData(1);
//    a->setCheckable(true);

    a = m_pMnuLineCutMode->addAction(tr("- && max"));
    a->setToolTip(tr("horizontal line cut through global maximum value"));
    a->setData(2);
//    a->setCheckable(true);

    a = m_pMnuLineCutMode->addAction(tr("| && min"));
    a->setToolTip(tr("vertical line cut through global minimum value"));
    a->setData(3);
//    a->setCheckable(true);

    a = m_pMnuLineCutMode->addAction(tr("| && max"));
    a->setToolTip(tr("vertical line cut through global maximum value"));
    a->setData(4);
//    a->setCheckable(true);
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
        m_pActVolumeCut->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/volumeCut.png"));
        m_pActToggleColorBar->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorbar.png"));
        m_pActValuePicker->setIcon(QIcon(":/itomDesignerPlugins/general/icons/crosshairs.png"));

        QList<QAction*> dataChannels = m_pMnuDataChannel->actions();
        dataChannels[0]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelAuto.png"));
        dataChannels[1]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelRgba.png"));
        dataChannels[2]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelGray.png"));
        dataChannels[3]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelRed.png"));
        dataChannels[4]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelGreen.png"));
        dataChannels[5]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelBlue.png"));
        dataChannels[6]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelAlpha.png"));
        m_pActDataChannel->setIcon(m_pMnuDataChannel->defaultAction()->icon());


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

        m_pActGrid->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/grid.png"));
    }
    else
    {
        m_pActScaleSettings->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/autoscal_lt.png"));
        m_pActColorPalette->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/colorPalette_lt.png"));
        m_pActCntrMarker->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/markerCntr_lt.png"));
        m_pActStackCut->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/zStack_lt.png"));
        m_pActLineCut->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/pntline_lt.png"));
        m_pActVolumeCut->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/volumeCut_lt.png"));
        m_pActToggleColorBar->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/colorbar_lt.png"));
        m_pActValuePicker->setIcon(QIcon(":/itomDesignerPlugins/general_lt/icons/marker_lt.png"));
        m_pActDataChannel->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/rgba_lt.png"));

        QList<QAction*> dataChannels = m_pMnuDataChannel->actions();
        dataChannels[0]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelAuto.png"));
        dataChannels[1]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelRgba.png"));
        dataChannels[2]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelGray.png"));
        dataChannels[3]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelRed.png"));
        dataChannels[4]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelGreen.png"));
        dataChannels[5]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelBlue.png"));
        dataChannels[6]->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/colorChannelAlpha.png"));
        m_pActDataChannel->setIcon(m_pMnuDataChannel->defaultAction()->icon());

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

        m_pActGrid->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/grid_lt.png"));
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
template <typename _Tp> void PlotCanvas::parseVolumeCutObj( const ito::DataObject* srcObj, const unsigned int& offsetByte, const QVector<int>& stepByte)
{
    if(m_dir == dirX || m_dir == dirY)
    {
        unsigned char* val=NULL;
        int i;
        const cv::Mat** srcCvMatVec= srcObj->get_mdata();
        cv::Mat* dstMat = m_dObjVolumeCut.getCvPlaneMat(0);
        _Tp* dstPtr = (_Tp*)(dstMat->data);
        int firstMatOff = srcObj->seekMat(0);
        for (int layer = 0; layer < dstMat->rows; ++layer)
        {
            val= srcCvMatVec[firstMatOff +layer]->data+offsetByte; //first element of host volume cut
            for (i = 0; i < dstMat->cols; i++)
            {
                *dstPtr++ =*reinterpret_cast<_Tp*>(val+i*stepByte[0]);

            }
        }
    }
    else if(m_dir == dirXY)
    {
        uchar* val=NULL;
        int i;
        int firstMatOff = srcObj->seekMat(0);
        const cv::Mat** srcCvMatVec= srcObj->get_mdata();
        cv::Mat* dstMat = m_dObjVolumeCut.getCvPlaneMat(0);
        _Tp* dstPtr = (_Tp*)(dstMat->data);
        for (int layer = 0; layer < dstMat->rows; ++layer)
        {
            val= srcCvMatVec[firstMatOff+layer]->data+offsetByte; //first element of host volume cut
            for (i = 0; i < dstMat->cols; i++)
            {
                *dstPtr++ = *reinterpret_cast<_Tp*>(val+stepByte[i]);

            }
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotCanvas::cutVolume(const ito::DataObject* dataObj, const QVector<QPointF> bounds)
{
    ito::RetVal retval;
    if (bounds.size() == 0)
    {
        m_dir = inPlane;
    }
    if (bounds.size() == 2)
    {
        std::string description, unit;
        int dims = dataObj->getDims();
        unsigned int offsetByte;
        QVector<int> stepByte; //step to be done to next elem

        // iterIdx is the axis index in the original dataObj that is used for the
        // y-axis of the volume cut object. iterIdx is the first index of the first
        // n-2 dimensions, whose size is > 1. If not possible, it is 0.
        int iterIdx = 0;

        for (int i = 0; i < dims - 2; ++i)
        {
            if (dataObj->getSize(i) > 1)
            {
                iterIdx = i;
                break;
            }
        }

        int pxX1, pxX2, pxY1, pxY2, xSize, ySize;
        double yScaling, xScaling, yOffset, xOffset;
        bool _unused;
        pxX1 = qRound(dataObj->getPhysToPix(dims - 1, bounds[0].x(), _unused));
        pxY1 = qRound(dataObj->getPhysToPix(dims - 2, bounds[0].y(), _unused));
        pxX2 = qRound(dataObj->getPhysToPix(dims - 1, bounds[1].x(), _unused));
        pxY2 = qRound(dataObj->getPhysToPix(dims - 2, bounds[1].y(), _unused));

        saturation(pxX1, 0, dataObj->getSize(dims - 1) - 1);
        saturation(pxX2, 0, dataObj->getSize(dims - 1) - 1);
        saturation(pxY1, 0, dataObj->getSize(dims - 2) - 1);
        saturation(pxY2, 0, dataObj->getSize(dims - 2) - 1);

        const cv::Mat* mat = dataObj->getCvPlaneMat(dataObj->seekMat(0)); //first plane in ROI

        if (pxX2 == pxX1)
        {

            m_dir = dirY;
            stepByte.resize(1);

            yScaling = dims > 2 ? dataObj->getAxisScale(iterIdx) : 1.0; // scaling along z of the host object
            xScaling = dims > 2 ? dataObj->getAxisScale(dims - 2) : 1.0; // scaling along y of the host object
            yOffset = dims > 2 ? dataObj->getAxisOffset(iterIdx) : 0.0;
            xOffset = dims > 2 ? dataObj->getAxisOffset(dims - 2) : 0.0;
            ySize = dims > 2 ? dataObj->getSize(iterIdx) : 0;

            if (pxY2 >= pxY1)
            {
                xSize = 1 + pxY2 - pxY1;
            }
            else
            {
                xSize = 1 + pxY1 - pxY2;
            }

            offsetByte = pxX1 * dataObj->get_mdata()[0]->step[1] + pxY1 * dataObj->get_mdata()[0]->step[0];

            if (pxY1 < pxY2)
            {
                stepByte[0] = dataObj->get_mdata()[0]->step[0];
            }
            else // go in negative direction
            {
                stepByte[0] = -static_cast<int>(dataObj->get_mdata()[0]->step[0]);
            }

            m_dObjVolumeCut = ito::DataObject(ySize, xSize, dataObj->getType());

            switch(dataObj->getType())
            {
            case(ito::tUInt8):
                parseVolumeCutObj<ito::uint8>(dataObj,offsetByte,stepByte);
                break;
            case(ito::tUInt16):
                parseVolumeCutObj<ito::uint16>(dataObj,offsetByte,stepByte);
                break;
            case(ito::tInt8):
                parseVolumeCutObj<ito::int8>(dataObj,offsetByte,stepByte);
                break;
            case(ito::tInt16):
                parseVolumeCutObj<ito::int16>(dataObj,offsetByte,stepByte);
                break;
            case(ito::tInt32):
                parseVolumeCutObj<ito::int32>(dataObj,offsetByte,stepByte);
                break;
            case(ito::tFloat32):
                parseVolumeCutObj<ito::float32>(dataObj, offsetByte, stepByte);
                break;
            case(ito::tFloat64):
                parseVolumeCutObj<ito::float64>(dataObj, offsetByte, stepByte);
                break;
            case(ito::tComplex64):
                parseVolumeCutObj<ito::complex64>(dataObj,offsetByte,stepByte);
                break;
            case(ito::tComplex128):
                parseVolumeCutObj<ito::complex128>(dataObj,offsetByte,stepByte);
                break;
            case(ito::tRGBA32):
                parseVolumeCutObj<ito::Rgba32>(dataObj,offsetByte,stepByte);
                break;
            default:
                retval += ito::RetVal(ito::retError, 0, tr("type not implemented yet").toLatin1().data());
            }

            description = dataObj->getAxisDescription(dims - 2, _unused);
            unit = dataObj->getAxisUnit(dims - 2, _unused);

            if (description == "")
            {
                description = QObject::tr("y-axis").toLatin1().data();
            }

            m_dObjVolumeCut.setAxisDescription(1, description);
            m_dObjVolumeCut.setAxisUnit(1, unit);

            description = dataObj->getAxisDescription(iterIdx, _unused);
            unit = dataObj->getAxisUnit(iterIdx, _unused);

            if (description == "")
            {
                if (iterIdx == dims - 3)
                {
                    description = QObject::tr("z-axis").toLatin1().data();
                }
                else
                {
                    description = QObject::tr("axis %1").arg(iterIdx).toLatin1().data();
                }
            }

            m_dObjVolumeCut.setAxisDescription(0, description);
            m_dObjVolumeCut.setAxisUnit(0, unit);
            m_dObjVolumeCut.setValueUnit(dataObj->getValueUnit());
            m_dObjVolumeCut.setValueDescription(dataObj->getValueDescription());

            m_dObjVolumeCut.setAxisOffset(0, dataObj->getAxisOffset(dims - 3));
            m_dObjVolumeCut.setAxisScale(0, dataObj->getAxisScale(dims - 3));

            double startPhys = dataObj->getPixToPhys(dims - 2, pxY1, _unused);
            double right = dataObj->getPixToPhys(dims - 2, pxY2, _unused);
            double scale = xSize > 1 ? (right - startPhys) / (float)(xSize - 1) : 0.0;

            m_dObjVolumeCut.setAxisScale(1, scale);
            m_dObjVolumeCut.setAxisOffset(1, -startPhys/scale);
        }
        else if (pxY2 == pxY1) // pure x
        {
            m_dir = dirX;
            stepByte.resize(1);
            yScaling = dims > 2 ? dataObj->getAxisScale(iterIdx) : 1.0; // scaling along z of the host object
            xScaling = dims > 2 ? dataObj->getAxisScale(dims - 1) : 1.0; // scaling along y of the host object
            yOffset = dims > 2 ? dataObj->getAxisOffset(iterIdx) : 0.0;
            xOffset = dims > 2 ? dataObj->getAxisOffset(dims - 1) : 0.0;
            ySize = dims > 2 ? dataObj->getSize(iterIdx) : 0;

            if (pxX2 >= pxX1)
            {
                xSize = 1 + pxX2 - pxX1;
            }
            else
            {
                xSize = 1 + pxX1 - pxX2;
            }

            offsetByte=pxX1 * dataObj->get_mdata()[0]->step[1] + pxY1 * dataObj->get_mdata()[0]->step[0];

            if (pxX1 < pxX2)
            {
                stepByte[0] = dataObj->get_mdata()[0]->step[1];
            }
            else// go in negative direction
            {
                stepByte[0] = -static_cast<int>(dataObj->get_mdata()[0]->step[1]);
            }
            // todo: check if this DataObject is deleted when the plot is closed

            m_dObjVolumeCut = ito::DataObject(ySize, xSize, dataObj->getType());

            switch(dataObj->getType())
            {
            case(ito::tUInt8):
                parseVolumeCutObj<ito::uint8>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tUInt16):
                parseVolumeCutObj<ito::uint16>(dataObj,offsetByte, stepByte);;
                break;
            case(ito::tInt8):
                parseVolumeCutObj<ito::int8>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tInt16):
                parseVolumeCutObj<ito::int16>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tInt32):
                parseVolumeCutObj<ito::int32>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tFloat32):
                parseVolumeCutObj<ito::float32>(dataObj, offsetByte, stepByte);
                break;
            case(ito::tFloat64):
                parseVolumeCutObj<ito::float64>(dataObj, offsetByte, stepByte);
                break;
            case(ito::tComplex64):
                parseVolumeCutObj<ito::complex64>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tComplex128):
                parseVolumeCutObj<ito::complex128>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tRGBA32):
                parseVolumeCutObj<ito::Rgba32>(dataObj,offsetByte, stepByte);
                break;
            default:
                retval += ito::RetVal(ito::retError, 0, tr("type not implemented yet").toLatin1().data());
            }

            description = dataObj->getAxisDescription(dims - 1, _unused);
            unit = dataObj->getAxisUnit(dims - 1, _unused);

            if (description == "")
            {
                description = QObject::tr("x-axis").toLatin1().data();
            }

            m_dObjVolumeCut.setAxisDescription(1, description);
            m_dObjVolumeCut.setAxisUnit(1, unit);

            description = dataObj->getAxisDescription(iterIdx, _unused);
            unit = dataObj->getAxisUnit(iterIdx, _unused);

            if (description == "")
            {
                if (iterIdx == dims - 3)
                {
                    description = QObject::tr("z-axis").toLatin1().data();
                }
                else
                {
                    description = QObject::tr("axis %1").arg(iterIdx).toLatin1().data();
                }
            }

            m_dObjVolumeCut.setAxisDescription(0, description);
            m_dObjVolumeCut.setAxisUnit(0, unit);
            m_dObjVolumeCut.setValueUnit(dataObj->getValueUnit());
            m_dObjVolumeCut.setValueDescription(dataObj->getValueDescription());

            m_dObjVolumeCut.setAxisOffset(0, dataObj->getAxisOffset(iterIdx));
            m_dObjVolumeCut.setAxisScale(0, dataObj->getAxisScale(iterIdx));

            double startPhys = dataObj->getPixToPhys(dims - 1, pxX1, _unused);
            double right = dataObj->getPixToPhys(dims - 1, pxX2, _unused);
            double scale = xSize > 1 ? (right - startPhys) / (float)(xSize - 1) : 0.0;
            m_dObjVolumeCut.setAxisScale(1, xSize > 1 ? (right - startPhys) / (float)(xSize - 1) : 0.0);
            m_dObjVolumeCut.setAxisOffset(1, -startPhys/scale);
        }
        else
        {
            m_dir=dirXY;
            int dx = abs(pxX2 - pxX1);
            int incx = pxX1 <= pxX2 ? 1 : -1;
            int dy = abs(pxY2 - pxY1);
            int incy = pxY1 <= pxY2 ? 1 : -1;

            xSize = 1 + std::max(dx,dy);

            //m_d.startPhys= 0.0;  //there is no physical starting point for diagonal lines.
            yScaling = dims > 2 ? dataObj->getAxisScale(iterIdx) : 1.0;
            yOffset = dims > 2 ? dataObj->getAxisOffset(iterIdx) : 0.0;
            ySize = dims > 2 ? dataObj->getSize(iterIdx) : 0;

            if (xSize > 0)
            {
                double dxPhys = dataObj->getPixToPhys(dims - 1, pxX2, _unused) - dataObj->getPixToPhys(dims-1, pxX1, _unused);
                double dyPhys = dataObj->getPixToPhys(dims - 2, pxY2, _unused) - dataObj->getPixToPhys(dims-2, pxY1, _unused);
                xScaling = sqrt((dxPhys * dxPhys) + (dyPhys * dyPhys)) / (xSize - 1);
            }
            else
            {
                xScaling = 0.0;
            }

            offsetByte = pxX1 * dataObj->get_mdata()[0]->step[1] + pxY1 * dataObj->get_mdata()[0]->step[0];

            stepByte.resize(xSize);
            int pdx, pdy, ddx, ddy, es, el;

            if (dx>dy)
            {
                pdx = incx;
                pdy = 0;
                ddx = incx;
                ddy = incy;
                es = dy;
                el = dx;
            }
            else
            {
                pdx = 0;
                pdy = incy;
                ddx = incx;
                ddy = incy;
                es = dx;
                el = dy;
            }

            int err = el / 2; //0; /* error value e_xy */
            int x = 0; //pxX1;
            int y = 0; //pxY1;

            for (unsigned int n = 0; n < (unsigned int)xSize; n++)
            {  /* loop */
                //setPixel(x,y)
                stepByte[n] = (int)mat->step[0] * y + (int)mat->step[1] * x;
                err -= es;

                if (err < 0)
                {
                    err += el;
                    x += ddx;
                    y += ddy;
                }
                else
                {
                    x += pdx;
                    y += pdy;
                }
            }

            m_dObjVolumeCut = ito::DataObject(ySize, xSize, dataObj->getType());

            switch(dataObj->getType())
            {
            case(ito::tUInt8):
                parseVolumeCutObj<ito::uint8>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tUInt16):
                parseVolumeCutObj<ito::uint16>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tInt8):
                parseVolumeCutObj<ito::int8>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tInt16):
                parseVolumeCutObj<ito::int16>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tInt32):
                parseVolumeCutObj<ito::int32>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tFloat32):
                parseVolumeCutObj<ito::float32>(dataObj, offsetByte, stepByte);
                break;
            case(ito::tFloat64):
                parseVolumeCutObj<ito::float64>(dataObj, offsetByte, stepByte);
                break;
            case(ito::tComplex64):
                parseVolumeCutObj<ito::complex64>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tComplex128):
                parseVolumeCutObj<ito::complex128>(dataObj,offsetByte, stepByte);
                break;
            case(ito::tRGBA32):
                parseVolumeCutObj<ito::Rgba32>(dataObj,offsetByte, stepByte);
                break;
            default:
                retval += ito::RetVal(ito::retError, 0, tr("type not implemented yet").toLatin1().data());
            }

            description = dataObj->getAxisDescription(dims - 2, _unused);
            unit = dataObj->getAxisUnit(dims - 2, _unused);

            if (unit == "") unit = "px";

            std::string descr2 = dataObj->getAxisDescription(dims - 1, _unused);
            std::string unit2 = dataObj->getAxisUnit(dims - 1, _unused);

            if (unit2 == "") unit2 = "px";

            if (description == "" && descr2 == "")
            {
                    m_dObjVolumeCut.setAxisDescription(1, tr("x/y-axis").toLatin1().data());
                    m_dObjVolumeCut.setAxisUnit(1,QString("%1/%2").arg(QString::fromLatin1(unit.data()), QString::fromLatin1(unit2.data())).toLatin1().data());
            }
            else
            {
                m_dObjVolumeCut.setAxisDescription(1,QString("%1/%2").arg(QString::fromLatin1(description.data()), QString::fromLatin1(descr2.data())).toLatin1().data());
                m_dObjVolumeCut.setAxisUnit(1, QString("%1/%2").arg(QString::fromLatin1(unit.data()), QString::fromLatin1(unit2.data())).toLatin1().data());
            }

            description = dataObj->getAxisDescription(iterIdx, _unused);
            unit = dataObj->getAxisUnit(iterIdx, _unused);

            if (description == "")
            {
                if (iterIdx == dims - 3)
                {
                    description = QObject::tr("z-axis").toLatin1().data();
                }
                else
                {
                    description = QObject::tr("axis %1").arg(iterIdx).toLatin1().data();
                }
            }

            m_dObjVolumeCut.setAxisDescription(0, description);
            m_dObjVolumeCut.setAxisUnit(0, unit);
            m_dObjVolumeCut.setValueUnit(dataObj->getValueUnit());
            m_dObjVolumeCut.setValueDescription(dataObj->getValueDescription());
            m_dObjVolumeCut.setAxisScale(1, xScaling);
            m_dObjVolumeCut.setAxisOffset(0, dataObj->getAxisOffset(iterIdx));
            m_dObjVolumeCut.setAxisScale(0, dataObj->getAxisScale(iterIdx));
        }
    }

    return retval;
}
//---------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::refreshPlot(const ito::DataObject *dObj,int plane /*= -1*/, const QVector<QPointF> bounds /*=QVector<QPointF>()*/ )
{
    ito::RetVal retval;

    if (m_isRefreshingPlot || !m_pData)
    {
        return;
    }

    int dims = 0;

    // the 2d plot does not accept a datetime or timedelta object
    if (dObj)
    {
        dims = dObj->getDims();

        switch (dObj->getType())
        {
        case ito::tDateTime:
        case ito::tTimeDelta:
            dObj = nullptr;
            emit statusBarMessage(QObject::tr("Objects of type dateTime or timeDelta not supported by this plot."), 10000);
            break;
        }
    }

    m_isRefreshingPlot = true;

    ito::uint8 updateState = 0; //changeNo (0): nothing changed, changeAppearance (1): appearance changed (yAxisFlipped, cmplxFlag, plane...), changeData (2): data changed (dimensions, sizes, other data object...)

    m_dObjPtr = dObj;

    //QString valueLabel, axisLabel, title;
    if (dObj)
    {
        retval+=cutVolume(dObj, bounds);
        if (!retval.containsError())
        {
            if (m_dir != inPlane)
            {
                dObj = &m_dObjVolumeCut;
                m_dObjPtr = &m_dObjVolumeCut;
                dims = dObj->getDims();
            }
        }
        else
        {
            emit statusBarMessage(QObject::tr(retval.errorMessage()).toLatin1().data(), 10000);
        }

        int width = dims > 0 ? dObj->getSize(dims - 1) : 0;
        int height = dims > 1 ? dObj->getSize(dims - 2) : 1;

        updateState = m_rasterData->updateDataObject(dObj, plane);

        if ((updateState & changeData) || m_unitLabelChanged)
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
			tr("2D Object Plot"),
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
            int type = dObj->getType();

            if (type != m_currentDataType)
            {
                switch (dObj->getType())
                {
                case ito::tRGBA32:
                    //coloured objects have no color bar, therefore the value axis cropping is disabled
                    adjustColorDataTypeRepresentation();
                    m_pData->m_valueScaleAuto = false;
                    m_pData->m_valueMin = std::numeric_limits<ito::uint8>::min();
                    m_pData->m_valueMax = std::numeric_limits<ito::uint8>::max();
                    m_pActCmplxSwitch->setVisible(false);
                    break;
                case ito::tComplex64:
                case ito::tComplex128:
                    adjustColorDataTypeRepresentation();
                    m_pActCmplxSwitch->setVisible(true);
                    break;
                default:
                    adjustColorDataTypeRepresentation();
                    m_pActCmplxSwitch->setVisible(false);
                    break;
                }

                m_currentDataType = type;
            }

            if (dims)
            {
                int numFirstDimsGreaterThan1 = 0;

                for (int i = 0; i < dims - 2; ++i)
                {
                    // a zCut and volumeCut is only possible if only one
                    // dimension of the first dims-2 dimensions has a size != 1.
                    if (dObj->getSize(i) > 1)
                    {
                        numFirstDimsGreaterThan1++;
                    }
                }

                p->setPlaneRange(0, dObj->calcNumMats() - 1, numFirstDimsGreaterThan1 <= 1);
            }
            else
            {
                p->setPlaneRange(0, 0);
            }
        }

        updateScaleValues(false, updateState & changeData); //no replot here
        updateAxes();
        updateZoomOptionState();


        //set the base view for the zoomer (click on 'house' symbol) to the current representation (only if data changed)
        if (updateState & changeData)
        {
            QStack<QRectF> stack; //we can not call setZoomBase since this scales to the current scalingRect, leading to a wrong zoomStack in case of keepAspectRatio is set by the api
            QRectF boundingRect(
                m_pData->m_xaxisMin,
                m_pData->m_yaxisMin,
                (m_pData->m_xaxisMax - m_pData->m_xaxisMin),
                (m_pData->m_yaxisMax - m_pData->m_yaxisMin));
            stack.push(boundingRect);
            zoomer()->setZoomStack(stack, -1);
            updateZoomOptionState();
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
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::adjustColorDataTypeRepresentation()
{
    if (!m_pData)
        return;

    if (m_dObjPtr && \
        (m_dObjPtr->getType() == ito::tRGBA32) && \
        ((m_pData->m_dataChannel & ItomQwtPlotEnums::ChannelGray) == 0))
    {
        setColorBarVisible(false);
        m_pActColorPalette->setVisible(false);
        m_pActToggleColorBar->setVisible(false);
        m_pActDataChannel->setVisible(true);
        if (m_dObjItem)
        {
            m_dObjItem->setDisplayMode(QwtPlotSpectrogram::ContourMode, false);
        }
    }
    else
    {
        m_pActColorPalette->setVisible(true);
        m_pActToggleColorBar->setVisible(true);
        setColorBarVisible(m_pActToggleColorBar->isChecked());
        m_pActDataChannel->setVisible(m_dObjPtr->getType() == ito::tRGBA32);
        if (m_dObjItem)
        {
            //only enable contour mode, if contour levels available
            m_dObjItem->setDisplayMode(QwtPlotSpectrogram::ContourMode, m_dObjItem->contourLevels().size() > 0);
        }
    }

    if (m_pMnuDataChannel->defaultAction()->data().toInt() != m_pData->m_dataChannel)
    {
        foreach (QAction *a, m_pMnuDataChannel->actions())
        {
            if (a->data().toInt() == m_pData->m_dataChannel)
            {
                m_pMnuDataChannel->setDefaultAction(a);
                m_pActDataChannel->setIcon(m_pMnuDataChannel->defaultAction()->icon());
                break;
            }
        }
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
bool PlotCanvas::setColorMap(const QString &colormap /*= "__next__"*/)
{
    ItomColorMap *colorMap = NULL;
	ItomColorMap *colorBarMap = NULL;
    ito::ItomPalette newPalette;
    ito::RetVal retval(ito::retOk);
    int numPalettes = 1;

    if(!ito::ITOM_API_FUNCS_GRAPH)
    {
        //emit statusBarMessage(tr("Could not change color bar, api is missing"), 4000);
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
	else if (colormap == "__equal__")
	{
		retval += apiPaletteGetColorBarIdx(m_curColorMapIndex, newPalette);
	}
    else
    {
		ito::int32 idx;
		retval += apiPaletteGetColorBarIdxFromName(colormap, idx);
        retval += apiPaletteGetColorBarIdx(idx, newPalette);
		if (!retval.containsError())
		{
			m_curColorMapIndex = idx;
		}
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

	//refresh check state of all color palette sub items in menu
	if (m_pMenuColorPalette)
	{
		QList<QAction*> actions = m_pMenuColorPalette->actions();
		for (int i = 0; i < actions.size(); ++i)
		{
			actions[i]->setChecked(actions[i]->data().toString() == m_colorMapName);
		}
	}

    setInverseColors(newPalette.inverseColorOne, newPalette.inverseColorTwo);
	m_pPaletteIsChanging = true;
	refreshStyles(false);
	m_pPaletteIsChanging = false;

    if (newPalette.colorStops[totalStops - 1].first == newPalette.colorStops[totalStops - 2].first)  // BuxFix - For Gray-Marked
    {
        colorMap    = new ItomColorMap(newPalette.colorStops[0].second, newPalette.colorStops[totalStops - 2].second, m_valueScale, QwtColorMap::Indexed);
		colorBarMap = new ItomColorMap(newPalette.colorStops[0].second, newPalette.colorStops[totalStops - 2].second, m_valueScale, QwtColorMap::Indexed);
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
		colorMap = new ItomColorMap(newPalette.colorStops.first().second, newPalette.colorStops.last().second, m_valueScale, QwtColorMap::Indexed);
		colorBarMap = new ItomColorMap(newPalette.colorStops.first().second, newPalette.colorStops.last().second, m_valueScale, QwtColorMap::Indexed);
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
bool PlotCanvas::setOverlayColorMap(const QString &colormap /*= "__next__"*/)
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
        emit statusBarMessage(tr("error when loading color map"), 4000);
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
void PlotCanvas::setValueAxisScaleEngine(const ItomQwtPlotEnums::ScaleEngine &scaleEngine)
{
	if (scaleEngine != m_valueScale)
	{
		if (scaleEngine == ItomQwtPlotEnums::Linear)
		{
			setAxisScaleEngine(QwtPlot::yRight, new QwtLinearScaleEngine());
		}
		else if ((int)scaleEngine < (int)ItomQwtPlotEnums::LogLog2)
		{
			setAxisScaleEngine(QwtPlot::yRight, new QwtLogScaleEngine((int)scaleEngine));
		}
		else
		{
			setAxisScaleEngine(QwtPlot::yRight, new ItomLogLogScaleEngine((int)scaleEngine - 1000));
		}

		m_valueScale = scaleEngine;

		setColorMap("__equal__");

		replot();
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setShowCenterMarker(bool show)
{
    m_showCenterMarker = show;
    setState(state());
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setGridStyle(Itom2dQwtPlot::GridStyle gridStyle)
{
    m_gridStyle = gridStyle;
    m_pPlotGrid->enableX(gridStyle == Itom2dQwtPlot::GridMajorX || gridStyle == Itom2dQwtPlot::GridMajorXY || gridStyle == Itom2dQwtPlot::GridMinorX || gridStyle == Itom2dQwtPlot::GridMinorXY);
    m_pPlotGrid->enableY(gridStyle == Itom2dQwtPlot::GridMajorY || gridStyle == Itom2dQwtPlot::GridMajorXY || gridStyle == Itom2dQwtPlot::GridMinorY || gridStyle == Itom2dQwtPlot::GridMinorXY);
    m_pPlotGrid->enableXMin(gridStyle == Itom2dQwtPlot::GridMinorX || gridStyle == Itom2dQwtPlot::GridMinorXY);
    m_pPlotGrid->enableYMin(gridStyle == Itom2dQwtPlot::GridMinorY || gridStyle == Itom2dQwtPlot::GridMinorXY);
    m_pActGrid->setChecked(gridStyle != Itom2dQwtPlot::GridNo);

    foreach(QAction* a, m_pMnuGrid->actions())
    {
        if (a->data().toInt() == gridStyle)
        {
            m_pMnuGrid->setDefaultAction(a);
        }
    }

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

            ito::uint32 childFigureUID;
            p->displayZStackCut(pts, &childFigureUID);

			if (((Itom2dQwtPlot*)parent())->pickerWidget() && childFigureUID > 0)
			{
				(((Itom2dQwtPlot*)parent())->pickerWidget())->updateChildPlot(childFigureUID, ito::Shape::Point, QVector4D(pts[0]));
			}

            setCoordinates(pts, true);

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
        else if (m_pLineCutLine->dataSize() >= 2)
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
        else
        {
            event->ignore();
        }

        if (event->isAccepted() && m_rasterData->pointValid(pts[0]) && m_rasterData->pointValid(pts[1]))
        {
            m_pLineCutLine->setSamples(pts);
            m_pLineCutLine->setVisible(true);

            if(m_dObjPtr && m_dObjPtr->getDims() > 2)
            {
                pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(),m_rasterData->getCurrentPlane()));
            }

            ito::uint32 childFigureUID;
            Itom2dQwtPlot* p = (Itom2dQwtPlot*)parent();

            p->displayLineCut(pts, &childFigureUID);

			if (p->pickerWidget() && childFigureUID > 0)
			{
				QVector4D vec;
                if (pts.size() == 3)
                {
                    vec = QVector4D(pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
                }
                else
                {
                    vec = QVector4D(pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
                }

				p->pickerWidget()->updateChildPlot(childFigureUID, ito::Shape::Line, vec);
			}

            replot();
        }
    }
    else if (state() == stateVolumeCut)
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
        else if (m_pVolumeCutLine->dataSize() >= 2)
        {
            pts.append(m_pVolumeCutLine->sample(0));
            pts.append(m_pVolumeCutLine->sample(1));

            switch (event->key())
            {
            case Qt::Key_Left:
                pts[0].rx() -= incr.rx();
                pts[1].rx() -= incr.rx();
                break;
            case Qt::Key_Right:
                pts[0].rx() += incr.rx();
                pts[1].rx() += incr.rx();
                break;
            case Qt::Key_Up:
                pts[0].ry() -= incr.ry();
                pts[1].ry() -= incr.ry();
                break;
            case Qt::Key_Down:
                pts[0].ry() += incr.ry();
                pts[1].ry() += incr.ry();
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
        else
        {
            event->ignore();
        }

        if (event->isAccepted() && m_rasterData->pointValid(pts[0]) && m_rasterData->pointValid(pts[1]))
        {
            m_pVolumeCutLine->setSamples(pts);
            m_pVolumeCutLine->setVisible(true);

            p->displayVolumeCut(pts);

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
    if (!m_pData)
        return;

    m_pData->m_colorBarVisible = visible;
    enableAxis(QwtPlot::yRight, visible);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::updateLabels()
{
    if (!m_pData)
        return;

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
        ((ito::AbstractFigure*)(parent()))->setWindowTitleExtension(m_pData->m_titleDObj);
    }
    else
    {
        setTitle(m_pData->m_title);
        ((ito::AbstractFigure*)(parent()))->setWindowTitleExtension(m_pData->m_title);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::synchronizeScaleValues()
{
    if (!m_pData)
        return;

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

//---------------------------------------------------------------------------------
//!< if an interval has size of 0, the automatically determined scale will have strange values.
void fixZeroInterval(QwtInterval &interval)
{
    auto dist = qAbs(interval.maxValue() - interval.minValue());

    if (dist < std::numeric_limits<double>::epsilon())
    {
        interval = QwtInterval(interval.minValue() - 0.5, interval.maxValue() + 0.5);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
/*
@param doReplot forces a replot of the content
@param doZoomBase if true, the x/y-zoom is reverted to the full x-y-area of the manually set ranges (the same holds for the value range)
*/
void PlotCanvas::updateScaleValues(bool doReplot /*= true*/, bool doZoomBase /*= true*/, bool clearStack /*=false*/)
{
    Q_UNUSED(clearStack);

    if (!m_pData)
        return;

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
        fixZeroInterval(ival);
        m_pData->m_xaxisMin = ival.minValue();
        m_pData->m_xaxisMax = ival.maxValue();
    }

    if (m_pData->m_yaxisScaleAuto)
    {
        ival = m_rasterData->interval(Qt::YAxis);
        fixZeroInterval(ival);
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
            QRectF rect = zoomer()->zoomRect();
            bool isEqualRect = true;
            qreal x1, x2, y1, y2 = 0;
            qreal zoomx1, zoomx2, zoomy1, zoomy2 = 0;
            rect.getCoords(&x1, &y1, &x2, &y2);
            zoom.getCoords(&zoomx1, &zoomy1, &zoomx2, &zoomy2);

            isEqualRect = (x1 - zoomx1) < std::numeric_limits<qreal>::epsilon();
            isEqualRect &= (y1 - zoomy1) < std::numeric_limits<qreal>::epsilon();
            isEqualRect &= (y2 - zoomy2) < std::numeric_limits<qreal>::epsilon();
            isEqualRect &= (x2 - zoomx2) < std::numeric_limits<qreal>::epsilon();

            if (!isEqualRect)
            {
                zoomer()->zoom(zoom);
            }
            zoomer()->rescale(false); //zoom of zoomer does not call rescale in this case, therefore we do it here
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
    QRectF boundingRect(
        m_pData->m_xaxisMin,
        m_pData->m_yaxisMin,
        (m_pData->m_xaxisMax - m_pData->m_xaxisMin),
        (m_pData->m_yaxisMax - m_pData->m_yaxisMin));
    QStack<QRectF> stack = zoomer()->zoomStack();
    if (boundingRect != zoomer()->zoomRect())
    {
        zoomer()->zoom(boundingRect);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::zoomUndo() const
{
    const unsigned int index = zoomer()->zoomRectIndex();
    if (index > 0)
    {
        zoomer()->zoom(-1);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::zoomRedo() const
{
    const unsigned int index = zoomer()->zoomRectIndex();
    if (index < zoomer()->zoomStack().length()-1)
    {
        zoomer()->zoom(1);
    }

}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setInterval(Qt::Axis axis, const ito::AutoInterval &interval)
{
    if (!m_pData)
        return;

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
    if (!m_pData)
        return;

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
    if (!m_pData)
        return ito::AutoInterval();

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
    if (m_pVolumeCutPicker) m_pVolumeCutPicker->setEnabled(state == stateVolumeCut);
    if (m_pStackPicker) m_pStackPicker->setEnabled(state == stateStackCut);

    m_pActLineCut->setChecked(state == stateLineCut);
    m_pActVolumeCut->setChecked(state == stateVolumeCut);
    m_pActValuePicker->setChecked(state == stateValuePicker);
    m_pActStackCut->setChecked(state == stateStackCut);

    if (state != stateLineCut && state != stateStackCut && state != stateVolumeCut)
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

            Itom2dQwtPlot *p = (Itom2dQwtPlot*)parent();

            ito::uint32 childFigureUID;
            p->displayZStackCut(pts, &childFigureUID);

			if (p->pickerWidget() && childFigureUID > 0)
			{
				p->pickerWidget()->updateChildPlot(childFigureUID, ito::Shape::Point, QVector4D(pt));
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

            Itom2dQwtPlot* p = (Itom2dQwtPlot*)parent();
            ito::uint32 childFigureUID;

            p->displayZStackCut(pts, &childFigureUID);

			if (p->pickerWidget() && childFigureUID > 0)
			{
				p->pickerWidget()->updateChildPlot(childFigureUID, ito::Shape::Point, QVector4D(pts[0]));
			}

            replot();
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::lineCutMovedPx(const QPoint &pt)
{
    QPointF phys(invTransform(QwtPlot::xBottom, pt.x()), invTransform(QwtPlot::yLeft, pt.y()));
    lineCutMovedPhys(phys);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::volumeCutMovedPx(const QPoint & pt)
{
    QPointF phys(invTransform(QwtPlot::xBottom, pt.x()), invTransform(QwtPlot::yLeft, pt.y()));
    volumeCutMovedPhys(phys);
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::lineCutMovedPhys(const QPointF &pt)
{
    if (state() == stateLineCut)
    {
        QVector<QPointF> pts;
        pts.resize(2);

        QwtInterval xInterval = m_rasterData->interval(Qt::XAxis);
        QwtInterval yInterval = m_rasterData->interval(Qt::YAxis);

        if (m_pLineCutLine->dataSize() > 0)
        {
            pts[0] = m_pLineCutLine->sample(0);
        }
        else
        {
            return;
        }

        if (m_lineCutValidStart)
        {
            pts[1] = pt;

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
                //Ctrl pressed: line should be horizontally or vertically aligned. This has to be checked with respect to screen coordinates,
                //if it is done on scale coordinates, problems can occur if one dimension has a scaling that varies strongly from the other scaling.
                int dx = transform(QwtPlot::xBottom, pts[0].x()) - transform(QwtPlot::xBottom, pts[1].x());
                int dy = transform(QwtPlot::yLeft, pts[0].y()) - transform(QwtPlot::yLeft, pts[1].y());

                if (abs(dx) > abs(dy))
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

            Itom2dQwtPlot* p = (Itom2dQwtPlot*)parent();

            if (m_dObjPtr && m_dObjPtr->getDims() > 2)
            {
                pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(), m_rasterData->getCurrentPlane()));
            }

            ito::uint32 childFigureUID;
            p->displayLineCut(pts, &childFigureUID);

            if (childFigureUID > 0 && p->pickerWidget())
            {
                QVector4D vec;
                if (pts.size() == 3)
                {
                    vec = QVector4D(pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
                }
                else
                {
                    vec = QVector4D(pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
                }

                p->pickerWidget()->updateChildPlot(childFigureUID, ito::Shape::Line, vec);
            }

            replot();
        }
        else //first point is still not valid, try to find a valid first point
        {
            QPointF pts0 = pts[0];

            pts[0] = pt;

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

            Itom2dQwtPlot* p = (Itom2dQwtPlot*)parent();

            // check for m_dObjPtr first otherwise crash
            if (m_dObjPtr && m_dObjPtr->getDims() > 2)
            {
                pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(), m_rasterData->getCurrentPlane()));
            }

            ito::uint32 childFigureUID;

            p->displayLineCut(pts, &childFigureUID);

            if (p->pickerWidget() && childFigureUID > 0)
            {
                QVector4D vec;
                if (pts.size() == 3)
                {
                    vec = QVector4D(pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
                }
                else
                {
                    vec = QVector4D(pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
                }

                p->pickerWidget()->updateChildPlot(childFigureUID, ito::Shape::Line, vec);
            }
            replot();
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::lineCutAppendedPx(const QPoint &pt)
{
    QPointF phys(invTransform(QwtPlot::xBottom, pt.x()), invTransform(QwtPlot::yLeft, pt.y()));
    lineCutAppendedPhys(phys);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::lineCutAppendedPhys(const QPointF &pt)
{
    if (state() == stateLineCut)
    {
        QVector<QPointF> pts;
        pts.resize(2);
        pts[0] = pt;

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

        Itom2dQwtPlot* p = (Itom2dQwtPlot*)parent();

        // check for m_dObjPtr first otherwise crash
        if (m_dObjPtr && m_dObjPtr->getDims() > 2)
        {
            pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(), m_rasterData->getCurrentPlane()));
        }

        ito::uint32 childFigureUID;

        p->displayLineCut(pts, &childFigureUID);

        if (p->pickerWidget() && childFigureUID > 0)
        {
            QVector4D vec;
            if (pts.size() == 3)
            {
                vec = QVector4D(pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
            }
            else
            {
                vec = QVector4D(pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
            }

            p->pickerWidget()->updateChildPlot(childFigureUID, ito::Shape::Line, vec);
        }

        replot();
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::volumeCutAppendedPx(const QPoint &pt)
{
    QPointF phys(invTransform(QwtPlot::xBottom, pt.x()), invTransform(QwtPlot::yLeft, pt.y()));
    volumeCutAppendedPhys(phys);
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::volumeCutMovedPhys(const QPointF &pt)
{
    if (state() == stateVolumeCut)
    {
        QVector<QPointF> pts;
        pts.resize(2);

        QwtInterval xInterval = m_rasterData->interval(Qt::XAxis);
        QwtInterval yInterval = m_rasterData->interval(Qt::YAxis);

        if (m_pVolumeCutLine->dataSize() > 0)
        {
            pts[0] = m_pVolumeCutLine->sample(0);
        }
        else
        {
            return;
        }

        if (m_volumeCutValidStart)
        {
            pts[1] = pt;

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
                //Ctrl pressed: line should be horizontally or vertically aligned. This has to be checked with respect to screen coordinates,
                //if it is done on scale coordinates, problems can occur if one dimension has a scaling that varies strongly from the other scaling.
                int dx = transform(QwtPlot::xBottom, pts[0].x()) - transform(QwtPlot::xBottom, pts[1].x());
                int dy = transform(QwtPlot::yLeft, pts[0].y()) - transform(QwtPlot::yLeft, pts[1].y());

                if (abs(dx) > abs(dy))
                {
                    pts[1].setY(pts[0].y());
                }
                else
                {
                    pts[1].setX(pts[0].x());
                }
            }

            m_pVolumeCutLine->setSamples(pts);

            setCoordinates(pts, true);

            Itom2dQwtPlot* p = (Itom2dQwtPlot*)parent();
            ito::uint32 childFigureUID;

            p->displayVolumeCut(pts, &childFigureUID);

            if (p->pickerWidget() && childFigureUID > 0)
            {
                QVector4D vec;
                if (pts.size() == 3)
                {
                    vec = QVector4D(pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
                }
                else
                {
                    vec = QVector4D(pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
                }

                p->pickerWidget()->updateChildPlot(childFigureUID, ito::Shape::Line, vec);
            }

            replot();
        }
        else //first point is still not valid, try to find a valid first point
        {
            QPointF pts0 = pts[0];

            pts[0] = pt;

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

                m_volumeCutValidStart = true;
            }
            else
            {
                //first point not valid
                m_volumeCutValidStart = false;
            }

            pts[1] = pts[0];

            m_pVolumeCutLine->setVisible(true);
            m_pVolumeCutLine->setSamples(pts);

            setCoordinates(pts, true);

            Itom2dQwtPlot* p = (Itom2dQwtPlot*)parent();

            // check for m_dObjPtr first otherwise crash
            if (m_dObjPtr && m_dObjPtr->getDims() > 2)
            {
                pts.insert(0, 1, QPointF(m_rasterData->getCurrentPlane(), m_rasterData->getCurrentPlane()));
            }

            ito::uint32 childFigureUID;

            p->displayVolumeCut(pts, &childFigureUID);

            if (p->pickerWidget() && childFigureUID > 0)
            {
                QVector4D vec;
                if (pts.size() == 3)
                {
                    vec = QVector4D(pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
                }
                else
                {
                    vec = QVector4D(pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
                }

                p->pickerWidget()->updateChildPlot(childFigureUID, ito::Shape::Line, vec);
            }
            replot();
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::volumeCutAppendedPhys(const QPointF &pt)
{
    if (state() == stateVolumeCut)
    {
        QVector<QPointF> pts;
        pts.resize(2);
        pts[0] = pt;

        if (m_rasterData->pointValid(pts[0]))
        {
            m_volumeCutValidStart = true;
        }
        else
        {
            //first point not valid
            m_volumeCutValidStart = false;
        }

        pts[1] = pts[0];

        m_pVolumeCutLine->setVisible(true);
        m_pVolumeCutLine->setSamples(pts);

        Itom2dQwtPlot* p = (Itom2dQwtPlot*)parent();
        ito::uint32 childFigureUID;

        p->displayVolumeCut(pts, &childFigureUID);

        if (p->pickerWidget() && childFigureUID > 0)
        {
            QVector4D vec;
            if (pts.size() == 3)
            {
                vec = QVector4D(pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
            }
            else
            {
                vec = QVector4D(pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
            }

            p->pickerWidget()->updateChildPlot(childFigureUID, ito::Shape::Line, vec);
        }

        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::removeChildPlotIndicators(bool lineChildPlot, bool zStackChildPlot, bool volumeChildPlot, bool resetState /*= false*/)
{
    if (zStackChildPlot)
    {
        m_pStackCutMarker->setVisible(false);

        if (resetState && state() == stateStackCut)
        {
            setState(stateIdle);
        }
    }

    if (lineChildPlot)
    {
        m_pLineCutLine->setVisible(false);
        setCoordinates(QVector<QPointF>(), false);

        if (resetState && state() == stateLineCut)
        {
            setState(stateIdle);
        }
    }

    if (volumeChildPlot)
    {
        m_pVolumeCutLine->setVisible(false);
        setCoordinates(QVector<QPointF>(), false);

        if (resetState && state() == stateVolumeCut)
        {
            setState(stateIdle);
        }
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> PlotCanvas::getDisplayed()
{
    if (!m_rasterData)
    {
        return QSharedPointer<ito::DataObject>();
    }

    return m_rasterData->rasterToObject(axisInterval(QwtPlot::xBottom), axisInterval(QwtPlot::yLeft), ItomQwtPlot::m_copyDisplayedAsComplex, PlotCanvas::getComplexStyle());
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> PlotCanvas::getDisplayedOverlayObject()
{
    if (!m_rasterOverlayData)
    {
        return QSharedPointer<ito::DataObject>();
    }

    return m_rasterOverlayData->rasterToObject(axisInterval(QwtPlot::xBottom), axisInterval(QwtPlot::yLeft), ItomQwtPlot::m_copyDisplayedAsComplex, PlotCanvas::getComplexStyle());
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> PlotCanvas::getOverlayObject()
{
    if (!m_rasterOverlayData)
    {
        return QSharedPointer<ito::DataObject>();
    }
    return m_rasterOverlayData->rasterToObject();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setOverlayObject(ito::DataObject* newOverlay)
{
    if (!m_pData)
        return;

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
template<typename _Tp> void parseContourLevels(const QSharedPointer<ito::DataObject> &obj, QList<double>* list)
{
    if (list)
    {
        const _Tp* rowPtr = NULL;
        cv::Mat_<_Tp> *mat = NULL;
        int m, n;
        for (int layer = 0; layer < obj->getNumPlanes(); ++layer)
        {
            mat = (cv::Mat_<_Tp>*)obj->get_mdata()[obj->seekMat(layer)];
            for (m = 0; m < mat->rows; ++m)
            {
                rowPtr = (_Tp*)mat->ptr(m);
                for (n = 0; n < mat->cols; ++n)
                {
                    list->append(rowPtr[n]);
                }
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setContourLevels(QSharedPointer<ito::DataObject> contourLevels)
{
    ito::RetVal retval(ito::retOk);

    int trueDims = 0;
    bool isInPlane = true;

    if (!contourLevels.isNull())
    {
        int dims = contourLevels->getDims();

        if (dims == 0)
        {
            m_dObjItem->setDisplayMode(QwtPlotSpectrogram::ContourMode, false);
            m_dObjItem->setContourLevels(QList<double>());
            m_pContourObj = QSharedPointer<ito::DataObject>();
        }
        else
        {
            if (contourLevels->getType() == ito::tRGBA32 || contourLevels->getType() == ito::tComplex128 || contourLevels->getType() == ito::tComplex64)
            {
                retval += ito::RetVal(ito::retError, 0, tr("Can not set dataObjects of type RGBA32, Complex128 or Complex64 for contour Lines").toLatin1().data());
                emit statusBarMessage(tr("Can not set dataObjects of type RGBA32, Complex128 or Complex64 for contour Lines"), 4000);
            }

            if (!retval.containsError())
            {
                for (int i = 0; i < contourLevels->getDims()-1; ++i)
                {
                    if (contourLevels->getSize(i) != 1)
                    {
                        ++trueDims;
                    }
                }

                if (trueDims != 0)
                {
                    retval += ito::RetVal(ito::retError, 0, tr("Can not set dataObject with a shape greater than one in the last but one dimensions for contour Lines").toLatin1().data());
                    emit statusBarMessage(tr("Can not set dataObject with a shape greater than one in the last but one dimensions for contour Lines"), 4000);
                }

                if (!retval.containsError())
                {
                    QList<double> list;
                    switch (contourLevels->getType())
                    {
                    case ito::tUInt8:
                        parseContourLevels<ito::uint8>(contourLevels, &list);
                        break;
                    case ito::tUInt16:
                        parseContourLevels<ito::uint16>(contourLevels, &list);
                        break;
                    case ito::tUInt32:
                        parseContourLevels<ito::uint32>(contourLevels, &list);
                        break;
                    case ito::tFloat32:
                        parseContourLevels<ito::float32>(contourLevels, &list);
                        break;
                    case ito::tFloat64:
                        parseContourLevels<ito::float64>(contourLevels, &list);
                        break;
                    case ito::tInt8:
                        parseContourLevels<ito::int8>(contourLevels, &list);
                        break;
                    case ito::tInt16:
                        parseContourLevels<ito::int16>(contourLevels, &list);
                        break;
                    case ito::tInt32:
                        parseContourLevels<ito::int32>(contourLevels, &list);
                        break;

                    }

                    if (m_dObjItem)
                    {
                        m_dObjItem->setConrecFlag(QwtRasterData::IgnoreAllVerticesOnLevel, true);
                        m_dObjItem->setContourLevels(list);
                        m_pContourObj = contourLevels;

                        if (m_curContourColorMapIndex == -1)
                        {
                            QPen pen(m_dObjItem->defaultContourPen());
                            pen.setColor(inverseColor1());
                            m_dObjItem->setDefaultContourPen(pen);
                        }

                        if ((m_dObjPtr->getType() == ito::tRGBA32))
                        {
                            m_dObjItem->setDisplayMode(QwtPlotSpectrogram::ContourMode, (m_pData->m_dataChannel & ItomQwtPlotEnums::ChannelGray) > 0);
                        }
                        else
                        {
                            m_dObjItem->setDisplayMode(QwtPlotSpectrogram::ContourMode, true);
                        }
                    }
                    else
                    {
                        m_dObjItem->setDisplayMode(QwtPlotSpectrogram::ContourMode, false);
                    }
                }
            }
        }
        replot();
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> PlotCanvas::getContourLevels() const
{
    if (!m_pContourObj)
    {
        return QSharedPointer<ito::DataObject>();
    }
    return QSharedPointer<ito::DataObject>(m_pContourObj);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotCanvas::setContourColorMap(const QString& name /*=__next__*/)
{
    ito::ItomPalette newPalette;
    ito::RetVal retval(ito::retOk);
    int numPalettes = 1;
    ito::int32 idx = -1;

    if (!ito::ITOM_API_FUNCS_GRAPH)
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
    if (name == "")
    {
        m_curContourColorMapIndex = -1;
        if (m_dObjItem)
        {
            QPen pen(m_dObjItem->defaultContourPen());
            pen.setColor(inverseColor1());
            pen.setStyle(Qt::SolidLine);
            m_dObjItem->setDefaultContourPen(pen);
            m_colorContourMapName = "";
            replot();
            return true;
        }

    }
    else if (name == "__next__")
    {
        m_curContourColorMapIndex++;
        m_curContourColorMapIndex %= numPalettes; //map index to [0,numPalettes)
        retval += apiPaletteGetColorBarIdx(m_curContourColorMapIndex, newPalette);
    }
    else if (name == "__first__")
    {
        m_curContourColorMapIndex = 0;
        retval += apiPaletteGetColorBarIdx(m_curContourColorMapIndex, newPalette);
    }
    else
    {
        retval += apiPaletteGetColorBarName(name, newPalette);

        retval += apiPaletteGetColorBarIdxFromName(name, idx);

    }

    if (retval.containsError() && retval.errorMessage() != NULL)
    {
        emit statusBarMessage(QString("%1").arg(QLatin1String(retval.errorMessage())), 4000);
        return false;
    }
    else if (retval.containsError())
    {
        emit statusBarMessage(tr("error when loading color map"), 4000);
        return false;
    }

    m_curContourColorMapIndex = idx;
    int totalStops = newPalette.colorStops.size();

    if (totalStops < 2)
    {
        emit statusBarMessage(tr("Selected color map has less than two points."), 4000);
        return false;
    }

    m_colorContourMapName = newPalette.name;
    //m_curContourColorMapIndex = newPalette.index;

        if (m_dObjItem)
        {
            m_dObjItem->setContourPalette(newPalette);
        }
    replot();
    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setContourLineWidth(const float& width)
{
    QPen pen(m_dObjItem->defaultContourPen());
    pen.setWidthF(width);
    m_dObjItem->setDefaultContourPen(pen);
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
float PlotCanvas::getContourLineWidth() const
{
    return m_dObjItem->defaultContourPen().widthF();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::alphaChanged()
{
    if (!m_pData)
        return;

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
        return ito::RetVal(ito::retError, 0, tr("Could not activate the line cut.").toLatin1().data());
    }

    lineCutAppendedPhys(QPointF(x0, y0));
    lineCutMovedPhys(QPointF(x1,y1));

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtPlotEnums::ComplexType PlotCanvas::getComplexStyle() const
{
    if (!m_pData)
        //ItomQwtPlotEnums::ComplexType
        return (ItomQwtPlotEnums::ComplexType)0;

    return m_pData->m_cmplxType;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::setComplexStyle(const ItomQwtPlotEnums::ComplexType &type)
{
	foreach(QAction* a, m_pMnuCmplxSwitch->actions())
	{
		if (a->data().toInt() == type)
		{
			mnuCmplxSwitch(a);
			break;
		}
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuScaleSettings()
{
    if (!m_pData)
        return;

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
    if (!m_pData)
        return;

    m_pMnuCmplxSwitch->setDefaultAction(action);
    setButtonStyle(buttonStyle()); //to change icon of menu

    int idx = action->data().toInt();
    m_pData->m_cmplxType = (ItomQwtPlotEnums::ComplexType)idx;
    internalDataUpdated();
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuDataChannel(QAction* action)
{
    if (!m_pData)
        return;

    m_pMnuDataChannel->setDefaultAction(action);
    m_pActDataChannel->setIcon(m_pMnuDataChannel->defaultAction()->icon());

    if (action->data().toInt() != m_pData->m_dataChannel)
    {
        m_pData->m_dataChannel = (ItomQwtPlotEnums::DataChannel)action->data().toInt();
        adjustColorDataTypeRepresentation();
        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuColorPalette()
{
    setColorMap("__next__");
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuGroupColorPalette(QAction *action)
{
	if (action)
	{
		QString colorPalette = action->data().toString();
		if (colorPalette != "")
		{
			if (!setColorMap(colorPalette))
			{
				action->setChecked(false);
			}
		}
	}
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
void PlotCanvas::mnuVolumeCut(bool checked)
{
    if (checked)
    {
        setState(stateVolumeCut);
    }
    else if (state() == stateVolumeCut)
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
    case 0: //interactive line
        if (ito::isFinite(min) && ito::isFinite(max))
        {

            double dy = maxLoc[1] - minLoc[1];
            double dx = maxLoc[2] - minLoc[2];

            if (!ito::isNotZero(dy) && !ito::isNotZero(dx))
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
    case 1: //horizontal line through global minimum
        if (ito::isFinite(min) && ito::isFinite(max))
        {
            QwtInterval xIntervalDataObj = m_rasterData->interval(Qt::XAxis);
            x.setMinimum(std::max((double)x.minimum(), xIntervalDataObj.minValue()));
            x.setMaximum(std::min((double)x.maximum(), xIntervalDataObj.maxValue()));
            y.setMinimum(minLoc[1]);
            y.setMaximum(minLoc[1]);
        }
        break;
    case 2: //horizontal line through global maximum
        if (ito::isFinite(min) && ito::isFinite(max))
        {
            QwtInterval xIntervalDataObj = m_rasterData->interval(Qt::XAxis);
            x.setMinimum(std::max((double)x.minimum(), xIntervalDataObj.minValue()));
            x.setMaximum(std::min((double)x.maximum(), xIntervalDataObj.maxValue()));
            y.setMinimum(maxLoc[1]);
            y.setMaximum(maxLoc[1]);
        }
        break;

    case 3: //vertical line through global minimum
        if (ito::isFinite(min) && ito::isFinite(max))
        {
            QwtInterval yIntervalDataObj = m_rasterData->interval(Qt::YAxis);
            y.setMinimum(std::max((double)y.minimum(), yIntervalDataObj.minValue()));
            y.setMaximum(std::min((double)y.maximum(), yIntervalDataObj.maxValue()));
            x.setMinimum(minLoc[2]);
            x.setMaximum(minLoc[2]);
        }
        break;

    case 4: //vertical line through global maximum
        if (ito::isFinite(min) && ito::isFinite(max))
        {
            QwtInterval yIntervalDataObj = m_rasterData->interval(Qt::YAxis);
            y.setMinimum(std::max((double)y.minimum(), yIntervalDataObj.minValue()));
            y.setMaximum(std::min((double)y.maximum(), yIntervalDataObj.maxValue()));
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
    if (!m_pData)
        return;

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

//----------------------------------------------------------------------------------------------------------------------------------
void PlotCanvas::mnuSetGrid(QAction *action)
{
    Itom2dQwtPlot::GridStyle style = Itom2dQwtPlot::GridNo;

    if (action)
    {
        style = (Itom2dQwtPlot::GridStyle)action->data().toInt();
        if (style != action->data().toInt())
        {
            style = Itom2dQwtPlot::GridNo;
        }
    }

    bool ok;
    foreach(QAction *a, m_pMnuGrid->actions())
    {
        if (a->data().toInt(&ok) == style && ok)
        {
            //m_pMnuGrid->setIcon(a->icon());
            m_pMnuGrid->setDefaultAction(a);
        }
    }

    setGridStyle(style);
}

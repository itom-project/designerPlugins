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

#include "itom1DQwtPlot.h"
#include "plot1DWidget.h"
#include "dataObjectSeriesData.h"
#include "dataObjectSeriesDataXY.h"
#include "DataObject/dataObjectFuncs.h"
#include "qwtPlotCurveDataObject.h"
#include "qwtPlotCurveProperty.h"
#include "common/sharedStructuresGraphics.h"
#include "common/apiFunctionsGraphInc.h"
#include "qnumeric.h"
#include "dialog1DScale.h"
#include "widgetCurveProperties.h"

#include "itomLogLogScaleEngine.h"

#include "itomPlotZoomer.h"
#include <qwt_color_map.h>
#include <qwt_plot_layout.h>
#include <qwt_plot_renderer.h>
#include <qwt_plot_grid.h>
#include <qwt_plot_canvas.h>
#include <qwt_legend.h>
#include <qwt_legend_label.h>
#include <qwt_symbol.h>
#include <qwt_picker.h>
#include <qwt_picker_machine.h>
#include <qwt_scale_widget.h>
#include <qwt_scale_engine.h>
#include <qwt_text_label.h>
#include <qwt_date_scale_draw.h>
#include <qwt_date_scale_engine.h>

#include <qimage.h>
#include <qpixmap.h>
#include <qdebug.h>
#include <qmessagebox.h>
#include <qnumeric.h>
#include <qinputdialog.h>

/*static*/ QStringList Plot1DWidget::siLengthUnits = QStringList() << "pm" << "nm" << (QLatin1String("\u00B5m")) << "mm" << "m" << "km";
/*static*/ QStringList Plot1DWidget::siTimeUnits = QStringList() << "ps" << "ns" << (QLatin1String("\u00B5s")) << "ms" << "s";

//namespace ito {
//    extern void **ITOM_API_FUNCS_GRAPH;
//}
//----------------------------------------------------------------------------------------------------------------------------------
Plot1DWidget::Plot1DWidget(InternalData *data, ItomQwtDObjFigure *parent) :
    ItomQwtPlot(parent),
    m_pPlotGrid(NULL),
    m_xDirect(false),
    m_yDirect(false),
    m_lineCol(0),
    m_lineWidth(1.0),
    m_lineStyle(Qt::SolidLine),
    m_hasParentForRescale(false),
    m_layerState(false),
    m_pData(data),
    m_gridStyle(Itom1DQwtPlot::GridNo),
    m_legendPosition(BottomLegend),
    m_legendVisible(false),
    m_pLegendLabelWidth(15),
    m_qwtCurveStyle(ItomQwtPlotEnums::Lines),
    m_baseLine(0.0),
    m_fillCurveAlpa(128),
    m_filledColor(QColor::Invalid),
    m_curveFilled(ItomQwtPlotEnums::NoCurveFill),
    m_pLegend(NULL),
    m_guiModeCache(-1),
    m_valueScale(ItomQwtPlotEnums::Linear),
    m_axisScale(ItomQwtPlotEnums::Linear),
    m_pActScaleSettings(NULL),
    m_pRescaleParent(NULL),
    m_pActForward(NULL),
    m_pActBack(NULL),
    m_pActPicker(NULL),
    m_pActSetPickerMinMaxGlobal(NULL),
    m_pActSetPickerMinMaxLocal(NULL),
    m_pActSetPicker(NULL),
    m_pActDeletePickers(NULL),
    m_pMnuSetPicker(NULL),
    m_pMnuCmplxSwitch(NULL),
    m_pMnuRGBSwitch(NULL),
    m_pMnuGrid(NULL),
    m_pLblMarkerOffsets(NULL),
    m_pLblMarkerCoords(NULL),
    m_pMnuMultiRowSwitch(NULL),
    m_pActXVAuto(NULL),
    m_pActXVFR(NULL),
    m_pActXVFC(NULL),
    m_pActXVMR(NULL),
    m_pActXVMC(NULL),
    m_pActXVML(NULL),
    m_pActRGBA(NULL),
    m_pActGray(NULL),
    m_pActRGBL(NULL),
    m_pActRGBAL(NULL),
    m_pActRGBG(NULL),
    m_pActMultiRowSwitch(NULL),
    m_pActRGBSwitch(NULL),
    m_pActCmplxSwitch(NULL),
    m_pActLegendSwitch(NULL),
    m_pMnuLegendSwitch(NULL),
    m_antiAliased(false),
    m_pComplexStyle(ItomQwtPlotEnums::CmplxAbs),
    m_legendOffset(0)
{
    createActions();
    setButtonStyle(buttonStyle());

    canvas()->setCursor(Qt::ArrowCursor);

    m_colorList.reserve(12);
    /*m_colorList.append("blue");
    m_colorList.append("green");
    m_colorList.append("red");
    m_colorList.append("magenta");
    m_colorList.append("cyan");
    m_colorList.append("yellow");
    m_colorList.append("darkBlue");
    m_colorList.append("darkGreen");
    m_colorList.append("darkRed");
    m_colorList.append("darkMagenta");
    m_colorList.append("darkCyan");
    m_colorList.append("darkYellow");*/

    //12-class Paired list from colorbrewer2.org (Cynthia Brewer, Geography, Pennsylvania State University). (reordered)
    m_colorList.append("#1f78b4"); //also used for blue in RGBA values
    m_colorList.append("#33a02c"); //also used for green in RGBA values
    m_colorList.append("#e31a1c"); //also used for red in RGBA values
    m_colorList.append("#ff7f00");
    m_colorList.append("#6a3d9a");
    m_colorList.append("#b15928");
    m_colorList.append("#97c9e3");
    m_colorList.append("#b2df8a");
    m_colorList.append("#fb9a99");
    m_colorList.append("#fdbf6f");
    m_colorList.append("#cab2d6");
    m_colorList.append("#fdff7a"); //("#ffff99");
    
    //value picker
    m_pValuePicker = new ValuePicker1D(QwtPlot::xBottom, QwtPlot::yLeft, canvas());
    m_pValuePicker->setEnabled(false);
    m_pValuePicker->setTrackerMode(QwtPicker::AlwaysOn);

    //all others settings for tracker are set in init (since they need access to the settings via api)

    m_pPlotGrid = new QwtPlotGrid();
    m_pPlotGrid->attach(this);
    setGridStyle(m_gridStyle);
    m_pPlotGrid->setMajorPen(Qt::gray, 1);
    m_pPlotGrid->setMinorPen(Qt::gray, 1, Qt::DashLine);

    QWidget *guiParent = parent;
    if (!guiParent) guiParent = this;

    QToolBar *mainTb = new QToolBar(tr("Plotting Tools"), guiParent);
    mainTb->setObjectName("mainToolBar");
    m_toolbars.append(mainTb);

    // first block is zoom, scale settings, home
    mainTb->addAction(m_pActSave);
    mainTb->addAction(m_pActPrint);
    mainTb->addSeparator();
    mainTb->addAction(m_pActHome);
    mainTb->addAction(m_pActProperties);
    mainTb->addAction(m_pActCamParameters);
    mainTb->addAction(m_pActScaleSettings);
    mainTb->addAction(m_pRescaleParent);
    mainTb->addAction(m_pActPan);
    mainTb->addAction(m_pActZoom);
    mainTb->addAction(m_pActAspectRatio);
    mainTb->addAction(m_pActMultiRowSwitch);
    mainTb->addAction(m_pActRGBSwitch);
    // first block is zoom, scale settings, home
    mainTb->addSeparator();
    mainTb->addAction(m_pActPicker);
    mainTb->addSeparator();
    mainTb->addAction(m_pActShapeType);
    mainTb->addAction(m_pActClearShapes);

    // Add labels to toolbar
    QAction *lblAction = mainTb->addWidget(m_pLblMarkerCoords);
    lblAction->setVisible(true);

    QAction *lblAction2 = mainTb->addWidget(m_pLblMarkerOffsets);
    lblAction2->setVisible(true);

    // next block is for complex and stacks
    mainTb->addSeparator();
    mainTb->addAction(m_pActBack);
    mainTb->addAction(m_pActForward);
    mainTb->addAction(m_pActCmplxSwitch);

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
    menuView->addAction(m_pActGrid);
    menuView->addAction(m_pActLegendSwitch);
    menuView->addSeparator();
    menuView->addAction(m_pActScaleSettings);
    menuView->addAction(m_pRescaleParent);
    menuView->addSeparator();
    menuView->addMenu(m_pMnuCmplxSwitch);
    menuView->addSeparator();
    menuView->addMenu(m_pMenuToolboxes);
    menuView->addSeparator();
    QAction *actCurveProperties = ((Itom1DQwtPlot*)(this->parent()))->getCurvePropertiesToggleViewAction();
	menuView->addAction(actCurveProperties);
    menuView->addAction(m_pActCamParameters);
    menuView->addAction(m_pActProperties);
	m_menus.append(menuView);

    QMenu *menuTools = new QMenu(tr("Tools"), guiParent);
    menuTools->addAction(m_pActPicker);
    menuTools->addSeparator();
    menuTools->addMenu(m_pMenuShapeType);
    menuTools->addAction(m_pActClearShapes);
    m_menus.append(menuTools);

    m_pContextMenu = new QMenu(QObject::tr("Plot1D"), guiParent);
    m_pContextMenu->addAction(m_pActSave);
    m_pContextMenu->addAction(m_pActPrint);
    m_pContextMenu->addAction(m_pActCopyClipboard);
    m_pContextMenu->addAction(m_pActSendCurrentToWorkspace);
    m_pContextMenu->addSeparator();
    m_pContextMenu->addAction(m_pActHome);
    m_pContextMenu->addAction(m_pActScaleSettings);
    m_pContextMenu->addSeparator();
    m_pContextMenu->addAction(m_pActPan);
    m_pContextMenu->addAction(m_pActZoom);
    m_pContextMenu->addAction(m_pActPicker);
    m_pContextMenu->addSeparator();
    m_pContextMenu->addAction(m_pActGrid);
    m_pContextMenu->addAction(m_pActLegendSwitch);
    m_pContextMenu->addSeparator();
    m_pContextMenu->addAction(m_pActProperties);
    m_pContextMenu->addAction(actCurveProperties);
    m_pContextMenu->addAction(m_pActCamParameters);
    m_pContextMenu->addAction(mainTb->toggleViewAction());
}

//----------------------------------------------------------------------------------------------------------------------------------
Plot1DWidget::~Plot1DWidget()
{
    foreach(QwtPlotCurve* c, m_plotCurveItems)
    {
        c->detach();
        delete c;
    }
    m_plotCurveItems.clear();

    foreach(QwtPlotCurveProperty* c, m_plotCurvePropertyItems)
    {
        if (c)
        {
            delete c;
        }
    }
    m_plotCurvePropertyItems.clear();

    foreach (Picker m, m_pickers)
    {
        m.item->detach();
        delete m.item;
    }
    m_pickers.clear();

    if (m_pPlotGrid)
    {
        m_pPlotGrid->detach();
        delete m_pPlotGrid;
        m_pPlotGrid = NULL;
    }

    

    if (m_pLegend)
    {
        m_pLegend->deleteLater();
        m_pLegend = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::init(bool overwriteDesignableProperties)
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

    ito::RetVal retVal;
    ItomQwtPlot::loadStyles(overwriteDesignableProperties);

    QPen trackerPen = QPen(QBrush(Qt::red), 2);
    QFont trackerFont = QFont("Verdana", 10);
    QBrush trackerBg = QBrush(Qt::white, Qt::SolidPattern);

    QFont titleFont = QFont("Verdana", 12); //designable
    QFont labelFont =  QFont("Verdana", 10); //designable
    QFont axisFont = QFont("Verdana", 10); //designable

    if (overwriteDesignableProperties)
    {
        m_legendFont  = QFont("Verdana", 8); //designable
        setUnitLabelStyle(ito::AbstractFigure::UnitLabelSlash); //designable
    }

    int buttonSet = buttonStyle(); //designable

    if (ito::ITOM_API_FUNCS_GRAPH)
    {
		ito::ItomPalette defaultColorPalette;
		apiPaletteGetColorBarIdx(0, defaultColorPalette);
		setInverseColors(defaultColorPalette.inverseColorOne, defaultColorPalette.inverseColorTwo);

        trackerPen = apiGetFigureSetting(parent(), "trackerPen", trackerPen, &retVal).value<QPen>();
        trackerFont = apiGetFigureSetting(parent(), "trackerFont", trackerFont, &retVal).value<QFont>();
        trackerBg = apiGetFigureSetting(parent(), "trackerBackground", trackerBg, &retVal).value<QBrush>();

        m_pPlotGrid->setMajorPen(apiGetFigureSetting(parent(), "gridMajorPen", QPen(QBrush(Qt::gray), 1, Qt::SolidLine), &retVal).value<QPen>());
        m_pPlotGrid->setMinorPen(apiGetFigureSetting(parent(), "gridMinorPen", QPen(QBrush(Qt::gray), 1, Qt::DashLine), &retVal).value<QPen>());

        if (overwriteDesignableProperties)
        {
            titleFont = apiGetFigureSetting(parent(), "titleFont", titleFont, &retVal).value<QFont>(); //designable
            labelFont = apiGetFigureSetting(parent(), "labelFont", labelFont, &retVal).value<QFont>(); //designable
            axisFont = apiGetFigureSetting(parent(), "axisFont", axisFont, &retVal).value<QFont>(); //designable

            buttonSet = apiGetFigureSetting(parent(), "buttonSet", buttonSet, NULL).value<int>(); //usually this property is only asked to inherit the buttonSet from the parent plot. //designable

            m_lineStyle = (Qt::PenStyle)(apiGetFigureSetting(parent(), "lineStyle", (int)m_lineStyle, &retVal).value<int>()); //designable
            m_lineWidth = apiGetFigureSetting(parent(), "lineWidth", m_lineWidth, &retVal).value<qreal>(); //designable

            m_curveFilled = (ItomQwtPlotEnums::FillCurveStyle)apiGetFigureSetting(parent(), "fillCurve", (int)m_curveFilled, &retVal).value<int>(); //designable
            m_filledColor = apiGetFigureSetting(parent(), "curveFillColor", m_filledColor, &retVal).value<QColor>(); //designable
            m_fillCurveAlpa = cv::saturate_cast<ito::uint8>(apiGetFigureSetting(parent(), "curveFillAlpha", m_fillCurveAlpa, &retVal).value<int>()); //designable

			setUnitLabelStyle((ito::AbstractFigure::UnitLabelStyle)(apiGetFigureSetting(parent(), "unitLabelStyle", unitLabelStyle(), &retVal).value<int>())); //designable

            m_antiAliased = apiGetFigureSetting(parent(), "antiAliased", m_antiAliased, &retVal).value<bool>(); //designable

            m_legendFont = apiGetFigureSetting(parent(), "legendFont", m_legendFont, &retVal).value<QFont>(); //designable
		    m_pLegendLabelWidth = apiGetFigureSetting(parent(), "legendLabelWidth", m_pLegendLabelWidth, &retVal).value<int>(); //designable
        }
    }

    m_pValuePicker->setTrackerFont(trackerFont);
    m_pValuePicker->setTrackerPen(trackerPen);
    m_pValuePicker->setBackgroundFillBrush(trackerBg);

    if (overwriteDesignableProperties)
    {
        title().setFont(titleFont);
        titleLabel()->setFont(titleFont);

        axisTitle(QwtPlot::xBottom).setFont(axisFont);
        axisTitle(QwtPlot::yLeft).setFont(axisFont);
        setAxisFont(QwtPlot::xBottom, axisFont);
        setAxisFont(QwtPlot::yLeft, axisFont);

        QwtText t = axisWidget(QwtPlot::xBottom)->title();
        t.setFont(labelFont);
        axisWidget(QwtPlot::xBottom)->setTitle(t);

        t = axisWidget(QwtPlot::yLeft)->title();
        t.setFont(labelFont);
        axisWidget(QwtPlot::yLeft)->setTitle(t);

        if (buttonSet != buttonStyle())
        {
            setButtonStyle(buttonSet);
        }
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::createActions()
{
    QAction *a = NULL;
    ItomQwtDObjFigure *p = qobject_cast<ItomQwtDObjFigure*>(parent());

    //m_actScaleSetting
    m_pActScaleSettings = a = new QAction(tr("Scale Settings..."), p);
    a->setObjectName("actScaleSetting");
    a->setToolTip(tr("Set the ranges and offsets of this view"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuScaleSettings()));

    //m_rescaleParent
    m_pRescaleParent = a = new QAction(tr("Parent Scale Settings"), p);
    a->setObjectName("rescaleParent");
    a->setToolTip(tr("Set the value-range of the parent view according to this plot"));
    a->setVisible(false);
    connect(a, SIGNAL(triggered()), this, SLOT(mnuParentScaleSetting()));

    //m_actForward
    m_pActForward = a = new QAction(tr("Forward"), p);
    a->setObjectName("actionForward");
    a->setEnabled(false);
    a->setToolTip(tr("Forward to next line"));
    m_pActForward->setVisible(false);

    //m_actBack
    m_pActBack = a = new QAction(tr("Back"), p);
    a->setObjectName("actionBack");
    a->setEnabled(false);
    a->setToolTip(tr("Back to previous line"));
    m_pActBack->setVisible(false);

    //m_actMarker
    m_pActPicker = a = new QAction(tr("Picker"), p);
    a->setObjectName("actionPickerMenu");
    a->setCheckable(true);
    a->setChecked(false);
    connect(a, SIGNAL(toggled(bool)), this, SLOT(mnuPickerClick(bool)));

    m_pMnuSetPicker = new QMenu("Picker Menu", p);
    m_pActPicker->setMenu(m_pMnuSetPicker);

    m_pActSetPicker = a = m_pMnuSetPicker->addAction(tr("Manual Picker Selection"));
    a->setObjectName("actionPicker");
    a->setCheckable(true);
    a->setChecked(false);
    connect(a, SIGNAL(toggled(bool)), this, SLOT(mnuPickerClick(bool)));

    m_pActSetPickerMinMaxGlobal = a = m_pMnuSetPicker->addAction(tr("To Global Min-Max"));
    a->setToolTip(tr("set two pickers to the absolute minimum and maximum of the curve. \nIf multiple curves are visible, the user can select the appropriate one."));
    connect(a, &QAction::triggered, this, &Plot1DWidget::mnuSetPickerGlobalMinMax);

    m_pActSetPickerMinMaxLocal = a = m_pMnuSetPicker->addAction(tr("To Min-Max In Current View"));
    a->setToolTip(tr("set two pickers to the absolute minimum and maximum of the curve (within the current view). \nIf multiple curves are visible, the user can select the appropriate one."));
    connect(a, &QAction::triggered, this, &Plot1DWidget::mnuSetPickerRoiMinMax);

    m_pActDeletePickers = a = m_pMnuSetPicker->addAction(tr("Delete All Pickers"));
    a->setToolTip(tr("delete all pickers"));
    connect(a, &QAction::triggered, this, &Plot1DWidget::mnuDeletePicker);

    m_pMnuSetPicker->addSeparator();

    m_pMnuSetPicker->addAction(p->pickerDockWidget()->toggleViewAction());

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
    
    //m_actCmplxSwitch
    m_pActLegendSwitch = new QAction(tr("Legend"), p);
    m_pActLegendSwitch->setCheckable(true);
    m_pMnuLegendSwitch = new QMenu(tr("Legend"), p);
    m_pActLegendSwitch->setMenu(m_pMnuLegendSwitch);
    a = m_pMnuLegendSwitch->addAction(tr("Off"));
    a->setData(-1);
    m_pMnuLegendSwitch->setDefaultAction(a);
    a = m_pMnuLegendSwitch->addAction(tr("Right"));
    a->setData(RightLegend);

    a = m_pMnuLegendSwitch->addAction(tr("Bottom"));
    a->setData(BottomLegend);

    a = m_pMnuLegendSwitch->addAction(tr("Left"));
    a->setData(LeftLegend);

    a = m_pMnuLegendSwitch->addAction(tr("Top"));
    a->setData(TopLegend);

    connect(m_pMnuLegendSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuLegendSwitch(QAction*)));

    //m_pActMultiRowSwitch
    m_pActMultiRowSwitch = new QAction(tr("Data Representation"), p);
    m_pMnuMultiRowSwitch = new QMenu(tr("Data Representation"), p);
    m_pActMultiRowSwitch->setMenu(m_pMnuMultiRowSwitch);
    m_pActXVAuto = a = m_pMnuMultiRowSwitch->addAction(tr("Auto"));
    a->setData(ItomQwtPlotEnums::AutoRowCol);
    m_pMnuMultiRowSwitch->setDefaultAction(a);

    m_pActXVFR = a = m_pMnuMultiRowSwitch->addAction(tr("First Row"));
    a->setData(ItomQwtPlotEnums::FirstRow);

    m_pActXVFC = a = m_pMnuMultiRowSwitch->addAction(tr("First Column"));
    a->setData(ItomQwtPlotEnums::FirstCol);

    m_pActXVMR = a = m_pMnuMultiRowSwitch->addAction(tr("Multi Row"));
    a->setData(ItomQwtPlotEnums::MultiRows);

    m_pActXVMC = a = m_pMnuMultiRowSwitch->addAction(tr("Multi Column"));
    a->setData(ItomQwtPlotEnums::MultiCols);

    m_pActXVML = a = m_pMnuMultiRowSwitch->addAction(tr("Multi Layer"));
    a->setData(ItomQwtPlotEnums::MultiLayerAuto);

    m_pActMultiRowSwitch->setVisible(true);
    connect(m_pMnuMultiRowSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuMultiRowSwitch(QAction*)));

    //m_pActRGBSwitch
    m_pActRGBSwitch = new QAction(tr("Color Representation"), p);
    m_pMnuRGBSwitch = new QMenu(tr("Color Representation"), p);
    m_pActRGBSwitch->setMenu(m_pMnuRGBSwitch);
    m_pActRGBA = a = m_pMnuRGBSwitch->addAction(tr("Auto Value"));
    a->setData(ItomQwtPlotEnums::AutoColor);
    m_pMnuRGBSwitch->setDefaultAction(a);

    m_pActGray = a = m_pMnuRGBSwitch->addAction(tr("Gray Value"));
    a->setData(ItomQwtPlotEnums::Gray);

    m_pActRGBL = a = m_pMnuRGBSwitch->addAction(tr("RGB-lines"));
    a->setData(ItomQwtPlotEnums::RGB);

    m_pActRGBAL = a = m_pMnuRGBSwitch->addAction(tr("RGBA-lines"));
    a->setData(ItomQwtPlotEnums::RGBA);

    m_pActRGBG = a = m_pMnuRGBSwitch->addAction(tr("RGB + Gray"));
    a->setData(ItomQwtPlotEnums::RGBGray);

    m_pActRGBSwitch->setVisible(false);
    connect(m_pMnuRGBSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuRGBSwitch(QAction*)));

    //Labels for current cursor position
    m_pLblMarkerCoords = new QLabel("    \n    ", p);
    m_pLblMarkerCoords->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    m_pLblMarkerCoords->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Ignored);
    m_pLblMarkerCoords->setObjectName(tr("Marker Positions"));

    m_pLblMarkerOffsets = new QLabel("    \n    ", p);
    m_pLblMarkerOffsets->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    m_pLblMarkerOffsets->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Ignored);
    m_pLblMarkerOffsets->setObjectName(tr("Marker Offsets"));

    m_pActGrid = a = new QAction(tr("Grid"), p);
    a->setObjectName("actGrid");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Shows/hides a grid"));
    //connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuGridEnabled(bool)));

	m_pMnuGrid = new QMenu(tr("Grid"), p);
	m_pActGrid->setMenu(m_pMnuGrid);
	
	a = m_pMnuGrid->addAction(tr("No Grid"));
	a->setData(Itom1DQwtPlot::GridNo);
	m_pMnuGrid->setDefaultAction(a);

	a = m_pMnuGrid->addAction(tr("Major XY"));
	a->setData(Itom1DQwtPlot::GridMajorXY);

	a = m_pMnuGrid->addAction(tr("Major X"));
	a->setData(Itom1DQwtPlot::GridMajorX);

	a = m_pMnuGrid->addAction(tr("Major Y"));
	a->setData(Itom1DQwtPlot::GridMajorY);

	a = m_pMnuGrid->addAction(tr("Minor XY"));
	a->setData(Itom1DQwtPlot::GridMinorXY);

	a = m_pMnuGrid->addAction(tr("Minor X"));
	a->setData(Itom1DQwtPlot::GridMinorX);

	a = m_pMnuGrid->addAction(tr("Minor Y"));
	a->setData(Itom1DQwtPlot::GridMinorY);

	connect(m_pMnuGrid, SIGNAL(triggered(QAction*)), this, SLOT(mnuSetGrid(QAction*)));
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setButtonStyle(int style)
{
    ItomQwtPlot::setButtonStyle(style);

    if (style == 0)
    {
        m_pActScaleSettings->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/autoscal.png"));
        m_pRescaleParent->setIcon(QIcon(":/itom1DQwtFigurePlugin/icons/parentScale.png"));
        m_pActForward->setIcon(QIcon(":/itomDesignerPlugins/general/icons/forward.png"));
        m_pActBack->setIcon(QIcon(":/itomDesignerPlugins/general/icons/back.png"));
        m_pActPicker->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/picker.png"));
        m_pActSetPicker->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/picker.png"));
        m_pActSetPickerMinMaxGlobal->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/picker_min_max.png"));
        m_pActSetPickerMinMaxLocal->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/picker_min_max_cropped.png"));
        m_pActDeletePickers->setIcon(QIcon(":/itomDesignerPlugins/general/icons/editDelete.png"));
        m_pActXVAuto->setIcon(QIcon(":/itomDesignerPlugins/axis/icons/xvauto_plot.png"));
        m_pActXVFR->setIcon(QIcon(":/itomDesignerPlugins/axis/icons/xv_plot.png"));
        m_pActXVFC->setIcon(QIcon(":/itomDesignerPlugins/axis/icons/yv_plot.png"));
        m_pActXVMR->setIcon(QIcon(":/itomDesignerPlugins/axis/icons/xvm_plot.png"));
        m_pActXVMC->setIcon(QIcon(":/itomDesignerPlugins/axis/icons/yvm_plot.png"));
        m_pActXVML->setIcon(QIcon(":/itomDesignerPlugins/axis/icons/yxvzm_plot.png"));
        m_pActRGBA->setIcon(QIcon(":/itomDesignerPlugins/axis/icons/RGBA_RGB.png"));
        m_pActGray->setIcon(QIcon(":/itomDesignerPlugins/axis/icons/RGB_Gray.png"));
        m_pActRGBL->setIcon(QIcon(":/itomDesignerPlugins/axis/icons/RGBA_RGB.png"));
        m_pActRGBAL->setIcon(QIcon(":/itomDesignerPlugins/axis/icons/RGBA_RGBA.png"));
        m_pActRGBG->setIcon(QIcon(":/itomDesignerPlugins/axis/icons/RGB_RGBGray.png"));
        m_pActRGBSwitch->setIcon(m_pMnuRGBSwitch->defaultAction()->icon());
        m_pActMultiRowSwitch->setIcon(m_pMnuMultiRowSwitch->defaultAction()->icon());
        m_pActGrid->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/grid.png"));
        

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
        m_pRescaleParent->setIcon(QIcon(":/itom1DQwtFigurePlugin/icons/parentScale_lt.png"));
        m_pActForward->setIcon(QIcon(":/itomDesignerPlugins/general_lt/icons/forward_lt.png"));
        m_pActBack->setIcon(QIcon(":/itomDesignerPlugins/general_lt/icons/back_lt.png"));
        m_pActPicker->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/picker_lt.png"));
        m_pActSetPicker->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/picker_lt.png"));
        m_pActSetPickerMinMaxGlobal->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/picker_min_max_lt.png"));
        m_pActSetPickerMinMaxLocal->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/picker_min_max_cropped_lt.png"));
        m_pActDeletePickers->setIcon(QIcon(":/itomDesignerPlugins/general/icons/editDelete_lt.png"));
        m_pActXVAuto->setIcon(QIcon(":/itomDesignerPlugins/axis_lt/icons/xvauto_plot_lt.png"));
        m_pActXVFR->setIcon(QIcon(":/itomDesignerPlugins/axis_lt/icons/xv_plot_lt.png"));
        m_pActXVFC->setIcon(QIcon(":/itomDesignerPlugins/axis_lt/icons/yv_plot_lt.png"));
        m_pActXVMR->setIcon(QIcon(":/itomDesignerPlugins/axis_lt/icons/xvm_plot_lt.png"));
        m_pActXVMC->setIcon(QIcon(":/itomDesignerPlugins/axis_lt/icons/yvm_plot_lt.png"));
        m_pActXVML->setIcon(QIcon(":/itomDesignerPlugins/axis_lt/icons/yxvzm_plot_lt.png"));
        m_pActRGBA->setIcon(QIcon(":/itomDesignerPlugins/axis_lt/icons/RGBA_RGB_lt.png"));
        m_pActGray->setIcon(QIcon(":/itomDesignerPlugins/axis_lt/icons/RGB_Gray_lt.png"));
        m_pActRGBL->setIcon(QIcon(":/itomDesignerPlugins/axis_lt/icons/RGBA_RGB_lt.png"));
        m_pActRGBAL->setIcon(QIcon(":/itomDesignerPlugins/axis_lt/icons/RGBA_RGBA_lt.png"));
        m_pActRGBG->setIcon(QIcon(":/itomDesignerPlugins/axis_lt/icons/RGB_RGBGray_lt.png"));
        m_pActRGBSwitch->setIcon(m_pMnuRGBSwitch->defaultAction()->icon());
        m_pActMultiRowSwitch->setIcon(m_pMnuMultiRowSwitch->defaultAction()->icon());
        m_pActGrid->setIcon(QIcon(":/itomDesignerPlugins/plot_lt/icons/grid_lt.png"));

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
void Plot1DWidget::setDefaultValueScaleEngine(const ItomQwtPlotEnums::ScaleEngine &scaleEngine)
{
    if (scaleEngine != m_valueScale)
    {
        if (scaleEngine == ItomQwtPlotEnums::Linear)
        {
            setAxisScaleEngine(QwtPlot::yLeft, new QwtLinearScaleEngine());
        }
        else if ((int)scaleEngine < 1000)
        {
            setAxisScaleEngine(QwtPlot::yLeft, new QwtLogScaleEngine((int)scaleEngine));
        }
        else
        {
            setAxisScaleEngine(QwtPlot::yLeft, new ItomLogLogScaleEngine((int)scaleEngine- 1000));
        }

        m_valueScale = scaleEngine;

        bool recalculateBoundaries = false;

        if (m_pData->m_valueScaleAuto == true)
        {
            m_pData->m_valueMin = qQNaN();
            recalculateBoundaries = true;
        }

        if (m_pData->m_axisScaleAuto == true)
        {
            m_pData->m_axisMin = qQNaN();
            recalculateBoundaries = true;
        }

        updateScaleValues(recalculateBoundaries);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setDefaultAxisScaleEngine(const ItomQwtPlotEnums::ScaleEngine &scaleEngine, bool forceUpdate /*= false*/)
{
    if (scaleEngine != m_axisScale || forceUpdate)
    {
        if (scaleEngine == ItomQwtPlotEnums::Linear)
        {
            if (m_pData && m_pData->m_hasDateTimeXAxis)
            {
                setAxisScaleEngine(QwtPlot::xBottom, new QwtDateScaleEngine(Qt::UTC));
            }
            else
            {
                setAxisScaleEngine(QwtPlot::xBottom, new QwtLinearScaleEngine());
            }
        }
        else if ((int)scaleEngine < 1000)
        {
            setAxisScaleEngine(QwtPlot::xBottom, new QwtLogScaleEngine((int)scaleEngine));
        }
        else
        {
            setAxisScaleEngine(QwtPlot::xBottom, new ItomLogLogScaleEngine((int)scaleEngine- 1000));
        }

        m_axisScale = scaleEngine;

        bool recalculateBoundaries = false;

        if (m_pData->m_valueScaleAuto == true)
        {
            m_pData->m_valueMin = qQNaN();
            recalculateBoundaries = true;
        }

        if (m_pData->m_axisScaleAuto == true)
        {
            m_pData->m_axisMin = qQNaN();
            recalculateBoundaries = true;
        }

        updateScaleValues(recalculateBoundaries);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::toggleLegendLabel(QwtPlotCurve* curve, const bool state)
{
	if (m_pLegend)
	{
		QwtLegendLabel *legendLabel(qobject_cast<QwtLegendLabel*>(m_pLegend->legendWidget(itemToInfo(curve))));
        if (legendLabel)
        {
		    legendLabel->setChecked(state);
        }
	}

}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setLegendPosition(LegendPosition position, bool visible)
{
	if (position != m_legendPosition || visible != m_legendVisible)
	{
		if (m_pLegend)
		{
			m_pLegend->deleteLater();
			m_pLegend = NULL;
		}

		if (visible)
		{
			m_pLegend = new QwtLegend(this);
			m_pLegend->setDefaultItemMode(QwtLegendData::Checkable);
			connect(m_pLegend, SIGNAL(checked(QVariant, bool, int)), this, SLOT(legendItemChecked(QVariant, bool)));
			insertLegend(m_pLegend, position);

			updateLegendItems();

			foreach(QAction *a, m_pMnuLegendSwitch->actions())
			{
				if (a->data().toInt() == position)
				{
					m_pMnuLegendSwitch->setDefaultAction(a);
					a->setChecked(true);
				}
				else
				{
					a->setChecked(false);
				}
			}
		}
		else
		{
			foreach(QAction *a, m_pMnuLegendSwitch->actions())
			{
				if (a->data().toInt() == -1)
				{
					m_pMnuLegendSwitch->setDefaultAction(a);
					a->setChecked(true);
				}
				else
				{
					a->setChecked(false);
				}
			}
			insertLegend(NULL);
		}

		m_legendVisible = visible;
		m_legendPosition = position;
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::updateLegendItems()
{
    if (m_pLegend)
    {
        QwtLegendLabel *legendLabel = NULL;
        QSize maxLegendIconSize(m_pLegendLabelWidth, 0);
        WidgetCurveProperties* widgetCurveProperties = (WidgetCurveProperties*)((Itom1DQwtPlot*)(this->parent()))->getWidgetCurveProperties();
		QwtText text;

        foreach(QwtPlotCurve *item, m_plotCurveItems)
        {
			text = item->title();
			text.setFont(m_legendFont);
			item->setTitle(text);

            item->setLegendAttribute(QwtPlotCurve::LegendShowLine, true);
            item->setLegendAttribute(QwtPlotCurve::LegendShowSymbol, true);
            maxLegendIconSize.rheight() = std::max(maxLegendIconSize.height(), item->legendIconSize().height());
            legendLabel = qobject_cast<QwtLegendLabel*>(m_pLegend->legendWidget(itemToInfo(item)));

            if (legendLabel)
            {
                //set font
                text = legendLabel->data().title();
                text.setFont(m_legendFont);
                legendLabel->setText(text);

                //at first try to disconnect the signal before connecting it again...
                disconnect(legendLabel, SIGNAL(checked(bool)), widgetCurveProperties, SLOT(on_listWidget_itemSelectionChanged()));

                //TODO: the following connection is lost once the single legend entry becomes invisible. If it is displayed again, the signal is not re-established. Changing the legend-position
                //re-connects it again!
                connect(legendLabel, SIGNAL(checked(bool)), widgetCurveProperties, SLOT(on_listWidget_itemSelectionChanged()));

                if (legendLabel)
                {
                    //the check status is again set in QwtPlotCurveProperty::setLegendVisible
                    legendLabel->setChecked(item->isVisible());
                }
            }
        }

        //maxLegendIconSize.rwidth() = m_pLengendLineLength;
        //max icon size: 30w, 18h
        //maxLegendIconSize.rwidth() = std::min(maxLegendIconSize.width(), 30);
        maxLegendIconSize.rheight() = std::min(maxLegendIconSize.height(), 18);

        //adjust legend icon size
        foreach(QwtPlotCurve *item, m_plotCurveItems)
        {
            item->setLegendIconSize(maxLegendIconSize);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setLegendLabelWidth(const int &width)
{
	m_pLegendLabelWidth = width;
	if (m_pLegend)
	{
		QSize maxLegendIconSize(width, 0);
		QwtPlotCurve *item;
		maxLegendIconSize.rwidth() = std::max(width, 10);
		foreach(item, m_plotCurveItems)
		{
			maxLegendIconSize.rheight() = std::max(maxLegendIconSize.height(), item->legendIconSize().height());

		}
		//max icon height: 18
		maxLegendIconSize.rheight() = std::min(maxLegendIconSize.height(), 18);
		m_pLegendLabelWidth = maxLegendIconSize.rwidth();

		//adjust legend icon size
		foreach(QwtPlotCurve *item, m_plotCurveItems)
		{
			item->setLegendIconSize(maxLegendIconSize);
		}
	}
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setLegendFont(const QFont &font)
{
    m_legendFont = font;
    if (m_pLegend)
    {
		QwtText text;
        QwtLegendLabel* legendLabel;
        foreach(QwtPlotCurve *item, m_plotCurveItems)
        {
            for (int i = 0; i < item->legendData().size(); ++i)
			{
                if (item->legendData()[i].hasRole(QwtLegendData::TitleRole))
				{
                    text = item->legendData()[i].title();
					text.setFont(m_legendFont);
					QVariant titleValue;
                    titleValue.setValue(text);
                    item->legendData()[i].setValue(QwtLegendData::TitleRole, titleValue);
				}
			}

            legendLabel = qobject_cast<QwtLegendLabel*>(m_pLegend->legendWidget(itemToInfo(item)));
            if (legendLabel)
            {
                text = legendLabel->data().title();
                text.setFont(m_legendFont);
                legendLabel->setText(text);
            }
        }
    }
    
}
//----------------------------------------------------------------------------------------------------------------------------------
const QFont Plot1DWidget::getLegendFont() const
{
    return m_legendFont;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::applyLegendFont()
{
    if (m_pLegend)
    {
        QwtLegendLabel* legendLabel;
        foreach(QwtPlotCurve *item, m_plotCurveItems)
        {
            if (item->testItemAttribute(QwtPlotItem::Legend) == true)
            {
                legendLabel = qobject_cast<QwtLegendLabel*>(m_pLegend->legendWidget(itemToInfo(item)));
                if (legendLabel)
                {
                    QwtText text(legendLabel->data().title());
                    text.setFont(m_legendFont);
                    legendLabel->setText(text);
                }
            }
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setLegendTitles(const QStringList &legends, const ito::DataObject *object)
{
    // idea: m_legendTitles must always have a size >= number of currently displayed curves
    /*
    1. take the given legends
    2. if there are more curve items, than current legends,
       fill additional values up...
    */
    int index = 0;
    m_legendTitles = legends;

    bool valid = false;
    ito::DataObjectTagType tag;
    QwtPlotCurve *item = NULL;

    for (index = 0; index < m_plotCurveItems.size(); ++index)
    {
        item = m_plotCurveItems[index];

        if (m_legendTitles.size() <= index)
        {
            if (object)
            {
                tag = object->getTag(QString("legendTitle%1").arg(index).toLatin1().toStdString(), valid);              
            }

            if (valid) // plots with legend, defined by tags: legendTitle0, legendTitle1, ...
            {
                m_legendTitles.append(QString(tag.getVal_ToString().data()));
            }
            else // plots with empty tags: curce 0, curve 1, ...
            {
                m_legendTitles.append(tr("curve %1").arg(index));
            }
            
        }

        item->setTitle(m_legendTitles[index]);
    }

    replot();
    applyLegendFont();
	emit legendModified();

}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setGridStyle(const Itom1DQwtPlot::GridStyle gridStyle)
{
    m_gridStyle = gridStyle;
    m_pPlotGrid->enableX(gridStyle == Itom1DQwtPlot::GridMajorX || gridStyle == Itom1DQwtPlot::GridMajorXY || gridStyle == Itom1DQwtPlot::GridMinorX || gridStyle == Itom1DQwtPlot::GridMinorXY);
    m_pPlotGrid->enableY(gridStyle == Itom1DQwtPlot::GridMajorY || gridStyle == Itom1DQwtPlot::GridMajorXY || gridStyle == Itom1DQwtPlot::GridMinorY || gridStyle == Itom1DQwtPlot::GridMinorXY);
    m_pPlotGrid->enableXMin(gridStyle == Itom1DQwtPlot::GridMinorX || gridStyle == Itom1DQwtPlot::GridMinorXY);
    m_pPlotGrid->enableYMin(gridStyle == Itom1DQwtPlot::GridMinorY || gridStyle == Itom1DQwtPlot::GridMinorXY);
    m_pActGrid->setChecked(gridStyle != Itom1DQwtPlot::GridNo);

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
void Plot1DWidget::setLineWidth(const qreal &width)
{
    if (m_lineWidth != width)
    {
        m_lineWidth = width;

        foreach(QwtPlotCurve *c, m_plotCurveItems)
        {
            QPen pen = c->pen();
            pen.setWidthF(width);
            c->setPen(pen);
        }
        applyLegendFont();
        replot();
		emit curveChanged();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setLineStyle(const Qt::PenStyle &style)
{
    if (m_lineStyle != style)
    {
        m_lineStyle = style;

        foreach(QwtPlotCurve *c, m_plotCurveItems)
        {
            QPen pen = c->pen();
            pen.setStyle(style);
            c->setPen(pen);
        }
        applyLegendFont();
        replot();
		emit curveChanged();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setSymbolStyle(const QwtSymbol::Style style, int size)
{
    if (m_pData->m_lineSymbole != style && m_pData->m_lineSymboleSize != size)
    {
        foreach(QwtPlotCurve *c, m_plotCurveItems)
        {
            QPen pen = c->pen();
            const QwtSymbol *s = c->symbol();
            QSize newSize(size, size);
            if (style == QwtSymbol::NoSymbol || size == 0)
            {
                c->setSymbol(NULL);
            }
            else
            {
                c->setSymbol(new QwtSymbol(style, QBrush(Qt::white), QPen(pen.color()), newSize));
            }
        }
        applyLegendFont();
        replot();
		emit curveChanged();
    }
    else if (m_pData->m_lineSymbole != style)
    {
        foreach(QwtPlotCurve *c, m_plotCurveItems)
        {
            QPen pen = c->pen();
            const QwtSymbol *s = c->symbol();
            if (style == QwtSymbol::NoSymbol)
            {
                c->setSymbol(NULL);
            }
            else
            {
                QSize size = s ? s->size() : QSize(m_pData->m_lineSymboleSize, m_pData->m_lineSymboleSize);
                c->setSymbol(new QwtSymbol(style, QBrush(Qt::white), QPen(pen.color()), size));
            }
        }
        applyLegendFont();
        replot();
		emit curveChanged();
    }
    else if (m_pData->m_lineSymboleSize != size)
    {
        foreach(QwtPlotCurve *c, m_plotCurveItems)
        {
            QPen pen = c->pen();
            const QwtSymbol *s = c->symbol();
            if (size == 0)
            {
                c->setSymbol(NULL);
            }
            else
            {
                c->setSymbol(new QwtSymbol(s ? s->style() : m_pData->m_lineSymbole, QBrush(Qt::white), QPen(pen.color()), QSize(size,size)));
            }
        }
        applyLegendFont();
        replot();
		emit curveChanged();
    }

    m_pData->m_lineSymbole = style;
    m_pData->m_lineSymboleSize = size;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::updateLabels()
{
    QwtText t;
    t = axisTitle(QwtPlot::yLeft);
    if (m_pData->m_autoValueLabel)
    {
        t.setText(m_pData->m_valueLabelDObj);
    }
    else
    {
        t.setText(m_pData->m_valueLabel);
    }
    setAxisTitle(QwtPlot::yLeft, t);

    t = axisTitle(QwtPlot::xBottom);
    if (m_pData->m_autoAxisLabel)
    {
        t.setText(m_pData->m_axisLabelDObj);
    }
    else
    {
        t.setText(m_pData->m_axisLabel);
    }
    setAxisTitle(QwtPlot::xBottom, t);

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
void Plot1DWidget::refreshPlot(const ito::DataObject* dataObj, QVector<QPointF> bounds, const ito::DataObject* xVec /*=NULL*/)
{
    ito::RetVal retval(ito::retOk);
    DataObjectSeriesData* seriesData = nullptr;
    int colorIndex;
    int numCurves = 1;
    QwtPlotCurve *curve = nullptr;
    QwtPlotCurveDataObject *dObjCurve = nullptr;
    bool boundsChanged = false;
    bool _unused;
    QwtLegendLabel *legendLabel = nullptr;
    int index;

    // currently this feature is not implemented for an x-over-y plot.
    m_pActSetPickerMinMaxLocal->setEnabled(xVec == nullptr);

    // check for valid data types
    if (dataObj && (dataObj->getType() == ito::tDateTime || dataObj->getType() == ito::tTimeDelta))
    {
        emit statusBarMessage(tr("A dateTime or timeDelta dataObject is not supported by this plot."), 10000);
        dataObj = nullptr;
    }
    else if (xVec && xVec->getType() == ito::tTimeDelta)
    {
        emit statusBarMessage(tr("A timeDelta x-axis dataObject is not supported by this plot."), 10000);
        dataObj = nullptr;
    }

    if (dataObj)
    {
        int dims = dataObj->getDims();

        if (dims == 0)
        {
            bounds.clear();
        }

        int width = dims > 0 ? dataObj->getSize(dims - 1) : 0;
        int height = dims > 1 ? dataObj->getSize(dims - 2) : (width == 0) ? 0 : 1;
		m_pData->m_dataType = (ito::tDataType)dataObj->getType();
        m_pData->m_hasDateTimeXAxis = false;

        if (m_pData->m_dataType == ito::tComplex128 || 
            m_pData->m_dataType == ito::tComplex64)
        {
            enableObjectGUIElements(2 /*complex*/ | (dims > 1 ? 0x10 : 0x00) /*multi-layer: yes : no*/);
            m_pRescaleParent->setVisible(bounds.size() > 1 && m_hasParentForRescale); //a z-stack 1d plot should not be able to rescale its parent (therefore the bounds.size() check, 1: z-stack, 2: line-cut 2D object, 3: line-cut 3D object (first bounds is the plane)).
        }
        else if (m_pData->m_dataType == ito::tRGBA32)
        {
            enableObjectGUIElements(1 /*rgba, no multi-layer*/);
            m_pRescaleParent->setVisible(false); //a coloured data object has no color map and can therefore not be cropped.
        }
        else
        {
            enableObjectGUIElements(0 /*gray*/ | (dims > 1 ? 0x10 : 0x00) /*multi-layer: yes : no*/);
            m_pRescaleParent->setVisible(bounds.size() > 1 && m_hasParentForRescale); //a z-stack 1d plot should not be able to rescale its parent (therefore the bounds.size() check, 1: z-stack, 2: line-cut 2D object, 3: line-cut 3D object (first bounds is the plane)).
        }

        //if this 1d plot is based on bounds (hence, a line cut or similar of a 2d cut, all pickers should be deleted if the boundaries changed)
        if (bounds.size() != m_currentBounds.size())
        {
            clearPicker(-1, false); 
        }
        else
        {
            //if a 3d object is displayed, the first bounds is the plane, however the plane is no reason for deleting old markers
            for (int i = bounds.size() == 3 ? 1 : 0; i < bounds.size(); ++i) 
            {
                if (bounds[i] != m_currentBounds[i])
                {
                    this->updatePickerPosition(true, false);
                    break;
                }
            }
        }

        if (m_currentBounds != bounds)
        {
            boundsChanged = true;
            m_currentBounds = bounds;
        }
        
        ItomQwtPlotEnums::MultiLineMode multiLineMode = m_pData->m_multiLine;

        if (m_pData->m_dataType == ito::tRGBA32)
        {
            m_layerState = false;
            if (bounds.size() == 0) m_pData->m_multiLine = width == 1 ? ItomQwtPlotEnums::FirstCol : ItomQwtPlotEnums::FirstRow;
            m_legendTitles.clear();
            switch(m_pData->m_colorLine)
            {
                case ItomQwtPlotEnums::Gray:
                    numCurves = 1;
                    m_legendTitles << tr("gray");
                break;
                case ItomQwtPlotEnums::RGB:
                    numCurves = 3;
                    m_legendTitles << tr("blue") << tr("green") << tr("red");
                break;
                case ItomQwtPlotEnums::RGBA:
                    numCurves = 4;
                    m_legendTitles << tr("blue") << tr("green") << tr("red") << tr("alpha");
                break;
                case ItomQwtPlotEnums::RGBGray:
                    numCurves = 4;
                    m_legendTitles << tr("blue") << tr("green") << tr("red") << tr("gray");
                break;
                default:
                    numCurves = 3;
                    m_pData->m_colorLine = ItomQwtPlotEnums::RGB;
                    m_legendTitles << tr("blue") << tr("green") << tr("red");
                break;
            }
        }
        else if (bounds.size() == 0)
        {
            m_pData->m_colorLine = ItomQwtPlotEnums::AutoColor;
            switch (m_pData->m_multiLine)
            {
                case ItomQwtPlotEnums::FirstRow:
                case ItomQwtPlotEnums::FirstCol:
                    m_layerState = false;
                    numCurves = 1;
                    break;
                case ItomQwtPlotEnums::MultiRows:
                    m_layerState = false;
                    numCurves = height;
                    break;
                case ItomQwtPlotEnums::MultiCols:
                    m_layerState = false;
                    numCurves = width;
                    break;
                case ItomQwtPlotEnums::MultiLayerAuto:
                    if (width == 1 && height == 1)
                    {
                        multiLineMode = ItomQwtPlotEnums::MultiLayerRows;
                    }
                    else if (width >= height)
                    {
                        multiLineMode = ItomQwtPlotEnums::MultiLayerRows;
                    }
                    else
                    {
                        multiLineMode = ItomQwtPlotEnums::MultiLayerCols;
                    }
                case ItomQwtPlotEnums::MultiLayerCols:
                case ItomQwtPlotEnums::MultiLayerRows:
                    numCurves = dims > 2 ? dataObj->getSize(dims - 3) : 1;
                    m_layerState = true;
                    break;
                default:
                {
                    m_layerState = false;
                    if (width == 1 && height == 1 && dims < 3)
                    {
                        multiLineMode = ItomQwtPlotEnums::MultiRows;
                        numCurves = height;
                    }
                    else if (width >= height)
                    {
                        numCurves = height;
                        multiLineMode = ItomQwtPlotEnums::MultiRows;
                    }
                    else
                    {
                        numCurves = width;
                        multiLineMode = ItomQwtPlotEnums::MultiCols;
                    }
                }
            }
        }
        else //if there are boundaries, only plot one curve from bounds[0] to bounds[1]
        {
            if (bounds.size() == 3)
            {
                if (m_pData->m_multiLine == ItomQwtPlotEnums::MultiLayerCols || 
                   m_pData->m_multiLine == ItomQwtPlotEnums::MultiLayerRows || 
                   m_pData->m_multiLine == ItomQwtPlotEnums::MultiLayerAuto)
                {
                    m_layerState = true;
                    numCurves = dims > 2 ? dataObj->getSize(dims - 3) : 1;
                }
                else
                {
                    m_layerState = false;
                    numCurves = 1;
                }
            }
            else
            {
                numCurves = 1;
                m_layerState = false;
            }
            m_pData->m_colorLine = ItomQwtPlotEnums::AutoColor;
        }

        //check if current number of curves does not correspond to height. If so, adjust the number of curves to the required number
        //remove and delete all 'old' curves, whose index is bigger than 'numCurves'
		bool refreshWidgetCurveProperties = removeAllCurvesBesideTheNFirst(numCurves);

        //add new curves to the canvas until a number of 'numCurves' exists.
        bool valid;
        ito::DataObjectTagType tag;
        
        QList<QString> curveNames = m_legendTitles.mid(0, m_plotCurveItems.size());
        int legendOffset = getLegendOffset(dataObj, multiLineMode);
        WidgetCurveProperties* widgetCurveProperties = (WidgetCurveProperties*)((Itom1DQwtPlot*)(this->parent()))->getWidgetCurveProperties();

        while (m_plotCurveItems.size() < numCurves) // do until all curves are present
        {
            index = m_plotCurveItems.size();

            if (m_legendTitles.size() <= index) //true if there are fewer or equal legend entries such as curves
            {
                tag = dataObj->getTag(QString("legendTitle%1").arg(index + legendOffset).toLatin1().toStdString(), valid);
                
				if(valid) // plots with legend, defined by tags: legendTitle0, legendTitle1, ...
				{
					dObjCurve = new QwtPlotCurveDataObject(QString::fromLatin1(tag.getVal_ToString().data()));   
                    curveNames.append(QString(tag.getVal_ToString().data()));
				}
				else // plots with empty tags: curce 0, curve 1, ...
				{
					dObjCurve = new QwtPlotCurveDataObject(tr("curve %1").arg(index+legendOffset));
                    curveNames.append(tr("curve %1").arg(index + legendOffset));
				}	
            }
            else // if (m_legendTitles.size() > index)
            {
                dObjCurve = new QwtPlotCurveDataObject(m_legendTitles[index]);
                curveNames.append(m_legendTitles[index]);
            }           

            dObjCurve->setData(nullptr);
            dObjCurve->setRenderHint(QwtPlotItem::RenderAntialiased, m_antiAliased);
            dObjCurve->attach(this);

            if (m_pLegend)
            {
                legendLabel = qobject_cast<QwtLegendLabel*>(m_pLegend->legendWidget(itemToInfo(dObjCurve)));
                if (legendLabel)
                {
                    legendLabel->setChecked(true);
                }
            }

			QwtText text = dObjCurve->title();
			text.setFont(m_legendFont);
			dObjCurve->setTitle(text);

            QPen plotPen;
            colorIndex = m_plotCurveItems.size() % m_colorList.size();
            plotPen.setColor(m_colorList[colorIndex]);
            plotPen.setStyle(m_lineStyle);
            plotPen.setWidth(m_lineWidth);
            dObjCurve->setPen(plotPen);

            // Add Symbol here
            if (m_pData->m_lineSymbole != QwtSymbol::NoSymbol)
            {
                dObjCurve->setSymbol(
                    new QwtSymbol(
                        m_pData->m_lineSymbole, 
                        QBrush(Qt::white), 
                        QPen(m_colorList[colorIndex]), 
                        QSize(m_pData->m_lineSymboleSize, m_pData->m_lineSymboleSize)
                    )
                );
            }

            setQwtLineStyle(dObjCurve, m_qwtCurveStyle);

            dObjCurve->setBaseline(m_baseLine);
            dObjCurve->setCurveFilled(m_curveFilled);
            if (m_curveFilled != ItomQwtPlotEnums::NoCurveFill)
            {
                if (m_filledColor.isValid())
                {
                    dObjCurve->setBrush(QBrush(m_filledColor));
                }
                else
                {
                    QColor fill = m_colorList[colorIndex];
                    fill.setAlpha(m_fillCurveAlpa);
                    dObjCurve->setBrush(QBrush(fill));
                }
            }
            else
            {
                dObjCurve->setBrush(Qt::NoBrush);
            }
            m_plotCurveItems.append(dObjCurve);
            m_plotCurvePropertyItems.append(new QwtPlotCurveProperty(dObjCurve));

            connect(
                m_plotCurvePropertyItems.last(),
                &QwtPlotCurveProperty::curveChanged,
                widgetCurveProperties,
                &WidgetCurveProperties::on_listWidget_itemSelectionChanged);

			refreshWidgetCurveProperties = true;
        }
 

		// re-use old legend titles entries, if no new curves have been appended 
        // (e.g. important if 'the' same shape of object is only updated via resetting its source property
		for (int i = curveNames.size(); i < std::min(numCurves, m_legendTitles.size()); ++i)
		{
			curveNames.append(m_legendTitles[i]);
		}

        // update legend since offset has changed
        if (m_legendOffset != legendOffset) 
        {
            for (int i = 0; i < curveNames.size(); ++i)
            {
                tag = dataObj->getTag(QString("legendTitle%1").arg(i + legendOffset).toLatin1().toStdString(), valid);

                if (valid) // plots with legend, defined by tags: legendTitle0, legendTitle1, ...
                {
                    m_plotCurveItems[i]->setTitle(QString::fromLatin1(tag.getVal_ToString().data()));
                    curveNames[i] = QString(tag.getVal_ToString().data());
                }
                else // plots with empty tags: curce 0, curve 1, ...
                {
                    m_plotCurveItems[i]->setTitle(tr("curve %1").arg(i + legendOffset));
                    curveNames[i] = QString(tr("curve %1").arg(i + legendOffset));
                }
            }
        }

        m_legendTitles = curveNames;
        m_legendOffset = legendOffset;

		if (refreshWidgetCurveProperties) 
		{	
            // if true a curve was added or deleted and the widget has to be updated
			widgetCurveProperties->updateProperties();
            updateLegendItems();
		}

        if (bounds.size() == 0) //default plot
        {
            QVector<QPointF> pts(2);
            QVector<QPointF> ptsX(2);
            int previousAxisState;

            switch(multiLineMode)
            {
            case ItomQwtPlotEnums::MultiLayerCols:
            case ItomQwtPlotEnums::FirstCol:
            case ItomQwtPlotEnums::MultiCols:
                if (m_layerState)
                {
                    pts.resize(3);
                    pts[1].setY(dataObj->getPixToPhys(dims-2, 0, _unused));
                    pts[2].setY(dataObj->getPixToPhys(dims-2, height-1, _unused)); 
                }
                else
                {
                    pts[0].setY(dataObj->getPixToPhys(dims-2, 0, _unused));
                    pts[1].setY(dataObj->getPixToPhys(dims-2, height-1, _unused)); 
                }

                previousAxisState = m_pData->m_axisState;

                if (xVec) //we need to check if the dataObject is valid
                {
                        m_pData->m_axisState = ItomQwtPlotEnums::xAxisObject | ItomQwtPlotEnums::colState;
                        retval += validateXData(dataObj, xVec, pts);

                        if ((m_pData->m_axisState & ItomQwtPlotEnums::mismatch) || (m_pData->m_axisState & ItomQwtPlotEnums::noPerfectFit))
                        {
                            emit statusBarMessage(QLatin1String(retval.errorMessage()), 10000);
                        }
                        if (m_pData->m_axisState & ItomQwtPlotEnums::mismatch)
                        {
                            // since the xVec does not fit we go back to evenlyspaced plot
                            m_pData->m_axisState = ItomQwtPlotEnums::evenlySpaced; 
                        }
                        else
                        {
                            ptsX[0].setY(xVec->getPixToPhys(xVec->getDims() - 2, 0, _unused));
                            ptsX[1].setY(xVec->getPixToPhys(xVec->getDims() - 2, xVec->getSize(xVec->getDims() - 2)-1, _unused));
                        }
                }
                else
                {
                    m_pData->m_axisState = ItomQwtPlotEnums::evenlySpaced;
                }

                for (int n = 0; n < numCurves; n++)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[n]->data());

                    if (m_layerState)
                    {
                        pts[0].setX(n);
                        pts[0].setY(n);
                        pts[1].setX(dataObj->getPixToPhys(dims-1, 0, _unused));
                        pts[2].setX(dataObj->getPixToPhys(dims-1, 0, _unused));
                    }
                    else
                    {
                        pts[0].setX(dataObj->getPixToPhys(dims-1, m_pData->m_dataType == ito::tRGBA32 ? 0 : n, _unused));
                        pts[1].setX(dataObj->getPixToPhys(dims-1, m_pData->m_dataType == ito::tRGBA32 ? 0 : n, _unused));
                    }

                    if (m_pData->m_axisState & ItomQwtPlotEnums::xAxisObject)
                    {
                        ptsX[0].setX(xVec->getPixToPhys(xVec->getDims() - 1, n, _unused));
                        ptsX[1].setX(ptsX[0].x());
                    }

                    m_pData->m_axisState & ItomQwtPlotEnums::xAxisObject ? setSendCurrentViewState(false) : setSendCurrentViewState(true);

                    if (seriesData && seriesData->isDobjInit())
                    {
                        if (m_pData->m_axisState & ItomQwtPlotEnums::evenlySpaced)
                        {
                            if (previousAxisState & ItomQwtPlotEnums::evenlySpaced)
                            {
                                seriesData->updateDataObject(dataObj, pts);
                            }
                            else
                            {
                                seriesData = new DataObjectSeriesData(1);
                                seriesData->updateDataObject(dataObj, pts);
                                m_plotCurveItems[n]->setData(seriesData);
                            }
                        }
                        else if (m_pData->m_axisState & ItomQwtPlotEnums::xAxisObject)
                        {
                            if (previousAxisState & ItomQwtPlotEnums::xAxisObject)
                            {
                                seriesData->updateDataObject(dataObj, pts, xVec, ptsX);
                            }
                            else
                            {
                                seriesData = new DataObjectSeriesDataXY(1);
                                seriesData->updateDataObject(dataObj, pts, xVec, ptsX);
                                m_plotCurveItems[n]->setData(seriesData);
                            }
                        }
                    }
                    else
                    {
                        if (m_pData->m_axisState & ItomQwtPlotEnums::xAxisObject)
                        {
                            seriesData = new DataObjectSeriesDataXY(1);
                            seriesData->updateDataObject(dataObj, pts, xVec, ptsX);

                        }
                        else
                        {
                            seriesData = new DataObjectSeriesData(1);
                            seriesData->updateDataObject(dataObj, pts);
                        }
                        m_plotCurveItems[n]->setData(seriesData);
                    }

                    if (m_pData->m_dataType == ito::tRGBA32)
                    {
                        if (m_pData->m_colorLine == ItomQwtPlotEnums::Gray) seriesData->setColorState(DataObjectSeriesData::grayColor);
                        else if (m_pData->m_colorLine == ItomQwtPlotEnums::RGB || m_pData->m_colorLine == ItomQwtPlotEnums::RGBA) seriesData->setColorState(n);
                        else if (m_pData->m_colorLine == ItomQwtPlotEnums::RGBGray) seriesData->setColorState(n == 3 ? 4 : n);
                    }
                }

                if (numCurves > 0)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
                    m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel(unitLabelStyle());
                    m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel(unitLabelStyle());
                }
                break;

            case ItomQwtPlotEnums::AutoRowCol:
            case ItomQwtPlotEnums::MultiLayerAuto:
            case ItomQwtPlotEnums::MultiLayerRows:
            case ItomQwtPlotEnums::FirstRow:
            case ItomQwtPlotEnums::MultiRows:

                if (m_layerState)
                {
                    pts.resize(3);
                    pts[1].setX(dataObj->getPixToPhys(dims-1, 0, _unused));
                    pts[2].setX(dataObj->getPixToPhys(dims-1, width-1, _unused));
                }
                else
                {
                    pts[0].setX(dataObj->getPixToPhys(dims - 1, 0, _unused));
                    pts[1].setX(dataObj->getPixToPhys(dims - 1, width - 1, _unused));            
                }
                previousAxisState = m_pData->m_axisState;

                if (xVec) //we need to check if the dataObject is valid
                {

                        m_pData->m_axisState = ItomQwtPlotEnums::xAxisObject | ItomQwtPlotEnums::rowState;
                        retval += validateXData(dataObj, xVec, pts);
                        if ((m_pData->m_axisState & ItomQwtPlotEnums::mismatch) || (m_pData->m_axisState & ItomQwtPlotEnums::noPerfectFit))
                        {
                            emit statusBarMessage(QObject::tr(retval.errorMessage()).toLatin1().data(), 10000);
                        }
                        if (m_pData->m_axisState & ItomQwtPlotEnums::mismatch)
                        {
                            //xVec = NULL;
                            m_pData->m_axisState = ItomQwtPlotEnums::evenlySpaced; //since the xVec does not fit we go back to evenlyspaced plot
                        }
                        else
                        {
                            ptsX[0].setX(xVec->getPixToPhys(xVec->getDims() - 1, 0, _unused));
                            ptsX[1].setX(xVec->getPixToPhys(xVec->getDims() - 1, xVec->getSize(xVec->getDims() - 1)-1, _unused));
                        }
                }
                else
                {
                    m_pData->m_axisState = ItomQwtPlotEnums::evenlySpaced;
                }

                for (int n = 0; n < numCurves; n++)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[n]->data());

                    if (m_layerState)
                    {
                        pts[0].setX(n);
                        pts[0].setY(n);
                        pts[1].setY(dataObj->getPixToPhys(dims-2, 0, _unused));
                        pts[2].setY(dataObj->getPixToPhys(dims-2, 0, _unused));
                    }
                    else
                    {
                        pts[0].setY(dataObj->getPixToPhys(dims-2, m_pData->m_dataType == ito::tRGBA32 ? 0 : n, _unused));
                        pts[1].setY(dataObj->getPixToPhys(dims-2, m_pData->m_dataType == ito::tRGBA32 ? 0 : n, _unused));
                    }

                    if (m_pData->m_axisState & ItomQwtPlotEnums::xAxisObject)
                    {
                        ptsX[0].setY(xVec->getPixToPhys(xVec->getDims() - 2, n, _unused));
                        ptsX[1].setY(ptsX[0].y());
                    }

                    m_pData->m_axisState & ItomQwtPlotEnums::xAxisObject ? setSendCurrentViewState(false) : setSendCurrentViewState(true);

                    if (seriesData && seriesData->isDobjInit())
                    {
                        if (m_pData->m_axisState & ItomQwtPlotEnums::evenlySpaced)
                        {
                            if (previousAxisState & ItomQwtPlotEnums::evenlySpaced)
                            {
                                seriesData->updateDataObject(dataObj, pts);
                            }
                            else
                            {
                                seriesData = new DataObjectSeriesData(1);
                                seriesData->updateDataObject(dataObj, pts);
                                m_plotCurveItems[n]->setData(seriesData);
                            }
                        }
                        else if(m_pData->m_axisState & ItomQwtPlotEnums::xAxisObject)
                        {
                            if (previousAxisState & ItomQwtPlotEnums::xAxisObject)
                            {
                                seriesData->updateDataObject(dataObj, pts, xVec, ptsX);
                            }
                            else
                            {
                                seriesData = new DataObjectSeriesDataXY(1);
                                seriesData->updateDataObject(dataObj, pts, xVec, ptsX);
                                m_plotCurveItems[n]->setData(seriesData);
                            }
                        }
                    }
                    else
                    {
                        if (m_pData->m_axisState & ItomQwtPlotEnums::xAxisObject)
                        {
                            seriesData = new DataObjectSeriesDataXY(1);
                            seriesData->updateDataObject(dataObj, pts, xVec, ptsX);
                            
                        }
                        else 
                        {
                            seriesData = new DataObjectSeriesData(1);
                            seriesData->updateDataObject(dataObj, pts);
                        }
                        m_plotCurveItems[n]->setData(seriesData);
                    }

                    if (m_pData->m_dataType == ito::tRGBA32)
                    {
                        if (m_pData->m_colorLine == ItomQwtPlotEnums::Gray) seriesData->setColorState(DataObjectSeriesData::grayColor);
                        else if (m_pData->m_colorLine == ItomQwtPlotEnums::RGB || m_pData->m_colorLine == ItomQwtPlotEnums::RGBA) seriesData->setColorState(n);
                        else if (m_pData->m_colorLine == ItomQwtPlotEnums::RGBGray) seriesData->setColorState(n == 3 ? 4 : n);
                    }
                }

                if (numCurves > 0)
                {
                    seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
                    m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel(unitLabelStyle());
                    m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel(unitLabelStyle());
                }
                break;
            } 
        }
        else if (bounds.size() > 1 && bounds.size() < 4) //boundaries given ->line plot
        {
            QVector<QPointF> tmpBounds = bounds;
            for (int n = 0; n < numCurves; n++)
            {
                seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[n]->data());
                if (tmpBounds.size() == 3 && m_layerState)
                {
                    tmpBounds[0].setX(n);
                    tmpBounds[0].setY(n);
                }
                if (seriesData && seriesData->isDobjInit())
                {
                    seriesData->updateDataObject(dataObj, tmpBounds);
                }
                else
                {
                    seriesData = new DataObjectSeriesData(1);
                    seriesData->updateDataObject(dataObj, tmpBounds);
                    seriesData->setCmplxState(m_pComplexStyle);
                    m_plotCurveItems[n]->setData(seriesData);
                }
                if (m_pData->m_dataType == ito::tRGBA32)
                {
                    if (m_pData->m_colorLine == ItomQwtPlotEnums::Gray)
                    {
                        seriesData->setColorState(DataObjectSeriesData::grayColor);
                    }
                    else if (m_pData->m_colorLine == ItomQwtPlotEnums::RGB || m_pData->m_colorLine == ItomQwtPlotEnums::RGBA)
                    {
                        seriesData->setColorState(n);
                    }
                    else if (m_pData->m_colorLine == ItomQwtPlotEnums::RGBGray)
                    {
                        seriesData->setColorState(n == 3 ? 4 : n);
                    }
                }
            }

            if (numCurves > 0)
            {
                seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
                m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel(unitLabelStyle());
                m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel(unitLabelStyle());
            }
        }
        else if (bounds.size() == 1) //point in third dimension
        {
            for (int n = 0; n < numCurves; n++)
            {
                seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[n]->data());
                if (seriesData && seriesData->isDobjInit())
                {
                    seriesData->updateDataObject(dataObj, bounds);
                }
                else
                {
                    seriesData = new DataObjectSeriesData(1);
                    seriesData->updateDataObject(dataObj, bounds);
                    seriesData->setCmplxState(m_pComplexStyle);
                    m_plotCurveItems[n]->setData(seriesData);
                }

                if (m_pData->m_dataType == ito::tRGBA32)
                {
                    if (m_pData->m_colorLine == ItomQwtPlotEnums::Gray) seriesData->setColorState(DataObjectSeriesData::grayColor);
                    else if (m_pData->m_colorLine == ItomQwtPlotEnums::RGB || m_pData->m_colorLine == ItomQwtPlotEnums::RGBA) seriesData->setColorState(n);
                    else if (m_pData->m_colorLine == ItomQwtPlotEnums::RGBGray) seriesData->setColorState(n == 3 ? 4 : n);
                }
            }
            if (numCurves > 0)
            {
                seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[0]->data());
                m_pData->m_valueLabelDObj = seriesData->getDObjValueLabel(unitLabelStyle());
                m_pData->m_axisLabelDObj = seriesData->getDObjAxisLabel(unitLabelStyle());
            }
        }

        tag = dataObj->getTag("title", valid);
        m_pData->m_titleDObj = valid? QString::fromLatin1(tag.getVal_ToString().data()) : "";

        if (xVec && (xVec->getType() == ito::tDateTime))
        {
            m_pData->m_hasDateTimeXAxis = true;
            m_pValuePicker->setXAxisIsDateTime(true);

            if (dynamic_cast<QwtDateScaleDraw*>(axisScaleDraw(QwtAxis::XBottom)) == nullptr)
            {
                setDefaultAxisScaleEngine(m_axisScale, true);

                auto bottomScaleDraw = new QwtDateScaleDraw(Qt::UTC);

                // ISO 8601 (en)
                bottomScaleDraw->setDateFormat(QwtDate::Millisecond, "yyyy-MM-dd\nhh:mm:ss:zzz");
                bottomScaleDraw->setDateFormat(QwtDate::Second, "yyyy-MM-dd\nhh:mm:ss");
                bottomScaleDraw->setDateFormat(QwtDate::Minute, "yyyy-MM-dd\nhh:mm");
                bottomScaleDraw->setDateFormat(QwtDate::Hour, "yyyy-MM-dd\nhh:mm");
                bottomScaleDraw->setDateFormat(QwtDate::Day, "yyyy-MM-dd");
                bottomScaleDraw->setDateFormat(QwtDate::Week, "yyyy-Www");
                bottomScaleDraw->setDateFormat(QwtDate::Month, "MMM yyyy");
                bottomScaleDraw->setDateFormat(QwtDate::Year, "yyyy");

                setAxisScaleDraw(QwtAxis::XBottom, bottomScaleDraw);
                styleXYScaleWidgets();
            }
        }
        else
        {
            m_pData->m_hasDateTimeXAxis = false;
            m_pValuePicker->setXAxisIsDateTime(false);

            if (dynamic_cast<QwtDateScaleDraw*>(axisScaleDraw(QwtAxis::XBottom)) != nullptr)
            {
                setDefaultAxisScaleEngine(m_axisScale, true);
                setAxisScaleDraw(QwtAxis::XBottom, new QwtScaleDraw());
                styleXYScaleWidgets();
            }
        }
    } 
    else
    {
        removeAllCurvesBesideTheNFirst(0);
    }

    updateLabels();

    if (seriesData)
    {
        QByteArray hash = seriesData->getHash();
        QByteArray hashX(m_pData->m_axisState & ItomQwtPlotEnums::xAxisObject ? seriesData->getHash() : "");

        if (hash != m_hash || hashX != m_hashX || m_pData->m_valueScaleAuto)
        {
            updatePickerPosition(true);

            //get the bounding rectangle of all bounding rectangles of all curves
            QRectF rect;

            foreach(QwtPlotCurve *curve, m_plotCurveItems)
            {
                rect |= curve->boundingRect();
            }

            if (rect.width() < 0 || rect.height() <0)
            {
                axisScaleEngine(QwtPlot::xBottom)->setAttribute(QwtScaleEngine::Inverted);
            }
            else
            {
                axisScaleEngine(QwtPlot::xBottom)->setAttribute(QwtScaleEngine::Inverted, false);
            }

            if (m_pData->m_valueScaleAuto)
            {
                if (qIsFinite(rect.height()))
                {
                    m_pData->m_valueMin = rect.top();
                    m_pData->m_valueMax = rect.bottom();
                }
                else
                {
                    m_pData->m_valueMin = -0.01;
                    m_pData->m_valueMax = 0.01;
                }
            }

            if (m_pData->m_axisScaleAuto)
            {
                m_pData->m_axisMin = rect.left();
                m_pData->m_axisMax = rect.right();
            }

            updateScaleValues(false, true, boundsChanged);

            zoomer()->rescale(false);

        }
        else if (m_pData->m_forceValueParsing)
        {

            for (int i = 0; m_pickers.size() > i; ++i)
            {
                if (m_pickers[i].curveIdx >= m_plotCurveItems.size())//check if curve of picker still exists. If not set curveIDx of picker to zero
                    m_pickers[i].curveIdx = 0;
            }
            updatePickerPosition(true);

            
            //get the bounding rectangle of all bounding rectangles of all curves
            QRectF rect;

            foreach(QwtPlotCurve *curve, m_plotCurveItems)
            {
                rect |= curve->boundingRect();
            }

            if (m_pData->m_valueScaleAuto)
            {
                m_pData->m_valueMin = rect.top();
                m_pData->m_valueMax = rect.bottom();
            }

            if (m_pData->m_axisScaleAuto)
            {
                m_pData->m_axisMin = rect.left();
                m_pData->m_axisMax = rect.right();
            }

            updateScaleValues(false, false); //no replot is done here
            zoomer()->rescale(false);

            m_pData->m_forceValueParsing = false;
        }
        else
        {
            updatePickerPosition(true, false);
            updateScaleValues(true, false); //do replot
            //replot();
        }

        m_hash = hash;
        m_hashX = hashX;

		if (dataObj && ((Itom1DQwtPlot*)(this->parent()))->dObjectWidget())
		{
			PlotInfoDObject* infoWidget = (((Itom1DQwtPlot*)(this->parent()))->dObjectWidget());
			infoWidget->updateInfoHeader(
				tr("1D Object Slice"),
				dataObj->getType(),
				dataObj->getDims(),
				dataObj->getSize());

			if (infoWidget->useDetailInfo())
			{
				double min = 0.0, max = 0.0, mean = 0.0, dev = 0.0;
				ito::uint32 minLoc[3];
				ito::uint32 maxLoc[3];
				ito::dObjHelper::minMaxValue(dataObj, min, minLoc, max, maxLoc, true);
				ito::dObjHelper::devValue(dataObj, 0, mean, dev, true);
				infoWidget->updateInfoDetail(min, max, mean, dev);
			}
		}
    }
    else
    {
        replot();
    }
}

//------------------------------------------------------------------------------
bool Plot1DWidget::removeAllCurvesBesideTheNFirst(int numCurvesToKeep)
{
    QwtPlotCurveProperty *curveProp = nullptr;
    bool curvesDeleted = false;
    QwtPlotCurve* curve = nullptr;

    //remove and delete all 'old' curves, whose index is bigger than 'numCurves'
    while (m_plotCurveItems.size() > numCurvesToKeep)
    {
        curve = m_plotCurveItems.takeLast();
        curveProp = m_plotCurvePropertyItems.takeLast();
        DELETE_AND_SET_NULL(curveProp);

        curve->detach();
        DELETE_AND_SET_NULL(curve);

        curvesDeleted = true;
    }

    return curvesDeleted;
}

//------------------------------------------------------------------------------
int Plot1DWidget::getLegendOffset(const ito::DataObject *dataObj, ItomQwtPlotEnums::MultiLineMode multiLineMode) const
{
    int legendOffsetRoiIdx = -1;
    int legendOffset = 0;

    switch (multiLineMode)
    {
    case ItomQwtPlotEnums::MultiLayerAuto:
    case ItomQwtPlotEnums::MultiLayerRows:
    case ItomQwtPlotEnums::FirstRow:
    case ItomQwtPlotEnums::MultiRows:
    case ItomQwtPlotEnums::AutoRowCol:
        legendOffsetRoiIdx = 2 * dataObj->getDims() - 4;
        break;
    case ItomQwtPlotEnums::MultiLayerCols:
    case ItomQwtPlotEnums::FirstCol:
    case ItomQwtPlotEnums::MultiCols:
        legendOffsetRoiIdx = 2 * dataObj->getDims() - 2;
        break;
    default:
        legendOffset = 0;
    }

    if (legendOffsetRoiIdx >= 0)
    {
        int *roiLims = new int[2 * dataObj->getDims()];
        dataObj->locateROI(roiLims);
        legendOffset = roiLims[legendOffsetRoiIdx];
        delete[] roiLims;
        roiLims = nullptr;
    }

    return legendOffset;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::validateXData(const ito::DataObject* dataObj, const ito::DataObject* xVec, const QVector<QPointF> &bounds)
{
    ito::RetVal retval;

    if (dataObj != nullptr && xVec != nullptr)
    {
        int curveAxisShapeX, dataAxisShapeX, dataAxisShapeDObj ,curveAxisShapeDObj;

        if (bounds.size() == 2)
        {
            if(m_pData->m_axisState & ItomQwtPlotEnums::rowState)
            {
                curveAxisShapeX = xVec->getSize(xVec->getDims() - 2);//representing the number of x-vectors
                curveAxisShapeDObj = dataObj -> getSize(dataObj->getDims()-2);//representing the number of values inside an x-vector
                dataAxisShapeX = xVec->getSize(xVec->getDims() - 1);
                dataAxisShapeDObj = dataObj -> getSize(dataObj->getDims()-1);
            }
            else
            {
                curveAxisShapeX = xVec->getSize(xVec->getDims() - 1);
                curveAxisShapeDObj = dataObj -> getSize(dataObj->getDims()-1);
                dataAxisShapeX = xVec->getSize(xVec->getDims() - 2);
                dataAxisShapeDObj = dataObj -> getSize(dataObj->getDims()-2);
            }
            int dimsX = xVec->getDims();
            int prependOneDimsX = 0;
            int i;
            for (i = 0; i < dimsX - 2; ++i)
            {
                if (xVec->getSize(i) != 1)
                {
                    break;
                }
                ++prependOneDimsX;
            }
            if (dimsX == 0) //dataObject is empty set to missmatch to remove xVec
            {
                //xVec = NULL; // if XVec is set to None the dataObject will have no dimensions
                m_pData->m_axisState = m_pData->m_axisState | ItomQwtPlotEnums::mismatch;
            }
            else if ((dimsX - prependOneDimsX) != 2)//check if xVec is 2d
            {
                retval += RetVal(retError, 0, tr("xData requires a 2d dataObject or the first (n-2) dimension must have a shape of 1").toLatin1().data());
                m_pData->m_axisState = m_pData->m_axisState | ItomQwtPlotEnums::mismatch;
            }

            else if (dataAxisShapeDObj > dataAxisShapeX)
            {
                m_pData->m_axisState = m_pData->m_axisState | ItomQwtPlotEnums::mismatch;

                if (m_pData->m_axisState & ItomQwtPlotEnums::rowState)
                {
                    retval += RetVal(retError, 0, tr("wrong xData shape. Expect a shape of (1 x %2) or (%1 x %2)").arg(curveAxisShapeDObj).arg(dataAxisShapeDObj).toLatin1().data());
                }
                else if (m_pData->m_axisState & ItomQwtPlotEnums::colState)
                {
                    retval += RetVal(retError, 0, tr("wrong xData shape. Expect a shape of (%1 x 1) or (%1 x %2)").arg(dataAxisShapeDObj).arg(curveAxisShapeDObj).toLatin1().data());
                }
                else
                {
                    retval += RetVal(retError, 0, tr("unexpected axis state").toLatin1().data());
                }
            }
            else if (curveAxisShapeDObj > curveAxisShapeX && curveAxisShapeX != 1) //todo for other dimension and check if correct
            {
                m_pData->m_axisState = m_pData->m_axisState | ItomQwtPlotEnums::mismatch;

                if (m_pData->m_axisState & ItomQwtPlotEnums::rowState)
                {
                    retval += RetVal(retError, 0, tr("wrong xData shape. Expect a shape of (1 x %2) or (%1 x %2)").arg(curveAxisShapeDObj).arg(dataAxisShapeDObj).toLatin1().data());
                }
                else if (m_pData->m_axisState & ItomQwtPlotEnums::colState)
                {
                    retval += RetVal(retError, 0, tr("wrong xData shape. Expect a shape of (%1 x 1) or (%1 x %2)").arg(dataAxisShapeDObj).arg(curveAxisShapeDObj).toLatin1().data());
                }
                else
                {
                    retval += RetVal(retError, 0, tr("unexpected axis state").toLatin1().data());
                }

            }
            else if (xVec->getType() == ito::tComplex128 || 
                xVec->getType() == ito::tComplex64 || 
                xVec->getType() == ito::tRGBA32 || 
                xVec->getType() == ito::tTimeDelta)
            {
                retval += ito::RetVal(ito::retError, 0, QObject::tr("wrong xData data type. Complex64, complex128, rgba32 and timeDelta are not supported.").toLatin1().data());
                m_pData->m_axisState = m_pData->m_axisState | ItomQwtPlotEnums::mismatch;
            }

            if (curveAxisShapeDObj < curveAxisShapeX || dataAxisShapeDObj < dataAxisShapeX)
            {
                retval += RetVal(retWarning, 0, tr("xData contains more values than the source dataObject. Unused values will be ignored.").toLatin1().data());
                m_pData->m_axisState = m_pData->m_axisState | ItomQwtPlotEnums::noPerfectFit;
            }
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("received a NULL pointer").toLatin1().data());
        m_pData->m_axisState = m_pData->m_axisState | ItomQwtPlotEnums::mismatch;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::keyPressEvent (QKeyEvent * event)
{
    event->ignore();

    if (state() == stateValuePicker)
    {
        Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
        Picker *m;
        int curves = m_plotCurveItems.size();

        event->accept();

        switch(event->key())
        {
        case Qt::Key_Left:
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                m = &(m_pickers[i]);
                if (m->active)
                {
                    stickPickerToXPx(m, m->item->xValue(), -1, m->item->yValue());
                }
            }
            break;
        case Qt::Key_Right:
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                m = &(m_pickers[i]);
                if (m->active)
                {
                     stickPickerToXPx(m, m->item->xValue(), 1, m->item->yValue());
                }
            }
            break;
        case Qt::Key_Up:
            //jump to next visible line at the same x-coordinate
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                m = &(m_pickers[i]);
                if (m->active)
                {
                    //find next visible curve
                    bool found = false;
                    for (int j = m->curveIdx + 1; j < m_plotCurveItems.size(); ++j)
                    {
                        if (m_plotCurveItems[j]->isVisible())
                        {
                            m->curveIdx = j;
                            found = true;
                            break;
                        }
                    }

                    if (!found) //try to restart from beginning
                    {
                        for (int j = 0; j < m->curveIdx; ++j)
                        {
                            if (m_plotCurveItems[j]->isVisible())
                            {
                                m->curveIdx = j;
                                found = true;
                                break;
                            }
                        }
                    }

                    if (found)
                    {
                        stickPickerToXPx(m, m->item->xValue(), 0, m->item->yValue());
                    }
                }
            }
            break;
        case Qt::Key_Down:
            //jump to previous visible line at the same x-coordinate
            for (int i = 0; i < m_pickers.size(); i++)
            {
                m = &(m_pickers[i]);
                if (m->active)
                {
                    //find previous visible curve
                    bool found = false;
                    for (int j = m->curveIdx - 1; j >= 0; --j)
                    {
                        if (m_plotCurveItems[j]->isVisible())
                        {
                            m->curveIdx = j;
                            found = true;
                            break;
                        }
                    }

                    if (!found) //try to restart from the end
                    {
                        for (int j = m_plotCurveItems.size() - 1; j > m->curveIdx; --j)
                        {
                            if (m_plotCurveItems[j]->isVisible())
                            {
                                m->curveIdx = j;
                                found = true;
                                break;
                            }
                        }
                    }

                    if (found)
                    {
                        stickPickerToXPx(m, m->item->xValue(), 0, m->item->yValue());
                    }
                }
            }
            break;
        case Qt::Key_Delete:
        {
            QList<Picker>::iterator it = m_pickers.begin();
            bool deletedAtLeastOne = false;

            while (it != m_pickers.end())
            {
                if (it->active)
                {
                    it->item->detach();
                    delete it->item;
                    deletedAtLeastOne = true;
                    it = m_pickers.erase(it);
					if (((Itom1DQwtPlot*)(this->parent()))->pickerWidget())
					{
						(((Itom1DQwtPlot*)(this->parent()))->pickerWidget())->removePickers();
					}
                }
                else
                {
                    ++it;
                }
            }

            //active the first picker
            if (deletedAtLeastOne && m_pickers.size() > 0)
            {
                m_pickers[0].active = true;
            }

            break;
        }
        default:
            event->ignore();
            break;
        }

        if (event->isAccepted())
        {
            updatePickerPosition(false,false);
            replot();
        }
    }

    if (!event->isAccepted())
    {
        ItomQwtPlot::keyPressEvent(event);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mousePressEvent (QMouseEvent * event)
{
    if (state() == stateValuePicker)
    {
        event->accept();
        int xPx = m_pValuePicker->trackerPosition().x();
        int yPx = m_pValuePicker->trackerPosition().y();
        
        //double yScale = invTransform(yLeft, yPx);
        bool closeToPicker = false;

        if (event->button() == Qt::LeftButton)
        {
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                if (!closeToPicker && abs(transform(xBottom, m_pickers[i].item->xValue()) - xPx) < 20 && abs(transform(yLeft, m_pickers[i].item->yValue()) - yPx) < 20)
                {
                    closeToPicker = true;
                    m_pickers[i].active = true;
                    
                }
                else if ((event->modifiers() & Qt::ControlModifier) == false && m_pickers[i].active)
                {
                    m_pickers[i].active = false;
                    //m_pickers[i].item->setSymbol(new QwtSymbol(QwtSymbol::Diamond, m_pickers[i].color, QPen(m_pickers[i].color,1), QSize(6,6)));
                }
            }

            if (!closeToPicker && m_plotCurveItems.size() > 0 && m_pickers.size() < m_pData->m_pickerLimit)
            {
                Picker picker;
                picker.item = new ItomPlotMarker(m_pData->m_pickerLabelVisible, 
                                                 m_pData->m_pickerType, 
                                                 m_pData->m_pickerLabelAlignment, 
                                                 m_pData->m_pickerLabelOrientation);
                picker.item->attach(this);
                picker.active = true;
                //marker.color = Qt::darkGreen;
                //marker.item->setSymbol(new QwtSymbol(QwtSymbol::Diamond,QBrush(Qt::white), QPen(marker.color,1),  QSize(8,8)));

                //check which curve is the closest to the cursor:
                DataObjectSeriesData *data = NULL;
                int sampleIdx;
                int bestCurveIdx = 0;
                double bestDist = std::numeric_limits<double>::max();
                QPointF currentDist;
                QPointF pt;
                QPointF mouseCoords(xPx, yPx);
                double xScale_;
                double yScale;
                double bestXScale = invTransform(QwtPlot::xBottom, xPx);
                double bestYScale = invTransform(QwtPlot::xBottom, yPx);
                for (int i = 0; i < m_plotCurveItems.size(); ++i)
                {
                    DataObjectSeriesData *data = (DataObjectSeriesData*)(m_plotCurveItems[i]->data());

                    //check for the closest point in a +-5px region
                    for (int xPx_ = xPx - 5; xPx_ <= xPx + 5; ++xPx_)
                    {
                        xScale_ = invTransform(QwtPlot::xBottom, xPx_);
                        yScale = invTransform(QwtPlot::yLeft, yPx);
                        sampleIdx = qBound(0, data->getPosToPix(xScale_, yScale), (int)data->size() - 1);
                        pt = data->sample(sampleIdx);
                        currentDist = QPointF(transform(xBottom, pt.rx()), transform(yLeft, pt.ry())) - mouseCoords;
                        if (currentDist.manhattanLength() < bestDist)
                        {
                            bestDist = currentDist.manhattanLength();
                            bestCurveIdx = i;
                            bestXScale = xScale_;
                            bestYScale = yScale;
                        }
                    }
                }
                
                picker.curveIdx = bestCurveIdx;
                stickPickerToXPx(&picker, bestXScale, 0, bestYScale);

                picker.item->setVisible(true);
                
                m_pickers.append(picker);
            }

            updatePickerPosition(false,false);

            replot();
        }
    }
    else
    {
        event->ignore();
    }

    if (!event->isAccepted())
    {
        ItomQwtPlot::mousePressEvent(event);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mouseMoveEvent (QMouseEvent * event)
{
    if (state() == stateValuePicker)
    {
        event->accept();
        int xPx = m_pValuePicker->trackerPosition().x();
        int yPx = m_pValuePicker->trackerPosition().y();
        double xScale = invTransform(xBottom, xPx);
        double yScale = invTransform(yLeft, yPx);

        if (event->buttons() & Qt::LeftButton)
        {
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                if (m_pickers[i].active == true)
                {
                    stickPickerToXPx(&m_pickers[i], xScale, 0, yScale);
                }
            }
            updatePickerPosition(false,false);

            replot();
        } 
    }
    else
    {
        event->ignore();
    }

    if (!event->isAccepted())
    {
        ItomQwtPlot::mouseMoveEvent(event);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mouseReleaseEvent (QMouseEvent * event)
{
    ItomQwtPlot::mouseReleaseEvent(event);
    return;

    if (state() == stateValuePicker)
    {
        event->accept();
        Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
        int xPx = m_pValuePicker->trackerPosition().x();
        int yPx = m_pValuePicker->trackerPosition().y();
        double xScale = invTransform(xBottom, xPx);
        double yScale = invTransform(yLeft, yPx);

        if (event->button() == Qt::LeftButton)
        {
            for (int i = 0 ; i < m_pickers.size() ; i++)
            {
                if (m_pickers[i].active == true)
                {
                    stickPickerToXPx(&m_pickers[i], xScale, 0, yScale);
                }
            }

            updatePickerPosition(false,false);

            replot();
        }
    }
    else
    {
        event->ignore();
    }

    if (!event->isAccepted())
    {
        ItomQwtPlot::mouseReleaseEvent(event);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setMainPickersToIndex(int idx1, int idx2, int curveIdx)
{
    while (m_pickers.size() < 2)
    {
        //prepend
        Picker picker;
        picker.item = new ItomPlotMarker(m_pData->m_pickerLabelVisible, 
                                                 m_pData->m_pickerType, 
                                                 m_pData->m_pickerLabelAlignment, 
                                                 m_pData->m_pickerLabelOrientation);
        picker.item->attach(this);
        picker.active = false;
                
        picker.curveIdx = curveIdx;
        picker.item->setVisible(true);
                
        m_pickers.prepend(picker);
    }

    for (int i = 0; i < m_pickers.size() ; ++i)
    {
        if (i == 0)
        {
            m_pickers[0].active = true;
            m_pickers[0].curveIdx = curveIdx;
            stickPickerToSampleIdx(&(m_pickers[0]), idx1, 0);
        }
        else if (i == 1)
        {
            m_pickers[1].active = false;
            m_pickers[1].curveIdx = curveIdx;
            stickPickerToSampleIdx(&(m_pickers[1]), idx2, 0);
        }
        else
        {
            m_pickers[i].active = false;
        }
    }

    updatePickerPosition(false,false);

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::stickPickerToXPx(Picker* picker, double xScaleStart, int dir, const double& yScaleStart) //dir: 0: this point, -1: next valid to the left or this if not possible, 1: next valid to the right or this if not possible
{
    if (m_plotCurveItems.size() <= picker->curveIdx)
    {
        return;
    }

    const DataObjectSeriesData *data = (const DataObjectSeriesData*)(m_plotCurveItems[picker->curveIdx]->data());

    if (!qIsFinite(xScaleStart))
    {
        xScaleStart = picker->item->xValue();
    }

    int thisIdx = data->getPosToPix(xScaleStart, yScaleStart, picker->dObjDataIdx); //yScaleStart is ignored in case of DataObjectSeriesData
    int s = (int)data->size();
    QPointF p;
    bool found = false;
    bool d = true;

    if (dir == 0)
    {
        if (thisIdx < 0)
        {
            thisIdx = 0;
        }
        else if (thisIdx >= s)
        {
            thisIdx = s-1;
        }

        while (!found)
        {
            if (thisIdx >= 0 && thisIdx < s)
            {
                p = data->sample(thisIdx);
                if (qIsFinite(p.ry()))
                {
                    picker->item->setXValue(p.rx());
                    picker->item->setYValue(p.ry());
                    picker->dObjDataIdx = thisIdx;
                    found = true;
                }
            }
            else
            {
                break;
            }

            if (d) //iteratively search for the next valid point at the left or right of thisIdx
            {
                thisIdx = -thisIdx + 1;
                d = !d;
            }
            else
            {
                thisIdx = -thisIdx;
                d = !d;
            }
        }
    }
    else if (dir == -1)
    {
        if (thisIdx <= 0)
        {
            thisIdx = 1; //1 since it is decremented to 0 afterwards
        }
        else if (thisIdx > s)
        {
            thisIdx = s;
        }

        while (!found)
        {
            thisIdx -= 1;
            if (thisIdx >= 0)
            {
                p = data->sample(thisIdx);
                if (qIsFinite(p.ry()))
                {
                    if (std::abs(p.rx() - xScaleStart) > std::numeric_limits<double>::epsilon() ||
                        std::abs(p.ry() - yScaleStart) > std::numeric_limits<double>::epsilon())
                    {
                        //the next point is at another position --> take it, else continue in the loop
                        picker->item->setXValue(p.rx());
                        picker->item->setYValue(p.ry());
                        picker->dObjDataIdx = thisIdx;
                        found = true;
                    }
                }
            }
            else
            {
                break;
            }
        }
    }
    else //dir > 0
    {
        if (thisIdx <= -1)
        {
            thisIdx = -1; //-1 since it is incremented to 0 afterwards
        }
        else if (thisIdx > (s-2))
        {
            thisIdx = s-2;
        }

        while (!found)
        {
            thisIdx += 1;
            if (thisIdx < s)
            {
                p = data->sample(thisIdx);
                if (qIsFinite(p.ry()))
                {
                    if (std::abs(p.rx() - xScaleStart) > std::numeric_limits<double>::epsilon() ||
                        std::abs(p.ry() - yScaleStart) > std::numeric_limits<double>::epsilon())
                    {
                        //the next point is at another position --> take it, else continue in the loop
                        picker->item->setXValue(p.rx());
                        picker->item->setYValue(p.ry());
                        picker->dObjDataIdx = thisIdx;
                        found = true;
                    }
                }
            }
            else
            {
                break;
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::stickPickerToSampleIdx(Picker *m, int idx, int dir)
{
    DataObjectSeriesData *data = (DataObjectSeriesData*)(m_plotCurveItems[m->curveIdx]->data());

    int thisIdx = idx;
    int s = (int)data->size();
    QPointF p;
    bool found = false;
    bool d = true;

    if (dir == 0)
    {
        while (!found)
        {
            if (thisIdx >= 0 && thisIdx < s)
            {
                p = data->sample(thisIdx);
                if (qIsFinite(p.ry()))
                {
                    m->item->setXValue(p.rx());
                    m->item->setYValue(p.ry());
                    m->dObjDataIdx = thisIdx;
                    found = true;
                }
            }
            else
            {
                break;
            }

            if (d)
            {
                thisIdx = -thisIdx + 1;
                d = !d;
            }
            else
            {
                thisIdx = -thisIdx;
                d = !d;
            }
        }
    }
    else if (dir == -1)
    {
        while (!found)
        {
            thisIdx -= 1;
            if (thisIdx >= 0)
            {
                p = data->sample(thisIdx);
                if (qIsFinite(p.ry()))
                {
                    m->item->setXValue(p.rx());
                    m->item->setYValue(p.ry());
                    m->dObjDataIdx = thisIdx;
                    found = true;
                }
            }
            else
            {
                break;
            }
        }
    }
    else //dir > 0
    {
        while (!found)
        {
            thisIdx += 1;
            if (thisIdx >= 0 && thisIdx < s)
            {
                p = data->sample(thisIdx);
                if (qIsFinite(p.ry()))
                {
                    m->item->setXValue(p.rx());
                    m->item->setYValue(p.ry());
                    m->dObjDataIdx = thisIdx;
                    found = true;
                }
            }
            else
            {
                break;
            }
        }
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::updateInterval(const Qt::Axis axis, const InternalData &data)
{
    bool recalculateBoundaries = false;
    switch(axis)
    {
        case Qt::YAxis:
            if (data.m_valueScaleAuto)
            {
                m_pData->m_valueScaleAuto = true;
                m_pData->m_valueMin = qQNaN();
                recalculateBoundaries = true;
            }
            else
            {
                m_pData->m_valueScaleAuto = false;
                m_pData->m_valueMin = data.m_valueMin;
                m_pData->m_valueMax = data.m_valueMax;
            }
        break;
        case Qt::XAxis:
            if (data.m_axisScaleAuto) 
            {
                m_pData->m_axisScaleAuto = true;
                m_pData->m_axisMin = qQNaN();
                recalculateBoundaries = true;
            }
            else
            {
                m_pData->m_axisScaleAuto = false;
                m_pData->m_axisMin = data.m_axisMin;
                m_pData->m_axisMax = data.m_axisMax;                        
            }
        break;
    }
    if (m_hash.size() != 0)
    {
        updateScaleValues(recalculateBoundaries); //replot is done here
    }
    else
    {
        updateScaleValues(recalculateBoundaries, true, true);
    }

    return retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setZoomerEnable(const bool checked)
{
    setState(stateZoom);
    //if (checked)
    //{
    //    setPickerEnable(false);
    //    setPannerEnable(false);

    //    m_pData->m_state = stateZoomer;

    //    panner()->setEnabled(false);

    //    DataObjectSeriesData *data = NULL;

    //    foreach(QwtPlotCurve *curve, m_plotCurveItems)
    //    {
    //        data = (DataObjectSeriesData *)curve->data();
    //        zoomer()->setZoomBase(data->boundingRect());
    //    }

    //    zoomer()->setEnabled(true);
    //    canvas()->setCursor(Qt::CrossCursor);
    //}
    //else
    //{
    //    m_pData->m_state = stateIdle;

    //    zoomer()->setEnabled(false);
    //    canvas()->setCursor(Qt::ArrowCursor);

    //    /*foreach(QwtPlotCurve *curve, m_plotCurveItems)
    //    {
    //        setAxisAutoScale(curve->xAxis(),true);
    //        setAxisAutoScale(curve->yAxis(),true);
    //    }*/
    //}
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setPickerEnable(const bool checked)
{
    setState(stateValuePicker);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setPannerEnable(const bool checked)
{
    setState(statePan);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setPickerLimit(int limit)
{
    if (m_pData) m_pData->m_pickerLimit = std::max(0, limit);

    m_pActPicker->setEnabled(state() != stateDrawShape && (m_pData->m_pickerLimit > 0));

    if (m_pickers.size() > m_pData->m_pickerLimit)
    {
        //remove pickers exceeding the new limit
        while (m_pickers.size() > m_pData->m_pickerLimit)
        {
            m_pickers.last().item->detach();
            delete m_pickers.last().item;
            m_pickers.pop_back();
        }

        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::synchronizeCurrentScaleValues()
{
    QwtScaleDiv div = axisScaleDiv(QwtPlot::yLeft);
    m_pData->m_valueMin = div.interval().minValue();
    m_pData->m_valueMax = div.interval().maxValue();
    
    div = axisScaleDiv(QwtPlot::xBottom);
    m_pData->m_axisMin = div.interval().minValue();
    m_pData->m_axisMax = div.interval().maxValue();
}

//----------------------------------------------------------------------------------------------------------------------------------
/*
@param doReplot forces a replot of the content
@param doZoomBase if true, the x/y-zoom is reverted to the full x-y-area of the manually set ranges (the same holds for the value range)
*/
void Plot1DWidget::updateScaleValues(bool doReplot /*= true*/, bool doZoomBase /*= true*/, bool clearStack /*= false*/)
{
    QStack<QRectF> prevStack = zoomer()->zoomStack();
    QRectF initialRect = zoomer()->zoomRect();
    int initialIdx = zoomer()->zoomRectIndex();
    if ((m_pData->m_valueScaleAuto && qIsNaN(m_pData->m_valueMin)) || 
        (m_pData->m_axisScaleAuto && qIsNaN(m_pData->m_axisMin)))
    {
        QRectF rect;
        DataObjectSeriesData *curveData = nullptr;
        
        foreach(QwtPlotCurve *curve, m_plotCurveItems)
        {
            curveData = (DataObjectSeriesData *)curve->data();

            if (curveData)
            {
                QRectF tmpRect = curveData->boundingRect();
                if (qIsFinite(tmpRect.height()))
                {
                    rect = rect.united(tmpRect);
                }
            }
        }

        if (m_pData->m_valueScaleAuto && qIsNaN(m_pData->m_valueMin))
        {
            m_pData->m_valueMin = rect.top();
            m_pData->m_valueMax = rect.bottom();
        }

        if (m_pData->m_axisScaleAuto && qIsNaN(m_pData->m_axisMin))
        {
            m_pData->m_axisMin = rect.left();
            m_pData->m_axisMax = rect.right();
        }

        if (initialIdx == 0)// if the plot wasn't zoomed we need to adjust the window to the current bounds
        {
            zoomer()->zoom(QRectF(m_pData->m_axisMin, m_pData->m_valueMin, (m_pData->m_axisMax - m_pData->m_axisMin), (m_pData->m_valueMax - m_pData->m_valueMin)));
            zoomer()->rescale(false);

        }
    }

    if (doZoomBase)
    {
        QRectF zoom(m_pData->m_axisMin, m_pData->m_valueMin, (m_pData->m_axisMax - m_pData->m_axisMin), (m_pData->m_valueMax - m_pData->m_valueMin));
        zoom = zoom.normalized();
        if (zoomer()->isActive())
        {
            if (initialIdx == 0)
            {
                zoomer()->zoom(0);
            }

           /* if (zoom == zoomer()->zoomRect())
            {
                zoomer()->zoom(zoom);
                zoomer()->rescale(false); //zoom of zoomer does not call rescale in this case, therefore we do it here
            }
            else
            {
                zoomer()->zoom(zoom);
            }*/
        }
        else
        {
            setAxisScale(QwtPlot::xBottom, m_pData->m_axisMin, m_pData->m_axisMax);
            setAxisScale(QwtPlot::yLeft, m_pData->m_valueMin, m_pData->m_valueMax);

            updateGeometry(); //if the interval changes, the tick positions... change as well. Therefore, the sizeHint() might change, too, possibly allowing a smaller plot window

            QStack<QRectF> stack = zoomer()->zoomStack();
            if ((initialIdx >= 0) && (initialIdx < stack.size()))
            {
                stack[initialIdx] = zoom;
            }
            zoomer()->setZoomStack(stack, initialIdx);

            zoomer()->rescale(false);
            /*zoomer()->setZoomBase(base); //set the new base
            int idx = zoomer()->zoomRectIndex();
            int i = 0;
            if (zoom == zoomer()->zoomRect())
            {
                zoomer()->zoom(zoom);
                zoomer()->rescale(false); //zoom of zoomer does not call rescale in this case, therefore we do it here
            }
            else
            {
                zoomer()->appendZoomStack(zoom);
            }*/
        }
    }
    if (clearStack)//set zoom to base and clear history
    {
        
        QStack<QRectF> stack;
        stack.push((zoomer()->zoomBase()));
        zoomer()->setZoomStack(stack);

    }
    if (doReplot)
    {
        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
/* if the unit is a SI length unit and if the distance d is below or above a certain range,
try to adapt d and the unit to a more readable representation. Returns the distance and unit as string.*/
QString Plot1DWidget::distanceWithUnit(qreal d, const QString &unit, bool isIntegerType) const
{
    if (unit == "")
    {
        if (isIntegerType)
        {
            return QString::number((int)d);
        }
        else
        {
            return QString("%1").arg(d, 0, 'f', 3);
        }
    }
    else if (siLengthUnits.contains(unit))
    {
        int index = siLengthUnits.indexOf(unit);
        qreal dTemp = std::abs(d);
        int sign = (d >= 0) ? 1 : -1;
        int maxIdx = siLengthUnits.size() - 1;

        while (true)
        {
            if (dTemp < 1e-12)
            {
                break;
            }
            else if (dTemp > 10000 && index < maxIdx)
            {
                dTemp /= 1000.0;
                index++;
                isIntegerType = false;
            }
            else if (dTemp < 0.1 && index > 0)
            {
                dTemp *= 1000.0;
                index--;
            }
            else
            {
                break;
            }
        }

        if (isIntegerType)
        {
            return QString("%1 %2").arg((int)(dTemp * sign)).arg(siLengthUnits[index]);
        }
        else
        {
            return QString("%1 %2").arg(dTemp * sign, 0, 'f', 2).arg(siLengthUnits[index]);
        }
    }
    else if (siTimeUnits.contains(unit))
    {
        int index = siTimeUnits.indexOf(unit);
        qreal dTemp = std::abs(d);
        int sign = (d >= 0) ? 1 : -1;
        int maxIdx = siTimeUnits.size() - 1;

        while (true)
        {
            if (dTemp < 1e-12)
            {
                break;
            }
            else if (dTemp >= 10000 && index < maxIdx)
            {
                dTemp /= 1000.0;
                index++;
                isIntegerType = false;
            }
            else if (dTemp < 0.1 && index > 0)
            {
                dTemp *= 1000.0;
                index--;
            }
            else
            {
                break;
            }
        }

        if (isIntegerType)
        {
            return QString("%1 %2").arg((int)(dTemp * sign)).arg(siTimeUnits[index]);
        }
        else
        {
            return QString("%1 %2").arg(dTemp * sign, 0, 'f', 2).arg(siTimeUnits[index]);
        }
    }
    else
    {
        if (isIntegerType)
        {
            return QString("%1 %2").arg((int)d).arg(unit);
        }
        else
        {
            return QString("%1 %2").arg(d, 0, 'f', 3).arg(unit);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::updatePickerPosition(bool updatePositions, bool clear/* = false*/)
{
    if (clear)
    {
        foreach(Picker m, m_pickers)
        {
            m.item->detach();
            delete m.item;
        }

        m_pickers.clear();

		if (((Itom1DQwtPlot*)(this->parent()))->pickerWidget())
		{
			(((Itom1DQwtPlot*)(this->parent()))->pickerWidget())->removePickers();
		}
    }

    QColor colors[3] = { Qt::red, Qt::darkGreen, Qt::darkGray };
    int cur = 0;
    Picker *m;
    QVector<QPointF> points;
    QVector<const DataObjectSeriesData*> seriesData;

	QVector< int > indices;
    int actIdx = -1;

    for (int i = 0 ; i < m_pickers.size() ; i++)
    {
        m = &(m_pickers[i]);

        if (updatePositions)
        {
            stickPickerToXPx(m, std::numeric_limits<double>::quiet_NaN(), 0);
        }

        seriesData << (DataObjectSeriesData*)(m_plotCurveItems[m->curveIdx]->data());

        if (m->active)
        {
            m_pickers[i].item->setSymbol(new QwtSymbol(QwtSymbol::Diamond, Qt::white, QPen(colors[cur],2), QSize(8,8)));
            actIdx = i;
        }
        else
        {
            m_pickers[i].item->setSymbol(new QwtSymbol(QwtSymbol::Diamond, colors[cur], QPen(colors[cur],2), QSize(6,6)));
        }

        if (cur < 2) 
        { 
            cur++; 
        }

        points << QPointF(m->item->xValue(), m->item->yValue());   
		indices << i;
    }

    Itom1DQwtPlot *plot1d = (Itom1DQwtPlot*)(this->parent());
    
    if (points.size() >= 1)
    {
        QString coords, offsets;

        if (plot1d->pickerWidget())
        {
            if (m_pData->m_hasDateTimeXAxis)
            {
                QVector<QDateTime> xpositions;
                QVector<qreal> ypositions;

                foreach(const auto &pt, points)
                {
                    xpositions << QwtDate::toDateTime(pt.x());
                    ypositions << pt.y();
                }

                (plot1d->pickerWidget())->updatePickers(indices, xpositions, ypositions);
            }
            else
            {
                (plot1d->pickerWidget())->updatePickers(indices, points);
            }
        }

        QStringList coordTexts;
        bool yIntegerType, xIntegerType;
        bool xDateTimeType;
        QString yCoord, xCoord;
        QString yUnit, xUnit;

        for (int i = 0; i < std::min(points.size(), 2); ++i) 
        {
            const DataObjectSeriesData* d = seriesData[i];
            const DataObjectSeriesDataXY* dxy = dynamic_cast<const DataObjectSeriesDataXY*>(d);

            yIntegerType = false;
            xDateTimeType = dxy && dxy->getXDataObject() && dxy->getXDataObject()->getType() == ito::tDateTime;
            xIntegerType = d->getXCoordsWholeNumber();
            const ito::DataObject *yObj = d->getDataObject();

            if (i == 0)
            {
                xUnit = d->getAxisUnit();
            }
            else if (xUnit != d->getAxisUnit())
            {
                xUnit = "";
            }

            if (i == 0)
            {
                yUnit = d->getValueUnit();
            }
            else if (yUnit != d->getValueUnit())
            {
                yUnit = "";
            }

            if (yObj)
            {
                switch (yObj->getType())
                {
                case ito::tUInt8:
                case ito::tInt8:
                case ito::tUInt16:
                case ito::tInt16:
                case ito::tUInt32:
                case ito::tInt32:
                case ito::tRGBA32:
                    yIntegerType = true;
                    break;
                }
            }

            auto floatformat = [](const qreal& value) 
            { 
                if (std::abs(value) < 100000)
                {
                    return QString::number(value, 'f', 3);
                }
                else
                {
                    return QString::number(value, 'g', 5);
                }
            };

            if (yIntegerType)
            {
                yCoord = QString("%1").arg(points[i].ry(), 0, 'f', 0);
            }
            else
            {
                yCoord = floatformat(points[i].y());
            }

            if (xDateTimeType)
            {
                auto dateTime = QwtDate::toDateTime(points[i].rx());

                if (dateTime.time().msec() != 0)
                {
                    xCoord = dateTime.toString(Qt::ISODateWithMs);
                }
                else
                {
                    xCoord = dateTime.toString(Qt::ISODate);
                }
                
            }
            else if (xIntegerType)
            {
                xCoord = QString("%1").arg(points[i].rx(), 0, 'f', 0);
            }
            else
            {
                xCoord = floatformat(points[i].x());
            }

            coordTexts << QString("[%1; %2]").arg(xCoord, yCoord);
        }

        if (points.size() > 1)
        {
            coords = coordTexts.join("\n");

            qreal dx = points[1].rx() - points[0].rx();
            qreal dy = points[1].ry() - points[0].ry();           

            offsets = QString(" width: %1\n height: %2"). \
                arg(distanceWithUnit(dx, xUnit, xIntegerType)). \
                arg(distanceWithUnit(dy, yUnit, yIntegerType));
        }
        else // == 1
        {
            coords = QString("%1\n      ").arg(coordTexts[0]);
        }

        setPickerText(coords, offsets);
    }
    else
    {
        setPickerText("", "");
    }

    

    if (actIdx >= 0)
    {
        //disable key movements for the picker since one picker is active, clicking any key should not move the mouse cursor but the active picker
        if (m_pValuePicker->keyPattern()[QwtEventPattern::KeyLeft].key != Qt::Key_Left)
        {
            m_pValuePicker->setKeyPattern(QwtEventPattern::KeyLeft, 0);
            m_pValuePicker->setKeyPattern(QwtEventPattern::KeyRight, 0);
            m_pValuePicker->setKeyPattern(QwtEventPattern::KeyUp, 0);
            m_pValuePicker->setKeyPattern(QwtEventPattern::KeyDown, 0);
        }

        emit((Itom1DQwtPlot*)(this->parent()))->pickerChanged(actIdx, m_pickers[actIdx].item->xValue(), m_pickers[actIdx].item->yValue(), m_pickers[actIdx].curveIdx);
    }
    else
    {
        if (m_pValuePicker->keyPattern()[QwtEventPattern::KeyLeft].key == 0)
        {
            //enable key movements again since no picker is selected and key movements should now move the mouse cursor again
            m_pValuePicker->setKeyPattern(QwtEventPattern::KeyLeft, Qt::Key_Left);
            m_pValuePicker->setKeyPattern(QwtEventPattern::KeyRight, Qt::Key_Right);
            m_pValuePicker->setKeyPattern(QwtEventPattern::KeyUp, Qt::Key_Up);
            m_pValuePicker->setKeyPattern(QwtEventPattern::KeyDown, Qt::Key_Down);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::stateChanged(int state)
{
    if (m_pValuePicker) m_pValuePicker->setEnabled(state == stateValuePicker);

    m_pActPicker->setEnabled(state != stateDrawShape && (m_pData->m_pickerLimit > 0));
    m_pActPicker->setChecked(state == stateValuePicker);

    switch (state)
    {
        case stateValuePicker:
            canvas()->setCursor(Qt::CrossCursor);
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::home()
{
    QStack<QRectF> currentZoomStack = zoomer()->zoomStack();

    //get total bounding box
    QRectF boundingRect;
    foreach(QwtPlotCurve *curve, m_plotCurveItems)
    {
        boundingRect = boundingRect.united(((DataObjectSeriesData *)curve->data())->boundingRect());
    }

    if (currentZoomStack.empty())
    {
        currentZoomStack.push(boundingRect);
    }
    else
    {
        currentZoomStack.first() = boundingRect;
    }

    zoomer()->setZoomStack(currentZoomStack, 0);
    zoomer()->zoom(0);
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer< ito::DataObject > Plot1DWidget::getPlotPicker() const
{
    if (m_pickers.size() == 0)
    {
        return QSharedPointer< ito::DataObject >(new ito::DataObject());
    }

    ito::DataObject tmp(m_pickers.size(), 4, ito::tFloat32);
    ito::float32 *rowPtr;

    for (int idx = 0; idx < m_pickers.size(); idx++)
    {
        rowPtr = tmp.rowPtr<ito::float32>(0, idx);

        ito::float64 xScaleStart = (m_pickers[idx]).item->xValue();
        ito::float64 yValue = (m_pickers[idx]).item->yValue();
        rowPtr[1] = cv::saturate_cast<ito::float32>(xScaleStart);
        rowPtr[2] = cv::saturate_cast<ito::float32>(yValue);
        rowPtr[3] = m_pickers[idx].curveIdx;

        if ((m_pickers[idx]).curveIdx < 0 || (m_pickers[idx]).curveIdx > m_plotCurveItems.size() - 1)
        {
            rowPtr[0] = std::numeric_limits<ito::float32>::quiet_NaN();
            continue;
        }

        DataObjectSeriesData *data = (DataObjectSeriesData*)(m_plotCurveItems[(m_pickers[idx]).curveIdx]->data());
        rowPtr[0] = cv::saturate_cast<ito::float32>(data->getPosToPix(xScaleStart));
    }

    QSharedPointer< ito::DataObject > exportItem(new ito::DataObject(tmp));

    return exportItem;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::setPicker(const QVector<double> &coords, int curveIndex /*= 0*/, bool physNotPix /*= true*/, bool append /*= false*/)
{
    ito::RetVal retVal;

    if (curveIndex < 0 || curveIndex >= m_plotCurveItems.size())
    {
        retVal += ito::RetVal::format(ito::retError, 0, tr("CurveIndex out of bounds [0,%i]").toLatin1().data(), m_plotCurveItems.size() - 1);
    }

    if (append && coords.size() > (m_pData->m_pickerLimit - m_pickers.size()))
    {
        retVal += ito::RetVal::format(ito::retError, 0, tr("Number of new pickers exceed the given picker limit of %i").toLatin1().data(), m_pData->m_pickerLimit);
    }
    else if (!append && coords.size() > m_pData->m_pickerLimit)
    {
        retVal += ito::RetVal::format(ito::retError, 0, tr("Number of pickers exceed the given picker limit of %i").toLatin1().data(), m_pData->m_pickerLimit);
    }

    if (!retVal.containsError())
    {
        int cnt = std::min(coords.size(), m_pData->m_pickerLimit);
        int coord_px;
        int picker_offset = (append ? m_pickers.size() : 0);

        //remove items, if m_pickers contains more items than desired
        while (m_pickers.size() > cnt + picker_offset)
        {
            clearPicker(m_pickers.size() - 1, false);
        }

        for (int i = 0; i < cnt; i++)
        {
            if ((i + picker_offset) > m_pickers.size() - 1)
            {
                Picker picker;
                picker.item = new ItomPlotMarker(m_pData->m_pickerLabelVisible,
                    m_pData->m_pickerType,
                    m_pData->m_pickerLabelAlignment,
                    m_pData->m_pickerLabelOrientation);
                picker.item->attach(this);
                picker.active = false;
                picker.curveIdx = curveIndex;

                if (physNotPix)
                {
                    stickPickerToXPx(&picker, coords[i], 0);
                }
                else
                {
                    coord_px = qRound(coords[i]);
                    stickPickerToSampleIdx(&picker, coord_px < 0 ? 0 : coord_px, 0);
                }

                picker.item->setVisible(true);

                m_pickers.append(picker);
            }
            else
            {
                if (physNotPix)
                {
                    stickPickerToXPx(&(m_pickers[i + picker_offset]), coords[i], 0);
                }
                else
                {
                    coord_px = qRound(coords[i]);
                    stickPickerToSampleIdx(&(m_pickers[i + picker_offset]), coord_px < 0 ? 0 : coord_px, 0);
                }
            }
        }
    }

    updatePickerPosition(false, false);
    replot();

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::clearPicker(int id /*=-1 (all)*/, bool doReplot /*= true*/)
{
    if (id == -1)
    {
        foreach(Picker m, m_pickers)
        {
            m.item->detach();
            delete m.item;
        }
        m_pickers.clear();
		if (((Itom1DQwtPlot*)(parent()))->pickerWidget())
		{
			(((Itom1DQwtPlot*)(parent()))->pickerWidget())->removePickers();
		}
    }
    else if (id < 0 || id >= m_pickers.size())
    {
        return ito::RetVal::format(ito::retError, 0, tr("ID out of range [0,%i]").toLatin1().data(), m_pickers.size() - 1);
    }
    else
    {
        m_pickers[id].item->detach();
        delete m_pickers[id].item;
        m_pickers.removeAt(id);
		if (((Itom1DQwtPlot*)(parent()))->pickerWidget())
		{
			(((Itom1DQwtPlot*)(parent()))->pickerWidget())->removePicker(id);
		}
    }

    updatePickerPosition(false, false);
    if (doReplot)
        replot();

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<int> Plot1DWidget::getPickerPixel() const
{
    int ysize = m_pickers.size();

    if (ysize == 0)
    {
        return QVector<int>();
    }

    QVector<int> exportItem(ysize, 0);

    for (int idx = 0; idx < ysize; idx++)
    {

        if ((m_pickers[idx]).curveIdx < 0 || (m_pickers[idx]).curveIdx > m_plotCurveItems.size() - 1)
            continue;

        const DataObjectSeriesData *data = (const DataObjectSeriesData*)(m_plotCurveItems[(m_pickers[idx]).curveIdx]->data());

        int thisIdx = data->getPosToPix((m_pickers[idx]).item->xValue(), (m_pickers[idx]).item->yValue(), (m_pickers[idx]).dObjDataIdx);
        exportItem[idx] = thisIdx;
    }

    return exportItem;
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<float> Plot1DWidget::getPickerPhys() const
{
    int ysize = m_pickers.size();

    if (ysize == 0)
    {
        return QVector<ito::float32>();
    }

    QVector<ito::float32> exportItem(ysize, 0);

    for (int idx = 0; idx < ysize; idx++)
    {
        exportItem[idx] = (m_pickers[idx]).item->xValue();
    }

    return exportItem;
}

//----------------------------------------------------------------------------------------------------------------------------------
QList<QwtPlotCurveProperty*> Plot1DWidget::getPlotCurveProperty()
{
	return this->m_plotCurvePropertyItems;
}

//----------------------------------------------------------------------------------------------------------------------------------
QList<QwtPlotCurve*> Plot1DWidget::getplotCurveItems()
{
	return this->m_plotCurveItems;
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtPlotEnums::ComplexType Plot1DWidget::getComplexStyle() const
{
	return m_pComplexStyle;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setComplexStyle(const ItomQwtPlotEnums::ComplexType &type)
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
void Plot1DWidget::legendItemChecked(const QVariant &itemInfo, bool on)
{
    QwtPlotItem *pi = infoToItem(itemInfo);
    if (pi)
    {
        pi->setVisible(on);
        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::updatePickerStyle(void)
{
    if (!m_pData) return;

    foreach (Picker m, m_pickers)
    {
        m.item->setLabelAlignment(m_pData->m_pickerLabelAlignment);
        m.item->setLabelOrientation(m_pData->m_pickerLabelOrientation);
        m.item->setLabelEnabled(m_pData->m_pickerLabelVisible);
        m.item->setPlotType(m_pData->m_pickerType);
    } 
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setQwtLineStyle(const ItomQwtPlotEnums::CurveStyle &style)
{
    if (style != m_qwtCurveStyle)
    {
        m_qwtCurveStyle = style;

        foreach(QwtPlotCurve *dObjCurve, m_plotCurveItems)
        {
            setQwtLineStyle(dObjCurve, style);
        }
        applyLegendFont();
        replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setQwtLineStyle(QwtPlotCurve *dObjCurve, const ItomQwtPlotEnums::CurveStyle &style)
{
    switch (style)
    {
    case ItomQwtPlotEnums::NoCurve:
        dObjCurve->setStyle(QwtPlotCurve::NoCurve);
        break;
    default:
    case ItomQwtPlotEnums::Lines:
        dObjCurve->setOrientation(Qt::Vertical);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Inverted, false);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Fitted, false);
        dObjCurve->setStyle(QwtPlotCurve::Lines);
        break;
    case ItomQwtPlotEnums::FittedLines:
        dObjCurve->setOrientation(Qt::Vertical);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Inverted, false);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Fitted, true);
        dObjCurve->setStyle(QwtPlotCurve::Lines);
        break;
    case ItomQwtPlotEnums::StepsLeft:
        dObjCurve->setOrientation(Qt::Vertical);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Inverted, false);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Fitted, false);
        dObjCurve->setStyle(QwtPlotCurve::Steps);
        break;
    case ItomQwtPlotEnums::StepsRight:
        dObjCurve->setOrientation(Qt::Vertical);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Inverted, true);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Fitted, false);
        dObjCurve->setStyle(QwtPlotCurve::Steps);
        break;
    case ItomQwtPlotEnums::Steps:
        dObjCurve->setOrientation(Qt::Vertical);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Inverted, false);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Fitted, false);
        dObjCurve->setStyle(QwtPlotCurve::UserCurve);
        break;
    case ItomQwtPlotEnums::SticksHorizontal:
        dObjCurve->setOrientation(Qt::Horizontal);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Inverted, false);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Fitted, false);
        dObjCurve->setStyle(QwtPlotCurve::Sticks);
        break;
    case ItomQwtPlotEnums::Sticks:
    case ItomQwtPlotEnums::SticksVertical:
        dObjCurve->setOrientation(Qt::Vertical);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Inverted, false);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Fitted, false);
        dObjCurve->setStyle(QwtPlotCurve::Sticks);
        break;
    case ItomQwtPlotEnums::Dots:
        dObjCurve->setOrientation(Qt::Vertical);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Inverted, false);
        dObjCurve->setCurveAttribute(QwtPlotCurve::Fitted, false);
        dObjCurve->setStyle(QwtPlotCurve::Dots);
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setBaseLine(const qreal &line)
{
    m_baseLine = line;

    foreach(QwtPlotCurve *c, m_plotCurveItems)
    {
        if (c->baseline() != m_baseLine)
        {
            c->setBaseline(m_baseLine);
        }
    }

    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setCurveFilled()
{
    int colorIndex = 0;
    foreach(QwtPlotCurve *c, m_plotCurveItems)
    {
        if (c->baseline() != m_baseLine)
        {
            c->setBaseline(m_baseLine);
        }

        ((QwtPlotCurveDataObject*)c)->setCurveFilled(m_curveFilled);
        if (m_curveFilled != ItomQwtPlotEnums::NoCurveFill)
        {
            if (m_filledColor.isValid())
            {
                ((QwtPlotCurveDataObject*)c)->setBrush(QBrush(m_filledColor));
            }
            else
            {
                QColor fill = m_colorList[colorIndex++];
                colorIndex = colorIndex % m_colorList.size();
                fill.setAlpha(m_fillCurveAlpa);
                ((QwtPlotCurveDataObject*)c)->setBrush(QBrush(fill));
            }
        }
        else
        {
            ((QwtPlotCurveDataObject*)c)->setBrush(Qt::NoBrush);
        }
    }
    applyLegendFont();
    replot();
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> Plot1DWidget::getDisplayed(bool copyDisplayedAsComplex)
{
    ito::DataObject *displayed = NULL;

    if (m_plotCurveItems.size() > 0)
    {
        const DataObjectSeriesData* seriesData = static_cast<const DataObjectSeriesData*>(m_plotCurveItems[0]->data());
        ito::tDataType type = (seriesData->getDataObject() ? (ito::tDataType)seriesData->getDataObject()->getType() : ito::tUInt8);

        foreach (const QwtPlotCurve *curve, m_plotCurveItems)
        {
            const DataObjectSeriesData* temp = static_cast<const DataObjectSeriesData*>(curve->data());
            if (!temp->getDataObject() || temp->getDataObject()->getType() != type)
            {
                return QSharedPointer<ito::DataObject>();
            }
        }
        //for linecuts from a 2d object the data must be copy in a different way, especially in case of a diagonal linecut
        //complex64 will be mapped to float32 and complex128 to float64
        if (copyDisplayedAsComplex == false)
        {
            type = (type == ito::tComplex64) ? ito::tFloat32 : ((type == ito::tComplex128) ? ito::tFloat64 : type);
        }        

        //until now, rgba32 will be mapped to uint8
        type = (type == ito::tRGBA32) ? ito::tUInt8 : type;

        //create data object according to first series data and set its axis scales and axis offsets values accordingly
        QwtInterval ival = axisInterval(QwtPlot::xBottom);
        size_t firstIdx = std::numeric_limits<size_t>::max();
        size_t lastIdx = 0;
        if (ival.minValue() > ival.maxValue())//change boarders to make the interval valid.
        {
            ival.setInterval(ival.maxValue(), ival.minValue());
        }
        //get start and end index of sample within interval
        for (size_t i = 0; i < seriesData->size(); ++i)
        {

            if (ival.contains(seriesData->sample(i).x()))
            {
                firstIdx = std::min(firstIdx, i);
                lastIdx = std::max(lastIdx, i);
            }
        }

        if (firstIdx == lastIdx)
        {
            return QSharedPointer<ito::DataObject>();
        }

        double lengthPhys = seriesData->sample(lastIdx).x() - seriesData->sample(firstIdx).x();
        size_t lengthPx = lastIdx - firstIdx;

        displayed = new ito::DataObject((int)m_plotCurveItems.size(), lengthPx + 1, type);

        std::string descr, unit;
        seriesData->getDObjValueDescriptionAndUnit(descr, unit);
        displayed->setValueUnit(unit);
        displayed->setValueDescription(descr);
        seriesData->getDObjAxisDescriptionAndUnit(descr, unit);
        displayed->setAxisUnit(1, unit);
        displayed->setAxisDescription(1, descr);

        if (lengthPx > 0)
        {
            displayed->setAxisScale(1, lengthPhys / lengthPx);
            displayed->setAxisOffset(1, -(seriesData->sample(firstIdx).x() / lengthPhys) * lengthPx);
        }
        else
        {
            displayed->setAxisScale(1, 1.0);
            displayed->setAxisOffset(1, -seriesData->sample(firstIdx).x());
        }

        //put data in
        for (int i = 0; i < m_plotCurveItems.size(); ++i)
        {
            seriesData = static_cast<const DataObjectSeriesData*>(m_plotCurveItems[i]->data());

            switch (type)
            {
            case ito::tUInt8:
            {
                ito::uint8 *rowPtr = (ito::uint8*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::uint8>(seriesData->sample(n).ry());
                }
            }
            break;
            case ito::tInt8:
            {
                ito::int8 *rowPtr = (ito::int8*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::int8>(seriesData->sample(n).ry());
                }
            }
            break;
            case ito::tUInt16:
            {
                ito::uint16 *rowPtr = (ito::uint16*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::uint16>(seriesData->sample(n).ry());
                }
            }
            break;
            case ito::tInt16:
            {
                ito::int16 *rowPtr = (ito::int16*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::int16>(seriesData->sample(n).ry());
                }
            }
            break;
            case ito::tUInt32:
            {
                ito::uint32 *rowPtr = (ito::uint32*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::uint32>(seriesData->sample(n).ry());
                }
            }
            break;
            case ito::tInt32:
            {
                ito::int32 *rowPtr = (ito::int32*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::int32>(seriesData->sample(n).ry());
                }
            }
            break;
            case ito::tFloat32:
            {
                ito::float32 *rowPtr = (ito::float32*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::float32>(seriesData->sample(n).ry());
                }
            }
            break;
            case ito::tFloat64:
            {
                ito::float64 *rowPtr = (ito::float64*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::float64>(seriesData->sample(n).ry());
                }
            }
            break;
            case ito::tComplex64:
            {
                ito::complex64 *rowPtr = (ito::complex64*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::complex64>(seriesData->sampleComplex<ito::complex64>(n));
                }
            }
            break;
            case ito::tComplex128:
            {
                ito::complex128 *rowPtr = (ito::complex128*)displayed->rowPtr(0, i);
                for (size_t n = firstIdx; n <= lastIdx; ++n)
                {
                    *(rowPtr++) = cv::saturate_cast<ito::complex128>(seriesData->sampleComplex<ito::complex128>(n));
                }
            }
            break;
            }
        }
    }

    return QSharedPointer<ito::DataObject>(displayed);
}

//----------------------------------------------------------------------------------------------------------------------------------   
void Plot1DWidget::setPickerText(const QString &coords, const QString &offsets)
{
    m_pLblMarkerCoords->setText(coords != "" ? coords : "    \n    ");
    m_pLblMarkerOffsets->setText(offsets != "" ? offsets : "    \n    ");
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setSendCurrentViewState(bool state)
{
    m_pActSendCurrentToWorkspace->setEnabled(state);
}
//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::enableObjectGUIElements(const int mode)
{
    if (mode != m_guiModeCache)
    {
        switch (mode & 0x0F)
        {
        case 0: // Standard
            m_pActCmplxSwitch->setVisible(false);
            m_pActRGBSwitch->setVisible(false);
            m_pMnuMultiRowSwitch->actions()[3]->setEnabled(true); //multi-row
            m_pMnuMultiRowSwitch->actions()[4]->setEnabled(true); //multi-column
            if ((mode & 0xF0) == 0x10)
            {
                m_pMnuMultiRowSwitch->actions()[5]->setEnabled(true); //multi-layer
            }
            else
            {
                m_pMnuMultiRowSwitch->actions()[5]->setEnabled(false); //multi-layer
            }

            break;
        case 1: // RGB
            m_pActCmplxSwitch->setVisible(false);
            m_pActRGBSwitch->setVisible(true);
            m_pMnuMultiRowSwitch->actions()[3]->setEnabled(false); //multi-row
            m_pMnuMultiRowSwitch->actions()[4]->setEnabled(false); //multi-column
            m_pMnuMultiRowSwitch->actions()[5]->setEnabled(false); //multi-layer

            break;
        case 2: // Complex
            m_pActCmplxSwitch->setVisible(true);
            m_pActRGBSwitch->setVisible(false);
            m_pMnuMultiRowSwitch->actions()[3]->setEnabled(true); //multi-row
            m_pMnuMultiRowSwitch->actions()[4]->setEnabled(true); //multi-column

            if ((mode & 0xF0) == 0x10)
            {
                m_pMnuMultiRowSwitch->actions()[5]->setEnabled(true); //multi-layer
            }
            else
            {
                m_pMnuMultiRowSwitch->actions()[5]->setEnabled(false); //multi-layer
            }
            break;
        }
    }

    m_guiModeCache = mode;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setRowPresentation(const ItomQwtPlotEnums::MultiLineMode idx)
{
    bool ok;
    foreach(QAction *a, m_pMnuMultiRowSwitch->actions())
    {
        if (a->data().toInt(&ok) == idx && ok)
        {
            m_pActMultiRowSwitch->setIcon(a->icon());
            m_pMnuMultiRowSwitch->setDefaultAction(a);
        }
    }

    switch (idx)
    {
    case ItomQwtPlotEnums::MultiLayerCols:
    case ItomQwtPlotEnums::MultiLayerRows:
        m_pData->m_multiLine = ItomQwtPlotEnums::MultiLayerAuto;
        break;
    default:
        m_pData->m_multiLine = idx;
        break;
    }

    Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
    if (p)
    {
        QVector<QPointF> bounds = p->getBounds();
        refreshPlot(p->getInputParam("source")->getVal<ito::DataObject*>(), bounds, p->getInputParam("xData")->getVal<ito::DataObject*>());

        //if y-axis is set to auto, it is rescaled here with respect to the new limits, else the manual range is kept unchanged.
        updateInterval(Qt::YAxis, *m_pData); //replot is done here 
        p->updatePropertyDock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setRGBPresentation(const ItomQwtPlotEnums::ColorHandling idx)
{
    bool ok;
    foreach(QAction *a, m_pMnuRGBSwitch->actions())
    {
        if (a->data().toInt(&ok) == idx && ok)
        {
            m_pActRGBSwitch->setIcon(a->icon());
            m_pMnuRGBSwitch->setDefaultAction(a);
        }
    }

    m_pData->m_colorLine = idx;

    Itom1DQwtPlot *p = (Itom1DQwtPlot*)(this->parent());
    if (p)
    {
        QVector<QPointF> bounds = p->getBounds();
        refreshPlot(p->getInputParam("source")->getVal<ito::DataObject*>(), bounds);

        //if y-axis is set to auto, it is rescaled here with respect to the new limits, else the manual range is kept unchanged.
        updateInterval(Qt::YAxis, *m_pData); //replot is done here 
        p->updatePropertyDock();
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mnuCmplxSwitch(QAction *action)
{
    DataObjectSeriesData *seriesData;
    m_pMnuCmplxSwitch->setDefaultAction(action);

    setButtonStyle(buttonStyle()); //to change icon of menu
	int idx = action->data().toInt();
    foreach(QwtPlotCurve *data, m_plotCurveItems)
    {
        seriesData = (DataObjectSeriesData*)data->data();
        
        if (seriesData)
        {
            seriesData->setCmplxState((ItomQwtPlotEnums::ComplexType)idx);


            //if pickers are visible, stick them to the new line form
            if (m_pickers.size())
            {
                foreach(Picker m, m_pickers)
                {
                    stickPickerToSampleIdx(&m, m.dObjDataIdx, 0);
                }

                updatePickerPosition(false);
            }
        }
    }
	m_pComplexStyle = (ItomQwtPlotEnums::ComplexType)idx;

    //if y-axis is set to auto, it is rescaled here with respect to the new limits, else the manual range is kept unchanged.
    updateInterval(Qt::YAxis, *m_pData); //replot is done here 
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mnuLegendSwitch(QAction *action)
{
    if (action->data().toInt() == -1)
    {
        setLegendPosition(QwtPlot::RightLegend, false);
    }
    else
    {
        setLegendPosition((QwtPlot::LegendPosition)action->data().toInt(), true);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mnuMultiRowSwitch(QAction *action)
{
    bool ok;
    int idx = action->data().toInt(&ok);

    if (ok)
    {
        setRowPresentation((ItomQwtPlotEnums::MultiLineMode)idx);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mnuRGBSwitch(QAction *action)
{
    bool ok;
    int idx = action->data().toInt(&ok);

    if (ok)
    {
        setRGBPresentation((ItomQwtPlotEnums::ColorHandling)idx);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mnuPickerClick(bool checked)
{
    if (checked)
    {
        setState(stateValuePicker);
    }
    else if (state() == stateValuePicker)
    {
        setState(stateIdle);
    }

    // two actions have the same meaning, therefore share the checked state.
    m_pActPicker->blockSignals(true);
    m_pActSetPicker->blockSignals(true);

    m_pActPicker->setChecked(checked);
    m_pActSetPicker->setChecked(checked);

    m_pActPicker->blockSignals(false);
    m_pActSetPicker->blockSignals(false);

}


//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mnuScaleSettings()
{
    //update m_data to current values
    synchronizeCurrentScaleValues();

    //if the following two values are not set to false but to the current value of the auto scale values,
    //a re-calc of the the auto limits is only executed, if a switch from manual to automatic boundaries happens.
    bool m_oldAxisScaleAuto = false; //m_pData->m_axisScaleAuto;
    bool m_oldValueScaleAuto = false; //m_pData->m_valueScaleAuto;

    Dialog1DScale *dlg = new Dialog1DScale(*m_pData, qobject_cast<QWidget*>(parent()));
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData((*m_pData));

        if (!m_oldAxisScaleAuto && m_pData->m_axisScaleAuto)
        {
            m_pData->m_axisMin = qQNaN(); //reforce to re-calc the axis min / max boundaries
        }

        if (!m_oldValueScaleAuto && m_pData->m_valueScaleAuto)
        {
            m_pData->m_valueMin = qQNaN(); //reforce to re-calc the value min / max boundaries
        }
        updateScaleValues();
    }

    delete dlg;
    dlg = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mnuGridEnabled(bool checked)
{
    setGridStyle(checked ? Itom1DQwtPlot::GridMajorXY : Itom1DQwtPlot::GridNo);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mnuSetGrid(QAction *action)
{
	Itom1DQwtPlot::GridStyle style = Itom1DQwtPlot::GridNo;

	if (action)
	{
		style = (Itom1DQwtPlot::GridStyle)action->data().toInt();
		if (style != action->data().toInt())
		{
			style = Itom1DQwtPlot::GridNo;
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

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mnuParentScaleSetting()
{
    if (m_plotCurveItems.size() > 0)
    {
        const QwtScaleDiv scale = axisScaleDiv(QwtPlot::yLeft);
        ito::AutoInterval bounds(scale.lowerBound(), scale.upperBound());
        const Itom1DQwtPlot *p = (const Itom1DQwtPlot*)(this->parent());

        foreach(QSharedPointer<ito::Channel> dataChannel, p->getConnectedInputChannels("source"))
        {
            if (dataChannel && dataChannel->getSender())
            {
                ((ito::AbstractDObjFigure*)(dataChannel->getSender()))->setZAxisInterval(bounds);
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mnuDeletePicker()
{
    // remove all pickers
    clearPicker(-1, true);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mnuSetPickerGlobalMinMax()
{
    setPickerToMinMax(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::mnuSetPickerRoiMinMax()
{
    // this is currently not implemented for x over y plots.
    setPickerToMinMax(false);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setPickerToMinMax(bool global)
{
    //absolute min/max within entire object or within current view
    QList<int> visibleIndices;
    QStringList visibleTitles;
    int index = -1;

    for (int i = 0; i < m_plotCurveItems.size(); ++i)
    {
        if (m_plotCurveItems[i]->isVisible())
        {
            visibleIndices << i;
            visibleTitles << m_plotCurveItems[i]->title().text();
        }
    }

    if (visibleIndices.size() == 1)
    {
        index = visibleIndices[0];
    }
    else if (visibleIndices.size() > 1)
    {
        bool ok;
        QString selectedTitle = QInputDialog::getItem(this, tr("Select a visible curve"), tr("Select the curve the picker values should be referred to"), visibleTitles, 0, false, &ok);
        if (!ok)
        {
            return;
        }

        index = visibleIndices[visibleTitles.indexOf(selectedTitle)];
    }

    if (index >= 0)
    {
        DataObjectSeriesData* seriesData = static_cast<DataObjectSeriesData*>(m_plotCurveItems[index]->data());

        ito::float64 minVal, maxVal;
        int minLoc, maxLoc;
        ito::RetVal retval;

        if (global) //global search
        {
            retval = seriesData->getMinMaxLoc(minVal, maxVal, minLoc, maxLoc);
        }
        else
        {
            retval = seriesData->getMinMaxLocCropped(axisInterval(QwtPlot::xBottom), axisInterval(QwtPlot::yLeft), minVal, maxVal, minLoc, maxLoc);
        }

        if (retval == ito::retOk)
        {
            if (minLoc < maxLoc)
            {
                setMainPickersToIndex(minLoc, maxLoc, index);
            }
            else
            {
                setMainPickersToIndex(maxLoc, minLoc, index);
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Plot1DWidget::setCurveProperty(int index, const QByteArray &property, const QVariant &value)
{
    ito::RetVal retval;
    if (index < 0 || index >= m_plotCurvePropertyItems.size())
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("Index out of bounds [0,%i]").toLatin1().data(), m_plotCurvePropertyItems.size() - 1);
    }
    else if (!m_plotCurvePropertyItems[index])
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("Properties of curve %i are not available.").toLatin1().data(), index);
    }
    else
    {
        QwtPlotCurveProperty *prop = m_plotCurvePropertyItems[index];

        if (ito::ITOM_API_FUNCS)
        {
            retval += apiQObjectPropertyWrite(prop, property.constData(), value);
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, tr("itom API not available.").toLatin1().data());
        }

        if (!retval.containsError())
        {
            applyLegendFont();
            replot();
        }

    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
QVariant Plot1DWidget::getCurveProperty(int index, const QByteArray &property)
{
    if (index < 0 || index >= m_plotCurvePropertyItems.size())
    {
        return QVariant();
    }
    else if (!m_plotCurvePropertyItems[index])
    {
        return QVariant();
    }
    else
    {
        QwtPlotCurveProperty *prop = m_plotCurvePropertyItems[index];

        if (ito::ITOM_API_FUNCS)
        {
            QVariant v;
            apiQObjectPropertyRead(prop, property.data(), v);
            return v;
        }
    }

    return QVariant();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Plot1DWidget::setAntiAliased(bool antiAliased)
{
    if (antiAliased != m_antiAliased)
    {
        foreach(QwtPlotCurve* curve, m_plotCurveItems)
        {
            curve->setRenderHint(QwtPlotItem::RenderAntialiased, antiAliased);
        }

        m_antiAliased = antiAliased;

        replot();
    }
}


/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut für Technische Optik (ITO),
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
#include "itom2dqwtplot.h"

#include "../sharedFiles/userInteractionPlotPicker.h"
#include "../sharedFiles/multiPointPickerMachine.h"

#include "../sharedFiles/dialogExportProperties.h"

#include <qwidgetaction.h>
#include <qfiledialog.h>
#include <qimagewriter.h>
#include <qinputdialog.h>
#include <qmessagebox.h>

#include <qwt_plot_renderer.h>
#include <qmenu.h>
#include "dialog2DScale.h"
#include <qwt_text_label.h>
#include <qwt_scale_widget.h>
#include <qwt_picker_machine.h>

#include <qwt_plot_shapeitem.h>

#include <qwt_plot_layout.h>

#include "common/sharedStructuresPrimitives.h"
#include "DataObject/dataObjectFuncs.h"
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::constructor()
{
    // Basic settings
    m_pContent = NULL;
    m_pActSave = NULL;
    m_pActCopyClipboard = NULL;
    m_pActHome = NULL;
    m_pActPan = NULL;
    m_pActZoom = NULL;
    m_pActScaleSettings = NULL;
    m_pActColorPalette = NULL;
    m_pActToggleColorBar = NULL;
    m_pActValuePicker = NULL;
    m_pActLineCut = NULL;
    m_pActStackCut = NULL;
    m_pActPlaneSelector = NULL;
    m_pActCmplxSwitch = NULL;
    m_pActClearDrawings = NULL;
    m_mnuCmplxSwitch = NULL;
    m_pActCoordinates = NULL;
    m_pCoordinates = NULL;
    m_pActDrawMode = NULL;
    m_pMnuDrawMode = NULL;
    m_pActCntrMarker = NULL;
    m_pActAspectRatio = NULL;
    m_pDrawModeActGroup = NULL;
    m_pActDrawModifyMode = NULL;
    m_pMnuDrawModifyMode = NULL;
    m_pDrawModifyModeActGroup = NULL;
    m_pOverlaySlider = NULL;
    m_pActOverlaySlider = NULL;

    //bounds and zCutPoint are two different output connections, since it is possible to have a line cut and a z-stack cut visible at the same time.
    m_pOutput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for line plots from 2d objects").toLatin1().data()));
    m_pOutput.insert("zCutPoint", new ito::Param("zCutPoint", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for z-stack cut in 3d objects").toLatin1().data()));
    m_pOutput.insert("sourceout", new ito::Param("sourceout", ito::ParamBase::DObjPtr, NULL, QObject::tr("shallow copy of input source object").toLatin1().data()));

    int id = qRegisterMetaType<QSharedPointer<ito::DataObject> >("QSharedPointer<ito::DataObject>");

    //init actions
    createActions();

    InternalData* pData = new InternalData();
    m_pVData = (void*) pData;
    
    //init internal data
    pData->m_dataType = ito::tFloat64;
    pData->m_autoTitle;
    pData->m_autoxAxisLabel = true;
    pData->m_autoyAxisLabel = true;
    pData->m_autoValueLabel = true;
    pData->m_valueScaleAuto = true;
    pData->m_valueMin = -127.0;
    pData->m_valueMax = 128.0;
    pData->m_xaxisScaleAuto = true;
    pData->m_yaxisScaleAuto = true;
    pData->m_xaxisVisible = true;
    pData->m_yaxisVisible = true;
    pData->m_colorBarVisible = false;
    pData->m_cmplxType = Itom2DQwt::Abs;
    pData->m_yaxisFlipped = false;
    pData->m_pConstOutput = &m_pOutput;
    pData->m_state = PlotCanvas::tIdle;
    pData->m_elementsToPick = 0;
    pData->m_pDrawItems.clear();

    //initialize actions
    QToolBar *mainTb = new QToolBar(tr("plotting tools"), this);
    addToolBar(mainTb, "mainToolBar");

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
    mainTb->addAction(m_pActDrawMode);
    mainTb->addAction(m_pActClearDrawings);
    mainTb->addAction(m_pActDrawModifyMode);
    mainTb->addSeparator();
    mainTb->addAction(m_pActPlaneSelector);
    mainTb->addAction(m_pActCmplxSwitch);
    mainTb->addAction(m_pActCoordinates);

    QMenu *menuView = new QMenu(tr("View"), this);
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
    addMenu(menuView); //AbstractFigure takes care of the menu

    QMenu *menuTools = new QMenu(tr("Tools"), this);
    menuTools->addAction(m_pActSave);
    menuTools->addAction(m_pActCopyClipboard);
    menuTools->addAction(m_pActSendCurrentToWorkspace);
    menuTools->addSeparator();
    menuTools->addAction(m_pActValuePicker);
    menuTools->addAction(m_pActCntrMarker);
    menuTools->addAction(m_pActLineCut);
    menuTools->addAction(m_pActStackCut);
    menuTools->addSeparator();
    menuTools->addAction(m_pActDrawMode);
    menuTools->addAction(m_pActClearDrawings);
    menuTools->addAction(m_pActDrawModifyMode);
    addMenu(menuTools); //AbstractFigure takes care of the menu

    QMenu *contextMenu = new QMenu(QObject::tr("plot2D"), this);
    contextMenu->addAction(m_pActSave);
    contextMenu->addAction(m_pActCopyClipboard);
    contextMenu->addAction(m_pActSendCurrentToWorkspace);
    contextMenu->addSeparator();
    contextMenu->addAction(m_pActHome);
    contextMenu->addAction(m_pActPan);
    contextMenu->addAction(m_pActZoom);
    contextMenu->addAction(m_pActAspectRatio);
    contextMenu->addSeparator();
    contextMenu->addAction(m_pActValuePicker);
    contextMenu->addAction(m_pActCntrMarker);
    contextMenu->addSeparator();
    contextMenu->addAction(mainTb->toggleViewAction());

    //initialize canvas
    m_pContent = new PlotCanvas(contextMenu, pData, this);
    connect(m_pContent, SIGNAL(statusBarClear()), (QObject*)statusBar(), SLOT(clearMessage()));
    connect(m_pContent, SIGNAL(statusBarMessage(QString)), (QObject*)statusBar(), SLOT(showMessage(QString)));
    connect(m_pContent, SIGNAL(statusBarMessage(QString,int)), (QObject*)statusBar(), SLOT(showMessage(QString,int)));
    setCentralWidget(m_pContent);

    setPropertyObservedObject(this);
}
//----------------------------------------------------------------------------------------------------------------------------------
Itom2dQwtPlot::Itom2dQwtPlot(QWidget *parent): AbstractDObjFigure("", AbstractFigure::ModeStandaloneInUi, parent)
{
    constructor();
}
//----------------------------------------------------------------------------------------------------------------------------------
Itom2dQwtPlot::Itom2dQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    AbstractDObjFigure(itomSettingsFile, windowMode, parent)
{
    constructor();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom2dQwtPlot::~Itom2dQwtPlot()
{
    m_pVData = NULL;
    if (m_mnuCmplxSwitch != NULL)
    {
        delete m_mnuCmplxSwitch;
        m_mnuCmplxSwitch = NULL;
    }
    m_pContent->deleteLater();
    m_pContent = NULL;

}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::init() 
{ 
    return m_pContent->init(); 
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::createActions()
{
    QAction *a = NULL;

    //m_actSave
    m_pActSave = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("Save..."), this);
    a->setShortcut(QKeySequence::Save);
    a->setObjectName("actSave");
    a->setToolTip(tr("Export current view..."));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActSave()));

    //m_actCopyClipboard
    m_pActCopyClipboard = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/clipboard.png"), tr("Copy to clipboard"), this);
    a->setShortcut(QKeySequence::Copy);
    a->setObjectName("actCopyClipboard");
    a->setToolTip(tr("Copies the current view to the clipboard"));
    connect(a, SIGNAL(triggered()), this, SLOT(copyToClipBoard()));

    //m_actHome
    m_pActHome = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"), tr("Home"), this);
    a->setObjectName("actHome");
    a->setToolTip(tr("Reset original view"));
    a->setShortcut(Qt::CTRL + Qt::Key_0);
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActHome()));

    //m_actPan
    m_pActPan = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), tr("Move"), this);
    a->setObjectName("actPan");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Pan axes with left mouse, zoom with right"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActPan(bool)));

    //m_pActClearDrawings
    m_pActClearDrawings = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/editDelete.png"), tr("Clear markers"), this);
    a->setObjectName("actClearGeometrics");
    a->setCheckable(false);
    a->setChecked(false);
    a->setToolTip(tr("Clear all existing geometric elements"));
    connect(a, SIGNAL(triggered()), this, SLOT(clearGeometricElements()));

    //m_actApectRatio
    m_pActAspectRatio = a = new QAction(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio11.png"), tr("Lock aspect ratio"), this);
    a->setObjectName("actRatio");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Toggle fixed / variable aspect ration between axis x and y"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActRatio(bool)));

    //m_actZoom
    m_pActZoom = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"), tr("Zoom to rectangle"), this);
    a->setObjectName("actZoom");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Zoom to rectangle"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActZoom(bool)));

    //m_pActSendCurrentToWorkspace
    m_pActSendCurrentToWorkspace = a = new QAction(QIcon(":/plugins/icons/sendToPython.png"), tr("Send current view to workspace..."), this);
    a->setObjectName("actSendCurrentToWorkspace");
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActSendCurrentToWorkspace()));

    //m_actScaleSetting
    m_pActScaleSettings = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/autoscal.png"), tr("Scale settings..."), this);
    a->setObjectName("actScaleSetting");
    a->setToolTip(tr("Set the ranges and offsets of this view"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActScaleSettings()));

    //m_actPalette
    m_pActColorPalette = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/colorPalette.png"), tr("Palette"), this);
    a->setObjectName("actColorPalette");
    a->setToolTip(tr("Switch between color palettes"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActColorPalette()));

    //m_actToggleColorBar
    m_pActToggleColorBar = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/colorbar.png"), tr("Show Colorbar"), this);
    a->setCheckable(true);
    a->setObjectName("actShowColorBar");
    a->setToolTip(tr("Toggle visibility of the color bar on right canvas side"));
    connect(a,SIGNAL(toggled(bool)),this,SLOT(mnuActToggleColorBar(bool)));

    //m_actMarker
    m_pActValuePicker = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/marker.png"), tr("Marker"), this);
    a->setObjectName("actValuePicker");
    a->setCheckable(true);
    a->setChecked(false);
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActValuePicker(bool)));

    //m_actLineCut
    m_pActLineCut = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"), tr("Linecut"), this);
    a->setCheckable(true);
    a->setObjectName("actLineCut");
    a->setToolTip(tr("Show a in plane line cut"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActLineCut(bool)));


    m_pActLineCutMode = new QMenu(tr("Linecut Mode"), this);
    m_pActLineCutGroup = new QActionGroup(this);

    a = m_pActLineCutGroup->addAction(tr("min & max"));
    a->setToolTip(tr("line cut through global minimum and maximum value"));
    a->setData(0);
    m_pActLineCutMode->addAction(a);
    a->setCheckable(true);
    //a->setChecked(true); //don't check any linecut mode points at beginning, since the default is the free-draw mode

    a = m_pActLineCutGroup->addAction(tr("- & min"));
    a->setToolTip(tr("horizontal line cut through global minimum value"));
    a->setData(1);
    m_pActLineCutMode->addAction(a);
    a->setCheckable(true);

    a = m_pActLineCutGroup->addAction(tr("- & max"));
    a->setToolTip(tr("horizontal line cut through global maximum value"));
    a->setData(2);
    m_pActLineCutMode->addAction(a);
    a->setCheckable(true);

    a = m_pActLineCutGroup->addAction(tr("| & min"));
    a->setToolTip(tr("vertical line cut through global minimum value"));
    a->setData(3);
    m_pActLineCutMode->addAction(a);
    a->setCheckable(true);

    a = m_pActLineCutGroup->addAction(tr("| & max"));
    a->setToolTip(tr("vertical line cut through global maximum value"));
    a->setData(4);
    m_pActLineCutMode->addAction(a);
    a->setCheckable(true);

    m_pActLineCut->setMenu(m_pActLineCutMode);
    connect(m_pActLineCutGroup, SIGNAL(triggered(QAction*)), this, SLOT(mnuLineCutMode(QAction*)));

    //m_pOverlaySlider
    m_pOverlaySlider = new QSlider(Qt::Horizontal, this);
    m_pOverlaySlider->setMinimum(0);
    m_pOverlaySlider->setMaximum(255);
    m_pOverlaySlider->setValue(0);
    m_pOverlaySlider->setWhatsThis(tr("Control alpha-value of overlay image"));
    m_pOverlaySlider->setToolTip(tr("Set alpha for overlay"));
    
    QWidgetAction *wa = new QWidgetAction(this);
    wa->setDefaultWidget(m_pOverlaySlider);
    m_pActOverlaySlider = wa;
    wa->setObjectName("overlaySlider");
    wa->setVisible(false);
    connect(m_pOverlaySlider, SIGNAL(valueChanged(int)), this, SLOT(mnuOverlaySliderChanged(int)));
    

    //m_actStackCut
    m_pActStackCut = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/zStack.png"), tr("Slice in z-direction"), this);
    a->setObjectName("actStackCut");
    a->setToolTip(tr("Show a slice through z-stack"));
    a->setCheckable(true);
    a->setVisible(false);
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActStackCut(bool)));

    QSpinBox *planeSelector = new QSpinBox(this);
    planeSelector->setMinimum(0);
    planeSelector->setMaximum(0);
    planeSelector->setValue(0);
    planeSelector->setKeyboardTracking(false);
    planeSelector->setToolTip(tr("Select image plane"));
    wa = new QWidgetAction(this);
    wa->setDefaultWidget(planeSelector);
    m_pActPlaneSelector = wa;
    wa->setObjectName("planeSelector");
    wa->setVisible(false);
    connect(planeSelector, SIGNAL(valueChanged(int)), this, SLOT(mnuActPlaneSelector(int)));

    //m_actDrawMode
    m_pActDrawMode = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/point.png"), tr("Switch Draw Mode"), this);
    m_pMnuDrawMode = new QMenu(tr("Draw Mode"), this);

    m_pDrawModeActGroup = new QActionGroup(this);
    a = m_pDrawModeActGroup->addAction(tr("Point"));
    a->setData(PlotCanvas::tPoint);
    m_pMnuDrawMode->addAction(a);
    a->setCheckable(true);
    a->setChecked(true);

    a = m_pDrawModeActGroup->addAction(tr("Line"));
    a->setData(PlotCanvas::tLine);
    m_pMnuDrawMode->addAction(a);
    a->setCheckable(true);

    a = m_pDrawModeActGroup->addAction(tr("Rectangle"));
    a->setData(PlotCanvas::tRect);
    m_pMnuDrawMode->addAction(a);
    a->setCheckable(true);

    a = m_pDrawModeActGroup->addAction(tr("Ellipse"));
    a->setData(PlotCanvas::tEllipse);
    m_pMnuDrawMode->addAction(a);
    a->setCheckable(true);

    m_pActDrawMode->setMenu(m_pMnuDrawMode);
    m_pActDrawMode->setVisible(true);
    m_pActDrawMode->setCheckable(true);
    connect(m_pDrawModeActGroup, SIGNAL(triggered(QAction*)), this, SLOT(mnuDrawMode(QAction*)));
    connect(m_pActDrawMode, SIGNAL(triggered(bool)), this, SLOT(mnuDrawMode(bool)));

    m_pActDrawModifyMode = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/geosMove.png"), tr("Switch Element Modification Mode"), this);
    m_pMnuDrawModifyMode = new QMenu(tr("Elemet Modify Mode"), this);

    m_pDrawModifyModeActGroup = new QActionGroup(this);
    a = m_pDrawModifyModeActGroup->addAction(tr("Move elements"));
    a->setData(Itom2DQwt::tMoveGeometricElements);
    a->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/geosMove.png"));
    m_pMnuDrawModifyMode->addAction(a);
    a->setCheckable(false);

    a = m_pDrawModifyModeActGroup->addAction(tr("Resize Elements"));
    a->setData(Itom2DQwt::tRotateGeometricElements);
    a->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/geosResize.png"));
    m_pMnuDrawModifyMode->addAction(a);
    a->setCheckable(false);
    a->setEnabled(false);

    a = m_pDrawModifyModeActGroup->addAction(tr("Rotate Elements"));
    a->setData(Itom2DQwt::tResizeGeometricElements);
    a->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/geosRotate.png"));
    m_pMnuDrawModifyMode->addAction(a);
    a->setCheckable(false);
    a->setEnabled(false);

    a = m_pDrawModifyModeActGroup->addAction(tr("Modify Points"));
    a->setData(Itom2DQwt::tModifyPoints);
    m_pMnuDrawModifyMode->addAction(a);
    a->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/geosPoints.png"));
    a->setCheckable(false);

    m_pActDrawModifyMode->setMenu(m_pMnuDrawModifyMode);
    m_pActDrawModifyMode->setVisible(true);
    m_pActDrawModifyMode->setCheckable(false);
    connect(m_pDrawModifyModeActGroup, SIGNAL(triggered(QAction*)), this, SLOT(mnuDrawModifyMode(QAction*)));

    //m_pActCntrMarker
    m_pActCntrMarker = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/markerCntr.png"), tr("Center marker"), this);
    a->setObjectName("actCenterMarker");
    a->setToolTip(tr("Show a marker at data object center"));
    a->setCheckable(true);
    a->setVisible(true);
    a->setChecked(false);
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActCenterMarker(bool)));
    
    //m_actCmplxSwitch
    m_pActCmplxSwitch = new QAction(QIcon(":/itomDesignerPlugins/complex/icons/ImRe.png"), tr("Switch Imag, Real, Abs, Pha"), this);
    m_mnuCmplxSwitch = new QMenu(tr("Complex Switch"));

    QActionGroup *m_pCmplxActGroup = new QActionGroup(this);
    a = m_pCmplxActGroup->addAction(tr("Real"));
    a->setData(Itom2DQwt::Real);
    m_mnuCmplxSwitch->addAction(a);
    a->setCheckable(true);

    a = m_pCmplxActGroup->addAction(tr("Imag"));
    a->setData(Itom2DQwt::Imag);
    m_mnuCmplxSwitch->addAction(a);
    a->setCheckable(true);

    a = m_pCmplxActGroup->addAction(tr("Abs"));
    a->setData(Itom2DQwt::Abs);
    m_mnuCmplxSwitch->addAction(a);
    a->setCheckable(true);
    a->setChecked(true);

    a = m_pCmplxActGroup->addAction(tr("Pha"));
    a->setData(Itom2DQwt::Phase);
    m_mnuCmplxSwitch->addAction(a);
    a->setCheckable(true);

    m_pActCmplxSwitch->setMenu(m_mnuCmplxSwitch);
    m_pActCmplxSwitch->setVisible(false);
    connect(m_pCmplxActGroup, SIGNAL(triggered(QAction*)), this, SLOT(mnuCmplxSwitch(QAction*)));


    m_pCoordinates = new QLabel("[0.0; 0.0]\n[0.0; 0.0]", this);
    m_pCoordinates->setAlignment(Qt::AlignRight | Qt::AlignTop);
    m_pCoordinates->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    m_pCoordinates->setObjectName("lblCoordinates");

    m_pActCoordinates = new QWidgetAction(this);
    m_pActCoordinates->setDefaultWidget(m_pCoordinates);
    m_pActCoordinates->setVisible(false);

    m_pActProperties = this->getPropertyDockWidget()->toggleViewAction();
    connect(m_pActProperties, SIGNAL(triggered(bool)), this, SLOT(mnuShowProperties(bool)));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::applyUpdate()
{
    //displayed and sourceout is set by dataObjRasterData, since the data is analyzed there
    /*
    if(m_lineCutType & ito::AbstractFigure::tUninitilizedExtern && m_pOutput["bounds"]->getLen() < 2 && m_pInput["source"]->getVal<ito::DataObject*>())
    {
        ito::DataObject* tmp = m_pInput["source"]->getVal<ito::DataObject*>();
        int dims = tmp->getDims();
        double bounds[6] = {0.0, 0.0, 0.0, 1.0, 0.5, 0.5};
        if(dims > 1)
        {
            bounds[2] = tmp->getPixToPhys(dims-1, 0);
            bounds[3] = tmp->getPixToPhys(dims-1, tmp->getSize(dims-1));
            bounds[4] = tmp->getPixToPhys(dims-2, tmp->getSize(dims-2)/2);
            bounds[5] = tmp->getPixToPhys(dims-2, tmp->getSize(dims-2)/2);
        }
        else
        {
        
        }
        m_pOutput["bounds"]->setVal<double*>(bounds, 6);
    }
    */
    m_pContent->refreshPlot(m_pInput["source"]->getVal<ito::DataObject*>());

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::colorBarVisible() const
{
    return m_pActToggleColorBar->isChecked();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setColorBarVisible(bool value)
{
    m_pActToggleColorBar->setChecked(value); //emits toggle signal of action
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getTitle() const
{
    if (!m_pVData || ((InternalData*) m_pVData)->m_autoTitle)
    {
        return "<auto>";
    }
    return ((InternalData*) m_pVData)->m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setTitle(const QString &title)
{
    if(m_pVData == NULL) return;
    if (title == "<auto>")
    {
        ((InternalData*) m_pVData)->m_autoTitle = true;
    }
    else
    {
        ((InternalData*) m_pVData)->m_autoTitle = false;
        ((InternalData*) m_pVData)->m_title = title;
    }

    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetTitle()
{
    if(m_pVData == NULL) return;
    ((InternalData*) m_pVData)->m_autoTitle = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getxAxisLabel() const
{
    if (((InternalData*) m_pVData)->m_autoxAxisLabel)
    {
        return "<auto>";
    }
    return ((InternalData*) m_pVData)->m_xaxisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setxAxisLabel(const QString &label)
{
    if(m_pVData == NULL) return;
    if (label == "<auto>")
    {
        ((InternalData*) m_pVData)->m_autoxAxisLabel = true;
    }
    else
    {
        ((InternalData*) m_pVData)->m_autoxAxisLabel = false;
        ((InternalData*) m_pVData)->m_xaxisLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetxAxisLabel()
{
    if(!m_pVData) return;
    ((InternalData*) m_pVData)->m_autoxAxisLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getyAxisLabel() const
{
    if (((InternalData*) m_pVData)->m_autoyAxisLabel)
    {
        return "<auto>";
    }
    return ((InternalData*) m_pVData)->m_yaxisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisLabel(const QString &label)
{
    if(!m_pVData) return;
    if (label == "<auto>")
    {
        ((InternalData*) m_pVData)->m_autoyAxisLabel = true;
    }
    else
    {
        ((InternalData*) m_pVData)->m_autoyAxisLabel = false;
        ((InternalData*) m_pVData)->m_yaxisLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetyAxisLabel()
{
    if(!m_pVData) return;
    ((InternalData*) m_pVData)->m_autoyAxisLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getValueLabel() const
{
    if (((InternalData*) m_pVData)->m_autoValueLabel)
    {
        return "<auto>";
    }
    return ((InternalData*) m_pVData)->m_valueLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setValueLabel(const QString &label)
{
    if(!m_pVData) return;
    if (label == "<auto>")
    {
        ((InternalData*) m_pVData)->m_autoValueLabel = true;
    }
    else
    {
        ((InternalData*) m_pVData)->m_autoValueLabel = false;
        ((InternalData*) m_pVData)->m_valueLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetValueLabel()
{
    if(!m_pVData) return;
    ((InternalData*) m_pVData)->m_autoValueLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getyAxisFlipped() const
{
    return ((InternalData*) m_pVData)->m_yaxisFlipped;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisFlipped(const bool &value)
{
    if(!m_pVData) return;
    if (((InternalData*) m_pVData)->m_yaxisFlipped != value)
    {
        ((InternalData*) m_pVData)->m_yaxisFlipped = value;
    }
    if (m_pContent)
    {
        m_pContent->updateScaleValues(true, false); //replot, but no change of the current x/y and value zoom ranges
        m_pContent->internalDataUpdated();
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getxAxisVisible() const
{
    return ((InternalData*) m_pVData)->m_xaxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setxAxisVisible(const bool &value)
{
    if(!m_pVData) return;
    ((InternalData*) m_pVData)->m_xaxisVisible = value;

    if (m_pContent) m_pContent->enableAxis(QwtPlot::xBottom, value);
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getyAxisVisible() const
{
    return ((InternalData*) m_pVData)->m_yaxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisVisible(const bool &value)
{
    if(!m_pVData) return;
    ((InternalData*) m_pVData)->m_yaxisVisible = value;

    if (m_pContent) m_pContent->enableAxis(QwtPlot::yLeft, value);
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom2dQwtPlot::getXAxisInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getInterval(Qt::XAxis);
    }
    return ito::AutoInterval();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setXAxisInterval(ito::AutoInterval interval)
{
    if (m_pContent)
    {
        m_pContent->setInterval(Qt::XAxis, interval);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom2dQwtPlot::getYAxisInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getInterval(Qt::YAxis);
    }
    return ito::AutoInterval();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setYAxisInterval(ito::AutoInterval interval)
{
    if (m_pContent)
    {
        m_pContent->setInterval(Qt::YAxis, interval);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom2dQwtPlot::getZAxisInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getInterval(Qt::ZAxis);
    }
    return ito::AutoInterval();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setZAxisInterval(ito::AutoInterval interval)
{
    if (m_pContent)
    {
        m_pContent->setInterval(Qt::ZAxis, interval);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom2dQwtPlot::getOverlayInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getOverlayInterval(Qt::ZAxis);
    }
    return ito::AutoInterval();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setOverlayInterval(ito::AutoInterval interval)
{
    if (m_pContent)
    {
        m_pContent->setOverlayInterval(Qt::ZAxis, interval);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getColorMap() const
{
    if (m_pContent)
    {
        return m_pContent->colorMapName();
    }
    return "";
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setColorMap(const QString &name)
{
    if (name != "" && m_pContent)
    {
        m_pContent->setColorMap(name);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getOverlayColorMap() const
{
    if (m_pContent)
    {
        return m_pContent->colorOverlayMapName();
    }
    return "";
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setOverlayColorMap(const QString &name)
{
    if (name != "" && m_pContent)
    {
        m_pContent->setOverlayColorMap(name);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom2dQwtPlot::getTitleFont(void) const
{
    if (m_pContent)
    {
        return m_pContent->titleLabel()->font();
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setTitleFont(const QFont &font)
{
    if (m_pContent)
    {
        m_pContent->titleLabel()->setFont(font);
        //m_pContent->replot();
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom2dQwtPlot::getLabelFont(void) const
{
    if (m_pContent)
    {
        QwtText t = m_pContent->axisWidget(QwtPlot::xBottom)->title();
        return t.font();
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setLabelFont(const QFont &font)
{
    if (m_pContent)
    {
        QwtText title;
        title = m_pContent->axisWidget(QwtPlot::xBottom)->title();
        title.setFont(font);
        m_pContent->axisWidget(QwtPlot::xBottom)->setTitle(title);

        title = m_pContent->axisWidget(QwtPlot::yLeft)->title();
        title.setFont(font);
        m_pContent->axisWidget(QwtPlot::yLeft)->setTitle(title);

        title = m_pContent->axisWidget(QwtPlot::yRight)->title();
        title.setFont(font);
        m_pContent->axisWidget(QwtPlot::yRight)->setTitle(title);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom2dQwtPlot::getAxisFont(void) const
{
    if (m_pContent)
    {
        return m_pContent->axisFont(QwtPlot::xBottom);
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setAxisFont(const QFont &font)
{
    if (m_pContent)
    {
        m_pContent->setAxisFont(QwtPlot::xBottom, font);
        m_pContent->setAxisFont(QwtPlot::yLeft, font);
        m_pContent->setAxisFont(QwtPlot::yRight, font);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActSave()
{
    //first get the output format information, then the filename (in order to let the user see what can be adjusted before defining a filename)
    bool abort = true;

    QSizeF curSize = m_pContent->size();
    int resolution = 300;

    DialogExportProperties *dlg = new DialogExportProperties("", curSize, this);
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData(curSize, resolution);

        abort = false;
    }

    delete dlg;
    dlg = NULL;

    if(abort)
    {
        return;
    }

    static QString saveDefaultPath;

    #ifndef QT_NO_PRINTER
    QString fileName = "plot2D.pdf";
#else
    QString fileName = "plot2D.png";
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
void Itom2dQwtPlot::mnuActHome()
{
    if (m_pContent) m_pContent->m_pZoomer->zoom(0);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActPan(bool checked)
{
    if (checked)
    {
        m_pActValuePicker->setChecked(false);
        m_pActZoom->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActStackCut->setChecked(false);
        m_pActDrawMode->setChecked(false);
        m_pContent->setState(PlotCanvas::tPan);
    }
    else
    {
        m_pContent->setState(PlotCanvas::tIdle);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActZoom(bool checked)
{
    if (checked)
    {
        m_pActValuePicker->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActStackCut->setChecked(false);
        m_pActDrawMode->setChecked(false);
        m_pContent->setState(PlotCanvas::tZoom);
    }
    else
    {
        m_pContent->setState(PlotCanvas::tIdle);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActScaleSettings()
{
    m_pContent->synchronizeScaleValues();

    Dialog2DScale *dlg = new Dialog2DScale(*((InternalData*)m_pVData), this);
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData(*((InternalData*)m_pVData));

        m_pContent->updateScaleValues();
    }

    delete dlg;
    dlg = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActColorPalette()
{
    if (m_pContent) m_pContent->setColorMap("__next__");
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActToggleColorBar(bool checked)
{
    if (m_pContent) m_pContent->setColorBarVisible(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActValuePicker(bool checked)
{
    if (checked)
    {
        m_pActZoom->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActStackCut->setChecked(false);
        m_pActDrawMode->setChecked(false);
    }

    m_pContent->setState(checked ? PlotCanvas::tValuePicker : PlotCanvas::tIdle);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActLineCut(bool checked)
{
    if (checked)
    {
        m_pActZoom->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActStackCut->setChecked(false);
        m_pActValuePicker->setChecked(false);
        m_pActDrawMode->setChecked(false);
    }

    m_pContent->setState(checked ? PlotCanvas::tLineCut : PlotCanvas::tIdle);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuLineCutMode(QAction *action)
{
    if(!m_pActLineCut->isChecked())
    {
        m_pActLineCut->setChecked(true);
    }

    ito::AutoInterval x = m_pContent->getInterval(Qt::XAxis);
    ito::AutoInterval y = m_pContent->getInterval(Qt::YAxis);

    double min = 0;
    double max = 0;

    double minLoc[3];
    double maxLoc[3];
    m_pContent->getMinMaxPhysLoc(min, minLoc, max, maxLoc);

    switch (action->data().toInt())
    {
        default:
        case 0:
            if( ito::dObjHelper::isFinite(min) && ito::dObjHelper::isFinite(max))
            {
                
                double dy = maxLoc[1] - minLoc[1];
                double dx = maxLoc[2] - minLoc[2];

                if(!ito::dObjHelper::isNotZero(dy) && !ito::dObjHelper::isNotZero(dx))
                {
                    y.setMinimum((y.rmin() + y.rmax()) / 2.0);
                    y.setMaximum(y.rmin());
                }
                else if(fabs(dx) < std::numeric_limits<double>::epsilon() * 100)
                {
                    y.setMinimum(minLoc[1]);
                    y.setMaximum(maxLoc[1]);
                }
                else if(fabs(dy) < std::numeric_limits<double>::epsilon() * 100)
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

                    if(dx / dy > 0)
                    {
                        if(xmin < xbmin)
                        {
                            x.setMinimum(xbmin);
                            y.setMinimum(ymin);
                        }
                        else
                        {
                            x.setMinimum(xmin);
                            y.setMinimum(ybmin);                    
                        }

                        if(xmax > xbmax)
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
                        if(xmin > xbmax)
                        {
                            x.setMinimum(xbmin);
                            y.setMinimum(ymin);
                        }
                        else
                        {
                            x.setMinimum(xmin);
                            y.setMinimum(ybmin);                    
                        }

                        if(xmax < xbmin)
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
            if( ito::dObjHelper::isFinite(min) && ito::dObjHelper::isFinite(max))
            {
                y.setMinimum(minLoc[1]);
                y.setMaximum(minLoc[1]);
            }
        break;
        case 2:
            if( ito::dObjHelper::isFinite(min) && ito::dObjHelper::isFinite(max))
            {
                y.setMinimum(maxLoc[1]);
                y.setMaximum(maxLoc[1]);
            }
        break;

        case 3:
            if( ito::dObjHelper::isFinite(min) && ito::dObjHelper::isFinite(max))
            {
                x.setMinimum(minLoc[2]);
                x.setMaximum(minLoc[2]);
            }
        break;

        case 4:
            if( ito::dObjHelper::isFinite(min) && ito::dObjHelper::isFinite(max))
            {
                x.setMinimum(maxLoc[2]);
                x.setMaximum(maxLoc[2]);
            }
        break;
    }

    setLinePlot(x.rmin(), y.rmin(), x.rmax(), y.rmax());
}



//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActStackCut(bool checked)
{
    if (checked)
    {
        m_pActZoom->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActValuePicker->setChecked(false);
        m_pActDrawMode->setChecked(false);
    }

    m_pContent->setState(checked ? PlotCanvas::tStackCut : PlotCanvas::tIdle);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActPlaneSelector(int plane)
{
    setPlaneIndex(plane);
}

//----------------------------------------------------------------------------------------------------------------------------------
int Itom2dQwtPlot::getPlaneIndex() const
{
    if (m_pContent) return m_pContent->getCurrentPlane();
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setPlaneIndex(const int &index)
{
    int idx = index;
    if (m_pActPlaneSelector)
    {
        QSpinBox *spinBox = qobject_cast<QSpinBox*>(m_pActPlaneSelector->defaultWidget());
        if (spinBox)
        {
            if (index < spinBox->minimum())
            {
                idx = spinBox->minimum();
            }
            else if (index > spinBox->maximum())
            {
                idx = spinBox->maximum();
            }

            spinBox->setValue(idx);
        }
    }

    if (m_pContent) m_pContent->changePlane(idx);
    
    QStringList paramNames;
    
    if(m_pOutput["bounds"]->getLen() == 6)
    {
        paramNames << "bounds"  << "sourceout";
        double * bounds = m_pOutput["bounds"]->getVal<double*>();

        double newBounds[6];

        for(int i = 2; i < 6;i ++)
        {
            newBounds[i] = bounds[i];
        }
        newBounds[0] = m_pContent->getCurrentPlane();
        newBounds[1] = m_pContent->getCurrentPlane();
        m_pOutput["bounds"]->setVal<double*>(newBounds, 6);
    }
    else
    {
        paramNames << "displayed" ;
    }


    updateChannels(paramNames);

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setPlaneRange(int min, int max)
{
    if (m_pActPlaneSelector)
    {
        QSpinBox *spinBox = qobject_cast<QSpinBox*>(m_pActPlaneSelector->defaultWidget());
        if (spinBox)
        {
            int value = spinBox->value();
            value = std::max(min, value);
            value = std::min(max, value);
            spinBox->setMinimum(min);
            spinBox->setMaximum(max);
            spinBox->setValue(value);
        }
        m_pActPlaneSelector->setVisible((max-min) > 0);
        m_pActStackCut->setVisible((max-min) > 0);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuDrawMode(bool checked)
{
    if (checked)
    {
        m_pActZoom->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActStackCut->setChecked(false);
        m_pActValuePicker->setChecked(false);
    }
    // we need to find out which draw mode we should activate here ...
//    m_pContent->setState(checked ? PlotCanvas::tDraw : PlotCanvas::tIdle);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuDrawModifyMode(QAction *action)
{

    switch (action->data().toInt())
    {
        default:
        case Itom2DQwt::tMoveGeometricElements:
            ((InternalData*) m_pVData)->m_modState = Itom2DQwt::tMoveGeometricElements;
            m_pActDrawModifyMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/geosMove.png"));
        break;

        case Itom2DQwt::tResizeGeometricElements:
            ((InternalData*) m_pVData)->m_modState = Itom2DQwt::tResizeGeometricElements;
            m_pActDrawModifyMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/geosResize.png"));
        break;

        case Itom2DQwt::tRotateGeometricElements:
            ((InternalData*) m_pVData)->m_modState = Itom2DQwt::tRotateGeometricElements;
            m_pActDrawModifyMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/geosRotate.png"));
        break;

        case Itom2DQwt::tModifyPoints:
            ((InternalData*) m_pVData)->m_modState = Itom2DQwt::tModifyPoints;
            m_pActDrawModifyMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/geosPoints.png"));
        break;
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuDrawMode(QAction *action)
{
    m_pActZoom->setChecked(false);
    m_pActPan->setChecked(false);
    m_pActLineCut->setChecked(false);
    m_pActStackCut->setChecked(false);
    m_pActValuePicker->setChecked(false);
    m_pActDrawMode->setChecked(true);

    switch (action->data().toInt())
    {
        default:
        case PlotCanvas::tPoint:
            m_pActDrawMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/marker.png"));
            m_pContent->userInteractionStart(PlotCanvas::tPoint, 1, 1);
//            connect(m_pContent->m_pMultiPointPicker, SIGNAL(selected(QVector<QPointF>)), this, SLOT(userInteractionEndPt(QVector<QPointF>)));
        break;

        case PlotCanvas::tLine:
            m_pActDrawMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"));
            m_pContent->userInteractionStart(PlotCanvas::tLine, 1, 2);
//            connect(m_pContent->m_pMultiPointPicker, SIGNAL(selected(QVector<QPointF>)), this, SLOT(userInteractionEndLine(QVector<QPointF>)));
        break;

        case PlotCanvas::tRect:
            m_pActDrawMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/rectangle.png"));
            m_pContent->userInteractionStart(PlotCanvas::tRect, 1, 2);
//            connect(m_pContent->m_pMultiPointPicker, SIGNAL(selected(QRectF)), this, SLOT(userInteractionEndRect(QRectF)));
        break;

        case PlotCanvas::tEllipse:
            m_pActDrawMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/ellipse.png"));
            m_pContent->userInteractionStart(PlotCanvas::tEllipse, 1, 2);
//            connect(m_pContent->m_pMultiPointPicker, SIGNAL(selected(QRectF)), this, SLOT(userInteractionEndEllipse(QRectF)));
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuCmplxSwitch(QAction *action)
{
    setCmplxSwitch((Itom2DQwt::tComplexType)(action->data().toInt()), true);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setCmplxSwitch(/*PlotCanvas::ComplexType*/ int type, bool visible)
{
    m_pActCmplxSwitch->setVisible(visible);
    if(!m_pVData) return;

    if (((InternalData*) m_pVData)->m_cmplxType != type)
    {

        if (visible)
        {
            ((InternalData*) m_pVData)->m_cmplxType = (Itom2DQwt::tComplexType)type;

            switch (type)
            {
                case Itom2DQwt::Imag:
                    m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReImag.png"));
                break;
                case Itom2DQwt::Real:
                    m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReReal.png"));
                break;
                case Itom2DQwt::Phase:
                    m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImRePhase.png"));
                break;
                case Itom2DQwt::Abs:
                    m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReAbs.png"));
                break;
            }
        }

        if (m_pContent) m_pContent->internalDataUpdated();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setColorDataTypeRepresentation(bool colorOn)
{
    if (colorOn)
    {
        if (m_pContent) m_pContent->setColorBarVisible(false);
        m_pActColorPalette->setVisible(false);
        m_pActToggleColorBar->setVisible(false);
    }
    else
    {
        m_pActColorPalette->setVisible(true);
        m_pActToggleColorBar->setVisible(true);
        if (m_pContent) m_pContent->setColorBarVisible(m_pActToggleColorBar->isChecked());
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::displayCut(QVector<QPointF> bounds, ito::uint32 &uniqueID, bool zStack /*= false*/)
{
    if(!ito::ITOM_API_FUNCS_GRAPH)
    {
        return ito::RetVal(ito::retError, 0, tr("Could not spawn lineCut due to missing API-handle").toLatin1().data());
    }
    ito::RetVal retval = ito::retOk;
    QList<QString> paramNames;
    ito::uint32 newUniqueID = uniqueID;
    QWidget *lineCutObj = NULL;

    bool needChannelUpdate = false;

    double *pointArr = new double[2 * bounds.size()];
    for (int np = 0; np < bounds.size(); np++)
    {
        pointArr[np * 2] = bounds[np].x();
        pointArr[np * 2 + 1] = bounds[np].y();
    }

    if (zStack)
    {
        m_pOutput["zCutPoint"]->setVal(pointArr, 2 * bounds.size());
        if(m_zSliceType & ito::AbstractFigure::tUninitilizedExtern)
        {
            needChannelUpdate = true;
            m_zSliceType &= ~ito::AbstractFigure::tUninitilizedExtern;
            m_zSliceType |= ito::AbstractFigure::tExternChild;
        }
    }
    else
    {
        if(m_lineCutType & ito::AbstractFigure::tUninitilizedExtern)
        {
            needChannelUpdate = true;
            m_lineCutType &= ~ito::AbstractFigure::tUninitilizedExtern;
            m_lineCutType |= ito::AbstractFigure::tExternChild;
        }
        m_pOutput["bounds"]->setVal(pointArr, 2 * bounds.size());
    }

    delete[] pointArr;
    //setOutpBounds(bounds);
    //setLinePlotCoordinates(bounds);

    retval += apiGetFigure("DObjStaticLine","", newUniqueID, &lineCutObj, this); //(newUniqueID, "itom1DQwtFigure", &lineCutObj);

    if (!retval.containsError())
    {
        if (uniqueID != newUniqueID || needChannelUpdate)
        {
            uniqueID = newUniqueID;
            ito::AbstractDObjFigure* figure = NULL;
            if (lineCutObj->inherits("ito::AbstractDObjFigure"))
            {
                figure = (ito::AbstractDObjFigure*)lineCutObj;
                if(!needChannelUpdate)
                {
                    m_childFigures[lineCutObj] = newUniqueID;
                    connect(lineCutObj, SIGNAL(destroyed(QObject*)), this, SLOT(childFigureDestroyed(QObject*)));
                }
            }
            else
            {
                return ito::RetVal(ito::retError, 0, tr("the opened figure is not inherited from ito::AbstractDObjFigure").toLatin1().data());
            }

            if(needChannelUpdate)
            {
                ito::Channel *tempChannel;
                foreach(tempChannel, m_pChannels)
                {
                    if (tempChannel->getParent() == (ito::AbstractNode*)this &&  tempChannel->getChild() == (ito::AbstractNode*)figure)
                    {
                        removeChannel(tempChannel);
                    }
                }
            }

            if (zStack)
            {
                ((QMainWindow*)figure)->setWindowTitle(tr("Z-Stack"));
                // for a linecut in z-direction we have to pass the input object to the linecut, otherwise the 1D-widget "sees" only a 2D object
                // with one plane and cannot display the points in z-direction
                retval += addChannel((ito::AbstractNode*)figure, m_pOutput["zCutPoint"], figure->getInputParam("bounds"), ito::Channel::parentToChild, 0, 1);
                retval += addChannel((ito::AbstractNode*)figure,  m_pOutput["sourceout"], figure->getInputParam("source"), ito::Channel::parentToChild, 0, 1);
                paramNames << "zCutPoint"  << "sourceout";
            }
            else if(bounds.size() == 3) // its a 3D-Object
            {
                ((QMainWindow*)figure)->setWindowTitle(tr("Linecut"));
                // otherwise pass the original plane and z0:z1, y0:y1, x0, x1 coordinates
                retval += addChannel((ito::AbstractNode*)figure, m_pOutput["bounds"], figure->getInputParam("bounds"), ito::Channel::parentToChild, 0, 1);
                retval += addChannel((ito::AbstractNode*)figure, m_pOutput["sourceout"], figure->getInputParam("source"), ito::Channel::parentToChild, 0, 1);
                paramNames << "bounds"  << "sourceout";
            }
            else
            {
                ((QMainWindow*)figure)->setWindowTitle(tr("Linecut"));
                // otherwise simply pass on the displayed plane
                retval += addChannel((ito::AbstractNode*)figure, m_pOutput["bounds"], figure->getInputParam("bounds"), ito::Channel::parentToChild, 0, 1);
                retval += addChannel((ito::AbstractNode*)figure, m_pOutput["displayed"], figure->getInputParam("source"), ito::Channel::parentToChild, 0, 1);
                paramNames << "bounds"  << "displayed";
            }

            retval += updateChannels(paramNames);

            if(needChannelUpdate) // we have an updated plot and want to show it
            {
                if (zStack && m_zSliceType & ito::AbstractFigure::tVisibleOnInit)
                {
                    m_zSliceType &= ~ito::AbstractFigure::tVisibleOnInit;
                    figure->setVisible(true);
                }
                else if(!zStack && m_lineCutType & ito::AbstractFigure::tVisibleOnInit)
                {
                    m_lineCutType &= ~ito::AbstractFigure::tVisibleOnInit;
                    figure->setVisible(true);
                }
                // Something to do?
            }
            else// we do not have a plot so we have to show it and its child of this plot
            {
                if (zStack)
                {
                    m_zSliceType = ito::AbstractFigure::tOwnChild;
                    figure->show();
                }
                else
                {
                    m_lineCutType = ito::AbstractFigure::tOwnChild;
                    figure->show();
                }

            }
            
            
        }
        else
        {
            if (zStack)
            {
                paramNames << "zCutPoint"  << "sourceout";
            }
            else if(bounds.size() == 3) // its a 3D-Object
            {
                paramNames << "bounds"  << "sourceout";
            }
            else
            {
                paramNames << "bounds"  << "displayed";
            }
            retval += updateChannels(paramNames);
        }
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::childFigureDestroyed(QObject *obj)
{
    QHash<QObject*,ito::uint32>::iterator it = m_childFigures.find(obj);

    if (it != m_childFigures.end())
    {
        m_pContent->childFigureDestroyed(obj, m_childFigures[obj]);
    }
    else
    {
        m_pContent->childFigureDestroyed(obj, 0);
    }

    m_childFigures.erase(it);
}

////----------------------------------------------------------------------------------------------------------------------------------
//void Itom2dQwtPlot::setLinePlotCoordinates(const QVector<QPointF> pts)
//{
//    char buf[60] = {0};
//    if (pts.size() > 1)
//    {
//        sprintf(buf, "[%.4g; %.4g]\n[%.4g; %.4g]", pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
//    }
//    else if (pts.size() == 1)
//    {
//        sprintf(buf, "[%.4g; %.4g]\n[ - ; - ]", pts[0].x(), pts[0].y());
//    }
//    else
//    {
//        sprintf(buf, "[ - ; - ]\n[ - ; - ]");
//    }
//    m_lblCoordinates->setText(buf);
//}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::plotMarkers(const ito::DataObject &coords, QString style, QString id /*= QString::Null()*/, int plane /*= -1*/)
{
    return m_pContent->plotMarkers(&coords, style, id, plane);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::deleteMarkers(QString id)
{
    return m_pContent->deleteMarkers(id);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::deleteMarkers(int id)
{
    ito::RetVal retVal = m_pContent->deleteMarkers(id);
    if(!retVal.containsWarningOrError()) emit plotItemDeleted(id);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::clearGeometricElements(void)
{
    ito::RetVal retVal = ito::retOk;
    if(!m_pVData) return retVal;
    QList<int> keys = ((InternalData*) m_pVData)->m_pDrawItems.keys();
    

    for(int i = 0; i < keys.size(); i++)
    {
        retVal += m_pContent->deleteMarkers(keys[i]);
    }
    emit plotItemsDeleted();
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::userInteractionStart(int type, bool start, int maxNrOfPoints /*= -1*/)
{
    m_pActValuePicker->setChecked(false);
    m_pActZoom->setChecked(false);
    m_pActPan->setChecked(false);
    m_pActLineCut->setChecked(false);
    m_pActStackCut->setChecked(false);
    m_pActDrawMode->setChecked(false);

    switch (type)
    {
        default:
            m_pContent->userInteractionStart(0, false, 0);
            break;
        case PlotCanvas::tMultiPointPick:
        case PlotCanvas::tPoint:
            //m_pContent->m_pMultiPointPicker->setStateMachine(new MultiPointPickerMachine());
            m_pContent->userInteractionStart(type, start, maxNrOfPoints);
            //m_pContent->m_pMultiPointPicker->setRubberBand(QwtPicker::CrossRubberBand);
        break;

        case PlotCanvas::tLine:
            m_pContent->userInteractionStart(type, start, maxNrOfPoints * 2);
        break;

        case PlotCanvas::tRect:
            m_pContent->userInteractionStart(type, start, maxNrOfPoints * 2);
        break;

        case PlotCanvas::tEllipse:
            m_pContent->userInteractionStart(type, start, maxNrOfPoints * 2);
        break;
    }

    //m_pContent->setWindowState((m_pContent->windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
    //m_pContent->raise(); //for MacOS
    //m_pContent->activateWindow(); //for Windows
    //m_pContent->setFocus();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setCoordinates(const QVector<QPointF> &pts, bool visible)
{
    m_pActCoordinates->setVisible(visible);

    if (visible)
    {
        char buf[60] = {0};
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
QSharedPointer< ito::DataObject > Itom2dQwtPlot::getGeometricElements()
{
    if(m_pVData == NULL)
    {
        return QSharedPointer< ito::DataObject >(new ito::DataObject());
    }    


    int ysize = ((InternalData*) m_pVData)->m_pDrawItems.size();
    int xsize = PRIM_ELEMENTLENGTH;

    if(ysize == 0)
    {
        return QSharedPointer< ito::DataObject >(new ito::DataObject());
    }

    QSharedPointer< ito::DataObject > exportItem(new ito::DataObject(ysize, xsize, ito::tFloat32));

    qvector2DataObject(exportItem.data());

    return exportItem;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::qvector2DataObject(const ito::DataObject *dstObject)
{
    int ysize = dstObject->getSize(0);

    if(ysize == 0 || ysize < ((InternalData*) m_pVData)->m_pDrawItems.size())
    {
        return ito::retError;
    }

    int xsize = dstObject->getSize(1);

    cv::Mat *tarMat = (cv::Mat*)(dstObject->get_mdata()[0]);
    ito::float32* rowPtr = tarMat->ptr<ito::float32>(0);
    memset(rowPtr, 0, sizeof(ito::float32) * xsize * ysize);

    QHash<int, DrawItem*>::Iterator it = ((InternalData*) m_pVData)->m_pDrawItems.begin();

//    for(int y = 0; y < ysize; y++)
//    {
    int y = 0;
    for (; it != ((InternalData*) m_pVData)->m_pDrawItems.end(); it++)
    {
        rowPtr = tarMat->ptr<ito::float32>(y);
        //if(((InternalData*) m_pVData)->m_pDrawItems[y] == NULL)
        if(it.value() == NULL)
        {
            continue;
        }
        //rowPtr[0] = (ito::float32) (((InternalData*) m_pVData)->m_pDrawItems[y]->m_idx);
        rowPtr[0] = (ito::float32) (it.value()->m_idx);
        switch (it.value()->m_type)
        {
            case PlotCanvas::tPoint:
                rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tPoint;
                rowPtr[2] = (ito::float32) (it.value()->x1);
                rowPtr[3] = (ito::float32) (it.value()->y1);
            break;

            case PlotCanvas::tLine:
                rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tLine;
                rowPtr[2] = (ito::float32) (it.value()->x1);
                rowPtr[3] = (ito::float32) (it.value()->y1);
                rowPtr[5] = (ito::float32) (it.value()->x2);
                rowPtr[6] = (ito::float32) (it.value()->y2);
            break;

            case PlotCanvas::tRect:
                rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tRectangle;
                rowPtr[2] = (ito::float32) (it.value()->x1);
                rowPtr[3] = (ito::float32) (it.value()->y1);
                rowPtr[5] = (ito::float32) (it.value()->x2);
                rowPtr[6] = (ito::float32) (it.value()->y2);
            break;

            case PlotCanvas::tEllipse:
                rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tEllipse;
                rowPtr[2] = (((ito::float32)it.value()->x1 + (ito::float32)it.value()->x2) / 2.0);
                rowPtr[3] = (((ito::float32)it.value()->y1 + (ito::float32)it.value()->y2) / 2.0);
                rowPtr[5] = (abs((ito::float32)it.value()->x1 - (ito::float32)it.value()->x2) / 2.0);
                rowPtr[6] = (abs((ito::float32)it.value()->y1 - (ito::float32)it.value()->y2) / 2.0);
            break;
/*
            case PlotCanvas::tCircle:
                rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tCircle;
                rowPtr[2] = (((ito::float32)it.value()->x1 + (ito::float32)it.value()->x2) / 2.0);
                rowPtr[3] = (((ito::float32)it.value()->y1 + (ito::float32)it.value()->y2) / 2.0);
                rowPtr[5] = (abs((ito::float32)it.value()->x1 - (ito::float32)it.value()->x2) / 4.0) + (abs((ito::float32)it.value()->y1 - (ito::float32)it.value()->y2) / 4.0);
            break;

            case PlotCanvas::tSquare:
                rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tSquare;
                rowPtr[2] = (((ito::float32)it.value()->x1 + (ito::float32)it.value()->x2) / 2.0);
                rowPtr[3] = (((ito::float32)it.value()->y1 + (ito::float32)it.value()->y2) / 2.0);
                rowPtr[5] = (abs((ito::float32)it.value()->x1 - (ito::float32)it.value()->x2) / 4.0) + (abs((ito::float32)it.value()->y1 - (ito::float32)it.value()->y2) / 4.0);
            break;
*/
        }
        y++;
    }

    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setGeometricElements(QSharedPointer< ito::DataObject > geometricElements)
{
    if(m_pVData == NULL) return;
    QList<int> keys = ((InternalData*) m_pVData)->m_pDrawItems.keys();
    ito::RetVal retVal = ito::retOk;

    for(int i = 0; i < keys.size(); i++)
    {
        retVal += m_pContent->deleteMarkers(keys[i]);
    }
    emit plotItemsDeleted();

    if(geometricElements.isNull() || 
       geometricElements->getDims() != 2 || 
       (geometricElements->getType() != ito::tFloat32 && geometricElements->getType() != ito::tFloat64) ||
       geometricElements->getSize(1) < PRIM_ELEMENTLENGTH)
    {
        m_pContent->statusBarMessage(tr("Element container did not match criteria, 2 dims, elements x 11, floating point value"), 600 );
        plotItemsFinished(0, true);
        return;
    }

    if(geometricElements->getSize(0) == 0)
    {
        m_pContent->statusBarMessage(tr("Deleted element, new element list was empty"), 600 );
        m_pContent->replot();
        return;
    }

    int ysize = geometricElements->getSize(0);
//    int xsize = PRIM_ELEMENTLENGTH;
    int type = geometricElements->getType();

    int rowStepDst = 8;
    ito::DataObject coords(rowStepDst, ysize, ito::tFloat32);

    ito::float32 *ids = (ito::float32*)coords.rowPtr(0, 0);            
    ito::float32 *types = (ito::float32*)coords.rowPtr(0, 1);
    ito::float32 *xCoords0 = (ito::float32*)coords.rowPtr(0, 2);
    ito::float32 *yCoords0 = (ito::float32*)coords.rowPtr(0, 3);
    ito::float32 *xCoords1 = (ito::float32*)coords.rowPtr(0, 4);
    ito::float32 *yCoords1 = (ito::float32*)coords.rowPtr(0, 5);

    ito::float32* ptrScr32 = NULL;
    ito::float64* ptrScr64 = NULL;
        
    int rowStep = static_cast<int>(((cv::Mat*)(geometricElements->get_mdata()[geometricElements->seekMat(0)]))->step[0]);

    if(type == ito::tFloat64)
    {
        rowStep /= sizeof(ito::float64);
        ptrScr64 = ((cv::Mat*)(geometricElements->get_mdata()[geometricElements->seekMat(0)]))->ptr<ito::float64>(0);
    }
    else
    {
        rowStep /= sizeof(ito::float32);
        ptrScr32 = ((cv::Mat*)(geometricElements->get_mdata()[geometricElements->seekMat(0)]))->ptr<ito::float32>(0);
    }

    ito::float32* ptrCurScr32 = NULL;
    ito::float64* ptrCurScr64 = NULL;

    for(int geoElement = 0; geoElement < ysize; geoElement++)
    {
        int type = 0;

        if(type == ito::tFloat64)
        {
            ptrCurScr64 =  &(ptrScr64[geoElement * rowStep]);
            type = static_cast<ito::int32>(ptrCurScr64[1]) & 0x0000FFFF;
        }
        else
        {
            ptrCurScr32 =  &(ptrScr32[geoElement * rowStep]);
            type = static_cast<ito::int32>(ptrCurScr32[1]) & 0x0000FFFF;
        }

        types[geoElement] = (ito::float32) type;

        switch (type)
        {
            case ito::PrimitiveContainer::tPoint:
            {     
                if(type == ito::tFloat64) // idx, type, x0, y0, z0
                {
                    ids[geoElement]                      = static_cast<ito::float32>(ptrCurScr64[0]);
                    xCoords0[geoElement] = static_cast<ito::float64>(ptrCurScr64[2]);
                    yCoords0[geoElement] = static_cast<ito::float64>(ptrCurScr64[3]);
                }
                else
                {
                    ids[geoElement]                       = ptrCurScr32[0];
                    xCoords0[geoElement] = ptrCurScr32[2];
                    yCoords0[geoElement] = ptrCurScr32[3];
                }

            }
            break;

            case ito::PrimitiveContainer::tLine:
            {
                if(type == ito::tFloat64)   // idx, type, x0, y0, z0, x1, y1, z1
                {
                    ids[geoElement]                      = static_cast<ito::float32>(ptrCurScr64[0]);
                    xCoords0[geoElement] = static_cast<ito::float64>(ptrCurScr64[2]);
                    yCoords0[geoElement] = static_cast<ito::float64>(ptrCurScr64[3]);
                    xCoords1[geoElement] = static_cast<ito::float64>(ptrCurScr64[5]);
                    yCoords1[geoElement] = static_cast<ito::float64>(ptrCurScr64[6]);
                }
                else
                {
                    ids[geoElement]      = ptrCurScr32[0];
                    xCoords0[geoElement] = ptrCurScr32[2];
                    yCoords0[geoElement] = ptrCurScr32[3];
                    xCoords1[geoElement] = ptrCurScr32[5];
                    yCoords1[geoElement] = ptrCurScr32[6];
                }

            }
            break;

            case ito::PrimitiveContainer::tRectangle:
            {

                if(type == ito::tFloat64)   // idx, type, x0, y0, z0, x1, y1, z1
                {
                    ids[geoElement]                      = static_cast<ito::float32>(ptrCurScr64[0]);
                    xCoords0[geoElement] = static_cast<ito::float64>(ptrCurScr64[2]);
                    yCoords0[geoElement] = static_cast<ito::float64>(ptrCurScr64[3]);
                    xCoords1[geoElement] = static_cast<ito::float64>(ptrCurScr64[5]);
                    yCoords1[geoElement] = static_cast<ito::float64>(ptrCurScr64[6]);
                }
                else
                {
                    ids[geoElement]      = ptrCurScr32[0];
                    xCoords0[geoElement] = ptrCurScr32[2];
                    yCoords0[geoElement] = ptrCurScr32[3];
                    xCoords1[geoElement] = ptrCurScr32[5];
                    yCoords1[geoElement] = ptrCurScr32[6];
                }

            }
            break;

            case ito::PrimitiveContainer::tSquare:
            {
                types[geoElement] = (ito::float32) ito::PrimitiveContainer::tRectangle;

                ito::float32 xC, yC, a;

                if(type == ito::tFloat64)   // idx, type, xC, yC, zC, a
                {
                    ids[geoElement] = static_cast<ito::float32>(ptrCurScr64[0]);
                    xC              = static_cast<ito::float64>(ptrCurScr64[2]);
                    yC              = static_cast<ito::float64>(ptrCurScr64[3]);
                    a               = static_cast<ito::float64>(ptrCurScr64[5]);
                }
                else
                {
                    ids[geoElement] = ptrCurScr32[0];
                    xC              = ptrCurScr32[2];
                    yC              = ptrCurScr32[3];
                    a               = ptrCurScr32[5];
                }

                xCoords0[geoElement] = xC - a / 2.0;
                yCoords0[geoElement] = yC - a / 2.0;
                xCoords1[geoElement] = xC + a / 2.0;
                yCoords1[geoElement] = yC + a / 2.0;

            }
            break;

            case ito::PrimitiveContainer::tEllipse:
            {
                ito::float32 xC, yC, r1, r2;

                if(type == ito::tFloat64)   // idx, type, xC, yC, zC, a
                {
                    ids[geoElement] = static_cast<ito::float32>(ptrCurScr64[0]);
                    xC              = static_cast<ito::float64>(ptrCurScr64[2]);
                    yC              = static_cast<ito::float64>(ptrCurScr64[3]);
                    r1              = static_cast<ito::float64>(ptrCurScr64[5]);
                    r2              = static_cast<ito::float64>(ptrCurScr64[6]);
                }
                else
                {
                    ids[geoElement] = ptrCurScr32[0];
                    xC              = ptrCurScr32[2];
                    yC              = ptrCurScr32[3];
                    r1              = ptrCurScr32[5];
                    r2              = ptrCurScr32[6];
                }

                xCoords0[geoElement] = xC - r1;
                yCoords0[geoElement] = yC - r2;
                xCoords1[geoElement] = xC + r1;
                yCoords1[geoElement] = yC + r2;

            }
            break;

            case ito::PrimitiveContainer::tCircle:
            {
                types[geoElement] = (ito::float32) ito::PrimitiveContainer::tEllipse;
                ito::float32 xC, yC, r;

                if(type == ito::tFloat64)   // idx, type, xC, yC, zC, a
                {
                    ids[geoElement] = static_cast<ito::float32>(ptrCurScr64[0]);
                    xC              = static_cast<ito::float64>(ptrCurScr64[2]);
                    yC              = static_cast<ito::float64>(ptrCurScr64[3]);
                    r              = static_cast<ito::float64>(ptrCurScr64[5]);
                }
                else
                {
                    ids[geoElement] = ptrCurScr32[0];
                    xC              = ptrCurScr32[2];
                    yC              = ptrCurScr32[3];
                    r              = ptrCurScr32[5];
                }

                xCoords0[geoElement] = xC - r;
                yCoords0[geoElement] = yC - r;
                xCoords1[geoElement] = xC + r;
                yCoords1[geoElement] = yC + r;

            }
            break;

            default:
                plotItemsFinished(0, true);
                m_pContent->statusBarMessage(tr("Could not convert elements, type undefined"), 600 );
                return;    
        }

    }

    ito::RetVal retval = m_pContent->plotMarkers(&coords, "b", "", 0);

    m_pContent->replot();

    if(retval.containsError())
    {
        m_pContent->statusBarMessage(tr("Could not set elements"), 600 );
        plotItemsFinished(0, true);
        return;    
    }

    plotItemsFinished(0, false);
    this->updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getkeepAspectRatio(void) const 
{
    return ((InternalData*) m_pVData)->m_keepAspect;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setkeepAspectRatio(const bool &keepAspectEnable)
{
    if (m_pActAspectRatio) //if property is set in designer or by python, the action should represent the current status, too
    {
        m_pActAspectRatio->setChecked(keepAspectEnable);
    }
    mnuActRatio(keepAspectEnable);
    this->updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActRatio(bool checked)
{
    /*((InternalData*) m_pVData)->m_keepAspect = checked;
    
    if(m_pActZoom->isChecked()) m_pActZoom->setChecked(false);
    if(m_pActPan->isChecked()) m_pActPan->setChecked(false);
    
    m_pActPan->setEnabled(!checked);
    m_pActZoom->setEnabled(!checked);

    if(m_pContent)
    {
        m_pContent->m_pZoomer->zoom(0);
        m_pContent->setState(PlotCanvas::tIdle);
        m_pContent->configRescaler();
    }*/
    if(m_pVData == NULL) return;
    ((InternalData*) m_pVData)->m_keepAspect = checked;
    if(m_pContent) (m_pContent)->configRescaler();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resizeEvent ( QResizeEvent * event )
{
    if(m_pContent) m_pContent->configRescaler();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActCenterMarker(bool checked)
{
    setEnabledCenterMarker(checked);
}
//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getEnabledCenterMarker(void) const 
{
    return ((InternalData*) m_pVData)->m_showCenterMarker;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setEnabledCenterMarker(const bool &enabled)
{
    if(m_pVData == NULL) return;
    if (m_pActCntrMarker && m_pActCntrMarker->isChecked() != enabled) //if property is set in designer or by python, the action should represent the current status, too
    {
        m_pActCntrMarker->setChecked(enabled);
    }

    ((InternalData*) m_pVData)->m_showCenterMarker = enabled;
    if(m_pContent)
    {
        m_pContent->setState(((InternalData*) m_pVData)->m_state);
        m_pContent->replot();
    }

    this->updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
int Itom2dQwtPlot::getOverlayAlpha () const 
{
    return ((InternalData*) m_pVData)->m_alpha;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setOverlayAlpha (const int alpha)
{
    if(m_pVData == NULL) return;
    ((InternalData*) m_pVData)->m_alpha = alpha > 0 && alpha < 255 ? alpha : ((InternalData*) m_pVData)->m_alpha;
    if(m_pContent) m_pContent->alphaChanged();
    this->m_pOverlaySlider->setValue(((InternalData*) m_pVData)->m_alpha);
    this->updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getEnabledPlotting(void) const 
{
    return ((InternalData*) m_pVData)->m_enablePlotting;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setEnabledPlotting(const bool &enabled)
{
    if(m_pVData == NULL) return;
    ((InternalData*) m_pVData)->m_enablePlotting = enabled;
    m_pActClearDrawings->setEnabled(enabled);
    m_pActDrawMode->setEnabled(enabled);
    if(m_pActDrawMode->isChecked() && !enabled) m_pActDrawMode->setChecked(enabled);
    this->updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> Itom2dQwtPlot::getDisplayed(void)
{
    if(!m_pContent)
    {
        return QSharedPointer<ito::DataObject>(); 
    }

    return m_pContent->getDisplayed();
}
//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> Itom2dQwtPlot::getDisplayedLineCut(void)
{
    if(!m_pContent)
    {
        return QSharedPointer<ito::DataObject>(); 
    }

    if(!ito::ITOM_API_FUNCS_GRAPH)
    {
        return QSharedPointer<ito::DataObject>(); 
    }

    ito::AbstractDObjFigure* figure = NULL;
    QList<QObject*> keys = m_childFigures.keys();

    for( int i = 0; i < keys.length(); i++)
    {
        if( m_childFigures[keys[i]] == m_pContent->m_lineCutUID &&
            keys[i]->inherits("ito::AbstractDObjFigure"))                        
        {
            return (qobject_cast<ito::AbstractDObjFigure*>(keys[i]))->getDisplayed();
        }
    }

    return QSharedPointer<ito::DataObject>(); 
}
//----------------------------------------------------------------------------------------------------------------------------------
int Itom2dQwtPlot::getSelectedElement(void)const
{
    QHash<int, DrawItem*>::const_iterator it = ((InternalData*) m_pVData)->m_pDrawItems.begin();
    for (;it != ((InternalData*) m_pVData)->m_pDrawItems.end(); ++it)        
    {
        if(it.value() != NULL && it.value()->selected() != 0)
        { 
            return it.value()->m_idx;
        }
    }
    return -1;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setSelectedElement(const int idx)
{
    if(m_pVData == NULL) return;
    bool replot = false;
    bool failed = idx == -1 ? false : true;
    QHash<int, DrawItem*>::const_iterator it = ((InternalData*) m_pVData)->m_pDrawItems.begin();
    for (;it != ((InternalData*) m_pVData)->m_pDrawItems.end(); ++it)        
    {
        if(it.value() != NULL && it.value()->m_idx == idx)
        {
            it.value()->setSelected(true);
            failed = false;
            replot = true;
            continue;
        }
        if(it.value() != NULL && (it.value()->m_active != 0 || it.value()->selected()))
        { 
            replot = true;
            it.value()->m_active = 0;
            it.value()->setActive(0);
            it.value()->setSelected(false);
        }
    }

    if(m_pContent)
    {
        if(replot) m_pContent->replot();
        if(failed) emit m_pContent->statusBarMessage(tr("Could not set active element, index out of range."), 12000 );
    }
    this->updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
int Itom2dQwtPlot::getGeometricElementsCount() const 
{ 
    return ((InternalData*) m_pVData)->m_pDrawItems.size();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::setLinePlot(const double x0, const double y0, const double x1, const double y1, const int /*destID*/)
{
    if(m_pActLineCut->isCheckable() && m_pActLineCut->isEnabled())
    {
        m_pActLineCut->setChecked(true);
        mnuActLineCut(true);
    }
    else
    {
        return ito::RetVal(ito::retError, 0, tr("Set lineCut coordinates failed. Could not activate lineCut.").toLatin1().data());
    }

    if(m_pContent)
    {
        QPoint first(m_pContent->transform(QwtPlot::xBottom, x0), m_pContent->transform(QwtPlot::yLeft, y0));
        QPoint second(m_pContent->transform(QwtPlot::xBottom, x1), m_pContent->transform(QwtPlot::yLeft, y1));
        m_pContent->lineCutAppended(first);
        m_pContent->lineCutMoved(second);
    }
    else
    {
        return ito::RetVal(ito::retError, 0, tr("Set lineCut coordinates failed. Widget not ready.").toLatin1().data());
    }
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuOverlaySliderChanged(int value)
{
    if(m_pVData == NULL) return;
    if(value != ((InternalData*) m_pVData)->m_alpha)
    {
        ((InternalData*) m_pVData)->m_alpha = value;
        if(m_pContent) m_pContent->alphaChanged();
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getContextMenuEnabled() const
{
    if (m_pContent) return (m_pContent)->m_showContextMenu;
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setContextMenuEnabled(bool show)
{
    if (m_pContent) (m_pContent)->m_showContextMenu = show;
    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer< ito::DataObject > Itom2dQwtPlot::getOverlayImage() const 
{
    if (m_pContent) return m_pContent->getOverlayObject();
    return QSharedPointer< ito::DataObject >(NULL); 
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setOverlayImage(QSharedPointer< ito::DataObject > newOverlayObj)
{
    if (m_dataPointer.contains("overlayImage"))
    {
        //check if pointer of shared incoming data object is different to pointer of previous data object
        //if so, free previous
        if (m_dataPointer["overlayImage"].data() != newOverlayObj.data())
        {
            QSharedPointer<ito::DataObject> oldSource = m_dataPointer["overlayImage"]; //possible backup for previous source, this backup must be alive until updateParam with the new one has been completely propagated
            if (oldSource)
                oldSource->lockWrite();
            // sometimes crash here when replacing the source
            m_dataPointer["overlayImage"] = newOverlayObj;
            if (oldSource)
                oldSource->unlock();
        }  
    }
    else
    {
        m_dataPointer["overlayImage"] = newOverlayObj;
    }


    if(m_pContent) m_pContent->setOverlayObject(newOverlayObj.data());
    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetOverlayImage(void)
{
    if(m_pContent) m_pContent->setOverlayObject(NULL);
}
//----------------------------------------------------------------------------------------------------------------------------------
QColor Itom2dQwtPlot::getBackgroundColor(void) const
{
    if(m_pVData) 
    {
        return ((InternalData*)m_pVData)->m_backgnd;
    }
    else
        return Qt::white;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setBackgroundColor(const QColor newVal)
{
    if(m_pVData) 
    {
        InternalData* intData = ((InternalData*)m_pVData);
        intData->m_backgnd = newVal.rgb() & 0x00FFFFFF;
    }
    if(m_pContent) m_pContent->updateColors();

    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
QColor Itom2dQwtPlot::getAxisColor(void) const
{
    if(m_pVData) 
    {
        return ((InternalData*)m_pVData)->m_axisColor;
    }
    else
        return Qt::black;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setAxisColor(const QColor newVal)
{
    if(m_pVData) 
    {
        InternalData* intData = ((InternalData*)m_pVData);
        intData->m_axisColor = newVal.rgb() & 0x00FFFFFF;
    }
    if(m_pContent) m_pContent->updateColors();

    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
QColor Itom2dQwtPlot::getTextColor(void) const
{
    if(m_pVData) 
    {
        return ((InternalData*)m_pVData)->m_textColor;
    }
    else
        return Qt::black;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setTextColor(const QColor newVal)
{
    if(m_pVData) 
    {
        InternalData* intData = ((InternalData*)m_pVData);
        intData->m_textColor = newVal.rgb() & 0x00FFFFFF;
    }
    if(m_pContent) m_pContent->updateColors();

    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------

ito::ItomPlotHandle Itom2dQwtPlot::getStaticLineCutID() const
{
    ito::ItomPlotHandle handle(NULL, NULL, 0);
    if(m_pContent && this->m_pContent->m_lineCutUID > 0)
    {
        if (apiGetItomPlotHandleByID(m_pContent->m_lineCutUID, handle) == ito::retOk)
        {
            return handle;
        }
    }
    return ito::ItomPlotHandle(NULL, NULL, 0);
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setStaticLineCutID(const ito::ItomPlotHandle idx)
{
    ito::RetVal retval = ito::retOk;
    if(!ito::ITOM_API_FUNCS_GRAPH) return;
    
    if(m_pContent || idx.getObjectID() > -1)
    {
        ito::uint32 thisID = 0;
        retval += apiGetFigureIDbyHandle(this, thisID);

        if(idx.getObjectID() == thisID || retval.containsError())
        {
            return;
        }
        else
        {
            thisID = idx.getObjectID();
        }

        QWidget *lineCutObj = NULL;
        

        this->m_pContent->m_lineCutUID = thisID;
        retval += apiGetFigure("DObjStaticLine","", this->m_pContent->m_lineCutUID, &lineCutObj, this); //(newUniqueID, "itom1DQwtFigure", &lineCutObj);
        if (!lineCutObj && !lineCutObj->inherits("ito::AbstractDObjFigure"))
        {
            this->m_pContent->m_lineCutUID = 0;
        }

        m_lineCutType = this->m_pContent->m_lineCutUID != 0 ? ito::AbstractFigure::tUninitilizedExtern | ito::AbstractFigure::tVisibleOnInit : ito::AbstractFigure::tNoChildPlot;
    }

}
//----------------------------------------------------------------------------------------------------------------------------------

ito::ItomPlotHandle Itom2dQwtPlot::getStaticZSliceID() const
{
    ito::ItomPlotHandle handle(NULL, NULL, 0);
    if(m_pContent && this->m_pContent->m_zstackCutUID > 0)
    {
        if (apiGetItomPlotHandleByID(m_pContent->m_zstackCutUID, handle) == ito::retOk)
        {
            return handle;
        }
    }
    return ito::ItomPlotHandle(NULL, NULL, 0);
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setStaticZSliceID(const ito::ItomPlotHandle idx)
{
    ito::RetVal retval = ito::retOk;
    if(!ito::ITOM_API_FUNCS_GRAPH) return;
    
    if(m_pContent || idx.getObjectID() > -1)
    {
        ito::uint32 thisID = 0;
        retval += apiGetFigureIDbyHandle(this, thisID);

        if(idx.getObjectID() == thisID || retval.containsError())
        {
            return;
        }
        else
        {
            thisID = idx.getObjectID();
        }

        QWidget *lineCutObj = NULL;
        

        this->m_pContent->m_zstackCutUID = thisID;
        retval += apiGetFigure("DObjStaticLine","", this->m_pContent->m_zstackCutUID, &lineCutObj, this); //(newUniqueID, "itom1DQwtFigure", &lineCutObj);
        if (!lineCutObj && !lineCutObj->inherits("ito::AbstractDObjFigure"))
        {
            this->m_pContent->m_zstackCutUID = 0;
        }

        m_zSliceType = this->m_pContent->m_zstackCutUID != 0 ? ito::AbstractFigure::tUninitilizedExtern | ito::AbstractFigure::tVisibleOnInit : ito::AbstractFigure::tNoChildPlot;
    }

}
//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getMarkerLablesVisible(void) const
{
    if(m_pVData) 
    {
        return ((InternalData*)m_pVData)->m_markerLabelVisible;
    }
    else
        return false;    
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setMarkerLablesVisible(const bool val)
{
    if(!m_pVData) 
    {
        return ;
    }
    if(val != ((InternalData*)m_pVData)->m_markerLabelVisible)
    {
        ((InternalData*)m_pVData)->m_markerLabelVisible = val;
        if(m_pContent) m_pContent->updateLabelVisibility();
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
Itom2DQwt::tModificationState Itom2dQwtPlot::getModState(void) const
{
    if(!m_pVData) 
    {
        return Itom2DQwt::tMoveGeometricElements;
    }
    return (((InternalData*)m_pVData)->m_modState);
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setModState(const Itom2DQwt::tModificationState val)
{
    if(!m_pVData) 
    {
        return;
    }
    //if(val < 1 || val > 4) return;

    ((InternalData*)m_pVData)->m_modState = val;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::exportCanvas(const bool copyToClipboardNotFile, const QString &fileName, QSizeF curSize /*= QSizeF(0.0,0.0)*/, const int resolution /*= 300*/)
{
    if(!m_pContent)
    {
        return ito::RetVal(ito::retError, 0, tr("Export image failed, canvas handle not initilized").toLatin1().data());
    }

    if(curSize.height() == 0 || curSize.width() == 0)
    {
        curSize = m_pContent->size();
    }
    QBrush curBrush = m_pContent->canvasBackground();

    QPalette curPalette = m_pContent->palette();

    m_pContent->setAutoFillBackground( true );
    m_pContent->setPalette( Qt::white );
    m_pContent->setCanvasBackground(Qt::white);    

    m_pContent->replot();

    QwtPlotRenderer renderer;

    // flags to make the document look like the widget
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    //renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true); //deprecated in qwt 6.1.0

    if(copyToClipboardNotFile)
    {
        m_pContent->statusBarMessage(tr("copy current view to clipboard..."));

        qreal resFaktor = resolution / 72.0 + 0.5;
        resFaktor = resFaktor < 1.0 ? 1.0 : resFaktor;

        QSize myRect(curSize.width() * resFaktor, curSize.height() * resFaktor);
        QClipboard *clipboard = QApplication::clipboard();
        QImage img(myRect, QImage::Format_ARGB32);
        QPainter painter(&img);
        painter.scale(resFaktor, resFaktor);
        renderer.render(m_pContent, &painter, m_pContent->rect());
        clipboard->setImage(img);    

        m_pContent->statusBarMessage(tr("copy current view to clipboard. done."), 1000);
    }
    else
    {
        renderer.renderDocument(m_pContent, fileName, curSize, resolution);
    }

    m_pContent->setPalette( curPalette);
    m_pContent->setCanvasBackground( curBrush);

    m_pContent->replot();
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::copyToClipBoard()
{
    return exportCanvas(true, "");
}
//----------------------------------------------------------------------------------------------------------------------------------
QPixmap Itom2dQwtPlot::renderToPixMap(const int xsize, const int ysize, const int resolution) 
{
    QSizeF curSize(xsize, ysize);
    if(!m_pContent)
    {
        QSize myRect(curSize.width(), curSize.height());
        QPixmap destinationImage(myRect);
        return destinationImage;
    }

    if(curSize.height() == 0 || curSize.width() == 0)
    {
        curSize = m_pContent->size();
    }

    int resFaktor = cv::saturate_cast<int>(resolution / 72.0 + 0.5);
    resFaktor = resFaktor < 1 ? 1 : resFaktor;
    resFaktor = resFaktor > 6 ? 6 : resFaktor;
    QSize myRect(curSize.width() * resFaktor, curSize.height() * resFaktor);

    QPixmap destinationImage(myRect);

    if(!m_pContent)
    {
        destinationImage.fill(Qt::red);
        return destinationImage;
    }
    destinationImage.fill(Qt::white);
    QBrush curBrush = m_pContent->canvasBackground();

    QPalette curPalette = m_pContent->palette();

    m_pContent->setAutoFillBackground( true );
    m_pContent->setPalette( Qt::white );
    m_pContent->setCanvasBackground(Qt::white);    

    m_pContent->replot();

    QwtPlotRenderer renderer;

    // flags to make the document look like the widget
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    //renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true); //deprecated in qwt 6.1.0

    //QImage img(myRect, QImage::Format_ARGB32);
    QPainter painter(&destinationImage);
    painter.scale(resFaktor, resFaktor);
    renderer.render(m_pContent, &painter, m_pContent->rect());
    //destinationImage.convertFromImage(img);


    m_pContent->setPalette( curPalette);
    m_pContent->setCanvasBackground( curBrush);

    m_pContent->replot();

    return destinationImage;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::setGeometricElementLabel(int id, QString label)
{
    if(!m_pVData) 
    {
        return ito::RetVal(ito::retError, 0, tr("Could not access internal data structur").toLatin1().data());
    }

    InternalData* pData = ((InternalData*)m_pVData);

    if(!pData->m_pDrawItems.contains(id))
    {
        return ito::RetVal(ito::retError, 0, tr("Geometric element not found").toLatin1().data());
    }

    pData->m_pDrawItems[id]->setLabel(label);
    if(m_pContent)
    {
        m_pContent->replot();
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::setGeometricElementLabelVisible(int id, bool setVisible)
{
    if(!m_pVData) 
    {
        return ito::RetVal(ito::retError, 0, tr("Could not access internal data structur").toLatin1().data());
    }

    InternalData* pData = ((InternalData*)m_pVData);

    if(!pData->m_pDrawItems.contains(id))
    {
        return ito::RetVal(ito::retError, 0, tr("Geometric element not found").toLatin1().data());
    }

    pData->m_pDrawItems[id]->setLabelVisible(setVisible);
    if(m_pContent)
    {
        m_pContent->replot();
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActSendCurrentToWorkspace()
{
    bool ok;
    QString varname = QInputDialog::getText(this, tr("Current to workspace"), tr("Indicate the python variable name for the currently visible object"), QLineEdit::Normal, "zoom_object", &ok);
    if (ok && varname != "")
    {
        QSharedPointer<ito::DataObject> obj = getDisplayed();
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
                msgBox.setInformativeText(retval.errorMessage());
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
                msgBox.setInformativeText(retval.errorMessage());
            }
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.exec();
        }
    }
}
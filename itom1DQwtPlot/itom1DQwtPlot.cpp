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

#include "itom1DQwtPlot.h"
#include "dialog1DScale.h"
#include "dataObjectSeriesData.h"

#include "DataObject/dataObjectFuncs.h"
#include "DataObject/dataobj.h"

#include "common/sharedStructuresPrimitives.h"
#include "../sharedFiles/dialogExportProperties.h"

#include <qmessagebox.h>
#include <qfiledialog.h>
#include <qimagewriter.h>
#include <qsharedpointer.h>

#include <qwt_plot.h>
#include <qgridlayout.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_renderer.h>
#include <qwt_text_label.h>
#include <qwt_scale_widget.h>
#include <qwt_symbol.h>

using namespace ito;

//: AbstractDObjFigure("", ModeInItomFigure, parent)
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::constructor()
{
    m_pInput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, tr("Points for line plots from 2d objects").toLatin1().data()));
    
    
    createActions();

    //int id = qRegisterMetaType<QSharedPointer<ito::DataObject> >("QSharedPointer<ito::DataObject>");

    QToolBar *mainTb = new QToolBar(tr("1D plotting toolbar"), this);
    addToolBar(mainTb, "mainToolBar");

    QMenu *contextMenu = new QMenu(QObject::tr("plot1D"), this);
    contextMenu->addAction(m_pActSave);
    contextMenu->addAction(m_pActCopyClipboard);
    contextMenu->addAction(m_pActSendCurrentToWorkspace);
    contextMenu->addSeparator();
    contextMenu->addAction(m_pActHome);
    contextMenu->addAction(m_pActScaleSetting);
    contextMenu->addSeparator();
    contextMenu->addAction(m_pActPan);
    contextMenu->addAction(m_pActZoomToRect);
    contextMenu->addAction(m_pActMarker);
    contextMenu->addSeparator();
    contextMenu->addAction(mainTb->toggleViewAction());

    // first block is zoom, scale settings, home
    mainTb->addAction(m_pActSave);
    mainTb->addSeparator();
    mainTb->addAction(m_pActHome);
    mainTb->addAction(m_pActScaleSetting);
    mainTb->addAction(m_pRescaleParent);
    mainTb->addAction(m_pActPan);
    mainTb->addAction(m_pActZoomToRect);
    mainTb->addAction(m_pActAspectRatio);
    mainTb->addAction(m_pActMultiRowSwitch);
    mainTb->addAction(m_pActRGBSwitch);
    // first block is zoom, scale settings, home
    mainTb->addSeparator();
    mainTb->addAction(m_pActMarker);
    mainTb->addAction(m_pActSetMarker);
    mainTb->addSeparator();
    mainTb->addAction(m_pActDrawMode);
    mainTb->addAction(m_pActClearDrawings);
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

    m_data = new InternalData();
    m_data->m_pDrawItems.clear();
    m_data->m_autoAxisLabel = true;
    m_data->m_autoValueLabel = true;
    m_data->m_valueScaleAuto = true;
    m_data->m_dataType = ito::tFloat64;
    m_data->m_valueMin = -127.0;
    m_data->m_valueMax = 128.0;
    m_data->m_axisScaleAuto = true;
    m_data->m_forceValueParsing = false;

    m_pContent = new Plot1DWidget(contextMenu, m_data, this);

    connect(m_pContent, SIGNAL(statusBarClear()), (QObject*)statusBar(), SLOT(clearMessage()));
    connect(m_pContent, SIGNAL(statusBarMessage(QString,int)), (QObject*)statusBar(), SLOT(showMessage(QString,int)));

    m_pContent->setObjectName("canvasWidget");

    connect(m_pContent, SIGNAL(setPickerText(const QString &, const QString &)), this, SLOT(setPickerText(const QString &, const QString &)));

    setFocus();
    setCentralWidget(m_pContent);
    m_pContent->setFocus();

    QMenu *menuView = new QMenu(tr("View"), this);
    menuView->addAction(m_pActHome);
    menuView->addAction(m_pActPan);
    menuView->addAction(m_pActZoomToRect);
    menuView->addAction(m_pActAspectRatio);
    menuView->addAction(m_pActGrid);
    menuView->addSeparator();
    menuView->addAction(m_pActScaleSetting);
    menuView->addAction(m_pRescaleParent);
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
    menuTools->addAction(m_pActMarker);
    menuTools->addAction(m_pActSetMarker);
    menuTools->addSeparator();
    menuTools->addAction(m_pActDrawMode);
    menuTools->addAction(m_pActClearDrawings);
    addMenu(menuTools); //AbstractFigure takes care of the menu

    setPropertyObservedObject(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::Itom1DQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    AbstractDObjFigure(itomSettingsFile, windowMode, parent),
    m_pContent(NULL),
    m_pActScaleSetting(NULL),
    m_pRescaleParent(NULL),
    m_pActForward(NULL),
    m_pActBack(NULL),
    m_pActHome(NULL),
    m_pActSave(NULL),
    m_pActCopyClipboard(NULL),
    m_pActPan(NULL),
    m_pActZoomToRect(NULL),
    m_pActMarker(NULL),
    m_pMnuSetMarker(NULL),
    m_pActSetMarker(NULL),
    m_pActCmplxSwitch(NULL),
    m_pMnuCmplxSwitch(NULL),
    m_pActRGBSwitch(NULL),
    m_pMnuRGBSwitch(NULL),
    m_pLblMarkerOffsets(NULL),
    m_pLblMarkerCoords(NULL),
    m_pActAspectRatio(NULL),
    m_pActClearDrawings(NULL),
    m_pActDrawMode(NULL),
    m_pMnuDrawMode(NULL),
    m_pDrawModeActGroup(NULL),
    m_pActProperties(NULL),
    m_pActMultiRowSwitch(NULL),
    m_pMnuMultiRowSwitch(NULL),
    m_pActSendCurrentToWorkspace(NULL)
{
    constructor();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::Itom1DQwtPlot(QWidget *parent) :
    AbstractDObjFigure("", AbstractFigure::ModeStandaloneInUi, parent),
    m_pContent(NULL),
    m_pActScaleSetting(NULL),
    m_pRescaleParent(NULL),
    m_pActForward(NULL),
    m_pActBack(NULL),
    m_pActHome(NULL),
    m_pActSave(NULL),
    m_pActCopyClipboard(NULL),
    m_pActPan(NULL),
    m_pActZoomToRect(NULL),
    m_pActMarker(NULL),
    m_pMnuSetMarker(NULL),
    m_pActSetMarker(NULL),
    m_pActCmplxSwitch(NULL),
    m_pMnuCmplxSwitch(NULL),
    m_pActRGBSwitch(NULL),
    m_pMnuRGBSwitch(NULL),
    m_pLblMarkerOffsets(NULL),
    m_pLblMarkerCoords(NULL),
    m_pActAspectRatio(NULL),
    m_pActClearDrawings(NULL),
    m_pActDrawMode(NULL),
    m_pMnuDrawMode(NULL),
    m_pDrawModeActGroup(NULL),
    m_pActProperties(NULL),
    m_pActMultiRowSwitch(NULL),
    m_pMnuMultiRowSwitch(NULL),
    m_pActSendCurrentToWorkspace(NULL)
{
    constructor();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::~Itom1DQwtPlot()
{
    if (m_pMnuMultiRowSwitch != NULL)
    {
        m_pMnuMultiRowSwitch->deleteLater();
        m_pMnuMultiRowSwitch = NULL;    
    }
    if (m_pMnuCmplxSwitch != NULL)
    {
        m_pMnuCmplxSwitch->deleteLater();
        m_pMnuCmplxSwitch = NULL;
    }
    if (m_pMnuRGBSwitch != NULL)
    {
        m_pMnuRGBSwitch->deleteLater();
        m_pMnuRGBSwitch = NULL;
    }

    m_pContent->deleteLater();
//    m_pContent = NULL;
    m_pContent = NULL;
    if (m_data)
        delete m_data;
    if (m_pMnuSetMarker)
        delete(m_pMnuSetMarker);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::init()
{ 
    return m_pContent->init(); 
} //called when api-pointers are transmitted, directly after construction
//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwt::tMultiLineMode Itom1DQwtPlot::getRowPresentation(void) const
{
    return m_data->m_multiLine;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setRowPresentation(const Itom1DQwt::tMultiLineMode idx)
{
    QAction *action = NULL;
    bool ok;
    foreach(QAction *a, m_pMnuMultiRowSwitch->actions())
    {
        if (a->data().toInt(&ok) == idx)
        {
            if (ok) action = a;
        }
    }

    if (action) m_pActMultiRowSwitch->setIcon(action->icon());

    switch(idx)
    {
        case Itom1DQwt::MultiLayerCols:
        case Itom1DQwt::MultiLayerRows:
            m_data->m_multiLine = Itom1DQwt::MultiLayerAuto;
            break;        
        default:
            m_data->m_multiLine = idx;
            break;

    }

    if(m_pContent)
    {
        QVector<QPointF> bounds = getBounds();
        m_pContent->refreshPlot((ito::DataObject*)m_pInput["source"]->getVal<char*>(), bounds);

        //if y-axis is set to auto, it is rescaled here with respect to the new limits, else the manual range is kept unchanged.
        m_pContent->setInterval(Qt::YAxis, m_data->m_valueScaleAuto, m_data->m_valueMin, m_data->m_valueMax); //replot is done here 
    }

    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetRowPresentation() 
{
    setRowPresentation(Itom1DQwt::AutoRowCol);
}
//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getRGBPresentation(void) const
{
    return m_data->m_colorLine;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setRGBPresentation(const int idx)
{
    QAction *action = NULL;
    bool ok;
    foreach(QAction *a, m_pMnuRGBSwitch->actions())
    {
        if (a->data().toInt(&ok) == idx)
        {
            if (ok) action = a;
        }
    }

    if (action) m_pMnuRGBSwitch->setIcon(action->icon());

    switch(idx)
    {
        default:
        case 0:
            m_data->m_colorLine = Itom1DQwt::AutoColor;
            break;
        case 1:
            m_data->m_colorLine = Itom1DQwt::Gray;
            break;
        case 2:
            m_data->m_colorLine = Itom1DQwt::RGB;
            break;
        case 3:
            m_data->m_colorLine = Itom1DQwt::RGBA;
            break;
        case 4:
            m_data->m_colorLine = Itom1DQwt::RGBGray;
            break;
    }

    if(m_pContent)
    {
        QVector<QPointF> bounds = getBounds();
        m_pContent->refreshPlot((ito::DataObject*)m_pInput["source"]->getVal<char*>(), bounds);

        //if y-axis is set to auto, it is rescaled here with respect to the new limits, else the manual range is kept unchanged.
        m_pContent->setInterval(Qt::YAxis, m_data->m_valueScaleAuto, m_data->m_valueMin, m_data->m_valueMax); //replot is done here 
    }

    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetRGBPresentation() 
{
    setRGBPresentation(0);
}
//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getPickerLimit(void) const
{
    if(m_data) return m_data->m_pickerLimit;
    return 2;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerLimit(const int idx)
{
    if(m_data) m_data->m_pickerLimit = idx;
    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetPickerLimit()
{
    if(m_data) m_data->m_pickerLimit = 2;
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getPickerCount(void) const
{
    if(m_pContent)
    {
        return m_pContent->getPickerCount();
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer< ito::DataObject > Itom1DQwtPlot::getPicker() const
{
    if(m_pContent)
    {
        return m_pContent->getPlotPicker();
    }
    return QSharedPointer< ito::DataObject >(new ito::DataObject());
}

//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getGeometricElementsCount() const 
{ 
    return m_data->m_pDrawItems.size();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom1DQwtPlot::getkeepAspectRatio(void) const 
{
    return m_data->m_keepAspect;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom1DQwtPlot::getEnabledPlotting(void) const 
{
    return m_data->m_enablePlotting;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::createActions()
{
    QAction *a = NULL;

    //m_actHome
    m_pActHome = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"), tr("Home"), this);
    a->setObjectName("actHome");
    a->setToolTip(tr("Reset original view"));
    a->setShortcut(Qt::CTRL + Qt::Key_0);
    connect(a, SIGNAL(triggered()), this, SLOT(mnuHome()));
 
    //m_actSave
    m_pActSave = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("Save..."), this);
    a->setObjectName("actSave");
    a->setShortcut(QKeySequence::Save);
    a->setToolTip(tr("Export current view..."));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuExport()));

    //m_pActSendCurrentToWorkspace
    m_pActSendCurrentToWorkspace = a = new QAction(QIcon(":/plugins/icons/sendToPython.png"), tr("Send current view to workspace..."), this);
    a->setObjectName("actSendCurrentToWorkspace");
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActSendCurrentToWorkspace()));

    //m_actCopyClipboard
    m_pActCopyClipboard = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/clipboard.png"), tr("Copy to clipboard"), this);
    a->setObjectName("actCopyClipboard");
    a->setShortcut(QKeySequence::Copy);
    a->setToolTip(tr("Copies the current view to the clipboard"));
    connect(a, SIGNAL(triggered()), this, SLOT(copyToClipBoard()));

    //m_actScaleSetting
    m_pActScaleSetting = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/autoscal.png"), tr("Scale Settings..."), this);
    a->setObjectName("actScaleSetting");
    a->setToolTip(tr("Set the ranges and offsets of this view"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuScaleSetting()));

    //m_rescaleParent
    m_pRescaleParent = a = new QAction(QIcon(":/itom1DQwtFigurePlugin/icons/parentScale.png"), tr("Parent Scale Settings"), this);
    a->setObjectName("rescaleParent");
    a->setToolTip(tr("Set the value-range of the parent view according to this plot"));
    a->setVisible(false);
    connect(a, SIGNAL(triggered()), this, SLOT(mnuParentScaleSetting()));

    //m_actForward
    m_pActForward = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/forward.png"), tr("Forward"), this);
    a->setObjectName("actionForward");
    a->setEnabled(false);
    a->setToolTip(tr("Forward to next line"));

    //m_actBack
    m_pActBack = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/back.png"), tr("Back"), this);
    a->setObjectName("actionBack");
    a->setEnabled(false);
    a->setToolTip(tr("Back to previous line"));

    //m_actPan
    m_pActPan = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), tr("Move"), this);
    a->setObjectName("actionPan");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Pan axes with left mouse, zoom with right"));
    connect(a, SIGNAL(toggled(bool)), this, SLOT(mnuPanner(bool)));

    //m_actZoomToRect
    m_pActZoomToRect = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"), tr("Zoom To Rectangle"), this);
    a->setObjectName("actionZoomToRect");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Zoom to rectangle"));
    connect(a, SIGNAL(toggled(bool)), this, SLOT(mnuZoomer(bool)));

    //m_pActAspectRatio
    m_pActAspectRatio = a = new QAction(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio11.png"), tr("Lock Aspect Ratio"), this);
    a->setObjectName("actRatio");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Toggle fixed / variable aspect ration between axis x and y"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActRatio(bool)));

    //m_actMarker
    m_pActMarker = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/marker.png"), tr("Marker"), this);
    a->setObjectName("actionMarker");
    a->setCheckable(true);
    a->setChecked(false);
    connect(a, SIGNAL(toggled(bool)), this, SLOT(mnuMarkerClick(bool)));    

    //m_actSetMarker
    m_pActSetMarker = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/markerPos.png"), tr("Set Markers to"), this);
    m_pMnuSetMarker = new QMenu("Marker Switch");
    m_pMnuSetMarker->addAction(tr("To Min-Max"));
    m_pActSetMarker->setMenu(m_pMnuSetMarker);
    connect(m_pMnuSetMarker, SIGNAL(triggered(QAction*)), this, SLOT(mnuSetMarker(QAction*)));

    //m_actCmplxSwitch
    m_pActCmplxSwitch = new QAction(QIcon(":/itomDesignerPlugins/complex/icons/ImRe.png"), tr("Switch Imag, Real, Abs, Pha"), this);
    m_pMnuCmplxSwitch = new QMenu("Complex Switch");
    m_pMnuCmplxSwitch->addAction(tr("Imag"));
    m_pMnuCmplxSwitch->addAction(tr("Real"));
    m_pMnuCmplxSwitch->addAction(tr("Abs"));
    m_pMnuCmplxSwitch->addAction(tr("Pha"));
    m_pActCmplxSwitch->setMenu(m_pMnuCmplxSwitch);
    m_pActCmplxSwitch->setVisible(false);
    connect(m_pMnuCmplxSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuCmplxSwitch(QAction*)));

    //m_pActMultiRowSwitch
    m_pActMultiRowSwitch = new QAction(QIcon(":/itomDesignerPlugins/axis/icons/xvauto_plot.png"), tr("Switch Auto, first row, first column, multi row, multi column"), this);
    m_pMnuMultiRowSwitch = new QMenu("Data Representation");
    a = m_pMnuMultiRowSwitch->addAction(QIcon(":/itomDesignerPlugins/axis/icons/xvauto_plot.png"), tr("Auto"));
    a->setData(0);
    a = m_pMnuMultiRowSwitch->addAction(QIcon(":/itomDesignerPlugins/axis/icons/xv_plot.png"), tr("first row"));
    a->setData(1);
    a = m_pMnuMultiRowSwitch->addAction(QIcon(":/itomDesignerPlugins/axis/icons/yv_plot.png"), tr("first column"));
    a->setData(2);
    a = m_pMnuMultiRowSwitch->addAction(QIcon(":/itomDesignerPlugins/axis/icons/xvm_plot.png"), tr("multi row"));
    a->setData(3);
    a = m_pMnuMultiRowSwitch->addAction(QIcon(":/itomDesignerPlugins/axis/icons/yvm_plot.png"), tr("multi column"));
    a->setData(4);
    a = m_pMnuMultiRowSwitch->addAction(QIcon(":/itomDesignerPlugins/axis/icons/yxvzm_plot.png"), tr("multi layer"));
    a->setData(5);
    m_pActMultiRowSwitch->setMenu(m_pMnuMultiRowSwitch);
    m_pActMultiRowSwitch->setVisible(true);
    connect(m_pMnuMultiRowSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuMultiRowSwitch(QAction*)));

    //m_pActRGBSwitch
    m_pActRGBSwitch = new QAction(QIcon(":/itomDesignerPlugins/axis/icons/RGBA_RGB.png"), tr("Switch Auto, gray, rgb, rgba, rgb-gray"), this);
    m_pMnuRGBSwitch = new QMenu("Color Representation");
    a = m_pMnuRGBSwitch->addAction(QIcon(":/itomDesignerPlugins/axis/icons/RGBA_RGB.png"), tr("auto value"));
    a->setData(0);
    a = m_pMnuRGBSwitch->addAction(QIcon(":/itomDesignerPlugins/axis/icons/RGB_Gray.png"), tr("gray value"));
    a->setData(1);
    a = m_pMnuRGBSwitch->addAction(QIcon(":/itomDesignerPlugins/axis/icons/RGBA_RGB.png"), tr("RGB-lines"));
    a->setData(2);
    a = m_pMnuRGBSwitch->addAction(QIcon(":/itomDesignerPlugins/axis/icons/RGBA_RGBA.png"), tr("RGBA-lines"));
    a->setData(3);
    a = m_pMnuRGBSwitch->addAction(QIcon(":/itomDesignerPlugins/axis/icons/RGB_RGBGray.png"), tr("RGB + Gray"));
    a->setData(4);
    m_pActRGBSwitch->setMenu(m_pMnuRGBSwitch);
    m_pActRGBSwitch->setVisible(false);
    connect(m_pMnuRGBSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuRGBSwitch(QAction*)));

    //m_actDrawMode
    m_pActDrawMode = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/marker.png"), tr("Switch Draw Mode"), this);
    m_pMnuDrawMode = new QMenu("Draw Mode", this);

    m_pDrawModeActGroup = new QActionGroup(this);
    a = m_pDrawModeActGroup->addAction(tr("Point"));
    a->setData(Plot1DWidget::tPoint);
    m_pMnuDrawMode->addAction(a);
    a->setCheckable(true);
    a->setChecked(true);

    a = m_pDrawModeActGroup->addAction(tr("Line"));
    a->setData(Plot1DWidget::tLine);
    m_pMnuDrawMode->addAction(a);
    a->setCheckable(true);

    a = m_pDrawModeActGroup->addAction(tr("Rectangle"));
    a->setData(Plot1DWidget::tRect);
    m_pMnuDrawMode->addAction(a);
    a->setCheckable(true);

    a = m_pDrawModeActGroup->addAction(tr("Ellipse"));
    a->setData(Plot1DWidget::tEllipse);
    m_pMnuDrawMode->addAction(a);
    a->setCheckable(true);

    m_pActDrawMode->setMenu(m_pMnuDrawMode);
    m_pActDrawMode->setVisible(true);
    m_pActDrawMode->setCheckable(true);

    connect(m_pDrawModeActGroup, SIGNAL(triggered(QAction*)), this, SLOT(mnuDrawMode(QAction*)));
    connect(m_pActDrawMode, SIGNAL(triggered(bool)), this, SLOT(mnuDrawMode(bool)));
    
    //m_pActClearDrawings
    m_pActClearDrawings = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/editDelete.png"), tr("Clear Marker"), this);
    a->setObjectName("actClearGeometricElements");
    a->setCheckable(false);
    a->setChecked(false);
    a->setToolTip(tr("Clear all existing geometric elements"));
    connect(a, SIGNAL(triggered()), this, SLOT(clearGeometricElements()));

    //Labels for current cursor position
    m_pLblMarkerCoords = new QLabel("    \n    ", this);
    m_pLblMarkerCoords->setAlignment(Qt::AlignRight | Qt::AlignTop);
    m_pLblMarkerCoords->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Ignored);
    m_pLblMarkerCoords->setObjectName(tr("Marker Positions"));

    m_pLblMarkerOffsets = new QLabel("    \n    ", this);
    m_pLblMarkerOffsets->setAlignment(Qt::AlignRight | Qt::AlignTop);
    m_pLblMarkerOffsets->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Ignored);
    m_pLblMarkerOffsets->setObjectName(tr("Marker Offsets"));

    m_pActProperties = this->getPropertyDockWidget()->toggleViewAction();
    connect(m_pActProperties, SIGNAL(triggered(bool)), this, SLOT(mnuShowProperties(bool)));

    m_pActGrid = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/grid.png"), tr("Grid"), this);
    a->setObjectName("actGrid");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Shows/hides a grid"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuGridEnabled(bool)));

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::applyUpdate()
{
    QVector<QPointF> bounds = getBounds();

    if ((ito::DataObject*)m_pInput["source"]->getVal<void*>())
    {
        m_pOutput["displayed"]->copyValueFrom(m_pInput["source"]);
        // why "source" is used here and not "displayed" .... ck 05/15/2013
        m_pContent->refreshPlot((ito::DataObject*)m_pInput["source"]->getVal<char*>(), bounds);

        ito::Channel* dataChannel = getInputChannel("source");
        m_pRescaleParent->setVisible(dataChannel && dataChannel->getParent());
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setSource(QSharedPointer<ito::DataObject> source)
{
    m_data->m_forceValueParsing = true; //recalculate boundaries since content of data object may have changed
    AbstractDObjFigure::setSource(source);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom1DQwtPlot::getContextMenuEnabled() const
{
    if (m_pContent) return (m_pContent)->m_showContextMenu;
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setContextMenuEnabled(bool show)
{
    if (m_pContent) (m_pContent)->m_showContextMenu = show;
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setBounds(QVector<QPointF> bounds) 
{ 
    double *pointArr = new double[2 * bounds.size()];
    for (int np = 0; np < bounds.size(); np++)
    {
        pointArr[np * 2] = bounds[np].x();
        pointArr[np * 2 + 1] = bounds[np].y();
    }
    m_pInput["bounds"]->setVal(pointArr, 2 * bounds.size());
    delete[] pointArr;
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<QPointF> Itom1DQwtPlot::getBounds(void) 
{ 
    int numPts = m_pInput["bounds"]->getLen();
    QVector<QPointF> boundsVec;

    if (numPts > 0)
    {
        double *ptsDblVec = m_pInput["bounds"]->getVal<double*>();
        boundsVec.reserve(numPts / 2);
        for (int n = 0; n < numPts / 2; n++)
        {
            boundsVec.append(QPointF(ptsDblVec[n * 2], ptsDblVec[n * 2 + 1]));
        }
    }
    return boundsVec;
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom1DQwtPlot::getTitle() const
{
    if (m_data->m_autoTitle)
    {
        return "<auto>";
    }
    return m_data->m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setTitle(const QString &title)
{
    if (title == "<auto>")
    {
        m_data->m_autoTitle = true;
    }
    else
    {
        m_data->m_autoTitle = false;
        m_data->m_title = title;
    }

    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetTitle()
{
    m_data->m_autoTitle = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom1DQwtPlot::getAxisLabel() const
{
    if (m_data->m_autoAxisLabel)
    {
        return "<auto>";
    }
    return m_data->m_axisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setAxisLabel(const QString &label)
{
    if (label == "<auto>")
    {
        m_data->m_autoAxisLabel = true;
    }
    else
    {
        m_data->m_autoAxisLabel = false;
        m_data->m_axisLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetAxisLabel()
{
    m_data->m_autoAxisLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom1DQwtPlot::getValueLabel() const
{
    if (m_data->m_autoValueLabel)
    {
        return "<auto>";
    }
    return m_data->m_valueLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setValueLabel(const QString &label)
{
    if (label == "<auto>")
    {
        m_data->m_autoValueLabel = true;
    }
    else
    {
        m_data->m_autoValueLabel = false;
        m_data->m_valueLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetValueLabel()
{
    m_data->m_autoValueLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwt::ScaleEngine Itom1DQwtPlot::getValueScale() const
{
    if (m_pContent) 
    {
        return m_pContent->m_valueScale;
    }
    return Itom1DQwt::Linear;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setValueScale(const Itom1DQwt::ScaleEngine &scale)
{
    if (m_pContent)
    {
        m_pContent->setDefaultValueScaleEngine(scale);
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwt::ScaleEngine Itom1DQwtPlot::getAxisScale() const
{
    if (m_pContent) 
    {
        return m_pContent->m_axisScale;
    }
    return Itom1DQwt::Linear;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setAxisScale(const Itom1DQwt::ScaleEngine &scale)
{
    if (m_pContent)
    {
        m_pContent->setDefaultAxisScaleEngine(scale);
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom1DQwtPlot::getTitleFont(void) const
{
    if (m_pContent)
    {
        return m_pContent->titleLabel()->font();
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setTitleFont(const QFont &font)
{
    if (m_pContent)
    {
        m_pContent->titleLabel()->setFont(font);
        //m_pContent->replot();
        updatePropertyDock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom1DQwtPlot::getLabelFont(void) const
{
    if (m_pContent)
    {
        QwtText t = m_pContent->axisWidget(QwtPlot::xBottom)->title();
        return t.font();
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLabelFont(const QFont &font)
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

        updatePropertyDock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom1DQwtPlot::getAxisFont(void) const
{
    if (m_pContent)
    {
        return m_pContent->axisFont(QwtPlot::xBottom);
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setAxisFont(const QFont &font)
{
    if (m_pContent)
    {
        m_pContent->setAxisFont(QwtPlot::xBottom, font);
        m_pContent->setAxisFont(QwtPlot::yLeft, font);

        updatePropertyDock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom1DQwtPlot::getGrid(void) const
{
    if (m_pContent)
    {
        return m_pContent->m_gridEnabled;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setGrid(const bool &enabled)
{
    if (m_pContent)
    {
        m_pContent->setGridEnabled(enabled);
    }
    m_pActGrid->setChecked(enabled);

    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
qreal Itom1DQwtPlot::getLineWidth(void) const
{
    if (m_pContent)
    {
        return m_pContent->m_lineWidth;
    }
    return 1.0;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLineWidth(const qreal &width)
{
    if (m_pContent)
    {
        if (width >= 0.0)
        {
            m_pContent->setLineWidth(width);
        }
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
Qt::PenStyle Itom1DQwtPlot::getLineStyle(void) const
{
    if (m_pContent)
    {
        return m_pContent->m_lineStyle;
    }
    return Qt::SolidLine;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLineStyle(const Qt::PenStyle &style)
{
    if (m_pContent)
    {
        if (/*style != Qt::NoPen &&*/ style != Qt::CustomDashLine)
        {
            m_pContent->setLineStyle(style);
        }
    }

    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwt::tCurveStyle Itom1DQwtPlot::getCurveStyle(void) const
{
    if (m_pContent)
    {
        return m_pContent->m_qwtCurveStyle;
    }
    return Itom1DQwt::Lines;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setCurveStyle(const Itom1DQwt::tCurveStyle state)
{
    if (m_pContent)
    {
        m_pContent->setQwtLineStyle(state);
    }
    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::tSymbol Itom1DQwtPlot::getLineSymbol() const
{
    return (tSymbol)(m_data->m_lineSymbole);
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLineSymbol(const tSymbol symbol)
{
    if (m_pContent)
    {
        
        if(symbol < QwtSymbol::Path && symbol > -2 )
        {
            m_pContent->setSymbolStyle((QwtSymbol::Style)symbol, m_data->m_lineSymboleSize);
        }
    }

    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetLineSymbol()
{
    setLineSymbol(NoSymbol);
}
//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getLineSymbolSize() const
{
    return m_data->m_lineSymboleSize;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLineSymbolSize(const int size)
{
    if(size < 1 || size > 21)
        return;

    if (m_pContent)
    {
         m_pContent->setSymbolStyle(m_data->m_lineSymbole, size);
    }
    else
    {
        m_data->m_lineSymboleSize = size;
    }

    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetLineSymbolSize()
{
    if (m_pContent)
    {
        m_pContent->setSymbolStyle(m_data->m_lineSymbole, 1);
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::LegendPos Itom1DQwtPlot::getLegendPosition() const
{
    if (m_pContent)
    {
        if (m_pContent->m_legendVisible)
        {
            switch (m_pContent->m_legendPosition)
            {
            case QwtPlot::BottomLegend:
                return Bottom;
            case QwtPlot::TopLegend:
                return Top;
            case QwtPlot::LeftLegend:
                return Left;
            case QwtPlot::RightLegend:
                return Right;
            }
        }
    }
    return Off;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLegendPosition(LegendPos legendPosition)
{
    if (m_pContent)
    {
        switch (legendPosition)
        {
        case Off:
            m_pContent->setLegendPosition(QwtPlot::BottomLegend, false);
            break;
        case Left:
            m_pContent->setLegendPosition(QwtPlot::LeftLegend, true);
            break;
        case Top:
            m_pContent->setLegendPosition(QwtPlot::TopLegend, true);
            break;
        case Right:
            m_pContent->setLegendPosition(QwtPlot::RightLegend, true);
            break;
        case Bottom:
            m_pContent->setLegendPosition(QwtPlot::BottomLegend, true);
            break;
        }
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QStringList Itom1DQwtPlot::getLegendTitles() const
{
    if (m_pContent)
    {
        return m_pContent->m_legendTitles;
    }
    return QStringList();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLegendTitles(const QStringList &legends)
{
    if (m_pContent)
    {
        return m_pContent->setLegendTitles(legends);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuPanner(bool checked)
{
    if (checked)
    {
        m_pActZoomToRect->setChecked(false);
        m_pActMarker->setChecked(false);
        m_pActDrawMode->setChecked(false);
        m_pContent->setPannerEnable(true);
    }
    else
    {
        m_pContent->setPannerEnable(false);
        //(m_pContent)->setMouseTracking(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuZoomer(bool checked)
{
    if (checked)
    {
        m_pActMarker->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActDrawMode->setChecked(false);
        m_pContent->setZoomerEnable(true);
    }
    else
    {
        m_pContent->setZoomerEnable(false);
        //(m_pContent)->setMouseTracking(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuMarkerClick(bool checked)
{
    if (checked)
    {
        m_pActDrawMode->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActZoomToRect->setChecked(false);
    }
    
    m_pContent->setPickerEnable(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuDrawMode(bool checked)
{
    if (checked)
    {
        m_pActPan->setChecked(false);
        m_pActZoomToRect->setChecked(false);
        m_pActMarker->setChecked(false);
        m_pContent->setZoomerEnable(false);
        m_pContent->setState(m_pContent->stateIdle);
    }
    // we need to find out which draw mode we should activate here ...
//    m_pContent->setState(checked ? PlotCanvas::tDraw : PlotCanvas::stateIdle);
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuDrawMode(QAction *action)
{
    m_pActPan->setChecked(false);
    m_pActZoomToRect->setChecked(false);
    m_pActMarker->setChecked(false);
    m_pContent->setZoomerEnable(false);

    m_pActDrawMode->setChecked(true);

    switch (action->data().toInt())
    {
        default:
        case Plot1DWidget::tPoint:
            m_pActDrawMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/marker.png"));
            m_pContent->userInteractionStart(Plot1DWidget::tPoint, 1, 1);
//            connect(m_pContent->m_pMultiPointPicker, SIGNAL(selected(QVector<QPointF>)), this, SLOT(userInteractionEndPt(QVector<QPointF>)));
        break;

        case Plot1DWidget::tLine:
            m_pActDrawMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"));
            m_pContent->userInteractionStart(Plot1DWidget::tLine, 1, 2);
//            connect(m_pContent->m_pMultiPointPicker, SIGNAL(selected(QVector<QPointF>)), this, SLOT(userInteractionEndLine(QVector<QPointF>)));
        break;

        case Plot1DWidget::tRect:
            m_pActDrawMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/rectangle.png"));
            m_pContent->userInteractionStart(Plot1DWidget::tRect, 1, 2);
//            connect(m_pContent->m_pMultiPointPicker, SIGNAL(selected(QRectF)), this, SLOT(userInteractionEndRect(QRectF)));
        break;

        case Plot1DWidget::tEllipse:
            m_pActDrawMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/ellipse.png"));
            m_pContent->userInteractionStart(Plot1DWidget::tEllipse, 1, 2);
//            connect(m_pContent->m_pMultiPointPicker, SIGNAL(selected(QRectF)), this, SLOT(userInteractionEndEllipse(QRectF)));
        break;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuExport()
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
    QString fileName = "plot1D.pdf";
#else
    QString fileName = "plot1D.png";
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
void Itom1DQwtPlot::mnuScaleSetting()
{
    //update m_data to current values
    m_pContent->synchronizeCurrentScaleValues();

    Dialog1DScale *dlg = new Dialog1DScale(*m_data, this);
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData((*m_data));

        bool recalculateBoundaries = false;

        if (m_data->m_valueScaleAuto == true || m_data->m_axisScaleAuto == true)
        {
            recalculateBoundaries = true;
        }

        m_pContent->updateScaleValues(recalculateBoundaries);
    }

    delete dlg;
    dlg = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuGridEnabled(bool checked)
{
    if (m_pContent)
    {
        m_pContent->setGridEnabled(checked);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuParentScaleSetting()
{
    if (m_pContent && m_pContent->m_plotCurveItems.size() > 0)
    {
        const QwtScaleDiv scale = m_pContent->axisScaleDiv(QwtPlot::yLeft);
        ito::AutoInterval bounds(scale.lowerBound(), scale.upperBound());
        /*
        DataObjectSeriesData* seriesData = static_cast<DataObjectSeriesData*>((m_pContent)->m_plotCurveItems[0]->data());
        int cmlpState = seriesData->getCmplxState();
        ito::uint32  minLoc[3], maxLoc[3];
        ito::float64 minVal, maxVal;

        ito::DataObject temp = seriesData->getResampledDataObject();

        if ((temp.getType() != ito::tFloat64) || (temp.getDims() == 0))
            return;*/

        //ito::dObjHelper::minMaxValueFunc<ito::float64>(&temp, minVal, minLoc, maxVal, maxLoc, true, cmlpState);
        
        ito::Channel* dataChannel = getInputChannel("source");
        if (dataChannel && dataChannel->getParent())
        {
            ((ito::AbstractDObjFigure*)(dataChannel->getParent()))->setZAxisInterval(bounds);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuSetMarker(QAction *action)
{
    if (m_pContent && m_pContent->m_plotCurveItems.size() > 0)
    {
        DataObjectSeriesData* seriesData = static_cast<DataObjectSeriesData*>((m_pContent)->m_plotCurveItems[0]->data());

        if (action->text() == QString(tr("To Min-Max")))
        {
//            DataObjectSeriesData::ComplexType cmlpState = seriesData->getCmplxState();

            ito::float64 minVal, maxVal;
            int minLoc, maxLoc;
            if (seriesData->getMinMaxLoc(minVal, maxVal, minLoc, maxLoc) == ito::retOk)
            {
                if (minLoc < maxLoc)
                {
                    m_pContent->setMainPickersToIndex(minLoc, maxLoc, 0);
                }
                else
                {
                    m_pContent->setMainPickersToIndex(maxLoc, minLoc, 0);
                }
            }
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::plotMarkers(const ito::DataObject &coords, QString style, QString id /*= QString::Null()*/, int plane /*= -1*/)
{
    return m_pContent->plotMarkers(&coords, style, id, plane);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::deleteMarkers(int id)
{
    ito::RetVal retVal = m_pContent->deleteMarkers(id);
    if(!retVal.containsWarningOrError()) emit plotItemDeleted(id);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::clearGeometricElements(void)
{
    QList<int> keys = m_data->m_pDrawItems.keys();
    ito::RetVal retVal = ito::retOk;

    for(int i = 0; i < keys.size(); i++)
    {
        retVal += m_pContent->deleteMarkers(keys[i]);
    }
    emit plotItemsDeleted();
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::userInteractionStart(int type, bool start, int maxNrOfPoints /*= -1*/)
{
    m_pActPan->setChecked(false);
    m_pActZoomToRect->setChecked(false);
    m_pActMarker->setChecked(false);
    m_pContent->setZoomerEnable(true);

    switch (type)
    {
        default:
        case Plot1DWidget::tPoint:
            m_pContent->userInteractionStart(type, start, maxNrOfPoints);
        break;

        case Plot1DWidget::tLine:
            m_pContent->userInteractionStart(type, start, maxNrOfPoints * 2);
        break;

        case Plot1DWidget::tRect:
            m_pContent->userInteractionStart(type, start, maxNrOfPoints * 2);
        break;

        case Plot1DWidget::tEllipse:
            m_pContent->userInteractionStart(type, start, maxNrOfPoints * 2);
        break;
    }

}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuCmplxSwitch(QAction *action)
{
    DataObjectSeriesData *seriesData;
    if (m_pContent)
    {
        foreach(QwtPlotCurve *data, m_pContent->m_plotCurveItems)
        {
            seriesData = (DataObjectSeriesData*)data->data();
            if (seriesData)
            {
                if (action->text() == QString(tr("Imag")))
                {
                    seriesData->setCmplxState(DataObjectSeriesData::cmplxImag);
                    m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReImag.png"));
                }
                else if (action->text() == QString(tr("Real")))
                {
                    seriesData->setCmplxState(DataObjectSeriesData::cmplxReal);
                    m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReReal.png"));
                }
                else if (action->text() == QString(tr("Pha")))
                {
                    seriesData->setCmplxState(DataObjectSeriesData::cmplxArg);
                    m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImRePhase.png"));
                }
                else
                {
                    seriesData->setCmplxState(DataObjectSeriesData::cmplxAbs);
                    m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReAbs.png"));
                }
            }
        }

        //if y-axis is set to auto, it is rescaled here with respect to the new limits, else the manual range is kept unchanged.
        m_pContent->setInterval(Qt::YAxis, m_data->m_valueScaleAuto, m_data->m_valueMin, m_data->m_valueMax); //replot is done here 
        
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuMultiRowSwitch(QAction *action)
{
    bool ok;
    int idx = action->data().toInt(&ok);

    if (ok)
    {
        setRowPresentation((Itom1DQwt::tMultiLineMode)idx);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuRGBSwitch(QAction *action)
{
    bool ok;
    int idx = action->data().toInt(&ok);

    if (ok)
    {
        setRGBPresentation(idx);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom1DQwtPlot::getYAxisInterval(void) const
{ 
    m_pContent->synchronizeCurrentScaleValues();
    ito::AutoInterval interval(m_data->m_valueMin, m_data->m_valueMax, m_data->m_valueScaleAuto);
    return interval;
}

//----------------------------------------------------------------------------------------------------------------------------------        
void Itom1DQwtPlot::setYAxisInterval(ito::AutoInterval interval) 
{ 
    m_data->m_valueMin = interval.minimum();
    m_data->m_valueMax = interval.maximum();
    m_data->m_valueScaleAuto = interval.isAuto();
    m_pContent->updateScaleValues( interval.isAuto() ); 
    updatePropertyDock();
}   

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom1DQwtPlot::getXAxisInterval(void) const
{ 
    m_pContent->synchronizeCurrentScaleValues();
    ito::AutoInterval interval(m_data->m_axisMin, m_data->m_axisMax, m_data->m_axisScaleAuto);
    return interval;
}

//----------------------------------------------------------------------------------------------------------------------------------        
void Itom1DQwtPlot::setXAxisInterval(ito::AutoInterval interval) 
{ 
    m_data->m_axisMin = interval.minimum();
    m_data->m_axisMax = interval.maximum();
    m_data->m_axisScaleAuto = interval.isAuto();
    m_pContent->updateScaleValues( interval.isAuto() );
    updatePropertyDock();
}  

//----------------------------------------------------------------------------------------------------------------------------------   
void Itom1DQwtPlot::setPickerText(const QString &coords, const QString &offsets)
{
    m_pLblMarkerCoords->setText(coords);
    m_pLblMarkerOffsets->setText(offsets);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::enableObjectGUIElements(const int mode)
{ 
    switch(mode & 0x0F)
    {
        case 0: // Standard
            m_pActCmplxSwitch->setEnabled(false);
            m_pActCmplxSwitch->setVisible(false);
            m_pActRGBSwitch->setEnabled(false);
            m_pActRGBSwitch->setVisible(false);
            m_pMnuMultiRowSwitch->actions()[3]->setEnabled(true);
            m_pMnuMultiRowSwitch->actions()[4]->setEnabled(true);
            if((mode & 0xF0) == 0x10)
            {
                m_pMnuMultiRowSwitch->actions()[5]->setEnabled(true);
            }
            else
            {
                m_pMnuMultiRowSwitch->actions()[5]->setEnabled(false);
            }

            break;
        case 1: // RGB
            m_pActCmplxSwitch->setEnabled(false);
            m_pActCmplxSwitch->setVisible(false);
            m_pActRGBSwitch->setEnabled(true);
            m_pActRGBSwitch->setVisible(true);
            m_pMnuMultiRowSwitch->actions()[3]->setEnabled(false);
            m_pMnuMultiRowSwitch->actions()[4]->setEnabled(false);
            m_pMnuMultiRowSwitch->actions()[5]->setEnabled(false);

            break;
        case 2: // Complex
            m_pActCmplxSwitch->setEnabled(true);
            m_pActCmplxSwitch->setVisible(true);
            m_pActRGBSwitch->setEnabled(false);
            m_pActRGBSwitch->setVisible(false);
            m_pMnuMultiRowSwitch->actions()[3]->setEnabled(true);
            m_pMnuMultiRowSwitch->actions()[4]->setEnabled(true);
            
            if((mode & 0xF0) == 0x10)
            {
                m_pMnuMultiRowSwitch->actions()[5]->setEnabled(true);
            }
            else
            {
                m_pMnuMultiRowSwitch->actions()[5]->setEnabled(false);
            }
            break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuHome()
{
    m_pContent->home();
}
//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> Itom1DQwtPlot::getDisplayed(void)
{
    if(!m_pContent)
    {
        return QSharedPointer<ito::DataObject>(); 
    }
    else
    {
        return m_pContent->getDisplayed();
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setkeepAspectRatio(const bool &keepAspectEnable)
{
    if (m_pActAspectRatio) //if property is set in designer or by python, the action should represent the current status, too
    {
        m_pActAspectRatio->setChecked(keepAspectEnable);
    }
    mnuActRatio(keepAspectEnable);

    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuActRatio(bool checked)
{
    m_data->m_keepAspect = checked;
    if(m_pContent) m_pContent->configRescaler();

    /*if(m_pActZoomToRect->isChecked()) m_pActZoomToRect->setChecked(false);
    if(m_pActPan->isChecked()) m_pActPan->setChecked(false);
    
    m_pActPan->setEnabled(!checked);
    m_pActZoomToRect->setEnabled(!checked);

    if(m_pContent)
    {
        m_pContent->m_pZoomer->zoom(0);
        m_pContent->setState(Plot1DWidget::stateIdle);
        m_pContent->configRescaler();
    }*/
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resizeEvent ( QResizeEvent * event )
{
    //resizeEvent(event);
    //if(m_pContent) m_pContent->configRescaler();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setEnabledPlotting(const bool &enabled)
{
    m_data->m_enablePlotting = enabled;
    m_pActClearDrawings->setEnabled(enabled);
    m_pActDrawMode->setEnabled(enabled);
    if(m_pActDrawMode->isChecked() && !enabled) m_pActDrawMode->setChecked(enabled);

    updatePropertyDock();

}
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer< ito::DataObject > Itom1DQwtPlot::getGeometricElements()
{
    int ysize = m_data->m_pDrawItems.size();
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
ito::RetVal Itom1DQwtPlot::qvector2DataObject(const ito::DataObject *dstObject)
{
    int ysize = dstObject->getSize(0);

    if(ysize == 0 || ysize < m_data->m_pDrawItems.size())
    {
        return ito::retError;
    }

    int xsize = dstObject->getSize(1);

    cv::Mat *tarMat = (cv::Mat*)(dstObject->get_mdata()[0]);
    ito::float32* rowPtr = tarMat->ptr<ito::float32>(0);
    memset(rowPtr, 0, sizeof(ito::float32) * xsize * ysize);

    QHash<int, DrawItem*>::Iterator it = m_data->m_pDrawItems.begin();

//    for(int y = 0; y < ysize; y++)
//    {
    int y = 0;
    for (; it != m_data->m_pDrawItems.end(); it++)
    {
        rowPtr = tarMat->ptr<ito::float32>(y);
        //if(m_data->m_pDrawItems[y] == NULL)
        if(it.value() == NULL)
        {
            continue;
        }
        //rowPtr[0] = (ito::float32) (m_data->m_pDrawItems[y]->m_idx);
        rowPtr[0] = (ito::float32) (it.value()->m_idx);
        switch (it.value()->m_type)
        {
            case Plot1DWidget::tPoint:
                rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tPoint;
                rowPtr[2] = (ito::float32) (it.value()->x1);
                rowPtr[3] = (ito::float32) (it.value()->y1);
            break;

            case Plot1DWidget::tLine:
                rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tLine;
                rowPtr[2] = (ito::float32) (it.value()->x1);
                rowPtr[3] = (ito::float32) (it.value()->y1);
                rowPtr[5] = (ito::float32) (it.value()->x2);
                rowPtr[6] = (ito::float32) (it.value()->y2);
            break;

            case Plot1DWidget::tRect:
                rowPtr[1] = (ito::float32) ito::PrimitiveContainer::tRectangle;
                rowPtr[2] = (ito::float32) (it.value()->x1);
                rowPtr[3] = (ito::float32) (it.value()->y1);
                rowPtr[5] = (ito::float32) (it.value()->x2);
                rowPtr[6] = (ito::float32) (it.value()->y2);
            break;

            case Plot1DWidget::tEllipse:
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
        y++;
    }

    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setGeometricElements(QSharedPointer< ito::DataObject > geometricElements)
{
    QList<int> keys = m_data->m_pDrawItems.keys();
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
                    ids[geoElement]      = static_cast<ito::float32>(ptrCurScr64[0]);
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
                    ids[geoElement]      = static_cast<ito::float32>(ptrCurScr64[0]);
                    xCoords0[geoElement] = static_cast<ito::float64>(ptrCurScr64[2]);
                    yCoords0[geoElement] = static_cast<ito::float64>(ptrCurScr64[3]);
                    xCoords1[geoElement] = static_cast<ito::float64>(ptrCurScr64[5]);
                    yCoords1[geoElement] = static_cast<ito::float64>(ptrCurScr64[6]);
                }
                else
                {
                    ids[geoElement]                       = ptrCurScr32[0];
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
                    ids[geoElement]      = static_cast<ito::float32>(ptrCurScr64[0]);
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

    updatePropertyDock();

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getSelectedElement(void)const
{
    QHash<int, DrawItem*>::const_iterator it = m_data->m_pDrawItems.begin();
    for (;it != m_data->m_pDrawItems.end(); ++it)        
    {
        if(it.value() != NULL && it.value()->selected() != 0)
        { 
            return it.value()->m_idx;
        }
    }
    return -1;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setSelectedElement(const int idx)
{
    bool replot = false;
    bool failed = idx == -1 ? false : true;
    QHash<int, DrawItem*>::const_iterator it = m_data->m_pDrawItems.begin();
    for (;it != m_data->m_pDrawItems.end(); ++it)        
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

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor Itom1DQwtPlot::getBackgroundColor(void) const
{
    if(m_data) 
    {
        return m_data->m_backgnd;
    }
    else
        return Qt::white;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setBackgroundColor(const QColor newVal)
{
    if(m_data) 
    {
        InternalData* intData = m_data;
        intData->m_backgnd = newVal.rgb() & 0x00FFFFFF;
    }
    if(m_pContent)
    {
        m_pContent->updateColors();
    }

    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
QColor Itom1DQwtPlot::getAxisColor(void) const
{
    if(m_data) 
    {
        return m_data->m_axisColor;
    }
    else
        return Qt::black;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setAxisColor(const QColor newVal)
{
    if(m_data) 
    {
        InternalData* intData = m_data;
        intData->m_axisColor = newVal.rgb() & 0x00FFFFFF;
    }
    if(m_pContent)
    {
        m_pContent->updateColors();
    }

    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
QColor Itom1DQwtPlot::getTextColor(void) const
{
    if(m_data) 
    {
        return m_data->m_textColor;
    }
    else
        return Qt::black;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setTextColor(const QColor newVal)
{
    if(m_data) 
    {
        InternalData* intData = m_data;
        intData->m_textColor = newVal.rgb() & 0x00FFFFFF;
    }
    if(m_pContent)
    {
        m_pContent->updateColors();
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::exportCanvas(const bool copyToClipboardNotFile, const QString &fileName, QSizeF curSize /*= QSizeF(0.0,0.0)*/, const int resolution /*= 300*/)
{
    if(!m_data)
    {
        return ito::RetVal(ito::retError, 0, tr("Export image failed, internal setting handle not initilized").toLatin1().data());
    }
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
    //qreal linewidth = m_pContent->m_lineWidth;
    m_pContent->setAutoFillBackground( true );
    m_pContent->setPalette( Qt::white );
    m_pContent->setCanvasBackground(Qt::white);
    qreal resFaktor = 1.0;

    if(copyToClipboardNotFile)
    {
        m_pContent->statusBarMessage(tr("copy current view to clipboard..."));

        resFaktor = resFaktor = resolution / 72.0 + 0.5;;
        resFaktor = resFaktor < 1.0 ? 1.0 : resFaktor;
        //m_pContent->setLineWidth(linewidth * resFaktor);
    }
    m_pContent->replot();

    QwtPlotRenderer renderer;

    // flags to make the document look like the widget
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    //renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true); //deprecated in qwt 6.1.0
    
    if(copyToClipboardNotFile)
    {
        m_pContent->statusBarMessage(tr("copy current view to clipboard..."));
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
        renderer.renderDocument((m_pContent), fileName, curSize, resolution);
    }

    m_pContent->setPalette( curPalette);
    m_pContent->setCanvasBackground( curBrush);
    //m_pContent->setLineWidth(linewidth);
    m_pContent->replot();
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::copyToClipBoard()
{
    return exportCanvas(true, "");
}

//----------------------------------------------------------------------------------------------------------------------------------
QPixmap Itom1DQwtPlot::renderToPixMap(const int xsize, const int ysize, const int resolution) 
{

    QSizeF curSize(xsize, ysize);
    if(!m_pContent || !m_data)
    {
        QSize myRect(curSize.width(), curSize.height());
        QPixmap destinationImage(myRect);
        return destinationImage;
    }

    if(curSize.height() == 0 || curSize.width() == 0)
    {
        curSize = ((Plot1DWidget *)m_pContent)->size();
    }
    //qreal linewidth = m_pContent->m_lineWidth;
    qreal resFaktor = resolution / 72.0 + 0.5;
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
    //m_pContent->setLineWidth(linewidth*resFaktor);
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
    //m_pContent->setLineWidth(linewidth);
    m_pContent->replot();

    return destinationImage;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::setPicker(const QVector<int> &pxCords)
{
    if(!m_pContent)
    {
        return ito::RetVal(ito::retError, 0, tr("Set picker failed, canvas handle not initilized").toLatin1().data());
    }
    return m_pContent->setPicker(pxCords);
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::setPicker(const QVector<float> &physCords)
{
    if(!m_pContent)
    {
        return ito::RetVal(ito::retError, 0, tr("Set picker failed, canvas handle not initilized").toLatin1().data());
    }
    return m_pContent->setPicker(physCords);
}
//----------------------------------------------------------------------------------------------------------------------------------
QVector<int> Itom1DQwtPlot::getPickerPixel() const
{
    if(!m_pContent)
    {
        emit m_pContent->statusBarMessage(tr("Get picker failed, canvas handle not initilized."), 12000 );
    }
    return m_pContent->getPickerPixel();
}
//----------------------------------------------------------------------------------------------------------------------------------
QVector<float> Itom1DQwtPlot::getPickerPhys() const
{
    if(!m_pContent)
    {
        emit m_pContent->statusBarMessage(tr("Get picker failed, canvas handle not initilized."), 12000 );
    }
    return m_pContent->getPickerPhys();
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::setGeometricElementLabel(int id, QString label)
{
    if(!m_data) 
    {
        return ito::RetVal(ito::retError, 0, tr("Could not access internal data structur").toLatin1().data());
    }

    InternalData* pData = m_data;

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
ito::RetVal Itom1DQwtPlot::setGeometricElementLabelVisible(int id, bool setVisible)
{
    if(!m_data) 
    {
        return ito::RetVal(ito::retError, 0, tr("Could not access internal data structur").toLatin1().data());
    }

    InternalData* pData = m_data;

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
void Itom1DQwtPlot::setPickerLabelVisible(const bool state)
{
    if(m_data) 
    {
        m_data->m_pickerLabelVisible = state;
    }
    if(m_pContent)
    {
        ((Plot1DWidget*)m_pContent)->updatePickerStyle();
    }
    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
bool Itom1DQwtPlot::getPickerLabelVisible() const
{
    if(!m_data) 
    {
        return false;
    }
    return m_data->m_pickerLabelVisible;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerLabelOrientation(const Qt::Orientation val)
{
    if(m_data) 
    {
        m_data->m_pickerLabelOrientation = val;
    }
    if(m_pContent)
    {
        ((Plot1DWidget*)m_pContent)->updatePickerStyle();
    }
    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
Qt::Orientation Itom1DQwtPlot::getPickerLabelOrientation() const
{
    if(!m_data) 
    {
        return Qt::Horizontal;
    }
    return m_data->m_pickerLabelOrientation;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerLabelAlignment(const Qt::Alignment val)
{
    if(m_data) 
    {
        m_data->m_pickerLabelAlignment = val;
    }
    if(m_pContent)
    {
        ((Plot1DWidget*)m_pContent)->updatePickerStyle();
    }
    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
Qt::Alignment Itom1DQwtPlot::getPickerLabelAlignment()const
{
    if(!m_data) 
    {
        return Qt::AlignRight;
    }
    return m_data->m_pickerLabelAlignment;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setPickerType(const Itom1DQwt::tPlotPickerType val)
{
    if(m_data) 
    {
        m_data->m_pickerType = val;
    }
    if(m_pContent)
    {
        ((Plot1DWidget*)m_pContent)->updatePickerStyle();
    }
    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwt::tPlotPickerType Itom1DQwtPlot::getPickerType() const
{
    if(!m_data) 
    {
        return Itom1DQwt::DefaultMarker;
    }
    return m_data->m_pickerType;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setBaseLine(const qreal val)
{
    if(m_pContent) 
    {
        return m_pContent->setBaseLine(val);
    }
    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
qreal Itom1DQwtPlot::getBaseLine() const
{
    if(m_pContent) 
    {
        return m_pContent->m_baseLine;
    }
    return 0.0;
}
//----------------------------------------------------------------------------------------------------------------------------------
QColor Itom1DQwtPlot::getCurveFillColor() const
{
    if(m_pContent) 
    {
        return m_pContent->m_filledColor;
    }
    return QColor::Invalid;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setCurveFillColor(const QColor val)
{
    if(m_pContent) 
    {
        m_pContent->m_filledColor = val;
        m_pContent->m_filledColor.setAlpha(m_pContent->m_fillCurveAlpa);
        return m_pContent->setCurveFilled();
    }
    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetCurveFillColor()
{
    setCurveFillColor(QColor::Invalid);
}
//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwt::tFillCurveStyle Itom1DQwtPlot::getCurveFilled() const
{
    if(m_pContent) 
    {

        return m_pContent->m_curveFilled;
    }
    return Itom1DQwt::NoCurveFill;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setCurveFilled(const Itom1DQwt::tFillCurveStyle state)
{
    if(m_pContent) 
    {
        m_pContent->m_curveFilled = state;
        m_pContent->setCurveFilled();
    }
    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setCurveFillAlpha(const int val)
{
    if(m_pContent) 
    {
        m_pContent->m_fillCurveAlpa = cv::saturate_cast<ito::uint8>(val);
        m_pContent->m_filledColor.setAlpha(m_pContent->m_fillCurveAlpa);
        m_pContent->setCurveFilled();
    }
    updatePropertyDock();
}
//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getCurveFillAlpha() const
{
    if(m_pContent) 
    {
        return m_pContent->m_fillCurveAlpa;
    }
    return 128;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuActSendCurrentToWorkspace()
{
    bool ok;
    QString varname = QInputDialog::getText(this, tr("Current to workspace"), tr("Indicate the python variable name for the currently visible object"), QLineEdit::Normal, "displayed_line", &ok);
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
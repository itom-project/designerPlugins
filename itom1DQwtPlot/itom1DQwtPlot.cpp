/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut f�r Technische Optik (ITO), 
   Universit�t Stuttgart, Germany 
 
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

    m_data = new InternalData;
    ((InternalData*)m_data)->m_pDrawItems.clear();
    ((InternalData*)m_data)->m_autoAxisLabel = true;
    ((InternalData*)m_data)->m_autoValueLabel = true;
    ((InternalData*)m_data)->m_valueScaleAuto = true;
    ((InternalData*)m_data)->m_dataType = ito::tFloat64;
    ((InternalData*)m_data)->m_valueMin = -127.0;
    ((InternalData*)m_data)->m_valueMax = 128.0;
    ((InternalData*)m_data)->m_axisScaleAuto = true;
    ((InternalData*)m_data)->m_forceValueParsing = false;

    m_pContent = new Plot1DWidget(contextMenu, (InternalData*)m_data, this);

    connect(((Plot1DWidget *)m_pContent), SIGNAL(statusBarClear()), (QObject*)statusBar(), SLOT(clearMessage()));
    connect(((Plot1DWidget *)m_pContent), SIGNAL(statusBarMessage(QString,int)), (QObject*)statusBar(), SLOT(showMessage(QString,int)));

    ((Plot1DWidget *)m_pContent)->setObjectName("canvasWidget");

    connect(((Plot1DWidget *)m_pContent), SIGNAL(setMarkerText(const QString &, const QString &)), this, SLOT(setMarkerText(const QString &, const QString &)));

    setFocus();
    setCentralWidget(((Plot1DWidget *)m_pContent));
    ((Plot1DWidget *)m_pContent)->setFocus();

    QMenu *menuView = new QMenu(tr("View"), this);
    menuView->addAction(m_pActHome);
    menuView->addAction(m_pActPan);
    menuView->addAction(m_pActZoomToRect);
    menuView->addAction(m_pActAspectRatio);
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
    m_pActPan(NULL),
    m_pActZoomToRect(NULL),
    m_pActMarker(NULL),
    m_pMnuSetMarker(NULL),
    m_pActSetMarker(NULL),
    m_pActCmplxSwitch(NULL),
    m_pMnuCmplxSwitch(NULL),
    m_pLblMarkerOffsets(NULL),
    m_pLblMarkerCoords(NULL),
    m_pActAspectRatio(NULL),
    m_pActClearDrawings(NULL),
    m_pActDrawMode(NULL),
    m_pMnuDrawMode(NULL),
    m_pDrawModeActGroup(NULL),
    m_pActProperties(NULL)
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
    m_pActPan(NULL),
    m_pActZoomToRect(NULL),
    m_pActMarker(NULL),
    m_pMnuSetMarker(NULL),
    m_pActSetMarker(NULL),
    m_pActCmplxSwitch(NULL),
    m_pMnuCmplxSwitch(NULL),
    m_pLblMarkerOffsets(NULL),
    m_pLblMarkerCoords(NULL),
    m_pActAspectRatio(NULL),
    m_pActClearDrawings(NULL),
    m_pActDrawMode(NULL),
    m_pMnuDrawMode(NULL),
    m_pDrawModeActGroup(NULL),
    m_pActProperties(NULL)
{
    constructor();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::~Itom1DQwtPlot()
{
    if (m_pMnuCmplxSwitch != NULL)
    {
        m_pMnuCmplxSwitch->deleteLater();
        m_pMnuCmplxSwitch = NULL;
    }
    ((Plot1DWidget *)m_pContent)->deleteLater();
//    ((Plot1DWidget *)m_pContent) = NULL;
    m_pContent = NULL;
    if (m_data)
        delete ((InternalData*)m_data);
    if (m_pMnuSetMarker)
        delete(m_pMnuSetMarker);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::init()
{ 
    return ((Plot1DWidget *)m_pContent)->init(); 
} //called when api-pointers are transmitted, directly after construction

//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getGeometricElementsCount() const 
{ 
    return ((InternalData*)m_data)->m_pDrawItems.size();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom1DQwtPlot::getkeepAspectRatio(void) const 
{
    return ((InternalData*)m_data)->m_keepAspect;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom1DQwtPlot::getEnabledPlotting(void) const 
{
    return ((InternalData*)m_data)->m_enablePlotting;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::createActions()
{
    QAction *a = NULL;

    //m_actHome
    m_pActHome = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"), tr("Home"), this);
    a->setObjectName("actHome");
    a->setToolTip(tr("Reset original view"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuHome()));
 
    //m_actSave
    m_pActSave = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("Save..."), this);
    a->setObjectName("actSave");
    a->setToolTip(tr("Export current view..."));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuExport()));

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
    m_pActForward = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/forward.png"), tr("forward"), this);
    a->setObjectName("actionForward");
    a->setEnabled(false);
    a->setToolTip(tr("Forward to next line"));

    //m_actBack
    m_pActBack = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/back.png"), tr("back"), this);
    a->setObjectName("actionBack");
    a->setEnabled(false);
    a->setToolTip(tr("Back to previous line"));

    //m_actPan
    m_pActPan = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), tr("move"), this);
    a->setObjectName("actionPan");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Pan axes with left mouse, zoom with right"));
    connect(a, SIGNAL(toggled(bool)), this, SLOT(mnuPanner(bool)));

    //m_actZoomToRect
    m_pActZoomToRect = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"), tr("zoom to rectangle"), this);
    a->setObjectName("actionZoomToRect");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Zoom to rectangle"));
    connect(a, SIGNAL(toggled(bool)), this, SLOT(mnuZoomer(bool)));

    //m_pActAspectRatio
    m_pActAspectRatio = a = new QAction(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio11.png"), tr("lock aspect ratio"), this);
    a->setObjectName("actRatio");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Toggle fixed / variable aspect ration between axis x and y"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActRatio(bool)));

    //m_actMarker
    m_pActMarker = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/marker.png"), tr("marker"), this);
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
    m_pActClearDrawings = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/editDelete.png"), tr("clear Marker"), this);
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

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::applyUpdate()
{
    QVector<QPointF> bounds = getBounds();

    if ((ito::DataObject*)m_pInput["source"]->getVal<void*>())
    {
        m_pOutput["displayed"]->copyValueFrom(m_pInput["source"]);
        // why "source" is used here and not "displayed" .... ck 05/15/2013
        ((Plot1DWidget *)m_pContent)->refreshPlot((ito::DataObject*)m_pInput["source"]->getVal<char*>(), bounds);

        ito::Channel* dataChannel = getInputChannel("source");
        m_pRescaleParent->setVisible(dataChannel && dataChannel->getParent());
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setSource(QSharedPointer<ito::DataObject> source)
{
    ((InternalData*)m_data)->m_forceValueParsing = true; //recalculate boundaries since content of data object may have changed
    AbstractDObjFigure::setSource(source);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom1DQwtPlot::getContextMenuEnabled() const
{
    if (((Plot1DWidget *)m_pContent)) return (((Plot1DWidget *)m_pContent))->m_showContextMenu;
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setContextMenuEnabled(bool show)
{
    if (((Plot1DWidget *)m_pContent)) (((Plot1DWidget *)m_pContent))->m_showContextMenu = show;
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
    if (((InternalData*)m_data)->m_autoTitle)
    {
        return "<auto>";
    }
    return ((InternalData*)m_data)->m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setTitle(const QString &title)
{
    if (title == "<auto>")
    {
        ((InternalData*)m_data)->m_autoTitle = true;
    }
    else
    {
        ((InternalData*)m_data)->m_autoTitle = false;
        ((InternalData*)m_data)->m_title = title;
    }

    if (((Plot1DWidget *)m_pContent)) ((Plot1DWidget *)m_pContent)->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetTitle()
{
    ((InternalData*)m_data)->m_autoTitle = true;
    if (((Plot1DWidget *)m_pContent)) ((Plot1DWidget *)m_pContent)->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom1DQwtPlot::getAxisLabel() const
{
    if (((InternalData*)m_data)->m_autoAxisLabel)
    {
        return "<auto>";
    }
    return ((InternalData*)m_data)->m_axisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setAxisLabel(const QString &label)
{
    if (label == "<auto>")
    {
        ((InternalData*)m_data)->m_autoAxisLabel = true;
    }
    else
    {
        ((InternalData*)m_data)->m_autoAxisLabel = false;
        ((InternalData*)m_data)->m_axisLabel = label;
    }
    if (((Plot1DWidget *)m_pContent)) ((Plot1DWidget *)m_pContent)->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetAxisLabel()
{
    ((InternalData*)m_data)->m_autoAxisLabel = true;
    if (((Plot1DWidget *)m_pContent)) ((Plot1DWidget *)m_pContent)->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom1DQwtPlot::getValueLabel() const
{
    if (((InternalData*)m_data)->m_autoValueLabel)
    {
        return "<auto>";
    }
    return ((InternalData*)m_data)->m_valueLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setValueLabel(const QString &label)
{
    if (label == "<auto>")
    {
        ((InternalData*)m_data)->m_autoValueLabel = true;
    }
    else
    {
        ((InternalData*)m_data)->m_autoValueLabel = false;
        ((InternalData*)m_data)->m_valueLabel = label;
    }
    if (((Plot1DWidget *)m_pContent)) ((Plot1DWidget *)m_pContent)->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetValueLabel()
{
    ((InternalData*)m_data)->m_autoValueLabel = true;
    if (((Plot1DWidget *)m_pContent)) ((Plot1DWidget *)m_pContent)->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom1DQwtPlot::getTitleFont(void) const
{
    if (((Plot1DWidget *)m_pContent))
    {
        return ((Plot1DWidget *)m_pContent)->titleLabel()->font();
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setTitleFont(const QFont &font)
{
    if (((Plot1DWidget *)m_pContent))
    {
        ((Plot1DWidget *)m_pContent)->titleLabel()->setFont(font);
        //((Plot1DWidget *)m_pContent)->replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom1DQwtPlot::getLabelFont(void) const
{
    if (((Plot1DWidget *)m_pContent))
    {
        QwtText t = ((Plot1DWidget *)m_pContent)->axisWidget(QwtPlot::xBottom)->title();
        return t.font();
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setLabelFont(const QFont &font)
{
    if (((Plot1DWidget *)m_pContent))
    {
        QwtText title;
        title = ((Plot1DWidget *)m_pContent)->axisWidget(QwtPlot::xBottom)->title();
        title.setFont(font);
        ((Plot1DWidget *)m_pContent)->axisWidget(QwtPlot::xBottom)->setTitle(title);

        title = ((Plot1DWidget *)m_pContent)->axisWidget(QwtPlot::yLeft)->title();
        title.setFont(font);
        ((Plot1DWidget *)m_pContent)->axisWidget(QwtPlot::yLeft)->setTitle(title);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom1DQwtPlot::getAxisFont(void) const
{
    if (((Plot1DWidget *)m_pContent))
    {
        return ((Plot1DWidget *)m_pContent)->axisFont(QwtPlot::xBottom);
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setAxisFont(const QFont &font)
{
    if (((Plot1DWidget *)m_pContent))
    {
        ((Plot1DWidget *)m_pContent)->setAxisFont(QwtPlot::xBottom, font);
        ((Plot1DWidget *)m_pContent)->setAxisFont(QwtPlot::yLeft, font);
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuPanner(bool checked)
{
    if (checked)
    {
        m_pActZoomToRect->setChecked(false);
        m_pActMarker->setChecked(false);
        m_pActDrawMode->setChecked(false);
        ((Plot1DWidget *)m_pContent)->setPannerEnable(true);
    }
    else
    {
        ((Plot1DWidget *)m_pContent)->setPannerEnable(false);
        //(((Plot1DWidget *)m_pContent))->setMouseTracking(false);
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
        ((Plot1DWidget *)m_pContent)->setZoomerEnable(true);
    }
    else
    {
        ((Plot1DWidget *)m_pContent)->setZoomerEnable(false);
        //(((Plot1DWidget *)m_pContent))->setMouseTracking(false);
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
    
    ((Plot1DWidget *)m_pContent)->setPickerEnable(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuDrawMode(bool checked)
{
    if (checked)
    {
        m_pActPan->setChecked(false);
        m_pActZoomToRect->setChecked(false);
        m_pActMarker->setChecked(false);
        ((Plot1DWidget *)m_pContent)->setZoomerEnable(false);
        ((Plot1DWidget *)m_pContent)->setState(((Plot1DWidget *)m_pContent)->stateIdle);
    }
    // we need to find out which draw mode we should activate here ...
//    ((Plot1DWidget *)m_pContent)->setState(checked ? PlotCanvas::tDraw : PlotCanvas::stateIdle);
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuDrawMode(QAction *action)
{
    m_pActPan->setChecked(false);
    m_pActZoomToRect->setChecked(false);
    m_pActMarker->setChecked(false);
    ((Plot1DWidget *)m_pContent)->setZoomerEnable(false);

    m_pActDrawMode->setChecked(true);

    switch (action->data().toInt())
    {
        default:
        case Plot1DWidget::tPoint:
            m_pActDrawMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/marker.png"));
            ((Plot1DWidget *)m_pContent)->userInteractionStart(Plot1DWidget::tPoint, 1, 1);
//            connect(((Plot1DWidget *)m_pContent)->m_pMultiPointPicker, SIGNAL(selected(QVector<QPointF>)), this, SLOT(userInteractionEndPt(QVector<QPointF>)));
        break;

        case Plot1DWidget::tLine:
            m_pActDrawMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"));
            ((Plot1DWidget *)m_pContent)->userInteractionStart(Plot1DWidget::tLine, 1, 2);
//            connect(((Plot1DWidget *)m_pContent)->m_pMultiPointPicker, SIGNAL(selected(QVector<QPointF>)), this, SLOT(userInteractionEndLine(QVector<QPointF>)));
        break;

        case Plot1DWidget::tRect:
            m_pActDrawMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/rectangle.png"));
            ((Plot1DWidget *)m_pContent)->userInteractionStart(Plot1DWidget::tRect, 1, 2);
//            connect(((Plot1DWidget *)m_pContent)->m_pMultiPointPicker, SIGNAL(selected(QRectF)), this, SLOT(userInteractionEndRect(QRectF)));
        break;

        case Plot1DWidget::tEllipse:
            m_pActDrawMode->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/ellipse.png"));
            ((Plot1DWidget *)m_pContent)->userInteractionStart(Plot1DWidget::tEllipse, 1, 2);
//            connect(((Plot1DWidget *)m_pContent)->m_pMultiPointPicker, SIGNAL(selected(QRectF)), this, SLOT(userInteractionEndEllipse(QRectF)));
        break;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuExport()
{
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
        bool abort = true;

        QSizeF curSize = ((Plot1DWidget *)m_pContent)->size();
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

        QFileInfo fi(fileName);
        saveDefaultPath = fi.path();

        exportCanvas(false, fileName, curSize, resolution);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuScaleSetting()
{
    Dialog1DScale *dlg = new Dialog1DScale(*((InternalData*)m_data), this);
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData((*(InternalData*)m_data));

         bool recalculateBoundaries = false;

        if (((InternalData*)m_data)->m_valueScaleAuto == true || ((InternalData*)m_data)->m_axisScaleAuto == true)
        {
            recalculateBoundaries = true;
        }

        ((Plot1DWidget *)m_pContent)->updateScaleValues(recalculateBoundaries);
    }

    delete dlg;
    dlg = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuParentScaleSetting()
{
    if (((Plot1DWidget *)m_pContent) && ((Plot1DWidget *)m_pContent)->m_plotCurveItems.size() > 0)
    {
        const QwtScaleDiv scale = ((Plot1DWidget *)m_pContent)->axisScaleDiv(QwtPlot::yLeft);
        QPointF bounds = QPointF(scale.lowerBound(), scale.upperBound());
        /*
        DataObjectSeriesData* seriesData = static_cast<DataObjectSeriesData*>((((Plot1DWidget *)m_pContent))->m_plotCurveItems[0]->data());
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
    if (((Plot1DWidget *)m_pContent) && ((Plot1DWidget *)m_pContent)->m_plotCurveItems.size() > 0)
    {
        DataObjectSeriesData* seriesData = static_cast<DataObjectSeriesData*>((((Plot1DWidget *)m_pContent))->m_plotCurveItems[0]->data());

        if (action->text() == QString(tr("To Min-Max")))
        {
//            DataObjectSeriesData::ComplexType cmlpState = seriesData->getCmplxState();

            ito::float64 minVal, maxVal;
            int minLoc, maxLoc;
            if (seriesData->getMinMaxLoc(minVal, maxVal, minLoc, maxLoc) == ito::retOk)
            {
                if (minLoc < maxLoc)
                {
                    ((Plot1DWidget *)m_pContent)->setMainMarkersToIndex(minLoc, maxLoc, 0);
                }
                else
                {
                    ((Plot1DWidget *)m_pContent)->setMainMarkersToIndex(maxLoc, minLoc, 0);
                }
            }
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::plotMarkers(const ito::DataObject &coords, QString style, QString id /*= QString::Null()*/, int plane /*= -1*/)
{
    return ((Plot1DWidget *)m_pContent)->plotMarkers(&coords, style, id, plane);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::deleteMarkers(int id)
{
    ito::RetVal retVal = ((Plot1DWidget *)m_pContent)->deleteMarkers(id);
    if(!retVal.containsWarningOrError()) emit plotItemDeleted(id);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::clearGeometricElements(void)
{
    QList<int> keys = ((InternalData*)m_data)->m_pDrawItems.keys();
    ito::RetVal retVal = ito::retOk;

    for(int i = 0; i < keys.size(); i++)
    {
        retVal += ((Plot1DWidget *)m_pContent)->deleteMarkers(keys[i]);
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
    ((Plot1DWidget *)m_pContent)->setZoomerEnable(true);

    switch (type)
    {
        default:
        case Plot1DWidget::tPoint:
            ((Plot1DWidget *)m_pContent)->userInteractionStart(type, start, maxNrOfPoints);
        break;

        case Plot1DWidget::tLine:
            ((Plot1DWidget *)m_pContent)->userInteractionStart(type, start, maxNrOfPoints * 2);
        break;

        case Plot1DWidget::tRect:
            ((Plot1DWidget *)m_pContent)->userInteractionStart(type, start, maxNrOfPoints * 2);
        break;

        case Plot1DWidget::tEllipse:
            ((Plot1DWidget *)m_pContent)->userInteractionStart(type, start, maxNrOfPoints * 2);
        break;
    }

}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuCmplxSwitch(QAction *action)
{
    DataObjectSeriesData *seriesData;
    if (((Plot1DWidget *)m_pContent))
    {
        foreach(QwtPlotCurve *data, ((Plot1DWidget *)m_pContent)->m_plotCurveItems)
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

        ((Plot1DWidget *)m_pContent)->setInterval(Qt::ZAxis, true, 0, 0); //replot is done here
        
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF Itom1DQwtPlot::getYAxisInterval(void) const
{ 
    QPointF interval(((InternalData*)m_data)->m_valueMin, ((InternalData*)m_data)->m_valueMax);
    return interval;
}

//----------------------------------------------------------------------------------------------------------------------------------        
void Itom1DQwtPlot::setYAxisInterval(QPointF interval) 
{ 
    if (interval.isNull())
    {
        ((Plot1DWidget *)m_pContent)->setInterval(Qt::YAxis, true, 0.0, 0.0);
    }
    else
    {
        ((Plot1DWidget *)m_pContent)->setInterval(Qt::YAxis, false, interval.x(), interval.y());
    }
    return; 
}   

//----------------------------------------------------------------------------------------------------------------------------------   
void Itom1DQwtPlot::setMarkerText(const QString &coords, const QString &offsets)
{
    m_pLblMarkerCoords->setText(coords);
    m_pLblMarkerOffsets->setText(offsets);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::enableComplexGUI(const bool checked)
{ 
    m_pActCmplxSwitch->setEnabled(checked);
    m_pActCmplxSwitch->setVisible(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuHome()
{
    ((Plot1DWidget *)m_pContent)->m_pZoomer->zoom(0);
}
//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> Itom1DQwtPlot::getDisplayed(void)
{
    if(!((Plot1DWidget *)m_pContent))
    {
        return QSharedPointer<ito::DataObject>(); 
    }

    ito::DataObject dataObjectOut;

    if (((Plot1DWidget *)m_pContent) && ((Plot1DWidget *)m_pContent)->m_plotCurveItems.size() > 0)
    {
        DataObjectSeriesData* seriesData = static_cast<DataObjectSeriesData*>((((Plot1DWidget *)m_pContent))->m_plotCurveItems[0]->data());

        if(seriesData->size() < 2)
        {
            return QSharedPointer<ito::DataObject>();
        }

        ito::float64 length = 0.0;

        if(seriesData->floatingPointValues())
        {
            dataObjectOut = ito::DataObject(1, (int)seriesData->size(), ito::tFloat64);
            ito::float64* rowPtr = ((cv::Mat*)dataObjectOut.get_mdata()[0])->ptr<ito::float64>();
            QPointF curPos;
            for(size_t i = 0; i < seriesData->size(); i++)
            {
                curPos = seriesData->sample(i);
                length += curPos.x();
                rowPtr[i] = curPos.y();
            }
        }
        else
        {
            dataObjectOut = ito::DataObject(1, (int)seriesData->size(), ito::tInt32);
            ito::int32* rowPtr = ((cv::Mat*)dataObjectOut.get_mdata()[0])->ptr<ito::int32>();
            QPointF curPos;            
            for(size_t i = 0; i < seriesData->size(); i++)
            {
                curPos = seriesData->sample(i);
                length += curPos.x();
                rowPtr[i] = (ito::int32)(curPos.y());
            }
        }

        dataObjectOut.setAxisScale(1, length / (seriesData->size() - 1));

        dataObjectOut.setAxisUnit(1, seriesData->getDObjAxisLabel().toLatin1().data());
        dataObjectOut.setValueUnit(seriesData->getDObjValueLabel().toLatin1().data());

    }

    return QSharedPointer<ito::DataObject>(new ito::DataObject(dataObjectOut));
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setkeepAspectRatio(const bool &keepAspectEnable)
{
    if (m_pActAspectRatio) //if property is set in designer or by python, the action should represent the current status, too
    {
        m_pActAspectRatio->setChecked(keepAspectEnable);
    }
    mnuActRatio(keepAspectEnable);
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuActRatio(bool checked)
{
    ((InternalData*)m_data)->m_keepAspect = checked;
    if(((Plot1DWidget *)m_pContent)) ((Plot1DWidget *)m_pContent)->configRescaler();

    if(m_pActZoomToRect->isChecked()) m_pActZoomToRect->setChecked(false);
    if(m_pActPan->isChecked()) m_pActPan->setChecked(false);
    
    m_pActPan->setEnabled(!checked);
    m_pActZoomToRect->setEnabled(!checked);

    if(((Plot1DWidget *)m_pContent))
    {
        ((Plot1DWidget *)m_pContent)->m_pZoomer->zoom(0);
        ((Plot1DWidget *)m_pContent)->setState(Plot1DWidget::stateIdle);
        ((Plot1DWidget *)m_pContent)->configRescaler();
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resizeEvent ( QResizeEvent * event )
{
    //resizeEvent(event);
    if(((Plot1DWidget *)m_pContent)) ((Plot1DWidget *)m_pContent)->configRescaler();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setEnabledPlotting(const bool &enabled)
{
    ((InternalData*)m_data)->m_enablePlotting = enabled;
    m_pActClearDrawings->setEnabled(enabled);
    m_pActDrawMode->setEnabled(enabled);
    if(m_pActDrawMode->isChecked() && !enabled) m_pActDrawMode->setChecked(enabled);

}
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer< ito::DataObject > Itom1DQwtPlot::getGeometricElements()
{
    int ysize = ((InternalData*)m_data)->m_pDrawItems.size();
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

    if(ysize == 0 || ysize < ((InternalData*)m_data)->m_pDrawItems.size())
    {
        return ito::retError;
    }

    int xsize = dstObject->getSize(1);

    cv::Mat *tarMat = (cv::Mat*)(dstObject->get_mdata()[0]);
    ito::float32* rowPtr = tarMat->ptr<ito::float32>(0);
    memset(rowPtr, 0, sizeof(ito::float32) * xsize * ysize);

    QHash<int, DrawItem*>::Iterator it = ((InternalData*)m_data)->m_pDrawItems.begin();

//    for(int y = 0; y < ysize; y++)
//    {
    int y = 0;
    for (; it != ((InternalData*)m_data)->m_pDrawItems.end(); it++)
    {
        rowPtr = tarMat->ptr<ito::float32>(y);
        //if(((InternalData*)m_data)->m_pDrawItems[y] == NULL)
        if(it.value() == NULL)
        {
            continue;
        }
        //rowPtr[0] = (ito::float32) (((InternalData*)m_data)->m_pDrawItems[y]->m_idx);
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
    QList<int> keys = ((InternalData*)m_data)->m_pDrawItems.keys();
    ito::RetVal retVal = ito::retOk;

    for(int i = 0; i < keys.size(); i++)
    {
        retVal += ((Plot1DWidget *)m_pContent)->deleteMarkers(keys[i]);
    }
    emit plotItemsDeleted();

    if(geometricElements.isNull() || 
       geometricElements->getDims() != 2 || 
       (geometricElements->getType() != ito::tFloat32 && geometricElements->getType() != ito::tFloat64) ||
       geometricElements->getSize(1) < PRIM_ELEMENTLENGTH)
    {
        ((Plot1DWidget *)m_pContent)->statusBarMessage(tr("Element container did not match criteria, 2 dims, elements x 11, floating point value"), 600 );
        plotItemsFinished(0, true);
        return;
    }

    if(geometricElements->getSize(0) == 0)
    {
        ((Plot1DWidget *)m_pContent)->statusBarMessage(tr("Deleted element, new element list was empty"), 600 );
        ((Plot1DWidget *)m_pContent)->replot();
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
                ((Plot1DWidget *)m_pContent)->statusBarMessage(tr("Could not convert elements, type undefined"), 600 );
                return;    
        }

    }

    ito::RetVal retval = ((Plot1DWidget *)m_pContent)->plotMarkers(&coords, "b", "", 0);

    ((Plot1DWidget *)m_pContent)->replot();

    if(retval.containsError())
    {
        ((Plot1DWidget *)m_pContent)->statusBarMessage(tr("Could not set elements"), 600 );
        plotItemsFinished(0, true);
        return;    
    }

    plotItemsFinished(0, false);
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
int Itom1DQwtPlot::getSelectedElement(void)const
{
    QHash<int, DrawItem*>::const_iterator it = ((InternalData*)m_data)->m_pDrawItems.begin();
    for (;it != ((InternalData*)m_data)->m_pDrawItems.end(); ++it)        
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
    QHash<int, DrawItem*>::const_iterator it = ((InternalData*)m_data)->m_pDrawItems.begin();
    for (;it != ((InternalData*)m_data)->m_pDrawItems.end(); ++it)        
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

    if(((Plot1DWidget *)m_pContent))
    {
        if(replot) ((Plot1DWidget *)m_pContent)->replot();
        if(failed) emit ((Plot1DWidget *)m_pContent)->statusBarMessage(tr("Could not set active element, index out of range."), 12000 );
    }
    return;
}

ito::RetVal Itom1DQwtPlot::exportCanvas(const bool exportType, const QString &fileName, QSizeF curSize, const int resolution)
{
    if(!m_pContent)
    {
        return ito::RetVal(ito::retError, 0, tr("Export image failed, canvas handle not initilized").toLatin1().data());
    }
    if(curSize.height() == 0 || curSize.width() == 0)
    {
        curSize = ((Plot1DWidget *)m_pContent)->size();
    }
    QBrush curBrush = ((Plot1DWidget *)m_pContent)->canvasBackground();

    QPalette curPalette = ((Plot1DWidget *)m_pContent)->palette();

    ((Plot1DWidget *)m_pContent)->setAutoFillBackground( true );
    ((Plot1DWidget *)m_pContent)->setPalette( Qt::white );
    ((Plot1DWidget *)m_pContent)->setCanvasBackground(Qt::white);    

    ((Plot1DWidget *)m_pContent)->replot();

    QwtPlotRenderer renderer;

    // flags to make the document look like the widget
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    //renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true); //deprecated in qwt 6.1.0

    if(exportType)
    {
        QSize myRect(curSize.width(), curSize.height());
        QClipboard *clipboard = QApplication::clipboard();
        QwtPlotRenderer renderer;
        QImage img(myRect, QImage::Format_ARGB32);
        QPainter painter(&img);
        renderer.render(((Plot1DWidget *)m_pContent), &painter, ((Plot1DWidget *)m_pContent)->rect());
        clipboard->setImage(img);    
    }
    else renderer.renderDocument((((Plot1DWidget *)m_pContent)), fileName, curSize, resolution);

    ((Plot1DWidget *)m_pContent)->setPalette( curPalette);
    ((Plot1DWidget *)m_pContent)->setCanvasBackground( curBrush);

    ((Plot1DWidget *)m_pContent)->replot();
    return ito::retOk;
}
ito::RetVal Itom1DQwtPlot::copyToClipBoard()
{
    return exportCanvas(true, "");
}
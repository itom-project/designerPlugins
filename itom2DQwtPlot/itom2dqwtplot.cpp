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

#include "itom2dqwtplot.h"
#include "../sharedFiles/userInteractionPlotPicker.h"
#include "../sharedFiles/multiPointPickerMachine.h"

#include "../sharedFiles/dialogExportProperties.h"

#include <qwidgetaction.h>
#include <qfiledialog.h>
#include <qimagewriter.h>
#include <qwt_plot_renderer.h>
#include <qmenu.h>
#include "dialog2DScale.h"
#include <qwt_text_label.h>
#include <qwt_scale_widget.h>
#include <qwt_picker_machine.h>

#include <qwt_plot_layout.h>

#include "common/sharedStructuresPrimitives.h"
#include "DataObject/dataObjectFuncs.h"

//----------------------------------------------------------------------------------------------------------------------------------
Itom2dQwtPlot::Itom2dQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    AbstractDObjFigure(itomSettingsFile, windowMode, parent),
    m_pContent(NULL),
    m_pActSave(NULL),
    m_pActHome(NULL),
    m_pActPan(NULL),
    m_pActZoom(NULL),
    m_pActScaleSettings(NULL),
    m_pActColorPalette(NULL),
    m_pActToggleColorBar(NULL),
    m_pActValuePicker(NULL),
    m_pActLineCut(NULL),
    m_pActStackCut(NULL),
    m_pActPlaneSelector(NULL),
    m_pActCmplxSwitch(NULL),
    m_pActClearDrawings(NULL),
    m_mnuCmplxSwitch(NULL),
    m_pActCoordinates(NULL),
    m_pCoordinates(NULL),
    m_pActDrawMode(NULL),
    m_pMnuDrawMode(NULL),
    m_pActCntrMarker(NULL),
    m_pActAspectRatio(NULL),
    m_pDrawModeActGroup(NULL),
    m_pOverlaySlider(NULL),
    m_pActOverlaySlider(NULL)
{
    m_pOutput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for line plots from 2d objects").toLatin1().data()));
    m_pOutput.insert("sourceout", new ito::Param("sourceout", ito::ParamBase::DObjPtr, NULL, QObject::tr("shallow copy of input source object").toLatin1().data()));

    int id = qRegisterMetaType<QSharedPointer<ito::DataObject> >("QSharedPointer<ito::DataObject>");

    //init actions
    createActions();

    //init internal data
    m_data.m_dataType = ito::tFloat64;
    m_data.m_autoTitle;
    m_data.m_autoxAxisLabel = true;
    m_data.m_autoyAxisLabel = true;
    m_data.m_autoValueLabel = true;
    m_data.m_valueScaleAuto = true;
    m_data.m_valueMin = -127.0;
    m_data.m_valueMax = 128.0;
    m_data.m_xaxisScaleAuto = true;
    m_data.m_yaxisScaleAuto = true;
    m_data.m_xaxisVisible = true;
    m_data.m_yaxisVisible = true;
    m_data.m_colorBarVisible = false;
    m_data.m_cmplxType = PlotCanvas::Abs;
    m_data.m_yaxisFlipped = false;
    m_data.m_pConstOutput = &m_pOutput;
    m_data.m_state = PlotCanvas::tIdle;
    m_data.m_elementsToPick = 0;
    m_data.m_pDrawItems.clear();

    //initialize canvas
    m_pContent = new PlotCanvas(&m_data, this);
    connect(m_pContent, SIGNAL(statusBarClear()), (QObject*)statusBar(), SLOT(clearMessage()));
    connect(m_pContent, SIGNAL(statusBarMessage(QString)), (QObject*)statusBar(), SLOT(showMessage(QString)));
    connect(m_pContent, SIGNAL(statusBarMessage(QString,int)), (QObject*)statusBar(), SLOT(showMessage(QString,int)));
    setCentralWidget(m_pContent);

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
    menuTools->addSeparator();
    menuTools->addAction(m_pActValuePicker);
    menuTools->addAction(m_pActCntrMarker);
    menuTools->addAction(m_pActLineCut);
    menuTools->addAction(m_pActStackCut);
    menuTools->addSeparator();
    menuTools->addAction(m_pActDrawMode);
    menuTools->addAction(m_pActClearDrawings);
    addMenu(menuTools); //AbstractFigure takes care of the menu

    setPropertyObservedObject(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom2dQwtPlot::~Itom2dQwtPlot()
{
    if (m_mnuCmplxSwitch != NULL)
    {
        delete m_mnuCmplxSwitch;
        m_mnuCmplxSwitch = NULL;
    }
    m_pContent->deleteLater();
    m_pContent = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::createActions()
{
    QAction *a = NULL;

    //m_actSave
    m_pActSave = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("Save..."), this);
    a->setObjectName("actSave");
    a->setToolTip(tr("Export current view..."));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActSave()));

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

    //m_actScaleSetting
    m_pActScaleSettings = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/autoscal.png"), tr("Scale Settings..."), this);
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
    a->setToolTip(tr("Show a slice through z-Stack"));
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
    m_pActDrawMode = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/marker.png"), tr("Switch Draw Mode"), this);
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
    a->setData(PlotCanvas::Real);
    m_mnuCmplxSwitch->addAction(a);
    a->setCheckable(true);

    a = m_pCmplxActGroup->addAction(tr("Imag"));
    a->setData(PlotCanvas::Imag);
    m_mnuCmplxSwitch->addAction(a);
    a->setCheckable(true);

    a = m_pCmplxActGroup->addAction(tr("Abs"));
    a->setData(PlotCanvas::Abs);
    m_mnuCmplxSwitch->addAction(a);
    a->setCheckable(true);
    a->setChecked(true);

    a = m_pCmplxActGroup->addAction(tr("Pha"));
    a->setData(PlotCanvas::Phase);
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
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getTitle() const
{
    if (m_data.m_autoTitle)
    {
        return "<auto>";
    }
    return m_data.m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setTitle(const QString &title)
{
    if (title == "<auto>")
    {
        m_data.m_autoTitle = true;
    }
    else
    {
        m_data.m_autoTitle = false;
        m_data.m_title = title;
    }

    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetTitle()
{
    m_data.m_autoTitle = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getxAxisLabel() const
{
    if (m_data.m_autoxAxisLabel)
    {
        return "<auto>";
    }
    return m_data.m_xaxisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setxAxisLabel(const QString &label)
{
    if (label == "<auto>")
    {
        m_data.m_autoxAxisLabel = true;
    }
    else
    {
        m_data.m_autoxAxisLabel = false;
        m_data.m_xaxisLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetxAxisLabel()
{
    m_data.m_autoxAxisLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getyAxisLabel() const
{
    if (m_data.m_autoyAxisLabel)
    {
        return "<auto>";
    }
    return m_data.m_yaxisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisLabel(const QString &label)
{
    if (label == "<auto>")
    {
        m_data.m_autoyAxisLabel = true;
    }
    else
    {
        m_data.m_autoyAxisLabel = false;
        m_data.m_yaxisLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetyAxisLabel()
{
    m_data.m_autoyAxisLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getValueLabel() const
{
    if (m_data.m_autoValueLabel)
    {
        return "<auto>";
    }
    return m_data.m_valueLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setValueLabel(const QString &label)
{
    if (label == "<auto>")
    {
        m_data.m_autoValueLabel = true;
    }
    else
    {
        m_data.m_autoValueLabel = false;
        m_data.m_valueLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetValueLabel()
{
    m_data.m_autoValueLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getyAxisFlipped() const
{
    return m_data.m_yaxisFlipped;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisFlipped(const bool &value)
{
    if (m_data.m_yaxisFlipped != value)
    {
        m_data.m_yaxisFlipped = value;

        if (m_pContent)
        {
            m_pContent->updateScaleValues();
            m_pContent->internalDataUpdated();
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getxAxisVisible() const
{
    return m_data.m_xaxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setxAxisVisible(const bool &value)
{
    m_data.m_xaxisVisible = value;

    if (m_pContent) m_pContent->enableAxis(QwtPlot::xBottom, value);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getyAxisVisible() const
{
    return m_data.m_yaxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisVisible(const bool &value)
{
    m_data.m_yaxisVisible = value;

    if (m_pContent) m_pContent->enableAxis(QwtPlot::yLeft, value);
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF Itom2dQwtPlot::getXAxisInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getInterval(Qt::XAxis);
    }
    return QPointF();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setXAxisInterval(QPointF point)
{
    if (m_pContent)
    {
        m_pContent->setInterval(Qt::XAxis, point);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF Itom2dQwtPlot::getYAxisInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getInterval(Qt::YAxis);
    }
    return QPointF();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setYAxisInterval(QPointF point)
{
    if (m_pContent)
    {
        m_pContent->setInterval(Qt::YAxis, point);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF Itom2dQwtPlot::getZAxisInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getInterval(Qt::ZAxis);
    }
    return QPointF();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setZAxisInterval(QPointF point)
{
    if (m_pContent)
    {
        m_pContent->setInterval(Qt::ZAxis, point);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF Itom2dQwtPlot::getoverlayInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getOverlayInterval(Qt::ZAxis);
    }
    return QPointF();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setoverlayInterval(QPointF point)
{
    if (m_pContent)
    {
        m_pContent->setOverlayInterval(Qt::ZAxis, point);
    }
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
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActSave()
{
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

        QFileInfo fi(fileName);
        saveDefaultPath = fi.path();
        /*
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

        renderer.renderDocument((m_pContent), fileName, curSize, resolution);

        m_pContent->setPalette( curPalette);
        m_pContent->setCanvasBackground( curBrush);

        m_pContent->replot();
        */
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
    Dialog2DScale *dlg = new Dialog2DScale(m_data, this);
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData(m_data);

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
    if (m_pContent) m_pContent->changePlane(plane);

    QStringList paramNames;
    paramNames << "displayed";
    updateChannels(paramNames);
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
    setCmplxSwitch((PlotCanvas::ComplexType)(action->data().toInt()), true);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setCmplxSwitch(PlotCanvas::ComplexType type, bool visible)
{
    m_pActCmplxSwitch->setVisible(visible);

    if (m_data.m_cmplxType != type)
    {

        if (visible)
        {
            m_data.m_cmplxType = type;

            switch (type)
            {
                case PlotCanvas::Imag:
                    m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReImag.png"));
                break;
                case PlotCanvas::Real:
                    m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReReal.png"));
                break;
                case PlotCanvas::Phase:
                    m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImRePhase.png"));
                break;
                case PlotCanvas::Abs:
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
    ito::RetVal retval = ito::retOk;
    QList<QString> paramNames;
    ito::uint32 newUniqueID = uniqueID;
    QWidget *lineCutObj = NULL;

    double *pointArr = new double[2 * bounds.size()];
    for (int np = 0; np < bounds.size(); np++)
    {
        pointArr[np * 2] = bounds[np].x();
        pointArr[np * 2 + 1] = bounds[np].y();
    }
    m_pOutput["bounds"]->setVal(pointArr, 2 * bounds.size());
    delete[] pointArr;
    //setOutpBounds(bounds);
    //setLinePlotCoordinates(bounds);

    retval += apiGetFigure("DObjStaticLine","", newUniqueID, &lineCutObj, this); //(newUniqueID, "itom1DQwtFigure", &lineCutObj);

    if (!retval.containsError())
    {
        if (uniqueID != newUniqueID)
        {
            uniqueID = newUniqueID;
            ito::AbstractDObjFigure* figure = NULL;
            if (lineCutObj->inherits("ito::AbstractDObjFigure"))
            {
                figure = (ito::AbstractDObjFigure*)lineCutObj;
                m_childFigures[lineCutObj] = newUniqueID;
                connect(lineCutObj, SIGNAL(destroyed(QObject*)), this, SLOT(childFigureDestroyed(QObject*)));
            }
            else
            {
                return ito::RetVal(ito::retError, 0, tr("the opened figure is not inherited from ito::AbstractDObjFigure").toLatin1().data());
            }

            retval += addChannel((ito::AbstractNode*)figure, m_pOutput["bounds"], figure->getInputParam("bounds"), ito::Channel::parentToChild, 0, 1);

            if (zStack)
            {
                // for a linecut in z-direction we have to pass the input object to the linecut, otherwise the 1D-widget "sees" only a 2D object
                // with one plane and cannot display the points in z-direction

                retval += addChannel((ito::AbstractNode*)figure,  m_pOutput["sourceout"], figure->getInputParam("source"), ito::Channel::parentToChild, 0, 1);
                paramNames << "bounds"  << "sourceout";
            }
            else
            {
                // otherwise simply pass on the displayed plane
                retval += addChannel((ito::AbstractNode*)figure, m_pOutput["displayed"], figure->getInputParam("source"), ito::Channel::parentToChild, 0, 1);
                paramNames << "bounds"  << "displayed";
            }

            retval += updateChannels(paramNames);

            figure->show();
        }
        else
        {
            if (zStack)
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
    QList<int> keys = m_data.m_pDrawItems.keys();
    ito::RetVal retVal = ito::retOk;

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
    int ysize = m_data.m_pDrawItems.size();
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

    if(ysize == 0 || ysize < m_data.m_pDrawItems.size())
    {
        return ito::retError;
    }

    int xsize = dstObject->getSize(1);

    cv::Mat *tarMat = (cv::Mat*)(dstObject->get_mdata()[0]);
    ito::float32* rowPtr = tarMat->ptr<ito::float32>(0);
    memset(rowPtr, 0, sizeof(ito::float32) * xsize * ysize);

    QHash<int, DrawItem*>::Iterator it = m_data.m_pDrawItems.begin();

//    for(int y = 0; y < ysize; y++)
//    {
    int y = 0;
    for (; it != m_data.m_pDrawItems.end(); it++)
    {
        rowPtr = tarMat->ptr<ito::float32>(y);
        //if(m_data.m_pDrawItems[y] == NULL)
        if(it.value() == NULL)
        {
            continue;
        }
        //rowPtr[0] = (ito::float32) (m_data.m_pDrawItems[y]->m_idx);
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

    QList<int> keys = m_data.m_pDrawItems.keys();
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
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setkeepAspectRatio(const bool &keepAspectEnable)
{
    if (m_pActAspectRatio) //if property is set in designer or by python, the action should represent the current status, too
    {
        m_pActAspectRatio->setChecked(keepAspectEnable);
    }
    mnuActRatio(keepAspectEnable);
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActRatio(bool checked)
{
    /*m_data.m_keepAspect = checked;
    
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

    m_data.m_keepAspect = checked;
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

void Itom2dQwtPlot::setEnabledCenterMarker(const bool &enabled)
{
    if (m_pActCntrMarker && m_pActCntrMarker->isChecked() != enabled) //if property is set in designer or by python, the action should represent the current status, too
    {
        m_pActCntrMarker->setChecked(enabled);
    }

    m_data.m_showCenterMarker = enabled;
    if(m_pContent)
    {
        m_pContent->setState(m_data.m_state);
        m_pContent->replot();
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setEnabledPlotting(const bool &enabled)
{
    m_data.m_enablePlotting = enabled;
    m_pActClearDrawings->setEnabled(enabled);
    m_pActDrawMode->setEnabled(enabled);
    if(m_pActDrawMode->isChecked() && !enabled) m_pActDrawMode->setChecked(enabled);

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
int Itom2dQwtPlot::getSelectedElement(void)const
{
    QHash<int, DrawItem*>::const_iterator it = m_data.m_pDrawItems.begin();
    for (;it != m_data.m_pDrawItems.end(); ++it)        
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
    bool replot = false;
    bool failed = idx == -1 ? false : true;
    QHash<int, DrawItem*>::const_iterator it = m_data.m_pDrawItems.begin();
    for (;it != m_data.m_pDrawItems.end(); ++it)        
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
    return;
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
    if(value != m_data.m_alpha)
    {
        m_data.m_alpha = value;
        if(m_pContent) m_pContent->alphaChanged();
    }
}
//----------------------------------------------------------------------------------------------------------------------------------

ito::RetVal Itom2dQwtPlot::exportCanvas(const bool exportType, const QString &fileName, QSizeF curSize, const int resolution)
{
    if(!m_pContent)
    {
        return ito::RetVal(ito::retError, 0, tr("Export image failed, canvas handle not initilized").toLatin1().data());
    }
    if(curSize.height() == 0 || curSize.width() == 0)
    {
        curSize = ((PlotCanvas *)m_pContent)->size();
    }
    QBrush curBrush = ((PlotCanvas *)m_pContent)->canvasBackground();

    QPalette curPalette = ((PlotCanvas *)m_pContent)->palette();

    ((PlotCanvas *)m_pContent)->setAutoFillBackground( true );
    ((PlotCanvas *)m_pContent)->setPalette( Qt::white );
    ((PlotCanvas *)m_pContent)->setCanvasBackground(Qt::white);    

    ((PlotCanvas *)m_pContent)->replot();

    QwtPlotRenderer renderer;

    // flags to make the document look like the widget
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    //renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true); //deprecated in qwt 6.1.0

    if(exportType)
    {
        int resFaktor = cv::saturate_cast<int>(resolution / 72.0 + 0.5);
        resFaktor = resFaktor < 1 ? 1 : resFaktor;

        QSize myRect(curSize.width() * resFaktor, curSize.height() * resFaktor);
        QClipboard *clipboard = QApplication::clipboard();
        QImage img(myRect, QImage::Format_ARGB32);
        QPainter painter(&img);
        painter.scale(resFaktor, resFaktor);
        renderer.render(((PlotCanvas *)m_pContent), &painter, ((PlotCanvas *)m_pContent)->rect());
        clipboard->setImage(img);    
    }
    else renderer.renderDocument((((PlotCanvas *)m_pContent)), fileName, curSize, resolution);

    ((PlotCanvas *)m_pContent)->setPalette( curPalette);
    ((PlotCanvas *)m_pContent)->setCanvasBackground( curBrush);

    ((PlotCanvas *)m_pContent)->replot();
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
        curSize = ((PlotCanvas *)m_pContent)->size();
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
    QBrush curBrush = ((PlotCanvas *)m_pContent)->canvasBackground();

    QPalette curPalette = ((PlotCanvas *)m_pContent)->palette();

    ((PlotCanvas *)m_pContent)->setAutoFillBackground( true );
    ((PlotCanvas *)m_pContent)->setPalette( Qt::white );
    ((PlotCanvas *)m_pContent)->setCanvasBackground(Qt::white);    

    ((PlotCanvas *)m_pContent)->replot();

    QwtPlotRenderer renderer;

    // flags to make the document look like the widget
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    //renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true); //deprecated in qwt 6.1.0

    //QImage img(myRect, QImage::Format_ARGB32);
    QPainter painter(&destinationImage);
    painter.scale(resFaktor, resFaktor);
    renderer.render(((PlotCanvas *)m_pContent), &painter, ((PlotCanvas *)m_pContent)->rect());
    //destinationImage.convertFromImage(img);


    ((PlotCanvas *)m_pContent)->setPalette( curPalette);
    ((PlotCanvas *)m_pContent)->setCanvasBackground( curBrush);

    ((PlotCanvas *)m_pContent)->replot();

    return destinationImage;
}
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

#include "evaluateGeometrics.h"

#include "DataObject/dataObjectFuncs.h"
#include "DataObject/dataobj.h"

#include <qmessagebox.h>
#include <qsharedpointer.h>

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
EvaluateGeometricsFigure::EvaluateGeometricsFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    AbstractDObjFigure(itomSettingsFile, windowMode, parent),
    m_pContent(NULL),
    m_actScaleSetting(NULL),
    m_rescaleParent(NULL),
    m_actForward(NULL),
    m_actBack(NULL),
	m_actHome(NULL),
	m_actSave(NULL),
    m_actPan(NULL),
    m_actZoomToRect(NULL),
    m_actMarker(NULL)
{
    m_pInput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, tr("Points for line plots from 2d objects").toAscii().data()));
    
    //int id = qRegisterMetaType<QSharedPointer<ito::DataObject> >("QSharedPointer<ito::DataObject>");

	//m_actHome
    m_actHome = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"), tr("Home"), this);
    m_actHome->setObjectName("actHome");
    m_actHome->setToolTip(tr("Reset original view"));

	//m_actSave
    m_actSave = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("Save"), this);
    m_actSave->setObjectName("actSave");
    m_actSave->setToolTip(tr("Export current view"));

    //m_actScaleSetting
    m_actScaleSetting = new QAction(QIcon(":/plots/icons/itom_icons/autoscal.png"), tr("Scale Settings"), this);
    m_actScaleSetting->setObjectName("actScaleSetting");
    m_actScaleSetting->setToolTip(tr("Set the ranges and offsets of this view"));

    //m_rescaleParent
    m_rescaleParent = new QAction(QIcon(":/itom1DQwtFigurePlugin/icons/parentScale.png"), tr("Parent Scale Settings"), this);
    m_rescaleParent->setObjectName("rescaleParent");
    m_rescaleParent->setToolTip(tr("Set the value-range of the parent view according to this plot"));
    m_rescaleParent->setVisible(false);

    //m_actForward
    m_actForward = new QAction(QIcon(":/itomDesignerPlugins/general/icons/forward.png"), tr("forward"), this);
    m_actForward->setObjectName("actionForward");
    m_actForward->setEnabled(false);
    m_actForward->setToolTip(tr("Forward to next line"));

    //m_actBack
    m_actBack = new QAction(QIcon(":/itomDesignerPlugins/general/icons/back.png"), tr("back"), this);
    m_actBack->setObjectName("actionBack");
    m_actBack->setEnabled(false);
    m_actBack->setToolTip(tr("Back to previous line"));

    //m_actPan
    m_actPan = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), tr("move"), this);
    m_actPan->setObjectName("actionPan");
    m_actPan->setCheckable(true);
    m_actPan->setChecked(false);
    m_actPan->setToolTip(tr("Pan axes with left mouse, zoom with right"));

    //m_actZoomToRect
    m_actZoomToRect = new QAction(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"), tr("zoom to rectangle"), this);
    m_actZoomToRect->setObjectName("actionZoomToRect");
    m_actZoomToRect->setCheckable(true);
    m_actZoomToRect->setChecked(false);
    m_actZoomToRect->setToolTip(tr("Zoom to rectangle"));


    connect(m_actHome, SIGNAL(triggered()), this, SLOT(mnuHome()));
    connect(m_actSave, SIGNAL(triggered()), this, SLOT(mnuExport()));
    
    connect(m_actScaleSetting, SIGNAL(triggered()), this, SLOT(mnuScaleSetting()));
    connect(m_rescaleParent, SIGNAL(triggered()), this, SLOT(mnuParentScaleSetting()));

    connect(m_actMarker, SIGNAL(toggled(bool)), this, SLOT(mnuMarkerClick(bool)));
    connect(m_actZoomToRect, SIGNAL(toggled(bool)), this, SLOT(mnuZoomer(bool)));
    connect(m_actPan, SIGNAL(toggled(bool)), this, SLOT(mnuPanner(bool)));

	QToolBar *toolbar = new QToolBar(tr("1D plot toolbar"), this);
	addToolBar(toolbar, "mainToolBar");

	QMenu *contextMenu = new QMenu(QObject::tr("plot1D"), this);
    contextMenu->addAction(m_actSave);
    contextMenu->addSeparator();
    contextMenu->addAction(m_actHome);
    contextMenu->addAction(m_actScaleSetting);
    contextMenu->addSeparator();
    contextMenu->addAction(m_actPan);
    contextMenu->addAction(m_actZoomToRect);
    contextMenu->addSeparator();
    contextMenu->addAction(toolbar->toggleViewAction());

    // first block is zoom, scale settings, home
	toolbar->addAction(m_actSave);
	toolbar->addSeparator();
	toolbar->addAction(m_actHome);
    toolbar->addAction(m_actScaleSetting);
    toolbar->addAction(m_rescaleParent);
    toolbar->addAction(m_actPan);
    toolbar->addAction(m_actZoomToRect);


    // next block is for complex and stacks
    toolbar->addSeparator();
    toolbar->addAction(m_actBack);
    toolbar->addAction(m_actForward);


    m_pContent = new PlotTable(contextMenu, &m_info, this);
    m_pContent->setObjectName("canvasWidget");

    setFocus();
    setCentralWidget(m_pContent);
    
    

    m_pContent->setFocus();

    m_info.m_autoAxisLabel = true;
    m_info.m_autoValueLabel = true;
    m_info.m_autoTitle = true;

    m_info.m_title = "";
    m_info.m_axisLabel = "";
    m_info.m_valueLabel = "";
    m_info.titleLabel = "";

}

//----------------------------------------------------------------------------------------------------------------------------------
EvaluateGeometricsFigure::~EvaluateGeometricsFigure()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal EvaluateGeometricsFigure::applyUpdate()
{
    QVector<QPointF> bounds = getBounds();

    if ((ito::DataObject*)m_pInput["source"]->getVal<void*>())
    {
        m_pOutput["displayed"]->copyValueFrom(m_pInput["source"]);
        // why "source" is used here and not "displayed" .... ck 05/15/2013
        m_pContent->refreshPlot((ito::DataObject*)m_pInput["source"]->getVal<char*>());

        ito::Channel* dataChannel = getInputChannel("source");
        m_rescaleParent->setVisible(dataChannel && dataChannel->getParent());
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setSource(QSharedPointer<ito::DataObject> source)
{
    AbstractDObjFigure::setSource(source);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool EvaluateGeometricsFigure::getContextMenuEnabled() const
{
    if (m_pContent) return (m_pContent)->m_showContextMenu;
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setContextMenuEnabled(bool show)
{
    if (m_pContent) (m_pContent)->m_showContextMenu = show;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setBounds(QVector<QPointF> bounds) 
{ 

}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<QPointF> EvaluateGeometricsFigure::getBounds(void) 
{ 
    QVector<QPointF> val(4);
    val[0] = QPointF(0.0, 0.0);
    val[1] = QPointF(0.0, 1.0);
    val[2] = QPointF(1.0, 0.0);
    val[3] = QPointF(1.0, 1.0);
    return val;
}

//----------------------------------------------------------------------------------------------------------------------------------
QString EvaluateGeometricsFigure::getTitle() const
{
    if (m_info.m_autoTitle)
    {
        return "<auto>";
    }
    return m_info.m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setTitle(const QString &title)
{
    if (title == "<auto>")
    {
        m_info.m_autoTitle = true;
    }
    else
    {
        m_info.m_autoTitle = false;
        m_info.m_title = title;
    }

    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::resetTitle()
{
    m_info.m_autoTitle = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString EvaluateGeometricsFigure::getAxisLabel() const
{
    if (m_info.m_autoAxisLabel)
    {
        return "<auto>";
    }
    return m_info.m_axisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setAxisLabel(const QString &label)
{
    if (label == "<auto>")
    {
        m_info.m_autoAxisLabel = true;
    }
    else
    {
        m_info.m_autoAxisLabel = false;
        m_info.m_axisLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::resetAxisLabel()
{
    m_info.m_autoAxisLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString EvaluateGeometricsFigure::getValueLabel() const
{
    if (m_info.m_autoValueLabel)
    {
        return "<auto>";
    }
    return m_info.m_valueLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setValueLabel(const QString &label)
{
    if (label == "<auto>")
    {
        m_info.m_autoValueLabel = true;
    }
    else
    {
        m_info.m_autoValueLabel = false;
        m_info.m_valueLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::resetValueLabel()
{
    m_info.m_autoValueLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont EvaluateGeometricsFigure::getTitleFont(void) const
{
    if (m_pContent)
    {

    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setTitleFont(const QFont &font)
{
    if (m_pContent)
    {

    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont EvaluateGeometricsFigure::getLabelFont(void) const
{
    if (m_pContent)
    {

    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setLabelFont(const QFont &font)
{
    if (m_pContent)
    {

    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont EvaluateGeometricsFigure::getAxisFont(void) const
{
    if (m_pContent)
    {

    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::setAxisFont(const QFont &font)
{
    if (m_pContent)
    {

    }
}


//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuPanner(bool checked)
{
    if (checked)
    {
        m_actZoomToRect->setChecked(false);
        m_actMarker->setChecked(false);
        //(m_pContent)->setMouseTracking(true);
        m_pContent->setPannerEnable(true);
    }
    else
    {
        m_pContent->setPannerEnable(false);
        //(m_pContent)->setMouseTracking(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuZoomer(bool checked)
{
    if (checked)
    {
        m_actMarker->setChecked(false);
        m_actPan->setChecked(false);
        //(m_pContent)->setMouseTracking(true);
        m_pContent->setZoomerEnable(true);
    }
    else
    {
        m_pContent->setZoomerEnable(false);
        //(m_pContent)->setMouseTracking(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuMarkerClick(bool checked)
{
    if (checked)
    {
        m_actPan->setChecked(false);
        m_actZoomToRect->setChecked(false);
    }
    
    m_pContent->setPickerEnable(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuExport()
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuScaleSetting()
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuParentScaleSetting()
{
   
}


//----------------------------------------------------------------------------------------------------------------------------------
QPointF EvaluateGeometricsFigure::getYAxisInterval(void) const
{ 
    QPointF interval(0.0, 1.0);
    return interval;
}

//----------------------------------------------------------------------------------------------------------------------------------        
void EvaluateGeometricsFigure::setYAxisInterval(QPointF interval) 
{ 

    return; 
}   

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::enableComplexGUI(const bool checked)
{ 

}

//----------------------------------------------------------------------------------------------------------------------------------
void EvaluateGeometricsFigure::mnuHome()
{

}

//----------------------------------------------------------------------------------------------------------------------------------
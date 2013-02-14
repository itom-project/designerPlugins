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
#include <qwidgetaction.h>

Itom2dQwtPlot::Itom2dQwtPlot(const QString &itomSettingsFile, QWidget *parent) :
    AbstractDObjFigure(itomSettingsFile, parent),
	m_pCanvas(NULL),
	m_pActions(NULL)
{
	//init actions
	m_pActions = new Itom2dQwtPlotActions();
	createActions();

	//initialize canvas
	m_pCanvas = new PlotCanvas(m_pActions, this);
	setCentralWidget(m_pCanvas);

	//initialize actions
	QToolBar *mainTb = new QToolBar("plotting tools",this);
	addToolBar(mainTb, "mainToolBar");

	

	mainTb->addAction(m_pActions->m_actSave);
	mainTb->addSeparator();
	mainTb->addAction(m_pActions->m_actHome);
	mainTb->addAction(m_pActions->m_actPan);
	mainTb->addAction(m_pActions->m_actZoom);
	mainTb->addSeparator();
	mainTb->addAction(m_pActions->m_actScaleSettings);
	mainTb->addAction(m_pActions->m_actToggleColorBar);
	mainTb->addAction(m_pActions->m_actColorPalette);
	mainTb->addSeparator();
	mainTb->addAction(m_pActions->m_actTracker);
	mainTb->addAction(m_pActions->m_actLineCut);
	mainTb->addAction(m_pActions->m_actStackCut);
	mainTb->addSeparator();
	mainTb->addAction(m_pActions->m_actPlaneSelector);

	
}

Itom2dQwtPlot::~Itom2dQwtPlot()
{
	delete m_pCanvas;
	m_pCanvas = NULL;

	delete m_pActions;
	m_pActions = NULL;
}

void Itom2dQwtPlot::createActions()
{
	QAction *a = NULL;
	/*m_actions["save"]
    m_actions["home"]
    m_actions["pan"]
    m_actions["zoom"]
    m_actions["scaleSettings"]
    m_actions["colorPalette"]
    m_actions["toggleColorBar"]
    m_actions["pointTracker"]
    m_actions["lineCut"]
    m_actions["stackCut"]
	m_actions["planeSelector"]*/

	//m_actSave
    m_pActions->m_actSave = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"),tr("Save"), this);
    a->setObjectName("actSave");
    a->setToolTip("Export current view");
	
	//m_actHome
    m_pActions->m_actHome = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"),tr("Home"), this);
    a->setObjectName("actHome");
    a->setToolTip("Reset original view");

	//m_actPan
    m_pActions->m_actPan = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), QObject::tr("move"), this);
    a->setObjectName("actionPan");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip("Pan axes with left mouse, zoom with right");

	//m_actZoomToRect
    m_pActions->m_actZoom = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"), QObject::tr("zoom to rectangle"), this);
    a->setObjectName("actionZoomToRect");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip("Zoom to rectangle");

    //m_actScaleSetting
    m_pActions->m_actScaleSettings = a = new QAction(QIcon(":/plots/icons/itom_icons/autoscal.png"),tr("Scale Settings"), this);
    a->setObjectName("actScaleSetting");
    a->setToolTip("Set the ranges and offsets of this view");

    //m_actPalette
    m_pActions->m_actColorPalette = a = new QAction(QIcon(":/plots/icons/itom_icons/color.png"),tr("Palette"),this);
    a->setObjectName("TogglePalette");
    a->setToolTip("Switch between color palettes");
	connect(a, SIGNAL(triggered()), this, SLOT(mnuSwitchColorPalette()));

    //m_actToggleColorBar
    m_pActions->m_actToggleColorBar = a = new QAction(QIcon(":/plots/icons/itom_icons/colorbar.png"),tr("Show Colorbar"), this);
    a->setCheckable(true);
    a->setObjectName("ShowColorBar");
    a->setToolTip("Toggle visibility of the color bar on right canvas side");
	connect(a,SIGNAL(toggled()),this,SLOT(mnuToggleColorBar()));

    //m_actMarker
    m_pActions->m_actTracker = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/marker.png"), QObject::tr("marker"), this);
    a->setObjectName("actionMarker");
    a->setCheckable(true);
    a->setChecked(false);

    //m_actLineCut
    m_pActions->m_actLineCut = a = new QAction(QIcon(":/plots/icons/itom_icons/pntline.png"),tr("Linecut"),this);
    a->setCheckable(true);
    a->setObjectName("LineCut");
    a->setToolTip("Show a in plane line cut");

    //m_actStackCut
    m_pActions->m_actStackCut = a = new QAction(QIcon(":/plots/icons/itom_icons/1dzdir.png"),tr("Slice in z-direction"),this);
    a->setObjectName("a-Scan");
    a->setToolTip("Show a slice through z-Stack");
    a->setCheckable(true);
    a->setVisible(false);

	m_pActions->m_planeSelector = new QSpinBox(this);
	m_pActions->m_planeSelector->setToolTip("Select image plane");
	QWidgetAction *wa = new QWidgetAction(this);
	wa->setDefaultWidget(m_pActions->m_planeSelector);
	m_pActions->m_actPlaneSelector = a = wa;
	a->setObjectName("planeSelector");
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::applyUpdate()
{
    m_pOutput["displayed"]->copyValueFrom(m_pInput["source"]);
    m_pCanvas->refreshPlot( m_pOutput["displayed"] );
    //m_pContent->refreshPlot(m_pOutput["displayed"]); //push the displayed DataObj into the actual plot widget for displaying

    return ito::retOk;
}

bool Itom2dQwtPlot::showColorBar() const
{
	return m_pActions->m_actToggleColorBar->isChecked();
}

void Itom2dQwtPlot::setShowColorBar(bool value)
{
	m_pActions->m_actToggleColorBar->setChecked(value); //emits toggle signal of action
}

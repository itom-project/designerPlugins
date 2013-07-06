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
#include <qfiledialog.h>
#include <qimagewriter.h>
#include "qwt_plot_renderer.h"

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
    m_pActPlaneSelector(NULL)
{
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
    m_data.m_colorBarVisible = false;

	//initialize canvas
	m_pContent = new PlotCanvas(&m_data, this);
	setCentralWidget(m_pContent);

	//initialize actions
	QToolBar *mainTb = new QToolBar("plotting tools",this);
	addToolBar(mainTb, "mainToolBar");

	mainTb->addAction(m_pActSave);
	mainTb->addSeparator();
	mainTb->addAction(m_pActHome);
	mainTb->addAction(m_pActPan);
	mainTb->addAction(m_pActZoom);
	mainTb->addSeparator();
	mainTb->addAction(m_pActScaleSettings);
	mainTb->addAction(m_pActToggleColorBar);
	mainTb->addAction(m_pActColorPalette);
	mainTb->addSeparator();
	mainTb->addAction(m_pActValuePicker);
	mainTb->addAction(m_pActLineCut);
	mainTb->addAction(m_pActStackCut);
	mainTb->addSeparator();
	mainTb->addAction(m_pActPlaneSelector);
}

Itom2dQwtPlot::~Itom2dQwtPlot()
{
	delete m_pContent;
	m_pContent = NULL;
}

//---------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::createActions()
{
	QAction *a = NULL;
	
	//m_actSave
    m_pActSave = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"),tr("Save"), this);
    a->setObjectName("actSave");
    a->setToolTip("Export current view");
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActSave()));

	//m_actHome
    m_pActHome = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"),tr("Home"), this);
    a->setObjectName("actHome");
    a->setToolTip("Reset original view");
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActHome()));

	//m_actPan
    m_pActPan = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), QObject::tr("move"), this);
    a->setObjectName("actPan");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip("Pan axes with left mouse, zoom with right");
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActPan(bool)));

	//m_actZoom
    m_pActZoom = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"), QObject::tr("zoom to rectangle"), this);
    a->setObjectName("actZoom");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip("Zoom to rectangle");
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActZoom(bool)));

    //m_actScaleSetting
    m_pActScaleSettings = a = new QAction(QIcon(":/plots/icons/itom_icons/autoscal.png"),tr("Scale Settings"), this);
    a->setObjectName("actScaleSetting");
    a->setToolTip("Set the ranges and offsets of this view");
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActScaleSettings()));

    //m_actPalette
    m_pActColorPalette = a = new QAction(QIcon(":/plots/icons/itom_icons/color.png"),tr("Palette"),this);
    a->setObjectName("actColorPalette");
    a->setToolTip("Switch between color palettes");
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActColorPalette()));

    //m_actToggleColorBar
    m_pActToggleColorBar = a = new QAction(QIcon(":/plots/icons/itom_icons/colorbar.png"),tr("Show Colorbar"), this);
    a->setCheckable(true);
    a->setObjectName("actShowColorBar");
    a->setToolTip("Toggle visibility of the color bar on right canvas side");
	connect(a,SIGNAL(toggled(bool)),this,SLOT(mnuActToggleColorBar(bool)));

    //m_actMarker
    m_pActValuePicker = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/marker.png"), QObject::tr("marker"), this);
    a->setObjectName("actValuePicker");
    a->setCheckable(true);
    a->setChecked(false);
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActValuePicker(bool)));

    //m_actLineCut
    m_pActLineCut = a = new QAction(QIcon(":/plots/icons/itom_icons/pntline.png"),tr("Linecut"),this);
    a->setCheckable(true);
    a->setObjectName("actLineCut");
    a->setToolTip("Show a in plane line cut");
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActLineCut(bool)));

    //m_actStackCut
    m_pActStackCut = a = new QAction(QIcon(":/plots/icons/itom_icons/1dzdir.png"),tr("Slice in z-direction"),this);
    a->setObjectName("actStackCut");
    a->setToolTip("Show a slice through z-Stack");
    a->setCheckable(true);
    a->setVisible(false);
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActStackCut(bool)));

	QSpinBox *planeSelector = new QSpinBox(this);
	planeSelector->setToolTip("Select image plane");
	QWidgetAction *wa = new QWidgetAction(this);
	wa->setDefaultWidget(planeSelector);
	m_pActPlaneSelector = a = wa;
	a->setObjectName("planeSelector");
    connect(planeSelector, SIGNAL(valueChanged(int)), this, SLOT(mnuActPlaneSelector(int)));
	
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::applyUpdate()
{
    m_pOutput["displayed"]->copyValueFrom(m_pInput["source"]);
    m_pContent->refreshPlot( m_pOutput["displayed"] );
    //m_pContent->refreshPlot(m_pOutput["displayed"]); //push the displayed DataObj into the actual plot widget for displaying

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
    if(m_data.m_autoTitle)
    {
        return "<auto>";
    }
    return m_data.m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setTitle(const QString &title)
{
    if(title == "<auto>")
    {
        m_data.m_autoTitle = true;
    }
    else
    {
        m_data.m_autoTitle = false;
        m_data.m_title = title;
    }

    if(m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetTitle()
{
    m_data.m_autoTitle = true;
    if(m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getxAxisLabel() const
{
    if(m_data.m_autoxAxisLabel)
    {
        return "<auto>";
    }
    return m_data.m_xaxisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setxAxisLabel(const QString &label)
{
    if(label == "<auto>")
    {
        m_data.m_autoxAxisLabel = true;
    }
    else
    {
        m_data.m_autoxAxisLabel = false;
        m_data.m_xaxisLabel = label;
    }
    if(m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetxAxisLabel()
{
    m_data.m_autoxAxisLabel = true;
    if(m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getyAxisLabel() const
{
    if(m_data.m_autoyAxisLabel)
    {
        return "<auto>";
    }
    return m_data.m_yaxisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisLabel(const QString &label)
{
    if(label == "<auto>")
    {
        m_data.m_autoyAxisLabel = true;
    }
    else
    {
        m_data.m_autoyAxisLabel = false;
        m_data.m_yaxisLabel = label;
    }
    if(m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetyAxisLabel()
{
    m_data.m_autoyAxisLabel = true;
    if(m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getValueLabel() const
{
    if(m_data.m_autoValueLabel)
    {
        return "<auto>";
    }
    return m_data.m_valueLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setValueLabel(const QString &label)
{
    if(label == "<auto>")
    {
        m_data.m_autoValueLabel = true;
    }
    else
    {
        m_data.m_autoValueLabel = false;
        m_data.m_valueLabel = label;
    }
    if(m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetValueLabel()
{
    m_data.m_autoValueLabel = true;
    if(m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActSave()
{
    #ifndef QT_NO_PRINTER
    QString fileName = "plot2D.pdf";
#else
    QString fileName = "plot2D.png";
#endif

#ifndef QT_NO_FILEDIALOG
    const QList<QByteArray> imageFormats =
        QImageWriter::supportedImageFormats();

    QStringList filter;
    filter += "PDF Documents (*.pdf)";
#ifndef QWT_NO_SVG
#ifdef QT_SVG_LIB
    filter += "SVG Documents (*.svg)";
#endif
#endif
    filter += "Postscript Documents (*.ps)";

    if ( imageFormats.size() > 0 )
    {
        QString imageFilter("Images (");
        for ( int i = 0; i < imageFormats.size(); i++ )
        {
            if ( i > 0 )
                imageFilter += " ";
            imageFilter += "*.";
            imageFilter += imageFormats[i];
        }
        imageFilter += ")";

        filter += imageFilter;
    }

    fileName = QFileDialog::getSaveFileName(
        this, "Export File Name", fileName,
        filter.join(";;"), NULL, QFileDialog::DontConfirmOverwrite);
#endif

    if ( !fileName.isEmpty() )
    {
        QwtPlotRenderer renderer;

        // flags to make the document look like the widget
        renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
        renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true);

        renderer.renderDocument((m_pContent), fileName, QSizeF(300, 200), 85);
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
    if(checked)
    {
        m_pActValuePicker->setChecked(false);
        m_pActZoom->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActStackCut->setChecked(false);
        m_pContent->setState( PlotCanvas::tPan );
    }
    else
    {
        m_pContent->setState( PlotCanvas::tIdle );
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActZoom(bool checked)
{
    if(checked)
    {
        m_pActValuePicker->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActStackCut->setChecked(false);
        m_pContent->setState( PlotCanvas::tZoom );
    }
    else
    {
        m_pContent->setState( PlotCanvas::tIdle );
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActScaleSettings()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActColorPalette()
{
    if (m_pContent) m_pContent->setColorMap("__next__");
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActToggleColorBar(bool checked)
{
    if (m_pContent) m_pContent->setColorBarVisible( checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActValuePicker(bool checked)
{
    if(checked)
    {
        m_pActZoom->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActStackCut->setChecked(false);
    }
    
    m_pContent->setState( checked ? PlotCanvas::tValuePicker : PlotCanvas::tIdle );
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActLineCut(bool checked)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActStackCut(bool checked)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActPlaneSelector(int plane)
{
}
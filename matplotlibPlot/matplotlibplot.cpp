/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2016, Institut fuer Technische Optik (ITO), 
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

#include "matplotlibplot.h"
#include "matplotlibWidget.h"
#include "matplotlibSubfigConfig.h"

//----------------------------------------------------------------------------------------------------------------------------------
MatplotlibPlot::MatplotlibPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent /*= 0*/)
    : ito::AbstractFigure(itomSettingsFile, windowMode, parent),
    m_actHome(NULL),
    m_actForward(NULL),
    m_actBack(NULL),
    m_actPan(NULL),
    m_actZoomToRect(NULL), 
    m_actSubplotConfig(NULL), 
    m_actSave(NULL),
    m_actMarker(NULL), 
    m_contextMenu(NULL), 
    m_pContent(NULL),
    m_pMatplotlibSubfigConfig(NULL),
    m_forceWindowResize(true),
    m_keepSizeFixed(false),
    m_pResetFixedSizeTimer(NULL)
{
    setWindowFlags(Qt::Widget); //this is important such that this main window reacts as widget

    m_pResetFixedSizeTimer = new QTimer(this);
    connect(m_pResetFixedSizeTimer, SIGNAL(timeout()), this, SLOT(resetFixedSize()));
    m_pResetFixedSizeTimer->setSingleShot(true);
    m_pResetFixedSizeTimer->stop();

    m_actHome = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"), tr("Home"), this);
    m_actHome->setObjectName("actionHome");
    m_actHome->setShortcut(Qt::CTRL + Qt::Key_0);
    m_actHome->setToolTip(tr("Reset original view"));

    m_actForward = new QAction(QIcon(":/itomDesignerPlugins/general/icons/forward.png"), tr("Forward"), this);
    m_actForward->setObjectName("actionForward");
    m_actForward->setToolTip(tr("Forward to next view"));

    m_actBack = new QAction(QIcon(":/itomDesignerPlugins/general/icons/back.png"), tr("Back"), this);
    m_actBack->setObjectName("actionBack");
    m_actBack->setToolTip(tr("Back to previous view"));

    m_actPan = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), tr("Move"), this);
    m_actPan->setObjectName("actionPan");
    m_actPan->setCheckable(true);
    m_actPan->setChecked(false);
    m_actPan->setToolTip(tr("Pan axes with left mouse, zoom with right"));

    m_actZoomToRect = new QAction(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"), tr("Zoom to Rectangle"), this);
    m_actZoomToRect->setObjectName("actionZoomToRect");
    m_actZoomToRect->setCheckable(true);
    m_actZoomToRect->setChecked(false);
    m_actZoomToRect->setToolTip(tr("Zoom to rectangle"));

    m_actMarker = new QAction(QIcon(":/itomDesignerPlugins/general/icons/marker.png"), tr("Marker"), this);
    m_actMarker->setObjectName("actionMarker");
    m_actMarker->setCheckable(true);
    m_actMarker->setChecked(false);
    m_actZoomToRect->setToolTip(tr("Show coordinates under mouse cursor"));
    m_actMarker->connect(m_actMarker, SIGNAL(toggled(bool)), this, SLOT(mnuMarkerClick(bool)));

    m_actSubplotConfig = new QAction(QIcon(":/itomDesignerPlugins/general/icons/subplots.png"), tr("Subplot Configuration..."), this);
    m_actSubplotConfig->setObjectName("actionSubplotConfig");
    m_actSubplotConfig->setToolTip(tr("Configure subplots..."));

    m_actSave = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("Save..."), this);
    m_actSave->setShortcut(QKeySequence::Save);
    m_actSave->setObjectName("actionSave");
    m_actSave->setToolTip(tr("Save the figure..."));

    //m_actCopyClipboard
    m_actCopyClipboard = new QAction(tr("Copy To Clipboard"), this);
    m_actCopyClipboard->setShortcut(QKeySequence::Copy);
    m_actCopyClipboard->setObjectName("actionCopyClipboard");
    m_actCopyClipboard->setToolTip(tr("Copies the current view to the clipboard"));
    connect(m_actCopyClipboard, SIGNAL(triggered()), this, SLOT(mnuCopyToClipboard()));

    m_actProperties = this->getPropertyDockWidget()->toggleViewAction();
    connect(m_actProperties, SIGNAL(triggered(bool)), this, SLOT(mnuShowProperties(bool)));

    m_lblCoordinates = new QLabel("",this);
    m_lblCoordinates->setAlignment(Qt::AlignLeft | Qt::AlignCenter);
    m_lblCoordinates->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Ignored);
    m_lblCoordinates->setObjectName("lblCoordinates");

    m_toolbar = new QToolBar(tr("matplotlib toolbar"), this);
    addToolBar(m_toolbar, "mainToolBar");
    m_toolbar->setObjectName("toolbar");
    m_toolbar->addAction(m_actSave);
    m_toolbar->addSeparator();
    m_toolbar->addAction(m_actHome);
    m_toolbar->addAction(m_actBack);
    m_toolbar->addAction(m_actForward);
    m_toolbar->addAction(m_actPan);
    m_toolbar->addAction(m_actZoomToRect);
    m_toolbar->addSeparator();
    m_toolbar->addAction(m_actSubplotConfig);
    m_toolbar->addSeparator();
    m_toolbar->addAction(m_actMarker);
    
    QAction *lblAction = m_toolbar->addWidget(m_lblCoordinates);
    lblAction->setVisible(true);

    QMenu *contextMenu = new QMenu(tr("Matplotlib"),this);
    contextMenu->addAction(m_actSave);
    contextMenu->addAction(m_actCopyClipboard);
    contextMenu->addSeparator();
    contextMenu->addAction(m_actHome);
    contextMenu->addAction(m_actBack);
    contextMenu->addAction(m_actForward);
    contextMenu->addAction(m_actPan);
    contextMenu->addAction(m_actZoomToRect);
    contextMenu->addSeparator();
    contextMenu->addAction(m_actMarker);
    contextMenu->addSeparator();
    contextMenu->addAction(m_actSubplotConfig);
    contextMenu->addSeparator();
    contextMenu->addAction(m_toolbar->toggleViewAction());
    contextMenu->addAction(m_actProperties);
    addMenu(contextMenu);

    m_pContent = new MatplotlibWidget(contextMenu, this);
    setContentsMargins(0, 0, 0, 0);
    m_pContent->setObjectName("canvasWidget");

    setCentralWidget(m_pContent);

    setPropertyObservedObject(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
MatplotlibPlot::~MatplotlibPlot()
{
    if (m_pMatplotlibSubfigConfig)
    {
        m_pMatplotlibSubfigConfig->deleteLater();
        m_pMatplotlibSubfigConfig = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::resizeCanvas(int width, int height)
{
    QSize newSize(width, height);
    if (m_pContent)
    {
        //sometimes the size of the canvas is smaller than the size of this widget.
        //Since resizeCanvas should control the size of the canvas, the offset is added here.
        newSize += (size() - m_pContent->size());
    }

    resize(newSize);

    if (m_forceWindowResize)
    {
        //qDebug() << "fixed size (resizeCanvas)" << width << height;
        setFixedSize(newSize); //forces the window to a fixed size...
        updateGeometry();

        if (!m_keepSizeFixed)
        {
            m_pResetFixedSizeTimer->start(50); //and fire a trigger in order to cancel the fixed size (this is a hack in order to really force the window to its new size)
        }
    }
    else
    {
        updateGeometry();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::setKeepSizeFixed(bool fixed)
{
    if (fixed != m_keepSizeFixed)
    {
        if (fixed)
        {
            //qDebug() << "fixed size" << geometry().size();
            setFixedSize(geometry().size());
            m_pResetFixedSizeTimer->stop();
        }
        else
        {
            //qDebug() << "resetFixedSize";
            resetFixedSize();
        }
        m_keepSizeFixed = fixed;
        
        if (m_pContent)
            m_pContent->setKeepSizeFixed(fixed);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::resetFixedSize() 
{ 
    setFixedSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX); 
    //qDebug() << "resetFixedSize"; 
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::mnuMarkerClick(bool checked)
{
    if (m_pContent)
    {
        m_pContent->m_trackerActive = checked;
        m_pContent->setMouseTracking(checked);

        if (!checked)
        {
            m_lblCoordinates->setText("");
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::mnuCopyToClipboard()
{
    copyToClipBoard();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MatplotlibPlot::copyToClipBoard()
{
    if (m_pContent)
    {
        int dpi = 200;

        if (ito::ITOM_API_FUNCS_GRAPH)
        {
            dpi = qBound(48, apiGetFigureSetting(this, "copyClipboardResolutionDpi", 200, NULL).value<int>(), 2000);
        }

        m_pContent->copyToClipboard(dpi);
        return ito::retOk;
    }
    return ito::RetVal(ito::retError, 0, tr("no content widget available.").toLatin1().data());
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::setContextMenuEnabled(bool show)
{
    if (m_pContent) 
    {
        m_pContent->m_showContextMenu = show;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool MatplotlibPlot::getContextMenuEnabled() const
{
    if (m_pContent)
    {
        return m_pContent->m_showContextMenu;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
//valLeft, valTop, valRight, valBottom, valWSpace, valHSpace are
//fractions of the image width and height multiplied by 1000 (resolution: 0.1%)
void MatplotlibPlot::showSubplotConfig(float valLeft, float valTop, float valRight, float valBottom, float valWSpace, float valHSpace)
{
    if (m_pMatplotlibSubfigConfig == NULL)
    {
        m_pMatplotlibSubfigConfig = new MatplotlibSubfigConfig(valLeft, valTop, valRight, valBottom, valWSpace, valHSpace, this);
        connect((m_pMatplotlibSubfigConfig)->sliderLeftRight(), SIGNAL(minimumValueChanged(double)), this, SLOT(subplotConfigSliderLeftChanged(double)));
        connect((m_pMatplotlibSubfigConfig)->sliderLeftRight(), SIGNAL(maximumValueChanged(double)), this, SLOT(subplotConfigSliderRightChanged(double)));
        connect((m_pMatplotlibSubfigConfig)->sliderBottomTop(), SIGNAL(minimumValueChanged(double)), this, SLOT(subplotConfigSliderBottomChanged(double)));
        connect((m_pMatplotlibSubfigConfig)->sliderBottomTop(), SIGNAL(maximumValueChanged(double)), this, SLOT(subplotConfigSliderTopChanged(double)));
        connect((m_pMatplotlibSubfigConfig)->sliderWSpace(), SIGNAL(valueChanged(double)), this, SLOT(subplotConfigSliderWSpaceChanged(double)));
        connect((m_pMatplotlibSubfigConfig)->sliderHSpace(), SIGNAL(valueChanged(double)), this, SLOT(subplotConfigSliderHSpaceChanged(double)));
        connect((m_pMatplotlibSubfigConfig)->resetButton(), SIGNAL(clicked()), this, SIGNAL(subplotConfigTight()));
        connect((m_pMatplotlibSubfigConfig)->tightButton(), SIGNAL(clicked()), this, SIGNAL(subplotConfigReset()));
    }

    m_pMatplotlibSubfigConfig->setModal(false);
    m_pMatplotlibSubfigConfig->show();
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::modifySubplotSliders(float left, float top, float right, float bottom, float wSpace, float hSpace)
{
    if (m_pMatplotlibSubfigConfig)
    {
        m_pMatplotlibSubfigConfig->modifyValues(left, top, right, bottom, wSpace, hSpace);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::subplotConfigSliderLeftChanged(double value)
{
    emit subplotConfigSliderChanged(0, qRound(value * 10.0));
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::subplotConfigSliderTopChanged(double value)
{
    emit subplotConfigSliderChanged(1, qRound(value * 10.0));
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::subplotConfigSliderRightChanged(double value)
{
    emit subplotConfigSliderChanged(2, qRound(value * 10.0));
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::subplotConfigSliderBottomChanged(double value)
{
    emit subplotConfigSliderChanged(3, qRound(value * 10.0));
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::subplotConfigSliderWSpaceChanged(double value)
{
    emit subplotConfigSliderChanged(4, qRound(value * 10.0));
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::subplotConfigSliderHSpaceChanged(double value)
{
    emit subplotConfigSliderChanged(5, qRound(value * 10.0));
}


//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::replot()
{
    if (m_pContent) 
    {
        m_pContent->replot();
    }
}





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

#include "graphicViewPlot.h"
#include "dObjToQImage.h"
#include <qmessagebox.h>
#include <qfiledialog.h>
#include <qimagewriter.h>
#include <qpainter.h>

//#include <GV_plot_renderer.h>

#include "DataObject/dataObjectFuncs.h"

#include "dialog2DScale.h"

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
GraphicViewPlot::GraphicViewPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    AbstractDObjFigure(itomSettingsFile, windowMode, parent),
    m_pContent(NULL),
    m_pActScaleSetting(NULL),
    m_pActPan(NULL),
    m_pActZoomToRect(NULL),
    m_pActValuePicker(NULL),
    m_pActLineCut(NULL),
    m_pActPalette(NULL),
    m_pActToggleColorBar(NULL),
    m_pActAScan(NULL),
    m_pActPlaneSelector(NULL),
    m_pActCmplxSwitch(NULL),
    m_pMnuCmplxSwitch(NULL),
    m_pActAspectSwitch(NULL),
    m_pMnuAspectSwitch(NULL),
    m_pActColorSwitch(NULL),
    m_pMnuColorSwitch(NULL),
    m_pPaletteRep(NULL),
    m_curPalette(NULL),
    m_lblCoordinates(NULL),
    m_pActProperties(NULL)
{
    
    m_pOutput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, tr("Points for line plots from 2D objects").toLatin1().data()));

    int id = qRegisterMetaType<QSharedPointer<ito::DataObject> >("QSharedPointer<ito::DataObject>");

    //init actions
    createActions();

    //init internal data

    QToolBar *toolbar = new QToolBar(tr("plotting tools"), this);
    addToolBar(toolbar, "mainToolBar");

    // first block is zoom, scale settings, home
    toolbar->addAction(m_pActScaleSetting);
    toolbar->addAction(m_pActPan);
    toolbar->addAction(m_pActZoomToRect);

    // next block get pixel-Info
    toolbar->addSeparator();
    toolbar->addAction(m_pActValuePicker);
    toolbar->addAction(m_pActLineCut);

    m_lblCoordinates = new QLabel(" [0.000; 0.000]\n [0.000; 0.000]", this);
    m_lblCoordinates->setAlignment(Qt::AlignRight | Qt::AlignTop);
    m_lblCoordinates->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_lblCoordinates->setObjectName("lblCoordinates");

    QAction *lblAction = toolbar->addWidget(m_lblCoordinates);
    lblAction->setVisible(true);

    // next block is colorbar
    toolbar->addSeparator();
    toolbar->addAction(m_pActColorSwitch);
    toolbar->addAction(m_pActPalette);
    toolbar->addAction(m_pActToggleColorBar);

    m_pixMap.fromImage(QImage(10, 10, QImage::Format_Indexed8));

    m_pPaletteRep = new QLabel(tr("-     NOIMAGE     -"), this);
    m_pPaletteRep->setPixmap(m_pixMap);
    m_pPaletteRep->setAlignment(Qt::AlignRight | Qt::AlignTop);
    m_pPaletteRep->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_pPaletteRep->setObjectName("Colors-Palette");
    m_curPalette = toolbar->addWidget(m_pPaletteRep);
    m_curPalette->setVisible(false);

    toolbar->addAction(m_pActAspectSwitch);

    // next block is for complex and stacks
    toolbar->addSeparator();
    toolbar->addAction(m_pActAScan);
    toolbar->addAction(m_pActPlaneSelector);
    toolbar->addAction(m_pActCmplxSwitch);   

    QMenu *menuView = new QMenu(tr("View"), this);
    //menuView->addAction(m_pActHome);
    //menuView->addAction(m_pActPan);
    //menuView->addAction(m_pActZoom);
    menuView->addAction(m_pActZoomToRect);
    menuView->addSeparator();
    menuView->addAction(m_pActPalette);
    menuView->addAction(m_pActToggleColorBar);
    menuView->addAction(m_pActColorSwitch);
    menuView->addSeparator();
    //menuView->addAction(m_pActScaleSettings);
    //menuView->addSeparator();
    menuView->addAction(m_pActCmplxSwitch);
    menuView->addSeparator();
    menuView->addAction(m_pActProperties);
    addMenu(menuView); //AbstractFigure takes care of the menu

    QMenu *menuTools = new QMenu(tr("Tools"), this);
    menuTools->addAction(m_pActSave);
    menuTools->addSeparator();
    menuTools->addAction(m_pActValuePicker);
    //menuTools->addAction(m_pActCntrMarker);
    menuTools->addAction(m_pActLineCut);
    //menuTools->addAction(m_pActStackCut);
    menuTools->addSeparator();
    addMenu(menuTools); //AbstractFigure takes care of the menu

    QMenu *contextMenu = new QMenu(tr("plot2D"), this);
    contextMenu->addAction(m_pActSave);
    contextMenu->addSeparator();
    contextMenu->addAction(m_pActHome);
    contextMenu->addAction(m_pActScaleSetting);
    contextMenu->addSeparator();
    contextMenu->addAction(m_pActPan);
    contextMenu->addAction(m_pActZoomToRect);
    contextMenu->addAction(m_pActValuePicker);
    contextMenu->addSeparator();
    contextMenu->addAction(toolbar->toggleViewAction());

    //initialize canvas
    
    m_pContent = new PlotWidget(&m_data, contextMenu, this);
    m_pContent->setObjectName("canvasWidget");
    setCentralWidget(m_pContent);

    //connect(m_pContent, SIGNAL(statusBarClear()), statusBar(), SLOT(clearMessage()));
    //connect(m_pContent, SIGNAL(statusBarMessage(QString,int)), statusBar(), SLOT(showMessage(QString,int)));

    //m_pContent->setFocus();
    //resize(600,400);
    
    setPropertyObservedObject(this);
}

//----------------------------------------------------------------------------------------------------------
void GraphicViewPlot::createActions()
{

    //m_pActHome
    m_pActHome = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"), tr("home"), this);
    m_pActHome->setObjectName("actHome");
    m_pActHome->setToolTip(tr("Reset original view"));
    m_pActHome->setVisible(false);
    connect(m_pActHome, SIGNAL(triggered()), this, SLOT(mnuHome()));

    //m_pActSave
    m_pActSave = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("save..."), this);
    m_pActSave->setObjectName("actSave");
    m_pActSave->setToolTip(tr("Export current view..."));
    connect(m_pActSave, SIGNAL(triggered()), this, SLOT(mnuActSave()));
    
    //m_pActScaleSetting
    m_pActScaleSetting = new QAction(QIcon(":/plots/icons/itom_icons/autoscal.png"), tr("scale settings..."), this);
    m_pActScaleSetting->setObjectName("actScaleSetting");
    m_pActScaleSetting->setToolTip(tr("Set the ranges and offsets of this view..."));
    m_pActScaleSetting->setVisible(false);
    connect(m_pActScaleSetting, SIGNAL(triggered()), this, SLOT(mnuScaleSetting()));


    //m_pActPan
    m_pActPan = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), tr("move"), this);
    m_pActPan->setObjectName("actionPan");
    m_pActPan->setCheckable(true);
    m_pActPan->setChecked(false);
    m_pActPan->setEnabled(false);
    m_pActPan->setToolTip(tr("Pan axes with left mouse, zoom with right"));
    m_pActPan->setVisible(false);
    connect(m_pActPan, SIGNAL(toggled(bool)), this, SLOT(mnuPanner(bool)));


    //m_pActZoomToRect
    m_pActZoomToRect = new QAction(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"), tr("rectangle zoom"), this);
    m_pActZoomToRect->setObjectName("actionZoomToRect");
    m_pActZoomToRect->setCheckable(true);
    m_pActZoomToRect->setChecked(false);
    m_pActZoomToRect->setToolTip(tr("Zoom to rectangle"));
    m_pActZoomToRect->setVisible(false);
    connect(m_pActZoomToRect, SIGNAL(toggled(bool)), this, SLOT(mnuZoomer(bool)));


    //m_pActValuePicker
    m_pActValuePicker = new QAction(QIcon(":/itomDesignerPlugins/general/icons/marker.png"), tr("marker"), this);
    m_pActValuePicker->setObjectName("actionMarker");
    m_pActValuePicker->setCheckable(true);
    m_pActValuePicker->setChecked(false);
    m_pActValuePicker->setToolTip(tr("Show a point marker"));
    connect(m_pActValuePicker, SIGNAL(toggled(bool)), this, SLOT(mnuValuePicker(bool)));

    //m_pActLineCut
    m_pActLineCut = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"), tr("linecut"),this);
    m_pActLineCut->setCheckable(true);
    m_pActLineCut->setObjectName("LineCut");
    m_pActLineCut->setToolTip(tr("Show a in plane line cut"));
    connect(m_pActLineCut, SIGNAL(toggled(bool)), this, SLOT(mnuLinePicker(bool)));  

    //m_pActPalette
    m_pActPalette = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/colorPalette.png"), tr("color palettes"),this);
    m_pActPalette->setObjectName("TogglePalette");
    m_pActPalette->setToolTip(tr("Switch between color palettes"));
    connect(m_pActPalette, SIGNAL(triggered()), this, SLOT(mnuPalette()));   

    //m_pActToggleColorBar
    m_pActToggleColorBar = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/colorbar.png"), tr("color bar"), this);
    m_pActToggleColorBar->setCheckable(true);
    m_pActToggleColorBar->setEnabled(true);
    m_pActToggleColorBar->setObjectName("ShowColorBar");
    m_pActToggleColorBar->setToolTip(tr("Toggle visibility of the color bar name"));
    connect(m_pActToggleColorBar, SIGNAL(toggled(bool)), this, SLOT(mnuColorBar(bool)));

    //m_pActAspectSwitch
    m_pActAspectSwitch = new QAction(QIcon(":/itomDesignerPlugins/aspect/icons/off.png"), tr("zoom level"), this);
    m_pMnuAspectSwitch = new QMenu(tr("zoom level"), this);
    m_pMnuAspectSwitch->addAction(tr("off"));
    m_pMnuAspectSwitch->addAction("1:4");
    m_pMnuAspectSwitch->addAction("1:2");
    m_pMnuAspectSwitch->addAction("1:1");
    m_pMnuAspectSwitch->addAction("2:1");
    m_pMnuAspectSwitch->addAction("4:1");
    m_pActAspectSwitch->setMenu(m_pMnuAspectSwitch);
    m_pActAspectSwitch->setToolTip(tr("Switch between different zoom levels with fixed aspect ration"));
    connect(m_pMnuAspectSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuAspectSwitch(QAction*)));

    //m_pActAScan
    m_pActAScan = new QAction(QIcon(":/plots/icons/itom_icons/1dzdir.png"), tr("slice in z-direction"), this);
    m_pActAScan->setObjectName("a-Scan");
    m_pActAScan->setToolTip(tr("Show a slice through z-Stack"));
    m_pActAScan->setCheckable(true);
    m_pActAScan->setVisible(false);
    connect(m_pActAScan, SIGNAL(toggled(bool)), this, SLOT(mnuAScanPicker(bool)));
    
    //m_pActPlaneSelector
    QSpinBox *planeSelector = new QSpinBox(this);
    planeSelector->setMinimum(0);
    planeSelector->setMaximum(0);
    planeSelector->setValue(0);
    planeSelector->setKeyboardTracking(false);
    planeSelector->setToolTip(tr("Select image plane"));
    QWidgetAction *wa = new QWidgetAction(this);
    wa->setDefaultWidget(planeSelector);
    m_pActPlaneSelector = wa;
    wa->setObjectName("planeSelector");
    wa->setVisible(false);
    connect(planeSelector, SIGNAL(valueChanged(int)), this, SLOT(mnuActPlaneSelector(int)));
    
    //m_pActCmplxSwitch
    m_pActCmplxSwitch = new QAction(QIcon(":/itomDesignerPlugins/complex/icons/ImRe.png"), tr("complex switch"), this);
    m_pMnuCmplxSwitch = new QMenu(tr("Complex Switch"), this);
    m_pMnuCmplxSwitch->addAction(tr("imaginary"));
    m_pMnuCmplxSwitch->addAction(tr("real"));
    m_pMnuCmplxSwitch->addAction(tr("absolute"));
    m_pMnuCmplxSwitch->addAction(tr("phase"));
    m_pActCmplxSwitch->setMenu(m_pMnuCmplxSwitch);
    m_pActCmplxSwitch->setVisible(false);
    m_pActCmplxSwitch->setToolTip(tr("Switch imaginary, real, absolute, phase"));
    connect(m_pMnuCmplxSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuCmplxSwitch(QAction*)));

    //m_pActColorSwitch
    m_pActColorSwitch = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/falseColor.png"), tr("color switch"), this);
    m_pMnuColorSwitch = new QMenu(tr("Color Switch"), this);
    m_pMnuColorSwitch->addAction(tr("autoColor"));
    m_pMnuColorSwitch->addAction(tr("falseColor, bitshift"));
    m_pMnuColorSwitch->addAction(tr("falseColor, scaled"));
    m_pMnuColorSwitch->addAction(tr("Color, 24-Bit"));
    m_pMnuColorSwitch->addAction(tr("Color, 32-Bit"));
    m_pActColorSwitch->setMenu(m_pMnuColorSwitch);
    m_pActColorSwitch->setToolTip(tr("Switch index and direct color mode"));
    connect(m_pMnuColorSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuSwitchColorMode(QAction*)));

    m_pActProperties = this->getPropertyDockWidget()->toggleViewAction();
    connect(m_pActProperties, SIGNAL(triggered(bool)), this, SLOT(mnuShowProperties(bool)));
}
//----------------------------------------------------------------------------------------------------------------------------------
GraphicViewPlot::~GraphicViewPlot()
{
    if (m_pContent)
    {
        m_pContent->deleteLater();
    }

    if (m_pActScaleSetting)
    {
        m_pActScaleSetting->deleteLater();
    }

    if (m_pActPan)
    {
        m_pActPan->deleteLater();
    }

    if (m_pActZoomToRect)
    {
        m_pActZoomToRect->deleteLater();
    }

    if (m_pActValuePicker)
    {
        m_pActValuePicker->deleteLater();
    }

    if (m_pActLineCut)
    {
        m_pActLineCut->deleteLater();
    }

    if (m_pActPalette)
    {
        m_pActPalette->deleteLater();
    }

    if (m_pActToggleColorBar)
    {
        m_pActToggleColorBar->deleteLater();
    }

    if (m_pActAScan)
    {
        m_pActAScan->deleteLater();
    }

    if (m_pActPlaneSelector)
    {
        m_pActPlaneSelector->deleteLater();
    }

    if (m_pActCmplxSwitch)
    {
        m_pActCmplxSwitch->deleteLater();
    }

    if (m_pMnuCmplxSwitch)
    {
        m_pMnuCmplxSwitch->clear();
        m_pMnuCmplxSwitch->deleteLater();
    }

    if (m_pActColorSwitch)
    {
        m_pActColorSwitch->deleteLater();
    }

    if (m_pMnuColorSwitch)
    {
        m_pMnuColorSwitch->clear();
        m_pMnuColorSwitch->deleteLater();
    }

    if (m_pActAspectSwitch)
    {
        m_pActAspectSwitch->deleteLater();
    }

    if (m_pMnuAspectSwitch)
    {
        m_pMnuAspectSwitch->clear();
        m_pMnuAspectSwitch->deleteLater();
    }

    if (m_pPaletteRep)
    {
        m_pPaletteRep->deleteLater();
    }

    if (m_curPalette)
    {
        m_curPalette->deleteLater();
    }

    if (m_lblCoordinates)
    {
        m_lblCoordinates->deleteLater();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GraphicViewPlot::applyUpdate()
{
    m_pOutput["displayed"]->copyValueFrom(m_pInput["source"]);

    if(!m_pContent)
        return ito::retError;

    if(m_pOutput["displayed"]->getType() & ito::ParamBase::DObjPtr)
        ((PlotWidget*)m_pContent)->refreshPlot(m_pOutput["displayed"]->getVal<ito::DataObject*>()); //push the displayed DataObj into the actual plot widget for displaying
    else
        ((PlotWidget*)m_pContent)->refreshPlot(NULL); //push the displayed DataObj into the actual plot widget for displaying

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> GraphicViewPlot::getDisplayed(void)
{
    return QSharedPointer<ito::DataObject>(m_pOutput["displayed"]->getVal<ito::DataObject*>());
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> GraphicViewPlot::getSource(void) const
{
    return QSharedPointer<ito::DataObject>(m_pInput["source"]->getVal<ito::DataObject*>());
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setContextMenuEnabled(bool show)
{
    if (m_pContent)
    {
        ((PlotWidget*)m_pContent)->m_showContextMenu = show;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool GraphicViewPlot::getContextMenuEnabled() const
{
    if (m_pContent)
    {
        return ((PlotWidget*)m_pContent)->m_showContextMenu;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GraphicViewPlot::displayLineCut(QVector<QPointF> bounds, ito::uint32 &uniqueID)
{
    if(ito::ITOM_API_FUNCS_GRAPH == NULL || !m_pContent)
        return ito::retError;

    ito::RetVal retval = ito::retOk;
    QList<QString> paramNames;
    ito::uint32 newUniqueID = uniqueID;
    QWidget *lineCutObj = NULL;

    setOutpBounds(bounds);
    setCoordinates(bounds, true);

    retval += apiGetFigure("DObjLiveLine","",newUniqueID,&lineCutObj,this); //(newUniqueID, "itom1DQwtFigure", &lineCutObj);

    if (uniqueID != newUniqueID)
    {
        uniqueID = newUniqueID;
        ito::AbstractDObjFigure* lineCut = NULL;
        if (lineCutObj->inherits("ito::AbstractDObjFigure"))
            lineCut = (ito::AbstractDObjFigure*)lineCutObj;
        else
            return ito::retError;
        retval += addChannel((ito::AbstractNode*)lineCut, m_pOutput["bounds"], lineCut->getInputParam("bounds"), Channel::parentToChild, 0, 1);
        retval += addChannel((ito::AbstractNode*)lineCut, m_pOutput["displayed"], lineCut->getInputParam("source"), Channel::parentToChild, 0, 1);
        paramNames << "bounds"  << "displayed";
        retval += updateChannels(paramNames);

        lineCut->show();
    }
    else
    {
        paramNames << "bounds"  << "displayed";
        retval += updateChannels(paramNames);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuHome()
{
    QAction temp("", 0);
    this->mnuAspectSwitch(&temp);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuPanner(bool checked)
{
    if (checked)
    {
        m_pActAScan->setChecked(false);
        m_pActZoomToRect->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActValuePicker->setChecked(false);
    }

    if(m_pContent) m_pContent->setState(checked ? PlotWidget::tPan : PlotWidget::tIdle);
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuZoomer(bool checked)
{
    if (checked)
    {
        m_pActAScan->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActValuePicker->setChecked(false);
        m_pActPan->setChecked(false);      
    }

    if(m_pContent) m_pContent->setState(checked ? PlotWidget::tZoom : PlotWidget::tIdle);
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuValuePicker(bool checked)
{
    if (checked)
    {
        m_pActAScan->setChecked(false);
        m_pActZoomToRect->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActPan->setChecked(false);      
    }

    if(m_pContent) m_pContent->setState(checked ? PlotWidget::tValuePicker : PlotWidget::tIdle);
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuPalette()
{
    if(m_pContent) ((PlotWidget*)m_pContent)->setColorMap();
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuAScanPicker(bool checked)
{
    if (checked)
    {
        m_pActPan->setChecked(false);
        m_pActZoomToRect->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActValuePicker->setChecked(false);
    }

    if(m_pContent) m_pContent->setState(checked ? PlotWidget::tStackCut : PlotWidget::tIdle);
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuLinePicker(bool checked)
{
    if (checked)
    {
        m_pActPan->setChecked(false);
        m_pActZoomToRect->setChecked(false);
        m_pActAScan->setChecked(false);
        m_pActValuePicker->setChecked(false);
    }

    if(m_pContent) m_pContent->setState(checked ? PlotWidget::tLineCut : PlotWidget::tIdle);
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuActSave()
{
    static QString saveDefaultPath;

#ifndef QT_NO_PRINTER
    QString fileName = "bode.pdf";
#else
    QString fileName = "bode.png";
#endif

#ifndef QT_NO_FILEDIALOG
    const QList<QByteArray> imageFormats =
        QImageWriter::supportedImageFormats();

    QStringList filter;
    filter += tr("PDF Documents (*.pdf)");
#ifndef GV_NO_SVG
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

    fileName = QFileDialog::getSaveFileName(
        this, tr("Export File Name"), fileName,
        filter.join(";;"), NULL, QFileDialog::DontConfirmOverwrite);
#endif

    if (!fileName.isEmpty())
    {
        bool abort = true;

        QSizeF curSize = m_pContent->size();
        int resolution = 300;

        QFileInfo fi(fileName);
        saveDefaultPath = fi.path();

        exportCanvas(false, fileName, curSize, resolution);
        
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuScaleSetting()
{
/*
    DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(((PlotWidget*)m_pContent)->m_pContent->data());
    double minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0, minZ = 0.0, maxZ = 0.0;
    double minRangeX = 0.0, maxRangeX = 0.0, minRangeY = 0.0, maxRangeY = 0.0;
    int dims = 2, numPlanes = 1, curPlane = 0;
    bool autoCalcX, autoCalcY, autoCalcZ;
    GVInterval interval;

    if (rasterData)
    {
        interval = rasterData->interval(Qt::XAxis);
        minX = interval.minValue();
        maxX = interval.maxValue();
        interval = rasterData->interval(Qt::YAxis);
        minY = interval.minValue();
        maxY = interval.maxValue();
        interval = rasterData->interval(Qt::ZAxis);
        minZ = interval.minValue();
        maxZ = interval.maxValue();

        QSharedPointer<ito::DataObject> dataObj = rasterData->getDataObject();

        if (dataObj != NULL)
        {
            int dims = dataObj->getDims();
            bool test = false;
            minRangeX = dataObj->getPixToPhys(dims - 1, 0.0, test);
            maxRangeX = dataObj->getPixToPhys(dims - 1, dataObj->getSize(dims-1, true), test);
            minRangeY = dataObj->getPixToPhys(dims - 2, 0.0, test);
            maxRangeY = dataObj->getPixToPhys(dims - 2, dataObj->getSize(dims-2, true), test);

            dims = dataObj->getDims();
            if (dims > 2)
            {
                int* wholeSize = new int[dims];
                int* offsets = new int[dims];

                dataObj->locateROI(wholeSize, offsets);

                for (int cntPlane = 0; cntPlane < (dims-2); cntPlane++)
                {
                    numPlanes *= wholeSize[cntPlane];
                }
                curPlane = offsets[dims-3];
                if (dims > 4)
                {
                    for (int cntPlane = 0; cntPlane < (dims-3); cntPlane++)
                    {
                        curPlane += curPlane + offsets[cntPlane] * wholeSize[cntPlane - 1];
                    }
                }
                delete wholeSize;
                delete offsets;
            }

        }
        else
        {
            minRangeX = minX;
            maxRangeX = maxX;
            minRangeY = minY;
            maxRangeY = maxY;
        }
    }
    else
    {
        QMessageBox::warning(this, tr("no data available"), tr("no data object is currently being displayed in this widget."));
    }

    Dialog2DScale *dlg = new Dialog2DScale(minX, maxX, minRangeX, maxRangeX, minY, maxY, minRangeY, maxRangeY, minZ, maxZ, dims, curPlane, numPlanes);
    dlg->exec();
    if (dlg->result() == QDialog::Accepted)
    {
        dlg->getData(minX,maxX,minY,maxY,minZ,maxZ, curPlane, autoCalcX, autoCalcY, autoCalcZ);

        ((PlotWidget*)m_pContent)->setInterval(Qt::XAxis, autoCalcX, minX, maxX);
        ((PlotWidget*)m_pContent)->setInterval(Qt::YAxis, autoCalcY, minY, maxY);
        ((PlotWidget*)m_pContent)->setInterval(Qt::ZAxis, autoCalcZ, minZ, maxZ);
        ((PlotWidget*)m_pContent)->refreshColorMap();
    }

    delete dlg;
    dlg = NULL;
*/
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuColorBar(bool checked)
{
    m_curPalette->setVisible(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuCmplxSwitch(QAction *action)
{
    //GVScaleWidget *rightAxis = axisWidget(GVPlot::yRight);
    //enableAxis(GVPlot::yRight, checked);

    if (m_pContent)
    {
//        RasterToQImageObj* rasterData = static_cast<RasterToQImageObj*>(((PlotWidget*)m_pContent)->m_ObjectContainer);

        if (action->text() == tr("imaginary"))
        {
            m_data.m_cmplxType = RasterToQImageObj::tImag;
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReImag.png"));
        }
        else if (action->text() == tr("real"))
        {
            m_data.m_cmplxType = RasterToQImageObj::tReal;
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReReal.png"));
        }
        else if (action->text() == tr("phase"))
        {
            m_data.m_cmplxType = RasterToQImageObj::tPhase;
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImRePhase.png"));
        }
        else
        {
            m_data.m_cmplxType = RasterToQImageObj::tAbsolute;
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReAbs.png"));
        }
        if(m_pContent)
        {
            if(m_data.m_valueScaleAuto)
            {
            
                QPointF val = m_pContent->calcInterval(Qt::ZAxis);
                m_data.m_valueMin = val.x();
                m_data.m_valueMax = val.y();
            }
            
            ((PlotWidget*)m_pContent)->internalDataUpdated();
            //((PlotWidget*)m_pContent)->refreshPlot(NULL);
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuSwitchColorMode(QAction *action)
{
    if (action->text() == tr("falseColor, bitshift"))
    {
        setColorMode(RasterToQImageObj::ColorIndex8Bitshift);
    }
    else if (action->text() == tr("falseColor, scaled"))
    {
        setColorMode(RasterToQImageObj::ColorIndex8Scaled);
    }
    else if (action->text() == tr("Color, 24-Bit"))
    {
        setColorMode(RasterToQImageObj::ColorRGB24);
    }
    else if (action->text() == tr("Color, 32-Bit"))
    {
        setColorMode(RasterToQImageObj::ColorRGB32);
    }
    else
    {
        setColorMode(RasterToQImageObj::ColorAutoSelect);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuAspectSwitch(QAction *action)
{
    //GVScaleWidget *rightAxis = axisWidget(GVPlot::yRight);
    //enableAxis(GVPlot::yRight, checked);

    if (m_pContent)
    {
        if (action->text() == QString("4:1"))
        {
            m_pActPan->setEnabled(true);
            mnuZoomer(false);
            m_pActZoomToRect->setEnabled(false);

            m_pActAspectSwitch->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio41.png"));
            ((PlotWidget*)m_pContent)->setCanvasZoom(PlotWidget::Ratio4_1);
        }
        else if (action->text() == QString("2:1"))
        {
            m_pActPan->setEnabled(true);
            mnuZoomer(false);
            m_pActZoomToRect->setEnabled(false);

            m_pActAspectSwitch->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio21.png"));
            ((PlotWidget*)m_pContent)->setCanvasZoom(PlotWidget::Ratio2_1);
        }
        else if (action->text() == QString("1:1"))
        {
            m_pActPan->setEnabled(true);
            mnuZoomer(false);
            m_pActZoomToRect->setEnabled(false);

            m_pActAspectSwitch->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio11.png"));
            ((PlotWidget*)m_pContent)->setCanvasZoom(PlotWidget::Ratio1_1);
        }
        else if (action->text() == QString("1:2"))
        {
            m_pActPan->setEnabled(true);
            mnuZoomer(false);
            m_pActZoomToRect->setEnabled(false);

            m_pActAspectSwitch->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio12.png"));
            ((PlotWidget*)m_pContent)->setCanvasZoom(PlotWidget::Ratio1_2);
        }
        else if (action->text() == QString("1:4"))
        {
            m_pActPan->setEnabled(true);
            mnuZoomer(false);
            m_pActZoomToRect->setEnabled(false);

            m_pActAspectSwitch->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio14.png"));
            ((PlotWidget*)m_pContent)->setCanvasZoom(PlotWidget::Ratio1_4);
        }
        else
        {
            m_pActPan->setEnabled(false);
            m_pActZoomToRect->setEnabled(true);

            m_pActAspectSwitch->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/off.png"));
            ((PlotWidget*)m_pContent)->setCanvasZoom(PlotWidget::RatioOff);
        }
    }

}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::enableComplexGUI(const bool checked)
{ 
    m_pActCmplxSwitch->setEnabled(checked);
    m_pActCmplxSwitch->setVisible(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::enableZStackGUI(const bool checked)
{
    m_pActPlaneSelector->setEnabled(checked);
    m_pActPlaneSelector->setVisible(checked);
    //m_pActAScan->setEnabled(checked);
    //m_pActAScan->setVisible(checked);
    m_pActLineCut->setVisible(!checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setCoordinates(const QVector<QPointF> pts, const bool visible)
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
    m_lblCoordinates->setText(buf);
    m_lblCoordinates->setVisible(visible);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool GraphicViewPlot::colorBarVisible() const
{
    return m_pActToggleColorBar->isChecked();
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setColorBarVisible(bool value)
{
    m_pActToggleColorBar->setChecked(value); //emits toggle signal of action
}

//----------------------------------------------------------------------------------------------------------------------------------
QString GraphicViewPlot::getTitle() const
{
    if (m_data.m_autoTitle)
    {
        return "<auto>";
    }
    return m_data.m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setTitle(const QString &title)
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
void GraphicViewPlot::resetTitle()
{
    m_data.m_autoTitle = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString GraphicViewPlot::getxAxisLabel() const
{
    if (m_data.m_autoxAxisLabel)
    {
        return "<auto>";
    }
    return m_data.m_xaxisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setxAxisLabel(const QString &label)
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
void GraphicViewPlot::resetxAxisLabel()
{
    m_data.m_autoxAxisLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString GraphicViewPlot::getyAxisLabel() const
{
    if (m_data.m_autoyAxisLabel)
    {
        return "<auto>";
    }
    return m_data.m_yaxisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setyAxisLabel(const QString &label)
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
void GraphicViewPlot::resetyAxisLabel()
{
    m_data.m_autoyAxisLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString GraphicViewPlot::getValueLabel() const
{
    if (m_data.m_autoValueLabel)
    {
        return "<auto>";
    }
    return m_data.m_valueLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setValueLabel(const QString &label)
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
void GraphicViewPlot::resetValueLabel()
{
    m_data.m_autoValueLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool GraphicViewPlot::getxAxisVisible() const
{
    return m_data.m_xaxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setxAxisVisible(const bool &value)
{
    m_data.m_xaxisVisible = value;

    if (m_pContent) m_pContent->enableAxis(Qt::XAxis, value);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool GraphicViewPlot::getyAxisVisible() const
{
    return m_data.m_yaxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setyAxisVisible(const bool &value)
{
    m_data.m_yaxisVisible = value;

    if (m_pContent) m_pContent->enableAxis(Qt::YAxis, value);
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF GraphicViewPlot::getXAxisInterval(void) const
{   
    if (m_data.m_xaxisScaleAuto && m_pContent)
    {
        return m_pContent->calcInterval(Qt::XAxis);
    }
    return QPointF(m_data.m_xaxisMin, m_data.m_xaxisMax);
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setXAxisInterval(QPointF point)
{
    if (m_pContent)
    {
        emit m_pContent->statusBarMessage( tr("Not implemented yet."), 2000 );
    }
    
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF GraphicViewPlot::getYAxisInterval(void) const
{
    if (m_data.m_yaxisScaleAuto && m_pContent)
    {
        return m_pContent->calcInterval(Qt::YAxis);
    }
    return QPointF(m_data.m_yaxisMin, m_data.m_yaxisMax);
    
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setYAxisInterval(QPointF point)
{

    if (m_pContent)
    {
        emit m_pContent->statusBarMessage( tr("Not implemented yet."), 2000 );
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF GraphicViewPlot::getZAxisInterval(void) const
{
    if (m_data.m_valueScaleAuto && m_pContent)
    {
        return m_pContent->calcInterval(Qt::ZAxis);
    }
    return QPointF(m_data.m_valueMin, m_data.m_valueMax);
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setZAxisInterval(QPointF point)
{
    if(m_data.m_colorMode != RasterToQImageObj::ColorAutoSelect && m_data.m_colorMode != RasterToQImageObj::ColorIndex8Scaled && m_data.m_colorMode != RasterToQImageObj::ColorIndex8Bitshift)
    {
        
        return;
    }

    if(!ito::dObjHelper::isFinite(point.x() || !ito::dObjHelper::isFinite(point.y())) ||
      (!ito::dObjHelper::isNotZero(point.x()) && !ito::dObjHelper::isNotZero(point.y())))
    {
        m_data.m_valueScaleAuto = true;
        if (m_pContent) ((PlotWidget*)m_pContent)->internalDataUpdated();
        return;
    }

    if(!ito::dObjHelper::isNotZero(point.x()) && ito::dObjHelper::isNotZero(point.y()))
    {
        switch((int)point.y())
        {
            case 255:
                //m_data.m_colorMode = RasterToQImageObj::ColorIndex8Bitshift;
                m_data.m_numBits = 8;
                break;
            case 511:
                //m_data.m_colorMode = RasterToQImageObj::ColorIndex8Bitshift;
                m_data.m_numBits = 9;
                break;
            case 1023:
                //m_data.m_colorMode = RasterToQImageObj::ColorIndex8Bitshift;
                m_data.m_numBits = 10;
                break;
            case 2047:
                //m_data.m_colorMode = RasterToQImageObj::ColorIndex8Bitshift;
                m_data.m_numBits = 11;
                break;
            case 4095:
                //m_data.m_colorMode = RasterToQImageObj::ColorIndex8Bitshift;
                m_data.m_numBits = 12;
                break;
            case 8191:
                //m_data.m_colorMode = RasterToQImageObj::ColorIndex8Bitshift;
                m_data.m_numBits = 13;
                break;
            case 16383:
                //m_data.m_colorMode = RasterToQImageObj::ColorIndex8Bitshift;
                m_data.m_numBits = 14;
                break;
            case 65535:
                //m_data.m_colorMode = RasterToQImageObj::ColorIndex8Bitshift;
                m_data.m_numBits = 16;
                break;
            default:
                //m_data.m_colorMode = RasterToQImageObj::ColorAutoSelect;
                break;
        }

    }
    else
    {
        m_data.m_colorMode = RasterToQImageObj::ColorIndex8Scaled;
    }
    m_data.m_valueScaleAuto = false;
    m_data.m_valueMin = point.x();
    m_data.m_valueMax = point.y();
    if (m_pContent) ((PlotWidget*)m_pContent)->internalDataUpdated();
    //if (m_pContent) m_pContent->refreshPlot(NULL);
    
}

//----------------------------------------------------------------------------------------------------------------------------------
QString GraphicViewPlot::getColorMap() const
{
    if (m_pContent)
    {
        return m_pPaletteRep->text();
    }
    return "";
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setColorMap(const QString &name)
{
    if (name != "" && m_pContent)
    {
        m_pContent->setColorMap(name);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont GraphicViewPlot::getTitleFont(void) const
{
    
    if (m_pContent)
    {
        emit m_pContent->statusBarMessage( tr("Not implemented yet."), 2000 );
        //return m_pContent->titleLabel()->font();
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setTitleFont(const QFont &font)
{
    
    if (m_pContent)
    {
        emit m_pContent->statusBarMessage( tr("Not implemented yet."), 2000 );
        //m_pContent->titleLabel()->setFont(font);
        //m_pContent->replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont GraphicViewPlot::getLabelFont(void) const
{
    
    if (m_pContent)
    {
        emit m_pContent->statusBarMessage( tr("Not implemented yet."), 2000 );
        //QwtText t = m_pContent->axisWidget(QwtPlot::xBottom)->title();
        //return t.font();
    }
    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setLabelFont(const QFont &font)
{
    
    
    if (m_pContent)
    {
        emit m_pContent->statusBarMessage( tr("Not implemented yet."), 2000 );
        /*
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
        */
    }
    
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont GraphicViewPlot::getAxisFont(void) const
{
    
    if (m_pContent)
    {
        emit m_pContent->statusBarMessage( tr("Not implemented yet."), 2000 );
        //return m_pContent->axisFont(QwtPlot::xBottom);
    }
    return QFont();
    
}

//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setAxisFont(const QFont &font)
{

    if (m_pContent)
    {
        emit m_pContent->statusBarMessage( tr("Not implemented yet."), 2000 );
        //m_pContent->setAxisFont(QwtPlot::xBottom, font);
        //m_pContent->setAxisFont(QwtPlot::yLeft, font);
        //m_pContent->setAxisFont(QwtPlot::yRight, font);
    }

}

//----------------------------------------------------------------------------------------------------------------------------------
bool GraphicViewPlot::getxAxisFlipped() const
{
    return m_data.m_xaxisFlipped;
}
//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setxAxisFlipped(const bool &value)
{
    m_data.m_xaxisFlipped = value;
    if(m_pContent) m_pContent->updateTransformation();
}
//----------------------------------------------------------------------------------------------------------------------------------
bool GraphicViewPlot::getyAxisFlipped() const
{
    return m_data.m_yaxisFlipped;
}
//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setyAxisFlipped(const bool &value)
    {
    m_data.m_yaxisFlipped = value;
    if(m_pContent) m_pContent->updateTransformation();
}
//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setColorMode(const int type)
{
    if (!m_pContent)
    {
        return;
    }

    if(type < 0)
    {
        emit m_pContent->statusBarMessage( tr("Specified color type out of range [0, 4]."), 2000 );
        return;
    }
    if(type > 4)
    {
        emit m_pContent->statusBarMessage( tr("Specified color type out of range [0, 4]."), 2000 );
        return;
    }

    m_data.m_colorMode = (RasterToQImageObj::tValueType)type;

    switch(m_data.m_colorMode)
    {
        case RasterToQImageObj::ColorIndex8Bitshift:
        {
            QPointF val = m_pContent->calcInterval(Qt::ZAxis);
            m_data.m_valueMin = val.x();
            m_data.m_valueMax = val.y();
            m_data.m_numBits = 8;
            int maxval = (int)(m_data.m_valueMax > 255 ? (int)(m_data.m_valueMax + 0.5) : 255);
            while(maxval > 255)
            {
                m_data.m_numBits++;
                maxval >>= 1;
            }
                 

            m_data.m_valueScaleAuto = false;
            break;
        }
        default:
        case RasterToQImageObj::ColorIndex8Scaled:

        case RasterToQImageObj::ColorAutoSelect:
        {
            if(m_data.m_valueScaleAuto)
            {
            
                QPointF val = m_pContent->calcInterval(Qt::ZAxis);
                m_data.m_valueMin = val.x();
                m_data.m_valueMax = val.y();
            }

            m_pActColorSwitch->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/falseColor.png"));
            m_pActToggleColorBar->setVisible(true);
            m_pActPalette->setVisible(true);
        }
        break;
        case RasterToQImageObj::ColorRGB24:
        case RasterToQImageObj::ColorRGB32:
            m_pActColorSwitch->setIcon(QIcon(":/itomDesignerPlugins/plot/icons/rgba.png"));
            m_pActToggleColorBar->setVisible(false);
            m_pActPalette->setVisible(false);
            if(m_pActToggleColorBar->isCheckable()) m_pActToggleColorBar->setChecked(false);
            break;
    }
    if (m_pContent) ((PlotWidget*)m_pContent)->internalDataUpdated();
    //if(m_pContent) m_pContent->refreshPlot(NULL);  
}
//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::resetColorMode(void)
{
    m_data.m_colorMode = RasterToQImageObj::ColorAutoSelect;
    if(m_pContent)
    {
        if(m_data.m_valueScaleAuto)
        {
            
            QPointF val = m_pContent->calcInterval(Qt::ZAxis);
            m_data.m_valueMin = val.x();
            m_data.m_valueMax = val.y();
        }
        if (m_pContent) ((PlotWidget*)m_pContent)->internalDataUpdated();
        //m_pContent->refreshPlot(NULL); 
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::setPlaneRange(int min, int max)
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
        //m_pActAScan->setVisible((max-min) > 0);
        m_pActLineCut->setVisible((max-min) < 2);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void GraphicViewPlot::mnuActPlaneSelector(int plane)
{
    if (m_pContent) m_pContent->changePlane(plane);

    QStringList paramNames;
    paramNames << "displayed";
    updateChannels(paramNames);
}
//----------------------------------------------------------------------------------------------------------------------------------

ito::RetVal GraphicViewPlot::exportCanvas(const bool exportType, const QString &fileName, QSizeF curSize, const int resolution)
{
    if(!m_pContent)
    {
        return ito::RetVal(ito::retError, 0, tr("Export image failed, canvas handle not initilized").toLatin1().data());
    }
    if(curSize.height() == 0 || curSize.width() == 0)
    {
        curSize = QSizeF(((PlotWidget *)m_pContent)->m_pContent->width(), ((PlotWidget *)m_pContent)->m_pContent->height());
    }
 

    ((PlotWidget *)m_pContent)->repaint();

    int resFaktor = cv::saturate_cast<int>(resolution / 72.0 + 0.5);
    resFaktor = resFaktor < 1 ? 1 : resFaktor;

    QSize myRect(curSize.width() * resFaktor, curSize.height() * resFaktor);
    QImage img(myRect, QImage::Format_ARGB32);

    QPainter painter(&img);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.scale(resFaktor, resFaktor);
    ((PlotWidget *)m_pContent)->m_pContent->render(&painter);
    painter.end();
 
    if(exportType)
    {
        QClipboard *clipboard = QApplication::clipboard();
        clipboard->setImage(img);    
    }
    else img.save(fileName);

    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GraphicViewPlot::copyToClipBoard()
{
    return exportCanvas(true, "");
}
//----------------------------------------------------------------------------------------------------------------------------------
QPixmap GraphicViewPlot::renderToPixMap(const int xsize, const int ysize, const int resolution) 
{
    QSizeF curSize(xsize, ysize);
    if(curSize.height() == 0 || curSize.width() == 0)
    {
        curSize = QSizeF(((PlotWidget *)m_pContent)->m_pContent->width(), ((PlotWidget *)m_pContent)->m_pContent->height());
    }
    QPixmap destinationImage(xsize, ysize);
    if(!m_pContent || !(((PlotWidget *)m_pContent)->m_pContent))
    {
        destinationImage.fill(Qt::red);
        return destinationImage;
    }

    int resFaktor = cv::saturate_cast<int>(resolution / 72.0 + 0.5);
    resFaktor = resFaktor < 1 ? 1 : resFaktor;

    QSize myRect(curSize.width() * resFaktor, curSize.height() * resFaktor);
    QImage img(myRect, QImage::Format_ARGB32);

    QPainter painter(&img);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.scale(resFaktor, resFaktor);

    ((PlotWidget *)m_pContent)->m_pContent->render(&painter);
    painter.end();
    destinationImage.convertFromImage(img);

    return destinationImage;
}
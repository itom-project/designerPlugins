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

#include "itom2DGVFigure.h"
#include "rasterToQImage.h"
#include <qmessagebox.h>
#include <qfiledialog.h>
#include <qimagewriter.h>

//#include <GV_plot_renderer.h>

#include "dialog2DScale.h"

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
itom2DGVFigure::itom2DGVFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    AbstractDObjFigure(itomSettingsFile, windowMode, parent),
    m_pContent(NULL),
    m_actScaleSetting(NULL),
    m_actPan(NULL),
    m_actZoomToRect(NULL),
    m_actMarker(NULL),
    m_actLineCut(NULL),
    m_actPalette(NULL),
    m_actToggleColorBar(NULL),
    m_actColorDisplay(NULL),
    m_actGrayDisplay(NULL),
    m_actAScan(NULL),
    m_actForward(NULL),
    m_actBack(NULL),
	m_actCmplxSwitch(NULL),
	m_mnuCmplxSwitch(NULL),
	m_actAspectSwitch(NULL),
	m_mnuAspectSwitch(NULL),
    m_pPaletteRep(NULL),
    m_curPalette(NULL),
	m_lblCoordinates(NULL)
{
    m_pOutput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for line plots from 2d objects").toAscii().data()));

    int id = qRegisterMetaType<QSharedPointer<ito::DataObject> >("QSharedPointer<ito::DataObject>");

    //m_actHome
    m_actHome = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"), tr("Home"), this);
    m_actHome->setObjectName("actHome");
    m_actHome->setToolTip("Reset original view");

	//m_actSave
    m_actSave = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"),tr("Save"), this);
    m_actSave->setObjectName("actSave");
    m_actSave->setToolTip("Export current view");

    //m_actScaleSetting
    m_actScaleSetting = new QAction(QIcon(":/plots/icons/itom_icons/autoscal.png"),tr("Scale Settings"), this);
    m_actScaleSetting->setObjectName("actScaleSetting");
    m_actScaleSetting->setToolTip("Set the ranges and offsets oif this view");

    //m_actPan
    m_actPan = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), QObject::tr("move"), this);
    m_actPan->setObjectName("actionPan");
    m_actPan->setCheckable(true);
    m_actPan->setChecked(false);
    m_actPan->setEnabled(false);
    m_actPan->setToolTip("Pan axes with left mouse, zoom with right");

    //m_actZoomToRect
    m_actZoomToRect = new QAction(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"), QObject::tr("zoom to rectangle"), this);
    m_actZoomToRect->setObjectName("actionZoomToRect");
    m_actZoomToRect->setCheckable(true);
    m_actZoomToRect->setChecked(false);
    m_actZoomToRect->setToolTip("Zoom to rectangle");

    //m_actMarker
    m_actMarker = new QAction(QIcon(":/itomDesignerPlugins/general/icons/marker.png"), QObject::tr("marker"), this);
    m_actMarker->setObjectName("actionMarker");
    m_actMarker->setCheckable(true);
    m_actMarker->setChecked(false);

    //m_actLineCut
    m_actLineCut = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"),tr("Linecut"),this);
    m_actLineCut->setCheckable(true);
    m_actLineCut->setObjectName("LineCut");
    m_actLineCut->setToolTip("Show a in plane line cut");

    //m_actPalette
    m_actPalette = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/colorPalette.png"),tr("Palette"),this);
    m_actPalette->setObjectName("TogglePalette");
    m_actPalette->setToolTip("Switch between color palettes");

    //m_actToggleColorBar
    m_actToggleColorBar = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/colorbar.png"),tr("Show Colorbar"), this);
    m_actToggleColorBar->setCheckable(true);
    m_actToggleColorBar->setEnabled(true);
    m_actToggleColorBar->setObjectName("ShowColorBar");
    m_actToggleColorBar->setToolTip("Toggle visibility of the color bar on right canvas side");

    //m_actAspectSwitch
    m_actAspectSwitch = new QAction(QIcon(":/itomDesignerPlugins/aspect/icons/off.png"),tr("Switch between different aspect ratios"), this);
	m_mnuAspectSwitch = new QMenu("Aspect Switch");
    m_mnuAspectSwitch->addAction(tr("off"));
    m_mnuAspectSwitch->addAction(tr("1:4"));
	m_mnuAspectSwitch->addAction(tr("1:2"));
	m_mnuAspectSwitch->addAction(tr("1:1"));
	m_mnuAspectSwitch->addAction(tr("2:1"));
	m_mnuAspectSwitch->addAction(tr("4:1"));
	m_actAspectSwitch->setMenu(m_mnuAspectSwitch);

    //m_actAScan
    m_actAScan = new QAction(QIcon(":/plots/icons/itom_icons/1dzdir.png"),tr("Slice in z-direction"),this);
    m_actAScan->setObjectName("a-Scan");
    m_actAScan->setToolTip("Show a slice through z-Stack");
    m_actAScan->setCheckable(true);
    m_actAScan->setVisible(false);

    //m_actForward
    m_actForward = new QAction(QIcon(":/itomDesignerPlugins/general/icons/forward.png"), tr("forward"), this);
    m_actForward->setObjectName("actionForward");
    m_actForward->setVisible(false);
    m_actForward->setToolTip("Forward to next plane");

    //m_actBack
    m_actBack = new QAction(QIcon(":/itomDesignerPlugins/general/icons/back.png"), tr("back"), this);
    m_actBack->setObjectName("actionBack");
    m_actBack->setVisible(false);
    m_actBack->setToolTip("Back to previous plane");
    
    //m_actCmplxSwitch
    m_actCmplxSwitch = new QAction(QIcon(":/itomDesignerPlugins/complex/icons/ImRe.png"),tr("Switch Imag, Real, Abs, Pha"), this);
	m_mnuCmplxSwitch = new QMenu("Complex Switch");
	m_mnuCmplxSwitch->addAction(tr("Imag"));
	m_mnuCmplxSwitch->addAction(tr("Real"));
	m_mnuCmplxSwitch->addAction(tr("Abs"));
	m_mnuCmplxSwitch->addAction(tr("Pha"));
	m_actCmplxSwitch->setMenu(m_mnuCmplxSwitch);
    m_actCmplxSwitch->setVisible(false);

    connect(m_actSave, SIGNAL(triggered()), this, SLOT(mnuExport()));
    connect(m_actHome, SIGNAL(triggered()), this, SLOT(mnuHome()));

    connect(m_actScaleSetting, SIGNAL(triggered()), this, SLOT(mnuScaleSetting()));
    connect(m_actPan, SIGNAL(toggled(bool)), this, SLOT(mnuPanner(bool)));
    connect(m_actZoomToRect, SIGNAL(toggled(bool)), this, SLOT(mnuZoomer(bool)));
    connect(m_actMarker, SIGNAL(toggled(bool)), this, SLOT(mnuValuePicker(bool)));
    connect(m_actLineCut, SIGNAL(toggled(bool)), this, SLOT(mnuLinePicker(bool)));  

    connect(m_actPalette, SIGNAL(triggered()), this, SLOT(mnuPalette()));   
    connect(m_actToggleColorBar, SIGNAL(toggled(bool)), this, SLOT(mnuColorBar(bool)));
    
    connect(m_actAScan, SIGNAL(toggled(bool)), this, SLOT(mnuAScanPicker(bool)));
	connect(m_mnuCmplxSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuCmplxSwitch(QAction*)));
    connect(m_mnuAspectSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuAspectSwitch(QAction*)));

	QToolBar *toolbar = new QToolBar(this);
	addToolBar(toolbar, "mainToolBar");

	QMenu *contextMenu = new QMenu(QObject::tr("plot2D"), this);
    contextMenu->addAction(m_actSave);
    contextMenu->addSeparator();
    contextMenu->addAction(m_actHome);
    contextMenu->addAction(m_actScaleSetting);
    contextMenu->addSeparator();
    contextMenu->addAction(m_actPan);
    contextMenu->addAction(m_actZoomToRect);
    contextMenu->addAction(m_actMarker);
    contextMenu->addSeparator();
    contextMenu->addAction(toolbar->toggleViewAction());

    // first block is zoom, scale settings, home
    toolbar->addAction(m_actScaleSetting);
    toolbar->addAction(m_actPan);
    toolbar->addAction(m_actZoomToRect);

    // next block get pixel-Info
    toolbar->addSeparator();
    toolbar->addAction(m_actMarker);
    toolbar->addAction(m_actLineCut);

    m_lblCoordinates = new QLabel(" [0.000; 0.000]\n [0.000; 0.000]", this);
    m_lblCoordinates->setAlignment( Qt::AlignRight | Qt::AlignTop);
    m_lblCoordinates->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_lblCoordinates->setObjectName("lblCoordinates");

    QAction *lblAction = toolbar->addWidget(m_lblCoordinates);
    lblAction->setVisible(true);

    // next block is colorbar
    toolbar->addSeparator();
    toolbar->addAction(m_actPalette);
    toolbar->addAction(m_actToggleColorBar);

    m_pixMap.fromImage(QImage(10, 10, QImage::Format_Indexed8));

    m_pPaletteRep = new QLabel("-     NOIMAGE     -", this);
    m_pPaletteRep->setPixmap(m_pixMap);

    //m_pPaletteRep->setGeometry(0, 0, 128, 12);
    m_pPaletteRep->setAlignment( Qt::AlignRight | Qt::AlignTop);
    m_pPaletteRep->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    m_pPaletteRep->setObjectName("Colors-Palette");

    m_curPalette = toolbar->addWidget(m_pPaletteRep);
    m_curPalette->setVisible(false);

    toolbar->addAction(m_actAspectSwitch);

    // next block is for complex and stacks
    toolbar->addSeparator();
    toolbar->addAction(m_actAScan);
    toolbar->addAction(m_actBack);
    toolbar->addAction(m_actForward);
    toolbar->addAction(m_actCmplxSwitch);

    m_pContent = new plot2DWidget(contextMenu, this);
    m_pContent->setObjectName("canvasWidget");
    
    setCentralWidget(m_pContent);
    m_pContent->setFocus();
    resize(600,400);
}

//----------------------------------------------------------------------------------------------------------------------------------
itom2DGVFigure::~itom2DGVFigure()
{
    if (m_pContent)
    {
        m_pContent->deleteLater();
    }

    if (m_actScaleSetting)
    {
        delete m_actScaleSetting;
    }

    if (m_actPan)
    {
        delete m_actPan;
    }

    if (m_actZoomToRect)
    {
        delete m_actZoomToRect;
    }

    if (m_actMarker)
    {
        delete m_actMarker;
    }

    if (m_actLineCut)
    {
        delete m_actLineCut;
    }

    if (m_actPalette)
    {
        delete m_actPalette;
    }

    if (m_actToggleColorBar)
    {
        delete m_actToggleColorBar;
    }

    if (m_actColorDisplay)
    {
        delete m_actColorDisplay;
    }

    if (m_actGrayDisplay)
    {
        delete m_actGrayDisplay;
    }

    if (m_actAScan)
    {
        delete m_actAScan;
    }

    if (m_actForward)
    {
        delete m_actForward;
    }

    if (m_actBack)
    {
        delete m_actBack;
    }

    if (m_actCmplxSwitch)
    {
        delete m_actCmplxSwitch;
    }

    if (m_mnuCmplxSwitch)
    {
        m_mnuCmplxSwitch->clear();
        delete m_mnuCmplxSwitch;
    }

    if (m_actAspectSwitch)
    {
        delete m_actAspectSwitch;
    }

    if (m_mnuAspectSwitch)
    {
        m_mnuAspectSwitch->clear();
        delete m_mnuAspectSwitch;
    }

    if (m_pPaletteRep)
    {
        delete m_pPaletteRep;
    }

    if (m_curPalette)
    {
        delete m_curPalette;
    }

    if (m_lblCoordinates)
    {
        delete m_lblCoordinates;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal itom2DGVFigure::applyUpdate()
{
    m_pOutput["displayed"]->copyValueFrom(m_pInput["source"]);
    ((plot2DWidget*)m_pContent)->refreshPlot(m_pOutput["displayed"]); //push the displayed DataObj into the actual plot widget for displaying

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> itom2DGVFigure::getDisplayed(void)
{
    return QSharedPointer<ito::DataObject>(m_pOutput["displayed"]->getVal<ito::DataObject*>());
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> itom2DGVFigure::getSource(void) const
{
    return QSharedPointer<ito::DataObject>(m_pInput["source"]->getVal<ito::DataObject*>());
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::setContextMenuEnabled(bool show)
{
    if(m_pContent) ((plot2DWidget*)m_pContent)->m_showContextMenu = show;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool itom2DGVFigure::getContextMenuEnabled() const
{
    if(m_pContent) return ((plot2DWidget*)m_pContent)->m_showContextMenu;
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal itom2DGVFigure::displayLineCut(QVector<QPointF> bounds, ito::uint32 &uniqueID)
{
    ito::RetVal retval = ito::retOk;
    QList<QString> paramNames;
    ito::uint32 newUniqueID = uniqueID;
    QWidget *lineCutObj = NULL;

    setOutpBounds(bounds);
    setLinePlotCoordinates(bounds);

    retval += apiGetFigure("DObjLiveLine","",newUniqueID,&lineCutObj,this); //(newUniqueID, "itom1DQwtFigure", &lineCutObj);

    if(uniqueID != newUniqueID)
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
void itom2DGVFigure::mnuHome()
{
    QAction temp("", 0);
    this->mnuAspectSwitch(&temp);
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::mnuPanner(bool checked)
{
    if(checked)
    {
        m_actAScan->setChecked(false);
        m_actZoomToRect->setChecked(false);
        m_actLineCut->setChecked(false);
        m_actMarker->setChecked(false);
//        ((plot2DWidget*)m_pContent)->m_pPanner->setEnabled(true);
    }
    else
    {
        m_actPan->setChecked(false);
//        ((plot2DWidget*)m_pContent)->m_pPanner->setEnabled(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::mnuZoomer(bool checked)
{
    if(checked)
    {
        m_actAScan->setChecked(false);
        m_actLineCut->setChecked(false);
        m_actMarker->setChecked(false);
        m_actPan->setChecked(false);
//        ((plot2DWidget*)m_pContent)->m_pValuePicker->setEnabled(true);        
    }
    else
    {
        m_actZoomToRect->setChecked(false);
//        ((plot2DWidget*)m_pContent)->m_pValuePicker->setEnabled(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::mnuValuePicker(bool checked)
{

    if(checked)
    {
        m_actAScan->setChecked(false);
        m_actZoomToRect->setChecked(false);
        m_actLineCut->setChecked(false);
        m_actPan->setChecked(false);
        ((plot2DWidget*)m_pContent)->enableMarker(true);
    }
    else
    {
        m_actMarker->setChecked(false);
        ((plot2DWidget*)m_pContent)->enableMarker(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::mnuPalette()
{
    ((plot2DWidget*)m_pContent)->m_paletteNum++;
    ((plot2DWidget*)m_pContent)->refreshColorMap();
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::mnuAScanPicker(bool checked)
{
    if(checked && m_pContent && ((plot2DWidget*)m_pContent)->getStackStatus())
    {
        m_actPan->setChecked(false);
        m_actZoomToRect->setChecked(false);
        m_actLineCut->setChecked(false);
        m_actMarker->setChecked(false);
//        ((plot2DWidget*)m_pContent)->m_pAScanPicker->setEnabled(true);
//        ((plot2DWidget*)m_pContent)->m_pAScanMarker->setVisible(true);
    }
    else
    {
        m_actAScan->setChecked(false);
        //((plot2DWidget*)m_pContent)->m_pAScanPicker->setEnabled(false);
        //((plot2DWidget*)m_pContent)->m_pAScanMarker->setVisible(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::mnuLinePicker(bool checked)
{
    if(checked)
    {
        m_actPan->setChecked(false);
        m_actZoomToRect->setChecked(false);
        m_actAScan->setChecked(false);
        m_actMarker->setChecked(false);
        ((plot2DWidget*)m_pContent)->enableLinePointer(true);
    }
    else
    {
        m_actLineCut->setChecked(false);
        ((plot2DWidget*)m_pContent)->enableLinePointer(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::mnuExport()
{
#ifndef QT_NO_PRINTER
    QString fileName = "bode.pdf";
#else
    QString fileName = "bode.png";
#endif

#ifndef QT_NO_FILEDIALOG
    const QList<QByteArray> imageFormats =
        QImageWriter::supportedImageFormats();

    QStringList filter;
    filter += "PDF Documents (*.pdf)";
#ifndef GV_NO_SVG
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

/*
    if ( !fileName.isEmpty() )
    {
        GVPlotRenderer renderer;

        // flags to make the document look like the widget
        renderer.setDiscardFlag(GVPlotRenderer::DiscardBackground, false);
        renderer.setLayoutFlag(GVPlotRenderer::KeepFrames, true);

        renderer.renderDocument(((plot2DWidget*)m_pContent), fileName, QSizeF(300, 200), 85);
    }
*/
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::mnuScaleSetting()
{
/*
    DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(((plot2DWidget*)m_pContent)->m_pContent->data());
    double minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0, minZ = 0.0, maxZ = 0.0;
    double minRangeX = 0.0, maxRangeX = 0.0, minRangeY = 0.0, maxRangeY = 0.0;
    int dims = 2, numPlanes = 1, curPlane = 0;
    bool autoCalcX, autoCalcY, autoCalcZ;
    GVInterval interval;

    if(rasterData)
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

        if(dataObj != NULL)
        {
            int dims = dataObj->getDims();
            bool test = false;
            minRangeX = dataObj->getPixToPhys(dims - 1, 0.0, test);
            maxRangeX = dataObj->getPixToPhys(dims - 1, dataObj->getSize(dims-1, true), test);
            minRangeY = dataObj->getPixToPhys(dims - 2, 0.0, test);
            maxRangeY = dataObj->getPixToPhys(dims - 2, dataObj->getSize(dims-2, true), test);

            dims = dataObj->getDims();
            if(dims > 2)
            {
                int* wholeSize = new int[dims];
                int* offsets = new int[dims];

                dataObj->locateROI(wholeSize, offsets);

                for(int cntPlane = 0; cntPlane < (dims-2); cntPlane++)
                {
                    numPlanes *= wholeSize[cntPlane];
                }
                curPlane = offsets[dims-3];
                if(dims > 4)
                {
                    for(int cntPlane = 0; cntPlane < (dims-3); cntPlane++)
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
    if(dlg->result() == QDialog::Accepted)
    {
        dlg->getData(minX,maxX,minY,maxY,minZ,maxZ, curPlane, autoCalcX, autoCalcY, autoCalcZ);

        ((plot2DWidget*)m_pContent)->setInterval(Qt::XAxis, autoCalcX, minX, maxX);
        ((plot2DWidget*)m_pContent)->setInterval(Qt::YAxis, autoCalcY, minY, maxY);
        ((plot2DWidget*)m_pContent)->setInterval(Qt::ZAxis, autoCalcZ, minZ, maxZ);
        ((plot2DWidget*)m_pContent)->refreshColorMap();
    }

    delete dlg;
    dlg = NULL;
*/
}
//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::mnuColorBar(bool checked)
{
    m_curPalette->setVisible(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::mnuCmplxSwitch(QAction *action)
{
    //GVScaleWidget *rightAxis = axisWidget(GVPlot::yRight);
    //enableAxis(GVPlot::yRight, checked);

	if (m_pContent)
	{
		RasterToQImageObj* rasterData = static_cast<RasterToQImageObj*>(((plot2DWidget*)m_pContent)->m_ObjectContainer);

		if (action->text() == QString("Imag"))
        {
			rasterData->setCmplxState(1);
            m_actCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReImag.png"));
        }
		else if (action->text() == QString("Real"))
        {
			rasterData->setCmplxState(2);
            m_actCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReReal.png"));
        }
		else if (action->text() == QString("Pha"))
        {
			rasterData->setCmplxState(3);
            m_actCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImRePhase.png"));
        }
		else
        {
			rasterData->setCmplxState(0);
            m_actCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReAbs.png"));
        }
		rasterData->setIntervalRange(Qt::ZAxis, true, 0,0);
        ((plot2DWidget*)m_pContent)->refreshPlot(NULL);
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::mnuAspectSwitch(QAction *action)
{
    //GVScaleWidget *rightAxis = axisWidget(GVPlot::yRight);
    //enableAxis(GVPlot::yRight, checked);

	if (m_pContent)
	{
        if (action->text() == QString("4:1"))
        {
            m_actPan->setEnabled(true);
            mnuZoomer(false);
            m_actZoomToRect->setEnabled(false);

            m_actAspectSwitch->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio41.png"));
            ((plot2DWidget*)m_pContent)->setCanvasZoom(plot2DWidget::Ratio4_1);
        }
		else if (action->text() == QString("2:1"))
        {
            m_actPan->setEnabled(true);
            mnuZoomer(false);
            m_actZoomToRect->setEnabled(false);

            m_actAspectSwitch->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio21.png"));
			((plot2DWidget*)m_pContent)->setCanvasZoom(plot2DWidget::Ratio2_1);
        }
		else if (action->text() == QString("1:1"))
        {
            m_actPan->setEnabled(true);
            mnuZoomer(false);
            m_actZoomToRect->setEnabled(false);

            m_actAspectSwitch->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio11.png"));
			((plot2DWidget*)m_pContent)->setCanvasZoom(plot2DWidget::Ratio1_1);
        }
		else if (action->text() == QString("1:2"))
        {
            m_actPan->setEnabled(true);
            mnuZoomer(false);
            m_actZoomToRect->setEnabled(false);

            m_actAspectSwitch->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio12.png"));
			((plot2DWidget*)m_pContent)->setCanvasZoom(plot2DWidget::Ratio1_2);
        }
		else if (action->text() == QString("1:4"))
        {
            m_actPan->setEnabled(true);
            mnuZoomer(false);
            m_actZoomToRect->setEnabled(false);

            m_actAspectSwitch->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio14.png"));
			((plot2DWidget*)m_pContent)->setCanvasZoom(plot2DWidget::Ratio1_4);
        }
		else
        {
            m_actPan->setEnabled(false);
            m_actZoomToRect->setEnabled(true);

            m_actAspectSwitch->setIcon(QIcon(":/itomDesignerPlugins/aspect/icons/off.png"));
			((plot2DWidget*)m_pContent)->setCanvasZoom(plot2DWidget::RatioOff);
        }
	}

}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::mnuSwitchColorMode(QAction *action)
{
	if (m_pContent)
	{
        if (action->text() == QString("RGB"))
        {

        }
		else if (action->text() == QString("Index8"))
        {

        }
		else if (action->text() == QString("Idx8"))
        {

        }
	}
}

//----------------------------------------------------------------------------------------------------------------------------------        
QPointF itom2DGVFigure::getZAxisInterval(void) const
{ 
    return ((plot2DWidget*)m_pContent)->m_startRangeZ;
}

//----------------------------------------------------------------------------------------------------------------------------------        
void itom2DGVFigure::setZAxisInterval(QPointF interval) 
{ 
    ((plot2DWidget*)m_pContent)->setInterval(Qt::ZAxis, 0, interval.x(), interval.y());
    return; 
}

//----------------------------------------------------------------------------------------------------------------------------------        
QString itom2DGVFigure::getColorMap(void) const
{ 
    return QString(); 
}

//----------------------------------------------------------------------------------------------------------------------------------        
void itom2DGVFigure::setColorMap(QString palette) 
{ 
    ((plot2DWidget*)m_pContent)->refreshColorMap(palette);
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::enableComplexGUI(const bool checked)
{ 
    m_actCmplxSwitch->setEnabled(checked);
    m_actCmplxSwitch->setVisible(checked);
}
//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::enableZStackGUI(const bool checked)
{
    m_actBack->setEnabled(checked);
    m_actBack->setVisible(checked);
    m_actForward->setEnabled(checked);
    m_actForward->setVisible(checked);
    m_actAScan->setEnabled(checked);
    m_actAScan->setVisible(checked);
}
//----------------------------------------------------------------------------------------------------------------------------------
void itom2DGVFigure::setLinePlotCoordinates(const QVector<QPointF> pts)
{
    char buf[60] = {0};
    if(pts.size() > 1)
    {
        sprintf(buf, "[%.4g; %.4g]\n[%.4g; %.4g]", pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
    }
    else if(pts.size() == 1)
    {
        sprintf(buf, "[%.4g; %.4g]\n[ - ; - ]", pts[0].x(), pts[0].y());
    }
    else
    {
        sprintf(buf, "[ - ; - ]\n[ - ; - ]");
    }
    m_lblCoordinates->setText(buf);
}



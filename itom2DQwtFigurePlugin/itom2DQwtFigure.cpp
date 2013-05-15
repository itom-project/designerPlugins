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

#include "itom2DQwtFigure.h"
#include <qmessagebox.h>
#include <qfiledialog.h>
#include <qimagewriter.h>

#include <qwt_plot_renderer.h>

#include "dialog2DScale.h"

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
itom2DQwtFigure::itom2DQwtFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    AbstractDObjFigure(itomSettingsFile, windowMode, parent),
    m_pContent(NULL),
    m_actScaleSetting(NULL),
    m_actPan(NULL),
    m_actZoomToRect(NULL),
    m_actMarker(NULL),
    m_actLineCut(NULL),
    m_actPalette(NULL),
    m_actToggleColorBar(NULL),
    m_actAScan(NULL),
    m_actForward(NULL),
    m_actBack(NULL),
	m_actCmplxSwitch(NULL),
	m_mnuCmplxSwitch(NULL),
	m_lblCoordinates(NULL)
{
    m_pOutput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for line plots from 2d objects").toAscii().data()));
    m_pOutput.insert("sourceout", new ito::Param("sourceout", ito::ParamBase::DObjPtr, NULL, QObject::tr("shallow copy pass through of input source object").toAscii().data()));

    int id = qRegisterMetaType<QSharedPointer<ito::DataObject> >("QSharedPointer<ito::DataObject>");

    //m_actHome
    m_actHome = new QAction(QIcon(":/itom2DQwtFigurePlugin/icons/home.png"),tr("Home"), this);
    m_actHome->setObjectName("actHome");
    m_actHome->setToolTip("Reset original view");

	//m_actSave
    m_actSave = new QAction(QIcon(":/itom2DQwtFigurePlugin/icons/filesave.png"),tr("Save"), this);
    m_actSave->setObjectName("actSave");
    m_actSave->setToolTip("Export current view");

    //m_actScaleSetting
    m_actScaleSetting = new QAction(QIcon(":/plots/icons/itom_icons/autoscal.png"),tr("Scale Settings"), this);
    m_actScaleSetting->setObjectName("actScaleSetting");
    m_actScaleSetting->setToolTip("Set the ranges and offsets oif this view");

    //m_actPan
    m_actPan = new QAction(QIcon(":/matplotlibFigure/icons/move.png"), QObject::tr("move"), this);
    m_actPan->setObjectName("actionPan");
    m_actPan->setCheckable(true);
    m_actPan->setChecked(false);
    m_actPan->setToolTip("Pan axes with left mouse, zoom with right");

    //m_actZoomToRect
    m_actZoomToRect = new QAction(QIcon(":/matplotlibFigure/icons/zoom_to_rect.png"), QObject::tr("zoom to rectangle"), this);
    m_actZoomToRect->setObjectName("actionZoomToRect");
    m_actZoomToRect->setCheckable(true);
    m_actZoomToRect->setChecked(false);
    m_actZoomToRect->setToolTip("Zoom to rectangle");

    //m_actMarker
    m_actMarker = new QAction(QIcon(":/matplotlibFigure/icons/marker.png"), QObject::tr("marker"), this);
    m_actMarker->setObjectName("actionMarker");
    m_actMarker->setCheckable(true);
    m_actMarker->setChecked(false);

    //m_actLineCut
    m_actLineCut = new QAction(QIcon(":/plots/icons/itom_icons/pntline.png"),tr("Linecut"),this);
    m_actLineCut->setCheckable(true);
    m_actLineCut->setObjectName("LineCut");
    m_actLineCut->setToolTip("Show a in plane line cut");

    //m_actPalette
    m_actPalette = new QAction(QIcon(":/plots/icons/itom_icons/color.png"),tr("Palette"),this);
    m_actPalette->setObjectName("TogglePalette");
    m_actPalette->setToolTip("Switch between color palettes");

    //m_actToggleColorBar
    m_actToggleColorBar = new QAction(QIcon(":/plots/icons/itom_icons/colorbar.png"),tr("Show Colorbar"), this);
    m_actToggleColorBar->setCheckable(true);
    m_actToggleColorBar->setObjectName("ShowColorBar");
    m_actToggleColorBar->setToolTip("Toggle visibility of the color bar on right canvas side");

    //m_actAScan
    m_actAScan = new QAction(QIcon(":/plots/icons/itom_icons/1dzdir.png"),tr("Slice in z-direction"),this);
    m_actAScan->setObjectName("a-Scan");
    m_actAScan->setToolTip("Show a slice through z-Stack");
    m_actAScan->setCheckable(true);
    m_actAScan->setVisible(false);

    //m_actForward
    m_actForward = new QAction(QIcon(":/itom2DQwtFigurePlugin/icons/forward.png"), tr("forward"), this);
    m_actForward->setObjectName("actionForward");
    m_actForward->setVisible(false);
    m_actForward->setToolTip("Forward to next plane");

    //m_actBack
    m_actBack = new QAction(QIcon(":/itom2DQwtFigurePlugin/icons/back.png"), tr("back"), this);
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

    connect(m_actForward, SIGNAL(triggered()), this, SLOT(mnuForward()));
    connect(m_actBack, SIGNAL(triggered()), this, SLOT(mnuBack()));

    //this->menuBar()->addAction(m_actPan);
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

	toolbar->addAction(m_actSave);
	toolbar->addSeparator();

    // first block is zoom, scale settings, home
    toolbar->addAction(m_actHome);
    toolbar->addAction(m_actPan);
    toolbar->addAction(m_actZoomToRect);
	toolbar->addAction(m_actScaleSetting);

    // next block get pixel-Info
    toolbar->addSeparator();
    toolbar->addAction(m_actMarker);
    toolbar->addAction(m_actLineCut);

    m_lblCoordinates = new QLabel("[0.0; 0.0]\n[0.0; 0.0]", this);
    m_lblCoordinates->setAlignment( Qt::AlignRight | Qt::AlignTop);
    m_lblCoordinates->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    m_lblCoordinates->setObjectName("lblCoordinates");

    QAction *lblAction = toolbar->addWidget(m_lblCoordinates);
    lblAction->setVisible(true);

    // next block is colorbar
    toolbar->addSeparator();
    toolbar->addAction(m_actPalette);
    toolbar->addAction(m_actToggleColorBar);

    // next block is for complex and stacks
    toolbar->addSeparator();
    toolbar->addAction(m_actAScan);
    toolbar->addAction(m_actBack);
    toolbar->addAction(m_actForward);
    toolbar->addAction(m_actCmplxSwitch);


    m_pContent = new Plot2DWidget(contextMenu, this);
    m_pContent->setObjectName("canvasWidget");
    
    setCentralWidget(m_pContent);
    m_pContent->setFocus();
}

//----------------------------------------------------------------------------------------------------------------------------------
itom2DQwtFigure::~itom2DQwtFigure()
{
    if(m_pContent)
    {
        delete m_pContent;
        m_pContent = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal itom2DQwtFigure::applyUpdate()
{

    // inserted shallow a shallow copy for the source -> displayed transision and an additional sourceout for the z-stack linecut
    // maybe Mark will do some changes here?? ck 05/15/2013
    // anyway z-stack linecut is working currently ;-)

//    m_pOutput["displayed"]->copyValueFrom(m_pInput["source"]);

    DataObject *tmpDObj = NULL;
    if ((tmpDObj = (ito::DataObject*)m_pOutput["displayed"]->getVal<void*>()))
    {
        delete tmpDObj;
    }
    tmpDObj = new DataObject(*(ito::DataObject*)m_pInput["source"]->getVal<void*>());
    m_pOutput["displayed"]->setVal<void*>(tmpDObj);

    if ((tmpDObj = (ito::DataObject*)m_pOutput["sourceout"]->getVal<void*>()))
    {
        delete tmpDObj;
    }
    tmpDObj = new DataObject(*(ito::DataObject*)m_pInput["source"]->getVal<void*>());
    m_pOutput["sourceout"]->setVal<void*>(tmpDObj);

    m_pContent->refreshPlot(m_pOutput["displayed"]); //push the displayed DataObj into the actual plot widget for displaying

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> itom2DQwtFigure::getDisplayed(void)
{
	ito::DataObject *dObj = m_pOutput["displayed"]->getVal<ito::DataObject*>();
	if(dObj)
	{
		return QSharedPointer<ito::DataObject>( new ito::DataObject(*(dObj)) );
	}
	return QSharedPointer<ito::DataObject>();
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> itom2DQwtFigure::getSource(void)
{
	ito::DataObject *dObj = m_pInput["source"]->getVal<ito::DataObject*>();
	if(dObj)
	{
		return QSharedPointer<ito::DataObject>( new ito::DataObject(*(dObj)) );
	}
	return QSharedPointer<ito::DataObject>();
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::setShowContextMenu(bool show)
{
    if(m_pContent) m_pContent->m_showContextMenu = show;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool itom2DQwtFigure::showContextMenu() const
{
    if(m_pContent) return m_pContent->m_showContextMenu;
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal itom2DQwtFigure::displayLineCut(QVector<QPointF> bounds, ito::uint32 &uniqueID, const ito::uint8 direction)
{
    ito::RetVal retval = ito::retOk;
    QList<QString> paramNames;
    ito::uint32 newUniqueID = uniqueID;
    QWidget *lineCutObj = NULL;

    setOutpBounds(bounds);
    setLinePlotCoordinates(bounds);

    retval += apiGetFigure("DObjStaticLine","", newUniqueID, &lineCutObj, this); //(newUniqueID, "itom1DQwtFigure", &lineCutObj);

    if(!retval.containsError())
    {
        if(uniqueID != newUniqueID)
        {
            uniqueID = newUniqueID;
            ito::AbstractDObjFigure* lineCut = NULL;
            if (lineCutObj->inherits("ito::AbstractDObjFigure"))
                lineCut = (ito::AbstractDObjFigure*)lineCutObj;
            else
                return ito::retError;
            retval += addChannel((ito::AbstractNode*)lineCut, m_pOutput["bounds"], lineCut->getInputParam("bounds"), Channel::parentToChild, 0, 1);
            switch (direction)
            {
                // for a linecut in z-direction we have to pass the input object to the linecut, otherwise the 1D-widget "sees" only a 2D object
                // with one plane and cannot display the points in z-direction
                case 2:
                    retval += addChannel((ito::AbstractNode*)lineCut,  m_pOutput["sourceout"], lineCut->getInputParam("source"), Channel::parentToChild, 0, 1);
                    paramNames << "bounds"  << "sourceout";
                break;

                // otherwise simply pass on the displayed plane
                case 0:
                case 1:
                default:
                    retval += addChannel((ito::AbstractNode*)lineCut, m_pOutput["displayed"], lineCut->getInputParam("source"), Channel::parentToChild, 0, 1);
                    paramNames << "bounds"  << "displayed";
                break;
            }
            retval += updateChannels(paramNames);

            lineCut->show();
        }
        else
        {
            switch (direction)
            {
                case 2:
                    paramNames << "bounds"  << "sourceout";
                break;

                case 0:
                case 1:
                default:
                    paramNames << "bounds"  << "displayed";
                break;
            }
            retval += updateChannels(paramNames);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::mnuPanner(bool checked)
{
    if(checked)
    {
        m_actAScan->setChecked(false);
        m_actZoomToRect->setChecked(false);
        m_actLineCut->setChecked(false);
        m_actMarker->setChecked(false);
        m_pContent->m_pPanner->setEnabled(true);
    }
    else
    {
        m_pContent->m_pPanner->setEnabled(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::mnuZoomer(bool checked)
{
    if(checked)
    {
        m_actAScan->setChecked(false);
        m_actLineCut->setChecked(false);
        m_actMarker->setChecked(false);
        m_actPan->setChecked(false);
        m_pContent->m_pZoomer->setEnabled(true);
    }
    else
    {
        m_pContent->m_pZoomer->setEnabled(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::mnuValuePicker(bool checked)
{

    if(checked)
    {
        m_actAScan->setChecked(false);
        m_actZoomToRect->setChecked(false);
        m_actLineCut->setChecked(false);
        m_actPan->setChecked(false);
        m_pContent->m_pValuePicker->setEnabled(true);
    }
    else
    {
        m_pContent->m_pValuePicker->setEnabled(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::mnuPalette()
{
    m_pContent->m_paletteNum++;
    m_pContent->refreshColorMap();
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::mnuAScanPicker(bool checked)
{
    if(checked && m_pContent && m_pContent->getStackStatus())
    {
        m_actPan->setChecked(false);
        m_actZoomToRect->setChecked(false);
        m_actLineCut->setChecked(false);
        m_actMarker->setChecked(false);
        m_pContent->m_pAScanPicker->setEnabled(true);
        m_pContent->m_pAScanMarker->setVisible(true);
    }
    else
    {
        m_pContent->m_pAScanPicker->setEnabled(false);
        m_pContent->m_pAScanMarker->setVisible(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::mnuLinePicker(bool checked)
{
    if(checked)
    {
        m_actPan->setChecked(false);
        m_actZoomToRect->setChecked(false);
        m_actAScan->setChecked(false);
        m_actMarker->setChecked(false);
        m_pContent->m_pLinePicker->setEnabled(true);
    }
    else
    {
        m_pContent->m_pLinePicker->setEnabled(false);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::mnuExport()
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

        renderer.renderDocument(m_pContent, fileName, QSizeF(300, 200), 85);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::mnuScaleSetting()
{
    DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->m_pContent->data());
    double minX = 0.0, maxX = 0.0, minY = 0.0, maxY = 0.0, minZ = 0.0, maxZ = 0.0;
    double minRangeX = 0.0, maxRangeX = 0.0, minRangeY = 0.0, maxRangeY = 0.0;
    int dims = 2, numPlanes = 1, curPlane = 0;
    bool autoCalcX, autoCalcY, autoCalcZ;
    QwtInterval interval;

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
            maxRangeX = dataObj->getPixToPhys(dims - 1, dataObj->getSize(dims-1), test);
            minRangeY = dataObj->getPixToPhys(dims - 2, 0.0, test);
            maxRangeY = dataObj->getPixToPhys(dims - 2, dataObj->getSize(dims-2), test);

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

        m_pContent->setInterval(Qt::XAxis, autoCalcX, minX, maxX);
        m_pContent->setInterval(Qt::YAxis, autoCalcY, minY, maxY);
        m_pContent->setInterval(Qt::ZAxis, autoCalcZ, minZ, maxZ);
        m_pContent->refreshColorMap();
    }

    delete dlg;
    dlg = NULL;
}
//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::mnuColorBar(bool checked)
{
    m_pContent->enableAxis(QwtPlot::yRight, checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::mnuCmplxSwitch(QAction *action)
{
	if (m_pContent)
	{
		DataObjectRasterData* rasterData = static_cast<DataObjectRasterData*>(m_pContent->m_pContent->data());

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
		((Plot2DWidget*)m_pContent)->replot();
	}
}

void itom2DQwtFigure::mnuHome()
{
	m_pContent->m_pZoomer->zoom(0);
}

//----------------------------------------------------------------------------------------------------------------------------------        
QPointF itom2DQwtFigure::getZAxisInterval(void) 
{ 
    return ((Plot2DWidget*)m_pContent)->m_startRangeZ;
}

//----------------------------------------------------------------------------------------------------------------------------------        
void itom2DQwtFigure::setZAxisInterval(QPointF interval) 
{ 
    m_pContent->setInterval(Qt::ZAxis, 0, interval.x(), interval.y());
    return; 
}

//----------------------------------------------------------------------------------------------------------------------------------        
QString itom2DQwtFigure::getColorPalette(void) 
{ 
    return QString(); 
}

//----------------------------------------------------------------------------------------------------------------------------------        
void itom2DQwtFigure::setColorPalette(QString palette) 
{ 
    m_pContent->refreshColorMap(palette);
}

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::enableComplexGUI(const bool checked)
{ 
    m_actCmplxSwitch->setEnabled(checked);
    m_actCmplxSwitch->setVisible(checked);
}
//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::enableZStackGUI(const bool checked)
{
    m_actBack->setEnabled(checked);
    m_actBack->setVisible(checked);
    m_actForward->setEnabled(checked);
    m_actForward->setVisible(checked);
    m_actAScan->setEnabled(checked);
    m_actAScan->setVisible(checked);
}
//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::setLinePlotCoordinates(const QVector<QPointF> pts)
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

//----------------------------------------------------------------------------------------------------------------------------------
void itom2DQwtFigure::mnuForward()
{
    m_pContent->stackForward();
}

//----------------------------------------------------------------------------------------------------------------------------------

void itom2DQwtFigure::mnuBack()
{
    m_pContent->stackBack();
}
//----------------------------------------------------------------------------------------------------------------------------------
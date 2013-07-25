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

#include <qmessagebox.h>
#include <qfiledialog.h>
#include <qimagewriter.h>
#include <qsharedpointer.h>

#include <qwt_plot_renderer.h>

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::Itom1DQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
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
    m_actMarker(NULL),
    m_mnuSetMarker(NULL),
    m_actSetMarker(NULL),
	m_actCmplxSwitch(NULL),
	m_mnuCmplxSwitch(NULL),
    m_lblMarkerOffsets(NULL),
	m_lblMarkerCoords(NULL)
{
    m_pInput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for line plots from 2d objects").toAscii().data()));
    
    //int id = qRegisterMetaType<QSharedPointer<ito::DataObject> >("QSharedPointer<ito::DataObject>");

	//m_actHome
    m_actHome = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"),tr("Home"), this);
    m_actHome->setObjectName("actHome");
    m_actHome->setToolTip("Reset original view");

	//m_actSave
    m_actSave = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"),tr("Save"), this);
    m_actSave->setObjectName("actSave");
    m_actSave->setToolTip("Export current view");

    //m_actScaleSetting
    m_actScaleSetting = new QAction(QIcon(":/plots/icons/itom_icons/autoscal.png"),tr("Scale Settings"), this);
    m_actScaleSetting->setObjectName("actScaleSetting");
    m_actScaleSetting->setToolTip("Set the ranges and offsets of this view");

    //m_rescaleParent
    m_rescaleParent = new QAction(QIcon(":/itom1DQwtFigurePlugin/icons/parentScale.png"),tr("Parent Scale Settings"), this);
    m_rescaleParent->setObjectName("rescaleParent");
    m_rescaleParent->setToolTip("Set the value-range of the parent view according to this plot");
    m_rescaleParent->setVisible(false);

    //m_actForward
    m_actForward = new QAction(QIcon(":/itomDesignerPlugins/general/icons/forward.png"), tr("forward"), this);
    m_actForward->setObjectName("actionForward");
    m_actForward->setEnabled(false);
    m_actForward->setToolTip("Forward to next line");

    //m_actBack
    m_actBack = new QAction(QIcon(":/itomDesignerPlugins/general/icons/back.png"), tr("back"), this);
    m_actBack->setObjectName("actionBack");
    m_actBack->setEnabled(false);
    m_actBack->setToolTip("Back to previous line");

    //m_actPan
    m_actPan = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), QObject::tr("move"), this);
    m_actPan->setObjectName("actionPan");
    m_actPan->setCheckable(true);
    m_actPan->setChecked(false);
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

    //m_actSetMarker
    m_actSetMarker = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/markerPos.png"),tr("Set Markers to"), this);
	m_mnuSetMarker = new QMenu("Marker Switch");
	m_mnuSetMarker->addAction(tr("To Min-Max"));
//	m_mnuSetMarker->addAction(tr("To Mean-Max"));
//	m_mnuSetMarker->addAction(tr("To Mean-Min"));
	m_actSetMarker->setMenu(m_mnuSetMarker);

    //m_actCmplxSwitch
    m_actCmplxSwitch = new QAction(QIcon(":/itomDesignerPlugins/complex/icons/ImRe.png"),tr("Switch Imag, Real, Abs, Pha"), this);
	m_mnuCmplxSwitch = new QMenu("Complex Switch");
	m_mnuCmplxSwitch->addAction(tr("Imag"));
	m_mnuCmplxSwitch->addAction(tr("Real"));
	m_mnuCmplxSwitch->addAction(tr("Abs"));
	m_mnuCmplxSwitch->addAction(tr("Pha"));
	m_actCmplxSwitch->setMenu(m_mnuCmplxSwitch);
    m_actCmplxSwitch->setVisible(false);

    connect(m_actHome, SIGNAL(triggered()), this, SLOT(mnuHome()));
    connect(m_actSave, SIGNAL(triggered()), this, SLOT(mnuExport()));
    
    connect(m_actScaleSetting, SIGNAL(triggered()), this, SLOT(mnuScaleSetting()));
    connect(m_rescaleParent, SIGNAL(triggered()), this, SLOT(mnuParentScaleSetting()));

    connect(m_actMarker, SIGNAL(toggled(bool)), this, SLOT(mnuMarkerClick(bool)));
    connect(m_actZoomToRect, SIGNAL(toggled(bool)), this, SLOT(mnuZoomer(bool)));
    connect(m_actPan, SIGNAL(toggled(bool)), this, SLOT(mnuPanner(bool)));
    connect(m_mnuSetMarker, SIGNAL(triggered(QAction*)), this, SLOT(mnuSetMarker(QAction*)));
	connect(m_mnuCmplxSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuCmplxSwitch(QAction*)));

	QToolBar *toolbar = new QToolBar("1D Qwt Figure Toolbar", this);
	addToolBar(toolbar, "mainToolBar");

	QMenu *contextMenu = new QMenu(QObject::tr("plot1D"), this);
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
	toolbar->addAction(m_actSave);
	toolbar->addSeparator();
	toolbar->addAction(m_actHome);
    toolbar->addAction(m_actScaleSetting);
    toolbar->addAction(m_rescaleParent);
    toolbar->addAction(m_actPan);
    toolbar->addAction(m_actZoomToRect);

    // first block is zoom, scale settings, home
    toolbar->addSeparator();
    toolbar->addAction(m_actMarker);
    toolbar->addAction(m_actSetMarker);
    
    m_lblMarkerCoords = new QLabel("    \n    ", this);
    m_lblMarkerCoords->setAlignment( Qt::AlignRight | Qt::AlignTop);
    m_lblMarkerCoords->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Ignored);
    m_lblMarkerCoords->setObjectName("Marker Positions");

    m_lblMarkerOffsets = new QLabel("    \n    ", this);
    m_lblMarkerOffsets->setAlignment( Qt::AlignRight | Qt::AlignTop);
    m_lblMarkerOffsets->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Ignored);
    m_lblMarkerOffsets->setObjectName("Marker Offsets");

    QAction *lblAction = toolbar->addWidget(m_lblMarkerCoords);
    lblAction->setVisible(true);

    QAction *lblAction2 = toolbar->addWidget(m_lblMarkerOffsets);
    lblAction->setVisible(true);

    // next block is for complex and stacks
    toolbar->addSeparator();
    toolbar->addAction(m_actBack);
    toolbar->addAction(m_actForward);
    toolbar->addAction(m_actCmplxSwitch);

    m_data.m_autoAxisLabel = true;
    m_data.m_autoValueLabel = true;
    m_data.m_valueScaleAuto = true;
    m_data.m_dataType = ito::tFloat64;
    m_data.m_valueMin = -127.0;
    m_data.m_valueMax = 128.0;
    m_data.m_axisScaleAuto = true;

    m_pContent = new Plot1DWidget(contextMenu, &m_data, this);
    m_pContent->setObjectName("canvasWidget");

    connect(m_pContent, SIGNAL(setMarkerText(const QString &, const QString &)), this, SLOT(setMarkerText(const QString &, const QString &)));

    setFocus();
    setCentralWidget(m_pContent);
    m_pContent->setFocus();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom1DQwtPlot::~Itom1DQwtPlot()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom1DQwtPlot::applyUpdate()
{
    QVector<QPointF> bounds = getBounds();

    if ((ito::DataObject*)m_pInput["source"]->getVal<void*>())
    {
        m_pOutput["displayed"]->copyValueFrom(m_pInput["source"]);
        // why "source" is used here and not "displayed" .... ck 05/15/2013
        m_pContent->refreshPlot( (ito::DataObject*)m_pInput["source"]->getVal<char*>(), bounds);

        ito::Channel* dataChannel = getInputChannel("source");
        m_rescaleParent->setVisible( dataChannel && dataChannel->getParent() );
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom1DQwtPlot::getContextMenuEnabled() const
{
    if(m_pContent) return (m_pContent)->m_showContextMenu;
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setContextMenuEnabled(bool show)
{
    if (m_pContent) (m_pContent)->m_showContextMenu = show;
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
    delete pointArr;
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<QPointF> Itom1DQwtPlot::getBounds(void) 
{ 
    int numPts = m_pInput["bounds"]->getLen();
    QVector<QPointF> boundsVec;

    if(numPts > 0)
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
    if(m_data.m_autoTitle)
    {
        return "<auto>";
    }
    return m_data.m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setTitle(const QString &title)
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
void Itom1DQwtPlot::resetTitle()
{
    m_data.m_autoTitle = true;
    if(m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom1DQwtPlot::getAxisLabel() const
{
    if(m_data.m_autoAxisLabel)
    {
        return "<auto>";
    }
    return m_data.m_axisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setAxisLabel(const QString &label)
{
    if(label == "<auto>")
    {
        m_data.m_autoAxisLabel = true;
    }
    else
    {
        m_data.m_autoAxisLabel = false;
        m_data.m_axisLabel = label;
    }
    if(m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::resetAxisLabel()
{
    m_data.m_autoAxisLabel = true;
    if(m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom1DQwtPlot::getValueLabel() const
{
    if(m_data.m_autoValueLabel)
    {
        return "<auto>";
    }
    return m_data.m_valueLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::setValueLabel(const QString &label)
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
void Itom1DQwtPlot::resetValueLabel()
{
    m_data.m_autoValueLabel = true;
    if(m_pContent) m_pContent->updateLabels();
}


//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuPanner(bool checked)
{
    if(checked)
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
void Itom1DQwtPlot::mnuZoomer(bool checked)
{
    if(checked)
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
void Itom1DQwtPlot::mnuMarkerClick(bool checked)
{
    if(checked)
    {
        m_actPan->setChecked(false);
        m_actZoomToRect->setChecked(false);
    }
    
    m_pContent->setPickerEnable(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuExport()
{
#ifndef QT_NO_PRINTER
    QString fileName = "plot1D.pdf";
#else
    QString fileName = "plot1D.png";
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
        //renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true); //deprecated in qwt 6.1.0

        renderer.renderDocument((m_pContent), fileName, QSizeF(300, 200), 85);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuScaleSetting()
{
    Dialog1DScale *dlg = new Dialog1DScale(m_data, this);
    if(dlg->exec() == QDialog::Accepted)
    {
        dlg->getData(m_data);
        m_pContent->updateScaleValues();
    }

    delete dlg;
    dlg = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuParentScaleSetting()
{
    if(m_pContent && m_pContent->m_plotCurveItems.size() > 0)
    {
        const QwtScaleDiv scale = m_pContent->axisScaleDiv(QwtPlot::yLeft);
        QPointF bounds = QPointF( scale.lowerBound(), scale.upperBound() );
        /*
        DataObjectSeriesData* seriesData = static_cast<DataObjectSeriesData*>((m_pContent)->m_plotCurveItems[0]->data());
        int cmlpState = seriesData->getCmplxState();
        ito::uint32  minLoc[3], maxLoc[3];
        ito::float64 minVal, maxVal;

        ito::DataObject temp = seriesData->getResampledDataObject();

        if((temp.getType() != ito::tFloat64) || (temp.getDims() == 0))
            return;*/

        //ito::dObjHelper::minMaxValueFunc<ito::float64>(&temp, minVal, minLoc, maxVal, maxLoc, true, cmlpState);
        
        ito::Channel* dataChannel = getInputChannel("source");
        if( dataChannel && dataChannel->getParent() )
        {
            ( (ito::AbstractDObjFigure*)(dataChannel->getParent()) )->setZAxisInterval(bounds);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuSetMarker(QAction *action)
{
    if(m_pContent && m_pContent->m_plotCurveItems.size() > 0)
    {
        DataObjectSeriesData* seriesData = static_cast<DataObjectSeriesData*>((m_pContent)->m_plotCurveItems[0]->data());

		if (action->text() == QString("To Min-Max"))
        {
            DataObjectSeriesData::ComplexType cmlpState = seriesData->getCmplxState();

            ito::float64 minVal, maxVal;
            int minLoc, maxLoc;
            if(seriesData->getMinMaxLoc(minVal, maxVal, minLoc, maxLoc) == ito::retOk)
            {
                if(minLoc < maxLoc)
                {
                    m_pContent->setMainMarkersToIndex( minLoc, maxLoc, 0 );
                }
                else
                {
                    m_pContent->setMainMarkersToIndex( maxLoc, minLoc, 0 );
                }
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuCmplxSwitch(QAction *action)
{
    DataObjectSeriesData *seriesData;
	if (m_pContent)
	{
        foreach( QwtPlotCurve *data, m_pContent->m_plotCurveItems )
        {
            seriesData = (DataObjectSeriesData*)data->data();
            if(seriesData)
            {
		        if (action->text() == QString("Imag"))
                {
			        seriesData->setCmplxState(DataObjectSeriesData::cmplxImag);
                    m_actCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReImag.png"));
                }
		        else if (action->text() == QString("Real"))
                {
			        seriesData->setCmplxState(DataObjectSeriesData::cmplxReal);
                    m_actCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReReal.png"));
                }
		        else if (action->text() == QString("Pha"))
                {
			        seriesData->setCmplxState(DataObjectSeriesData::cmplxArg);
                    m_actCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImRePhase.png"));
                }
		        else
                {
			        seriesData->setCmplxState(DataObjectSeriesData::cmplxAbs);
                    m_actCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReAbs.png"));
                }
            }
        }

		(m_pContent)->setInterval(Qt::ZAxis, true, 0, 0);
		(m_pContent)->replot();
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF Itom1DQwtPlot::getYAxisInterval(void) 
{ 
    return (m_pContent)->m_startRangeY;
}

//----------------------------------------------------------------------------------------------------------------------------------        
void Itom1DQwtPlot::setYAxisInterval(QPointF interval) 
{ 
    (m_pContent)->setInterval(Qt::YAxis, 0, interval.x(), interval.y());
    return; 
}   

//----------------------------------------------------------------------------------------------------------------------------------   
void Itom1DQwtPlot::setMarkerText(const QString &coords, const QString &offsets)
{
    m_lblMarkerCoords->setText(coords);
    m_lblMarkerOffsets->setText(offsets);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::enableComplexGUI(const bool checked)
{ 
    m_actCmplxSwitch->setEnabled(checked);
    m_actCmplxSwitch->setVisible(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom1DQwtPlot::mnuHome()
{
    m_pContent->m_pZoomer->zoom(0);
}

//----------------------------------------------------------------------------------------------------------------------------------
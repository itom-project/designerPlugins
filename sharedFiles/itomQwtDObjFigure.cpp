/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2015, Institut fuer Technische Optik (ITO), 
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

#include "itomQwtDObjFigure.h"

#include <qbrush.h>
#include <qpalette.h>
#include <qstatusbar.h>

#include <qsharedpointer.h>
#include <qapplication.h>
#include <qtoolbar.h>

#include "DataObject/dataobj.h"
#include "itomQwtPlot.h"

#include <qwt_plot.h>
#include <qwt_plot_renderer.h>

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtDObjFigure::ItomQwtDObjFigure(QWidget *parent /*= NULL*/) : ito::AbstractDObjFigure("", AbstractFigure::ModeStandaloneInUi, parent),
    m_pBaseContent(NULL),
	m_pMarkerDock(NULL),
	m_pPickerDock(NULL),
	m_pShapesDock(NULL),
	m_pObjectInfoDock(NULL),
	m_pShapesInfo(NULL),
	m_pMarkerInfo(NULL),
	m_pPickerInfo(NULL),
	m_pObjectInfo(NULL)	
{
	construct();
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtDObjFigure::ItomQwtDObjFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent /*= NULL*/) : ito::AbstractDObjFigure(itomSettingsFile, windowMode, parent),
	m_pBaseContent(NULL),
	m_pMarkerDock(NULL),
	m_pPickerDock(NULL),
	m_pShapesDock(NULL),
	m_pObjectInfoDock(NULL),
	m_pShapesInfo(NULL),
	m_pMarkerInfo(NULL),
	m_pPickerInfo(NULL),
	m_pObjectInfo(NULL)
{
	construct();
}


//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::construct()
{
	m_pMarkerDock = new QDockWidget(tr("Marker Info"), this);
	m_pMarkerDock->setVisible(false);
	m_pMarkerDock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);

	m_pMarkerInfo = new PlotInfoMarker(m_pMarkerDock);
	m_pMarkerDock->setWidget(m_pMarkerInfo);

	m_pPickerDock = new QDockWidget(tr("Picker Info"), this);
	m_pPickerDock->setVisible(false);
	m_pPickerDock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);

	m_pPickerInfo = new PlotInfoPicker(m_pPickerDock);
	m_pPickerDock->setWidget(m_pPickerInfo);

	m_pShapesDock = new QDockWidget(tr("Shapes Info"), this);
	m_pShapesDock->setVisible(false);
	m_pShapesDock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);

	m_pShapesInfo = new PlotInfoShapes(m_pShapesDock);
	m_pShapesDock->setWidget(m_pShapesInfo);

	m_pObjectInfoDock = new QDockWidget(tr("Data Object Info"), this);
	m_pObjectInfoDock->setVisible(false);
	m_pObjectInfoDock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);

	m_pObjectInfo = new PlotInfoDObject(m_pObjectInfoDock);
	m_pObjectInfoDock->setWidget(m_pObjectInfo);

	addToolbox(m_pMarkerDock, "marker info", Qt::RightDockWidgetArea);
	addToolbox(m_pPickerDock, "picker info", Qt::RightDockWidgetArea);
	addToolbox(m_pShapesDock, "shapes info", Qt::RightDockWidgetArea);
	addToolbox(m_pObjectInfoDock, "object info", Qt::RightDockWidgetArea);
	
	if (getPropertyDockWidget())
	{
		tabifyDockWidget(m_pMarkerDock, getPropertyDockWidget());
	}

	tabifyDockWidget(m_pPickerDock, m_pMarkerDock);
	tabifyDockWidget(m_pShapesDock, m_pPickerDock);
	tabifyDockWidget(m_pObjectInfoDock, m_pShapesDock);
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtDObjFigure::~ItomQwtDObjFigure()
{
	
	//removeToolbox("shapes info");
	//m_pShapesDock->deleteLater();
	m_pShapesDock = NULL;
	m_pShapesInfo = NULL;
	
	//removeToolbox("picker info");
	//m_pPickerDock->deleteLater();
	m_pPickerDock = NULL;
	m_pPickerInfo = NULL;

	//removeToolbox("object info");
	//m_pObjectInfoDock->deleteLater();
	m_pObjectInfoDock = NULL;
	m_pObjectInfo = NULL;

	//removeToolbox("marker info");
	//m_pMarkerDock->deleteLater();
	m_pMarkerDock = NULL;
	
	m_pMarkerInfo = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::addToolbarsAndMenus()
{
    if (m_pBaseContent)
    {
        foreach(QToolBar* t, m_pBaseContent->getToolbars())
        {
            addToolBar(t, t->objectName());
        }

        foreach(QMenu* m, m_pBaseContent->getMenus())
        {
            addMenu(m);
        }
    }	
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::copyToClipBoard()
{
    if (m_pBaseContent)
        return m_pBaseContent->exportCanvas(true, "");
    return ito::RetVal(ito::retError, 0, tr("no content widget available.").toLatin1().data());
}

//----------------------------------------------------------------------------------------------------------------------------------
QPixmap ItomQwtDObjFigure::renderToPixMap(const int xsize, const int ysize, const int resolution)
{
    QwtPlot *plot = qobject_cast<QwtPlot*>(centralWidget());
    QSizeF curSize(xsize, ysize);

    if (!plot)
    {
        QSize myRect(curSize.width(), curSize.height());
        QPixmap destinationImage(myRect);
        return destinationImage;
    }

    if (curSize.height() == 0 || curSize.width() == 0)
    {
        curSize = plot->size();
    }
    //qreal linewidth = m_pContent->m_lineWidth;
    qreal resFaktor = resolution / 72.0 + 0.5;
    resFaktor = resFaktor < 1 ? 1 : resFaktor;
    resFaktor = resFaktor > 6 ? 6 : resFaktor;
    QSize myRect(curSize.width() * resFaktor, curSize.height() * resFaktor);

    QPixmap destinationImage(myRect);
    destinationImage.fill(Qt::white);

    QBrush curBrush = plot->canvasBackground();

    QPalette curPalette = plot->palette();

    plot->setAutoFillBackground(true);
    plot->setPalette(Qt::white);
    plot->setCanvasBackground(Qt::white);
    plot->replot();

    QwtPlotRenderer renderer;

    // flags to make the document look like the widget
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);

    QPainter painter(&destinationImage);
    painter.scale(resFaktor, resFaktor);
    renderer.render(plot, &painter, plot->rect());

    plot->setPalette(curPalette);
    plot->setCanvasBackground(curBrush);
    plot->replot();

    return destinationImage;
}




//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setButtonSet(const ButtonStyle newVal)
{
    if (m_pBaseContent)
    {
        m_pBaseContent->setButtonStyle(newVal);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtDObjFigure::ButtonStyle ItomQwtDObjFigure::getButtonSet(void) const
{
    if (m_pBaseContent)
    {
        return (ItomQwtDObjFigure::ButtonStyle)m_pBaseContent->buttonStyle();
    }
    return StyleBright;
}


//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::userInteractionStart(int type, bool start, int maxNrOfPoints /*= -1*/)
{
    if (m_pBaseContent)
    {
        switch (type)
        {
        default:
            m_pBaseContent->userInteractionStart(0, false, 0);
            break;
        case ito::Shape::MultiPointPick:
        case ito::Shape::Point:
            m_pBaseContent->userInteractionStart(type, start, maxNrOfPoints);
            break;

        case ito::Shape::Line:
            m_pBaseContent->userInteractionStart(type, start, maxNrOfPoints);
            break;

        case ito::Shape::Rectangle:
            m_pBaseContent->userInteractionStart(type, start, maxNrOfPoints);
            break;

        case ito::Shape::Ellipse:
            m_pBaseContent->userInteractionStart(type, start, maxNrOfPoints);
            break;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::clearGeometricShapes(void)
{
    if (m_pBaseContent)
    {
        m_pBaseContent->clearAllGeometricShapes();
        return ito::retOk;
    }
    return ito::RetVal(ito::retError, 0, tr("content widget not available").toLatin1().data());
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::deleteGeometricShape(int idx)
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->deleteGeometricShape(idx);
    }
    return ito::RetVal(ito::retError, 0, tr("content widget not available").toLatin1().data());
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::setGeometricShapeLabel(int idx, QString label)
{
    if (m_pBaseContent)
        return m_pBaseContent->setGeometricShapeLabel(idx, label);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::setGeometricShapeLabelVisible(int idx, bool setVisible)
{
    if (m_pBaseContent)
        return m_pBaseContent->setGeometricShapeLabelVisible(idx, setVisible);

    updatePropertyDock();

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool ItomQwtDObjFigure::getKeepAspectRatio(void) const
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->keepAspectRatio();
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setKeepAspectRatio(const bool &keepAspectEnable)
{
    if (m_pBaseContent)
    {
        m_pBaseContent->setKeepAspectRatio(keepAspectEnable);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool ItomQwtDObjFigure::getEnabledPlotting(void) const
{
    if (m_pBaseContent)
        return m_pBaseContent->plottingEnabled();
    return QSharedPointer<ito::DataObject>();
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setEnabledPlotting(const bool &enabled)
{
    if (m_pBaseContent)
        return m_pBaseContent->setPlottingEnabled(enabled);
    updatePropertyDock();

}

//----------------------------------------------------------------------------------------------------------------------------------
bool ItomQwtDObjFigure::getContextMenuEnabled() const
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->showContextMenu();
    }

    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setContextMenuEnabled(bool show)
{
    if (m_pBaseContent)
    {
        m_pBaseContent->setShowContextMenu(show);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtPlotEnums::ModificationModes ItomQwtDObjFigure::getModificationModes() const
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->shapeModificationModes();
    }
    
    return ItomQwtPlotEnums::Move | ItomQwtPlotEnums::Resize | ItomQwtPlotEnums::Rotate;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setModificationModes(const ItomQwtPlotEnums::ModificationModes modes)
{
    if (m_pBaseContent)
    {
        m_pBaseContent->setShapeModificationModes(modes);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<ito::Shape> ItomQwtDObjFigure::getGeometricShapes()
{
    if (m_pBaseContent)
        return m_pBaseContent->getGeometricShapes();
    return QVector<ito::Shape>();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::setGeometricShapes(QVector<ito::Shape> geometricShapes)
{
    if (m_pBaseContent)
    {
        ito::RetVal retval = m_pBaseContent->setGeometricShapes(geometricShapes);
        updatePropertyDock();
        return retval;
    }
    return ito::RetVal(ito::retError, 0, "content not available");
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::addGeometricShape(const ito::Shape &geometricShape)
{
    if (m_pBaseContent)
    {
        ito::RetVal retval = m_pBaseContent->addGeometricShape(geometricShape);
        updatePropertyDock();
        return retval;
    }
    return ito::RetVal(ito::retError, 0, "content not available");
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::modifyGeometricShape(const ito::Shape &geometricShape)
{
    if (m_pBaseContent)
    {
        ito::RetVal retval = m_pBaseContent->modifyGeometricShape(geometricShape);
        updatePropertyDock();
        return retval;
    }
    return ito::RetVal(ito::retError, 0, "content not available");
}

//----------------------------------------------------------------------------------------------------------------------------------
int ItomQwtDObjFigure::getSelectedGeometricShape(void)const
{
    if (m_pBaseContent) return m_pBaseContent->getSelectedGeometricShapeIdx();
    return -1;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setSelectedGeometricShape(const int idx)
{
    if (m_pBaseContent) return m_pBaseContent->setSelectedGeometricShapeIdx(idx);
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
int ItomQwtDObjFigure::getGeometricShapesCount() const
{
    return m_pBaseContent ? m_pBaseContent->countGeometricShapes() : 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool ItomQwtDObjFigure::getMarkerLabelsVisible(void) const
{
    return m_pBaseContent ? m_pBaseContent->markerLabelVisible() : false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setMarkerLabelsVisible(const bool &visible)
{
    if (m_pBaseContent) return m_pBaseContent->setMarkerLabelVisible(visible);
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool ItomQwtDObjFigure::getShapesLabelsVisible(void) const
{
    return m_pBaseContent ? m_pBaseContent->shapesLabelVisible() : false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setShapesLabelsVisible(const bool &visible)
{
    if (m_pBaseContent) return m_pBaseContent->setShapesLabelVisible(visible);
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::plotMarkers(QSharedPointer< ito::DataObject > coords, QString style, QString id /*= QString::Null()*/, int plane /*= -1*/)
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->plotMarkers(coords, style, id, plane);
    }

    return ito::RetVal(ito::retError, 0, "content not available");
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::deleteMarkers(QString id /*= ""*/)
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->deleteMarkers(id);
    }
    return ito::RetVal(ito::retError, 0, "content not available");
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AbstractFigure::UnitLabelStyle ItomQwtDObjFigure::getUnitLabelStyle() const
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->unitLabelStyle();
    }
    else
    {
        return AbstractFigure::UnitLabelSlash;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::replot()
{
    if (m_pBaseContent)
    {
        m_pBaseContent->replot();
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
QColor ItomQwtDObjFigure::getBackgroundColor(void) const
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->backgroundColor();
    }
    else
        return Qt::white;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setBackgroundColor(const QColor newVal)
{
    if (m_pBaseContent)
    {
        m_pBaseContent->setBackgroundColor(newVal.rgb() & 0x00FFFFFF);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor ItomQwtDObjFigure::getCanvasColor(void) const
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->canvasColor();
    }
    else
        return Qt::white;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setCanvasColor(const QColor newVal)
{
    if (m_pBaseContent)
    {
        m_pBaseContent->setCanvasColor(newVal.rgb() & 0x00FFFFFF);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor ItomQwtDObjFigure::getAxisColor(void) const
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->axisColor();
    }
    else
        return Qt::black;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setAxisColor(const QColor newVal)
{
    if (m_pBaseContent)
    {
        m_pBaseContent->setAxisColor(newVal.rgb() & 0x00FFFFFF);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor ItomQwtDObjFigure::getTextColor(void) const
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->textColor();
    }
    else
        return Qt::black;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setTextColor(const QColor newVal)
{
    if (m_pBaseContent)
    {
        m_pBaseContent->setTextColor(newVal.rgb() & 0x00FFFFFF);
    }

    updatePropertyDock();
}



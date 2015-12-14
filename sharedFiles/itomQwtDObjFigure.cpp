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
ItomQwtDObjFigure::ItomQwtDObjFigure(QWidget *parent /*= NULL*/) :
    ito::AbstractDObjFigure("", AbstractFigure::ModeStandaloneInUi, parent),
    m_pBaseContent(NULL)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtDObjFigure::ItomQwtDObjFigure(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent /*= NULL*/) :
    ito::AbstractDObjFigure(itomSettingsFile, windowMode, parent),
    m_pBaseContent(NULL)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtDObjFigure::~ItomQwtDObjFigure()
{
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
        case ito::PrimitiveContainer::tMultiPointPick:
        case ito::PrimitiveContainer::tPoint:
            m_pBaseContent->userInteractionStart(type, start, maxNrOfPoints);
            break;

        case ito::PrimitiveContainer::tLine:
            m_pBaseContent->userInteractionStart(type, start, maxNrOfPoints);
            break;

        case ito::PrimitiveContainer::tRectangle:
            m_pBaseContent->userInteractionStart(type, start, maxNrOfPoints);
            break;

        case ito::PrimitiveContainer::tEllipse:
            m_pBaseContent->userInteractionStart(type, start, maxNrOfPoints);
            break;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::clearGeometricElements(void)
{
    if (m_pBaseContent)
    {
        m_pBaseContent->clearAllGeometricShapes();
        return ito::retOk;
    }
    return ito::RetVal(ito::retError, 0, tr("content widget not available").toLatin1().data());
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::deleteGeometricElement(int id)
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->deleteGeometricShape(id);
    }
    return ito::RetVal(ito::retError, 0, tr("content widget not available").toLatin1().data());
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::setGeometricElementLabel(int id, QString label)
{
    if (m_pBaseContent)
        return m_pBaseContent->setGeometricShapeLabel(id, label);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::setGeometricElementLabelVisible(int id, bool setVisible)
{
    if (m_pBaseContent)
        return m_pBaseContent->setGeometricShapeLabelVisible(id, setVisible);

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
ItomQwtPlotEnums::ModificationState ItomQwtDObjFigure::getModState() const
{
    if (m_pBaseContent)
    {
        return m_pBaseContent->shapeModificationType();
    }

    return ItomQwtPlotEnums::ModifyPoints;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setModState(const ItomQwtPlotEnums::ModificationState val)
{
    if (m_pBaseContent)
    {
        m_pBaseContent->setShapeModificationType(val);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer< ito::DataObject > ItomQwtDObjFigure::getGeometricElements()
{
    if (m_pBaseContent)
        return m_pBaseContent->geometricShapes2DataObject();
    return QSharedPointer<ito::DataObject>();
}



//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::setGeometricElements(QSharedPointer< ito::DataObject > geometricElements)
{
    if (m_pBaseContent)
    {
        ito::RetVal retval = m_pBaseContent->setGeometricShapes(geometricElements);
        updatePropertyDock();
        return retval;
    }
    return ito::RetVal(ito::retError, 0, "content not available");
}

//----------------------------------------------------------------------------------------------------------------------------------
int ItomQwtDObjFigure::getSelectedElement(void)const
{
    if (m_pBaseContent) return m_pBaseContent->getSelectedGeometricShapeIdx();
    return -1;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ItomQwtDObjFigure::setSelectedElement(const int idx)
{
    if (m_pBaseContent) return m_pBaseContent->setSelectedGeometricShapeIdx(idx);
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
int ItomQwtDObjFigure::getGeometricElementsCount() const
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
ito::RetVal ItomQwtDObjFigure::deleteMarkers(int id) //deprecated
{
    ito::RetVal retval(ito::retWarning, 0, "slot 'deleteMarkers' is deprecated for deletion of geometric shapes. Use slot 'deleteGeometricElement' instead.");

    retval += deleteGeometricElement(id);
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::plotMarkers(QSharedPointer< ito::DataObject > coords, QString style, QString id /*= QString::Null()*/, int plane /*= -1*/)
{
    if (coords->getDims() == 2 && coords->getSize(1) >= 6)
    {
        ito::RetVal retval(ito::retWarning, 0, "slot 'plotMarkers' is deprecated for drawing geometric shapes. Use slot 'setGeometricElements' instead.");
        retval += setGeometricElements(coords);
        return retval;
    }
    else if (m_pBaseContent)
    {
        return m_pBaseContent->plotMarkers(coords, style, id, plane);
    }

    return ito::RetVal(ito::retError, 0, "content not available");
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ItomQwtDObjFigure::deleteMarkers(QString id)
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



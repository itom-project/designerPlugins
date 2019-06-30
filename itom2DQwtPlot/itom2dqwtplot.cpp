/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

#include "plotCanvas.h"
#include "itom2dqwtplot.h"

#include "userInteractionPlotPicker.h"
#include "multiPointPickerMachine.h"

#include "dialogExportProperties.h"

#include "plotLegends/infoWidgetPickers.h"

#include <qfiledialog.h>
#include <qimagewriter.h>
#include <qinputdialog.h>
#include <qmessagebox.h>
#include <qshortcut.h>
#include <qdesktopwidget.h>

#include <qwt_plot_renderer.h>
#include <qmenu.h>
#include "dialog2DScale.h"
#include <qwt_text_label.h>
#include <qwt_scale_widget.h>
#include <qwt_picker_machine.h>

#include <qwt_plot_shapeitem.h>

#include <qwt_plot_layout.h>

#include "DataObject/dataObjectFuncs.h"

//------------------------------------------------------------------------------------------------------------------------
class Itom2dQwtPlotPrivate 
{
public:
    Itom2dQwtPlotPrivate() : 
        m_pData(NULL)
    {}
    
    InternalData *m_pData;
    QHash<QObject*,ito::uint32> m_childFigures;
};

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::constructor()
{
    d = new Itom2dQwtPlotPrivate();

    // Basic settings
    m_pContent = NULL;
    
    m_pInput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, tr("Points for volume plots from 3d objects").toLatin1().data()));

    //bounds, volumeCutBounds and zCutPoint are three different output connections, since it is possible to have a line cut, volume cut and a z-stack cut visible at the same time.
    m_pOutput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for line plots from 2d objects").toLatin1().data()));
    m_pOutput.insert("zCutPoint", new ito::Param("zCutPoint", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for z-stack cut in 3d objects").toLatin1().data()));
    m_pOutput.insert("volumeCutBounds", new ito::Param("volumeCutBounds", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for volume cut in 3d objects").toLatin1().data()));
    m_pOutput.insert("sourceout", new ito::Param("sourceout", ito::ParamBase::DObjPtr, NULL, QObject::tr("shallow copy of input source object").toLatin1().data()));


    d->m_pData = new InternalData();
    
    //init internal data
    d->m_pData->m_dataType = ito::tFloat64;
    d->m_pData->m_autoTitle;
    d->m_pData->m_autoxAxisLabel = true;
    d->m_pData->m_autoyAxisLabel = true;
    d->m_pData->m_autoValueLabel = true;
    d->m_pData->m_valueScaleAuto = true;
    d->m_pData->m_valueMin = -127.0;
    d->m_pData->m_valueMax = 128.0;
    d->m_pData->m_xaxisScaleAuto = true;
    d->m_pData->m_yaxisScaleAuto = true;
    d->m_pData->m_xaxisVisible = true;
    d->m_pData->m_yaxisVisible = true;
    d->m_pData->m_colorBarVisible = false;
    d->m_pData->m_cmplxType = ItomQwtPlotEnums::CmplxAbs;
    d->m_pData->m_yaxisFlipped = false;
    d->m_pData->m_pConstOutput = &m_pOutput;

    //initialize canvas
    m_pContent = new PlotCanvas(d->m_pData, this);
    m_pBaseContent = m_pContent;
    m_pContent->setObjectName("canvasWidget");

    setCentralWidget(m_pContent);

    addToolbarsAndMenus();

    registerShortcutActions();

    setPropertyObservedObject(this);

    m_pContent->init(true); //here, the API is not ready, yet. Usually init() is called again once the event is raised signalling that the api is ready. However, in QtDesigner, the api is not available. Therefore, we want to init the canvas here.
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom2dQwtPlot::Itom2dQwtPlot(QWidget *parent) : 
    ItomQwtDObjFigure("", AbstractFigure::ModeStandaloneInUi, parent)
{
    constructor();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom2dQwtPlot::Itom2dQwtPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    ItomQwtDObjFigure(itomSettingsFile, windowMode, parent)
{
    constructor();
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom2dQwtPlot::~Itom2dQwtPlot()
{
    m_pContent->deleteLater();
    m_pContent = NULL;
    m_pBaseContent = NULL;

    if (d->m_pData)
        delete d->m_pData;
    d->m_pData = NULL;

    delete d;
    d = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::init() 
{ 
    //called when api-pointers are transmitted, directly after construction
    return m_pContent->init(m_windowMode != ito::AbstractFigure::ModeStandaloneInUi); 
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::applyUpdate()
{
    //displayed and sourceout is set by dataObjRasterData, since the data is analyzed there
    /*
    if (subplotStates()["lineCut"] & ito::AbstractFigure::tUninitilizedExtern && m_pOutput["bounds"]->getLen() < 2 && m_pInput["source"]->getVal<ito::DataObject*>())
    {
        ito::DataObject* tmp = m_pInput["source"]->getVal<ito::DataObject*>();
        int dims = tmp->getDims();
        double bounds[6] = {0.0, 0.0, 0.0, 1.0, 0.5, 0.5};
        if (dims > 1)
        {
            bounds[2] = tmp->getPixToPhys(dims-1, 0);
            bounds[3] = tmp->getPixToPhys(dims-1, tmp->getSize(dims-1));
            bounds[4] = tmp->getPixToPhys(dims-2, tmp->getSize(dims-2)/2);
            bounds[5] = tmp->getPixToPhys(dims-2, tmp->getSize(dims-2)/2);
        }
        else
        {
        
        }
        m_pOutput["bounds"]->setVal<double*>(bounds, 6);
    }
    */
    QVector<QPointF> bounds = getBounds();
    m_pContent->refreshPlot(m_pInput["source"]->getVal<ito::DataObject*>(),-1,bounds);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::colorBarVisible() const
{
    if (m_pContent)
        return m_pContent->m_pActToggleColorBar->isChecked();
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setColorBarVisible(bool value)
{
    if (m_pContent)
    {
        m_pContent->m_pActToggleColorBar->setChecked(value); //emits toggle signal of action
        updatePropertyDock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::enableOverlaySlider(bool enabled)
{
    if (m_pContent)
        m_pContent->m_pActOverlaySlider->setVisible(enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getTitle() const
{
    if (!d->m_pData || d->m_pData->m_autoTitle)
    {
        return "<auto>";
    }
    return d->m_pData->m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setTitle(const QString &title)
{
    if (d->m_pData == NULL)
    {
        return;
    }

    if (title == "<auto>")
    {
        d->m_pData->m_autoTitle = true;
    }
    else
    {
        d->m_pData->m_autoTitle = false;
        d->m_pData->m_title = title;
    }

    if (m_pContent)
    {
        m_pContent->updateLabels();
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetTitle()
{
    if (d->m_pData == NULL)
    {
        return;
    }

    d->m_pData->m_autoTitle = true;
    if (m_pContent)
    {
        m_pContent->updateLabels();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getxAxisLabel() const
{
    if (d->m_pData->m_autoxAxisLabel)
    {
        return "<auto>";
    }
    return d->m_pData->m_xaxisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setxAxisLabel(const QString &label)
{
    if (d->m_pData == NULL)
    {
        return;
    }

    if (label == "<auto>")
    {
        d->m_pData->m_autoxAxisLabel = true;
    }
    else
    {
        d->m_pData->m_autoxAxisLabel = false;
        d->m_pData->m_xaxisLabel = label;
    }
    if (m_pContent) m_pContent->updateLabels();
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetxAxisLabel()
{
    if (!d->m_pData)
    {
        return;
    }

    d->m_pData->m_autoxAxisLabel = true;
    if (m_pContent)
    {
        m_pContent->updateLabels();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getyAxisLabel() const
{
    if (d->m_pData->m_autoyAxisLabel)
    {
        return "<auto>";
    }
    return d->m_pData->m_yaxisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisLabel(const QString &label)
{
    if (!d->m_pData)
    {
        return;
    }

    if (label == "<auto>")
    {
        d->m_pData->m_autoyAxisLabel = true;
    }
    else
    {
        d->m_pData->m_autoyAxisLabel = false;
        d->m_pData->m_yaxisLabel = label;
    }
    if (m_pContent)
    {
        m_pContent->updateLabels();
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetyAxisLabel()
{
    if (!d->m_pData)
    {
        return;
    }
    d->m_pData->m_autoyAxisLabel = true;
    if (m_pContent)
    {
        m_pContent->updateLabels();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getValueLabel() const
{
    if (d->m_pData->m_autoValueLabel)
    {
        return "<auto>";
    }
    return d->m_pData->m_valueLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setValueLabel(const QString &label)
{
    if (!d->m_pData)
    {
        return;
    }

    if (label == "<auto>")
    {
        d->m_pData->m_autoValueLabel = true;
    }
    else
    {
        d->m_pData->m_autoValueLabel = false;
        d->m_pData->m_valueLabel = label;
    }
    if (m_pContent)
    {
        m_pContent->updateLabels();
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetValueLabel()
{
    if (!d->m_pData)
    {
        return;
    }

    d->m_pData->m_autoValueLabel = true;
    if (m_pContent)
    {
        m_pContent->updateLabels();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtPlotEnums::ScaleEngine Itom2dQwtPlot::getValueScale() const
{
	if (m_pContent)
	{
		return m_pContent->m_valueScale;
	}
	return ItomQwtPlotEnums::Linear;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setValueScale(const ItomQwtPlotEnums::ScaleEngine &scale)
{
	if (m_pContent)
	{
		m_pContent->setValueAxisScaleEngine(scale);
	}
	updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getyAxisFlipped() const
{
    return d->m_pData->m_yaxisFlipped;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisFlipped(const bool &value)
{
    if (!d->m_pData)
    {
        return;
    }

    if (d->m_pData->m_yaxisFlipped != value)
    {
        d->m_pData->m_yaxisFlipped = value;
    }
    if (m_pContent)
    {
        m_pContent->updateScaleValues(true, false); //replot, but no change of the current x/y and value zoom ranges
        m_pContent->internalDataUpdated();
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getxAxisVisible() const
{
    return d->m_pData->m_xaxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setxAxisVisible(const bool &value)
{
    if (!d->m_pData)
    {
        return;
    }

    d->m_pData->m_xaxisVisible = value;

    if (m_pContent)
    {
        m_pContent->enableAxis(QwtPlot::xBottom, value);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getyAxisVisible() const
{
    return d->m_pData->m_yaxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisVisible(const bool &value)
{
    if (!d->m_pData)
    {
        return;
    }

    d->m_pData->m_yaxisVisible = value;

    if (m_pContent)
    {
        m_pContent->enableAxis(QwtPlot::yLeft, value);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom2dQwtPlot::getXAxisInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getInterval(Qt::XAxis);
    }

    return ito::AutoInterval();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setXAxisInterval(ito::AutoInterval interval)
{
    if (m_pContent)
    {
        m_pContent->setInterval(Qt::XAxis, interval);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom2dQwtPlot::getYAxisInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getInterval(Qt::YAxis);
    }

    return ito::AutoInterval();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setYAxisInterval(ito::AutoInterval interval)
{
    if (m_pContent)
    {
        m_pContent->setInterval(Qt::YAxis, interval);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom2dQwtPlot::getZAxisInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getInterval(Qt::ZAxis);
    }

    return ito::AutoInterval();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setZAxisInterval(ito::AutoInterval interval)
{
    if (m_pContent)
    {
        m_pContent->setInterval(Qt::ZAxis, interval);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval Itom2dQwtPlot::getOverlayInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getOverlayInterval(Qt::ZAxis);
    }

    return ito::AutoInterval();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setOverlayInterval(ito::AutoInterval interval)
{
    if (m_pContent)
    {
        m_pContent->setOverlayInterval(Qt::ZAxis, interval);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getColorMap() const
{
    if (m_pContent)
    {
        return m_pContent->colorMapName();
    }

    return "";
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setColorMap(const QString &name)
{
    if (name != "" && m_pContent)
    {
        m_pContent->setColorMap(name);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getOverlayColorMap() const
{
    if (m_pContent)
    {
        return m_pContent->colorOverlayMapName();
    }

    return "";
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setOverlayColorMap(const QString &name)
{
    if (name != "" && m_pContent)
    {
        m_pContent->setOverlayColorMap(name);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom2dQwtPlot::getTitleFont(void) const
{
    if (m_pContent)
    {
        return m_pContent->titleLabel()->font();
    }

    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setTitleFont(const QFont &font)
{
    if (m_pContent)
    {
        m_pContent->titleLabel()->setFont(font);
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom2dQwtPlot::getLabelFont(void) const
{
    if (m_pContent)
    {
        QwtText t = m_pContent->axisWidget(QwtPlot::xBottom)->title();
        return t.font();
    }

    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setLabelFont(const QFont &font)
{
    if (m_pContent)
    {
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
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont Itom2dQwtPlot::getAxisFont(void) const
{
    if (m_pContent)
    {
        return m_pContent->axisFont(QwtPlot::xBottom);
    }

    return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setAxisFont(const QFont &font)
{
    if (m_pContent)
    {
        m_pContent->setAxisFont(QwtPlot::xBottom, font);
        m_pContent->setAxisFont(QwtPlot::yLeft, font);
        m_pContent->setAxisFont(QwtPlot::yRight, font);
    }

    updatePropertyDock();
}


//----------------------------------------------------------------------------------------------------------------------------------
int Itom2dQwtPlot::getPlaneIndex() const
{
    if (m_pContent)
    {
        return m_pContent->getCurrentPlane();
    }

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setPlaneIndex(const int &index)
{
    int idx = index;
    if (m_pContent)
    {
        QSpinBox *spinBox = qobject_cast<QSpinBox*>(m_pContent->m_pActPlaneSelector->defaultWidget());
        if (spinBox)
        {
            idx = qBound(spinBox->minimum(), idx, spinBox->maximum());
            spinBox->blockSignals(true); //block valueChanged signal from spinBox, which also calls this method.
            spinBox->setValue(idx);
            spinBox->blockSignals(false);
        }

        m_pContent->changePlane(idx);

        emit planeIndexChanged(idx);
    }
    
    QStringList paramNames;
    
    if (m_pOutput["bounds"]->getLen() == 6)
    {
        paramNames << "bounds"  << "sourceout";
        double * bounds = m_pOutput["bounds"]->getVal<double*>();

        double newBounds[6];

        for (int i = 2; i < 6;i ++)
        {
            newBounds[i] = bounds[i];
        }
        newBounds[0] = m_pContent->getCurrentPlane();
        newBounds[1] = m_pContent->getCurrentPlane();
        m_pOutput["bounds"]->setVal<double*>(newBounds, 6);
    }
    else
    {
        paramNames << "displayed" ;
    }

    updateChannels(paramNames);

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ItomQwtPlotEnums::DataChannel Itom2dQwtPlot::getDataChannel() const
{
    if (d->m_pData)
    {
        return d->m_pData->m_dataChannel;
    }

    return ItomQwtPlotEnums::ChannelAuto;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setDataChannel(const ItomQwtPlotEnums::DataChannel &dataChannel)
{
    if (!d->m_pData)
    {
        return;
    }

    if (dataChannel != d->m_pData->m_dataChannel)
    {
        d->m_pData->m_dataChannel = dataChannel;
        if (m_pContent)
        {
            m_pContent->adjustColorDataTypeRepresentation();
            m_pContent->replot();
        }
        updatePropertyDock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetDataChannel()
{
    if (!d->m_pData)
    {
        return;
    }

    d->m_pData->m_dataChannel = ItomQwtPlotEnums::ChannelAuto;
    if (m_pContent)
    {
        m_pContent->replot();
    }
    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setPlaneRange(int min, int max)
{
    if (m_pContent)
    {
        QSpinBox *spinBox = qobject_cast<QSpinBox*>(m_pContent->m_pActPlaneSelector->defaultWidget());
        if (spinBox)
        {
            int value = spinBox->value();
            value = std::max(min, value);
            value = std::min(max, value);
            spinBox->setMinimum(min);
            spinBox->setMaximum(max);
            spinBox->setValue(value);
        }
        m_pContent->m_pActPlaneSelector->setVisible((max - min) > 0);
        m_pContent->m_pActStackCut->setVisible((max - min) > 0);
        m_pContent->m_pActVolumeCut->setVisible((max - min) > 0);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::displayVolumeCut(QVector <QPointF> bounds, ito::uint32 &uniqueID)
{
    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        return ito::RetVal(ito::retError, 0, tr("Could not spawn volumeCut due to missing API-handle").toLatin1().data());
    }

    int infoType = ito::Shape::Line;

    ito::RetVal retval = ito::retOk;
    QList<QString> paramNames;
    ito::uint32 newUniqueID = uniqueID;
    QWidget *volumeCutObj = NULL;

    bool needChannelUpdate = false;

    double *pointArr = new double[2 * bounds.size()];
    for (int np = 0; np < bounds.size(); np++)
    {
        pointArr[np * 2] = bounds[np].x();
        pointArr[np * 2 + 1] = bounds[np].y();
    }

    if (subplotStates()["volumeCut"] & ito::AbstractFigure::tUninitilizedExtern)
    {
        needChannelUpdate = true;
        subplotStates()["volumeCut"] &= ~ito::AbstractFigure::tUninitilizedExtern;
        subplotStates()["volumeCut"] |= ito::AbstractFigure::tExternChild;
    }
    m_pOutput["volumeCutBounds"]->setVal(pointArr, 2 * bounds.size());
    

    delete[] pointArr;
    //setOutpBounds(bounds);
    //setLinePlotCoordinates(bounds);

    retval += apiGetFigure("DObjStaticImage", "", newUniqueID, &volumeCutObj, this);

    QWidget *w = this;
    while (w)
    {
        //qDebug() << w->geometry() << w->frameGeometry();
        w = qobject_cast<QWidget*>(w->parent());
    }

    if (!retval.containsError())
    {
        if (uniqueID != newUniqueID || needChannelUpdate)
        {
            uniqueID = newUniqueID;
            ito::AbstractDObjFigure* childFigure = NULL;
            if (volumeCutObj->inherits("ito::AbstractDObjFigure"))
            {
                childFigure = (ito::AbstractDObjFigure*)volumeCutObj;
                if (!needChannelUpdate)
                {
                    d->m_childFigures[volumeCutObj] = newUniqueID;
                    connect(volumeCutObj, SIGNAL(destroyed(QObject*)), this, SLOT(childFigureDestroyed(QObject*)));
                }

                //try to active this 2d plot again -> activatePlot will then raise this 2d plot window. 
                //Then, the focus is tried to be set to the canvas to receive key-events (like H or V for horizontal or vertical lines)
                QTimer::singleShot(0, this, SLOT(activatePlot()));

                retval += moveChildPlotCloseToThis(volumeCutObj);
            }
            else
            {
                return ito::RetVal(ito::retError, 0, tr("the opened figure is not inherited from ito::AbstractDObjFigure").toLatin1().data());
            }

            if (needChannelUpdate)
            {
                ito::Channel *tempChannel;
                foreach(tempChannel, m_pChannels)
                {
                    if (tempChannel->getParent() == (ito::AbstractNode*)this &&  tempChannel->getChild() == (ito::AbstractNode*)childFigure)
                    {
                        removeChannel(tempChannel);
                    }
                }
            }
            if (bounds.size() == 2)
            {
                ((QMainWindow*)childFigure)->setWindowTitle(tr("Volumecut"));
                if (childFigure->inherits("ItomQwtDObjFigure"))
                {
                    ((ItomQwtDObjFigure*)childFigure)->setComplexStyle(d->m_pData->m_cmplxType);
                }
                // otherwise pass the original plane and z0:z1, y0:y1, x0, x1 coordinates
                retval += addChannel((ito::AbstractNode*)childFigure, m_pOutput["volumeCutBounds"], childFigure->getInputParam("bounds"), ito::Channel::parentToChild, 0, 1);
                retval += addChannel((ito::AbstractNode*)childFigure, m_pOutput["sourceout"], childFigure->getInputParam("source"), ito::Channel::parentToChild, 0, 1);
                paramNames << "volumeCutBounds" << "sourceout";
            }
            else
            {
                return ito::RetVal(ito::retError, 0, tr("Expected a bound vector with 2 values").toLatin1().data());
            }

            retval += updateChannels(paramNames);

            if (needChannelUpdate) // we have an updated plot and want to show it
            {
                if (subplotStates()["volumeCut"] & ito::AbstractFigure::tVisibleOnInit)
                {
                    subplotStates()["volumeCut"] &= ~ito::AbstractFigure::tVisibleOnInit;
                    childFigure->setVisible(true);
                }
                // Something to do?
            }
            else// we do not have a plot so we have to show it and its child of this plot
            {
                    subplotStates()["volumeCut"] = ito::AbstractFigure::tOwnChild;
                    childFigure->show();
            }
        }
        else
        {
            if (bounds.size() == 2)
            {
                paramNames << "volumeCutBounds" << "sourceout";
            }
            else
            {
                paramNames << "volumeCutBounds" << "displayed";
            }
            retval += updateChannels(paramNames);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::displayCut(QVector<QPointF> bounds, ito::uint32 &uniqueID, bool zStack /*= false*/)
{
    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        return ito::RetVal(ito::retError, 0, tr("Could not spawn lineCut due to missing API-handle").toLatin1().data());
    }

	int infoType = ito::Shape::Line;

    ito::RetVal retval = ito::retOk;
    QList<QString> paramNames;
    ito::uint32 newUniqueID = uniqueID;
    QWidget *lineCutObj = NULL;

    bool needChannelUpdate = false;

    double *pointArr = new double[2 * bounds.size()];
    for (int np = 0; np < bounds.size(); np++)
    {
        pointArr[np * 2] = bounds[np].x();
        pointArr[np * 2 + 1] = bounds[np].y();
    }

    if (zStack)
    {	
		infoType = ito::Shape::Point;
        m_pOutput["zCutPoint"]->setVal(pointArr, 2 * bounds.size());
        if (subplotStates()["zSlice"] & ito::AbstractFigure::tUninitilizedExtern)
        {
            needChannelUpdate = true;
            subplotStates()["zSlice"] &= ~ito::AbstractFigure::tUninitilizedExtern;
            subplotStates()["zSlice"] |= ito::AbstractFigure::tExternChild;
        }
    }
    else
    {	
		if (subplotStates()["lineCut"] & ito::AbstractFigure::tUninitilizedExtern)
        {
            needChannelUpdate = true;
            subplotStates()["lineCut"] &= ~ito::AbstractFigure::tUninitilizedExtern;
            subplotStates()["lineCut"] |= ito::AbstractFigure::tExternChild;
        }
        m_pOutput["bounds"]->setVal(pointArr, 2 * bounds.size());
    }

    DELETE_AND_SET_NULL_ARRAY(pointArr);

    retval += apiGetFigure("DObjStaticLine","", newUniqueID, &lineCutObj, this); //(newUniqueID, "itom1DQwtFigure", &lineCutObj);

    if (!retval.containsError())
    {
        if (uniqueID != newUniqueID || needChannelUpdate)
        {
            uniqueID = newUniqueID;
            ito::AbstractDObjFigure* childFigure = NULL;
            if (lineCutObj->inherits("ito::AbstractDObjFigure"))
            {
                childFigure = (ito::AbstractDObjFigure*)lineCutObj;
                if (!needChannelUpdate)
                {
                    d->m_childFigures[lineCutObj] = newUniqueID;
                    connect(lineCutObj, SIGNAL(destroyed(QObject*)), this, SLOT(childFigureDestroyed(QObject*)));
                }

                //try to active this 2d plot again -> activatePlot will then raise this 2d plot window. 
                //Then, the focus is tried to be set to the canvas to receive key-events (like H or V for horizontal or vertical lines)
                QTimer::singleShot(0, this, SLOT(activatePlot()));

                retval += moveChildPlotCloseToThis(childFigure);
            }
            else
            {
                return ito::RetVal(ito::retError, 0, tr("the opened figure is not inherited from ito::AbstractDObjFigure").toLatin1().data());
            }

            if (needChannelUpdate)
            {
                ito::Channel *tempChannel;
                foreach(tempChannel, m_pChannels)
                {
                    if (tempChannel->getParent() == (ito::AbstractNode*)this &&  tempChannel->getChild() == (ito::AbstractNode*)childFigure)
                    {
                        removeChannel(tempChannel);
                    }
                }
            }

            if (zStack)
            {
                ((QMainWindow*)childFigure)->setWindowTitle(tr("Z-Stack"));
				if (childFigure->inherits("ItomQwtDObjFigure"))
				{
					((ItomQwtDObjFigure*)childFigure)->setComplexStyle(d->m_pData->m_cmplxType);
				}
                // for a linecut in z-direction we have to pass the input object to the linecut, otherwise the 1D-widget "sees" only a 2D object
                // with one plane and cannot display the points in z-direction
                retval += addChannel((ito::AbstractNode*)childFigure, m_pOutput["zCutPoint"], childFigure->getInputParam("bounds"), ito::Channel::parentToChild, 0, 1);
                retval += addChannel((ito::AbstractNode*)childFigure,  m_pOutput["sourceout"], childFigure->getInputParam("source"), ito::Channel::parentToChild, 0, 1);
                paramNames << "zCutPoint"  << "sourceout";
            }
            else if (bounds.size() == 3) // its a 3D-Object
            {
                ((QMainWindow*)childFigure)->setWindowTitle(tr("Linecut"));
				if (childFigure->inherits("ItomQwtDObjFigure"))
				{
					((ItomQwtDObjFigure*)childFigure)->setComplexStyle(d->m_pData->m_cmplxType);
				}
                // otherwise pass the original plane and z0:z1, y0:y1, x0, x1 coordinates
                retval += addChannel((ito::AbstractNode*)childFigure, m_pOutput["bounds"], childFigure->getInputParam("bounds"), ito::Channel::parentToChild, 0, 1);
                retval += addChannel((ito::AbstractNode*)childFigure, m_pOutput["sourceout"], childFigure->getInputParam("source"), ito::Channel::parentToChild, 0, 1);
                paramNames << "bounds"  << "sourceout";
            }
            else
            {
                ((QMainWindow*)childFigure)->setWindowTitle(tr("Linecut"));
				if (childFigure->inherits("ItomQwtDObjFigure"))
				{
					((ItomQwtDObjFigure*)childFigure)->setComplexStyle(d->m_pData->m_cmplxType);
				}
                // otherwise simply pass on the displayed plane
                retval += addChannel((ito::AbstractNode*)childFigure, m_pOutput["bounds"], childFigure->getInputParam("bounds"), ito::Channel::parentToChild, 0, 1);
                retval += addChannel((ito::AbstractNode*)childFigure, m_pOutput["displayed"], childFigure->getInputParam("source"), ito::Channel::parentToChild, 0, 1);
                paramNames << "bounds"  << "displayed";
            }

            retval += updateChannels(paramNames);

            if (needChannelUpdate) // we have an updated plot and want to show it
            {
                if (zStack && (subplotStates()["zSlice"] & ito::AbstractFigure::tVisibleOnInit))
                {
                    subplotStates()["zSlice"] &= ~ito::AbstractFigure::tVisibleOnInit;
                    childFigure->setVisible(true);
                }
                else if (!zStack && (subplotStates()["lineCut"] & ito::AbstractFigure::tVisibleOnInit))
                {
                    subplotStates()["lineCut"] &= ~ito::AbstractFigure::tVisibleOnInit;
                    childFigure->setVisible(true);
                }
                // Something to do?
            }
            else// we do not have a plot so we have to show it and its child of this plot
            {
                if (zStack)
                {
                    subplotStates()["zSlice"] = ito::AbstractFigure::tOwnChild;
                    childFigure->show();
                }
                else
                {
                    subplotStates()["lineCut"] = ito::AbstractFigure::tOwnChild;
                    childFigure->show();
                }
            }
        }
        else
        {
            if (zStack)
            {
                paramNames << "zCutPoint"  << "sourceout";
            }
            else if (bounds.size() == 3) // its a 3D-Object
            {
                paramNames << "bounds"  << "sourceout";
            }
            else
            {
                paramNames << "bounds"  << "displayed";
            }
            retval += updateChannels(paramNames);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//tries to place a line cut, volume cut ... a little bit to the right / bottom of this plot, however still in the same screen
ito::RetVal Itom2dQwtPlot::moveChildPlotCloseToThis(QWidget *child)
{
    //get global position of this window
    QWidget *w = this;
    
    QRect temp = geometry();

    QRect geom(0, 0, 0, 0); //final geometry of the child
    geom.setSize(size());

    while (w)
    {
        temp = w->geometry();
        geom = QRect(temp.x() + geom.x(), temp.y() + geom.y(), geom.width(), geom.height());
        w = qobject_cast<QWidget*>(w->parent());
    }

    //check if the desired geometry is within the available desktop
    QDesktopWidget *dw = QApplication::desktop();
    QRect screenGeom = dw->screenGeometry(this);

    //move the new figure close to the right, bottom position of this figure
    geom.setX(geom.x() + 2 * width() / 3);
    geom.setY(geom.y() + 2 * height() / 3);
    geom.setWidth(qMin(width(), screenGeom.width()));
    geom.setHeight(qMin(height(), screenGeom.height()));

    if (!screenGeom.contains(geom))
    {
        //new geometry is outside of this screen. Try to fit it into this screen...
        if (geom.right() > screenGeom.right())
        {
            geom.translate(screenGeom.right() - geom.right(), 0);
        }

        if (geom.left() < screenGeom.left())
        {
            geom.translate(screenGeom.left() - geom.left(), 0);
        }

        if (geom.top() < screenGeom.top())
        {
            geom.translate(0, screenGeom.top() - geom.top());
        }

        if (geom.bottom() > screenGeom.bottom())
        {
            geom.translate(0, screenGeom.bottom() - geom.bottom());
        }
    }

    child->move(geom.x(), geom.y());

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::childFigureDestroyed(QObject *obj)
{


    QHash<QObject*,ito::uint32>::iterator it = d->m_childFigures.find(obj);

    if (it != d->m_childFigures.end())
    {
        m_pContent->childFigureDestroyed(obj, d->m_childFigures[obj]);

		if (pickerWidget())
		{
			(pickerWidget())->removeChildPlot(d->m_childFigures[obj]);
		}
	
    }
	else
	{
		if (pickerWidget())
		{
			(pickerWidget())->removeChildPlots();
		}
	
        m_pContent->childFigureDestroyed(obj, 0);
    }

    d->m_childFigures.erase(it);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::activatePlot()
{
    //try to active this 2d plot again -> activatePlot will then raise this 2d plot window. 
    //Then, the focus is tried to be set to the canvas to receive key-events (like H or V for horizontal or vertical lines)
    //this method is invoked by displayCut
    activateWindow();
    if (m_pContent)
    {
        QTimer::singleShot(0, m_pContent, SLOT(setFocus()));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getEnabledCenterMarker(void) const 
{
    if (m_pContent)
    {
        return m_pContent->showCenterMarker();
    }

    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setEnabledCenterMarker(const bool &enabled)
{
    if (m_pContent)
    {
        m_pContent->setShowCenterMarker(enabled);
        updatePropertyDock();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
int Itom2dQwtPlot::getOverlayAlpha () const 
{
    return d->m_pData->m_alpha;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setOverlayAlpha (const int alpha)
{
    if (d->m_pData == NULL)
    {
        return;
    }

    int alphaClipped = qBound(0, alpha, 255);
    
    if (d->m_pData->m_alpha != alphaClipped)
    {
        d->m_pData->m_alpha = alphaClipped;

        if (m_pContent)
        {
            m_pContent->alphaChanged();
            m_pContent->m_pOverlaySlider->setValue(d->m_pData->m_alpha);
        }
    }

    this->updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> Itom2dQwtPlot::getDisplayed()
{
    if (!m_pContent)
    {
        return QSharedPointer<ito::DataObject>(); 
    }

    return m_pContent->getDisplayed();
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> Itom2dQwtPlot::getDisplayedLineCut(void)
{
    if (!m_pContent)
    {
        return QSharedPointer<ito::DataObject>(); 
    }

    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        return QSharedPointer<ito::DataObject>(); 
    }

    ito::AbstractDObjFigure* figure = NULL;
    QList<QObject*> keys = d->m_childFigures.keys();

    for (int i = 0; i < keys.length(); i++)
    {
        if (d->m_childFigures[keys[i]] == m_pContent->m_lineCutUID &&
            keys[i]->inherits("ito::AbstractDObjFigure"))                        
        {
            return (qobject_cast<ito::AbstractDObjFigure*>(keys[i]))->getDisplayed();
        }
    }

    return QSharedPointer<ito::DataObject>(); 
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::setLinePlot(const double x0, const double y0, const double x1, const double y1, const int /*destID*/)
{
    if (m_pContent)
    {
        m_pContent->setLinePlot(x0, y0, x1, y1);
    }
    else
    {
        return ito::RetVal(ito::retError, 0, tr("Set lineCut coordinates failed. Widget not ready.").toLatin1().data());
    }

    return ito::retOk;
}





//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer< ito::DataObject > Itom2dQwtPlot::getOverlayImage() const 
{
    if (m_pContent)
    {
        return m_pContent->getOverlayObject();
    }

    return QSharedPointer< ito::DataObject >(NULL); 
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setContourLevels(QSharedPointer<ito::DataObject> newLevelObj)
{
    if (m_dataPointer.contains("contourLevls"))
    {
        if (m_dataPointer["contourLevels"].data() != newLevelObj.data())
        {
            QSharedPointer<ito::DataObject> oldLevels = m_dataPointer["contourLevels"];//possible backup for previous source, this backup must be alive until updateParam with the new one has been completely propagated
            m_dataPointer["contourLevels"] = newLevelObj;
        }
    }
    else
    {
        m_dataPointer["contourLevels"] = newLevelObj;
    }
    if (m_pContent)
    {
        m_pContent->setContourLevels(newLevelObj);
    }

}
//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer< ito::DataObject > Itom2dQwtPlot::getContourLevels() const 
{
    if (m_pContent)
    {
        return m_pContent->getContourLevels();
    }

    return QSharedPointer< ito::DataObject >(NULL);
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetContourLevels()
{
    if(m_pContent)
    {
        m_pContent->setContourLevels(QSharedPointer<ito::DataObject>());
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setContourColorMap(const QString &name)
{
    if (m_pContent)
    {
        m_pContent->setContourColorMap(name);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getContourColorMap() const
{
    if (m_pContent)
    {
        return m_pContent->getContourColorMap();
    }

    return "";
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setContourLineWidth(const float& width)
{
    if (m_pContent)
    {
        m_pContent->setContourLineWidth(width);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
float Itom2dQwtPlot::getContourLineWidth() const
{
    if (m_pContent)
    {
        return m_pContent->getContourLineWidth();
    }

    return 1;
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setOverlayImage(QSharedPointer< ito::DataObject > newOverlayObj)
{
    if (m_dataPointer.contains("overlayImage"))
    {
        //check if pointer of shared incoming data object is different to pointer of previous data object
        //if so, free previous
        if (m_dataPointer["overlayImage"].data() != newOverlayObj.data())
        {
            QSharedPointer<ito::DataObject> oldSource = m_dataPointer["overlayImage"]; //possible backup for previous source, this backup must be alive until updateParam with the new one has been completely propagated

            // sometimes crash here when replacing the source
            m_dataPointer["overlayImage"] = newOverlayObj;
        }  
    }
    else
    {
        m_dataPointer["overlayImage"] = newOverlayObj;
    }


    if (m_pContent)
    {
        m_pContent->setOverlayObject(newOverlayObj.data());
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::resetOverlayImage(void)
{
    if (m_pContent)
    {
        m_pContent->setOverlayObject(NULL);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setUnitLabelStyle(const ito::AbstractFigure::UnitLabelStyle &style)
{
    if (m_pContent)
    {
        m_pContent->setUnitLabelStyle(style);
        m_pContent->m_unitLabelChanged = true;
        m_pContent->refreshPlot(m_pInput["source"]->getVal<ito::DataObject*>());
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::ItomPlotHandle Itom2dQwtPlot::getLineCutPlotItem() const
{
    ito::ItomPlotHandle handle(NULL, NULL, 0);
    if (m_pContent && this->m_pContent->m_lineCutUID > 0)
    {
        if (apiGetItomPlotHandleByID(m_pContent->m_lineCutUID, handle) == ito::retOk)
        {
            return handle;
        }
    }
    return ito::ItomPlotHandle(NULL, NULL, 0);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setLineCutPlotItem(const ito::ItomPlotHandle idx)
{
    ito::RetVal retval = ito::retOk;
    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        return;
    }
    
    if (m_pContent || idx.getObjectID() > -1)
    {
        ito::uint32 thisID = 0;
        retval += apiGetFigureIDbyHandle(this, thisID);

        if (idx.getObjectID() == thisID || retval.containsError())
        {
            return;
        }
        else
        {
            thisID = idx.getObjectID();
        }

        QWidget *lineCutObj = NULL;
        

        this->m_pContent->m_lineCutUID = thisID;
        retval += apiGetFigure("DObjStaticLine","", this->m_pContent->m_lineCutUID, &lineCutObj, this);
        if (lineCutObj == NULL || (!lineCutObj->inherits("ito::AbstractDObjFigure")))
        {
            m_pContent->m_lineCutUID = 0;
        }
        else
        {
            ito::AbstractFigure *af = qobject_cast<ito::AbstractFigure*>(lineCutObj);
            if (af->getInputParam("bounds") == NULL || af->getInputParam("source") == NULL)
            {
                m_pContent->m_lineCutUID = 0;
            }
        }

        subplotStates()["lineCut"] = this->m_pContent->m_lineCutUID != 0 ? ito::AbstractFigure::tUninitilizedExtern | ito::AbstractFigure::tVisibleOnInit : ito::AbstractFigure::tNoChildPlot;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------

ito::ItomPlotHandle Itom2dQwtPlot::getZSlicePlotItem() const
{
    ito::ItomPlotHandle handle(NULL, NULL, 0);
    if (m_pContent && this->m_pContent->m_zstackCutUID > 0)
    {
        if (apiGetItomPlotHandleByID(m_pContent->m_zstackCutUID, handle) == ito::retOk)
        {
            return handle;
        }
    }

    return ito::ItomPlotHandle(NULL, NULL, 0);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setZSlicePlotItem(const ito::ItomPlotHandle idx)
{
    ito::RetVal retval = ito::retOk;
    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        return;
    }
    
    if (m_pContent || idx.getObjectID() > -1)
    {
        ito::uint32 thisID = 0;
        retval += apiGetFigureIDbyHandle(this, thisID);

        if (idx.getObjectID() == thisID || retval.containsError())
        {
            return;
        }
        else
        {
            thisID = idx.getObjectID();
        }

        QWidget *lineCutObj = NULL;
        
        this->m_pContent->m_zstackCutUID = thisID;
        retval += apiGetFigure("DObjStaticLine","", this->m_pContent->m_zstackCutUID, &lineCutObj, this); //(newUniqueID, "itom1DQwtFigure", &lineCutObj);
        if (lineCutObj == NULL || (!lineCutObj->inherits("ito::AbstractDObjFigure")))
        {
            m_pContent->m_zstackCutUID = 0;
        }
        else
        {
            ito::AbstractFigure *af = qobject_cast<ito::AbstractFigure*>(lineCutObj);
            if (af->getInputParam("bounds") == NULL || af->getInputParam("source") == NULL)
            {
                m_pContent->m_lineCutUID = 0;
            }
        }

        subplotStates()["zSlice"] = this->m_pContent->m_zstackCutUID != 0 ? ito::AbstractFigure::tUninitilizedExtern | ito::AbstractFigure::tVisibleOnInit : ito::AbstractFigure::tNoChildPlot;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setBounds(QVector<QPointF> bounds)
{
    double *pointArr = new double[2 * bounds.size()];
    for (int np = 0; np < bounds.size(); np++)
    {
        pointArr[np * 2] = bounds[np].x();
        pointArr[np * 2 + 1] = bounds[np].y();
    }
    m_pInput["bounds"]->setVal(pointArr, 2 * bounds.size());
    delete[] pointArr;
}
//----------------------------------------------------------------------------------------------------------------------------------
QVector<QPointF> Itom2dQwtPlot::getBounds(void) const
{
    int numPts = m_pInput["bounds"]->getLen();
    QVector<QPointF> boundsVec;

    if (numPts > 0)
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
ItomQwtPlotEnums::ComplexType Itom2dQwtPlot::getComplexStyle() const
{
	return m_pContent->getComplexStyle();
}
//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setComplexStyle(const ItomQwtPlotEnums::ComplexType &type)
{
	m_pContent->setComplexStyle(type);
}
//----------------------------------------------------------------------------------------------------------------------------------

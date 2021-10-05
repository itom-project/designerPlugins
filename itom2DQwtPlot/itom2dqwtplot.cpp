/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2019, Institut fuer Technische Optik (ITO),
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

#include <qfiledialog.h>
#include <qimagewriter.h>
#include <qinputdialog.h>
#include <qmessagebox.h>
#include <qshortcut.h>
#include <qdesktopwidget.h>
#include <qpointer.h>

#include <qwt_plot_renderer.h>
#include <qmenu.h>
#include "dialog2DScale.h"
#include <qwt_text_label.h>
#include <qwt_scale_widget.h>
#include <qwt_picker_machine.h>

#include <qwt_plot_shapeitem.h>

#include <qwt_plot_layout.h>

#include "DataObject/dataObjectFuncs.h"

//! this class contains information about a child plot
class ChildPlotItem
{
public:
    enum ChildPlotState
    {
        StateChannelsConnected = 0x10, /*if this flag is set, the 'channels' are connected from this plot to this child plot. */
        StateShowPending = 0x20 /*if this flag is set, the child plot should be shown at first plot command (then reset this flag) */
    };

    Q_DECLARE_FLAGS(ChildPlotStates, ChildPlotState)

    ChildPlotItem() :
        m_UID(0),
        m_states(0)
    {};

    ChildPlotItem(ito::uint32 UID, QWidget* childFigure, ChildPlotStates states = ChildPlotStates()) :
        m_UID(UID),
        m_childFigure(childFigure),
        m_states(states)
    {}

    //default copy constructor and assignment operator can be used

    bool isValid() const { return !m_childFigure.isNull(); }
    ito::uint32 UID() const { return isValid() ? m_UID : 0; }
    QWidget *childFigure() const { return m_childFigure.data(); }
    ChildPlotStates states() const { return m_states; }
    ChildPlotStates& rStates() { return m_states; }

private:
    QPointer<QWidget> m_childFigure;
    ito::uint32 m_UID;
    ChildPlotStates m_states;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(ChildPlotItem::ChildPlotStates)

//------------------------------------------------------------------------------------------------------------------------
class Itom2dQwtPlotPrivate 
{
public:

    Itom2dQwtPlotPrivate() : 
        m_pData(NULL)
    {}
    
    PlotCanvas::InternalData *m_pData;

    //QHash<QObject*,ito::uint32> m_childFigures;
    ChildPlotItem m_volumeCutChildPlot; /*Line cut plot in the 2D plane, spanned by the last two dimensions and displayed as 1D plot*/
    ChildPlotItem m_lineCutChildPlot;   /*Line cut plot along the first dimension (in case of >= 3 dimensions), displayed as 1D plot */
    ChildPlotItem m_zStackChildPlot;    /*Plane cut along the x/z, y/z or x-y/z direction, displayed as 2D plot*/
};

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::constructor()
{
    d = new Itom2dQwtPlotPrivate();

    // Basic settings
    m_pContent = NULL;
    
    addInputParam(new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, tr("Points for volume plots from 3d objects").toLatin1().data()));

    //bounds, volumeCutBounds and zCutPoint are three different output connections, since it is possible to have a line cut, volume cut and a z-stack cut visible at the same time.
    addOutputParam(new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for line plots from 2d objects").toLatin1().data()));
    addOutputParam(new ito::Param("zCutPoint", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for z-stack cut in 3d objects").toLatin1().data()));
    addOutputParam(new ito::Param("volumeCutBounds", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for volume cut in 3d objects").toLatin1().data()));
    addOutputParam(new ito::Param("sourceout", ito::ParamBase::DObjPtr, NULL, QObject::tr("shallow copy of input source object").toLatin1().data()));


    d->m_pData = new PlotCanvas::InternalData();
    
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

    QHash<QString, ito::Param*> selectedOutputParameters;
    selectedOutputParameters["sourceout"] = getOutputParam("sourceout");
    selectedOutputParameters["displayed"] = getOutputParam("displayed");
    selectedOutputParameters["output"] = getOutputParam("output");
    d->m_pData->m_selectedOutputParameters = selectedOutputParameters;

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
    return m_pContent->init(getWindowMode() != ito::AbstractFigure::ModeStandaloneInUi);
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
    m_pContent->refreshPlot(getInputParam("source")->getVal<const ito::DataObject*>(),-1,bounds);

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
Itom2dQwtPlot::GridStyle Itom2dQwtPlot::getGrid(void) const
{
    if (m_pContent)
    {
        return m_pContent->gridStyle();
    }
    return GridNo;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setGrid(const Itom2dQwtPlot::GridStyle &gridStyle)
{
    if (m_pContent)
    {
        m_pContent->setGridStyle(gridStyle);
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
    
    if (getOutputParam("bounds")->getLen() == 6)
    {
        paramNames << "bounds"  << "sourceout";
        const double* bounds = getOutputParam("bounds")->getVal<const double*>();

        double newBounds[6];

        for (int i = 2; i < 6;i ++)
        {
            newBounds[i] = bounds[i];
        }
        newBounds[0] = m_pContent->getCurrentPlane();
        newBounds[1] = m_pContent->getCurrentPlane();
        getOutputParam("bounds")->setVal<double*>(newBounds, 6);
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
ito::RetVal Itom2dQwtPlot::displayVolumeCut(const QVector<QPointF> &bounds, ito::uint32 *childFigureUID /*= NULL*/)
{
    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        return ito::RetVal(ito::retError, 0, tr("Could not spawn lineCut due to missing API-handle").toLatin1().data());
    }

    ito::RetVal retval = ito::retOk;
    QList<QString> paramNames;
    ito::uint32 UID = 0;
    QWidget *childPlot = NULL;
    ito::AbstractDObjFigure* childFigure = NULL;

    double *pointArr = new double[2 * bounds.size()];
    for (int np = 0; np < bounds.size(); ++np)
    {
        pointArr[np * 2] = bounds[np].x();
        pointArr[np * 2 + 1] = bounds[np].y();
    }

    getOutputParam("volumeCutBounds")->setVal(pointArr, 2 * bounds.size());

    DELETE_AND_SET_NULL_ARRAY(pointArr);

    if (d->m_volumeCutChildPlot.isValid())
    {
        //there should exist a plot
        childPlot = d->m_volumeCutChildPlot.childFigure();
        UID = d->m_volumeCutChildPlot.UID();
    }

    if (UID == 0 || childPlot == NULL)
    {
        //try to create a new plot
        d->m_volumeCutChildPlot.rStates() = ChildPlotItem::StateShowPending;
        UID = 0;
        retval += apiGetFigure("DObjStaticImage", "", UID, &childPlot, this);

        if (!retval.containsError())
        {
            d->m_volumeCutChildPlot = ChildPlotItem(UID, childPlot, ChildPlotItem::StateShowPending);

            if (childPlot->inherits("ito::AbstractDObjFigure"))
            {
                childFigure = (ito::AbstractDObjFigure*)childPlot;
                connect(childPlot, SIGNAL(destroyed(QObject*)), this, SLOT(childFigureDestroyed(QObject*)));

                //try to active this 2d plot again -> activatePlot will then raise this 2d plot window. 
                //Then, the focus is tried to be set to the canvas to receive key-events (like H or V for horizontal or vertical lines)
                QTimer::singleShot(0, this, SLOT(activatePlot()));

                retval += moveChildPlotCloseToThis(childFigure);
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("the opened figure is not inherited from ito::AbstractDObjFigure").toLatin1().data());
            }
        }
    }

    if (!retval.containsError())
    {
        if (childFigureUID)
        {
            *childFigureUID = UID;
        }

        childFigure = (ito::AbstractDObjFigure*)childPlot;

        if (!(d->m_volumeCutChildPlot.states() & ChildPlotItem::StateChannelsConnected))
        {
            d->m_volumeCutChildPlot.rStates() &= ~ChildPlotItem::StateChannelsConnected;

            if (bounds.size() == 2)
            {
                ((QMainWindow*)childFigure)->setWindowTitle(tr("Volumecut"));
                if (childFigure->inherits("ItomQwtDObjFigure"))
                {
                    ((ItomQwtDObjFigure*)childFigure)->setComplexStyle(d->m_pData->m_cmplxType);
                }

                QList<ito::AbstractNode::ParamNamePair> excludes;
                excludes << ParamNamePair("volumeCutBounds", "bounds");
                excludes << ParamNamePair("sourceout", "source");
                removeAllChannelsToReceiver((ito::AbstractNode*)childFigure, excludes);

                // otherwise pass the original plane and z0:z1, y0:y1, x0, x1 coordinates
                retval += createChannel("volumeCutBounds", childFigure, "bounds", false);
                retval += createChannel("sourceout", childFigure, "source", false);
                paramNames << "volumeCutBounds" << "sourceout";
            }
            else
            {
                return ito::RetVal(ito::retError, 0, tr("Expected a bounds vector with 2 values").toLatin1().data());
            }

            retval += updateChannels(paramNames);

            if (d->m_volumeCutChildPlot.states() & ChildPlotItem::StateShowPending)
            {
                d->m_volumeCutChildPlot.rStates() &= ~ChildPlotItem::StateShowPending;
                childFigure->setVisible(true);
            }
            else // we do not have a plot so we have to show it and its child of this plot
            {
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
ito::RetVal Itom2dQwtPlot::displayLineCut(const QVector<QPointF> &bounds, ito::uint32 *childFigureUID /*= NULL*/)
{
    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        return ito::RetVal(ito::retError, 0, tr("Could not spawn lineCut due to missing API-handle").toLatin1().data());
    }

    ito::RetVal retval = ito::retOk;
    QList<QString> paramNames;
    ito::uint32 UID = 0;
    QWidget *childPlot = NULL;
    ito::AbstractDObjFigure* childFigure = NULL;

    double *pointArr = new double[2 * bounds.size()];
    for (int np = 0; np < bounds.size(); np++)
    {
        pointArr[np * 2] = bounds[np].x();
        pointArr[np * 2 + 1] = bounds[np].y();
    }

    getOutputParam("bounds")->setVal(pointArr, 2 * bounds.size());

    DELETE_AND_SET_NULL_ARRAY(pointArr);

    if (d->m_lineCutChildPlot.isValid())
    {
        //there should exist a plot
        childPlot = d->m_lineCutChildPlot.childFigure();
        UID = d->m_lineCutChildPlot.UID();
    }

    if (UID == 0 || childPlot == NULL)
    {
        //try to create a new plot
        d->m_lineCutChildPlot.rStates() = ChildPlotItem::StateShowPending;
        UID = 0;
        retval += apiGetFigure("DObjStaticLine", "", UID, &childPlot, this);

        if (!retval.containsError())
        {
            d->m_lineCutChildPlot = ChildPlotItem(UID, childPlot, ChildPlotItem::StateShowPending);

            if (childPlot->inherits("ito::AbstractDObjFigure"))
            {
                childFigure = (ito::AbstractDObjFigure*)childPlot;
                connect(childPlot, SIGNAL(destroyed(QObject*)), this, SLOT(childFigureDestroyed(QObject*)));

                //try to active this 2d plot again -> activatePlot will then raise this 2d plot window. 
                //Then, the focus is tried to be set to the canvas to receive key-events (like H or V for horizontal or vertical lines)
                QTimer::singleShot(0, this, SLOT(activatePlot()));

                retval += moveChildPlotCloseToThis(childFigure);
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("the opened figure is not inherited from ito::AbstractDObjFigure").toLatin1().data());
            }
        }
    }

    if (!retval.containsError())
    {
        if (childFigureUID)
        {
            *childFigureUID = UID;
        }

        childFigure = (ito::AbstractDObjFigure*)childPlot;

        if (!(d->m_lineCutChildPlot.states() & ChildPlotItem::StateChannelsConnected))
        {
            d->m_lineCutChildPlot.rStates() &= ~ChildPlotItem::StateChannelsConnected;

            if (bounds.size() == 3) // its a 3D-Object
            {
                ((QMainWindow*)childFigure)->setWindowTitle(tr("Linecut"));
                if (childFigure->inherits("ItomQwtDObjFigure"))
                {
                    ((ItomQwtDObjFigure*)childFigure)->setComplexStyle(d->m_pData->m_cmplxType);
                }

                QList<ito::AbstractNode::ParamNamePair> excludes;
                excludes << ParamNamePair("bounds", "bounds");
                excludes << ParamNamePair("sourceout", "source");
                removeAllChannelsToReceiver((ito::AbstractNode*)childFigure, excludes);

                // otherwise pass the original plane and z0:z1, y0:y1, x0, x1 coordinates
                retval += createChannel("bounds", childFigure, "bounds", false);
                retval += createChannel("sourceout", childFigure, "source", false);
                paramNames << "bounds" << "sourceout";
            }
            else
            {
                ((QMainWindow*)childFigure)->setWindowTitle(tr("Linecut"));
                if (childFigure->inherits("ItomQwtDObjFigure"))
                {
                    ((ItomQwtDObjFigure*)childFigure)->setComplexStyle(d->m_pData->m_cmplxType);
                }

                QList<ito::AbstractNode::ParamNamePair> excludes;
                excludes << ParamNamePair("bounds", "bounds");
                excludes << ParamNamePair("displayed", "source");
                removeAllChannelsToReceiver((ito::AbstractNode*)childFigure, excludes);

                // otherwise simply pass on the displayed plane
                retval += createChannel("bounds", childFigure, "bounds", false);
                retval += createChannel("displayed", childFigure, "source", false);
                paramNames << "bounds" << "displayed";
            }

            retval += updateChannels(paramNames);

            if (d->m_zStackChildPlot.states() & ChildPlotItem::StateShowPending)
            {
                d->m_zStackChildPlot.rStates() &= ~ChildPlotItem::StateShowPending;
                childFigure->setVisible(true);
            }
            else // we do not have a plot so we have to show it and its child of this plot
            {
                childFigure->show();
            }
        }
        else
        {
            if (bounds.size() == 3) // its a 3D-Object
            {
                paramNames << "bounds" << "sourceout";
            }
            else
            {
                paramNames << "bounds" << "displayed";
            }

            retval += updateChannels(paramNames);
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::displayZStackCut(const QVector<QPointF> &bounds, ito::uint32 *childFigureUID /*= NULL*/)
{
    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        return ito::RetVal(ito::retError, 0, tr("Could not spawn lineCut due to missing API-handle").toLatin1().data());
    }

    ito::RetVal retval = ito::retOk;
    QList<QString> paramNames;
    ito::uint32 UID = 0;
    QWidget *childPlot = NULL;
    ito::AbstractDObjFigure* childFigure = NULL;

    double *pointArr = new double[2 * bounds.size()];
    for (int np = 0; np < bounds.size(); np++)
    {
        pointArr[np * 2] = bounds[np].x();
        pointArr[np * 2 + 1] = bounds[np].y();
    }

    getOutputParam("zCutPoint")->setVal(pointArr, 2 * bounds.size());

    DELETE_AND_SET_NULL_ARRAY(pointArr);

    if (d->m_zStackChildPlot.isValid())
    {
        //there should exist a plot
        childPlot = d->m_zStackChildPlot.childFigure();
        UID = d->m_zStackChildPlot.UID();
    }

    if (UID == 0 || childPlot == NULL)
    {
        //try to create a new plot
        d->m_zStackChildPlot.rStates() = ChildPlotItem::StateShowPending;
        UID = 0;
        retval += apiGetFigure("DObjStaticLine", "", UID, &childPlot, this);

        if (!retval.containsError())
        {
            d->m_zStackChildPlot = ChildPlotItem(UID, childPlot, ChildPlotItem::StateShowPending);

            if (childPlot->inherits("ito::AbstractDObjFigure"))
            {
                childFigure = (ito::AbstractDObjFigure*)childPlot;
                connect(childPlot, SIGNAL(destroyed(QObject*)), this, SLOT(childFigureDestroyed(QObject*)));

                //try to active this 2d plot again -> activatePlot will then raise this 2d plot window. 
                //Then, the focus is tried to be set to the canvas to receive key-events (like H or V for horizontal or vertical lines)
                QTimer::singleShot(0, this, SLOT(activatePlot()));

                retval += moveChildPlotCloseToThis(childFigure);
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("the opened figure is not inherited from ito::AbstractDObjFigure").toLatin1().data());
            }
        }
    }

    if (!retval.containsError())
    {
        if (childFigureUID)
        {
            *childFigureUID = UID;
        }

        childFigure = (ito::AbstractDObjFigure*)childPlot;

        if (!(d->m_zStackChildPlot.states() & ChildPlotItem::StateChannelsConnected))
        {
            d->m_zStackChildPlot.rStates() &= ~ChildPlotItem::StateChannelsConnected;

            ((QMainWindow*)childFigure)->setWindowTitle(tr("Z-Stack"));
            if (childFigure->inherits("ItomQwtDObjFigure"))
            {
                ((ItomQwtDObjFigure*)childFigure)->setComplexStyle(d->m_pData->m_cmplxType);
            }

            QList<ito::AbstractNode::ParamNamePair> excludes;
            excludes << ParamNamePair("zCutPoint", "bounds");
            excludes << ParamNamePair("sourceout", "source");
            removeAllChannelsToReceiver((ito::AbstractNode*)childFigure, excludes);

            // for a linecut in z-direction we have to pass the input object to the linecut, otherwise the 1D-widget "sees" only a 2D object
            // with one plane and cannot display the points in z-direction
            retval += createChannel("zCutPoint", childFigure, "bounds", false);
            retval += createChannel("sourceout", childFigure, "source", false);
            paramNames << "zCutPoint" << "sourceout";

            retval += updateChannels(paramNames);

            if (d->m_zStackChildPlot.states() & ChildPlotItem::StateShowPending)
            {
                d->m_zStackChildPlot.rStates() &= ~ChildPlotItem::StateShowPending;
                childFigure->setVisible(true);
            }
            else // we do not have a plot so we have to show it and its child of this plot
            {
                childFigure->show();
            }
        }
        else
        {
            paramNames << "zCutPoint" << "sourceout";
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
    bool lineChildPlot = (d->m_lineCutChildPlot.childFigure() == obj);
    bool zStackChildPlot = (d->m_zStackChildPlot.childFigure() == obj);
    bool volumeChildPlot = (d->m_volumeCutChildPlot.childFigure() == obj);

    ito::uint32 childPlotUid = 0;

    if (lineChildPlot)
    {
        childPlotUid = d->m_lineCutChildPlot.UID();
        d->m_lineCutChildPlot = ChildPlotItem(); //reset it
    }

    if (zStackChildPlot)
    {
        childPlotUid = d->m_zStackChildPlot.UID();
        d->m_zStackChildPlot = ChildPlotItem(); //reset it
    }

    if (volumeChildPlot)
    {
        childPlotUid = d->m_volumeCutChildPlot.UID();
        d->m_volumeCutChildPlot = ChildPlotItem(); //reset it
    }

    m_pContent->removeChildPlotIndicators(lineChildPlot, zStackChildPlot, volumeChildPlot);

    PlotInfoPicker* pickerWid = pickerWidget();

    if (pickerWid)
    {
        if (childPlotUid == 0)
        {
            pickerWid->removeChildPlots();
        }
        else
        {
            pickerWid->removeChildPlot(childPlotUid);
        }
    }
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

    QWidget* widget = d->m_lineCutChildPlot.childFigure();

    if (widget && widget->inherits("ito::AbstractDObjFigure"))
    {
        return (qobject_cast<ito::AbstractDObjFigure*>(widget))->getDisplayed();
    }
    else
    {
        return QSharedPointer<ito::DataObject>();
    }
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
        m_pContent->refreshPlot(getInputParam("source")->getVal<const ito::DataObject*>());
    }

    updatePropertyDock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::ItomPlotHandle Itom2dQwtPlot::getLineCutPlotItem() const
{
    if (m_pContent && d->m_lineCutChildPlot.isValid())
    {
        ito::ItomPlotHandle handle;
        if (apiGetItomPlotHandleByID(d->m_lineCutChildPlot.UID(), handle) == ito::retOk)
        {
            return handle;
        }
    }

    return ito::ItomPlotHandle(NULL, NULL, 0);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setLineCutPlotItem(const ito::ItomPlotHandle &plotHandle)
{
    ito::RetVal retval = ito::retOk;
    QWidget *lineCutObj = NULL;

    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        return;
    }

    if (m_pContent)
    {
        if (plotHandle.getObjectID() == 0) //invalidate an existing line cut object
        {
            if (d->m_lineCutChildPlot.isValid())
            {
                lineCutObj = d->m_lineCutChildPlot.childFigure();
                ito::AbstractFigure *af = qobject_cast<ito::AbstractFigure*>(lineCutObj); //lineCutObj is a QWidget, however AbstractFigure is derived from QMainWindow and AbstractNode. Therefore this upcast... 

                if (!retval.containsError() && lineCutObj && af)
                {
                    childFigureDestroyed(d->m_lineCutChildPlot.childFigure());
                    m_pContent->removeChildPlotIndicators(true, false, false, true);

                    removeAllChannelsToReceiver((ito::AbstractNode*)af); //...and here the cast to AbstractNode again. Else, there would be an error
                }
            }

            d->m_lineCutChildPlot = ChildPlotItem();
        }
        else
        {
            ito::uint32 thisFigureUid = 0;
            ito::uint32 destinationFigureUid = -1;
            retval += apiGetFigureIDbyHandle(this, thisFigureUid);

            if (plotHandle.getObjectID() == thisFigureUid || retval.containsError())
            {
                //line cut plot handle cannot be the same than this 2d plot instance
                return;
            }
            else
            {
                destinationFigureUid = plotHandle.getObjectID();
            }

            if (destinationFigureUid > 0)
            {
                //plotHandle is valid
                retval += apiGetFigure("DObjStaticLine", "", destinationFigureUid, &lineCutObj, this);
            }
            else
            {
                //plotHandle is 0 (None), invalidate the connection to an existing 1d plot widget for line cuts
                lineCutObj = NULL;
            }

            if (lineCutObj == NULL || (!lineCutObj->inherits("ito::AbstractDObjFigure")))
            {
                d->m_lineCutChildPlot = ChildPlotItem(); //invalidate the line cut
            }
            else
            {
                ito::AbstractFigure *af = qobject_cast<ito::AbstractFigure*>(lineCutObj);
                if (af->getInputParam("bounds") == NULL || af->getInputParam("source") == NULL)
                {
                    d->m_lineCutChildPlot = ChildPlotItem(); //invalidate the line cut
                }
                else
                {
                    d->m_lineCutChildPlot = ChildPlotItem(destinationFigureUid, lineCutObj, ChildPlotItem::StateShowPending);
                }
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::ItomPlotHandle Itom2dQwtPlot::getZSlicePlotItem() const
{
    if (m_pContent && d->m_zStackChildPlot.isValid())
    {
        ito::ItomPlotHandle handle;
        if (apiGetItomPlotHandleByID(d->m_zStackChildPlot.UID(), handle) == ito::retOk)
        {
            return handle;
        }
    }

    return ito::ItomPlotHandle(NULL, NULL, 0);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setZSlicePlotItem(const ito::ItomPlotHandle &plotHandle)
{
    ito::RetVal retval = ito::retOk;
    QWidget *zStackObj = NULL;

    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        return;
    }

    if (m_pContent)
    {
        if (plotHandle.getObjectID() == 0) //invalidate an existing line cut object
        {
            if (d->m_zStackChildPlot.isValid())
            {
                zStackObj = d->m_zStackChildPlot.childFigure();
                ito::AbstractFigure *af = qobject_cast<ito::AbstractFigure*>(zStackObj); //lineCutObj is a QWidget, however AbstractFigure is derived from QMainWindow and AbstractNode. Therefore this upcast... 

                if (!retval.containsError() && zStackObj && af)
                {
                    childFigureDestroyed(d->m_zStackChildPlot.childFigure());
                    m_pContent->removeChildPlotIndicators(true, false, false, true);

                    removeAllChannelsToReceiver((ito::AbstractNode*)af); //...and here the cast to AbstractNode again. Else, there would be an error
                }
            }

            d->m_zStackChildPlot = ChildPlotItem();
        }
        else
        {
            ito::uint32 thisFigureUid = 0;
            ito::uint32 destinationFigureUid = -1;
            retval += apiGetFigureIDbyHandle(this, thisFigureUid);

            if (plotHandle.getObjectID() == thisFigureUid || retval.containsError())
            {
                //line cut plot handle cannot be the same than this 2d plot instance
                return;
            }
            else
            {
                destinationFigureUid = plotHandle.getObjectID();
            }

            if (destinationFigureUid > 0)
            {
                //plotHandle is valid
                retval += apiGetFigure("DObjStaticLine", "", destinationFigureUid, &zStackObj, this);
            }
            else
            {
                //plotHandle is 0 (None), invalidate the connection to an existing 1d plot widget for line cuts
                zStackObj = NULL;
            }

            if (zStackObj == NULL || (!zStackObj->inherits("ito::AbstractDObjFigure")))
            {
                d->m_zStackChildPlot = ChildPlotItem(); //invalidate the line cut
            }
            else
            {
                ito::AbstractFigure *af = qobject_cast<ito::AbstractFigure*>(zStackObj);
                if (af->getInputParam("bounds") == NULL || af->getInputParam("source") == NULL)
                {
                    d->m_zStackChildPlot = ChildPlotItem(); //invalidate the line cut
                }
                else
                {
                    d->m_zStackChildPlot = ChildPlotItem(destinationFigureUid, zStackObj, ChildPlotItem::StateShowPending);
                }
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::ItomPlotHandle Itom2dQwtPlot::getVolumeCutPlotItem() const
{
    if (m_pContent && d->m_volumeCutChildPlot.isValid())
    {
        ito::ItomPlotHandle handle;
        if (apiGetItomPlotHandleByID(d->m_volumeCutChildPlot.UID(), handle) == ito::retOk)
        {
            return handle;
        }
    }

    return ito::ItomPlotHandle(NULL, NULL, 0);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setVolumeCutPlotItem(const ito::ItomPlotHandle &plotHandle)
{
    ito::RetVal retval = ito::retOk;
    QWidget *volumeObj = NULL;

    if (!ito::ITOM_API_FUNCS_GRAPH)
    {
        return;
    }

    if (m_pContent)
    {
        if (plotHandle.getObjectID() == 0) //invalidate an existing line cut object
        {
            if (d->m_volumeCutChildPlot.isValid())
            {
                volumeObj = d->m_volumeCutChildPlot.childFigure();
                ito::AbstractFigure *af = qobject_cast<ito::AbstractFigure*>(volumeObj); //volumeObj is a QWidget, however AbstractFigure is derived from QMainWindow and AbstractNode. Therefore this upcast... 

                if (!retval.containsError() && volumeObj && af)
                {
                    childFigureDestroyed(d->m_volumeCutChildPlot.childFigure());
                    m_pContent->removeChildPlotIndicators(true, false, false, true);

                    removeAllChannelsToReceiver((ito::AbstractNode*)af); //...and here the cast to AbstractNode again. Else, there would be an error
                }
            }

            d->m_volumeCutChildPlot = ChildPlotItem();
        }
        else
        {
            ito::uint32 thisFigureUid = 0;
            ito::uint32 destinationFigureUid = -1;
            retval += apiGetFigureIDbyHandle(this, thisFigureUid);

            if (plotHandle.getObjectID() == thisFigureUid || retval.containsError())
            {
                //line cut plot handle cannot be the same than this 2d plot instance
                return;
            }
            else
            {
                destinationFigureUid = plotHandle.getObjectID();
            }

            if (destinationFigureUid > 0)
            {
                //plotHandle is valid
                retval += apiGetFigure("DObjStaticLine", "", destinationFigureUid, &volumeObj, this);
            }
            else
            {
                //plotHandle is 0 (None), invalidate the connection to an existing 1d plot widget for line cuts
                volumeObj = NULL;
            }

            if (volumeObj == NULL || (!volumeObj->inherits("ito::AbstractDObjFigure")))
            {
                d->m_volumeCutChildPlot = ChildPlotItem(); //invalidate the line cut
            }
            else
            {
                ito::AbstractFigure *af = qobject_cast<ito::AbstractFigure*>(volumeObj);
                if (af->getInputParam("bounds") == NULL || af->getInputParam("source") == NULL)
                {
                    d->m_volumeCutChildPlot = ChildPlotItem(); //invalidate the line cut
                }
                else
                {
                    d->m_volumeCutChildPlot = ChildPlotItem(destinationFigureUid, volumeObj, ChildPlotItem::StateShowPending);
                }
            }
        }
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
    getInputParam("bounds")->setVal(pointArr, 2 * bounds.size());
    delete[] pointArr;
}
//----------------------------------------------------------------------------------------------------------------------------------
QVector<QPointF> Itom2dQwtPlot::getBounds(void) const
{
    int numPts = getInputParam("bounds")->getLen();
    QVector<QPointF> boundsVec;

    if (numPts > 0)
    {
        const double *ptsDblVec = getInputParam("bounds")->getVal<const double*>();
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

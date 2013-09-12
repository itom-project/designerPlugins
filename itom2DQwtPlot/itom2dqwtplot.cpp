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
#include <qwt_plot_renderer.h>
#include <qmenu.h>
#include "dialog2DScale.h"
#include <qwt_text_label.h>
#include <qwt_scale_widget.h>

//----------------------------------------------------------------------------------------------------------------------------------
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
    m_pActPlaneSelector(NULL),
    m_pActCmplxSwitch(NULL),
	m_mnuCmplxSwitch(NULL),
    m_pActCoordinates(NULL),
    m_pCoordinates(NULL)
{
	m_pOutput.insert("bounds", new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, QObject::tr("Points for line plots from 2d objects").toAscii().data()));
    m_pOutput.insert("sourceout", new ito::Param("sourceout", ito::ParamBase::DObjPtr, NULL, QObject::tr("shallow copy pass through of input source object").toAscii().data()));

	int id = qRegisterMetaType<QSharedPointer<ito::DataObject> >("QSharedPointer<ito::DataObject>");

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
    m_data.m_xaxisVisible = true;
    m_data.m_yaxisVisible = true;
    m_data.m_colorBarVisible = false;
    m_data.m_cmplxType = PlotCanvas::Real;
    m_data.m_yaxisFlipped = false;
    m_data.m_pConstOutput = &m_pOutput;
	m_data.m_state = PlotCanvas::tIdle;

	//initialize canvas
	m_pContent = new PlotCanvas(&m_data, this);
    connect(m_pContent, SIGNAL(statusBarClear()), statusBar(), SLOT(clearMessage()));
    connect(m_pContent, SIGNAL(statusBarMessage(QString)), statusBar(), SLOT(showMessage(QString)));
    connect(m_pContent, SIGNAL(statusBarMessage(QString,int)), statusBar(), SLOT(showMessage(QString,int)));
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
    mainTb->addAction(m_pActCmplxSwitch);
    mainTb->addAction(m_pActCoordinates);
}

//----------------------------------------------------------------------------------------------------------------------------------
Itom2dQwtPlot::~Itom2dQwtPlot()
{
	m_pContent->deleteLater();
	m_pContent = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::createActions()
{
	QAction *a = NULL;
	
	//m_actSave
    m_pActSave = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("Save"), this);
    a->setObjectName("actSave");
    a->setToolTip(tr("Export current view"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActSave()));

	//m_actHome
    m_pActHome = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"), tr("Home"), this);
    a->setObjectName("actHome");
    a->setToolTip(tr("Reset original view"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActHome()));

	//m_actPan
    m_pActPan = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), tr("move"), this);
    a->setObjectName("actPan");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Pan axes with left mouse, zoom with right"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActPan(bool)));

	//m_actZoom
    m_pActZoom = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"), tr("zoom to rectangle"), this);
    a->setObjectName("actZoom");
    a->setCheckable(true);
    a->setChecked(false);
    a->setToolTip(tr("Zoom to rectangle"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActZoom(bool)));

    //m_actScaleSetting
    m_pActScaleSettings = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/autoscal.png"), tr("Scale Settings"), this);
    a->setObjectName("actScaleSetting");
    a->setToolTip(tr("Set the ranges and offsets of this view"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActScaleSettings()));

    //m_actPalette
    m_pActColorPalette = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/colorPalette.png"), tr("Palette"), this);
    a->setObjectName("actColorPalette");
    a->setToolTip(tr("Switch between color palettes"));
    connect(a, SIGNAL(triggered()), this, SLOT(mnuActColorPalette()));

    //m_actToggleColorBar
    m_pActToggleColorBar = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/colorbar.png"), tr("Show Colorbar"), this);
    a->setCheckable(true);
    a->setObjectName("actShowColorBar");
    a->setToolTip(tr("Toggle visibility of the color bar on right canvas side"));
	connect(a,SIGNAL(toggled(bool)),this,SLOT(mnuActToggleColorBar(bool)));

    //m_actMarker
    m_pActValuePicker = a = new QAction(QIcon(":/itomDesignerPlugins/general/icons/marker.png"), tr("marker"), this);
    a->setObjectName("actValuePicker");
    a->setCheckable(true);
    a->setChecked(false);
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActValuePicker(bool)));

    //m_actLineCut
    m_pActLineCut = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"), tr("Linecut"), this);
    a->setCheckable(true);
    a->setObjectName("actLineCut");
    a->setToolTip(tr("Show a in plane line cut"));
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActLineCut(bool)));

    //m_actStackCut
    m_pActStackCut = a = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/zStack.png"), tr("Slice in z-direction"), this);
    a->setObjectName("actStackCut");
    a->setToolTip(tr("Show a slice through z-Stack"));
    a->setCheckable(true);
    a->setVisible(false);
    connect(a, SIGNAL(triggered(bool)), this, SLOT(mnuActStackCut(bool)));

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

    //m_actCmplxSwitch
    m_pActCmplxSwitch = new QAction(QIcon(":/itomDesignerPlugins/complex/icons/ImRe.png"), tr("Switch Imag, Real, Abs, Pha"), this);
	m_mnuCmplxSwitch = new QMenu("Complex Switch");

    QActionGroup *m_pCmplxActGroup = new QActionGroup(this);
    a = m_pCmplxActGroup->addAction(tr("Real"));
    a->setData(PlotCanvas::Real);
    m_mnuCmplxSwitch->addAction(a);
    a->setCheckable(true);

	a = m_pCmplxActGroup->addAction(tr("Imag"));
    a->setData(PlotCanvas::Imag);
    m_mnuCmplxSwitch->addAction(a);
    a->setCheckable(true);

	a = m_pCmplxActGroup->addAction(tr("Abs"));
    a->setData(PlotCanvas::Abs);
    m_mnuCmplxSwitch->addAction(a);
    a->setCheckable(true);
    a->setChecked(true);

	a = m_pCmplxActGroup->addAction(tr("Pha"));
    a->setData(PlotCanvas::Phase);
    m_mnuCmplxSwitch->addAction(a);
    a->setCheckable(true);

	m_pActCmplxSwitch->setMenu(m_mnuCmplxSwitch);
    m_pActCmplxSwitch->setVisible(false);
    connect(m_pCmplxActGroup, SIGNAL(triggered(QAction*)), this, SLOT(mnuCmplxSwitch(QAction*)));

    m_pCoordinates = new QLabel("[0.0; 0.0]\n[0.0; 0.0]", this);
    m_pCoordinates->setAlignment(Qt::AlignRight | Qt::AlignTop);
    m_pCoordinates->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    m_pCoordinates->setObjectName("lblCoordinates");

    m_pActCoordinates = new QWidgetAction(this);
	m_pActCoordinates->setDefaultWidget(m_pCoordinates);
    m_pActCoordinates->setVisible(false);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::applyUpdate()
{
    //displayed and sourceout is set by dataObjRasterData, since the data is analyzed there
    m_pContent->refreshPlot(m_pInput["source"]->getVal<ito::DataObject*>());

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
    if (m_data.m_autoTitle)
    {
        return "<auto>";
    }
    return m_data.m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setTitle(const QString &title)
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
void Itom2dQwtPlot::resetTitle()
{
    m_data.m_autoTitle = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getxAxisLabel() const
{
    if (m_data.m_autoxAxisLabel)
    {
        return "<auto>";
    }
    return m_data.m_xaxisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setxAxisLabel(const QString &label)
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
void Itom2dQwtPlot::resetxAxisLabel()
{
    m_data.m_autoxAxisLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getyAxisLabel() const
{
    if (m_data.m_autoyAxisLabel)
    {
        return "<auto>";
    }
    return m_data.m_yaxisLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisLabel(const QString &label)
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
void Itom2dQwtPlot::resetyAxisLabel()
{
    m_data.m_autoyAxisLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
QString Itom2dQwtPlot::getValueLabel() const
{
    if (m_data.m_autoValueLabel)
    {
        return "<auto>";
    }
    return m_data.m_valueLabel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setValueLabel(const QString &label)
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
void Itom2dQwtPlot::resetValueLabel()
{
    m_data.m_autoValueLabel = true;
    if (m_pContent) m_pContent->updateLabels();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getyAxisFlipped() const
{
    return m_data.m_yaxisFlipped;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisFlipped(const bool &value)
{
    m_data.m_yaxisFlipped = value;

    if (m_pContent) 
    {
        m_pContent->updateScaleValues();
        m_pContent->internalDataUpdated();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getxAxisVisible() const
{
    return m_data.m_xaxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setxAxisVisible(const bool &value)
{
    m_data.m_xaxisVisible = value;

    if (m_pContent) m_pContent->enableAxis(QwtPlot::xBottom, value);
}
    
//----------------------------------------------------------------------------------------------------------------------------------
bool Itom2dQwtPlot::getyAxisVisible() const
{
    return m_data.m_yaxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setyAxisVisible(const bool &value)
{
    m_data.m_yaxisVisible = value;

    if (m_pContent) m_pContent->enableAxis(QwtPlot::yLeft, value);
}

//----------------------------------------------------------------------------------------------------------------------------------
QPointF Itom2dQwtPlot::getXAxisInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getInterval(Qt::XAxis);
    }
    return QPointF();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setXAxisInterval(QPointF point)
{
    if (m_pContent)
    {
        m_pContent->setInterval(Qt::XAxis, point);
    }
}
        
//----------------------------------------------------------------------------------------------------------------------------------
QPointF Itom2dQwtPlot::getYAxisInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getInterval(Qt::YAxis);
    }
    return QPointF();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setYAxisInterval(QPointF point)
{
    if (m_pContent)
    {
        m_pContent->setInterval(Qt::YAxis, point);
    }
}
   
//----------------------------------------------------------------------------------------------------------------------------------
QPointF Itom2dQwtPlot::getZAxisInterval(void) const
{
    if (m_pContent)
    {
        return m_pContent->getInterval(Qt::ZAxis);
    }
    return QPointF();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setZAxisInterval(QPointF point)
{
    if (m_pContent)
    {
        m_pContent->setInterval(Qt::ZAxis, point);
    }
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
        //m_pContent->replot();
    }
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
    filter += tr("PDF Documents (*.pdf)");
#ifndef QWT_NO_SVG
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
        QwtPlotRenderer renderer;

        // flags to make the document look like the widget
        renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
        //renderer.setLayoutFlag(QwtPlotRenderer::KeepFrames, true); //deprecated in qwt 6.1.0

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
    if (checked)
    {
        m_pActValuePicker->setChecked(false);
        m_pActZoom->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActStackCut->setChecked(false);
        m_pContent->setState(PlotCanvas::tPan);
    }
    else
    {
        m_pContent->setState(PlotCanvas::tIdle);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActZoom(bool checked)
{
    if (checked)
    {
        m_pActValuePicker->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActStackCut->setChecked(false);
        m_pContent->setState(PlotCanvas::tZoom);
    }
    else
    {
        m_pContent->setState(PlotCanvas::tIdle);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActScaleSettings()
{
    Dialog2DScale *dlg = new Dialog2DScale(m_data, this);
    if (dlg->exec() == QDialog::Accepted)
    {
        dlg->getData(m_data);

        m_pContent->updateScaleValues();
    }

    delete dlg;
    dlg = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActColorPalette()
{
    if (m_pContent) m_pContent->setColorMap("__next__");
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActToggleColorBar(bool checked)
{
    if (m_pContent) m_pContent->setColorBarVisible(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActValuePicker(bool checked)
{
    if (checked)
    {
        m_pActZoom->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActStackCut->setChecked(false);
    }
    
    m_pContent->setState(checked ? PlotCanvas::tValuePicker : PlotCanvas::tIdle);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActLineCut(bool checked)
{
	if (checked)
    {
        m_pActZoom->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActStackCut->setChecked(false);
        m_pActValuePicker->setChecked(false);
    }
    
    m_pContent->setState(checked ? PlotCanvas::tLineCut : PlotCanvas::tIdle);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActStackCut(bool checked)
{
	if (checked)
    {
        m_pActZoom->setChecked(false);
        m_pActPan->setChecked(false);
        m_pActLineCut->setChecked(false);
        m_pActValuePicker->setChecked(false);
    }
    
    m_pContent->setState(checked ? PlotCanvas::tStackCut : PlotCanvas::tIdle);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuActPlaneSelector(int plane)
{
    if (m_pContent) m_pContent->changePlane(plane);

	QStringList paramNames;
    paramNames << "displayed";
    updateChannels(paramNames);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setPlaneRange(int min, int max)
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
		m_pActStackCut->setVisible((max-min) > 0);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::mnuCmplxSwitch(QAction *action)
{
	setCmplxSwitch((PlotCanvas::ComplexType)(action->data().toInt()), true);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setCmplxSwitch(PlotCanvas::ComplexType type, bool visible)
{
    m_pActCmplxSwitch->setVisible(visible);

    if (m_data.m_cmplxType != type)
    {

        if (visible)
        {
            m_data.m_cmplxType = type;

            switch (type)
            {
            case PlotCanvas::Imag:
                m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReImag.png"));
                break;
            case PlotCanvas::Real:
                m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReReal.png"));
                break;
            case PlotCanvas::Phase:
                m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImRePhase.png"));
                break;
            case PlotCanvas::Abs:
                m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReAbs.png"));
                break;
            }
        }

        if (m_pContent) m_pContent->internalDataUpdated();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::displayCut(QVector<QPointF> bounds, ito::uint32 &uniqueID, bool zStack /*= false*/)
{
    ito::RetVal retval = ito::retOk;
    QList<QString> paramNames;
    ito::uint32 newUniqueID = uniqueID;
    QWidget *lineCutObj = NULL;

	double *pointArr = new double[2 * bounds.size()];
    for (int np = 0; np < bounds.size(); np++)
    {
        pointArr[np * 2] = bounds[np].x();
        pointArr[np * 2 + 1] = bounds[np].y();
    }
    m_pOutput["bounds"]->setVal(pointArr, 2 * bounds.size());
    delete pointArr;
    //setOutpBounds(bounds);
    //setLinePlotCoordinates(bounds);

    retval += apiGetFigure("DObjStaticLine","", newUniqueID, &lineCutObj, this); //(newUniqueID, "itom1DQwtFigure", &lineCutObj);

    if (!retval.containsError())
    {
        

        if (uniqueID != newUniqueID)
        {
            uniqueID = newUniqueID;
            ito::AbstractDObjFigure* figure = NULL;
            if (lineCutObj->inherits("ito::AbstractDObjFigure"))
			{
                figure = (ito::AbstractDObjFigure*)lineCutObj;
                m_childFigures[lineCutObj] = newUniqueID;
                connect(lineCutObj, SIGNAL(destroyed(QObject*)), this, SLOT(childFigureDestroyed(QObject*)));
			}
            else
			{
                return ito::RetVal(ito::retError, 0, tr("the opened figure is not inherited from ito::AbstractDObjFigure").toAscii().data());
			}

            retval += addChannel((ito::AbstractNode*)figure, m_pOutput["bounds"], figure->getInputParam("bounds"), ito::Channel::parentToChild, 0, 1);
            
            if (zStack)
            {
                // for a linecut in z-direction we have to pass the input object to the linecut, otherwise the 1D-widget "sees" only a 2D object
                // with one plane and cannot display the points in z-direction

                retval += addChannel((ito::AbstractNode*)figure,  m_pOutput["sourceout"], figure->getInputParam("source"), ito::Channel::parentToChild, 0, 1);
                paramNames << "bounds"  << "sourceout";
            }
            else
            {
                // otherwise simply pass on the displayed plane
                retval += addChannel((ito::AbstractNode*)figure, m_pOutput["displayed"], figure->getInputParam("source"), ito::Channel::parentToChild, 0, 1);
                paramNames << "bounds"  << "displayed";
            }

            retval += updateChannels(paramNames);

            figure->show();
        }
        else
        {
            if (zStack)
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
void Itom2dQwtPlot::childFigureDestroyed(QObject *obj)
{
    QHash<QObject*,ito::uint32>::iterator it = m_childFigures.find(obj);

    if (it != m_childFigures.end())
    {
        m_pContent->childFigureDestroyed(obj, m_childFigures[obj]);
    }
    else
    {
        m_pContent->childFigureDestroyed(obj, 0);
    }

    m_childFigures.erase(it);
}

////----------------------------------------------------------------------------------------------------------------------------------
//void Itom2dQwtPlot::setLinePlotCoordinates(const QVector<QPointF> pts)
//{
//    char buf[60] = {0};
//    if (pts.size() > 1)
//    {
//        sprintf(buf, "[%.4g; %.4g]\n[%.4g; %.4g]", pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
//    }
//    else if (pts.size() == 1)
//    {
//        sprintf(buf, "[%.4g; %.4g]\n[ - ; - ]", pts[0].x(), pts[0].y());
//    }
//    else
//    {
//        sprintf(buf, "[ - ; - ]\n[ - ; - ]");
//    }
//    m_lblCoordinates->setText(buf);
//}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::plotMarkers(const ito::DataObject &coords, QString style, QString id /*= QString::Null()*/, int plane /*= -1*/)
{
	return m_pContent->plotMarkers(&coords, style, id, plane);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Itom2dQwtPlot::deleteMarkers(QString id)
{
	return m_pContent->deleteMarkers(id);
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::userInteractionStart(int type, bool start, int maxNrOfPoints /*= -1*/)
{
    m_pActValuePicker->setChecked(false);
    m_pActZoom->setChecked(false);
    m_pActPan->setChecked(false);
    m_pActLineCut->setChecked(false);
    m_pActStackCut->setChecked(false);

    m_pContent->userInteractionStart(type, start, maxNrOfPoints);

    //m_pContent->setWindowState((m_pContent->windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
    //m_pContent->raise(); //for MacOS
    //m_pContent->activateWindow(); //for Windows
    //m_pContent->setFocus();
}

//----------------------------------------------------------------------------------------------------------------------------------
void Itom2dQwtPlot::setCoordinates(const QVector<QPointF> &pts, bool visible)
{
    m_pActCoordinates->setVisible(visible);

    if (visible)
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
        m_pCoordinates->setText(buf);
    }
}
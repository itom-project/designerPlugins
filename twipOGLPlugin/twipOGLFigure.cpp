/* ********************************************************************
#    twipOGLFigure-Plugin for itom
#    URL: http://www.twip-os.com
#    Copyright (C) 2014, twip optical solutions GmbH,
#    Stuttgart, Germany
#
#    This files is part of the designer-Plugin twipOGLFigure for the
#    measurement software itom. All files of this plugin, where not stated
#    as port of the itom sdk, fall under the GNU Library General
#    Public Licence and must behandled accordingly.
#
#    twipOGLFigure is free software; you can redistribute it and/or modify it
#    under the terms of the GNU Library General Public Licence as published by
#    the Free Software Foundation; either version 2 of the Licence, or (at
#    your option) any later version.
#
#    Information under www.twip-os.com or mail to info@twip-os.com.
#
#    This itom-plugin is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#    GNU General Public License for more details.
#
#    itom is free software by ITO, University Stuttgart published under
#    GNU General Public License as published by the Free Software
#    Foundation. See <https://github.com/itom-project/itom>
#
#    You should have received a copy of the GNU Library General Public License
#    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#include "twipOGLFigure.h"
#include "aboutTwip.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QImageWriter>
#include <QFileInfo>
#include <QMenu>
#include <QWidgetAction>
#include <QClipboard>
#include <QApplication>

using namespace ito;


int NTHREADS = 2;
//int TwipOGLFigure::m_pclID = 0;

//----------------------------------------------------------------------------------------------------------------------------------
TwipOGLFigure::TwipOGLFigure(const QString &itomSettingsFile, const ito::ParamBase::Type inpType, AbstractFigure::WindowMode windowMode, QWidget *parent) :
    AbstractDObjPclFigure(itomSettingsFile, inpType, windowMode, parent),
#if !USEWIDGETEVENTS
    m_pEventFilter(NULL),
#endif
    m_pContent(NULL),
    m_pActScaleSetting(NULL),
    m_pActPalette(NULL),
    m_pActToggleColorBar(NULL),
    m_pActChangeBGColor(NULL),
    m_pActSave(NULL),
    m_pActHome(NULL),
    m_pActRiseZAmpl(NULL),
    m_pActReduceZAmpl(NULL),
    m_pToggleIllumination(NULL),
    m_pToggleIlluminationRotation(NULL),
    m_pActCmplxSwitch(NULL),
    m_pMnuCmplxSwitch(NULL),
    m_pActTringModeSwitch(NULL),
    m_pMnuTringModeSwitch(NULL),
    m_pToggleInfoText(NULL),
    m_pLblCoordinates(NULL),
    m_pOverlaySlider(NULL),
    m_pActOverlaySlider(NULL),
    m_pActLegend(NULL),
    m_pActProperties(NULL),
    m_pLegend(NULL),
    m_pclID(0),
    m_dObjID(0)
{
    m_updateLook = 0;
    m_pvConfigData = (void*) new InternalData;

    addOutputParam(new ito::Param("bounds", ito::ParamBase::DoubleArray, NULL, tr("Points for line plots from 2d objects").toLatin1().data()));

    int id = qRegisterMetaType<QSharedPointer<ito::DataObject> >("QSharedPointer<ito::DataObject>");

    QMenu *contextMenu = new QMenu(tr("plotISO"), this);

#if QT_VERSION >= 0x050400
	QSurfaceFormat fmt;
	//fmt.setStereo(0);
	//    if (fmt.swapInterval() != -1)
	//        fmt.setSwapInterval(0);
	fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(24);
    fmt.setSwapBehavior(QSurfaceFormat::SingleBuffer);
	fmt.setVersion(3, 3);
	QSurfaceFormat::setDefaultFormat(fmt);

	m_pContent = new TwipOGLWidget(contextMenu, m_pvConfigData, fmt, this, 0);
#else
    QGLFormat fmt;
    fmt.setOverlay(0);
    fmt.setDepth(1);
//    fmt.setDoubleBuffer(0);
    fmt.setDirectRendering(1);
    fmt.setStereo(0);
//    if (fmt.swapInterval() != -1)
//        fmt.setSwapInterval(0);
    fmt.setProfile(QGLFormat::CoreProfile);

    int glVer = QGLFormat::openGLVersionFlags();
    if (glVer >= 32)
    {
        fmt.setVersion(1, 5);
    }
    QGLFormat::setDefaultFormat(fmt);

    m_pContent = new TwipOGLWidget(contextMenu, m_pvConfigData, fmt, this, 0);
#endif
    m_pContent->setObjectName("canvasWidget");
    setCentralWidget(m_pContent);

    m_pLegendDock = new QDockWidget(tr("Legend"), this);
    m_pLegendDock->setVisible(false);
    m_pLegendDock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);

    m_pLegend = new TwipLegend(m_pLegendDock, this);
    m_pLegend->setObjectName("LegendView");
    m_pLegendDock->setWidget(m_pLegend);

    switch (getWindowMode())
    {
        case AbstractFigure::ModeInItomFigure:
            /*default if figure is used for plotting data in itom, may also be part of a subfigure area.
            Then, the created DockWidget should be used by the outer window and managed/displayed by it */
            addDockWidget(Qt::LeftDockWidgetArea, m_pLegendDock);
            break;
        case AbstractFigure::ModeStandaloneInUi:
            /*figure is contained in an user interface. Then the dock widget is dock with floating mode (default) */
            addDockWidget(Qt::LeftDockWidgetArea, m_pLegendDock);
            m_pLegendDock->setFloating(true);
            break;

        case AbstractFigure::ModeStandaloneWindow:
            addDockWidget(Qt::LeftDockWidgetArea, m_pLegendDock);
            break;
    }

    //m_pContent->setFocus();

    QAction* tempAction = NULL;
    QMenu* tempMenu = NULL;

    //m_actHome
    m_pActHome = tempAction = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"), tr("Home"), this);
    //m_pActHome = tempAction = new QAction(QIcon(":/twipDesignerPlugins/general/icons/home.png"), tr("Home"), this);
    tempAction->setObjectName("actHome");
    tempAction->setToolTip(tr("Reset original view"));

    //m_actHome
    m_pKeepAspectRatio = tempAction = new QAction(QIcon(":/itomDesignerPlugins/aspect/icons/AspRatio11.png"), tr("Aspect ratio"), this);
    //m_pActHome = tempAction = new QAction(QIcon(":/twipDesignerPlugins/general/icons/home.png"), tr("Home"), this);
    tempAction->setObjectName("aspRatio");
    tempAction->setToolTip(tr("Set aspect ratio between x- and y-Axis"));

    //m_actSave
    m_pActSave = tempAction = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("Save..."), this);
    //m_pActSave = tempAction = new QAction(QIcon(":/twipDesignerPlugins/general/icons/ioIcon.png"), tr("Save..."), this);
    tempAction->setObjectName("actSave");
    tempAction->setToolTip(tr("Export current view..."));
    tempAction->setVisible(true);

    //m_actScaleSetting
    m_pActScaleSetting = tempAction = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/autoScale.png"), tr("Scale Settings..."), this);
    //m_pActScaleSetting = tempAction = new QAction(QIcon(":/twipDesignerPlugins/plot/icons/rescale.png"), tr("Scale Settings..."), this);
    tempAction->setObjectName("actScaleSetting");
    tempAction->setToolTip(tr("Set the ranges and offsets oif this view"));

    //m_actScaleSetting
    m_pActReduceZAmpl = tempAction = new QAction(QIcon(":/twipDesignerPlugins/plot/icons/lowZScale.png"), tr("reduce Z-Amplication..."), this);
    //m_pActScaleSetting = tempAction = new QAction(QIcon(":/twipDesignerPlugins/plot/icons/zScale.png"), tr("Scale Settings..."), this);
    tempAction->setObjectName("actReduceZAmpl");
    tempAction->setToolTip(tr("Reduce the current z amplication factor."));

    //m_actScaleSetting
    m_pActRiseZAmpl = tempAction = new QAction(QIcon(":/twipDesignerPlugins/plot/icons/highZScale.png"), tr("rise Z-Amplication..."), this);
    //m_pActScaleSetting = tempAction = new QAction(QIcon(":/twipDesignerPlugins/plot/icons/zScale.png"), tr("Scale Settings..."), this);
    tempAction->setObjectName("actRiseZAmpl");
    tempAction->setToolTip(tr("Increase the current z amplication factor."));

    //m_actPalette
    m_pActPalette = tempAction = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/colorPalette.png"), tr("Palette"), this);
    //m_pActPalette = tempAction = new QAction(QIcon(":/twipDesignerPlugins/plot/icons/changeColor.png"), tr("Palette"), this);
    tempAction->setObjectName("TogglePalette");
    tempAction->setToolTip(tr("Switch between color palettes"));

    //m_actToggleColorBar
    m_pActToggleColorBar = tempAction = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/colorbar.png"), tr("Show Colorbar"), this);
    //m_pActToggleColorBar = tempAction = new QAction(QIcon(":/twipDesignerPlugins/plot/icons/colorBar.png"), tr("Show Colorbar"), this);
    tempAction->setObjectName("ShowColorBar");
    tempAction->setToolTip(tr("Toggle visibility of the color bar on right canvas side"));

    //m_pActLegendIcon
    m_pActLegendIcon = tempAction = new QAction(QIcon(":/icons/listview.png"), tr("Show legend widget"), this);
    //m_pActToggleColorBar = tempAction = new QAction(QIcon(":/twipDesignerPlugins/plot/icons/colorBar.png"), tr("Show Colorbar"), this);
    tempAction->setCheckable(true);
    tempAction->setObjectName("ShowLegend");
    tempAction->setToolTip(tr("Toggle visibility of the legend dockwidget"));

    //m_actChangeBGColor
    m_pActChangeBGColor = tempAction = new QAction(QIcon(":/itomDesignerPlugins/plot/icons/blkorwht.png"), tr("Change Background Color"), this);
    //m_pActChangeBGColor = tempAction = new QAction(QIcon(":/twipDesignerPlugins/plot/icons/bgColor.png"), tr("Change Background Color"), this);
    tempAction->setObjectName("Change Background Color");
    tempAction->setToolTip(tr("Switch between the different background colors"));

    //m_toggleIllumination
    m_pToggleIllumination = tempAction = new QAction(QIcon(":/twipOGLFigure/icons/light.png"), tr("Enable Illumination"), this);
    //m_pToggleIllumination = tempAction = new QAction(QIcon(":/twipOGLFigure/icons/light.png"), tr("Enable Illumination"), this);
    tempAction->setCheckable(true);
    tempAction->setObjectName("Enable illumination rendering");
    tempAction->setToolTip(tr("Enable illumination rendering"));

    //m_toggleIlluminationRotation
    m_pToggleIlluminationRotation = tempAction = new QAction(QIcon(":/twipOGLFigure/icons/lightDir.png"), tr("Change illumination direction"), this);
    //m_pToggleIlluminationRotation = tempAction = new QAction(QIcon(":/twipOGLFigure/icons/lightDir.png"), tr("Change illumination direction"), this);
    tempAction->setCheckable(true);
    tempAction->setObjectName("Change illumination direction");
    tempAction->setToolTip(tr("Change illumination direction"));

    //m_actAspectSwitch
    m_pActTringModeSwitch = tempAction = new QAction(tr("Triangles"), this);
    m_pMnuTringModeSwitch = tempMenu   = new QMenu(tr("Mode Switch"));

    tempMenu->addAction(tr("Triangles"));
    tempMenu->actions().at(0)->setData(TwipOGLWidget::PAINT_TRIANG);
    tempMenu->addAction(tr("Points"));
    tempMenu->actions().at(1)->setData(TwipOGLWidget::PAINT_POINTS);

    tempAction->setMenu(m_pMnuTringModeSwitch);

    m_pToggleInfoText = tempAction = new QAction(QIcon(":/twipDesignerPlugins/general/icons/info.png"), tr("Show Infotext"), this);
    //m_pToggleInfoText = tempAction = new QAction(QIcon(":/twipDesignerPlugins/general/icons/info_or.png"), tr("Show Infotext"), this);
    tempAction->setCheckable(true);
    tempAction->setObjectName("Show Infotext");
    tempAction->setToolTip(tr("Show Infotext"));

    m_pActTwip = tempAction = new QAction(QIcon(":/twipDesignerPlugins/general/icons/twip.png"), tr("About"), this);
    tempAction->setCheckable(false);
    tempAction->setObjectName("Show About Dialog");
    tempAction->setToolTip(tr("Show About Dialog"));

    //m_pOverlaySlider
    m_pOverlaySlider = new QSlider(Qt::Horizontal, this);
    m_pOverlaySlider->setMinimum(0);
    m_pOverlaySlider->setMaximum(255);
    m_pOverlaySlider->setValue(0);
    m_pOverlaySlider->setWhatsThis(tr("Control alpha-value of overlay image"));
    m_pOverlaySlider->setToolTip(tr("Set alpha for overlay"));

    QWidgetAction *wa = new QWidgetAction(this);
    wa->setDefaultWidget(m_pOverlaySlider);
    m_pActOverlaySlider = wa;
    wa->setObjectName("overlaySlider");
    wa->setVisible(false);
    connect(m_pOverlaySlider, SIGNAL(valueChanged(int)), this, SLOT(mnuOverlaySliderChanged(int)));

    //m_actCmplxSwitch
    m_pActCmplxSwitch = tempAction = new QAction(QIcon(":/itomDesignerPlugins/complex/icons/ImRe.png"), tr("Switch Imag, Real, Abs, Pha"), this);
    m_pMnuCmplxSwitch = tempMenu   =  new QMenu(tr("Complex Switch"));
    tempMenu->addAction(tr("Imag"));
    tempMenu->addAction(tr("Real"));
    tempMenu->addAction(tr("Abs"));
    tempMenu->addAction(tr("Pha"));
    tempAction->setMenu(m_pMnuCmplxSwitch);
    tempAction->setVisible(false);

    //m_pActProperties
    m_pActProperties = this->getPropertyDockWidget()->toggleViewAction();
    m_pActProperties->setIcon(QIcon(":/itomDesignerPlugins/general/icons/settings.png"));
    connect(m_pActProperties, SIGNAL(triggered(bool)), this, SLOT(mnuShowProperties(bool)));

    //m_pActLegend
    m_pActLegend = m_pLegendDock->toggleViewAction();
    connect(m_pActLegend, SIGNAL(triggered(bool)), this, SLOT(mnuShowLegend(bool)));
    connect(m_pActLegendIcon, SIGNAL(triggered(bool)), this, SLOT(mnuShowLegend(bool)));

    //m_actSave
    connect(m_pActSave, SIGNAL(triggered()), this, SLOT(mnuActSave()));
    connect(m_pActHome, SIGNAL(triggered()), this, SLOT(mnuHome()));
    connect(m_pKeepAspectRatio, SIGNAL(triggered()), this, SLOT(mnuAspRatio()));

    connect(m_pActReduceZAmpl, SIGNAL(triggered()), this, SLOT(mnuRedZ()));
    connect(m_pActRiseZAmpl, SIGNAL(triggered()), this, SLOT(mnuRiseZ()));

    connect(m_pActPalette, SIGNAL(triggered()), this, SLOT(mnuPalette()));
    connect(m_pActToggleColorBar, SIGNAL(triggered()), this, SLOT(mnuColorBar()));
    connect(m_pActChangeBGColor, SIGNAL(triggered()), this, SLOT(mnuToggleBPColor()));
    connect(m_pToggleIllumination, SIGNAL(toggled(bool)), this, SLOT(mnutoggleIllumination(bool)));
    connect(m_pToggleIlluminationRotation, SIGNAL(toggled(bool)), this, SLOT(mnutoggleIlluminationRotation(bool)));

    connect(m_pToggleInfoText, SIGNAL(toggled(bool)), m_pContent, SLOT(toggleObjectInfoText(bool)));

    connect(m_pActTwip, SIGNAL(triggered()), this, SLOT(mnuTwip()));

    connect(m_pMnuCmplxSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuCmplxSwitch(QAction*)));
    connect(m_pMnuTringModeSwitch, SIGNAL(triggered(QAction*)), this, SLOT(mnuTringModeSwitch(QAction*)));

    QToolBar *toolbar = new QToolBar(tr("Iso Toolbar"), this);
    addToolBar(toolbar, "mainToolBar", Qt::TopToolBarArea, 1);

    contextMenu->addAction(m_pActSave);
    contextMenu->addSeparator();
    contextMenu->addAction(m_pActHome);
    contextMenu->addAction(m_pKeepAspectRatio);
    contextMenu->addAction(m_pActScaleSetting);
    contextMenu->addAction(m_pActReduceZAmpl);
    contextMenu->addAction(m_pActRiseZAmpl);
    contextMenu->addAction(m_pToggleInfoText);

    contextMenu->addSeparator();
    contextMenu->addAction(m_pActProperties);
    contextMenu->addAction(toolbar->toggleViewAction());

    // next block is colorbar
    toolbar->addAction(m_pActSave);
    toolbar->addSeparator();
    toolbar->addAction(m_pActHome);
    toolbar->addAction(m_pActProperties);
    toolbar->addAction(m_pKeepAspectRatio);
    toolbar->addAction(m_pActReduceZAmpl);
    toolbar->addAction(m_pActRiseZAmpl);
    toolbar->addSeparator();
    toolbar->addAction(m_pActPalette);
    toolbar->addAction(m_pActToggleColorBar);
    toolbar->addAction(m_pActLegendIcon);
    toolbar->addAction(m_pActChangeBGColor);
    toolbar->addAction(m_pActTringModeSwitch);
    toolbar->addAction(m_pActOverlaySlider);

    toolbar->addSeparator();
    toolbar->addAction(m_pToggleIllumination);
    toolbar->addAction(m_pToggleIlluminationRotation);

    toolbar->addSeparator();
    toolbar->addAction(m_pToggleInfoText);
    toolbar->addAction(m_pActTwip);

    // next block is for complex and stacks
    toolbar->addSeparator();
    toolbar->addAction(m_pActCmplxSwitch);

    this->setWindowIcon(QIcon("/twipDesignerPlugins/general/icons/twip.png"));

#if USEWIDGETEVENTS == 0
    m_pEventFilter = NULL;
    m_pEventFilter = new OGLEventFilter(this);
    installEventFilter(m_pEventFilter);
#endif

    NTHREADS  = QThread::idealThreadCount();

    setPropertyObservedObject(this);

    contextMenu->addAction(m_pActProperties);
    contextMenu->addAction(m_pActLegend);

}

//----------------------------------------------------------------------------------------------------------------------------------
TwipOGLFigure::~TwipOGLFigure()
{


    if(m_pActScaleSetting != NULL)
    {
        m_pActScaleSetting->deleteLater();
        m_pActScaleSetting = NULL;
    }

    if(m_pActPalette != NULL)
    {
        m_pActPalette->deleteLater();
        m_pActPalette = NULL;
    }

    if(m_pActToggleColorBar != NULL)
    {
        m_pActToggleColorBar->deleteLater();
        m_pActToggleColorBar = NULL;
    }

    if(m_pActChangeBGColor != NULL)
    {
        m_pActChangeBGColor->deleteLater();
        m_pActChangeBGColor = NULL;
    }

    if(m_pToggleIllumination != NULL)
    {
        m_pToggleIllumination->deleteLater();
        m_pToggleIllumination = NULL;
    }

    if(m_pToggleIlluminationRotation != NULL)
    {
        m_pToggleIlluminationRotation->deleteLater();
        m_pToggleIlluminationRotation = NULL;
    }

    if(m_pMnuCmplxSwitch != NULL)
    {
        m_pMnuCmplxSwitch->deleteLater();
        m_pMnuCmplxSwitch = NULL;
    }

    if(m_pActCmplxSwitch != NULL)
    {
        m_pActCmplxSwitch->deleteLater();
        m_pActCmplxSwitch = NULL;
    }

    if(m_pMnuTringModeSwitch != NULL)
    {
        m_pMnuTringModeSwitch->deleteLater();
        m_pMnuTringModeSwitch = NULL;
    }

    if(m_pActTringModeSwitch != NULL)
    {
        m_pActTringModeSwitch->deleteLater();
        m_pActTringModeSwitch = NULL;
    }

    if(m_pToggleInfoText != NULL)
    {
        m_pToggleInfoText->deleteLater();
        m_pToggleInfoText = NULL;
    }

    if(m_pLblCoordinates != NULL)
    {
        m_pLblCoordinates->deleteLater();
        m_pLblCoordinates = NULL;
    }

    if(m_pvConfigData)
    {
        // we will delete this within the GL-Widget!!!!
        m_pvConfigData = NULL;

    }

    if(m_pContent != NULL)
    {
        m_pContent->deleteLater();
        m_pContent = NULL;
    }

#if !USEWIDGETEVENTS
    if (m_pEventFilter != NULL)
    {
        delete m_pEventFilter;
        m_pEventFilter = NULL;
    }
#endif
    if (m_pOverlaySlider)
    {
        delete m_pOverlaySlider;
        m_pOverlaySlider = NULL;
    }

    if (m_pLegendDock)
    {
        m_pLegendDock->deleteLater();
    }

    if (m_pLegend)
    {
        m_pLegend->deleteLater();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal TwipOGLFigure::applyUpdate()
{
    if (m_inpType == ito::ParamBase::DObjPtr)
    {
        ((TwipOGLWidget*)m_pContent)->refreshPlot(getInputParam("dataObject"));
    }
    else if (m_inpType == ito::ParamBase::PointCloudPtr)
    {
        ((TwipOGLWidget*)m_pContent)->refreshPlot(getInputParam("pointCloud"));
    }
    else if (m_inpType == ito::ParamBase::PolygonMeshPtr)
    {
        ((TwipOGLWidget*)m_pContent)->refreshPlot(getInputParam("polygonMesh"));
    }
    else
        return ito::retError;

    ((TwipOGLWidget*)m_pContent)->refreshPlot(NULL);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> TwipOGLFigure::getDisplayed(void)
{
    return QSharedPointer<ito::DataObject>(new ito::DataObject(*getOutputParam("displayed")->getVal<const ito::DataObject*>()));
}

//----------------------------------------------------------------------------------------------------------------------------------
QSharedPointer<ito::DataObject> TwipOGLFigure::getSource(void) const
{
    return QSharedPointer<ito::DataObject>(new ito::DataObject(*getInputParam("source")->getVal<const ito::DataObject*>()));
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setContextMenuEnabled(bool show)
{
    if (m_pContent)
    {
        ((TwipOGLWidget*)m_pContent)->m_showContextMenu = show;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool TwipOGLFigure::getContextMenuEnabled() const
{
    if (m_pContent)
    {
        return ((TwipOGLWidget*)m_pContent)->m_showContextMenu;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal TwipOGLFigure::displayLineCut(QVector<QPointF> bounds, ito::uint32 &uniqueID)
{
    return ito::retError;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuTwip()
{
    DialogAboutTwip *dlgAbout = new DialogAboutTwip();
    dlgAbout->exec();
    if(dlgAbout) dlgAbout->deleteLater();
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuHome()
{
    if (m_pContent)
    {
        ((TwipOGLWidget*)m_pContent)->homeView();
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuAspRatio()
{
    if (m_pContent)
    {
        this->setkeepAspectRatio(!this->getkeepAspectRatio());
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuRedZ()
{
    if (m_pContent && m_pvConfigData)
    {
        ((InternalData*)m_pvConfigData)->m_zAmpl *= 0.95;
        ((TwipOGLWidget*)m_pContent)->validateZAmplification();
        ((TwipOGLWidget*)m_pContent)->update();
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuRiseZ()
{
    if (m_pContent)
    {
        ((InternalData*)m_pvConfigData)->m_zAmpl *= 1.05;
        ((TwipOGLWidget*)m_pContent)->validateZAmplification();
        ((TwipOGLWidget*)m_pContent)->update();
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuPalette()
{
    if(m_pvConfigData)((InternalData*)m_pvConfigData)->m_paletteNum++;
    if (m_pContent)
    {
        ((TwipOGLWidget*)m_pContent)->setColorMap("");
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuActSave()
{
#ifndef QT_NO_PRINTER
    QString fileName = "iso3D";
#else
    QString fileName = "iso3D.";
#endif

#ifndef QT_NO_FILEDIALOG
    const QList<QByteArray> imageFormats =
        QImageWriter::supportedImageFormats();

    QStringList filter;
    QString selfilter;
//    filter += tr("Postscript Documents (*.ps)");

    if (imageFormats.size() > 0)
    {

        for (int i = 0; i < imageFormats.size(); i++)
        {
            if (QString(imageFormats[i]).contains("jpg", Qt::CaseInsensitive))
            {
                QString imageFilter(tr("JPG Image ("));
                //if (i > 0)
                //    imageFilter += " ";
                imageFilter += "*.";
                imageFilter += imageFormats[i];
                imageFilter += ")";
                filter += imageFilter;
                selfilter = imageFilter;
            }
            else if (QString(imageFormats[i]).contains("bmp", Qt::CaseInsensitive))
            {
                QString imageFilter(tr("Bitmap ("));
                //if (i > 0)
                //    imageFilter += " ";
                imageFilter += "*.";
                imageFilter += imageFormats[i];
                imageFilter += ")";
                filter += imageFilter;
            }
            else if (QString(imageFormats[i]).contains("png", Qt::CaseInsensitive))
            {
                QString imageFilter(tr("Portable Network Graphics ("));
                //if (i > 0)
                //    imageFilter += " ";
                imageFilter += "*.";
                imageFilter += imageFormats[i];
                imageFilter += ")";
                filter += imageFilter;
            }
            else if (QString(imageFormats[i]).contains("tif", Qt::CaseInsensitive))
            {
                QString imageFilter(tr("Tagged Image File ("));
                //if (i > 0)
                //    imageFilter += " ";
                imageFilter += "*.";
                imageFilter += imageFormats[i];
                imageFilter += ")";
                filter += imageFilter;
            }

        }
        //imageFilter += ")";

        //filter += imageFilter;
    }

    //    filter += tr("PDF Documents (*.pdf)");
#ifndef GV_NO_SVG
#ifdef QT_SVG_LIB
    filter += tr("SVG Documents (*.svg)");
#endif
#endif

    fileName = QFileDialog::getSaveFileName(
        this, tr("Export File Name"), fileName,
        filter.join(";;"), &selfilter);
#endif


    if (!fileName.isEmpty())
    {

        //QImage img = this->m_pContent->grabFrameBuffer();

        QImage img;

        m_pContent->getFrameBuffer(img, 2);

        img.save(fileName, QFileInfo(fileName).completeSuffix().toLatin1().data(), 100);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal TwipOGLFigure::copyToClipBoard()
{
    if(!m_pContent) return ito::RetVal(ito::retError, 0, tr("Could not save data.").toLatin1().data());

    m_pContent->update();

    QImage img;

    m_pContent->getFrameBuffer(img, 2);
    //m_pContent->renderPixmap(512, 512, false);


    m_pContent->update();
    QClipboard *clipboard = QApplication::clipboard();
    clipboard->setImage(img);
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
QPixmap TwipOGLFigure::renderToPixMap(const int xsize, const int ysize, const int resolution)
{
    m_pContent->repaint();
    int resFaktor = cv::saturate_cast<int>(resolution / 72.0 + 0.5);
    resFaktor = resFaktor < 1 ? 1 : resFaktor;
    resFaktor = resFaktor > 2 ? 2 : resFaktor;
    QPixmap destinationImage(xsize , ysize );
    if(!m_pContent)
    {
        destinationImage.fill(Qt::red);
        return destinationImage;
    }
    //destinationImage.fill(Qt::white);

    QImage img;

    m_pContent->getFrameBuffer(img, resFaktor);

    destinationImage.convertFromImage(img);

    //Would be nice but does not work and destroys the gl-shader!
    //destinationImage = m_pContent->renderPixmap( xsize * resFaktor, ysize * resFaktor, true);

    return destinationImage;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuScaleSetting()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuColorBar()
{
    if (m_pContent)
    {
        ((TwipOGLWidget*)m_pContent)->togglePaletteMode();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnutoggleIllumination(const bool checked)
{
    if (m_pvConfigData)
    {
        if(((InternalData*)m_pvConfigData)->m_elementMode & TwipOGLWidget::ENABLE_ILLUMINATION)
            ((InternalData*)m_pvConfigData)->m_elementMode &= ~ TwipOGLWidget::ENABLE_ILLUMINATION;
        else
            ((InternalData*)m_pvConfigData)->m_elementMode |= TwipOGLWidget::ENABLE_ILLUMINATION;
        if (m_pContent)
            ((TwipOGLWidget*)m_pContent)->update();
    }
    m_pToggleIlluminationRotation->setEnabled(checked);
    if (!checked)
    {
        m_pToggleIllumination->setIcon(QIcon(":/twipOGLFigure/icons/light.png"));
        m_pToggleIlluminationRotation->setChecked(false);
    }
    else
    {
        m_pToggleIllumination->setIcon(QIcon(":/twipOGLFigure/icons/lightOn.png"));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnutoggleIlluminationRotation(const bool checked)
{
    if(m_pvConfigData)
    {
        ((InternalData*)m_pvConfigData)->m_drawLightDir = checked;
    }
    if (!checked)
    {
        m_pToggleIlluminationRotation->setIcon(QIcon(":/twipOGLFigure/icons/lightDir.png"));
    }
    else
    {
        m_pToggleIlluminationRotation->setIcon(QIcon(":/twipOGLFigure/icons/lightDirOn.png"));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuCmplxSwitch(QAction *action)
{
    if(m_pvConfigData)
    {
        if (action->text() == QString("Imag"))
        {
            ((InternalData*)m_pvConfigData)->m_cmplxMode = TwipOGLWidget::tImag;
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReImag.png"));
        }
        else if (action->text() == QString("Real"))
        {
            ((InternalData*)m_pvConfigData)->m_cmplxMode = TwipOGLWidget::tReal;
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReReal.png"));
        }
        else if (action->text() == QString("Pha"))
        {
            ((InternalData*)m_pvConfigData)->m_cmplxMode = TwipOGLWidget::tPhase;
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImRePhase.png"));
        }
        else
        {
            ((InternalData*)m_pvConfigData)->m_cmplxMode = TwipOGLWidget::tAbs;
            m_pActCmplxSwitch->setIcon(QIcon(":/itomDesignerPlugins/complex/icons/ImReAbs.png"));
        }
    }
    if (m_pContent)
    {
        ((TwipOGLWidget*)m_pContent)->refreshPlot(NULL);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuTringModeSwitch(QAction *action)
{
    switch(action->data().toInt())
    {
        default:
        case TwipOGLWidget::PAINT_TRIANG:
            setShowTriangles(true);
        break;
        case TwipOGLWidget::PAINT_POINTS:
            setShowTriangles(false);
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//QPointF TwipOGLFigure::getZAxisInterval(void) const
ito::AutoInterval TwipOGLFigure::getZAxisInterval(void) const
{
//    return QPointF(0.0, 1.0);
    return ito::AutoInterval(0.0, 1.0);
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::resetZAmplification(void)
{
    ((InternalData*)m_pvConfigData)->m_zAmpl = 1.00;
    ((TwipOGLWidget*)m_pContent)->validateZAmplification();
    ((TwipOGLWidget*)m_pContent)->update();
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setZAmplification(const double value)
{
    ((InternalData*)m_pvConfigData)->m_zAmpl = cv::saturate_cast<float>(value);
    ((TwipOGLWidget*)m_pContent)->validateZAmplification();
    ((TwipOGLWidget*)m_pContent)->update();
}

//----------------------------------------------------------------------------------------------------------------------------------
double TwipOGLFigure::getZAmplification(void) const
{
    return ((InternalData*)m_pvConfigData)->m_zAmpl;
}

//----------------------------------------------------------------------------------------------------------------------------------
//void TwipOGLFigure::setZAxisInterval(QPointF interval)
void TwipOGLFigure::setZAxisInterval(ito::AutoInterval interval)
{
    if (m_pContent)
    {
//        ((TwipOGLWidget*)m_pContent)->setInterval(Qt::ZAxis, 0, interval.x(), interval.y());
        ((TwipOGLWidget*)m_pContent)->setInterval(Qt::ZAxis, 0, interval.minimum(), interval.maximum());
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
QString TwipOGLFigure::getColorMap(void) const
{
    return QString();
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setColorMap(QString palette)
{
    if (m_pContent)
    {
        ((TwipOGLWidget*)m_pContent)->setColorMap(palette);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::enableIlluGUI(const bool checked)
{
    if (!checked)
    {
        m_pToggleIllumination->setChecked(false);
    }
    if (!checked)
    {
        m_pToggleIlluminationRotation->setChecked(false);
    }
    m_pToggleIllumination->setEnabled(checked);
    m_pToggleIlluminationRotation->setEnabled(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::enableComplexGUI(const bool checked)
{
    m_pActCmplxSwitch->setEnabled(checked);
    m_pActCmplxSwitch->setVisible(checked);
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::enableZStackGUI(const bool checked)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setLinePlotCoordinates(const QVector<QPointF> pts)
{
    char buf[60] = {0};
    if (pts.size() > 1)
    {
        sprintf(buf, "[%.6g; %.6g]\n[%.6g; %.6g]", pts[0].x(), pts[0].y(), pts[1].x(), pts[1].y());
    }
    else if (pts.size() == 1)
    {
        sprintf(buf, "[%.6g; %.6g]\n[ - ; - ]", pts[0].x(), pts[0].y());
    }
    else
    {
        sprintf(buf, "[ - ; - ]\n[ - ; - ]");
    }
    m_pLblCoordinates->setText(buf);
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::triggerReplot()
{
    if (m_pContent != NULL)
    {
        ((TwipOGLWidget*)m_pContent)->m_forceReplot = true;
        ((TwipOGLWidget*)m_pContent)->refreshPlot(NULL);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuToggleBPColor()
{
    if(m_pvConfigData)
    {
        InternalData* intData = ((InternalData*)m_pvConfigData);

        if (intData->m_backgnd == 0 || intData->m_backgnd == 0xFF000000)
        {
            intData->m_backgnd   = 0x00FFFFFF;
            intData->m_axisColor = 0x00000000;
            intData->m_textColor = 0x00000000;
        }
        else
        {
            intData->m_backgnd   = 0x00000000;
            intData->m_axisColor = 0x00FFFFFF;
            intData->m_textColor = 0x00FFFFFF;
        }
    }
    if(m_pContent) m_pContent->updateColorsAndVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor TwipOGLFigure::getBackgroundColor(void) const
{
    if(m_pvConfigData)
    {
        return ((InternalData*)m_pvConfigData)->m_backgnd;
    }
    else
        return Qt::white;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setBackgroundColor(const QColor newVal)
{
    if(m_pvConfigData && newVal.isValid())
    {
        InternalData* intData = ((InternalData*)m_pvConfigData);
        intData->m_backgnd = newVal.rgb() & 0x00FFFFFF;
    }
    if(m_pContent) m_pContent->updateColorsAndVisibility();

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor TwipOGLFigure::getAxisColor(void) const
{
    if(m_pvConfigData)
    {
        return ((InternalData*)m_pvConfigData)->m_axisColor;
    }
    else
        return Qt::black;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setAxisColor(const QColor newVal)
{
    if(m_pvConfigData && newVal.isValid())
    {
        InternalData* intData = ((InternalData*)m_pvConfigData);
        intData->m_axisColor = newVal.rgb() & 0x00FFFFFF;
    }
    if(m_pContent) m_pContent->updateColorsAndVisibility();

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor TwipOGLFigure::getTextColor(void) const
{
    if(m_pvConfigData)
    {
        return ((InternalData*)m_pvConfigData)->m_textColor;
    }
    else
        return Qt::black;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setTextColor(const QColor newVal)
{
    if(m_pvConfigData && newVal.isValid())
    {
        InternalData* intData = ((InternalData*)m_pvConfigData);
        intData->m_textColor = newVal.rgb() & 0x00FFFFFF;
    }
    if(m_pContent) m_pContent->updateColorsAndVisibility();

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
QColor TwipOGLFigure::getInvalidColor(void) const
{
    if(m_pvConfigData)
    {
        return ((InternalData*)m_pvConfigData)->m_invColor;
    }
    else
        return Qt::black;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setInvalidColor(const QColor newVal)
{
    if(m_pvConfigData && newVal.isValid())
    {
        InternalData* intData = ((InternalData*)m_pvConfigData);
        intData->m_invColor = newVal.rgba();
        if(m_pContent) m_pContent->ResetColors();
    }


    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::resetInvalidColor()
{
    if(m_pvConfigData)
    {
        InternalData* intData = ((InternalData*)m_pvConfigData);
        intData->m_invColor = 0x000F0F0F;
    }
    if(m_pContent) m_pContent->ResetColors();

    return;
}

#if USEWIDGETEVENTS == 0

//----------------------------------------------------------------------------------------------------------------------------------
bool OGLEventFilter::event(QEvent *e)
{
    if (e->type() == QEvent::User)
    {
        return true;
    }
    return QObject::event(e);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool OGLEventFilter::eventFilter(QObject *object, QEvent *e)
{
    if (e->type() == QEvent::Destroy)
        return false;

    if (m_plotObj == NULL || ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent) == NULL)
        return false;

    qDebug("The event type is %i", e->type());

    switch(e->type())
    {
        case QEvent::Wheel:
        {
            if (((const QWheelEvent *)e)->delta() > 0)
            {
                m_plotObj->riseZAmplification(1.05);
            }
            else
            {
                m_plotObj->reduceZAmplification(0.95);
            }
            break;
        }

        case QEvent::MouseButtonPress:
        {
            rotating = true;

            startPos[0] = QCursor::pos().x();
            startPos[1] = QCursor::pos().y();

            break;
        }

        case QEvent::MouseButtonRelease:
        case QEvent::MouseMove:
        {
            if (!rotating)
                break;

            double dyaw = (QCursor::pos().x() - startPos[0]) / 200.0;
            double dpitch = (QCursor::pos().y() - startPos[1]) / 200.0;
            double droll = 0;

            startPos[0] = QCursor::pos().x();
            startPos[1] = QCursor::pos().y();

            if (((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->lightArrowEnabled())
            {
                ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->rotateLightArrow(dpitch, droll, dyaw);
            }
            else
            {
                ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->rotateView(dpitch, droll, dyaw);
            }

            if (e->type() == QEvent::MouseButtonRelease)
            {
                rotating = true;
                moving = false;
            }

            ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->repaint();
            break;
        }

        case QEvent::KeyRelease:
        case QEvent::KeyPress:
        {

            if(((const QKeyEvent *)e)->matches(QKeySequence::Copy))
            {
                ((TwipOGLFigure*)m_plotObj)->copyToClipBoard();
            }
            else
            {
                switch(((const QKeyEvent *)e)->key())
                {
                    case Qt::Key_H:
                    {
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->setView(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->repaint();
                    }
                    break;

                    case Qt::Key_W:
                    case Qt::Key_Up:
                    {
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->rotateView(-0.05, 0.0, 0.0);
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->repaint();
                    }
                    break;

                    case Qt::Key_S:
                    case Qt::Key_Down:
                    {
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->rotateView(0.05, 0.0, 0.0);
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->repaint();
                    }
                    break;

                    case Qt::Key_D:
                    case Qt::Key_Right:
                    {
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->rotateView(0.0, 0.0, 0.05);
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->repaint();
                    }
                    break;

                    case Qt::Key_A:
                    case Qt::Key_Left:
                    {
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->rotateView(0.0, 0.0, -0.05);
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->repaint();
                    }
                    break;

                    case Qt::Key_Q:
                    {
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->rotateView(0.0, -0.05, 0.0);
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->repaint();
                    }
                    break;

                    case Qt::Key_E:
                    {
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->rotateView(0.0,  0.05, 0.0);
                        ((TwipOGLWidget*)((TwipOGLFigure*)m_plotObj)->m_pContent)->repaint();
                    }
                    break;

                    // The following keys represent a direction, they are
                    // organized on the keyboard.

                    case Qt::Key_V:
                    {

                    }
                    break;

                    default:
                    break;
                }
            }
        }
        default:
            break;
    }

    return QObject::eventFilter(object, e);
}
#endif

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::mnuOverlaySliderChanged(int value)
{
    InternalData *intData = (InternalData *)m_pvConfigData;
    if(m_pvConfigData)
    {
        intData->m_alpha = value / 255.0f;
        intData->m_alpha = intData->m_alpha < 0.0f ? 0.0f : intData->m_alpha > 1.0f ? 1.0f : intData->m_alpha;
    }

    if(m_pContent)
    {
        m_pContent->updateAlpha();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
int TwipOGLFigure::getOverlayAlpha () const
{
    if (m_pvConfigData)
        return (int)(((InternalData *)m_pvConfigData)->m_alpha * 255.0f);
    else
        return 0.0;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setOverlayAlpha (const int alpha)
{
    if(m_pOverlaySlider)
    {
        m_pOverlaySlider->setValue(alpha);
    }
    else
    {
        mnuOverlaySliderChanged(alpha);
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::resetOverlayAlpha(void)
{
    if(m_pOverlaySlider)
    {
        m_pOverlaySlider->setValue(0);
    }
    else
    {
        mnuOverlaySliderChanged(0);
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool TwipOGLFigure::getColorBarVisible() const
{
    if (!m_pvConfigData)
        return false;

    InternalData *intData = (InternalData *)m_pvConfigData;
    return (intData->m_colorBarMode != TwipOGLWidget::COLORBAR_NO);
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setColorBarVisible(const bool value)
{
    if (!m_pvConfigData)
        return;

    InternalData *intData = (InternalData *)m_pvConfigData;
    if (value && intData->m_colorBarMode == TwipOGLWidget::COLORBAR_NO)
        intData->m_colorBarMode = TwipOGLWidget::COLORBAR_LEFT;
    else if (!value)
        intData->m_colorBarMode = TwipOGLWidget::COLORBAR_NO;

    if(m_pContent) m_pContent->update();
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::AutoInterval TwipOGLFigure::getOverlayInterval() const
{
    if (!m_pvConfigData)
        return ito::AutoInterval(0, 0, true);

    return ((InternalData *)m_pvConfigData)->m_overlayInterval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//void TwipOGLFigure::setOverlayInterval(const QPointF interval)
void TwipOGLFigure::setOverlayInterval(const ito::AutoInterval interval)
{
    if (!m_pvConfigData)
        return;

    ((InternalData *)m_pvConfigData)->m_overlayInterval = interval;
    if(m_pContent)
        m_pContent->update();

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
//QPointF TwipOGLFigure::getCurvatureInterval() const
ito::AutoInterval TwipOGLFigure::getCurvatureInterval() const
{
    if (!m_pvConfigData)
        //return QPointF(0.0, 0.0);
        return ito::AutoInterval(0, 0, true);

    return ((InternalData *)m_pvConfigData)->m_curvatureInterval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//void TwipOGLFigure::setCurvatureInterval(const QPointF interval)
void TwipOGLFigure::setCurvatureInterval(const ito::AutoInterval interval)
{
    if (!m_pvConfigData)
        return;

    ((InternalData *)m_pvConfigData)->m_curvatureInterval = interval;

    if(m_pContent)
        m_pContent->updateCurvature();

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setOverlayImage(QSharedPointer< ito::DataObject > overlayImage)
{
    if(m_pContent)
        m_pContent->setOverlayImage(overlayImage, 0);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::resetOverlayImage(void)
{
    if(m_pContent)
        m_pContent->setOverlayImage(QSharedPointer<ito::DataObject>(NULL), 0);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setInvalidMap(QSharedPointer< ito::DataObject > invalidMap)
{
    if(m_pContent)
        m_pContent->setInvalidImage(invalidMap, 0);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::resetInvalidMap(void)
{
    if(m_pContent)
        m_pContent->setInvalidImage(QSharedPointer<ito::DataObject>(NULL), 0);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont TwipOGLFigure::getAxisFont() const
{
    return m_pContent->m_axes.getQFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setAxisFont(const QFont value)
{
    if(m_pContent) m_pContent->m_axes.fromQFont(value);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
QFont TwipOGLFigure::getTitleFont() const
{
    if(m_pContent) return m_pContent->m_titleFont.getQFont();
    else return QFont();
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setTitleFont(const QFont value)
{
    m_pContent->m_titleFont.fromQFont(value);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setIllumunationEnabled(const bool newVal)
{
    m_pToggleIllumination->setChecked(newVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool TwipOGLFigure::getShowTriangles(void) const
{
    if (!m_pvConfigData)
        return false;

    return ((InternalData *)m_pvConfigData)->m_elementMode & TwipOGLWidget::PAINT_TRIANG;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setShowTriangles(const bool newVal)
{
    if(m_pvConfigData)
    {
        if(newVal)
        {
            m_pMnuTringModeSwitch->setTitle("Triangles");
            ((InternalData*)m_pvConfigData)->m_elementMode |= TwipOGLWidget::PAINT_TRIANG;
            ((InternalData*)m_pvConfigData)->m_elementMode &= ~TwipOGLWidget::PAINT_POINTS;
        }
        else
        {
            m_pMnuTringModeSwitch->setTitle("Points");
            ((InternalData*)m_pvConfigData)->m_elementMode &=~ TwipOGLWidget::PAINT_TRIANG;
            ((InternalData*)m_pvConfigData)->m_elementMode |= TwipOGLWidget::PAINT_POINTS;
        }
    }
    if (m_pContent)
    {
        m_pContent->updateVisMode();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool TwipOGLFigure::getCurvature2Color(void) const
{
    if(m_pvConfigData)
    {
        return ((InternalData*)m_pvConfigData)->m_showCurvature;
    }

    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setCurvature2Color(const bool newVal)
{
    if(m_pvConfigData)
    {
        ((InternalData*)m_pvConfigData)->m_showCurvature = newVal;
    }
    if (m_pContent)
    {
        m_pContent->update();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::enableRenderModeSelector(const int newVal)
{
    if(newVal & TwipOGLWidget::ENABLE_ILLUMINATION)
    {
        m_pToggleIllumination->setEnabled(true);
        m_pToggleIlluminationRotation->setEnabled(true);
        m_pToggleIlluminationRotation->setChecked(false);
    }
    else
    {
        m_pToggleIllumination->setChecked(false);
        m_pToggleIllumination->setEnabled(false);
        m_pToggleIlluminationRotation->setEnabled(false);
        m_pToggleIlluminationRotation->setChecked(false);
    }

    if(newVal & TwipOGLWidget::PAINT_POINTS && newVal & TwipOGLWidget::PAINT_TRIANG)
    {
        m_pMnuTringModeSwitch->setEnabled(true);
        m_pMnuTringModeSwitch->setTitle("Points");
    }
    else if(newVal & TwipOGLWidget::PAINT_POINTS)
    {
        m_pMnuTringModeSwitch->setEnabled(false);
        m_pMnuTringModeSwitch->setTitle("Points");
    }
    else
    {
        m_pMnuTringModeSwitch->setEnabled(true);
        m_pMnuTringModeSwitch->setTitle("Triangles");
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool TwipOGLFigure::getIllumunationEnabled(void) const
{
    if (!m_pvConfigData)
        return false;

    return ((InternalData *)m_pvConfigData)->m_elementMode & TwipOGLWidget::ENABLE_ILLUMINATION;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool TwipOGLFigure::getxAxisVisible() const
{
    if (!m_pvConfigData)
        return false;

    return ((InternalData *)m_pvConfigData)->m_xAxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setxAxisVisible(const bool value)
{
    if (!m_pvConfigData)
        return;
    ((InternalData *)m_pvConfigData)->m_xAxisVisible = value;

    if(m_pContent) m_pContent->updateColorsAndVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool TwipOGLFigure::getyAxisVisible() const
{
    if (!m_pvConfigData)
        return false;

    return ((InternalData *)m_pvConfigData)->m_yAxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setyAxisVisible(const bool value)
{
    if (!m_pvConfigData)
        return;
    ((InternalData *)m_pvConfigData)->m_yAxisVisible = value;

    if(m_pContent) m_pContent->updateColorsAndVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool TwipOGLFigure::getVAxisVisible() const
{
    if (!m_pvConfigData)
        return false;

    return ((InternalData *)m_pvConfigData)->m_vAxisVisible;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setVAxisVisible(const bool value)
{
    if (!m_pvConfigData)
        return;
    ((InternalData *)m_pvConfigData)->m_vAxisVisible = value;

    if(m_pContent) m_pContent->updateColorsAndVisibility();
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setyAxisFlipped(const bool value)
{
    m_pContent->m_axes.m_axisY.m_isflipped = value;
    if(m_pContent) m_pContent->update();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool TwipOGLFigure::getkeepAspectRatio() const
{
    if (!m_pvConfigData)
        return false;
    return ((InternalData *)m_pvConfigData)->m_keepVoxel;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setkeepAspectRatio(const bool value)
{
    if (!m_pvConfigData)
        return;
    ((InternalData *)m_pvConfigData)->m_keepVoxel = value;
    if(m_pContent) m_pContent->update();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool TwipOGLFigure::getShowAngleInfo(void) const
{
    if (!m_pvConfigData)
        return false;
    return ((InternalData *)m_pvConfigData)->m_plotAngles;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setShowAngleInfo(const bool newVal)
{
    if (!m_pvConfigData)
        return;
    ((InternalData *)m_pvConfigData)->m_plotAngles = newVal;
    if(m_pContent) m_pContent->update();
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setRotationAngle(const QVector<float> newAngles)
{
    if(newAngles.size() < 3)
        return;
    if (!m_pvConfigData)
        return;

    ((InternalData *)m_pvConfigData)->m_rollAng = newAngles[0] / GL_RAD_GRAD;
    TwipOGLWidget::normalizeAngle(((InternalData *)m_pvConfigData)->m_rollAng);

    ((InternalData *)m_pvConfigData)->m_yawAng = newAngles[1] / GL_RAD_GRAD;
    TwipOGLWidget::normalizeAngle(((InternalData *)m_pvConfigData)->m_yawAng);

    ((InternalData *)m_pvConfigData)->m_pitchAng = newAngles[2] / GL_RAD_GRAD;
    TwipOGLWidget::normalizeAngle(((InternalData *)m_pvConfigData)->m_pitchAng);

    if(m_pContent) m_pContent->update();
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<float> TwipOGLFigure::getRotationAngle() const
{
    QVector<float> res(3);
    res[0] = ROLL0;
    res[1] = YAW0;
    res[2] = PITCH0;
    if (m_pvConfigData)
    {
        res[0] = ((InternalData *)m_pvConfigData)->m_rollAng * GL_RAD_GRAD;
        res[1] = ((InternalData *)m_pvConfigData)->m_yawAng * GL_RAD_GRAD;
        res[2] = ((InternalData *)m_pvConfigData)->m_pitchAng * GL_RAD_GRAD;
    }
    return res;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setRotationLightAngle(const QVector<float> newAngles)
{
    if(newAngles.size() < 2)
        return;
    if (!m_pvConfigData)
        return;

    ((InternalData *)m_pvConfigData)->m_lightDirPitch = newAngles[0] / GL_RAD_GRAD;
    TwipOGLWidget::normalizeAngle(((InternalData *)m_pvConfigData)->m_lightDirPitch);

    ((InternalData *)m_pvConfigData)->m_lightDirYaw = newAngles[1] / GL_RAD_GRAD;
    TwipOGLWidget::normalizeAngle(((InternalData *)m_pvConfigData)->m_lightDirYaw);

    if(m_pContent) m_pContent->update();
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<float> TwipOGLFigure::getRotationLightAngle() const
{
    QVector<float> res(2);
    res[0] = 0.0f;
    res[1] = 0.0f;
    if (m_pvConfigData)
    {
        res[0] = ((InternalData *)m_pvConfigData)->m_lightDirPitch * GL_RAD_GRAD;
        res[1] = ((InternalData *)m_pvConfigData)->m_lightDirYaw * GL_RAD_GRAD;
    }
    return res;
}

//----------------------------------------------------------------------------------------------------------------------------------
#ifdef USEPCL
int TwipOGLFigure::addPointCloud(ito::PCLPointCloud pointCloud)
{
    if(m_pContent)
    {
        m_pclID++;
        ito::ParamBase param("pcl", ito::Param::PointCloudPtr, (const char*)&pointCloud);
        m_pContent->refreshPlot(&param, m_pclID);
        m_pContent->refreshPlot(NULL, m_pclID);
    }

    return m_pclID;
}
#endif

//----------------------------------------------------------------------------------------------------------------------------------
int TwipOGLFigure::addDataObject(ito::DataObject dataObject)
{
    if (m_pContent)
    {
        // As we use the first dataObject in various places for comparison make sure it is filled
        if (m_pContent->m_pContentDObj[0] != NULL)
            m_dObjID++;
        ito::ParamBase param("dObj", ito::Param::DObjPtr, (const char*)&dataObject);
        m_pContent->refreshPlot(&param, m_dObjID);
        m_pContent->refreshPlot(NULL, m_dObjID);
    }

    return m_dObjID;
}

//----------------------------------------------------------------------------------------------------------------------------------
QString TwipOGLFigure::getTitle() const
{
    if (!m_pvConfigData || ((InternalData*) m_pvConfigData)->m_autoTitle)
    {
        return "<auto>";
    }
    return ((InternalData*) m_pvConfigData)->m_title;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setTitle(const QString &title)
{
    if(m_pvConfigData == NULL)
        return;
    if (title == "<auto>")
    {
        ((InternalData*) m_pvConfigData)->m_autoTitle = true;
    }
    else
    {
        ((InternalData*) m_pvConfigData)->m_autoTitle = false;
        ((InternalData*) m_pvConfigData)->m_title = title;
        if (title.length())
            ((InternalData*) m_pvConfigData)->m_drawTitle = true;
        else
            ((InternalData*) m_pvConfigData)->m_drawTitle = false;
    }

    if (m_pContent) m_pContent->update();
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::resetTitle()
{
    if(m_pvConfigData == NULL) return;
    ((InternalData*) m_pvConfigData)->m_autoTitle = true;
    if (m_pContent) m_pContent->update();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool TwipOGLFigure::getFillInvalidsStatus() const
{
    if(m_pvConfigData == NULL) return false;
    return ((InternalData*) m_pvConfigData)->m_showInvalids;
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::setFillInvalidsStatus(const bool enable)
{
    if(m_pvConfigData == NULL) return;
    ((InternalData*) m_pvConfigData)->m_showInvalids = enable;
    if (m_pContent)
    {
        m_pContent->m_forceReplot = true;
        m_pContent->refreshPlot(NULL);
        m_pContent->m_forceReplot = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal TwipOGLFigure::setPlaneAlpha(int idx, int alpha)
{
    if (m_pContent)
    {
        m_pContent->setPlaneAlpha(idx, alpha);
        return m_pLegend->changeAlpha(idx, cv::saturate_cast<ito::uint8>(alpha));
    }
    return ito::RetVal(ito::retError, 0, "Not content window");
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal TwipOGLFigure::toggleVisibility(int idx, bool state)
{
    if (m_pContent)
    {
        m_pContent->setPlaneVisState(idx, state);
        return m_pLegend->toggleState(idx, state);
    }
    return ito::RetVal(ito::retError, 0, "Not content window");
}

//----------------------------------------------------------------------------------------------------------------------------------
void TwipOGLFigure::updateLegend(const int index, const  int type, const int subtype, const ito::uint8 alpha, const bool enabled)
{
    m_pLegend->addEntry(index, type, subtype, alpha, enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
QVector<int> TwipOGLFigure::getPlaneIndices() const
{
    QVector<int> out;
    int key;
    foreach(key, m_pContent->m_VAO3D.keys())
    {
        out << key;
    }
    return out;
}

//----------------------------------------------------------------------------------------------------------------------------------

#include "matplotlibplot.h"

//----------------------------------------------------------------------------------------------------------------------------------
MatplotlibPlot::MatplotlibPlot(QWidget *parent)
    : QMainWindow(parent),
    m_actHome(NULL),
    m_actForward(NULL),
    m_actBack(NULL),
    m_actPan(NULL),
    m_actZoomToRect(NULL),
    m_actSubplotConfig(NULL),
    m_actSave(NULL),
    m_actMarker(NULL),
    m_contextMenu(NULL),
    m_pContent(NULL),
    m_pMatplotlibSubfigConfig(NULL)
{
    setWindowFlags(Qt::Widget); //this is important such that this main window reacts as widget

    m_actHome = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"), tr("home"), this);
    m_actHome->setObjectName("actionHome");
    m_actHome->setToolTip(tr("Reset original view"));

    m_actForward = new QAction(QIcon(":/itomDesignerPlugins/general/icons/forward.png"), tr("forward"), this);
    m_actForward->setObjectName("actionForward");
    m_actForward->setToolTip(tr("Forward to next view"));

    m_actBack = new QAction(QIcon(":/itomDesignerPlugins/general/icons/back.png"), tr("back"), this);
    m_actBack->setObjectName("actionBack");
    m_actBack->setToolTip(tr("Back to previous view"));

    m_actPan = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), tr("move"), this);
    m_actPan->setObjectName("actionPan");
    m_actPan->setCheckable(true);
    m_actPan->setChecked(false);
    m_actPan->setToolTip(tr("Pan axes with left mouse, zoom with right"));

    m_actZoomToRect = new QAction(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"), tr("zoom to rectangle"), this);
    m_actZoomToRect->setObjectName("actionZoomToRect");
    m_actZoomToRect->setCheckable(true);
    m_actZoomToRect->setChecked(false);
    m_actZoomToRect->setToolTip(tr("Zoom to rectangle"));

    m_actMarker = new QAction(QIcon(":/itomDesignerPlugins/general/icons/marker.png"), tr("marker"), this);
    m_actMarker->setObjectName("actionMarker");
    m_actMarker->setCheckable(true);
    m_actMarker->setChecked(false);
    m_actMarker->connect(m_actMarker, SIGNAL(toggled(bool)), this, SLOT(mnuMarkerClick(bool)));

    m_actSubplotConfig = new QAction(QIcon(":/itomDesignerPlugins/general/icons/subplots.png"), tr("subplot configuration"), this);
    m_actSubplotConfig->setObjectName("actionSubplotConfig");
    m_actSubplotConfig->setToolTip(tr("Configure subplots"));

    m_actSave = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("save"), this);
    m_actSave->setObjectName("actionSave");
    m_actSave->setToolTip(tr("Save the figure"));

    m_lblCoordinates = new QLabel("",this);
    m_lblCoordinates->setAlignment(Qt::AlignRight | Qt::AlignTop);
    m_lblCoordinates->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    m_lblCoordinates->setObjectName("lblCoordinates");

    m_toolbar = new QToolBar(tr("Toolbar"), this);
    m_toolbar->setObjectName("toolbar");
    m_toolbar->addAction(m_actHome);
    m_toolbar->addAction(m_actBack);
    m_toolbar->addAction(m_actForward);
    m_toolbar->addSeparator();
    m_toolbar->addAction(m_actPan);
    m_toolbar->addAction(m_actZoomToRect);
    m_toolbar->addAction(m_actMarker);
    m_toolbar->addSeparator();
    m_toolbar->addAction(m_actSubplotConfig);
    m_toolbar->addAction(m_actSave);
    QAction *lblAction = m_toolbar->addWidget(m_lblCoordinates);
    lblAction->setVisible(true);
    addToolBar(m_toolbar);

    QMenu *contextMenu = new QMenu(tr("Matplotlib"),this);
    contextMenu->addAction(m_actHome);
    contextMenu->addAction(m_actBack);
    contextMenu->addAction(m_actForward);
    contextMenu->addSeparator();
    contextMenu->addAction(m_actPan);
    contextMenu->addAction(m_actZoomToRect);
    contextMenu->addAction(m_actMarker);
    contextMenu->addSeparator();
    contextMenu->addAction(m_actSubplotConfig);
    contextMenu->addAction(m_actSave);
    contextMenu->addAction(m_toolbar->toggleViewAction());

    m_pContent = new MatplotlibWidget(contextMenu, this);
    m_pContent->setObjectName("canvasWidget");

    setCentralWidget(m_pContent);
}

//----------------------------------------------------------------------------------------------------------------------------------
MatplotlibPlot::~MatplotlibPlot()
{
    if (m_pMatplotlibSubfigConfig)
    {
        m_pMatplotlibSubfigConfig->deleteLater();
        m_pMatplotlibSubfigConfig = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::resizeCanvas(int width, int height)
{
    if (m_toolbar->isVisible() && m_toolbar->isFloating() == false)
    {
        if (m_toolbar->orientation() == Qt::Horizontal)
        {
            resize(width,height+m_toolbar->height());
        }
        else
        {
            resize(width + m_toolbar->width(),height);
        }
    }
    else
    {
        resize(width,height);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::setToolbarVisible(bool visible)
{
    if (m_toolbar) m_toolbar->setVisible(visible);
}

//----------------------------------------------------------------------------------------------------------------------------------
bool MatplotlibPlot::getToolbarVisible() const 
{ 
    if (m_toolbar) return m_toolbar->isVisible();
    return false; 
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::mnuMarkerClick(bool checked)
{
    if (m_pContent)
    {
        m_pContent->m_trackerActive = checked;
        m_pContent->setMouseTracking(checked);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::setContextMenuEnabled(bool show)
{
    if (m_pContent) 
    {
        m_pContent->m_showContextMenu = show;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool MatplotlibPlot::getContextMenuEnabled() const
{
    if (m_pContent) return m_pContent->m_showContextMenu;
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::showSubplotConfig(int valLeft, int valTop, int valRight, int valBottom, int valWSpace, int valHSpace)
{
    if (m_pMatplotlibSubfigConfig == NULL)
    {
        m_pMatplotlibSubfigConfig = new MatplotlibSubfigConfig(valLeft, valTop, valRight, valBottom, valWSpace, valHSpace, this);
        connect(m_pMatplotlibSubfigConfig->sliderLeft(), SIGNAL(valueChanged(int)), this, SLOT(subplotConfigSliderLeftChanged(int)));
        connect(m_pMatplotlibSubfigConfig->sliderTop(), SIGNAL(valueChanged(int)), this, SLOT(subplotConfigSliderTopChanged(int)));
        connect(m_pMatplotlibSubfigConfig->sliderRight(), SIGNAL(valueChanged(int)), this, SLOT(subplotConfigSliderRightChanged(int)));
        connect(m_pMatplotlibSubfigConfig->sliderBottom(), SIGNAL(valueChanged(int)), this, SLOT(subplotConfigSliderBottomChanged(int)));
        connect(m_pMatplotlibSubfigConfig->sliderWSpace(), SIGNAL(valueChanged(int)), this, SLOT(subplotConfigSliderWSpaceChanged(int)));
        connect(m_pMatplotlibSubfigConfig->sliderHSpace(), SIGNAL(valueChanged(int)), this, SLOT(subplotConfigSliderHSpaceChanged(int)));
    }

    m_pMatplotlibSubfigConfig->setModal(true);
    m_pMatplotlibSubfigConfig->show();
}


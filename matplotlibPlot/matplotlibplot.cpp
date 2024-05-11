/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2021, Institut für Technische Optik (ITO),
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

#include "matplotlibplot.h"
#include "matplotlibWidget.h"
#include "matplotlibSubfigConfig.h"
#include "editProperties/dialogEditProperties.h"

//----------------------------------------------------------------------------------------------------------------------------------
MatplotlibPlot::MatplotlibPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent /*= 0*/)
    : ito::AbstractFigure(itomSettingsFile, windowMode, parent),
    m_actHome(nullptr),
    m_actForward(nullptr),
    m_actBack(nullptr),
    m_actPan(nullptr),
    m_actZoomToRect(nullptr),
    m_actSubplotConfig(nullptr),
    m_actSave(nullptr),
    m_actMarker(nullptr),
    m_contextMenu(nullptr),
    m_pContent(nullptr),
    m_pMatplotlibSubfigConfig(nullptr),
    m_forceWindowResize(true),
    m_keepSizeFixed(false),
    m_pResetFixedSizeTimer(nullptr)
{
    setWindowFlags(Qt::Widget); //this is important such that this main window reacts as widget

    m_pResetFixedSizeTimer = new QTimer(this);
    connect(m_pResetFixedSizeTimer, SIGNAL(timeout()), this, SLOT(resetFixedSize()));
    m_pResetFixedSizeTimer->setSingleShot(true);
    m_pResetFixedSizeTimer->stop();

    m_actHome = new QAction(QIcon(":/itomDesignerPlugins/general/icons/home.png"), tr("Home"), this);
    m_actHome->setObjectName("actionHome");
    m_actHome->setShortcut(Qt::CTRL + Qt::Key_0);
    m_actHome->setToolTip(tr("Reset original view"));
    m_actHome->setVisible(false);

    m_actForward = new QAction(QIcon(":/itomDesignerPlugins/general/icons/forward.png"), tr("Forward"), this);
    m_actForward->setObjectName("actionForward");
    m_actForward->setToolTip(tr("Forward to next view"));
    m_actForward->setVisible(false);

    m_actBack = new QAction(QIcon(":/itomDesignerPlugins/general/icons/back.png"), tr("Back"), this);
    m_actBack->setObjectName("actionBack");
    m_actBack->setToolTip(tr("Back to previous view"));
    m_actBack->setVisible(false);

    m_actPan = new QAction(QIcon(":/itomDesignerPlugins/general/icons/move.png"), tr("Move"), this);
    m_actPan->setObjectName("actionPan");
    m_actPan->setCheckable(true);
    m_actPan->setChecked(false);
    m_actPan->setToolTip(tr("Pan axes with left mouse, zoom with right"));
    m_actPan->setVisible(false);

    m_actZoomToRect = new QAction(QIcon(":/itomDesignerPlugins/general/icons/zoom_to_rect.png"), tr("Zoom to Rectangle"), this);
    m_actZoomToRect->setObjectName("actionZoomToRect");
    m_actZoomToRect->setCheckable(true);
    m_actZoomToRect->setChecked(false);
    m_actZoomToRect->setToolTip(tr("Zoom to rectangle"));
    m_actZoomToRect->setVisible(false);

    m_actMarker = new QAction(QIcon(":/itomDesignerPlugins/general/icons/marker.png"), tr("Marker"), this);
    m_actMarker->setObjectName("actionMarker");
    m_actMarker->setCheckable(true);
    m_actMarker->setChecked(false);
    m_actMarker->setToolTip(tr("Show coordinates under mouse cursor"));
    m_actMarker->connect(m_actMarker, SIGNAL(toggled(bool)), this, SLOT(mnuMarkerClick(bool)));
    m_actMarker->setVisible(false);

    m_actSubplotConfig = new QAction(QIcon(":/itomDesignerPlugins/general/icons/subplots.png"), tr("Subplot Configuration..."), this);
    m_actSubplotConfig->setObjectName("actionSubplotConfig");
    m_actSubplotConfig->setToolTip(tr("Configure subplots..."));
    m_actSubplotConfig->setVisible(false);

    m_actSave = new QAction(QIcon(":/itomDesignerPlugins/general/icons/filesave.png"), tr("Save..."), this);
    m_actSave->setShortcut(QKeySequence::Save);
    m_actSave->setObjectName("actionSave");
    m_actSave->setToolTip(tr("Save the figure..."));
    m_actSave->setVisible(false);

    //m_actCopyClipboard
    m_actCopyClipboard = new QAction(tr("Copy To Clipboard"), this);
    m_actCopyClipboard->setObjectName("actionCopyClipboard");
    m_actCopyClipboard->setShortcut(QKeySequence::Copy);
    m_actCopyClipboard->setToolTip(tr("Copies the current view to the clipboard"));
    connect(m_actCopyClipboard, SIGNAL(triggered()), this, SLOT(mnuCopyToClipboard()));
    m_actCopyClipboard->setVisible(false);

    m_actProperties = this->getPropertyDockWidget()->toggleViewAction();
    connect(m_actProperties, SIGNAL(triggered(bool)), this, SLOT(mnuShowProperties(bool)));

    m_lblCoordinates = new QLabel("",this);
    m_lblCoordinates->setAlignment(Qt::AlignLeft | Qt::AlignCenter);
    m_lblCoordinates->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Ignored);
    m_lblCoordinates->setObjectName("lblCoordinates");

    m_toolbar = new QToolBar(tr("matplotlib toolbar"), this);
    addToolBar(m_toolbar, "mainToolBar");
    m_toolbar->setObjectName("toolbar");
    m_toolbar->addAction(m_actSave);
    m_toolbar->addAction(m_actHome);
    m_toolbar->addAction(m_actBack);
    m_toolbar->addAction(m_actForward);
    m_toolbar->addAction(m_actPan);
    m_toolbar->addAction(m_actZoomToRect);
    m_pToolbarBeforeAct = m_actSubplotConfig;
    m_toolbar->addAction(m_actSubplotConfig);
    m_toolbar->addAction(m_actMarker);

    QAction *lblAction = m_toolbar->addWidget(m_lblCoordinates);
    lblAction->setVisible(true);

    m_contextMenu = new QMenu(tr("Matplotlib"), this);
    m_contextMenu->addAction(m_actSave);
    m_contextMenu->addAction(m_actCopyClipboard);
    m_contextMenu->addSeparator();
    m_contextMenu->addAction(m_actHome);
    m_contextMenu->addAction(m_actBack);
    m_contextMenu->addAction(m_actForward);
    m_contextMenu->addAction(m_actPan);
    m_contextMenu->addAction(m_actZoomToRect);
    m_contextMenu->addSeparator();
    m_contextMenu->addAction(m_actMarker);
    m_contextMenu->addSeparator();
    m_contextMenu->addAction(m_actSubplotConfig);
    m_pContextMenuBeforeAct = m_contextMenu->addSeparator();
    m_contextMenu->addAction(m_toolbar->toggleViewAction());
    m_contextMenu->addAction(m_actProperties);
    addMenu(m_contextMenu);

    m_pContent = new MatplotlibWidget(m_contextMenu, this);
    setContentsMargins(0, 0, 0, 0);
    m_pContent->setObjectName("canvasWidget");

    setCentralWidget(m_pContent);

    setPropertyObservedObject(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
MatplotlibPlot::~MatplotlibPlot()
{
    if (m_pMatplotlibSubfigConfig)
    {
        m_pMatplotlibSubfigConfig->deleteLater();
        m_pMatplotlibSubfigConfig.clear();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::resizeCanvas(int width, int height)
{
    QSize newSize(width, height);
    if (m_pContent)
    {
        //sometimes the size of the canvas is smaller than the size of this widget.
        //Since resizeCanvas should control the size of the canvas, the offset is added here.
        newSize += (size() - m_pContent->size());
    }

    resize(newSize);

    if (m_forceWindowResize)
    {
        //qDebug() << "fixed size (resizeCanvas)" << width << height;
        setFixedSize(newSize); //forces the window to a fixed size...
        updateGeometry();

        if (!m_keepSizeFixed)
        {
            m_pResetFixedSizeTimer->start(50); //and fire a trigger in order to cancel the fixed size (this is a hack in order to really force the window to its new size)
        }
    }
    else
    {
        updateGeometry();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::setKeepSizeFixed(bool fixed)
{
    if (fixed != m_keepSizeFixed)
    {
        if (fixed)
        {
            //qDebug() << "fixed size" << geometry().size();
            setFixedSize(geometry().size());
            m_pResetFixedSizeTimer->stop();
        }
        else
        {
            //qDebug() << "resetFixedSize";
            resetFixedSize();
        }
        m_keepSizeFixed = fixed;

        if (m_pContent)
            m_pContent->setKeepSizeFixed(fixed);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::resetFixedSize()
{
    setFixedSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    //qDebug() << "resetFixedSize";
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::mnuMarkerClick(bool checked)
{
    if (m_pContent)
    {
        m_pContent->m_trackerActive = checked;
        m_pContent->setMouseTracking(checked);

        if (!checked)
        {
            m_lblCoordinates->setText("");
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::mnuCopyToClipboard()
{
    copyToClipBoard();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MatplotlibPlot::copyToClipBoard()
{
    if (m_pContent)
    {
        int dpi = 200;

        if (ito::ITOM_API_FUNCS_GRAPH)
        {
            dpi = qBound(48, apiGetFigureSetting(this, "copyClipboardResolutionDpi", 200, nullptr).value<int>(), 2000);
        }

        m_pContent->copyToClipboard(dpi);
        return ito::retOk;
    }
    return ito::RetVal(ito::retError, 0, tr("no content widget available.").toLatin1().data());
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
    if (m_pContent)
    {
        return m_pContent->m_showContextMenu;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
//valLeft, valTop, valRight, valBottom, valWSpace, valHSpace are
//fractions of the image width and height multiplied by 1000 (resolution: 0.1%)
void MatplotlibPlot::showSubplotConfig(float valLeft, float valTop, float valRight, float valBottom, float valWSpace, float valHSpace)
{
    if (m_pMatplotlibSubfigConfig.isNull())
    {
        m_pMatplotlibSubfigConfig = new MatplotlibSubfigConfig(valLeft, valTop, valRight, valBottom, valWSpace, valHSpace, this);
        m_pMatplotlibSubfigConfig->setAttribute(Qt::WA_DeleteOnClose);
        connect((m_pMatplotlibSubfigConfig)->sliderLeftRight(), SIGNAL(minimumValueChanged(double)), this, SLOT(subplotConfigSliderLeftChanged(double)));
        connect((m_pMatplotlibSubfigConfig)->sliderLeftRight(), SIGNAL(maximumValueChanged(double)), this, SLOT(subplotConfigSliderRightChanged(double)));
        connect((m_pMatplotlibSubfigConfig)->sliderBottomTop(), SIGNAL(minimumValueChanged(double)), this, SLOT(subplotConfigSliderBottomChanged(double)));
        connect((m_pMatplotlibSubfigConfig)->sliderBottomTop(), SIGNAL(maximumValueChanged(double)), this, SLOT(subplotConfigSliderTopChanged(double)));
        connect((m_pMatplotlibSubfigConfig)->sliderWSpace(), SIGNAL(valueChanged(double)), this, SLOT(subplotConfigSliderWSpaceChanged(double)));
        connect((m_pMatplotlibSubfigConfig)->sliderHSpace(), SIGNAL(valueChanged(double)), this, SLOT(subplotConfigSliderHSpaceChanged(double)));
        connect((m_pMatplotlibSubfigConfig)->resetButton(), SIGNAL(clicked()), this, SIGNAL(subplotConfigTight()));
        connect((m_pMatplotlibSubfigConfig)->tightButton(), SIGNAL(clicked()), this, SIGNAL(subplotConfigReset()));
    }

    m_pMatplotlibSubfigConfig->setModal(false);
    m_pMatplotlibSubfigConfig->show();
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::modifySubplotSliders(float left, float top, float right, float bottom, float wSpace, float hSpace)
{
    if (m_pMatplotlibSubfigConfig)
    {
        m_pMatplotlibSubfigConfig->modifyValues(left, top, right, bottom, wSpace, hSpace);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::subplotConfigSliderLeftChanged(double value)
{
    emit subplotConfigSliderChanged(0, qRound(value * 10.0));
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::subplotConfigSliderTopChanged(double value)
{
    emit subplotConfigSliderChanged(1, qRound(value * 10.0));
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::subplotConfigSliderRightChanged(double value)
{
    emit subplotConfigSliderChanged(2, qRound(value * 10.0));
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::subplotConfigSliderBottomChanged(double value)
{
    emit subplotConfigSliderChanged(3, qRound(value * 10.0));
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::subplotConfigSliderWSpaceChanged(double value)
{
    emit subplotConfigSliderChanged(4, qRound(value * 10.0));
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::subplotConfigSliderHSpaceChanged(double value)
{
    emit subplotConfigSliderChanged(5, qRound(value * 10.0));
}


//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::replot()
{
    if (m_pContent)
    {
        m_pContent->replot();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::setLabelText(QString text)
{
	QStringList splits = text.split("\\u"); //begin of unicode sign
	if (splits.size() < 2)
	{
		m_lblCoordinates->setText(text);
	}
	else
	{
		QString temp;
		int hexCode;

		for (int i = 1; i < splits.size(); ++i)
		{
			temp = splits[i];
			if (temp.size() >= 4)
			{
				hexCode = temp.left(4).toInt(nullptr, 16);
				temp = QChar(hexCode) + temp.mid(4);
				splits[i] = temp;
			}
		}

		m_lblCoordinates->setText(splits.join(""));
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
QAction* MatplotlibPlot::getActionFromGroupByName(const QString &name) const
{
    QMap<QString, ActionGroup>::const_iterator groupIt = m_actionGroups.constBegin();

    while (groupIt != m_actionGroups.constEnd())
    {
        foreach(QAction *a, groupIt->m_pActions)
        {
            if (a && a->objectName() == name)
            {
                return a;
            }
        }

        ++groupIt;
    }

    return nullptr;
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::addUserDefinedAction(const QString &name, const QString &text, const QString &iconFilename,
    const QString &tooltip, const QString &groupName, int position /*= -1*/)
{
    if (getActionFromGroupByName(name))
    {
        qDebug() << "action with name '" << name << "' already exists in action group";
        return;
    }

    if (!m_actionGroups.contains(groupName))
    {
        ActionGroup ag;
        ag.m_pSeparatorToolbar = m_toolbar->insertSeparator(m_pToolbarBeforeAct);
        ag.m_pSeparatorContextMenu = m_contextMenu->insertSeparator(m_pContextMenuBeforeAct);
        m_actionGroups[groupName] = ag;
    }

    ActionGroup &actGroup = m_actionGroups[groupName];

    QAction *action = nullptr;

    if (text == "")
    {
        action = new QAction(this);
        action->setSeparator(true);
    }
    else
    {
        QPixmap pm(iconFilename);
        pm.setDevicePixelRatio(this->devicePixelRatio());

        action = new QAction(QIcon(pm), text, this);
    }

    QAction *beforeToolbar = m_pToolbarBeforeAct;
    QAction *beforeContextMenu = m_pContextMenuBeforeAct;

    action->setToolTip(tooltip);

    if (name != "")
    {
        action->setObjectName(name);
    }

    if (position <= 0)
    {
        if (actGroup.m_pActions.size() > 0)
        {
            beforeToolbar = beforeContextMenu = actGroup.m_pActions[0];
        }
        actGroup.m_pActions.insert(std::max(0, position), action);
    }
    else if (position >= actGroup.m_pActions.size())
    {
        actGroup.m_pActions.append(action);
    }
    else
    {
        if (actGroup.m_pActions.size() > 0)
        {
            beforeToolbar = beforeContextMenu = actGroup.m_pActions[position];
        }
        actGroup.m_pActions.insert(position, action);
    }

    m_toolbar->insertAction(beforeToolbar, action);
    m_contextMenu->insertAction(beforeContextMenu, action);
}

//----------------------------------------------------------------------------------------------------------------------------------
void MatplotlibPlot::removeUserDefinedAction(const QString &name)
{
    QAction *a = nullptr;
    QMap<QString, ActionGroup>::iterator groupIt = m_actionGroups.begin();

    while (groupIt != m_actionGroups.end())
    {
        for (int idx = groupIt->m_pActions.size() - 1; idx >= 0; --idx)
        {
            a = groupIt->m_pActions[idx];
            if (a && a->objectName() == name)
            {
                a->deleteLater();
                groupIt.value().m_pActions.removeAt(idx);
            }
        }

        if (groupIt->m_pActions.size() == 0)
        {
            //delete the entire group
            groupIt->m_pSeparatorToolbar->deleteLater();
            groupIt->m_pSeparatorContextMenu->deleteLater();
            m_actionGroups.erase(groupIt);
        }
        else
        {
            ++groupIt;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QWidget* MatplotlibPlot::createDialogEditProperties(bool showApplyButton, const QString &title /*= ""*/)
{
    DialogEditProperties *dialog = new DialogEditProperties(showApplyButton, title, this);

    return dialog;

}

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

#ifndef MATPLOTLIBPLOT_H
#define MATPLOTLIBPLOT_H

#if defined(ITOMSHAREDDESIGNER)
    #define ITOMMATPLOTLIB_EXPORT Q_DECL_EXPORT
#else
    #define ITOMMATPLOTLIB_EXPORT Q_DECL_IMPORT
#endif

#include <QtCore/QtGlobal>
#include <qmainwindow.h>
#include <qaction.h>
#include <qtoolbar.h>
#include <qlabel.h>
#include <qpointer.h>

#include "plot/AbstractItomDesignerPlugin.h"

class MatplotlibSubfigConfig; //forward declaration
class MatplotlibWidget; //forward declaration
class QTimer;

class ITOMMATPLOTLIB_EXPORT MatplotlibPlot : public ito::AbstractFigure
{
    Q_OBJECT

    //DESIGNABLE (default: true): property is visible in QtDesigner property editor
    //USER (default: false): property is visible in property editor of plot

    Q_PROPERTY(bool forceWindowResize READ getForceWindowResize WRITE setForceWindowResize USER true)
    Q_PROPERTY(bool keepSizeFixed READ getKeepSizeFixed WRITE setKeepSizeFixed USER true)


    Q_CLASSINFO("prop://forceWindowResize", "If set, the plot widget / area is resized to the desired sizes given by matplotlib. Uncheck this option, if you want to keep the canvas unchanged e.g. in an user-defined GUI")
    Q_CLASSINFO("prop://keepSizeFixed", "If you want to control the size of the canvas by python / matplotlib (e.g. set_size_inches), set this to true. The canvas will then have a fixed size, that is not affected by the window size.")

    Q_CLASSINFO("slot://showSubplotConfig", "displays the subplot configuration dialog.\n"
    "\n"
    "This slot must usually not be used, since the dialog can be opened by the toolbar.\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "left : {float}\n"
    "    left border of the current subplot configuration.\n"
    "top : {float}\n"
    "    top border of the current subplot configuration.\n"
    "right : {float}\n"
    "    right border of the current subplot configuration.\n"
    "bottom : {float}\n"
    "    bottom border of the current subplot configuration.\n"
    "wSpace : {float}\n"
    "    horizontal space between every subplot of the current configuration.\n"
    "hSpace : {float}\n"
    "    vertical space between every subplot of the current configuration.")

    Q_CLASSINFO("slot://setLabelText", "displays a text in the toolbar\n"
    "\n"
    "The text is displayed in the label that is usually used for coordinates of the mouse cursor....\n"
    "\n"
    "Parameters\n"
    "-------------\n"
    "text : {str}\n"
    "    text to display")

    Q_CLASSINFO("slot://replot", "forces a replot of the plot")

    Q_CLASSINFO("signal://subplotConfigSliderChanged", "internal use between MatplotlibPlot and the subplot configuration dialog.")

    Q_CLASSINFO("slot://copyToClipBoard", "copies the entire plot to the clipboard as bitmap data (uses the default export resolution).")

    DESIGNER_PLUGIN_ITOM_API
public:
    MatplotlibPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
    ~MatplotlibPlot();

    //properties
    void setContextMenuEnabled(bool show);
    bool getContextMenuEnabled() const;

    void setForceWindowResize(bool force) { m_forceWindowResize = force; }
    bool getForceWindowResize() const { return m_forceWindowResize; }

    void setKeepSizeFixed(bool fixed);
    bool getKeepSizeFixed() const { return m_keepSizeFixed; }

    void resizeCanvas(int width, int height);

    ito::RetVal applyUpdate(void) { return ito::RetVal(ito::retWarning, 0, "not used in this plugin"); }
    ito::RetVal update(void) { return ito::RetVal(ito::retWarning, 0, "not used in this plugin"); }

private:
    QAction *m_actHome;
    QAction *m_actForward;
    QAction *m_actBack;
    QAction *m_actPan;
    QAction *m_actZoomToRect;
    QAction *m_actSubplotConfig;
    QAction *m_actSave;
    QAction *m_actMarker;
    QAction *m_actProperties;
    QAction *m_actCopyClipboard;
    QLabel *m_lblCoordinates;
    QToolBar *m_toolbar;
    QMenu *m_contextMenu;
    MatplotlibWidget *m_pContent;
    QPointer<MatplotlibSubfigConfig> m_pMatplotlibSubfigConfig;
    bool m_forceWindowResize;
    bool m_keepSizeFixed;
    QTimer *m_pResetFixedSizeTimer;
    QAction *m_pContextMenuBeforeAct; //action, before which all user-defined actions are added to the context menu
    QAction *m_pToolbarBeforeAct; //action, before which all user-defined actions are added to the toolbar

    struct ActionGroup
    {
        QAction *m_pSeparatorToolbar; //leading separator
        QAction *m_pSeparatorContextMenu; //leading separator
        QList<QAction*> m_pActions;
    };

    QMap<QString, ActionGroup> m_actionGroups;

    QAction *getActionFromGroupByName(const QString &name) const;

signals:
    void subplotConfigSliderChanged(int type, int value);
    void subplotConfigTight();
    void subplotConfigReset();

private slots:
    void mnuMarkerClick(bool /*checked*/);
    inline void mnuShowToolbar(bool /*checked*/) { setToolbarVisible(true); }
    void mnuCopyToClipboard();

    void subplotConfigSliderLeftChanged(double left);
    void subplotConfigSliderRightChanged(double left);
    void subplotConfigSliderBottomChanged(double bottom);
    void subplotConfigSliderTopChanged(double top);
    void subplotConfigSliderWSpaceChanged(double value);
    void subplotConfigSliderHSpaceChanged(double value);

    void resetFixedSize();

public slots:
    void showSubplotConfig(float left, float top, float right, float bottom, float wSpace, float hSpace);
    void modifySubplotSliders(float left, float top, float right, float bottom, float wSpace, float hSpace);
	void setLabelText(QString text);
    void replot();
    void addUserDefinedAction(const QString &name, const QString &text, const QString &iconFilename,
        const QString &tooltip, const QString &groupName, int position = 0);
    void removeUserDefinedAction(const QString &name);
    QWidget* createDialogEditProperties(bool showApplyButton, const QString &title = "");

    ito::RetVal copyToClipBoard();

};

#endif // MATPLOTLIBPLOT_H

/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2016, Institut fuer Technische Optik (ITO), 
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

#include "plot/AbstractItomDesignerPlugin.h"
#include "matplotlibWidget.h"
#include "matplotlibSubfigConfig.h"

class MatplotlibSubfigConfig; //forward declaration
class MatplotlibWidget; //forward declaration
class QTimer;

class ITOMMATPLOTLIB_EXPORT MatplotlibPlot : public ito::AbstractFigure
{
    Q_OBJECT

    Q_PROPERTY(bool forceWindowResize READ getForceWindowResize WRITE setForceWindowResize USER true)
    Q_PROPERTY(bool keepSizeFixed READ getKeepSizeFixed WRITE setKeepSizeFixed USER true)

    Q_CLASSINFO("prop://forceWindowResize", "If set, the plot widget / area is resized to the desired sizes given by matplotlib. Uncheck this option, if you want to keep the canvas unchanged e.g. in an user-defined GUI")
    Q_CLASSINFO("prop://keepSizeFixed", "If you want to control the size of the canvas by python / matplotlib (e.g. set_size_inches), set this to true. The canvas will then have a fixed size, that is not affected by the window size.")
    
    Q_CLASSINFO("slot://showSubplotConfig", "")
    Q_CLASSINFO("slot://setLabelText", "")

    Q_CLASSINFO("signal://subplotConfigSliderChanged", "")

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
    QLabel *m_lblCoordinates;
    QToolBar *m_toolbar;
    QMenu *m_contextMenu;
    MatplotlibWidget *m_pContent;
    MatplotlibSubfigConfig *m_pMatplotlibSubfigConfig;
    bool m_forceWindowResize;
    bool m_keepSizeFixed;
    QTimer *m_pResetFixedSizeTimer;

signals:
    void subplotConfigSliderChanged(int type, int value);

private slots:
    void mnuMarkerClick(bool /*checked*/);
    inline void mnuShowToolbar(bool /*checked*/) { setToolbarVisible(true); }

    void subplotConfigSliderLeftChanged(double left);
    void subplotConfigSliderRightChanged(double left);
    void subplotConfigSliderBottomChanged(double bottom);
    void subplotConfigSliderTopChanged(double top);
    void subplotConfigSliderWSpaceChanged(double value);
    void subplotConfigSliderHSpaceChanged(double value);

    void resetFixedSize() { setFixedSize(QWIDGETSIZE_MAX,QWIDGETSIZE_MAX); }

public slots:
void showSubplotConfig(float valLeft, float valTop, float valRight, float valBottom, float valWSpace, float valHSpace);
    void setLabelText(QString text) { m_lblCoordinates->setText(text); }

};

#endif // MATPLOTLIBPLOT_H

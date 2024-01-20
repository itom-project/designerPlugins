/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2024, Institut fuer Technische Optik (ITO),
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

#pragma once

#if defined(ITOMSHAREDDESIGNER)
    #define ITOMPLOTLY_EXPORT Q_DECL_EXPORT
#else
    #define ITOMPLOTLY_EXPORT Q_DECL_IMPORT
#endif

#include <QtCore/QtGlobal>
#include <qmainwindow.h>

#include <qscopedpointer.h>

#include "plot/AbstractItomDesignerPlugin.h"

class PlotlyPlotPrivate;

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
class QWebEngineDownloadItem;
#else
class QWebEngineDownloadRequest;
#endif

class ITOMPLOTLY_EXPORT PlotlyPlot : public ito::AbstractFigure
{
    Q_OBJECT

    //DESIGNABLE (default: true): property is visible in QtDesigner property editor
    //USER (default: false): property is visible in property editor of plot

    Q_CLASSINFO("slot://setHtml", "Set this html document as content.")
    Q_CLASSINFO("slot://clear", "Clears the canvas.")

    Q_DECLARE_PRIVATE(PlotlyPlot)

    DESIGNER_PLUGIN_ITOM_API
public:
    PlotlyPlot(const QString &itomSettingsFile, AbstractFigure::WindowMode windowMode, QWidget *parent = 0);
    ~PlotlyPlot();

    //properties
    void setContextMenuEnabled(bool show);
    bool getContextMenuEnabled() const;

    ito::RetVal applyUpdate(void) { return ito::RetVal(ito::retWarning, 0, "not used in this plugin"); }
    ito::RetVal update(void) { return ito::RetVal(ito::retWarning, 0, "not used in this plugin"); }

private:
    QScopedPointer<PlotlyPlotPrivate> d_ptr;

public slots:
    void setHtml(const QString &html);
    void clear();

private slots:

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    void downloadRequested(QWebEngineDownloadItem* download);
#else
    void downloadRequested(QWebEngineDownloadRequest* download);
#endif

};

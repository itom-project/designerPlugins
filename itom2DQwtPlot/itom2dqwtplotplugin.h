/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#ifndef ITOM2DQWTPLOTPLUGIN_H
#define ITOM2DQWTPLOTPLUGIN_H

#include "qglobal.h"
#include <QtUiPlugin/QDesignerCustomWidgetInterface>
#include "plot/AbstractItomDesignerPlugin.h"

class Itom2dQwtPlotPlugin : public ito::AbstractItomDesignerPlugin /*, public QDesignerCustomWidgetInterface*/
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.qt-project.Qt.QDesignerCustomWidgetInterface"  FILE "pluginMetaData.json")

public:
    Itom2dQwtPlotPlugin(QObject *parent = 0);

    bool isContainer() const;
    bool isInitialized() const;
    QIcon icon() const;
    QString domXml() const;
    QString group() const;
    QString includeFile() const;
    QString name() const;
    QString toolTip() const;
    QString whatsThis() const;
    QWidget *createWidget(QWidget *parent);
    QWidget *createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent);
    void initialize(QDesignerFormEditorInterface *core);

private:
    bool initialized;
};

#endif // ITOM2DQWTPLOTPLUGIN_H

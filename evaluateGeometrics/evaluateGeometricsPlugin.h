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

/*!
 * \file evaluateGeometricsPlugin.h
 * \brief This file contains the factory declaration for the evaluateGeometrics-Widget.
 */

#ifndef ITOMPLOTPLUGIN_H
#define ITOMPLOTPLUGIN_H

#include "plot/AbstractItomDesignerPlugin.h"


/*!
 * \class EvaluateGeometricsPlugin
 * \brief The factory class for the evaluate geometric plugin
 */

class EvaluateGeometricsPlugin : public ito::AbstractItomDesignerPlugin
{
    Q_PLUGIN_METADATA(IID "org.qt-project.Qt.QDesignerCustomWidgetInterface"  FILE "pluginMetaData.json")
    Q_OBJECT

public:
    EvaluateGeometricsPlugin(QObject *parent = 0);                                              /*!< Class constructor */

    bool isContainer() const;                                                                   /*!< This is no container, hence return false.  */
    bool isInitialized() const;                                                                 /*!< Check wether factory is initialized and return status */
    QIcon icon() const;                                                                         /*!< Return icon for the qt-designer */
    QString domXml() const;                                                                     /*!< The group of this widget is itom.... */
    QString group() const;
    QString includeFile() const;
    QString name() const;
    QString toolTip() const;
    QString whatsThis() const;
    QWidget *createWidget(QWidget *parent);
    QWidget *createWidgetWithMode(ito::AbstractFigure::WindowMode winMode, QWidget * parent);
    void initialize(QDesignerFormEditorInterface *core);

private:
    bool initialized;                                                                           /*!< If this plugin was initialized, this variable becomes true, else false */
};

#endif // ITOMPLOTPLUGIN_H

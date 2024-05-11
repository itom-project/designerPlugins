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

/**
* \file motorControllerFactory.h
* \brief In this file of the factory for the MotorController-Widget is declared
*
*    The MotorController-Class defines a widget for generic motor monitoring and control. The following files are
*   needed: MotorController.cpp, MotorController.h, MotorControllerFactory.h, MotorControllerFactory.cpp
*
*\sa MotorController, MotorControllerFactory, MotorController.h
*\author ITO
*\date    2013
*/

#ifndef MCFACTORY_H
#define MCFACTORY_H

#include "qglobal.h"
#include <QtUiPlugin/QDesignerCustomWidgetInterface>

class MotorControllerFactory : public QObject, public QDesignerCustomWidgetInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.qt-project.Qt.QDesignerCustomWidgetInterface"  FILE "pluginMetaData.json")
    Q_INTERFACES(QDesignerCustomWidgetInterface)

public:
    MotorControllerFactory(QObject *parent = 0);

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
    void initialize(QDesignerFormEditorInterface *core);

protected:

    int m_version;                                  //!< plugin version
    int m_maxItomVer;                               //!< minimum required version of the main program
    int m_minItomVer;                               //!< maximum supported version of the main program
    QString m_author;                                //!< the plugin author
    QString m_license;                              //!< a short license string for the plugin, default value is "LGPL with ITO itom-exception"
    QString m_aboutThis;

private:
    bool initialized;
};

#endif // MCFACTORY_H

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
#    Foundation. See <https://bitbucket.org/itom/> 
#
#    You should have received a copy of the GNU Library General Public License
#    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef TWIPOGLPLUGIN_H
#define TWIPOGLPLUGIN_H

#include <qglobal.h>
#if QT_VERSION >= QT_VERSION_CHECK(5,5,0)
#include <QtUiPlugin/QDesignerCustomWidgetInterface>
#else
#include <QtDesigner/QDesignerCustomWidgetInterface>
#endif
#include "plot/AbstractItomDesignerPlugin.h"

class TwipOGLPlugin : public ito::AbstractItomDesignerPlugin
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5,0,0)
    Q_PLUGIN_METADATA(IID "org.qt-project.Qt.QDesignerCustomWidgetInterface"  FILE "pluginMetaData.json")
#endif

public:
    TwipOGLPlugin(QObject *parent = 0);

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

#endif // TWIPOGLPLUGIN_H

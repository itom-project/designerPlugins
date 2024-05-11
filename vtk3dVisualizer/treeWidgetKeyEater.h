/* ********************************************************************
itom measurement system
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2018, Institut für Technische Optik (ITO),
Universität Stuttgart, Germany

This file is part of the designer widget 'vtk3dVisualizer' for itom.

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

#include <qobject.h>
#include <qevent.h>

class QPropertyEditorWidget;

class TreeWidgetKeyEater : public QObject
{
    Q_OBJECT

public:
    TreeWidgetKeyEater(QObject *obj, QPropertyEditorWidget* propertyEditorWidget = nullptr) : QObject(obj), pew(propertyEditorWidget) {}
    virtual ~TreeWidgetKeyEater() {}

protected:
    virtual bool eventFilter(QObject *obj, QEvent *event);

    QPropertyEditorWidget* pew;
};

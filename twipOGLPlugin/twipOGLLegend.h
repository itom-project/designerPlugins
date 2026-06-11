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
#    itom is free software by ITO, Universität Stuttgart published under
#    GNU General Public License as published by the Free Software
#    Foundation. See <https://github.com/itom-project/itom>
#
#    You should have received a copy of the GNU Library General Public License
#    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef TWIPOGLLEGEND_H
#define TWIPOGLLEGEND_H

#include <QTreeWidget>
#include "common/sharedStructures.h"

class TwipLegend : public QTreeWidget
{
    Q_OBJECT

public:

    enum tObjectType
    {
        tNoType,
        tDataObject,
        tPointCloud,
        tPolygonMesh
    };

    TwipLegend(QWidget* parent, QWidget* observedObject);

    ito::RetVal addEntry(const int index, const  int type, const int subtype, const ito::uint8 alpha, const bool enabled);
    ito::RetVal toggleState(const int index, const bool enabled);
    ito::RetVal changeAlpha(const int index, const ito::uint8 alpha);
private:
    ito::RetVal modifiyObject(const int row, const int index, const  int type, const int subtype, const ito::uint8 alpha, const bool enabled);

    QWidget* m_observedObject;
    bool m_isUpdating;
private slots:

    void itemChangedDone(QTreeWidgetItem * item, int column);

};


#endif

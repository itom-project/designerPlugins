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

#include "item.h"

#include <qtreewidget.h>

/*static*/ int Item::itemRole = Qt::UserRole + 1;

Q_DECLARE_METATYPE ( SharedItemPtr )

Item::Item(const QString &name, rttiItem rtti, QTreeWidgetItem *treeItem) :
    m_name(name),
    m_visible(true),
    m_treeItem(treeItem),
    m_rtti(rtti)
{
    m_type = "";
}

Item::~Item()
{
}

void Item::setVisible(bool value)
{
    m_visible = value;
    QTreeWidgetItem *item;
    SharedItemPtr _item;
    if (m_treeItem)
    {
        for (int i = 0; i < m_treeItem->childCount(); ++i)
        {
            item = m_treeItem->child(i);
            _item = item->data( 0, Item::itemRole).value<SharedItemPtr>();
            if (_item.data())
            {
                _item->setVisible(value);
            }
        }
    }
}


//######################################################################

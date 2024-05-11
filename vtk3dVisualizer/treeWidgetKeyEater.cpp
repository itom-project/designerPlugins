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

#include "treeWidgetKeyEater.h"

#include <qtreewidget.h>
#include "item.h"
#include "QPropertyEditor/QPropertyEditorWidget.h"


bool TreeWidgetKeyEater::eventFilter(QObject *obj, QEvent *event)
{
    QTreeWidget *tw = qobject_cast<QTreeWidget*>(obj);

    if (tw && event->type() == QEvent::KeyPress)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        //if v is pressed on item its visibility should be toggled
        if (keyEvent->key() == Qt::Key_V)
        {
            if (tw->currentItem() != NULL)
            {
                SharedItemPtr _item = tw->currentItem()->data(0, Item::itemRole).value<SharedItemPtr>();
                _item->setVisible(!_item->visible());
                if (pew)
                {
                    pew->updateObject(_item.data());
                }
            }
            return true;
        }
        return false;
    }
    else
    {
        // standard event processing
        return QObject::eventFilter(obj, event);
    }
}

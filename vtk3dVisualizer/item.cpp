#include "item.h"

#include <qtreewidget.h>

/*static*/ int Item::itemRole = Qt::UserRole + 1;

Q_DECLARE_METATYPE ( SharedItemPtr )

Item::Item(const QString &name, QTreeWidgetItem *treeItem) :
    m_name(name),
    m_visible(true),
    m_treeItem(treeItem)
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


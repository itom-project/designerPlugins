/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2021, Institut für Technische Optik (ITO),
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

#include "markerModel.h"

#include "qwt_plot.h"
#include "qwt_text.h"
#include "qwt_graphic.h"

//-------------------------------------------------------------------------------------
MarkerModel::MarkerModel(bool markerLabelsVisible, QObject* parent /*= nullptr*/) :
    QAbstractItemModel(parent), m_currentPlaneIdx(0), m_markerLabelsVisible(markerLabelsVisible)
{
    m_headers << tr("Index") << tr("X") << tr("Y") << tr("Plane Index");
    m_alignment << Qt::AlignLeft << Qt::AlignLeft << Qt::AlignLeft << Qt::AlignLeft;
}

//-------------------------------------------------------------------------------------
MarkerModel::~MarkerModel()
{
}

//-------------------------------------------------------------------------------------
void MarkerModel::addMarkers(const QString& setname, const QList<MarkerItem>& markers)
{
    int idx = m_setnames.indexOf(setname);

    QwtText label;

    if (m_markerLabelsVisible)
    {
        label = QString(" %1").arg(setname);
    }

    for (int i = 0; i < markers.size(); ++i)
    {
        markers[i].marker->setLabel(label);
        markers[i].marker->setVisible(
            markers[i].planeIndex == -1 || markers[i].planeIndex == m_currentPlaneIdx);
    }

    if (idx == -1)
    {
        // new setname
        beginInsertRows(QModelIndex(), m_setnames.size(), m_setnames.size());

        m_setnames.append(setname);
        m_markers[setname] = markers;

        endInsertRows();
    }
    else
    {
        // add to existing setname
        auto& existingMarkers = m_markers[setname];
        beginInsertRows(
            createIndex(idx, 0), existingMarkers.size(), existingMarkers.size() + markers.size());
        existingMarkers.append(markers);
        endInsertRows();
    }
}

//-------------------------------------------------------------------------------------
bool MarkerModel::removeAllMarkersFromSet(const QString& setname)
{
    int idx = m_setnames.indexOf(setname);

    if (idx >= 0)
    {
        beginRemoveRows(QModelIndex(), idx, idx);
        m_setnames.removeAt(idx);

        // detach and delete all markers
        foreach(auto &item, m_markers[setname])
        {
            item.marker->detach();
            delete item.marker;
        }

        m_markers.remove(setname);
        endRemoveRows();
    }

    return idx >= 0;
}

//-------------------------------------------------------------------------------------
bool MarkerModel::removeAllMarkers()
{
    beginResetModel();

    // detach and delete all markers
    foreach(const auto &setname, m_setnames)
    {
        foreach(auto &item, m_markers[setname])
        {
            item.marker->detach();
            delete item.marker;
        }
    }

    m_setnames.clear();
    m_markers.clear();
    endResetModel();

    return true;
}

//-------------------------------------------------------------------------------------
bool MarkerModel::setVisibility(const QString& setname, bool show)
{
    bool changes = false;
    int setIdx = -1;

    if (setname != "")
    {
        setIdx = m_setnames.indexOf(setname);

        if (setIdx >= 0)
        {
            auto& items = m_markers[setname];

            for (int idx = 0; idx < items.size(); ++idx)
            {
                if (items[idx].visible != show)
                {
                    changes = true;
                    items[idx].visible = show;
                }
            }
        }
    }
    else
    {
        foreach (const QString& sn, m_setnames)
        {
            auto& items = m_markers[sn];

            for (int idx = 0; idx < items.size(); ++idx)
            {
                if (items[idx].visible != show)
                {
                    changes = true;
                    items[idx].visible = show;
                }
            }
        }
    }

    if (changes)
    {
        changeCurrentPlane(m_currentPlaneIdx);
    }

    return setIdx >= 0 || setname == "";
}

//-------------------------------------------------------------------------------------
void MarkerModel::setMarkerLabelsVisible(bool visible)
{
    QwtPlot *plot = nullptr;

    if (m_markerLabelsVisible != visible)
    {
        foreach (const QString& setname, m_setnames)
        {
            QwtText label;

            if (visible)
            {
                label = QString(" %1").arg(setname);
            }

            foreach (auto& item, m_markers[setname])
            {
                item.marker->setLabel(label);
                plot = item.marker->plot();
            }
        }

        m_markerLabelsVisible = visible;

        if (plot)
        {
            plot->replot();
        }
    }
}

//-------------------------------------------------------------------------------------
QString MarkerModel::getNextDefaultSetname() const
{
    QString setname = tr("setname");
    int counter = 2;

    while (m_setnames.contains(setname))
    {
        setname = tr("setname%1").arg(counter++);
    }

    return setname;
}

//-------------------------------------------------------------------------------------
void MarkerModel::changeCurrentPlane(int planeIndex)
{
    bool visible;
    QwtPlot *plot = nullptr;

    foreach (const QString& setname, m_setnames)
    {
        foreach (const MarkerItem& item, m_markers[setname])
        {
            visible = (item.visible && (item.planeIndex == -1 || item.planeIndex == planeIndex));

            if (item.marker->isVisible() != visible)
            {
                item.marker->setVisible(visible);
                plot = item.marker->plot();
            }
        }
    }

    m_currentPlaneIdx = planeIndex;

    if (plot)
    {
        plot->replot();
    }
}

//-------------------------------------------------------------------------------------
Qt::ItemFlags MarkerModel::flags(const QModelIndex& index) const
{
    Qt::ItemFlags flags = QAbstractItemModel::flags(index);

    if (index.isValid() && !index.parent().isValid())
    {
        flags |= Qt::ItemIsUserCheckable;
    }

    return flags;
}

//-------------------------------------------------------------------------------------
QVariant MarkerModel::data(const QModelIndex& index, int role) const
{
    if (index.isValid())
    {
        QModelIndex parent = index.parent();

        if (parent.isValid())
        {
            QString name = m_setnames[parent.row()];
            const auto& markerList = m_markers[name];

            if (index.row() >= 0 && index.row() < markerList.size())
            {
                const auto& marker = markerList[index.row()];



                if (role == Qt::DisplayRole)
                {
                    switch (index.column())
                    {
                    case 0:
                        // name
                        return tr("#%1").arg(index.row());
                    case 1:
                        // X
                        return marker.marker->xValue();
                    case 2:
                        // Y
                        return marker.marker->yValue();
                    case 3:
                        if (marker.planeIndex == -1)
                        {
                            return tr("All");
                        }
                        else
                        {
                            return marker.planeIndex;
                        }
                    }
                }
                else if (role == Qt::DecorationRole && index.column() == 0)
                {
                    return marker.marker->legendIcon(0, QSizeF(8, 8)).toPixmap();
                }
            }
        }
        else
        {
            if (index.row() >= 0 && index.row() < m_setnames.size() && index.column() == 0)
            {
                QString setname = m_setnames[index.row()];

                switch (role)
                {
                case Qt::DisplayRole:
                    return setname;
                    break;
                case Qt::CheckStateRole:
                    if (m_markers[setname].size() > 0)
                    {
                        return m_markers[setname][0].visible ? Qt::Checked : Qt::Unchecked;
                    }
                    return Qt::Unchecked;
                }
            }
        }
    }

    return QVariant();
}

//-------------------------------------------------------------------------------------
bool MarkerModel::setData(
    const QModelIndex& index, const QVariant& value, int role /*= Qt::EditRole*/)
{
    if (index.isValid() && index.column() == 0 && !index.parent().isValid())
    {
        switch (role)
        {
        case Qt::CheckStateRole: {
            setVisibility(m_setnames[index.row()], value.toBool());

            emit dataChanged(index, index);

            return true;
            break;
        }
        }
    }

    return QAbstractItemModel::setData(index, value, role);
}

//-------------------------------------------------------------------------------------
QModelIndex MarkerModel::index(
    int row, int column, const QModelIndex& parent /*= QModelIndex()*/) const
{
    if (parent.isValid())
    {
        int parentRow = parent.row();

        if (parentRow >= 0 && parentRow < m_setnames.size())
        {
            return createIndex(row, column, qHash(m_setnames[parentRow]));
        }
        else
        {
            return QModelIndex();
        }
    }
    else
    {
        return createIndex(row, column);
    }
}

//-------------------------------------------------------------------------------------
QModelIndex MarkerModel::parent(const QModelIndex& index) const
{
    auto id = index.internalId();
    quintptr id2;

    if (id > 0)
    {
        for (int row = 0; row < m_markers.size(); ++row)
        {
            id2 = qHash(m_setnames[row]);

            if (id == id2)
            {
                return createIndex(row, 0);
            }
        }
    }

    return QModelIndex();
}

//-------------------------------------------------------------------------------------
int MarkerModel::rowCount(const QModelIndex& parent /*= QModelIndex()*/) const
{
    if (parent.isValid())
    {
        if (parent.row() < 0 || parent.row() >= m_setnames.size() || parent.internalId() != 0)
        {
            return 0;
        }

        return m_markers[m_setnames[parent.row()]].size();
    }
    else
    {
        return m_markers.size();
    }
}

//-------------------------------------------------------------------------------------
int MarkerModel::columnCount(const QModelIndex& parent /*= QModelIndex()*/) const
{
    return m_headers.size();
}

//-------------------------------------------------------------------------------------
QVariant MarkerModel::headerData(
    int section, Qt::Orientation orientation, int role /*= Qt::DisplayRole*/) const
{
    if (section < 0 || section >= m_headers.size() || role != Qt::DisplayRole)
    {
        return QVariant();
    }
    else
    {
        return m_headers[section];
    }
}

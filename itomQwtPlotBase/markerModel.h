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

#pragma once

#include "qwt_plot_marker.h"

#include <qabstractitemmodel.h>


struct MarkerItem
{
    MarkerItem(QwtPlotMarker* marker, int planeIndex, bool visible) :
        marker(marker), planeIndex(planeIndex), visible(visible)
    {
    }

    //!< the attached marker to the plot. Will be removed
    //!< if the plot is destroyed (automatically).
    QwtPlotMarker* marker;

    //!< -1 if the marker should be shown on all planes,
    //!< else the specific plane index.
    int planeIndex;

    //!< visible is the user-defined visibility,
    //!< independent on the current plane index.
    bool visible;
};


class MarkerModel : public QAbstractItemModel
{
    Q_OBJECT

public:
    MarkerModel(bool markerLabelsVisible, QObject* parent = nullptr);
    ~MarkerModel();

    void addMarkers(const QString& setname, const QList<MarkerItem>& markers);
    bool removeAllMarkersFromSet(const QString& setname);
    bool removeAllMarkers();
    bool setVisibility(const QString& setname, bool show);
    void changeCurrentPlane(int planeIndex);

    QVariant data(const QModelIndex& index, int role) const;
    bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole);
    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const;
    QModelIndex parent(const QModelIndex& index) const;
    int rowCount(const QModelIndex& parent = QModelIndex()) const;
    int columnCount(const QModelIndex& parent = QModelIndex()) const;
    Qt::ItemFlags flags(const QModelIndex& index) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;

    bool markerLabelsVisible() const
    {
        return m_markerLabelsVisible;
    }

    void setMarkerLabelsVisible(bool visible);

    QString getNextDefaultSetname() const;

protected:
private:
    //!< all current markers: marker setname -> list of marker items for this name
    QMap<QString, QList<MarkerItem>> m_markers;

    QList<QString> m_setnames;

    /*!<  string list of names of column headers */
    QList<QString> m_headers;

    /*!<  list of alignments for the corresponding headers */
    QList<QVariant> m_alignment;

    //!< current plane index
    int m_currentPlaneIdx;

    bool m_markerLabelsVisible;

signals:
};

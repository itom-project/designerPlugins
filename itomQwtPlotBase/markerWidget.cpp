/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2021, Institut fuer Technische Optik (ITO),
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

#include "markerWidget.h"

//-------------------------------------------------------------------------------------
MarkerWidget::MarkerWidget(QWidget* parent /*= nullptr*/) :
    QMainWindow(parent), m_pModel(nullptr), m_pToolbar(nullptr), m_pView(nullptr)
{
    m_pView = new QTreeView(this);
    //m_pView->setSortingEnabled(true);

    setCentralWidget(m_pView);

    //m_pToolbar = addToolBar(tr("Marker Toolbar"));
}

//-------------------------------------------------------------------------------------
MarkerWidget::~MarkerWidget()
{
}

//-------------------------------------------------------------------------------------
void MarkerWidget::setModel(QAbstractItemModel* model)
{
    if (m_pModel)
    {
        disconnect(m_pModel, &QAbstractItemModel::rowsInserted, this, &MarkerWidget::rowsInserted);
    }

    m_pView->setModel(model);
    m_pModel = model;
    connect(m_pModel, &QAbstractItemModel::rowsInserted, this, &MarkerWidget::rowsInserted);
    rowsInserted(QModelIndex(), 0, 0);
    m_pView->expandToDepth(1);

    m_pView->setColumnWidth(0, 90);
    m_pView->setColumnWidth(1, 50);
    m_pView->setColumnWidth(2, 50);
    m_pView->setColumnWidth(3, 40);
}

//-------------------------------------------------------------------------------------
void MarkerWidget::rowsInserted(const QModelIndex &parent, int first, int last)
{
    if (!parent.isValid())
    {
        for (int row = 0; row < m_pModel->rowCount(); ++row)
        {
            m_pView->setFirstColumnSpanned(row, QModelIndex(), true);

            if (row >= first && row >= last)
            {
                m_pView->expand(m_pModel->index(row, 0));
            }
        }
    }
}

/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut für Technische Optik (ITO), 
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

#ifndef DATAOBJECTTABLE_H
#define DATAOBJECT_H

#include "DataObject/dataobj.h"

#include <qtableview.h>
#include <qabstractitemmodel.h>
#include <qitemdelegate.h>

class DataObjectModel : public QAbstractItemModel
{
    Q_OBJECT

public:
    DataObjectModel();
    ~DataObjectModel();

    QVariant data(const QModelIndex &index, int role) const;
    bool setData ( const QModelIndex & index, const QVariant & value, int role = Qt::EditRole );

    QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const;
    QModelIndex parent(const QModelIndex &index) const;
    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    int columnCount(const QModelIndex &parent = QModelIndex()) const;
    Qt::ItemFlags flags ( const QModelIndex & index ) const;

    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;

    void setHeaderLabels(Qt::Orientation orientation, const QStringList &labels);

    void setDataObject(ito::DataObject &dataObj);
    inline ito::DataObject getDataObject() const { return m_dataObj; };

    inline int getType() const { return m_dataObj.getType(); }

protected:
    friend class DataObjectTable;

    QVariant at(const int row, const int column) const;
    bool setValue(const int &row, const int &column, const QVariant &value);

    void setReadOnly(bool value);
    inline bool getReadOnly() const { return m_readOnly; };

    void setDefaultGrid(int rows, int cols);

    QStringList m_verticalHeader;
    QStringList m_horizontalHeader;

    int m_defaultRows;
    int m_defaultCols;

    ito::DataObject m_dataObj;



private:
    bool m_readOnly;
};

class DataObjectDelegate : public QItemDelegate
{
    Q_OBJECT

public:
    DataObjectDelegate(QObject *parent = 0);

    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option,  const QModelIndex &index) const;
    void setEditorData(QWidget *editor, const QModelIndex &index) const;
    void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const;
    void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const;

    friend class DataObjectTable;

private:
    double m_min;
    double m_max;
    int m_decimals;

};

class DataObjectTable : public QTableView
{
    Q_OBJECT

    Q_PROPERTY(QSharedPointer<ito::DataObject> data READ getData WRITE setData DESIGNABLE false);
    Q_PROPERTY(bool readOnly READ getReadOnly WRITE setReadOnly DESIGNABLE true);
    Q_PROPERTY(double min READ getMin WRITE setMin DESIGNABLE true);
    Q_PROPERTY(double max READ getMax WRITE setMax DESIGNABLE true);
    Q_PROPERTY(int decimals READ getDecimals WRITE setDecimals DESIGNABLE true);
    Q_PROPERTY(int defaultCols READ getDefaultCols WRITE setDefaultCols DESIGNABLE true);
    Q_PROPERTY(int defaultRows READ getDefaultRows WRITE setDefaultRows DESIGNABLE true);
    Q_PROPERTY(QStringList verticalLabels READ getVerticalLabels WRITE setVerticalLabels DESIGNABLE true);
    Q_PROPERTY(QStringList horizontalLabels READ getHorizontalLabels WRITE setHorizontalLabels DESIGNABLE true);

public:
    DataObjectTable(QWidget *parent = 0);
    ~DataObjectTable();

    void setData(QSharedPointer<ito::DataObject> dataObj);
    QSharedPointer<ito::DataObject> getData() const;

    bool getReadOnly() const;
    void setReadOnly(bool value);

    double getMin() const;
    void setMin(double value);

    double getMax() const;
    void setMax(double value);

    int getDecimals() const;
    void setDecimals(int value);

    inline int getDefaultCols() const { return m_pModel->m_defaultCols; }
    void setDefaultCols(int value);

    inline int getDefaultRows() const { return m_pModel->m_defaultRows; }
    void setDefaultRows(int value);

    inline QStringList getVerticalLabels() const { return m_pModel->m_verticalHeader; }
    void setVerticalLabels(QStringList value);

    inline QStringList getHorizontalLabels() const { return m_pModel->m_horizontalHeader; }
    void setHorizontalLabels(QStringList value);

    virtual QSize sizeHint() const;


protected:
    DataObjectModel *m_pModel;
    DataObjectDelegate *m_pDelegate;

private:

};

#endif
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

#include "dataObjectTable.h"
#include <qspinbox.h>
#include <qheaderview.h>
#include <qscrollbar.h>


DataObjectModel::DataObjectModel() : 
    m_readOnly(false), m_defaultRows(3), m_defaultCols(3)
{
}

DataObjectModel::~DataObjectModel()
{
}

QVariant DataObjectModel::data(const QModelIndex &index, int role) const
{
    if(index.isValid())
    {
        if(role == Qt::DisplayRole || role == Qt::EditRole)
        {
            switch(m_dataObj.getDims())
            {
            case 0:
                return (double)0.0; //default case (for designer, adjustment can be done using the defaultRow and defaultCol property)
            case 1:
                if(index.column() == 0 && index.row() >= 0 && index.row() < (int)m_dataObj.getSize(0))
                {
                    return at(index.row(), index.column());
                }
                return QVariant();
            case 2:
                if(index.column() >= 0 && index.column() < (int)m_dataObj.getSize(1) && index.row() >= 0 && index.row() < (int)m_dataObj.getSize(0))
                {
                    return at(index.row(), index.column());
                }
                return QVariant();
            default:
                return QVariant();
            }
        }
        else
        {
            return QVariant();
        }
    }
    else
    {
        return QVariant();
    }
}

bool DataObjectModel::setData(const QModelIndex & index, const QVariant & value, int role/* = Qt::EditRole */)
{
    if(index.isValid())
    {
        if(role == Qt::EditRole)
        {
            switch(m_dataObj.getDims())
            {
            /*case 0:
                return QVariant();*/
            case 1:
                if(index.column() == 0 && index.row() >= 0 && index.row() < (int)m_dataObj.getSize(0))
                {
                    return setValue(index.row(), index.column(), value);
                }
                return false;
            case 2:
                if(index.column() >= 0 && index.column() < (int)m_dataObj.getSize(1) && index.row() >= 0 && index.row() < (int)m_dataObj.getSize(0))
                {
                    return setValue(index.row(), index.column(), value);
                }
                return false;
            }
        }
    }
    return false;
}



QVariant DataObjectModel::at(const int row, const int column) const
{
    Q_ASSERT(row >= 0);
    Q_ASSERT(column >= 0);
    Q_ASSERT(m_dataObj.getDims() == 1 || m_dataObj.getDims() == 2);
    Q_ASSERT(row < (int)m_dataObj.getSize(0));
    Q_ASSERT(column < (int)m_dataObj.getSize(1));

    switch(m_dataObj.getType())
    {
    case ito::tInt8:
        return (int)m_dataObj.at<ito::int8>(row,column);
    case ito::tUInt8:
        return (uint)m_dataObj.at<ito::uint8>(row,column);
    case ito::tInt16:
        return (int)m_dataObj.at<ito::int16>(row,column);
    case ito::tUInt16:
        return (uint)m_dataObj.at<ito::uint16>(row,column);
    case ito::tInt32:
        return (int)m_dataObj.at<ito::int32>(row,column);
    case ito::tUInt32:
        return (uint)m_dataObj.at<ito::uint32>(row,column);
    case ito::tFloat32:
        return (float)m_dataObj.at<ito::float32>(row,column);
    case ito::tFloat64:
        return (double)m_dataObj.at<ito::float64>(row,column);
    case ito::tComplex64:
        //return (float)m_dataObj.at<ito::complex64>(row,column);
        return QVariant();
    case ito::tComplex128:
        //return (double)m_dataObj.at<ito::complex128>(row,column);
        return QVariant();
    }
    return QVariant();
}

bool DataObjectModel::setValue(const int &row, const int &column, const QVariant &value)
{
    Q_ASSERT(row >= 0);
    Q_ASSERT(column >= 0);
    Q_ASSERT(m_dataObj.getDims() == 1 || m_dataObj.getDims() == 2);
    Q_ASSERT(row < (int)m_dataObj.getSize(0));
    Q_ASSERT(column < (int)m_dataObj.getSize(1));

    QModelIndex i = createIndex(row, column);
    bool ok = false;

    switch(m_dataObj.getType())
    {
    case ito::tInt8:
        {
            int val = value.toInt(&ok);
            if(!ok) return false;
            m_dataObj.at<ito::int8>(row,column) = cv::saturate_cast<ito::int8>(val);
            emit dataChanged( i,i );
            return true;
        }
    case ito::tUInt8:
        {
            uint val = value.toUInt(&ok);
            if(!ok) return false;
            m_dataObj.at<ito::uint8>(row,column) = cv::saturate_cast<ito::uint8>(val);
            emit dataChanged( i,i );
            return true;
        }
    case ito::tInt16:
        {
            int val = value.toInt(&ok);
            if(!ok) return false;
            m_dataObj.at<ito::int16>(row,column) = cv::saturate_cast<ito::int16>(val);
            emit dataChanged( i,i );
            return true;
        }
    case ito::tUInt16:
        {
            uint val = value.toUInt(&ok);
            if(!ok) return false;
            m_dataObj.at<ito::uint16>(row,column) = cv::saturate_cast<ito::uint16>(val);
            emit dataChanged( i,i );
            return true;
        }
    case ito::tInt32:
        {
            int val = value.toInt(&ok);
            if(!ok) return false;
            m_dataObj.at<ito::uint32>(row,column) = cv::saturate_cast<ito::int32>(val);
            emit dataChanged( i,i );
            return true;
        }
    case ito::tUInt32:
        {
            uint val = value.toUInt(&ok);
            if(!ok) return false;
            m_dataObj.at<ito::uint32>(row,column) = cv::saturate_cast<ito::uint32>(val);
            emit dataChanged( i,i );
            return true;
        }
    case ito::tFloat32:
        {
            double val = value.toDouble(&ok);
            if(!ok) return false;
            m_dataObj.at<ito::float32>(row,column) = cv::saturate_cast<ito::float32>(val);
            emit dataChanged( i,i );
            return true;
        }
    case ito::tFloat64:
        {
            double val = value.toDouble(&ok);
            if(!ok) return false;
            m_dataObj.at<ito::float64>(row,column) = cv::saturate_cast<ito::float64>(val);
            emit dataChanged( i,i );
            return true;
        }
    case ito::tComplex64:
        //return (float)m_dataObj.at<ito::complex64>(row,column);
        return false;
    case ito::tComplex128:
        //return (double)m_dataObj.at<ito::complex128>(row,column);
        return false;
    }
    return false;
}

QModelIndex DataObjectModel::index(int row, int column, const QModelIndex &parent) const
{
    if(parent.isValid() == false)
    {
        return createIndex(row,column);
    }
    return QModelIndex();
}

QModelIndex DataObjectModel::parent(const QModelIndex &/*index*/) const
{
    return QModelIndex();
}

int DataObjectModel::rowCount(const QModelIndex &parent) const
{
    if(parent.isValid() == false && m_dataObj.getDims() > 0)
    {
        return m_dataObj.getSize(0);
    }
    else if(parent.isValid() == false) //default case
    {
        return m_defaultRows;
    }
    return 0;
}

int DataObjectModel::columnCount(const QModelIndex &parent) const
{
    if(parent.isValid() == false)
    {
        if(m_dataObj.getDims() > 1)
        {
            return m_dataObj.getSize(1);
        }
        else if(m_dataObj.getDims() == 0) //default case
        {
            return m_defaultCols;
        }
        return 1;
    }
    return 0;
}


QVariant DataObjectModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole)
    {
        if(orientation == Qt::Horizontal)
        {
            switch(m_dataObj.getDims())
            {
            case 0:
                if(m_horizontalHeader.count() > section) //default case
                {
                    return m_horizontalHeader[section];
                }
                else
                {
                    return section;
                }
            case 1:
                return 0;
            default:
                {
                    if(section >= 0 && section < (int)m_dataObj.getSize(1))
                    {
                        if(m_horizontalHeader.count() > section)
                        {
                            return m_horizontalHeader[section];
                        }
                        else
                        {
                            return section;
                        }
                    }
                    return QVariant();
                }
            }
        }
        else //vertical
        {
            switch(m_dataObj.getDims())
            {
            case 0:
                if(m_verticalHeader.count() > section) //default case
                {
                    return m_verticalHeader[section];
                }
                else
                {
                    return section;
                }
            case 1:
                return 1;
            default:
                {
                    if(section >= 0 && section < (int)m_dataObj.getSize(0))
                    {
                        if(m_verticalHeader.count() > section)
                        {
                            return m_verticalHeader[section];
                        }
                        else
                        {
                            return section;
                        }
                    }
                    return QVariant();
                }
            }
        }
    }

    return QVariant();

}

void DataObjectModel::setHeaderLabels(Qt::Orientation orientation, const QStringList &labels)
{
    beginResetModel();
    if(orientation == Qt::Horizontal)
    {
        m_horizontalHeader = labels;
    }
    else
    {
        m_verticalHeader = labels;
    }
    endResetModel();
    emit headerDataChanged (orientation, 0, labels.count() - 1);
}

Qt::ItemFlags DataObjectModel::flags ( const QModelIndex & index ) const
{
    if(m_readOnly || m_dataObj.getDims() == 0)
    {
        return Qt::ItemIsSelectable | Qt::ItemIsEnabled;
    }
    return Qt::ItemIsEditable | Qt::ItemIsSelectable | Qt::ItemIsEnabled;
}


void DataObjectModel::setDataObject(ito::DataObject &dataObj)
{
    beginResetModel();
    m_dataObj = dataObj;
    endResetModel();
}

void DataObjectModel::setReadOnly(bool value)
{
    beginResetModel();
    m_readOnly = value;
    endResetModel();
}

void DataObjectModel::setDefaultGrid(int rows, int cols)
{
    if(m_defaultRows != rows || m_defaultCols != cols)
    {
        beginResetModel();
        m_defaultRows = rows;
        m_defaultCols = cols;
        endResetModel();
    }
}





//------------------------------------------------------------------------------------------------------
DataObjectDelegate::DataObjectDelegate(QObject *parent /*= 0*/)
    : QItemDelegate(parent), 
    m_min(-std::numeric_limits<double>::max()), 
    m_max(std::numeric_limits<double>::max()),
    m_decimals(2)
{
}

QWidget *DataObjectDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &/*option*/,  const QModelIndex &index) const
{
    const DataObjectModel *model = qobject_cast<const DataObjectModel*>(index.model());
    int type = model->getType();

    QWidget *result = NULL;

    //this is a workaround, since the saturate_cast from double::max() to limits of int8 for instance if malignious.
    int intMin = m_min < (double)(std::numeric_limits<int>::min()) ? std::numeric_limits<int>::min() : m_min;
    int intMax = m_max > (double)(std::numeric_limits<int>::max()) ? std::numeric_limits<int>::max() : m_max;

    switch(type)
    {
    case ito::tInt8:
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setMinimum( std::max( std::numeric_limits<ito::int8>::min(), cv::saturate_cast<ito::int8>(intMin) ) );
            editor->setMaximum( std::min( std::numeric_limits<ito::int8>::max(), cv::saturate_cast<ito::int8>(intMax) ) );
            result = editor;
        }
        break;
    case ito::tUInt8:
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setMinimum( std::max( std::numeric_limits<ito::uint8>::min(), cv::saturate_cast<ito::uint8>(intMin) ) );
            editor->setMaximum( std::min( std::numeric_limits<ito::uint8>::max(), cv::saturate_cast<ito::uint8>(intMax) ) );
            result = editor;
        }
        break;
    case ito::tInt16:
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setMinimum( std::max( std::numeric_limits<ito::int16>::min(), cv::saturate_cast<ito::int16>(intMin) ) );
            editor->setMaximum( std::min( std::numeric_limits<ito::int16>::max(), cv::saturate_cast<ito::int16>(intMax) ) );
            result = editor;
        }
        break;
    case ito::tUInt16:
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setMinimum( std::max( std::numeric_limits<ito::uint16>::min(), cv::saturate_cast<ito::uint16>(intMin) ) );
            editor->setMaximum( std::min( std::numeric_limits<ito::uint16>::max(), cv::saturate_cast<ito::uint16>(intMax) ) );
            result = editor;
        }
        break;
    case ito::tInt32:
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setMinimum( std::max( std::numeric_limits<ito::int32>::min(), cv::saturate_cast<ito::int32>(intMin) ) );
            editor->setMaximum( std::min( std::numeric_limits<ito::int32>::max(), cv::saturate_cast<ito::int32>(intMax) ) );
            result = editor;
        }
        break;
    case ito::tUInt32:
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setMinimum( std::max( std::numeric_limits<ito::uint32>::min(), cv::saturate_cast<ito::uint32>(intMin) ) );
            editor->setMaximum( std::min( std::numeric_limits<ito::uint32>::max(), cv::saturate_cast<ito::uint32>(intMax) ) );
            result = editor;
        }
        break;
    case ito::tFloat32:
        {
            QDoubleSpinBox *editor = new QDoubleSpinBox(parent);
            editor->setDecimals(m_decimals);
            editor->setMinimum( std::max( -std::numeric_limits<ito::float32>::max(), cv::saturate_cast<ito::float32>(m_min) ) );
            editor->setMaximum( std::min( std::numeric_limits<ito::float32>::max(), cv::saturate_cast<ito::float32>(m_max) ) );
            result = editor;
        }
        break;
    case ito::tFloat64:
        {
            QDoubleSpinBox *editor = new QDoubleSpinBox(parent);
            editor->setDecimals(m_decimals);
            editor->setMinimum( std::max( -std::numeric_limits<ito::float64>::max(), cv::saturate_cast<ito::float64>(m_min) ) );
            editor->setMaximum( std::min( std::numeric_limits<ito::float64>::max(), cv::saturate_cast<ito::float64>(m_max) ) );
            result = editor;
        }
        break;
    case ito::tComplex64:
        {
            
        }
        break;
    case ito::tComplex128:
        {
            
        }
        break;
    }

    return result;
    
}

void DataObjectDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
    const DataObjectModel *model = qobject_cast<const DataObjectModel*>(index.model());
    int type = model->getType();

    QWidget *result = NULL;

    switch(type)
    {
    case ito::tInt8:
    case ito::tInt16:
    case ito::tInt32:
        {
            QSpinBox *spinBox = static_cast<QSpinBox*>(editor);
            int value = model->data(index, Qt::EditRole).toInt();
            spinBox->setValue(value);
        }
        break;

    case ito::tUInt8:
    case ito::tUInt16:
    case ito::tUInt32:
        {
            QSpinBox *spinBox = static_cast<QSpinBox*>(editor);
            uint value = model->data(index, Qt::EditRole).toUInt();
            spinBox->setValue(value);
        }
        break;

    case ito::tFloat32:
    case ito::tFloat64:
        {
            QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox*>(editor);
            double value = model->data(index, Qt::EditRole).toDouble();
            spinBox->setValue(value);
        }
        break;
    case ito::tComplex64:
        {
            
        }
        break;
    case ito::tComplex128:
        {
            
        }
        break;
    }
}

void DataObjectDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
    const DataObjectModel *model2 = qobject_cast<DataObjectModel*>(model);
    int type = model2->getType();

    QWidget *result = NULL;

    switch(type)
    {
    case ito::tInt8:
    case ito::tInt16:
    case ito::tInt32:
        {
            QSpinBox *spinBox = static_cast<QSpinBox*>(editor);
            int value = model->setData(index, spinBox->value(), Qt::EditRole); 
            spinBox->setValue(value);
        }
        break;

    case ito::tUInt8:
    case ito::tUInt16:
    case ito::tUInt32:
        {
            QSpinBox *spinBox = static_cast<QSpinBox*>(editor);
            int value = model->setData(index, spinBox->value(), Qt::EditRole); 
            spinBox->setValue(value);
        }
        break;

    case ito::tFloat32:
    case ito::tFloat64:
        {
            QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox*>(editor);
            int value = model->setData(index, spinBox->value(), Qt::EditRole); 
            spinBox->setValue(value);
        }
        break;
    case ito::tComplex64:
        {
            
        }
        break;
    case ito::tComplex128:
        {
            
        }
        break;
    }
}

void DataObjectDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    editor->setGeometry(option.rect);
}


//-----------------------------------------------------------------------------------------------
DataObjectTable::DataObjectTable(QWidget *parent /*= 0*/)
    : QTableView(parent)
{
    m_pModel = new DataObjectModel();
    m_pDelegate = new DataObjectDelegate(this);

    setModel(m_pModel);
    setItemDelegate(m_pDelegate);

    setEditTriggers( QAbstractItemView::AnyKeyPressed | QAbstractItemView::DoubleClicked );
}

DataObjectTable::~DataObjectTable()
{
    m_pDelegate->deleteLater();
    m_pModel->deleteLater();
}

void DataObjectTable::setData(QSharedPointer<ito::DataObject> dataObj)
{
    m_pModel->setDataObject( *dataObj );
}

QSharedPointer<ito::DataObject> DataObjectTable::getData() const
{
    return QSharedPointer<ito::DataObject>( new ito::DataObject( m_pModel->getDataObject() ) );
}

bool DataObjectTable::getReadOnly() const
{
    return m_pModel->getReadOnly();
}

void DataObjectTable::setReadOnly(bool value)
{
    m_pModel->setReadOnly(value);
}


double DataObjectTable::getMin() const
{
    return m_pDelegate->m_min;
}

void DataObjectTable::setMin(double value)
{
    m_pDelegate->m_min = value;
}


double DataObjectTable::getMax() const
{
    return m_pDelegate->m_max;
}

void DataObjectTable::setMax(double value)
{
    m_pDelegate->m_max = value;
}

int DataObjectTable::getDecimals() const
{
    return m_pDelegate->m_decimals;
}

void DataObjectTable::setDecimals(int value)
{
    m_pDelegate->m_decimals = value;
}

void DataObjectTable::setHorizontalLabels(QStringList value)
{
    m_pModel->setHeaderLabels(Qt::Horizontal, value);
    horizontalHeader()->repaint();
}

void DataObjectTable::setVerticalLabels(QStringList value)
{
    m_pModel->setHeaderLabels(Qt::Vertical, value);
    verticalHeader()->repaint();
}

void DataObjectTable::setDefaultCols(int value)
{
    m_pModel->setDefaultGrid(m_pModel->m_defaultRows, value);
}

void DataObjectTable::setDefaultRows(int value)
{
    m_pModel->setDefaultGrid(value, m_pModel->m_defaultCols);
}

QSize DataObjectTable::sizeHint() const
{
    QHeaderView *hHeader = horizontalHeader();
    QHeaderView *vHeader = verticalHeader();

    /*QScrollBar *hScrollBar = horizontalScrollBar();
    QScrollBar *vScrollBar = verticalScrollBar();*/

    int h = 25;
    int w = 15;
    h += m_pModel->m_defaultRows * vHeader->defaultSectionSize();
    w += m_pModel->m_defaultCols * hHeader->defaultSectionSize();

    if(vHeader->isVisible())
    {
        w += vHeader->sizeHint().width();
    }
    if(hHeader->isVisible())
    {
        h += hHeader->sizeHint().height();
    }

    //if(vScrollBar->isVisible())
    //{
    //    w += vScrollBar->sizeHint().width();
    //}
    //if(hScrollBar->isVisible())
    //{
    //    h += hScrollBar->sizeHint().height();
    //}
    
    return QSize(w,h);
    
}


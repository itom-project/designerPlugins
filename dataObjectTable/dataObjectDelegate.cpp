/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2015, Institut für Technische Optik (ITO), 
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

#include "dataObjectDelegate.h"
#include "dataObjectModel.h"

#include <qspinbox.h>
#include <qitemdelegate.h>

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectDelegate::DataObjectDelegate(QObject *parent /*= 0*/)
    : QItemDelegate(parent), 
    m_min(-std::numeric_limits<double>::max()), 
    m_max(std::numeric_limits<double>::max()),
    m_editorDecimals(3)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectDelegate::~DataObjectDelegate()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
QWidget *DataObjectDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &/*option*/,  const QModelIndex &index) const
{
    const DataObjectModel *model = qobject_cast<const DataObjectModel*>(index.model());
    int type = model->getType();

    QWidget *result = NULL;

    //this is a workaround, since the saturate_cast from double::max() to limits of int8 for instance if malignious.
    int intMin = m_min < (double)(std::numeric_limits<int>::min()) ? std::numeric_limits<int>::min() : m_min;
    int intMax = m_max > (double)(std::numeric_limits<int>::max()) ? std::numeric_limits<int>::max() : m_max;

    QString suffix;
    if (m_suffixes.size() > 0)
    {
        suffix = m_suffixes[std::min(m_suffixes.size()-1, index.column())];
    }

    switch(type)
    {
    case ito::tInt8:
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setSuffix(suffix);
            editor->setMinimum(std::max(std::numeric_limits<ito::int8>::min(), cv::saturate_cast<ito::int8>(intMin)));
            editor->setMaximum(std::min(std::numeric_limits<ito::int8>::max(), cv::saturate_cast<ito::int8>(intMax)));
            result = editor;
        }
        break;
    case ito::tUInt8:
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setSuffix(suffix);
            editor->setMinimum(std::max(std::numeric_limits<ito::uint8>::min(), cv::saturate_cast<ito::uint8>(intMin)));
            editor->setMaximum(std::min(std::numeric_limits<ito::uint8>::max(), cv::saturate_cast<ito::uint8>(intMax)));
            result = editor;
        }
        break;
    case ito::tInt16:
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setSuffix(suffix);
            editor->setMinimum(std::max(std::numeric_limits<ito::int16>::min(), cv::saturate_cast<ito::int16>(intMin)));
            editor->setMaximum(std::min(std::numeric_limits<ito::int16>::max(), cv::saturate_cast<ito::int16>(intMax)));
            result = editor;
        }
        break;
    case ito::tUInt16:
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setSuffix(suffix);
            editor->setMinimum(std::max(std::numeric_limits<ito::uint16>::min(), cv::saturate_cast<ito::uint16>(intMin)));
            editor->setMaximum(std::min(std::numeric_limits<ito::uint16>::max(), cv::saturate_cast<ito::uint16>(intMax)));
            result = editor;
        }
        break;
    case ito::tInt32:
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setSuffix(suffix);
            editor->setMinimum(std::max(std::numeric_limits<ito::int32>::min(), cv::saturate_cast<ito::int32>(intMin)));
            editor->setMaximum(std::min(std::numeric_limits<ito::int32>::max(), cv::saturate_cast<ito::int32>(intMax)));
            result = editor;
        }
        break;
    case ito::tUInt32:
        {
            QSpinBox *editor = new QSpinBox(parent);
            editor->setSuffix(suffix);
            editor->setMinimum(std::max(std::numeric_limits<ito::uint32>::min(), cv::saturate_cast<ito::uint32>(intMin)));
            editor->setMaximum(std::min(std::numeric_limits<ito::uint32>::max(), cv::saturate_cast<ito::uint32>(intMax)));
            result = editor;
        }
        break;
    case ito::tFloat32:
        {
            QDoubleSpinBox *editor = new QDoubleSpinBox(parent);
            editor->setSuffix(suffix);
            editor->setDecimals(m_editorDecimals);
            editor->setMinimum(std::max(-std::numeric_limits<ito::float32>::max(), cv::saturate_cast<ito::float32>(m_min)));
            editor->setMaximum(std::min(std::numeric_limits<ito::float32>::max(), cv::saturate_cast<ito::float32>(m_max)));
            result = editor;
        }
        break;
    case ito::tFloat64:
        {
            QDoubleSpinBox *editor = new QDoubleSpinBox(parent);
            editor->setSuffix(suffix);
            editor->setDecimals(m_editorDecimals);
            editor->setMinimum(std::max(-std::numeric_limits<ito::float64>::max(), cv::saturate_cast<ito::float64>(m_min)));
            editor->setMaximum(std::min(std::numeric_limits<ito::float64>::max(), cv::saturate_cast<ito::float64>(m_max)));
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
    case ito::tRGBA32:
        {
        }
        break;
    }

    return result;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
    const DataObjectModel *model = qobject_cast<const DataObjectModel*>(index.model());
    int type = model->getType();

//    QWidget *result = NULL;

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
    case ito::tRGBA32:
        {
        }
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
    const DataObjectModel *model2 = qobject_cast<DataObjectModel*>(model);
    int type = model2->getType();

//    QWidget *result = NULL;

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

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    editor->setGeometry(option.rect);
}

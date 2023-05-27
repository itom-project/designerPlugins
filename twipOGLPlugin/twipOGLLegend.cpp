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
#    itom is free software by ITO, University Stuttgart published under
#    GNU General Public License as published by the Free Software
#    Foundation. See <https://github.com/itom-project/itom>
#
#    You should have received a copy of the GNU Library General Public License
#    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */


#include "twipOGLLegend.h"
#include <QString>
#include <QTreeWidgetItem>
#include <qcoreapplication.h>

#include "twipOGLFigure.h"

TwipLegend::TwipLegend(QWidget* parent, QWidget* observedObject) : QTreeWidget(parent)
{
    m_observedObject = observedObject;
    m_isUpdating = true;
    setAlternatingRowColors(true);
    setColumnCount(2);
    m_isUpdating = false;

    bool connected = connect(this, SIGNAL(itemChanged(QTreeWidgetItem*,int)), this, SLOT(itemChangedDone(QTreeWidgetItem*,int)));
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal TwipLegend::addEntry(const int index, const int type, const int subtype, const ito::uint8 alpha, const bool enabled)
{
    ito::RetVal retVal = ito::retOk;
    m_isUpdating = true;
    if(index < 0)
    {
        m_isUpdating = false;
        return ito::RetVal(ito::retError, 0, "Index out of range");
    }

    int rowToFill = -1;
    bool makeNew = true;

    for(int row = 0; row < topLevelItemCount(); row++)
    {
        if(topLevelItem(row)->text(0).toInt() == index)
        {
            rowToFill = row;
            makeNew = false;
            break;
        }
    }

    if(makeNew)
    {
        rowToFill = topLevelItemCount();
        QTreeWidgetItem* newRowItem = new QTreeWidgetItem(this);
        newRowItem->setFlags(newRowItem->flags() | Qt::ItemIsUserCheckable);
        QTreeWidgetItem* newChild = new QTreeWidgetItem(newRowItem);
        newChild->setText(0, "type");
        newRowItem->addChild(newChild);

        newChild = new QTreeWidgetItem(newRowItem);
        newChild->setFlags(newRowItem->flags() | Qt::ItemIsEditable);
        newChild->setText(0, "alpha");
        newRowItem->addChild(newChild);

        addTopLevelItem(newRowItem);
    }

    retVal += modifiyObject(rowToFill, index, type, subtype, alpha, enabled);
//    QCoreApplication::processEvents();
    m_isUpdating = false;
    return retVal;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal TwipLegend::modifiyObject(const int row, const int index, const  int type, const int subtype, const ito::uint8 alpha, const bool enabled)
{
    ito::RetVal retVal = ito::retOk;
    QTreeWidgetItem* curRowItem = topLevelItem(row);
    QString typeString = "";

    curRowItem->setCheckState(0, enabled ? Qt::Checked : Qt::Unchecked);
    switch(type)
    {
        case tDataObject:
            typeString = "DataObject";
            switch(subtype)
            {
                case ito::tInt8:
                    typeString.append(" (int8)");
                    break;
                case ito::tUInt8:
                    typeString.append(" (uint8)");
                    break;
                case ito::tInt16:
                    typeString.append(" (int16)");
                    break;
                case ito::tUInt16:
                    typeString.append(" (uint16)");
                    break;
                case ito::tInt32:
                    typeString.append(" (int32)");
                    break;
                case ito::tUInt32:
                    typeString.append(" (uint32)");
                    break;
                case ito::tFloat32:
                    typeString.append(" (float32)");
                    break;
                case ito::tFloat64:
                    typeString.append(" (float64)");
                    break;
                case ito::tComplex64:
                    typeString.append(" (complex64)");
                    break;
                case ito::tComplex128:
                    typeString.append(" (complex128)");
                    break;
                case ito::tRGBA32:
                    typeString.append(" (RGBA32)");
                    break;
                default:
                    retVal += ito::RetVal(ito::retError, 0, "Sub-Type not implemented yet");
                    typeString.append(" (N.A.)");
                    break;
            }
        break;
        case tPointCloud:
            typeString = "PointCloud";
            switch(subtype)
            {
                case ito::pclInvalid:
                    typeString.append(" (invalid)");
                    break;
                case ito::pclXYZ:
                    typeString.append(" (XYZ)");
                    break;
                case ito::pclXYZI:
                    typeString.append(" (XYZI)");
                    break;
                case ito::pclXYZRGBA:
                    typeString.append(" (XYZRGB)");
                    break;
                case ito::pclXYZNormal:
                    typeString.append(" (XYZN)");
                    break;
                case ito::pclXYZINormal:
                    typeString.append(" (XYZIN)");
                    break;
                case ito::pclXYZRGBNormal:
                    typeString.append(" (XYZRGBN)");
                    break;
                default:
                    retVal += ito::RetVal(ito::retError, 0, "Sub-Type not implemented yet");
                    typeString.append(" (N.A.)");
                break;
            }
        break;
        default:
            retVal += ito::RetVal(ito::retError, 0, "Type not implemented yet");
            typeString.append("Not implemented");
            break;
    }

    curRowItem->setText(0, QString::number(index));
    curRowItem->child(0)->setText(1, typeString);
    curRowItem->child(1)->setText(1, QString::number(alpha));
    //curRowItem->setText(3, enabled ? "visible" : "not visible");

    return retVal;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal TwipLegend::toggleState(const int index, const bool enabled)
{
    ito::RetVal retVal = ito::retOk;
    m_isUpdating = true;
    if(index < -1)
    {
        m_isUpdating = false;
        return ito::RetVal(ito::retError, 0, "Index out of range");
    }
    else if(index == -1)
    {
        for(int row = 0; row < topLevelItemCount(); row++)
        {
            //topLevelItem(row)->setText(3, enabled ? "visible" : "not visible");
            topLevelItem(row)->setCheckState(0, enabled ? Qt::Checked : Qt::Unchecked);
        }
    }
    else
    {
        bool found = false;

        for(int row = 0; row < topLevelItemCount(); row++)
        {
            if(topLevelItem(row)->text(0).toInt() == index)
            {
                //topLevelItem(row)->setText(3, enabled ? "visible" : "not visible");
                topLevelItem(row)->setCheckState(0, enabled ? Qt::Checked : Qt::Unchecked);
                found = true;
                break;
            }
        }
        if(!found)
        {
            retVal += ito::RetVal(ito::retError, 0, "Index not found");
        }
    }
//    QCoreApplication::processEvents();
    m_isUpdating = false;
    return retVal;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal TwipLegend::changeAlpha(const int index, const ito::uint8 alpha)
{
    m_isUpdating = true;
    ito::RetVal retVal = ito::retOk;

    if(index < -1)
    {
        return ito::RetVal(ito::retError, 0, "Index out of range");
    }
    else if(index == -1)
    {
        for(int row = 0; row < topLevelItemCount(); row++)
        {
            topLevelItem(row)->child(1)->setText(1, QString::number(alpha));
        }
    }
    else
    {
        bool found = false;

        for(int row = 0; row < topLevelItemCount(); row++)
        {
            if(topLevelItem(row)->text(0).toInt() == index)
            {
                topLevelItem(row)->child(1)->setText(1, QString::number(alpha));
                found = true;
                break;
            }
        }
        if(!found)
        {
            retVal += ito::RetVal(ito::retError, 0, "Index not found");
        }
    }
//    QCoreApplication::processEvents();
    m_isUpdating = false;
    return retVal;
}
//----------------------------------------------------------------------------------------------------------------------------------
void TwipLegend::itemChangedDone(QTreeWidgetItem * item, int column)
{
    if(m_isUpdating)
    {
        return;
    }

    if(column == 0 && item->flags() & Qt::ItemIsUserCheckable)  // Must be a first row item
    {
        bool ok;
        int idx = item->text(0).toInt(&ok);
        if(m_observedObject && ok)
        {
            ((TwipOGLFigure*)m_observedObject)->toggleVisibility(idx, item->checkState(0) == Qt::Checked);
        }
    }

    if(column == 1 && item->flags() & Qt::ItemIsEditable)  // Must be a first row item
    {
        bool ok;
        bool ok2;
        int idx = item->parent()->text(0).toInt(&ok);
        int alpha = item->text(1).toInt(&ok2);
        alpha = cv::saturate_cast<ito::uint8>(alpha);
        if(m_observedObject && ok)
        {
            if(item->text(0) == "alpha") ((TwipOGLFigure*)m_observedObject)->setPlaneAlpha(idx, alpha);
            //else if(item->text(0) == "type") ((TwipOGLFigure*)m_observedObject)->toogleVisibility(idx, item->checkState(0) == Qt::Checked);
        }
    }

}

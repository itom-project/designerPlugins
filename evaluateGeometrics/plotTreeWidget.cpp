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

#include "evaluateGeometrics.h"
#include "plotTreeWidget.h"
#include "common/../DataObject/dataObjectFuncs.h"
#include "common/sharedStructuresGraphics.h"
#include "common/apiFunctionsGraphInc.h"

#include <qdebug.h>
#include <qmessagebox.h>
#include <QDoubleSpinBox>
#include <qlayout.h>
#include <qfile.h>
#include <QXmlStreamWriter>
#include <QCoreApplication>

#define GEO_PI 3.14159265358979323846

double PlotTreeWidget::quietNaN = std::numeric_limits<double>::quiet_NaN();

//----------------------------------------------------------------------------------------------------------------------------------
PlotTreeWidget::PlotTreeWidget(QMenu *contextMenu, InternalInfo *data, QWidget * parent) :
    QTreeWidget(parent),
    m_contextMenu(contextMenu),
    m_pParent(parent),
    m_state(stateIdle),
    m_lastRetVal(ito::retOk)
{
    m_pData = data;
    //this is the border between the canvas and the axes and the overall mainwindow
    setContentsMargins(2,2,2,2);

    setColumnCount(5);

    setEditTriggers(QAbstractItemView::NoEditTriggers);
    
    m_rowHash.clear();
    //m_rowHash.reserve(24);
    //m_pData->m_rowHash.clear();
    //m_pData->m_rowHash.reserve(24);

    setColumnCount(5);
    setColumnWidth(0, 142);
    setColumnWidth(1, 72);
    setColumnWidth(2, 72);
    setColumnWidth(3, 48);
    setColumnWidth(4, 48);
    
    setIconSize(QSize(24, 24));
}

//----------------------------------------------------------------------------------------------------------------------------------
PlotTreeWidget::~PlotTreeWidget()
{

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTreeWidget::init()
{
    QFont titleFont = apiGetFigureSetting(parent(), "titleFont", QFont("Helvetica",12),NULL).value<QFont>();
    QFont labelFont = apiGetFigureSetting(parent(), "labelFont", QFont("Helvetica",12),NULL).value<QFont>();
    QFont axisFont = apiGetFigureSetting(parent(), "axisFont", QFont("Helvetica",10),NULL).value<QFont>();
    
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::setPrimitivElement(const int row, const bool update, ito::float32 *val)
{
    //QLabel** elements = (QLabel**)calloc(5, sizeof(QLabel*));

    ito::uint16 type = (ito::uint16)(((ito::uint32)(val[1])) & 0x0000FFFF);

    QString coordsString("[%1, %2, %3]");

    if (m_pData->m_consider2DOnly)
    {
        coordsString = "[%1, %2]";
    }

    if (!update)
    {
        if (m_pData->m_primitivNames.contains(type))
        {
            topLevelItem(row)->setText(0, tr("%1 %2").arg(m_pData->m_primitivNames[type]).arg(QString::number((ito::uint32)(val[0]))));
        }
        else
        {
            topLevelItem(row)->setText(0, tr("notype %1").arg(QString::number((ito::uint32)(val[0]))));
        }
        
        switch (type)
        {
            default:
            case ito::PrimitiveContainer::tNoType:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/notype.png"));
                break;
            }
            case ito::PrimitiveContainer::tPoint:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/marker.png"));
                break;
            }
            case ito::PrimitiveContainer::tLine:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"));
                break;
            }
            case ito::PrimitiveContainer::tCircle:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/circle.png"));
                break;
            }
            case ito::PrimitiveContainer::tEllipse:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/ellipse.png"));
                break;
            }
            case ito::PrimitiveContainer::tRectangle:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/rectangle.png"));
                break;
            }
            case ito::PrimitiveContainer::tSquare:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/square.png"));
                break;
            }
            case ito::PrimitiveContainer::tPolygon:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/polygon.png"));
                break;
            }
        }
    }

    switch (type)
    {
        default:
        case ito::PrimitiveContainer::tNoType:
            break;

        case ito::PrimitiveContainer::tPoint:
        {
            //elements[1]->setText(QString(coordsString).arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString(coordsString)
                                        .arg(QString::number(val[2], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[3], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[4], 'f', m_pData->m_numberOfDigits)));

            break;
        }
        case ito::PrimitiveContainer::tLine:
        {
            //elements[1]->setText(QString(coordsString).arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString(coordsString)
                                        .arg(QString::number(val[2], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[3], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[4], 'f', m_pData->m_numberOfDigits)));

            //elements[2]->setText(QString(coordsString).arg(QString::number(val[5])).arg(QString::number(val[6])).arg(QString::number(val[7])));
            topLevelItem(row)->setText(2, QString(coordsString)
                                        .arg(QString::number(val[5], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[6], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[7], 'f', m_pData->m_numberOfDigits)));

            break;
        }
        case ito::PrimitiveContainer::tCircle:
        {
            //elements[1]->setText(QString(coordsString).arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString(coordsString)
                                        .arg(QString::number(val[2], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[3], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[4], 'f', m_pData->m_numberOfDigits)));

            //elements[2]->setText(QString("r = %1 %2").arg(QString::number(val[5])).arg(m_pData->m_valueUnit));
            topLevelItem(row)->setText(2, QString("r = %1 %2")
                                        .arg(QString::number(val[5], 'f', m_pData->m_numberOfDigits))
                                        .arg(m_pData->m_valueUnit));

            break;
        }
        case ito::PrimitiveContainer::tEllipse:
        {
            //elements[1]->setText(QString(coordsString).arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString(coordsString)
                                        .arg(QString::number(val[2], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[3], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[4], 'f', m_pData->m_numberOfDigits)));

            //elements[2]->setText(QString("a,b = %1 %2 in %3").arg(QString::number(val[5])).arg(QString::number(val[6])).arg(m_pData->m_valueUnit));
            topLevelItem(row)->setText(2, QString("a,b = %1 %3, %2 %3")
                                        .arg(QString::number(val[5], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[6], 'f', m_pData->m_numberOfDigits))
                                        .arg(m_pData->m_valueUnit));

            if (ito::dObjHelper::isFinite(val[7]))
            {
                //elements[3]->setText(QString("alpha = %1%2").arg(QString::number(val[7])).arg(QChar((uchar)248)));
                topLevelItem(row)->setText(3, QString("alpha = %1%2")
                                            .arg(QString::number(val[7], 'f', m_pData->m_numberOfDigits))
                                            .arg(QChar(0x00B0)));
            }
            else
            {
                //elements[3]->setText("");
                 topLevelItem(row)->setText(3, "");
            }
            break;
        }
        case ito::PrimitiveContainer::tRectangle:
        {
            //elements[1]->setText(QString(coordsString).arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString(coordsString)
                                        .arg(QString::number(val[2], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[3], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[4], 'f', m_pData->m_numberOfDigits)));

            //elements[2]->setText(QString(coordsString).arg(QString::number(val[5])).arg(QString::number(val[6])).arg(QString::number(val[7])));
            topLevelItem(row)->setText(2, QString(coordsString)
                                        .arg(QString::number(val[5], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[6], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[7], 'f', m_pData->m_numberOfDigits)));

            if (ito::dObjHelper::isFinite(val[8]))
            {
                //elements[3]->setText(QString("alpha = %1%2").arg(QString::number(val[8])).arg(QChar((uchar)248)));
                 topLevelItem(row)->setText(3, QString("alpha = %1%2")
                                             .arg(QString::number(val[8], 'f', m_pData->m_numberOfDigits))
                                             .arg(QChar(0x00B0)));
            }
            else
            {
                //elements[3]->setText("");
                 topLevelItem(row)->setText(3, "");
            }
            break;
        }
        case ito::PrimitiveContainer::tSquare:
        {
            //elements[1]->setText(QString(coordsString).arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString(coordsString)
                                        .arg(QString::number(val[2], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[3], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[4], 'f', m_pData->m_numberOfDigits)));

            //elements[2]->setText(QString("a = %1 %2").arg(QString::number(val[5])).arg(m_pData->m_valueUnit));
            topLevelItem(row)->setText(2, QString("a = %1 %2")
                                        .arg(QString::number(val[5], 'f', m_pData->m_numberOfDigits))
                                        .arg(m_pData->m_valueUnit));

            if (ito::dObjHelper::isFinite(val[6]))
            {
                //elements[3]->setText(QString("alpha = %1%2").arg(QString::number(val[6])).arg(QChar((uchar)248)));
                 topLevelItem(row)->setText(3, QString("alpha = %1%2")
                                             .arg(QString::number(val[6], 'f', m_pData->m_numberOfDigits))
                                             .arg(QChar(0x00B0)));
            }
            else
            {
                //elements[3]->setText("");
                 topLevelItem(row)->setText(3, "");
            }
            break;
        }
        case ito::PrimitiveContainer::tPolygon:
        {
            //elements[1]->setText(QString(coordsString).arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString(coordsString)
                                        .arg(QString::number(val[2], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[3], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[4], 'f', m_pData->m_numberOfDigits)));

            //elements[2]->setText(QString(coordsString).arg(QString::number(val[4])).arg(QString::number(val[5])).arg(QString::number(val[6])));
            topLevelItem(row)->setText(2, QString(coordsString)
                                        .arg(QString::number(val[4], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[5], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[6], 'f', m_pData->m_numberOfDigits)));

            //elements[3]->setText(QString("%1 [%2]").arg(QString::number(val[7])).arg(QString::number(val[8])));
             topLevelItem(row)->setText(3, QString("%1 [%2]")
                                         .arg(QString::number(val[7], 'f', m_pData->m_numberOfDigits))
                                         .arg(QString::number(val[8], 'f', m_pData->m_numberOfDigits)));
            break;
        }
    }

    //free(elements);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::updateRelationShips(const bool fastUpdate)
{
    //QLabel** elements = (QLabel**)calloc(5, sizeof(QLabel*));

    QStringList tempList;

    tempList << QString("") << QString("") << QString("") << QString("") << QString("");

    if (fastUpdate)
    {
        // do nothing!!
    }
    else
    {
        QList<ito::int32> keys = m_rowHash.keys();
        for (int rel = 0; rel < m_pData->m_relationsList.size(); rel++)
        {
            m_pData->m_relationsList[rel].myWidget = NULL;
            //m_pData->m_relationsList[rel].firstElementRow = -1;
            //m_pData->m_relationsList[rel].secondElementRow = -1;
        }

        for (int geo = 0; geo < keys.size(); geo++)
        {
            QTreeWidgetItem* currentGeometry = topLevelItem(geo);
            QVector<ito::int16> relationIdxVec;
            relationIdxVec.reserve(24);
            for (int rel = 0; rel < m_pData->m_relationsList.size(); rel++)
            {
                if (m_pData->m_relationsList[rel].firstElementIdx == keys[geo] && m_pData->m_relationsList[rel].type != 0)
                {
                    relationIdxVec.append(rel);
                }
            }

            while(currentGeometry->childCount() < relationIdxVec.size())
            {
                currentGeometry->addChild(new QTreeWidgetItem(currentGeometry, tempList));
            }

            while(currentGeometry->childCount() > relationIdxVec.size())
            {
                currentGeometry->removeChild(currentGeometry->child(currentGeometry->childCount()-1));
            }

            for (int childIdx = 0; childIdx < currentGeometry->childCount(); childIdx++)
            {
                /*
                for (int i = 0; i < 5; i++)
                {
                    elements[i] = (QLabel*) (itemWidget(currentGeometry->child(childIdx), i));
                    if (elements[i] == NULL)
                    {
                        
                        elements[i]  = new QLabel("", this, 0);
                        setItemWidget(currentGeometry->child(childIdx), i, elements[i]);
                    }
                    else
                    {
                        elements[i]->setText("");
                    }
                }
                */

                for (int i = 0; i < 5; i++)
                {
                    currentGeometry->child(childIdx)->setText(i, "");
                }

                m_pData->m_relationsList[relationIdxVec[childIdx]].myWidget = currentGeometry->child(childIdx);

                int curRel = relationIdxVec[childIdx];
                int idx = m_pData->m_relationsList[curRel].type & 0x0FFF;

                idx = idx < m_pData->m_relationNames.length() ? idx : 0;
                //elements[0]->setText(m_pData->m_relationNames[idx]);
                currentGeometry->child(childIdx)->setText(0, m_pData->m_relationNames[idx]);

                int idx2 = m_pData->m_relationsList[curRel].secondElementIdx;

                int secondType = 0;

                //m_pData->m_relationsList[curRel].firstElementRow = geo;
                //m_pData->m_relationsList[curRel].secondElementRow = -1;

                for (int geo2 = 0; geo2 < geo; geo2++)
                {
                    if (idx2 ==  keys[geo2])
                    {
                        //m_pData->m_relationsList[curRel].secondElementRow = geo2;
                        secondType = (ito::int32)(m_rowHash[keys[geo2]].cells[1]) & 0x0000FFFF;
                    }
                }

                for (int geo2 = geo + 1; geo2 < keys.size(); geo2++)
                {
                    if (idx2 ==  keys[geo2])
                    {
                        //m_pData->m_relationsList[curRel].secondElementRow = geo2;
                        secondType = (ito::int32)(m_rowHash[keys[geo2]].cells[1]) & 0x0000FFFF;
                    }
                }

                secondType = secondType > 11 ? 0 : secondType; 

                //if (idx2 > - 1 && secondType > 0) elements[1]->setText(QString(primitivNames[secondType]).append(QString::number(idx2)));
                //else elements[1]->setText("");

                if (idx2 > - 1 && secondType > 0 && m_pData->m_primitivNames.contains(secondType))
                {
                    currentGeometry->child(childIdx)->setText(1, QString(m_pData->m_primitivNames[secondType]).append(QString::number(idx2)));
                }
                else
                {
                    currentGeometry->child(childIdx)->setText(1, "");
                }
            }
        }

        for (int rel = 0; rel < m_pData->m_relationsList.size(); rel++)
        {
            if (m_pData->m_relationsList[rel].myWidget == NULL)
            {
                continue;
            }

            switch(m_pData->m_relationsList[rel].type & 0x0FFF)
            {
            case tRadius:
                m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/radius.png"));
                m_pData->m_relationsList[rel].myWidget->setText(1, "");
                m_pData->m_relationsList[rel].myWidget->setBackgroundColor(1, QColor(255, 255, 255));
                break;
            case tAngle:
                m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/angle.png"));
                if (m_pData->m_relationsList[rel].secondElementIdx < 0)
                {
                    m_pData->m_relationsList[rel].myWidget->setBackgroundColor(1, QColor(255, 200, 200));
                }
                else
                {
                    m_pData->m_relationsList[rel].myWidget->setBackgroundColor(1, QColor(255, 255, 255));
                }
                break;
            case tDistance:
                m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/distance.png"));
                if (m_pData->m_relationsList[rel].secondElementIdx < 0)
                {
                    m_pData->m_relationsList[rel].myWidget->setBackgroundColor(1, QColor(255, 200, 200));
                }
                else
                {
                    m_pData->m_relationsList[rel].myWidget->setBackgroundColor(1, QColor(255, 255, 255));
                }
                break;
            case tIntersection:
            {
                m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/intersec.png"));
                if (m_pData->m_relationsList[rel].secondElementIdx < 0)
                {
                    m_pData->m_relationsList[rel].myWidget->setBackgroundColor(1, QColor(255, 200, 200));
                }
                else
                {
                    m_pData->m_relationsList[rel].myWidget->setBackgroundColor(1, QColor(255, 255, 255));
                }
                break;
            }
            case tLength:
            {
                m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/length.png"));
                m_pData->m_relationsList[rel].myWidget->setText(1, "");
                m_pData->m_relationsList[rel].myWidget->setBackgroundColor(1, QColor(255, 255, 255));
                break;
            }
            case tArea:
                m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/area.png"));
                m_pData->m_relationsList[rel].myWidget->setText(1, "");
                m_pData->m_relationsList[rel].myWidget->setBackgroundColor(1, QColor(255, 255, 255));
                break;
            default:

                break;
            }
        }
    }

    QString resultString("");
    resultString.reserve(50);

    for (int rel = 0; rel < m_pData->m_relationsList.size(); rel++)
    {
        ito::float32* first;
        ito::float32* second;
        bool check;
        
        /*
        for (int col = 1; col < 5; col++)
        {
            elements[col] = (QLabel*) (itemWidget(m_pData->m_relationsList[rel].myWidget, col));
        }
        */
        resultString = "NaN";

        /*
        if (elements[2] == NULL)
        {
            continue;
        }
        */

        if (m_pData->m_relationsList[rel].myWidget == NULL)
        {
            continue;
        }

        if (m_pData->m_relationsList[rel].type & tExtern)
        {
            resultString = QString("%1 %2").arg(QString::number(m_pData->m_relationsList[rel].extValue, 'f', m_pData->m_numberOfDigits))
                                           .arg(m_pData->m_valueUnit);

            //elements[2]->setText(resultString);
            m_pData->m_relationsList[rel].myWidget->setText(2, resultString); 
            continue;
        }
        //else if (m_pData->m_relationsList[rel].firstElementRow > -1)
        else if (m_rowHash.contains(m_pData->m_relationsList[rel].firstElementIdx))
        {
            //first = m_rowHash[m_pData->m_relationsList[rel].firstElementRow].cells;
            first = m_rowHash[m_pData->m_relationsList[rel].firstElementIdx].cells;
        }
        else
        {
            //elements[2]->setText(resultString);
            m_pData->m_relationsList[rel].myWidget->setText(2, resultString);
            m_pData->m_relationsList[rel].myWidget->setBackgroundColor(2, QColor(255, 200, 200));
            continue;
        }

        if (m_pData->m_relationsList[rel].type == tRadius)
        {
            //m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/radius.png"));
            check = calculateRadius(first, m_pData->m_relationsList[rel].extValue);

            resultString = QString("%1 %2").arg(QString::number(m_pData->m_relationsList[rel].extValue, 'f', m_pData->m_numberOfDigits))
                                           .arg(m_pData->m_valueUnit);

            //elements[1]->setText("");
            //m_pData->m_relationsList[rel].myWidget->setText(1, ""); 
        }
        else if (m_pData->m_relationsList[rel].type == tLength)
        {
            //m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/length.png"));
            check = calculateLength(first, m_pData->m_consider2DOnly, m_pData->m_relationsList[rel].extValue);
            resultString = QString("%1 %2").arg(QString::number(m_pData->m_relationsList[rel].extValue, 'f', m_pData->m_numberOfDigits))
                                           .arg(m_pData->m_valueUnit);
            //m_pData->m_relationsList[rel].myWidget->setText(1, ""); 
        }
        else if (m_pData->m_relationsList[rel].type == tArea)
        {
            check = calculateArea(first, m_pData->m_consider2DOnly, m_pData->m_relationsList[rel].extValue);
            resultString = QString("%1 %2%3").arg(QString::number(m_pData->m_relationsList[rel].extValue, 'f', m_pData->m_numberOfDigits))
                                            .arg(m_pData->m_valueUnit)
                                            .arg(QChar(0x00B2));
            //m_pData->m_relationsList[rel].myWidget->setText(1, "");       
        }
        else
        {
            //if (m_pData->m_relationsList[rel].secondElementRow > -1)
            if (m_rowHash.contains(m_pData->m_relationsList[rel].secondElementIdx))
            {
                second = m_rowHash[m_pData->m_relationsList[rel].secondElementIdx].cells;

                switch(m_pData->m_relationsList[rel].type & 0x0FFF)
                {
                case tAngle:
                    check = calculateAngle(first, second, m_pData->m_consider2DOnly, m_pData->m_relationsList[rel].extValue);
                    resultString = QString("%1 %2").arg(QString::number(m_pData->m_relationsList[rel].extValue, 'f', m_pData->m_numberOfDigits))
                                                  .arg(QChar(0x00B0));
                    break;
                case tDistance:
                    check = calculateDistance(first, second, m_pData->m_consider2DOnly, m_pData->m_relationsList[rel].extValue);
                    resultString = QString("%1 %2").arg(QString::number(m_pData->m_relationsList[rel].extValue, 'f', m_pData->m_numberOfDigits))
                                                  .arg(m_pData->m_valueUnit);
                    break;
                /*
                case tIntersection:
                {
                    cv::Vec3f val;
                    check = calculateIntersections(first, second, m_pData->m_consider2DOnly, val);
                    if (m_pData->m_consider2DOnly)
                    {
                        resultString = QString("%1, %2 [%4]").arg(QString::number(val[0], 'f', m_pData->m_numberOfDigits))
                                                                 .arg(QString::number(val[1], 'f', m_pData->m_numberOfDigits))
                                                                 .arg(m_pData->m_valueUnit);                       
                    }
                    else
                    {
                        resultString = QString("%1, %2, %3 [%4]").arg(QString::number(val[0], 'f', m_pData->m_numberOfDigits))
                                                                 .arg(QString::number(val[1], 'f', m_pData->m_numberOfDigits))
                                                                 .arg(QString::number(val[2], 'f', m_pData->m_numberOfDigits))
                                                                 .arg(m_pData->m_valueUnit);                    
                    }
                    break;
                }
                */
                default:
                    m_pData->m_relationsList[rel].myWidget->setText(2, resultString); 
                    continue;
                }
            }
            else
            {
                m_pData->m_relationsList[rel].myWidget->setText(2, resultString); 
                continue;
            }
        }        
        m_pData->m_relationsList[rel].myWidget->setText(2, resultString); 

        if (check)
        {
            m_pData->m_relationsList[rel].myWidget->setBackgroundColor(2, QColor(255,255,255));
        }
        else
        {
            m_pData->m_relationsList[rel].myWidget->setBackgroundColor(2, QColor(255, 200, 200));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateAngle(ito::float32 *first, ito::float32 *second, const bool eval2D, ito::float32 &angle)
{
    ito::uint16 typeOne = (ito::uint16)((ito::uint32)(first[1]) & 0x0000FFFF);
    ito::uint16 typeTwo = (ito::uint16)((ito::uint32)(second[1]) & 0x0000FFFF);

    if (typeOne != ito::PrimitiveContainer::tLine)
    {
        angle = quietNaN;
        return false;
    }
    if (typeTwo != ito::PrimitiveContainer::tLine)
    {
        angle = quietNaN;
        return false;
    }

    cv::Vec3f firstVector(first[5] - first[2], first[6] - first[3], first[7] - first[4]);
    cv::Vec3f secondVector(second[5] - second[2], second[6] - second[3], second[7] - second[4]);

    if (eval2D)
    {
        firstVector[2] = 0.0;
        secondVector[2] = 0.0;
    }

    ito::float32 abs = (sqrt(pow(firstVector[0],2) + pow(firstVector[1],2) + pow(firstVector[2],2)) * sqrt(pow(secondVector[0],2) + pow(secondVector[1],2) + pow(secondVector[2],2)));

    if (ito::dObjHelper::isNotZero(abs))
    {
        angle = acos(firstVector.dot(secondVector) / abs) * 180 / GEO_PI;
        return true;    
    }
    angle = quietNaN;
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateDistance(ito::float32 *first, ito::float32 *second, const bool eval2D, ito::float32 &distance)
{
    cv::Vec3f lineDirVector;
    cv::Vec3f linePosVector;
    cv::Vec3f pointPosVector;

    ito::uint16 typeOne = (ito::uint16)((ito::uint32)(first[1]) & 0x0000FFFF);
    ito::uint16 typeTwo = (ito::uint16)((ito::uint32)(second[1]) & 0x0000FFFF);

    // distance of two points or two circles or combination
    if ((typeOne == ito::PrimitiveContainer::tPoint || typeOne == ito::PrimitiveContainer::tCircle) &&
       (typeTwo == ito::PrimitiveContainer::tPoint || typeTwo == ito::PrimitiveContainer::tCircle))
    {
        if (eval2D)
        {
            pointPosVector = cv::Vec3f(first[2] - second[2], first[3] - second[3], 0.0);
        }
        else
        {
            pointPosVector = cv::Vec3f(first[2] - second[2], first[3] - second[3], first[4] - second[4]);
        }

        distance = sqrt(pow(pointPosVector[0],2) + pow(pointPosVector[1],2) + pow(pointPosVector[2],2));
        return true;
    }

    // distance of line to points or circles
    if (typeOne == ito::PrimitiveContainer::tLine && 
        typeTwo == ito::PrimitiveContainer::tPoint)
    {
        lineDirVector = cv::Vec3f(first[5] - first[2], first[6] - first[3], first[7] - first[4]);
        linePosVector = cv::Vec3f(first[2], first[3], first[4]);
        pointPosVector = cv::Vec3f(second[2], second[3], second[4]);
    }
    else if (typeTwo == ito::PrimitiveContainer::tLine && 
            typeOne == ito::PrimitiveContainer::tPoint)
    {
        lineDirVector = cv::Vec3f(second[5] - second[2], second[6] - second[3], second[7] - second[4]);
        linePosVector = cv::Vec3f(second[2], second[3], second[4]);
        pointPosVector = cv::Vec3f(first[2], first[3], first[4]);
    }
    else
    {
        distance = quietNaN;
        return false;
    }

    if (eval2D)
    {
        lineDirVector[2] = 0.0;
        linePosVector[2] = 0.0;
        pointPosVector[2] = 0.0;
    }

    if (!ito::dObjHelper::isNotZero(lineDirVector[0]) && !ito::dObjHelper::isNotZero(lineDirVector[1]) && !ito::dObjHelper::isNotZero(lineDirVector[2]))
    {
        distance = quietNaN;
        return false;
    }

    ito::float32 lambda = (pointPosVector - linePosVector).dot(lineDirVector) / lineDirVector.dot(lineDirVector);
    cv::Vec3f temp = pointPosVector - (linePosVector + lambda * lineDirVector);
    distance = sqrt(temp.dot(temp));

    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateRadius(ito::float32 *first, ito::float32 &radius)
{
    ito::uint16 type = (ito::uint16)((ito::uint32)(first[1]) & 0x0000FFFF);

    switch(type)
    {
        case ito::PrimitiveContainer::tCircle:
            radius = first[5];
            return true;
        case ito::PrimitiveContainer::tEllipse:
            radius = (first[5] +  first[6])/2;   
            return true;
        default:
            radius = quietNaN;
            return false;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateLength(ito::float32 *first, const bool eval2D, ito::float32 &length)
{
    if (((ito::uint32)(first[1]) & 0x0000FFFF) != ito::PrimitiveContainer::tLine)
    {
        length = quietNaN;
        return false;
    }

    if (eval2D)
    {
        length = sqrt(pow(first[2] - first[5], 2)  + pow(first[3] - first[6], 2));
    }
    else
    {
        length = sqrt(pow(first[2] - first[5], 2)  + pow(first[3] - first[6], 2) + pow(first[4] - first[7], 2));
    }

    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateArea(ito::float32 *first, const bool eval2D, ito::float32 &area)
{
    ito::uint16 type = (ito::uint16)((ito::uint32)(first[1]) & 0x0000FFFF);

    switch(type)
    {
        case ito::PrimitiveContainer::tRectangle:
            area = abs((first[5] - first[2]) * (first[6] - first[3]));
            return true;
        case ito::PrimitiveContainer::tSquare:
            area = (first[5] * first[5]);
            return true;
        case ito::PrimitiveContainer::tCircle:
            area = (first[5] * first[5])* GEO_PI;
            return true;
        case ito::PrimitiveContainer::tEllipse:
            area = (first[5] * first[6])* GEO_PI;
            return true;
        default:
            area = quietNaN;
            return false;
    }

    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateCircumference(ito::float32 *first, ito::float32 &length)
{
    ito::uint16 type = (ito::uint16)((ito::uint32)(first[1]) & 0x0000FFFF);

    switch(type)
    {
        case ito::PrimitiveContainer::tRectangle:
            length = abs(2 * (first[5] - first[2])) + abs (2 * (first[6] - first[3])); 
            return true;
        case ito::PrimitiveContainer::tSquare:
            length = 4 * first[5];
            return true;
        case ito::PrimitiveContainer::tCircle:
            length =  2 * first[5] * GEO_PI;
            return true;
        default:
            length = quietNaN;
            return false;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*
bool PlotTreeWidget::calculateIntersections(ito::float32 *first, ito::float32 *second, const bool eval2D, cv::Vec3f &point)
{

    if (((ito::uint32)(first[1]) & 0x0000FFFF) != ito::PrimitiveContainer::tLine || ((ito::uint32)(second[1]) & 0x0000FFFF) != ito::PrimitiveContainer::tLine)
    {
        point[0] = quietNaN;
        point[1] = quietNaN;
        point[2] = quietNaN;
        return false;
    }

    cv::Vec3f firstLineDirVector(first[5] - first[2], first[6] - first[3], first[7] - first[4]);
    cv::Vec3f firstLinePosVector(first[2], first[3], first[4]);
    cv::Vec3f secondLineDirVector(second[5] - second[2], second[6] - second[3], second[7] - second[4]);
    cv::Vec3f secondLinePosVector(second[2], second[3], second[4]);

    ito::float32 absFst = sqrt(pow(firstLineDirVector[0],2) + pow(firstLineDirVector[1],2) + pow(firstLineDirVector[2],2));
    ito::float32 absSec = sqrt(pow(secondLineDirVector[0],2) + pow(secondLineDirVector[1],2) + pow(secondLineDirVector[2],2));

    if (!ito::dObjHelper::isNotZero(absFst) ||  !ito::dObjHelper::isNotZero(absSec))
    {
        point[0] = quietNaN;
        point[1] = quietNaN;
        point[2] = quietNaN;
        return false;
    }

    firstLineDirVector *= 1/absFst;
    secondLineDirVector *= 1/absSec;

    ito::float32 lambda = 0.0;
    ito::float32 kappa  = 0.0;

    // Vectors are the same we have to check if the positions vectors are on the same line
    if (ito::dObjHelper::isNotZero(firstLineDirVector[0] - secondLineDirVector[0]) &&
        ito::dObjHelper::isNotZero(firstLineDirVector[1] - secondLineDirVector[1]) &&
        ito::dObjHelper::isNotZero(firstLineDirVector[2] - secondLineDirVector[2])) 
    {
        secondLinePosVector -= firstLinePosVector;
        lambda = secondLinePosVector[0] / firstLinePosVector[0];
        if ( ito::dObjHelper::isNotZero(secondLinePosVector[1] / firstLinePosVector[1] - lambda)
          && ito::dObjHelper::isNotZero(secondLinePosVector[2] / firstLinePosVector[2] - lambda))
        {
            point = firstLinePosVector;

            return true;
        }
        else
        {
            point[0] = quietNaN;
            point[1] = quietNaN;
            point[2] = quietNaN;
            return true;
        }
    }
    else if (eval2D ||  
            (!ito::dObjHelper::isNotZero(firstLinePosVector[2]) && 
            !ito::dObjHelper::isNotZero(secondLinePosVector[2]) &&
            !ito::dObjHelper::isNotZero(firstLineDirVector[2]) && 
            !ito::dObjHelper::isNotZero(secondLineDirVector[2]))) // is a two dimensional problem
    {

    }
    else // otherwise we have do do it the hard way
    {
    
        return false;
    }

    return true;
}
*/
//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::refreshPlot(const ito::DataObject* dataObj)
{
    bool changed = false;
    bool clear = false;
    bool identical = false;
    int cols = 0;
    int dims = 0;
   
    if (dataObj)
    {
        dims = dataObj->getDims();
        identical = true;
        if (dataObj->getDims() == 0)
        {
            clear = true;
            identical = false;
        }
        else if (dataObj->getType() != ito::tFloat32)
        {
            m_lastRetVal = ito::RetVal(ito::retError, 0,tr("DataObject must be ito::float32").toAscii().data());
            identical = false;            
        }
        else if (dataObj->getSize(dims-1) < 2)
        {
            m_lastRetVal = ito::RetVal(ito::retError, 0,tr("DataObject has not enough columns").toAscii().data());
            identical = false;
        }
        else
        {
            QList<ito::int32> hashKeys = m_rowHash.keys();
            cv::Mat* scrMat = (cv::Mat*)(dataObj->get_mdata()[dataObj->seekMat(0)]);

            bool found = false;
            
            cols = std::min(scrMat->cols, PRIM_ELEMENTLENGTH);

            ito::float32* srcPtr;

            if (scrMat->rows == hashKeys.size())
            {
                for (int dcnt = 0; dcnt < hashKeys.size(); dcnt++)
                {
                    srcPtr = scrMat->ptr<ito::float32>(dcnt);
                    if ((hashKeys[dcnt] != (ito::int32)srcPtr[0]) || ((ito::int32)m_rowHash[hashKeys[dcnt]].cells[1] != (ito::int32)srcPtr[1]))
                    {
                        identical = false;
                        break;
                    }
                }
            }
            else
            {
                identical = false;
            }

            if (!identical)
            {
                for (int dcnt = 0; dcnt < hashKeys.size(); dcnt++)
                {
                    found = false;
                    for (int scnt = 0; scnt < scrMat->rows; scnt++)
                    {
                        srcPtr = scrMat->ptr<ito::float32>(scnt);
                        if (hashKeys[dcnt] == (ito::int32)srcPtr[0])
                        {
                            found = true;
                            break;
                        }
                    }

                    if (!found)
                    {
                        changed = true;
                        m_rowHash.remove(hashKeys[dcnt]);
                    }
                }

                if (changed)
                {
                    hashKeys = m_rowHash.keys();
                }

                for (int scnt = 0; scnt < scrMat->rows; scnt++)
                {
                    found = false;
                    srcPtr = scrMat->ptr<ito::float32>(scnt);
                    for (int dcnt = 0; dcnt < hashKeys.size(); dcnt++)
                    {
                        if (hashKeys[dcnt] == (ito::int32)srcPtr[0])
                        {
                            std::fill(m_rowHash[hashKeys[dcnt]].cells, m_rowHash[hashKeys[dcnt]].cells + PRIM_ELEMENTLENGTH, quietNaN);
                            memcpy(m_rowHash[hashKeys[dcnt]].cells, srcPtr, sizeof(ito::float32) * cols);
                            found = true;
                            break;
                        }
                    }

                    if (!found && (((ito::int32)(srcPtr[1]) & 0x0000FFFF)!= 0))
                    {
                        geometricPrimitives newVal;
                        std::fill(newVal.cells, newVal.cells + PRIM_ELEMENTLENGTH, 0.0f);
                        changed = true;
                        memcpy(newVal.cells, srcPtr, sizeof(ito::float32) * cols);

                        int idx = 0;
                        if (ito::dObjHelper::isFinite(newVal.cells[0]) && newVal.cells[0] < 65355 && newVal.cells[0] > -1) idx = (ito::int32)newVal.cells[0];
                        m_rowHash.insert(idx, newVal);
                    }
                }
            }
        }
    }
     
    if (clear)
    {
        QList<ito::int32> hashTags = m_rowHash.keys();
        for (int i = 0; i < hashTags.size(); i++)
        {
            m_rowHash.remove(hashTags[i]);
        }
        
        m_pData->m_relationsList.clear();
        this->clear();
    }
    if (identical)
    {
        cv::Mat* scrMat = (cv::Mat*)(dataObj->get_mdata()[dataObj->seekMat(0)]);
        ito::float32* srcPtr;

        QList<ito::int32> hashTags = m_rowHash.keys();

        for (int dcnt = 0; dcnt < hashTags.size(); dcnt++)
        {
            srcPtr = scrMat->ptr<ito::float32>(dcnt);
            std::fill(m_rowHash[hashTags[dcnt]].cells, m_rowHash[hashTags[dcnt]].cells + PRIM_ELEMENTLENGTH, 0.0f);
            memcpy(m_rowHash[hashTags[dcnt]].cells, srcPtr, sizeof(ito::float32) * cols);
            setPrimitivElement(dcnt, true, m_rowHash[hashTags[dcnt]].cells);
        }
        updateRelationShips(true);
    }
    else if (changed)
    {
        this->clear();
        QList<ito::int32> hashTags = m_rowHash.keys();
        for (int dcnt = 0; dcnt < hashTags.size(); dcnt++)
        {
            QStringList tempList;
            tempList << QString("") << QString("") << QString("") << QString("") << QString("");
            QTreeWidgetItem* temp = new QTreeWidgetItem(this, tempList);
            addTopLevelItem(temp);
            setPrimitivElement(dcnt, false, m_rowHash[hashTags[dcnt]].cells);
        }

        updateRelationShips(false);
    }
    else
    {
        updateRelationShips(true);
    }

    expandAll();
    repaint();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTreeWidget::updateElement(const ito::int32 &idx,const ito::int32 &flags,const QVector<ito::float32> &values)
{
    if (m_rowHash.contains(idx))
    {        
        if ((ito::int32)m_rowHash[idx].cells[1] ==  flags)
        {  
            std::fill(&(m_rowHash[idx].cells[2]), m_rowHash[idx].cells + PRIM_ELEMENTLENGTH, 0.0f);
            memcpy(&(m_rowHash[idx].cells[2]), values.data(), sizeof(ito::float32) * std::min(values.size(), PRIM_ELEMENTLENGTH -2));
            setPrimitivElement(m_rowHash.keys().indexOf(idx), true, m_rowHash[idx].cells);
        }
    }
    updateRelationShips(true);
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTreeWidget::writeToCSV(const QFileInfo &fileName, const bool asTable)
{
    if (fileName.exists() && !fileName.isWritable())
    {
        return ito::RetVal(ito::retError, 0, tr("could not write csv-data to file %1").arg(fileName.fileName()).toAscii().data());
    }

    QFile saveFile(fileName.filePath());

    saveFile.open(QIODevice::WriteOnly | QIODevice::Text);

    QByteArray outBuffer;
    outBuffer.reserve(200);

    for (int geo = 0; geo < topLevelItemCount(); geo++)
    {
        outBuffer.clear();
        QTreeWidgetItem *curItem = topLevelItem(geo);
        
        outBuffer.append(curItem->text(0));
        for (int col = 1; col < this->columnCount(); col++)
        {
            if (curItem->text(col).isEmpty() && !asTable) continue;
            outBuffer.append(", ");
            outBuffer.append(curItem->text(col));
        }
        
        if (asTable)
        {
            outBuffer.append('\n');
            saveFile.write(outBuffer);
            int relCount = curItem->childCount();

            for (int rel = 0; rel < relCount; rel ++)
            {
                outBuffer.clear();
                outBuffer.append(curItem->text(0));
                for (int col = 0; col < this->columnCount(); col++)
                {
                    outBuffer.append(", ");
                    outBuffer.append(curItem->child(rel)->text(col));
                }
                outBuffer.append('\n');

                saveFile.write(outBuffer);
            }
        }
        else
        {
            saveFile.write(outBuffer);
            int relCount = curItem->childCount();
            outBuffer.clear();
            for (int rel = 0; rel < relCount; rel ++)
            {
                for (int col = 0; col < this->columnCount() -1; col++)
                {
                    if (curItem->child(rel)->text(col).isEmpty()) continue;
                    outBuffer.append(", ");
                    outBuffer.append(curItem->child(rel)->text(col));
                }
            }
            outBuffer.append('\n');

            saveFile.write(outBuffer);
        }
    }

    saveFile.close();

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTreeWidget::writeToXML(const QFileInfo &fileName)
{
    if (fileName.exists() && !fileName.isWritable())
    {
        return ito::RetVal(ito::retError, 0, tr("could not write csv-data to file %1").arg(fileName.fileName()).toAscii().data());
    }

    QFile saveFile(fileName.filePath());

    saveFile.open(QIODevice::WriteOnly | QIODevice::Text);

    QXmlStreamWriter stream(&saveFile);
    QString attrname;

    stream.setCodec("UTF-8");       // Set text codec
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("itomGeometricElements");
    {
        stream.writeAttribute("href", "http://www.ito.uni-stuttgart.de");
        
        QHash<ito::int32, geometricPrimitives >::const_iterator curValue = m_rowHash.constBegin();
        for (int geo = 0; curValue !=  m_rowHash.end(); ++curValue, geo++)
        //for (int geo = 0; geo < m_rowHash.size(); geo++)
        {
            stream.writeStartElement(QString::number(geo));
            stream.writeAttribute("index", QString::number((ito::int32)curValue->cells[0]));
            stream.writeAttribute("flags", QString::number((ito::int32)curValue->cells[1]));

            QVector<ito::int16> relationIdxVec;
            relationIdxVec.reserve(24);
            for (int rel = 0; rel < m_pData->m_relationsList.size(); rel++)
            {
                if (m_pData->m_relationsList[rel].firstElementIdx == (ito::uint32)(curValue->cells[0]) && m_pData->m_relationsList[rel].type != 0)
                {
                    relationIdxVec.append(rel);
                }
            }

            ito::uint16 type = ((ito::int32)curValue->cells[1]) & 0x0000FFFF;

            if (m_pData->m_primitivNames.contains(type))
            {
                stream.writeAttribute("name", m_pData->m_primitivNames[type]);
            }
            else
            {
                stream.writeAttribute("name", m_pData->m_primitivNames[ito::PrimitiveContainer::tNoType]);
            }

            switch(type)
            {
                case ito::PrimitiveContainer::tPoint:
                {                    
                    stream.writeAttribute("x0", QString::number((ito::float32)curValue->cells[2]));
                    stream.writeAttribute("y0", QString::number((ito::float32)curValue->cells[3]));
                    stream.writeAttribute("z0", QString::number((ito::float32)curValue->cells[4]));
                }
                break;
                case ito::PrimitiveContainer::tLine:
                {
                    stream.writeAttribute("x0", QString::number((ito::float32)curValue->cells[2]));
                    stream.writeAttribute("y0", QString::number((ito::float32)curValue->cells[3]));
                    stream.writeAttribute("z0", QString::number((ito::float32)curValue->cells[4]));
                    stream.writeAttribute("x1", QString::number((ito::float32)curValue->cells[5]));
                    stream.writeAttribute("y1", QString::number((ito::float32)curValue->cells[6]));
                    stream.writeAttribute("z1", QString::number((ito::float32)curValue->cells[7]));
                }
                break;
                case ito::PrimitiveContainer::tEllipse:
                {
                    stream.writeAttribute("x0", QString::number((ito::float32)curValue->cells[2]));
                    stream.writeAttribute("y0", QString::number((ito::float32)curValue->cells[3]));
                    stream.writeAttribute("z0", QString::number((ito::float32)curValue->cells[4]));
                    stream.writeAttribute("r1", QString::number((ito::float32)curValue->cells[5]));
                    stream.writeAttribute("r2", QString::number((ito::float32)curValue->cells[6]));
                    stream.writeAttribute("alpha", QString::number((ito::float32)curValue->cells[7]));
                }
                break;

                case ito::PrimitiveContainer::tCircle:
                {
                    stream.writeAttribute("x0", QString::number((ito::float32)curValue->cells[2]));
                    stream.writeAttribute("y0", QString::number((ito::float32)curValue->cells[3]));
                    stream.writeAttribute("z0", QString::number((ito::float32)curValue->cells[4]));
                    stream.writeAttribute("r1", QString::number((ito::float32)curValue->cells[5]));
                }
                break;

                case ito::PrimitiveContainer::tRectangle:
                {
                    stream.writeAttribute("x0", QString::number((ito::float32)curValue->cells[2]));
                    stream.writeAttribute("y0", QString::number((ito::float32)curValue->cells[3]));
                    stream.writeAttribute("z0", QString::number((ito::float32)curValue->cells[4]));
                    stream.writeAttribute("x1", QString::number((ito::float32)curValue->cells[5]));
                    stream.writeAttribute("y1", QString::number((ito::float32)curValue->cells[6]));
                    stream.writeAttribute("z1", QString::number((ito::float32)curValue->cells[7]));
                    stream.writeAttribute("alpha", QString::number((ito::float32)curValue->cells[8]));
                }
                break;

                case ito::PrimitiveContainer::tSquare:
                {
                    stream.writeAttribute("x0", QString::number((ito::float32)curValue->cells[2]));
                    stream.writeAttribute("y0", QString::number((ito::float32)curValue->cells[3]));
                    stream.writeAttribute("z0", QString::number((ito::float32)curValue->cells[4]));
                    stream.writeAttribute("a", QString::number((ito::float32)curValue->cells[5]));
                    stream.writeAttribute("alpha", QString::number((ito::float32)curValue->cells[6]));
                }
                break;

                case ito::PrimitiveContainer::tPolygon:
                {
                    stream.writeAttribute("x0", QString::number((ito::float32)curValue->cells[2]));
                    stream.writeAttribute("y0", QString::number((ito::float32)curValue->cells[3]));
                    stream.writeAttribute("z0", QString::number((ito::float32)curValue->cells[4]));
                    stream.writeAttribute("xDir", QString::number((ito::float32)curValue->cells[5]));
                    stream.writeAttribute("yDir", QString::number((ito::float32)curValue->cells[6]));
                    stream.writeAttribute("zDir", QString::number((ito::float32)curValue->cells[7]));
                    stream.writeAttribute("No", QString::number((ito::float32)curValue->cells[8]));
                    stream.writeAttribute("Total", QString::number((ito::float32)curValue->cells[9]));
                }
                break; 
            }

            for (int rel = 0; rel < relationIdxVec.size(); rel++)
            {
                stream.writeStartElement(QString::number(rel));

                stream.writeAttribute("element0", QString::number(m_pData->m_relationsList[relationIdxVec[rel]].firstElementIdx));
                stream.writeAttribute("type", QString::number(m_pData->m_relationsList[relationIdxVec[rel]].type));
                stream.writeAttribute("element1", QString::number(m_pData->m_relationsList[relationIdxVec[rel]].secondElementIdx));
                stream.writeAttribute("value", QString::number(m_pData->m_relationsList[relationIdxVec[rel]].extValue));

                stream.writeEndElement();
            }

            stream.writeEndElement();
        }
    }
    stream.writeEndDocument();

    saveFile.close();

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTreeWidget::writeToRAW(const QFileInfo &fileName)
{
    if (fileName.exists() && !fileName.isWritable())
    {
        return ito::RetVal(ito::retError, 0, tr("could not write csv-data to file %1").arg(fileName.fileName()).toAscii().data());
    }

    QFile saveFile(fileName.filePath());

    saveFile.open(QIODevice::WriteOnly | QIODevice::Text);

    QByteArray outBuffer;
    outBuffer.reserve(200);

    QHash<ito::int32, geometricPrimitives >::const_iterator curValue = m_rowHash.constBegin();
    for (int geo = 0; curValue !=  m_rowHash.end(); ++curValue, geo++)
    //for (int geo = 0; geo < m_rowHash.size(); geo++)
    {
        outBuffer.clear();
        outBuffer.append(QByteArray::number((ito::int32)curValue->cells[0]));
        for (int i = 1; i < PRIM_ELEMENTLENGTH; i++)
        {
            outBuffer.append(", ");
            outBuffer.append(QByteArray::number(curValue->cells[i]));
        }
        outBuffer.append('\n');
        saveFile.write(outBuffer);
    }

    for (int rel = 0; rel < m_pData->m_relationsList.size(); rel++)
    {
        outBuffer.clear();
        outBuffer.append(QByteArray::number(m_pData->m_relationsList[rel].firstElementIdx));
        outBuffer.append(", ");
        outBuffer.append(QByteArray::number(m_pData->m_relationsList[rel].type));
        outBuffer.append(", ");
        outBuffer.append(QByteArray::number(m_pData->m_relationsList[rel].secondElementIdx));
        outBuffer.append(", ");
        outBuffer.append(QByteArray::number(m_pData->m_relationsList[rel].extValue));
        outBuffer.append('\n');
        saveFile.write(outBuffer);
    }

    saveFile.close();

    return ito::retOk;
}

/*
//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::keyPressEvent (QKeyEvent * event)
{
    
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::mousePressEvent (QMouseEvent * event)
{
   
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::mouseMoveEvent (QMouseEvent * event)
{
   
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::mouseReleaseEvent (QMouseEvent * event)
{
    
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::contextMenuEvent(QContextMenuEvent * event)
{

}


//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::setPannerEnable(const bool checked)
{
   
}
*/
//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::updatePrimitives()
{
    QList<ito::int32> hashKeys = m_rowHash.keys();
    for (int dcnt = 0; dcnt < hashKeys.size(); dcnt++)
    {
        setPrimitivElement(dcnt, true, m_rowHash[hashKeys[dcnt]].cells);
    }   
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::autoFitCols()
{
    int fontWidth = 5;
    int max = 0;
    int val = 0;
    for (int topItem = 0; topItem < topLevelItemCount(); topItem++)
    {
        val = topLevelItem(topItem)->text(0).size() * fontWidth +  2 * fontWidth + iconSize().width() + 24;
        if (val > max)
        {
            max = val;
        }

        if (topLevelItem(topItem)->childCount() > 0)
        {
            for (int childItem = 0; childItem < topLevelItem(topItem)->childCount(); childItem++)
            {
                val = topLevelItem(topItem)->child(childItem)->text(0).size() * fontWidth + 2 * fontWidth + iconSize().width() + 46;
                if (val > max)
                {
                    max = val;
                }
            }            
        }
    }
    setColumnWidth(0 ,max);

    for (int col = 1; col < columnCount(); col++)
    {
        max = 0;
        val = 0;
        for (int topItem = 0; topItem < topLevelItemCount(); topItem++)
        {
            val = topLevelItem(topItem)->text(col).size() * fontWidth + 2 * fontWidth;
            if (val > max)
            {
                max = val;
            }

            if (topLevelItem(topItem)->childCount() > 0)
            {
                for (int childItem = 0; childItem < topLevelItem(topItem)->childCount(); childItem++)
                {
                    val = topLevelItem(topItem)->child(childItem)->text(col).size() * fontWidth + 2 * fontWidth;
                    if (val > max)
                    {
                        max = val;
                    }
                }            
            }
        }
        setColumnWidth(col, max);
    }
}
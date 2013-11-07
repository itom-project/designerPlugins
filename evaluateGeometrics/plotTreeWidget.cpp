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

#define ELEMENTLENGTH (sizeof(geometricPrimitives) / sizeof(ito::float32))

//----------------------------------------------------------------------------------------------------------------------------------
PlotTreeWidget::PlotTreeWidget(QMenu *contextMenu, InternalInfo *data, QWidget * parent) :
    QTreeWidget(parent),
    m_contextMenu(contextMenu),
    m_xDirect(false),
    m_yDirect(false),
    m_pParent(parent),
    m_state(stateIdle),
    m_lastRetVal(ito::retOk)
{
    m_data = data;
    //this is the border between the canvas and the axes and the overall mainwindow
	setContentsMargins(2,2,2,2);

    setColumnCount(5);

    setEditTriggers(QAbstractItemView::NoEditTriggers);

}

//----------------------------------------------------------------------------------------------------------------------------------
PlotTreeWidget::~PlotTreeWidget()
{


    return;
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
//void PlotTreeWidget::setLabels(const QString &title, const QString &valueLabel, const QString &axisLabel)
//{
//   
//}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::setPrimitivElement(const int row, const bool update, ito::float32 *val)
{
    //QLabel** elements = (QLabel**)calloc(5, sizeof(QLabel*));

    int type = ((ito::uint32)(val[1])) & 0x0000FFFF;

    if(!update)
    {
        /*
        for(int i = 0; i < 5; i++)
        {
            elements[i] = (QLabel*) (itemWidget(topLevelItem(row), i));
            if(elements[i] == NULL)
            {
                elements[i]  = new QLabel("", this, 0);
                setItemWidget(topLevelItem(row), i, elements[i]);
            }
            else
            {
                elements[i]->setText("");
            }
        }
        */

        switch (type)
        {
            default:
            case ito::PrimitiveContainer::tNoType:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/notype.png"));
                //elements[0]->setText(tr("notype %1").arg(QString::number((ito::uint32)(val[0]))));
                topLevelItem(row)->setText(0, tr("notype %1").arg(QString::number((ito::uint32)(val[0]))));
                break;
            }
            case ito::PrimitiveContainer::tPoint:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/marker.png"));
                //elements[0]->setText(tr("point %1").arg(QString::number((ito::uint32)(val[0]))));
                topLevelItem(row)->setText(0, tr("point %1").arg(QString::number((ito::uint32)(val[0]))));
                break;
            }
            case ito::PrimitiveContainer::tLine:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"));
                //elements[0]->setText(tr("line %1").arg(QString::number((ito::uint32)(val[0]))));
                topLevelItem(row)->setText(0, tr("line %1").arg(QString::number((ito::uint32)(val[0]))));
                break;
            }
            case ito::PrimitiveContainer::tCircle:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/circle.png"));
                //elements[0]->setText(tr("circle %1").arg(QString::number((ito::uint32)(val[0]))));
                topLevelItem(row)->setText(0, tr("circle %1").arg(QString::number((ito::uint32)(val[0]))));
                break;
            }
            case ito::PrimitiveContainer::tElipse:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/ellipse.png"));
                //elements[0]->setText(tr("ellipse %1").arg(QString::number((ito::uint32)(val[0]))));
                topLevelItem(row)->setText(0, tr("ellipse %1").arg(QString::number((ito::uint32)(val[0]))));
                break;
            }
            case ito::PrimitiveContainer::tRetangle:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/rectangle.png"));
                //elements[0]->setText(tr("retangle %1").arg(QString::number((ito::uint32)(val[0]))));
                topLevelItem(row)->setText(0, tr("retangle %1").arg(QString::number((ito::uint32)(val[0]))));
                break;
            }
            case ito::PrimitiveContainer::tSquare:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/square.png"));
                //elements[0]->setText(tr("square %1").arg(QString::number((ito::uint32)(val[0]))));
                topLevelItem(row)->setText(0, tr("square %1").arg(QString::number((ito::uint32)(val[0]))));
                break;
            }
            case ito::PrimitiveContainer::tPolygon:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/polygon.png"));
                //elements[0]->setText(tr("polygon %1").arg(QString::number((ito::uint32)(val[0]))));
                topLevelItem(row)->setText(0, tr("polygon %1").arg(QString::number((ito::uint32)(val[0]))));
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
            //elements[1]->setText(QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            break;
        }
        case ito::PrimitiveContainer::tLine:
        {
            //elements[1]->setText(QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            //elements[2]->setText(QString("[%1; %2; %3]").arg(QString::number(val[5])).arg(QString::number(val[6])).arg(QString::number(val[7])));
            topLevelItem(row)->setText(2, QString("[%1; %2; %3]").arg(QString::number(val[5])).arg(QString::number(val[6])).arg(QString::number(val[7])));
            break;
        }
        case ito::PrimitiveContainer::tCircle:
        {
            //elements[1]->setText(QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            //elements[2]->setText(QString("r = %1 %2").arg(QString::number(val[5])).arg(m_data->m_valueLabel));
            topLevelItem(row)->setText(2, QString("r = %1 %2").arg(QString::number(val[5])).arg(m_data->m_valueLabel));
            break;
        }
        case ito::PrimitiveContainer::tElipse:
        {
            //elements[1]->setText(QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            //elements[2]->setText(QString("a,b = %1 %2 in %3").arg(QString::number(val[5])).arg(QString::number(val[6])).arg(m_data->m_valueLabel));
            topLevelItem(row)->setText(2, QString("a,b = %1 %2 in %3").arg(QString::number(val[5])).arg(QString::number(val[6])).arg(m_data->m_valueLabel));
            if(ito::dObjHelper::isFinite(val[7]))
            {
                //elements[3]->setText(QString("alpha = %1%2").arg(QString::number(val[7])).arg(QChar((uchar)248)));
                topLevelItem(row)->setText(3, QString("alpha = %1%2").arg(QString::number(val[7])).arg(QChar((uchar)248)));
            }
            else
            {
                //elements[3]->setText("");
                 topLevelItem(row)->setText(3, "");
            }
            break;
        }
        case ito::PrimitiveContainer::tRetangle:
        {
            //elements[1]->setText(QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            //elements[2]->setText(QString("[%1; %2; %3]").arg(QString::number(val[5])).arg(QString::number(val[6])).arg(QString::number(val[7])));
            topLevelItem(row)->setText(2, QString("[%1; %2; %3]").arg(QString::number(val[5])).arg(QString::number(val[6])).arg(QString::number(val[7])));
            if(ito::dObjHelper::isFinite(val[8]))
            {
                //elements[3]->setText(QString("alpha = %1%2").arg(QString::number(val[8])).arg(QChar((uchar)248)));
                 topLevelItem(row)->setText(3, QString("alpha = %1%2").arg(QString::number(val[8])).arg(QChar((uchar)248)));
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
            //elements[1]->setText(QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            //elements[2]->setText(QString("a = %1 %2").arg(QString::number(val[5])).arg(m_data->m_valueLabel));
            topLevelItem(row)->setText(2, QString("a = %1 %2").arg(QString::number(val[5])).arg(m_data->m_valueLabel));
            if(ito::dObjHelper::isFinite(val[6]))
            {
                //elements[3]->setText(QString("alpha = %1%2").arg(QString::number(val[6])).arg(QChar((uchar)248)));
                 topLevelItem(row)->setText(3, QString("alpha = %1%2").arg(QString::number(val[6])).arg(QChar((uchar)248)));
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
            //elements[1]->setText(QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            topLevelItem(row)->setText(1, QString("[%1; %2; %3]").arg(QString::number(val[2])).arg(QString::number(val[3])).arg(QString::number(val[4])));
            //elements[2]->setText(QString("[%1; %2; %3]").arg(QString::number(val[4])).arg(QString::number(val[5])).arg(QString::number(val[6])));
            topLevelItem(row)->setText(2, QString("[%1; %2; %3]").arg(QString::number(val[4])).arg(QString::number(val[5])).arg(QString::number(val[6])));
            //elements[3]->setText(QString("%1 [%2]").arg(QString::number(val[7])).arg(QString::number(val[8])));
             topLevelItem(row)->setText(3, QString("%1 [%2]").arg(QString::number(val[7])).arg(QString::number(val[8])));
            break;
        }
    }

    //free(elements);

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::updateRelationShips(const bool fastUpdate)
{
    //QLabel** elements = (QLabel**)calloc(5, sizeof(QLabel*));

    QStringList tempList;

    tempList << QString("") << QString("") << QString("") << QString("") << QString("");

    if(fastUpdate)
    {
        // do nothing!!
    }
    else
    {

        for(int rel = 0; rel < m_data->m_relationsList.size(); rel++)
        {
            m_data->m_relationsList[rel].myWidget = NULL;
            m_data->m_relationsList[rel].firstElementRow = -1;
            m_data->m_relationsList[rel].secondElementRow = -1;
        }

        for(int geo = 0; geo < m_rowHash.size(); geo++)
        {
            QTreeWidgetItem* currentGeometry = topLevelItem(geo);
            QVector<ito::int16> relationIdxVec;
            relationIdxVec.reserve(24);
            for(int rel = 0; rel < m_data->m_relationsList.size(); rel++)
            {
                if(m_data->m_relationsList[rel].firstElementIdx == (int)m_rowHash[geo].cells[0] && m_data->m_relationsList[rel].type != 0)
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

            for(int childIdx = 0; childIdx < currentGeometry->childCount(); childIdx++)
            {
                /*
                for(int i = 0; i < 5; i++)
                {
                    elements[i] = (QLabel*) (itemWidget(currentGeometry->child(childIdx), i));
                    if(elements[i] == NULL)
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

                for(int i = 0; i < 5; i++)
                {
                    currentGeometry->child(childIdx)->setText(i, "");
                }

                m_data->m_relationsList[relationIdxVec[childIdx]].myWidget = currentGeometry->child(childIdx);

                int curRel = relationIdxVec[childIdx];
                int idx = m_data->m_relationsList[curRel].type & 0x0FFF;

                idx = idx < m_data->m_relationNames.length() ? idx : 0;
                //elements[0]->setText(m_data->m_relationNames[idx]);
                currentGeometry->child(childIdx)->setText(0, m_data->m_relationNames[idx]);

                int idx2 = m_data->m_relationsList[curRel].secondElementIdx;

                int secondType = 0;

                m_data->m_relationsList[curRel].firstElementRow = geo;

                m_data->m_relationsList[curRel].secondElementRow = -1;

                for(int geo2 = 0; geo2 < geo; geo2++)
                {
                    if(idx2 ==  (ito::int32)m_rowHash[geo2].cells[0])
                    {
                        m_data->m_relationsList[curRel].secondElementRow = geo2;
                        secondType = (ito::int32)(m_rowHash[geo2].cells[1]) & 0x0000FFFF;
                    }
                }

                for(int geo2 = geo + 1; geo2 < this->m_rowHash.size(); geo2++)
                {
                    if(idx2 ==  (ito::int32)m_rowHash[geo2].cells[0])
                    {
                        m_data->m_relationsList[curRel].secondElementRow = geo2;
                        secondType = (ito::int32)(m_rowHash[geo2].cells[1]) & 0x0000FFFF;
                    }
                }

                secondType = secondType > 11 ? 0 : secondType; 

                //if(idx2 > - 1 && secondType > 0) elements[1]->setText(QString(primitivNames[secondType]).append(QString::number(idx2)));
                //else elements[1]->setText("");

                if(idx2 > - 1 && secondType > 0) currentGeometry->child(childIdx)->setText(1, QString(primitivNames[secondType]).append(QString::number(idx2)));
                else currentGeometry->child(childIdx)->setText(1, "");

            }

        }

        for(int rel = 0; rel < m_data->m_relationsList.size(); rel++)
        {
            if(m_data->m_relationsList[rel].myWidget == NULL)
            {
                continue;
            }

            switch(m_data->m_relationsList[rel].type & 0x0FFF)
            {
            case tRadius:
                m_data->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/radius.png"));
                m_data->m_relationsList[rel].myWidget->setText(1, "");
                break;
            case tAngle:
                m_data->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/angle.png"));
                break;
            case tDistance:
                m_data->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/distance.png"));
                break;
            case tIntersection:
            {
                m_data->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/intersec.png"));
                break;
            }
            case tLength:
            {
                m_data->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/length.png"));
                m_data->m_relationsList[rel].myWidget->setText(1, ""); 
                break;
            }
            case tArea:
                m_data->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/area.png"));
                m_data->m_relationsList[rel].myWidget->setText(1, ""); 
                break;
            default:

                break;
            }
        }

    }

    QString resultString("");
    resultString.reserve(50);

    for(int rel = 0; rel < m_data->m_relationsList.size(); rel++)
    {
        ito::float32* first;
        ito::float32* second;
        bool check;
        
        /*
        for(int col = 1; col < 5; col++)
        {
            elements[col] = (QLabel*) (itemWidget(m_data->m_relationsList[rel].myWidget, col));
        }
        */
        resultString = "NaN";

        /*
        if(elements[2] == NULL)
        {
            continue;
        }
        */

        if(m_data->m_relationsList[rel].myWidget == NULL)
        {
            continue;
        }

        if(m_data->m_relationsList[rel].type & tExtern)
        {
            resultString = QString("%1%2").arg(QString::number(m_data->m_relationsList[rel].extValue)).arg(m_data->m_valueLabel);
            //elements[2]->setText(resultString);
            m_data->m_relationsList[rel].myWidget->setText(2, resultString); 
            continue;
        }
        else if(m_data->m_relationsList[rel].firstElementRow > -1)
        {
            first = m_rowHash[m_data->m_relationsList[rel].firstElementRow].cells;
        }
        else
        {
            //elements[2]->setText(resultString);
            m_data->m_relationsList[rel].myWidget->setText(2, resultString); 
            continue;
        }

        if(m_data->m_relationsList[rel].type == tRadius)
        {
            //m_data->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/radius.png"));
            check = calculateRadius(first, m_data->m_relationsList[rel].extValue);
            resultString = QString("%1%2").arg(QString::number(m_data->m_relationsList[rel].extValue)).arg(m_data->m_valueLabel);
            //elements[1]->setText("");
            //m_data->m_relationsList[rel].myWidget->setText(1, ""); 
        }
        else if(m_data->m_relationsList[rel].type == tLength)
        {
            //m_data->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/length.png"));
            check = calculateLength(first, m_data->m_relationsList[rel].extValue);
            resultString = QString("%1%2").arg(QString::number(m_data->m_relationsList[rel].extValue)).arg(m_data->m_valueLabel);
            //m_data->m_relationsList[rel].myWidget->setText(1, ""); 
        }
        else
        {
            if(m_data->m_relationsList[rel].secondElementRow > -1)
            {
                second = m_rowHash[m_data->m_relationsList[rel].secondElementRow].cells;

                switch(m_data->m_relationsList[rel].type & 0x0FFF)
                {
                case tAngle:
                    //m_data->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/angle.png"));
                    check = calculateAngle(first, second, m_data->m_relationsList[rel].extValue);
                    resultString = QString("%1 %2").arg(QString::number(m_data->m_relationsList[rel].extValue)).arg(QChar(248));
                    break;
                case tDistance:
                    //m_data->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/distance.png"));
                    check = calculateDistance(first, second, m_data->m_relationsList[rel].extValue);
                    resultString = QString("%1 %2").arg(QString::number(m_data->m_relationsList[rel].extValue)).arg(m_data->m_valueLabel);
                    break;
                case tIntersection:
                {
                    cv::Vec3f val;
                    //m_data->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/intersec.png"));
                    check = calculateIntersections(first, second, val);
                    resultString = QString("%1, %2, %3 [%4]").arg(QString::number(val[0])).arg(QString::number(val[1])).arg(QString::number(val[2])).arg(m_data->m_valueLabel);
                    break;
                }
                case tArea:
                    //check = calculateArea(first, m_data->m_relationsList[i].extValue);
                    //resultString = QString("%1%2²").arg(QString::number(m_data->m_relationsList[i].extValue)).arg(m_data->m_valueLabel);
                    //elements[1]->setText("");
                    break;
                default:
                    //elements[2]->setText(resultString);
                    m_data->m_relationsList[rel].myWidget->setText(2, resultString); 
                    continue;
                }
            }
            else
            {
                //elements[2]->setText(resultString);
                m_data->m_relationsList[rel].myWidget->setText(2, resultString); 
                continue;
            }
        }        
        //elements[2]->setText(resultString);
        m_data->m_relationsList[rel].myWidget->setText(2, resultString); 
    }
    //free(elements);
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateAngle(ito::float32 *first, ito::float32 *second, ito::float32 &angle)
{
    if(((ito::uint32)(first[1]) & 0x0000FFFF )!= ito::PrimitiveContainer::tLine && ((ito::uint32)(first[1]) & 0x0000FFFF)!= ito::PrimitiveContainer::tPolygon)
    {
        angle = std::numeric_limits<ito::float32>::signaling_NaN();
        return false;
    }
    if(((ito::uint32)(second[1]) & 0x0000FFFF)!= ito::PrimitiveContainer::tLine && ((ito::uint32)(second[1]) & 0x0000FFFF)!= ito::PrimitiveContainer::tPolygon)
    {
        angle = std::numeric_limits<ito::float32>::signaling_NaN();
        return false;
    }
    cv::Vec3f firstVector(first[5] - first[2], first[6] - first[3], first[7] - first[4]);
    cv::Vec3f secondVector(second[5] - second[2], second[6] - second[3], second[7] - second[4]);

    ito::float32 abs = (sqrt(pow(firstVector[0],2) + pow(firstVector[1],2) + pow(firstVector[2],2)) * sqrt(pow(secondVector[0],2) + pow(secondVector[1],2) + pow(secondVector[2],2)));

    if(ito::dObjHelper::isNotZero(abs))
    {
        angle = acos(firstVector.dot(secondVector) / abs) * 180 / 3.14159265358979323846;
        return true;    
    }
    angle = std::numeric_limits<ito::float32>::signaling_NaN();
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateDistance(ito::float32 *first, ito::float32 *second, ito::float32 &distance)
{
    cv::Vec3f lineDirVector;
    cv::Vec3f linePosVector;
    cv::Vec3f pointDirVector;
    cv::Vec3f pointPosVector;

    // distance of two points or two circles or combination
    if(((ito::uint32)(first[1]) & 0x0000FFFF) == ito::PrimitiveContainer::tPoint || ((ito::uint32)(first[1]) & 0x0000FFFF) == ito::PrimitiveContainer::tCircle &&
       ((ito::uint32)(second[1]) & 0x0000FFFF) == ito::PrimitiveContainer::tPoint || ((ito::uint32)(second[1]) & 0x0000FFFF) == ito::PrimitiveContainer::tCircle)
    {
        pointPosVector = cv::Vec3f(first[2] - second[2], first[3] - second[3], first[4] - second[4]);
        distance = sqrt( pow(pointPosVector[0],2) + pow(pointPosVector[1],2) + pow(pointPosVector[2],2) );
        return true;
    }

    // distance of line to points or circles
    if(((ito::uint32)(first[1]) & 0x0000FFFF) == ito::PrimitiveContainer::tLine && ((ito::uint32)(second[1]) & 0x0000FFFF ) != ito::PrimitiveContainer::tLine && ((ito::uint32)(second[1]) & 0x0000FFFF ) != ito::PrimitiveContainer::tRetangle)
    {
        lineDirVector = cv::Vec3f(first[5] - first[2], first[6] - first[3], first[7] - first[4]);
        linePosVector = cv::Vec3f(first[2], first[3], first[4]);
        pointPosVector = cv::Vec3f(second[2], second[3], second[4]);
    }
    else if((ito::uint32)(second[1]) == ito::PrimitiveContainer::tLine && (ito::uint32)(first[1]) != ito::PrimitiveContainer::tLine && (ito::uint32)(first[1]) != ito::PrimitiveContainer::tRetangle)
    {
        lineDirVector = cv::Vec3f(second[5] - second[2], second[6] - second[3], second[7] - second[4]);
        linePosVector = cv::Vec3f(second[2], second[3], second[4]);
        pointPosVector = cv::Vec3f(first[2], first[3], first[4]);
    }
    else
    {
        distance = std::numeric_limits<ito::float32>::signaling_NaN();
        return false;
    }

    if(!ito::dObjHelper::isNotZero(lineDirVector[0]) && !ito::dObjHelper::isNotZero(lineDirVector[1]) && !ito::dObjHelper::isNotZero(lineDirVector[2]))
    {
        distance = std::numeric_limits<ito::float32>::signaling_NaN();
        return false;
    }

    ito::float32 lambda = (pointPosVector.dot(lineDirVector) - lineDirVector.dot(linePosVector) /  lineDirVector.dot(lineDirVector)); 

    pointDirVector = (lambda * lineDirVector + linePosVector) - pointPosVector;

    distance = (sqrt(pow(pointDirVector[0],2) + pow(pointDirVector[1],2) + pow(pointDirVector[2],2)) * sqrt(pow(pointDirVector[0],2) + pow(pointDirVector[1],2) + pow(pointDirVector[2],2)));
    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateRadius(ito::float32 *first, ito::float32 &radius)
{
    if(((ito::uint32)(first[1]) & 0x0000FFFF) == ito::PrimitiveContainer::tCircle)
    {
        radius = first[5];
        return true;
    }
    else if((ito::uint32)(first[1]) == ito::PrimitiveContainer::tElipse)
    {
        radius = (first[5] +  first[6])/2;   
        return true;
    }
    radius = std::numeric_limits<ito::float32>::signaling_NaN();
    return false;
}
//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateLength(ito::float32 *first, ito::float32 &length)
{
    if(((ito::uint32)(first[1]) & 0x0000FFFF) != ito::PrimitiveContainer::tLine)
    {
        length = std::numeric_limits<ito::float32>::signaling_NaN();
        return false;
    }

    length = sqrt(pow(first[2] - first[5], 2)  + pow(first[3] - first[6], 2) + pow(first[4] - first[7], 2));

    return true;
}
//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateIntersections(ito::float32 *first, ito::float32 *second, cv::Vec3f &point)
{

    if(((ito::uint32)(first[1]) & 0x0000FFFF) != ito::PrimitiveContainer::tLine || ((ito::uint32)(second[1]) & 0x0000FFFF) != ito::PrimitiveContainer::tLine)
    {
        point[0] = std::numeric_limits<ito::float32>::signaling_NaN();
        point[1] = std::numeric_limits<ito::float32>::signaling_NaN();
        point[2] = std::numeric_limits<ito::float32>::signaling_NaN();
        return false;
    }

    cv::Vec3f firstLineDirVector(first[5] - first[2], first[6] - first[3], first[7] - first[4]);
    cv::Vec3f firstLinePosVector(first[2], first[3], first[4]);
    cv::Vec3f secondLineDirVector(second[5] - second[2], second[6] - second[3], second[7] - second[4]);
    cv::Vec3f secondLinePosVector(second[2], second[3], second[4]);

    ito::float32 absFst = sqrt(pow(firstLineDirVector[0],2) + pow(firstLineDirVector[1],2) + pow(firstLineDirVector[2],2));
    ito::float32 absSec = sqrt(pow(secondLineDirVector[0],2) + pow(secondLineDirVector[1],2) + pow(secondLineDirVector[2],2));

    if(!ito::dObjHelper::isNotZero(absFst) ||  !ito::dObjHelper::isNotZero(absSec))
    {
        point[0] = std::numeric_limits<ito::float32>::signaling_NaN();
        point[1] = std::numeric_limits<ito::float32>::signaling_NaN();
        point[2] = std::numeric_limits<ito::float32>::signaling_NaN();
        return false;
    }

    firstLineDirVector *= 1/absFst;
    secondLineDirVector *= 1/absSec;

    ito::float32 lambda = 0.0;
    ito::float32 kappa  = 0.0;

    // Vectors are the same we have to check if the positions vectors are on the same line
    if( ito::dObjHelper::isNotZero(firstLineDirVector[0] - secondLineDirVector[0]) &&
        ito::dObjHelper::isNotZero(firstLineDirVector[1] - secondLineDirVector[1]) &&
        ito::dObjHelper::isNotZero(firstLineDirVector[2] - secondLineDirVector[2])) 
    {
        secondLinePosVector -= firstLinePosVector;
        lambda = secondLinePosVector[0] / firstLinePosVector[0];
        if(  ito::dObjHelper::isNotZero(secondLinePosVector[1] / firstLinePosVector[1] - lambda)
          && ito::dObjHelper::isNotZero(secondLinePosVector[2] / firstLinePosVector[2] - lambda))
        {
            point = firstLinePosVector;

            return true;
        }
        else
        {
            point[0] = std::numeric_limits<ito::float32>::signaling_NaN();
            point[1] = std::numeric_limits<ito::float32>::signaling_NaN();
            point[2] = std::numeric_limits<ito::float32>::signaling_NaN();
            return true;
        }
    }
    else if(ito::dObjHelper::isNotZero(firstLinePosVector[2]) && 
            ito::dObjHelper::isNotZero(secondLinePosVector[2]) &&
            ito::dObjHelper::isNotZero(firstLineDirVector[2]) && 
            ito::dObjHelper::isNotZero(secondLineDirVector[2])) // is a two dimensional problem
    {
        secondLinePosVector -= firstLinePosVector;
        lambda = secondLinePosVector[0] / firstLinePosVector[0];


        return true;
    }
    else // otherwise we have do do it the hard way
    {
    
        return false;
    }

    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::updateLabels()
{
  
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::refreshPlot(const ito::DataObject* dataObj)
{

    bool changed = false;
    bool clear = false;
    bool identical = false;
    int cols = 0;
    int dims = 0;
   

    if(dataObj)
    {
        dims = dataObj->getDims();
        identical = true;
        if(dataObj->getDims() == 0)
        {
            clear = true;
            identical = false;
        }
        else if(dataObj->getType() != ito::tFloat32)
        {
            m_lastRetVal = ito::RetVal(ito::retError, 0,tr("DataObject must be ito::float32").toAscii().data());
            identical = false;            
        }
        else if(dataObj->getSize(dims-1) < 2)
        {
            m_lastRetVal = ito::RetVal(ito::retError, 0,tr("DataObject has not enough columns").toAscii().data());
            identical = false;
        }
        else
        {
            cv::Mat* scrMat = (cv::Mat*)(dataObj->get_mdata()[dataObj->seekMat(0)]);

            bool found = false;
            
            cols = std::min(scrMat->cols, (int)(sizeof(geometricPrimitives) / 4));

            ito::float32* srcPtr;

            if(scrMat->rows == m_rowHash.size())
            {
                for(int dcnt = 0; dcnt < m_rowHash.size(); dcnt++)
                {
                    srcPtr = scrMat->ptr<ito::float32>(dcnt);
                    if((m_rowHash[dcnt].cells[0] != srcPtr[0]) || ((int)m_rowHash[dcnt].cells[1] != (int)srcPtr[1]) )
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

            if(!identical)
            {
                geometricPrimitives newVal;

                for(int dcnt = 0; dcnt < m_rowHash.size(); dcnt++)
                {
                    found = false;
                    for(int scnt = 0; scnt < scrMat->rows; scnt++)
                    {
                        srcPtr = scrMat->ptr<ito::float32>(scnt);
                        if(m_rowHash[dcnt].cells[0] == srcPtr[0])
                        {
                            found = true;
                            break;
                        }
                    }

                    if(!found)
                    {
                        changed = true;
                        m_rowHash.remove(dcnt);
                    }
                }

                for(int scnt = 0; scnt < scrMat->rows; scnt++)
                {
                    found = false;
                    srcPtr = scrMat->ptr<ito::float32>(scnt);
                    for(int dcnt = 0; dcnt < m_rowHash.size(); dcnt++)
                    {
                   
                        if(m_rowHash[dcnt].cells[0] == srcPtr[0])
                        {
                            //std::fill(m_rowHash[dcnt].cells, m_rowHash[dcnt].cells + sizeof(geometricPrimitives), std::numeric_limits<ito::float32>::signaling_NaN());
                            std::fill(m_rowHash[dcnt].cells, m_rowHash[dcnt].cells + ELEMENTLENGTH, std::numeric_limits<ito::float32>::signaling_NaN());
                            memcpy(m_rowHash[dcnt].cells, srcPtr, sizeof(ito::float32) * cols);
                            found = true;
                            break;
                        }
                    }

                    if(!found && (((int)(srcPtr[1]) & 0x0000FFFF)!= 0))
                    {
                        changed = true;
                        memcpy(newVal.cells, srcPtr, sizeof(ito::float32) * cols);
                        m_rowHash.insert(scnt, newVal);
                    }

                }
            }
        }


    }
     
    if(identical)
    {
        cv::Mat* scrMat = (cv::Mat*)(dataObj->get_mdata()[dataObj->seekMat(0)]);
        ito::float32* srcPtr;

        for(int dcnt = 0; dcnt < m_rowHash.size(); dcnt++)
        {
            srcPtr = scrMat->ptr<ito::float32>(dcnt);
            memset(m_rowHash[dcnt].cells, 0, sizeof(geometricPrimitives));
            memcpy(m_rowHash[dcnt].cells, srcPtr, sizeof(ito::float32) * cols);
            setPrimitivElement(dcnt, true, m_rowHash[dcnt].cells);
        }
        updateRelationShips(true);
    }
    else if(changed)
    {
        this->clear();

        
        setColumnCount(5);
        setColumnWidth(0, 128);
        setColumnWidth(1, 72);
        setColumnWidth(2, 48);
        setColumnWidth(3, 48);
        setColumnWidth(4, 48);
    
        setIconSize(QSize(24, 24));

        for(int dcnt = 0; dcnt < m_rowHash.size(); dcnt++)
        {
            QStringList tempList;
            tempList << QString("") << QString("") << QString("") << QString("") << QString("");
            QTreeWidgetItem* temp = new QTreeWidgetItem(this, tempList);
            addTopLevelItem(temp);
            setPrimitivElement(dcnt, false, m_rowHash[dcnt].cells);
        }

        updateRelationShips(false);
    }
    else
    {
        updateRelationShips(true);
    }

}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTreeWidget::writeToCSV(const QFileInfo &fileName, const bool asTable)
{
    if(fileName.exists() && !fileName.isWritable())
    {
        return ito::RetVal(ito::retError, 0, tr("could not write csv-data to file %1").arg(fileName.fileName()).toAscii().data());
    }

    QFile saveFile(fileName.filePath());

    saveFile.open(QIODevice::OpenModeFlag::WriteOnly | QIODevice::OpenModeFlag::Text);

    QByteArray outBuffer;
    outBuffer.reserve(200);

    for(int geo = 0; geo < topLevelItemCount(); geo++)
    {
        outBuffer.clear();
        QTreeWidgetItem *curItem = topLevelItem(geo);
        
        outBuffer.append(curItem->text(0));
        for(int col = 1; col < this->columnCount(); col++)
        {
            if(curItem->text(col).isEmpty() && !asTable) continue;
            outBuffer.append(',');
            outBuffer.append(curItem->text(col));
        }
        

        if(asTable)
        {

            outBuffer.append('\n');
            saveFile.write(outBuffer);
            int relCount = curItem->childCount();

            for(int rel = 0; rel < relCount; rel ++)
            {
                outBuffer.clear();
                outBuffer.append(curItem->text(0));
                for(int col = 0; col < this->columnCount(); col++)
                {
                    outBuffer.append(',');
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
            for(int rel = 0; rel < relCount; rel ++)
            {
                for(int col = 0; col < this->columnCount() -1; col++)
                {
                    if(curItem->child(rel)->text(col).isEmpty()) continue;
                    outBuffer.append(',');
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
/*
//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::keyPressEvent ( QKeyEvent * event )
{
    
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::mousePressEvent ( QMouseEvent * event )
{
   
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::mouseMoveEvent ( QMouseEvent * event )
{
   
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::mouseReleaseEvent ( QMouseEvent * event )
{
    
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::contextMenuEvent(QContextMenuEvent * event)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTreeWidget::setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue)
{
 

    return ito::retOk;
}
*/
//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::setZoomerEnable(const bool checked)
{
 
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::setPickerEnable(const bool checked)
{
   
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::setPannerEnable(const bool checked)
{
   
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::updateScaleValues(bool recalculateBoundaries /*= false*/)
{
   
}


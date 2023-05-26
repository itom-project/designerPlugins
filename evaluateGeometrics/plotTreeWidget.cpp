/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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
#include <qtransform.h>

double PlotTreeWidget::quietNaN = std::numeric_limits<double>::quiet_NaN();

//----------------------------------------------------------------------------------------------------------------------------------
PlotTreeWidget::PlotTreeWidget(QMenu *contextMenu, InternalInfo *data, QWidget * parent) :
    QTreeWidget(parent),
    m_contextMenu(contextMenu),
    m_pParent(parent),
    m_lastRetVal(ito::retOk)
{
    m_pData = data;
    //this is the border between the canvas and the axes and the overall mainwindow
    setContentsMargins(2,2,2,2);

    setColumnCount(5);

    setEditTriggers(QAbstractItemView::NoEditTriggers);

    m_rowHash.clear();

    setColumnCount(5);
    setColumnWidth(0, 142);
    setColumnWidth(1, 72);
    setColumnWidth(2, 72);
    setColumnWidth(3, 72);
    setColumnWidth(4, 72);

    setIconSize(QSize(24, 24));


    connect(this, SIGNAL(itemPressed(QTreeWidgetItem*,int)), SLOT(itemPressed(QTreeWidgetItem*,int)));
}

//----------------------------------------------------------------------------------------------------------------------------------
PlotTreeWidget::~PlotTreeWidget()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTreeWidget::init()
{
    QFont titleFont = apiGetFigureSetting(parent(), "titleFont", QFont("Verdana", 12), NULL).value<QFont>();
    QFont labelFont = apiGetFigureSetting(parent(), "labelFont", QFont("Verdana", 10), NULL).value<QFont>();
    QFont axisFont = apiGetFigureSetting(parent(), "axisFont", QFont("Verdana", 10), NULL).value<QFont>();

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::displayShape(const int row, const bool update, const ito::Shape &shape)
{
    QString coordsString("[%1, %2]");

    if (!update)
    {
        if (m_pData->m_shapeTypeNames.contains(shape.type()))
        {
            topLevelItem(row)->setText(0, tr("%1 (%2)").arg(m_pData->m_shapeTypeNames[shape.type()]).arg(shape.index()));
        }
        else
        {
            topLevelItem(row)->setText(0, tr("unknown type (%1)").arg(shape.index()));
        }

        switch (shape.type())
        {
            default:
            case ito::Shape::Invalid:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/notype.png"));
                break;
            }
            case ito::Shape::Point:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/marker.png"));
                break;
            }
            case ito::Shape::Line:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/pntline.png"));
                break;
            }
            case ito::Shape::Circle:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/circle.png"));
                break;
            }
            case ito::Shape::Ellipse:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/ellipse.png"));
                break;
            }
            case ito::Shape::Rectangle:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/rectangle.png"));
                break;
            }
            case ito::Shape::Square:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/square.png"));
                break;
            }
            case ito::Shape::Polygon:
            {
                topLevelItem(row)->setIcon(0, QIcon(":/itomDesignerPlugins/plot/icons/polygon.png"));
                break;
            }
        }
    }

    QPolygonF contour;
    const QTransform &trafo = shape.rtransform();
    const QPolygonF &basePoints = shape.rbasePoints();

    switch (shape.type())
    {
        default:
        case ito::Shape::Invalid:
            break;

        case ito::Shape::Point:
        {
            contour = shape.contour(true);
            topLevelItem(row)->setText(1, QString(coordsString)
                .arg(QString::number(contour[0].x(), 'f', m_pData->m_numberOfDigits))
                .arg(QString::number(contour[0].y(), 'f', m_pData->m_numberOfDigits)));

            break;
        }
        case ito::Shape::Line:
        {
            contour = shape.contour(true);
            topLevelItem(row)->setText(1, QString(coordsString)
                .arg(QString::number(contour[0].x(), 'f', m_pData->m_numberOfDigits))
                .arg(QString::number(contour[0].y(), 'f', m_pData->m_numberOfDigits)));

            topLevelItem(row)->setText(2, QString(coordsString)
                .arg(QString::number(contour[1].x(), 'f', m_pData->m_numberOfDigits))
                .arg(QString::number(contour[1].y(), 'f', m_pData->m_numberOfDigits)));
            break;
        }
        case ito::Shape::Circle:
        {
            {
                QPointF center = 0.5 * (trafo.map(basePoints[0]) + trafo.map(basePoints[1]));
                QPointF radius = 0.5 * (trafo.map(basePoints[1]) - trafo.map(basePoints[0]));
                topLevelItem(row)->setText(1, QString(coordsString)
                    .arg(QString::number(center.x(), 'f', m_pData->m_numberOfDigits))
                    .arg(QString::number(center.y(), 'f', m_pData->m_numberOfDigits)));

                topLevelItem(row)->setText(2, QString("r = %1 %2")
                    .arg(QString::number(std::abs(radius.x()), 'f', m_pData->m_numberOfDigits))
                    .arg(m_pData->m_valueUnit));
            }

            break;
        }
        case ito::Shape::Ellipse:
        {
            {
                QPointF center = 0.5 * (trafo.map(basePoints[0]) + trafo.map(basePoints[1]));
                QPointF radius = 0.5 * (trafo.map(basePoints[1]) - trafo.map(basePoints[0]));
                topLevelItem(row)->setText(1, QString(coordsString)
                    .arg(QString::number(center.x(), 'f', m_pData->m_numberOfDigits))
                    .arg(QString::number(center.y(), 'f', m_pData->m_numberOfDigits)));

                topLevelItem(row)->setText(2, QString("a,b = %1 %3, %2 %3")
                    .arg(QString::number(std::abs(radius.x()), 'f', m_pData->m_numberOfDigits))
                    .arg(QString::number(std::abs(radius.y()), 'f', m_pData->m_numberOfDigits))
                    .arg(m_pData->m_valueUnit));

                if (trafo.isRotating())
                {
                    topLevelItem(row)->setText(3, QString("alpha = %1%2")
                        .arg(QString::number(shape.rotationAngleDeg(), 'f', m_pData->m_numberOfDigits))
                        .arg(QChar(0x00B0)));
                }
                else
                {
                    topLevelItem(row)->setText(3, "");
                }
                break;
            }
        }
        case ito::Shape::Rectangle:
        {
            {
                QPointF p1 = trafo.map(basePoints[0]);
                QPointF p2 = trafo.map(basePoints[1]);
                topLevelItem(row)->setText(1, QString(coordsString)
                    .arg(QString::number(p1.x(), 'f', m_pData->m_numberOfDigits))
                    .arg(QString::number(p1.y(), 'f', m_pData->m_numberOfDigits)));

                topLevelItem(row)->setText(2, QString(coordsString)
                    .arg(QString::number(p2.x(), 'f', m_pData->m_numberOfDigits))
                    .arg(QString::number(p2.y(), 'f', m_pData->m_numberOfDigits)));

                if (trafo.isRotating())
                {
                    topLevelItem(row)->setText(3, QString("alpha = %1%2")
                        .arg(QString::number(shape.rotationAngleDeg(), 'f', m_pData->m_numberOfDigits))
                        .arg(QChar(0x00B0)));
                }
                else
                {
                    topLevelItem(row)->setText(3, "");
                }
            }
            break;
        }
        case ito::Shape::Square:
        {
            {
                QPointF center = 0.5 * (trafo.map(basePoints[0]) + trafo.map(basePoints[1]));
                topLevelItem(row)->setText(1, QString(coordsString)
                    .arg(QString::number(center.x(), 'f', m_pData->m_numberOfDigits))
                    .arg(QString::number(center.y(), 'f', m_pData->m_numberOfDigits)));

                topLevelItem(row)->setText(2, QString("a = %1 %2")
                    .arg(QString::number(std::abs((basePoints[1] - basePoints[0]).x()), 'f', m_pData->m_numberOfDigits))
                    .arg(m_pData->m_valueUnit));

                if (trafo.isRotating())
                {
                    topLevelItem(row)->setText(3, QString("alpha = %1%2")
                        .arg(QString::number(shape.rotationAngleDeg(), 'f', m_pData->m_numberOfDigits))
                        .arg(QChar(0x00B0)));
                }
                else
                {
                    topLevelItem(row)->setText(3, "");
                }
            }
            break;
        }
        case ito::Shape::Polygon:
        {
            /*topLevelItem(row)->setText(1, QString(coordsString)
                                        .arg(QString::number(val[2], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[3], 'f', m_pData->m_numberOfDigits)));

            topLevelItem(row)->setText(2, QString(coordsString)
                                        .arg(QString::number(val[4], 'f', m_pData->m_numberOfDigits))
                                        .arg(QString::number(val[5], 'f', m_pData->m_numberOfDigits)));


             topLevelItem(row)->setText(3, QString("%1 [%2]")
                                         .arg(QString::number(val[7], 'f', m_pData->m_numberOfDigits))
                                         .arg(QString::number(val[8], 'f', m_pData->m_numberOfDigits)));*/
            break;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::updateRelationShips(const bool fastUpdate)
{

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
        }

        // first check if we have left over relations and remove them
        for (int rel = 0; rel < m_pData->m_relationsList.size(); rel++)
        {
            if (!keys.contains(m_pData->m_relationsList[rel].firstElementIdx)
                || (m_pData->m_relationsList[rel].secondElementIdx != -1) && !keys.contains(m_pData->m_relationsList[rel].secondElementIdx))
            {
                m_pData->m_relationsList.remove(rel);
            }
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
                for (int i = 0; i < 5; i++)
                {
                    currentGeometry->child(childIdx)->setText(i, "");
                }

                m_pData->m_relationsList[relationIdxVec[childIdx]].myWidget = currentGeometry->child(childIdx);

                int curRel = relationIdxVec[childIdx];
                int idx = m_pData->m_relationsList[curRel].type & 0x0FFF;

                idx = idx < m_pData->m_relationNames.length() ? idx : 0;
                currentGeometry->child(childIdx)->setText(0, m_pData->m_relationNames[idx]);
                currentGeometry->child(childIdx)->setData(0, Qt::UserRole, childIdx);

                int idx2 = m_pData->m_relationsList[curRel].secondElementIdx;

                int secondType = 0;

                for (int geo2 = 0; geo2 < geo; geo2++)
                {
                    if (idx2 ==  keys[geo2])
                    {
                        secondType = m_rowHash[keys[geo2]].type();
                    }
                }

                for (int geo2 = geo + 1; geo2 < keys.size(); geo2++)
                {
                    if (idx2 ==  keys[geo2])
                    {
                        secondType = m_rowHash[keys[geo2]].type();
                    }
                }

                if (idx2 > - 1 && secondType > 0 && m_pData->m_shapeTypeNames.contains(secondType))
                {
                    currentGeometry->child(childIdx)->setText(1, QString(m_pData->m_shapeTypeNames[secondType]).append(QString::number(idx2)));
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
                    m_pData->m_relationsList[rel].myWidget->setBackground(1, QColor(255, 255, 255));
                break;

                case tAngle:
                    m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/angle.png"));
                    if (m_pData->m_relationsList[rel].secondElementIdx < 0)
                    {
                        m_pData->m_relationsList[rel].myWidget->setBackground(
                            1, QColor(255, 200, 200));
                    }
                    else
                    {
                        m_pData->m_relationsList[rel].myWidget->setBackground(
                            1, QColor(255, 255, 255));
                    }
                break;

                case tDistance:
                    m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/distance.png"));
                    if (m_pData->m_relationsList[rel].secondElementIdx < 0)
                    {
                        m_pData->m_relationsList[rel].myWidget->setBackground(
                            1, QColor(255, 200, 200));
                    }
                    else
                    {
                        m_pData->m_relationsList[rel].myWidget->setBackground(
                            1, QColor(255, 255, 255));
                    }
                break;

                case tIntersection:
                {
                    m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/intersec.png"));
                    if (m_pData->m_relationsList[rel].secondElementIdx < 0)
                    {
                        m_pData->m_relationsList[rel].myWidget->setBackground(
                            1, QColor(255, 200, 200));
                    }
                    else
                    {
                        m_pData->m_relationsList[rel].myWidget->setBackground(
                            1, QColor(255, 255, 255));
                    }
                }
                break;

                case tLength:
                {
                    m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/length.png"));
                    m_pData->m_relationsList[rel].myWidget->setText(1, "");
                    m_pData->m_relationsList[rel].myWidget->setBackground(1, QColor(255, 255, 255));
                }
                break;

                case tArea:
                    m_pData->m_relationsList[rel].myWidget->setIcon(0, QIcon(":/evaluateGeometrics/icons/area.png"));
                    m_pData->m_relationsList[rel].myWidget->setText(1, "");
                    m_pData->m_relationsList[rel].myWidget->setBackground(1, QColor(255, 255, 255));
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
        ito::Shape first;
        ito::Shape second;
        bool check;
        resultString = "NaN";

        if (m_pData->m_relationsList[rel].myWidget == NULL)
        {
            continue;
        }

        if (m_pData->m_relationsList[rel].type & tExtern)
        {
            resultString = QString("%1 %2").arg(QString::number(m_pData->m_relationsList[rel].extValue, 'f', m_pData->m_numberOfDigits))
                                           .arg(m_pData->m_valueUnit);

            m_pData->m_relationsList[rel].myWidget->setText(2, resultString);
            continue;
        }
        else if (m_rowHash.contains(m_pData->m_relationsList[rel].firstElementIdx))
        {
            first = m_rowHash[m_pData->m_relationsList[rel].firstElementIdx];
        }
        else
        {
            m_pData->m_relationsList[rel].myWidget->setText(2, resultString);
            m_pData->m_relationsList[rel].myWidget->setBackground(2, QColor(255, 200, 200));
            continue;
        }

        if (m_pData->m_relationsList[rel].type == tRadius)
        {
            check = calculateRadius(first, m_pData->m_relationsList[rel].extValue);

            resultString = QString("%1 %2").arg(QString::number(m_pData->m_relationsList[rel].extValue, 'f', m_pData->m_numberOfDigits))
                                           .arg(m_pData->m_valueUnit);
        }
        else if (m_pData->m_relationsList[rel].type == tLength)
        {
            check = calculateLength(first, m_pData->m_relationsList[rel].extValue);
            resultString = QString("%1 %2").arg(QString::number(m_pData->m_relationsList[rel].extValue, 'f', m_pData->m_numberOfDigits))
                                           .arg(m_pData->m_valueUnit);
        }
        else if (m_pData->m_relationsList[rel].type == tArea)
        {
            check = calculateArea(first, m_pData->m_relationsList[rel].extValue);
            resultString = QString("%1 %2%3").arg(QString::number(m_pData->m_relationsList[rel].extValue, 'f', m_pData->m_numberOfDigits))
                                            .arg(m_pData->m_valueUnit)
                                            .arg(QChar(0x00B2));
        }
        else
        {
            if (m_rowHash.contains(m_pData->m_relationsList[rel].secondElementIdx))
            {
                second = m_rowHash[m_pData->m_relationsList[rel].secondElementIdx];

                switch(m_pData->m_relationsList[rel].type & 0x0FFF)
                {
                    case tAngle:
                        check = calculateAngle(first, second, m_pData->m_relationsList[rel].extValue);
                        resultString = QString("%1 %2").arg(QString::number(m_pData->m_relationsList[rel].extValue, 'f', m_pData->m_numberOfDigits))
                                                      .arg(QChar(0x00B0));
                    break;

                    case tDistance:
                        check = calculateDistance(first, second, m_pData->m_relationsList[rel].extValue);
                        resultString = QString("%1 %2").arg(QString::number(m_pData->m_relationsList[rel].extValue, 'f', m_pData->m_numberOfDigits))
                                                      .arg(m_pData->m_valueUnit);
                    break;
                    /*
                    case tIntersection:
                    {
                        cv::Vec3f val;
                        check = calculateIntersections(first, second, val);
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
            m_pData->m_relationsList[rel].myWidget->setBackground(2, QColor(255, 255, 255));
        }
        else
        {
            m_pData->m_relationsList[rel].myWidget->setBackground(2, QColor(255, 200, 200));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateAngle(const ito::Shape &first, const ito::Shape &second, ito::float32 &angle)
{
    if (first.type() != ito::Shape::Line || second.type() != ito::Shape::Line)
    {
        angle = quietNaN;
        return false;
    }

    QPolygonF contour1 = first.contour(true);
    QPolygonF contour2 = second.contour(true);
    QLineF line1(contour1[0], contour1[1]);
    QLineF line2(contour2[0], contour2[1]);
    QPointF intersection;

#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
    if (line1.intersects(line2, &intersection) != QLineF::NoIntersection)
#else
    if (line1.intersect(line2, &intersection) != QLineF::NoIntersection)
#endif
    {
        angle = line1.angleTo(line2);
        return true;
    }
    angle = quietNaN;
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateDistance(const ito::Shape &first, const ito::Shape &second, ito::float32 &distance)
{
    QPointF p1, p2;

    switch (first.type())
    {
        case ito::Shape::Point:
            p1 = first.rtransform().map(first.rbasePoints()[0]);
        break;

        case ito::Shape::Circle:
        case ito::Shape::Ellipse:
            p1 = first.rtransform().map(0.5 * (first.rbasePoints()[0] + first.rbasePoints()[1]));
        break;
    }

    switch (second.type())
    {
        case ito::Shape::Point:
            p2 = second.rtransform().map(second.rbasePoints()[0]);
        break;

        case ito::Shape::Circle:
        case ito::Shape::Ellipse:
            p2 = second.rtransform().map(0.5 * (second.rbasePoints()[0] + second.rbasePoints()[1]));
        break;
    }

    if (!p1.isNull() && !p2.isNull())
    {
        distance = std::abs(QLineF(p1, p2).length());
        return true;
    }
    else if (!p1.isNull() && second.type() == ito::Shape::Line)
    {
        QLineF line(second.rtransform().map(second.rbasePoints()[0]), second.rtransform().map(second.rbasePoints()[1]));
        QPointF line_vector = second.rtransform().map(second.rbasePoints()[1]) - second.rtransform().map(second.rbasePoints()[0]);

        if (line_vector.manhattanLength() == 0.0)
        {
            distance = quietNaN;
            return false;
        }

        QPointF dist_points = p1 - line.p1();
        distance = std::abs((line_vector.x() * dist_points.y() - line_vector.y() * dist_points.x()) / std::sqrt(line_vector.x()*line_vector.x() + line_vector.y()*line_vector.y()));
        return true;
    }
    else if (!p2.isNull() && first.type() == ito::Shape::Line)
    {
        QLineF line(first.rtransform().map(first.rbasePoints()[0]), first.rtransform().map(first.rbasePoints()[1]));
        QPointF line_vector = first.rtransform().map(first.rbasePoints()[1]) - first.rtransform().map(first.rbasePoints()[0]);

        if (line_vector.manhattanLength() == 0.0)
        {
            distance = quietNaN;
            return false;
        }

        QPointF dist_points = p2 - line.p1();
        distance = std::abs((line_vector.x() * dist_points.y() - line_vector.y() * dist_points.x()) / std::sqrt(line_vector.x()*line_vector.x() + line_vector.y()*line_vector.y()));
        return true;
    }

    distance = quietNaN;
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateRadius(const ito::Shape &first, ito::float32 &radius)
{
    switch (first.type())
    {
        case ito::Shape::Circle:
        case ito::Shape::Ellipse:
        {
            QPointF rad = 0.5 * (first.rbasePoints()[1] - first.rbasePoints()[0]);
            radius = 0.5 * (std::abs(rad.x()) + std::abs(rad.y()));
            return true;
        }
        default:
            radius = quietNaN;
            return false;
    }
    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateLength(const ito::Shape &first, ito::float32 &length)
{
    if (first.type() != ito::Shape::Line)
    {
        length = quietNaN;
        return false;
    }

    length = std::abs((QLineF(first.rbasePoints()[0], first.rbasePoints()[1])).length());
    return true;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTreeWidget::calculateArea(const ito::Shape &first, ito::float32 &area)
{
    switch(first.type())
    {
        case ito::Shape::Rectangle:
        case ito::Shape::Square:
        case ito::Shape::Circle:
        case ito::Shape::Ellipse:
        case ito::Shape::Polygon:
            area = first.area();
            return true;
        default:
            area = quietNaN;
            return false;
    }

    return false;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*
bool PlotTreeWidget::calculateIntersections(ito::float32 *first, ito::float32 *second, const bool eval2D, cv::Vec3f &point)
{

    if (((ito::uint32)(first[1]) & ito::PrimitiveContainer::tTypeMask) != ito::Shape::Line || ((ito::uint32)(second[1]) & ito::PrimitiveContainer::tTypeMask) != ito::Shape::Line)
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

    if (!ito::isNotZero(absFst) ||  !ito::isNotZero(absSec))
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
    if (ito::isNotZero(firstLineDirVector[0] - secondLineDirVector[0]) &&
        ito::isNotZero(firstLineDirVector[1] - secondLineDirVector[1]) &&
        ito::isNotZero(firstLineDirVector[2] - secondLineDirVector[2]))
    {
        secondLinePosVector -= firstLinePosVector;
        lambda = secondLinePosVector[0] / firstLinePosVector[0];
        if ( ito::isNotZero(secondLinePosVector[1] / firstLinePosVector[1] - lambda)
          && ito::isNotZero(secondLinePosVector[2] / firstLinePosVector[2] - lambda))
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
            (!ito::isNotZero(firstLinePosVector[2]) &&
            !ito::isNotZero(secondLinePosVector[2]) &&
            !ito::isNotZero(firstLineDirVector[2]) &&
            !ito::isNotZero(secondLineDirVector[2]))) // is a two dimensional problem
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
void PlotTreeWidget::setShapes(const QVector<ito::Shape> &shapes)
{
    bool changed = false;
    bool clear = false;
    bool identical = false;
    int cols = 0;
    int dims = 0;

    identical = true;
    QList<ito::int32> hashKeys = m_rowHash.keys();

    bool found = false;

    if (shapes.size() == hashKeys.size())
    {
        for (int dcnt = 0; dcnt < hashKeys.size(); dcnt++)
        {
            if ((hashKeys[dcnt] != shapes[dcnt].index()) || ((ito::int32)m_rowHash[hashKeys[dcnt]].type() != shapes[dcnt].type()))
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
        // check for stale elements and remove them
        for (int dcnt = 0; dcnt < hashKeys.size(); dcnt++)
        {
            found = false;
            for (int scnt = 0; scnt < shapes.size(); scnt++)
            {
                if (hashKeys[dcnt] == shapes[scnt].index())
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

        for (int scnt = 0; scnt < shapes.size(); scnt++)
        {
            found = false;
            for (int dcnt = 0; dcnt < hashKeys.size(); dcnt++)
            {
                if (hashKeys[dcnt] == shapes[scnt].index())
                {
                    m_rowHash[hashKeys[dcnt]] = shapes[scnt];
                    found = true;
                    break;
                }
            }

            if (!found && shapes[scnt].type() != ito::Shape::Invalid)
            {
                changed = true;

                int idx = 0;
                if (shapes[scnt].index() < 65355 && shapes[scnt].index() > -1)
                {
                    idx = shapes[scnt].index();
                }
                m_rowHash.insert(idx, shapes[scnt]);
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

    if (changed)
    {
        this->clear();
        QList<ito::int32> hashTags = m_rowHash.keys();
        for (int dcnt = 0; dcnt < hashTags.size(); dcnt++)
        {
            QStringList tempList;
            tempList << QString("") << QString("") << QString("") << QString("") << QString("");
            QTreeWidgetItem* temp = new QTreeWidgetItem(this, tempList);
            addTopLevelItem(temp);
            displayShape(dcnt, false, m_rowHash[hashTags[dcnt]]);
        }

        updateRelationShips(false);
    }
    else
    {
        // if we don't do this in case of identical, altered coordinates of elements
        // do not get displayed
        QList<ito::int32> hashTags = m_rowHash.keys();

        for (int dcnt = 0; dcnt < hashTags.size(); dcnt++)
        {
            //m_rowHash[hashTags[dcnt]] = shapes[dcnt];
            m_rowHash[shapes[dcnt].index()] = shapes[dcnt];
            displayShape(dcnt, true, m_rowHash[hashTags[dcnt]]);
        }
        updateRelationShips(true);

        QStringList headers;
        headers << "item" << "" << "" << "" << "";
        this->setHeaderLabels(headers);
    }

    expandAll();
    repaint();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTreeWidget::writeToCSV(const QFileInfo &fileName, const bool asTable)
{
    if (fileName.exists() && !fileName.isWritable())
    {
        return ito::RetVal(ito::retError, 0, tr("could not write csv-data to file %1").arg(fileName.fileName()).toLatin1().data());
    }

    QFile saveFile(fileName.filePath());

    saveFile.open(QIODevice::WriteOnly | QIODevice::Text);

    QByteArray outBuffer;
    outBuffer.reserve(200);

    for (int geo = 0; geo < topLevelItemCount(); geo++)
    {
        outBuffer.clear();
        QTreeWidgetItem *curItem = topLevelItem(geo);

        outBuffer.append(curItem->text(0).toLatin1());
        for (int col = 1; col < this->columnCount(); col++)
        {
            if (curItem->text(col).isEmpty() && !asTable) continue;
            outBuffer.append(", ");
            outBuffer.append(curItem->text(col).toLatin1());
        }

        if (asTable)
        {
            outBuffer.append('\n');
            saveFile.write(outBuffer);
            int relCount = curItem->childCount();

            for (int rel = 0; rel < relCount; rel ++)
            {
                outBuffer.clear();
                outBuffer.append(curItem->text(0).toLatin1());
                for (int col = 0; col < this->columnCount(); col++)
                {
                    outBuffer.append(", ");
                    outBuffer.append(curItem->child(rel)->text(col).toLatin1());
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
                    outBuffer.append(curItem->child(rel)->text(col).toLatin1());
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
        return ito::RetVal(ito::retError, 0, tr("could not write csv-data to file %1").arg(fileName.fileName()).toLatin1().data());
    }

    QFile saveFile(fileName.filePath());

    saveFile.open(QIODevice::WriteOnly | QIODevice::Text);

    QXmlStreamWriter stream(&saveFile);
    QString attrname;

    // utf-8 is always the default codec for XML
    //stream.setCodec("UTF-8");       // Set text codec
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("itomGeometricShapes");
    {
        stream.writeAttribute("href", "http://www.ito.uni-stuttgart.de");

        QHash<ito::int32, ito::Shape >::const_iterator curValue = m_rowHash.constBegin();
        for (int geo = 0; curValue !=  m_rowHash.end(); ++curValue, geo++)
        {
            const ito::Shape &shape = curValue.value();
            const QTransform &trafo = shape.rtransform();
            const QPolygonF &bp = shape.rbasePoints();

            stream.writeStartElement(QString::number(geo));
            stream.writeAttribute("index", QString::number(shape.index()));
            stream.writeAttribute("flags", QString::number(shape.flags()));

            QVector<ito::int16> relationIdxVec;
            relationIdxVec.reserve(24);
            for (int rel = 0; rel < m_pData->m_relationsList.size(); rel++)
            {
                if (m_pData->m_relationsList[rel].firstElementIdx == shape.index() && m_pData->m_relationsList[rel].type != 0)
                {
                    relationIdxVec.append(rel);
                }
            }

            if (m_pData->m_shapeTypeNames.contains(shape.type()))
            {
                stream.writeAttribute("name", m_pData->m_shapeTypeNames[shape.type()]);
            }
            else
            {
                stream.writeAttribute("name", m_pData->m_shapeTypeNames[ito::Shape::Invalid]);
            }

            switch(shape.type())
            {
                case ito::Shape::Point:
                {
                    QPointF p = trafo.map(bp[0]);
                    stream.writeAttribute("x0", QString::number(p.x()));
                    stream.writeAttribute("y0", QString::number(p.y()));
                }
                break;
                case ito::Shape::Line:
                {
                    QPointF p1 = trafo.map(bp[0]);
                    QPointF p2 = trafo.map(bp[1]);
                    stream.writeAttribute("x0", QString::number(p1.x()));
                    stream.writeAttribute("y0", QString::number(p1.y()));
                    stream.writeAttribute("x1", QString::number(p2.x()));
                    stream.writeAttribute("y1", QString::number(p2.x()));
                }
                break;
                case ito::Shape::Ellipse:
                {
                    QPointF p1 = trafo.map(0.5 * (bp[0] + bp[1]));
                    QPointF r = trafo.map(0.5 * (bp[1] - bp[0]));
                    stream.writeAttribute("x0", QString::number(p1.x()));
                    stream.writeAttribute("y0", QString::number(p1.y()));
                    stream.writeAttribute("r1", QString::number(std::abs(r.x())));
                    stream.writeAttribute("r2", QString::number(std::abs(r.y())));
                    stream.writeAttribute("alpha", QString::number(shape.rotationAngleDeg()));
                }
                break;

                case ito::Shape::Circle:
                {
                    QPointF p1 = trafo.map(0.5 * (bp[0] + bp[1]));
                    QPointF r = trafo.map(0.5 * (bp[1] - bp[0]));
                    stream.writeAttribute("x0", QString::number(p1.x()));
                    stream.writeAttribute("y0", QString::number(p1.y()));
                    stream.writeAttribute("r1", QString::number(std::abs(r.x())));
                }
                break;

                case ito::Shape::Rectangle:
                {
                    QPointF p1 = trafo.map(0.5 * (bp[0] + bp[1]));
                    QPointF r = trafo.map(0.5 * (bp[1] - bp[0]));
                    stream.writeAttribute("x0", QString::number(p1.x()));
                    stream.writeAttribute("y0", QString::number(p1.y()));
                    stream.writeAttribute("x1", QString::number(std::abs(r.x())));
                    stream.writeAttribute("y1", QString::number(std::abs(r.y())));
                    stream.writeAttribute("alpha", QString::number(shape.rotationAngleDeg()));
                }
                break;

                case ito::Shape::Square:
                {
                    QPointF p1 = trafo.map(0.5 * (bp[0] + bp[1]));
                    QPointF r = trafo.map(0.5 * (bp[1] - bp[0]));
                    stream.writeAttribute("x0", QString::number(p1.x()));
                    stream.writeAttribute("y0", QString::number(p1.y()));
                    stream.writeAttribute("a", QString::number(std::abs(r.x())));
                    stream.writeAttribute("alpha", QString::number(shape.rotationAngleDeg()));
                }
                break;

                case ito::Shape::Polygon:
                {
                    stream.writeAttribute("Total", QString::number(bp.size()));
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
        return ito::RetVal(ito::retError, 0, tr("could not write csv-data to file %1").arg(fileName.fileName()).toLatin1().data());
    }

    QFile saveFile(fileName.filePath());

    saveFile.open(QIODevice::WriteOnly | QIODevice::Text);

    QByteArray outBuffer;
    outBuffer.reserve(200);

    QHash<ito::int32, ito::Shape >::const_iterator curValue = m_rowHash.constBegin();
    for (int geo = 0; curValue !=  m_rowHash.end(); ++curValue, geo++)
    {
        const ito::Shape &shape = curValue.value();
        const QTransform &trafo = shape.rtransform();
        const QPolygonF &bp = shape.rbasePoints();

        outBuffer.clear();
        outBuffer.append(QByteArray::number(shape.index()));
        for (int i = 0; i < bp.size(); i++)
        {
            outBuffer.append(", ");
            outBuffer.append(QByteArray::number(bp[i].x()));
            outBuffer.append(", ");
            outBuffer.append(QByteArray::number(bp[i].y()));
        }
        outBuffer.append(", ");
        outBuffer.append(QByteArray::number(shape.rotationAngleDeg()));
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

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::updateGeometricShapes()
{
    QHash<ito::int32, ito::Shape>::const_iterator it = m_rowHash.constBegin();
    int c = 0;
    while (it != m_rowHash.constEnd())
    {
        displayShape(c, true, it.value());
        c++;
        ++it;
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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTreeWidget::updateElement(const ito::int32 &idx, const ito::Shape &shape)
{
    QHash<ito::int32, ito::Shape>::iterator it = m_rowHash.begin();
    int c = 0;
    while (it != m_rowHash.end())
    {
        if (it.value().index() == idx)
        {
            it.value() = shape;
            displayShape(c, true, shape);
            break;
        }

        c++;
        ++it;
    }

    updateRelationShips(true);
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::itemPressed(QTreeWidgetItem *item, int column)
{
    QStringList headers;
    // simple elements first
    if (item->text(0).startsWith("point"))
    {
        headers << "Item" << "x, y" << "" << "" << "";
    }
    else if (item->text(0).startsWith("line"))
    {
        headers << "Item" << "x0, y0" << "X1/Y1" << "" << "";
    }
    else if (item->text(0).startsWith("square"))
    {
        headers << "Item" << "x0, y0" << "x1, y1" << "" << "";
    }
    else if (item->text(0).startsWith("rectangle"))
    {
        headers << "Item" << "x0, y0" << "x1, y1" << "" << "";
    }
    else if (item->text(0).startsWith("circle"))
    {
        headers << "Item" << "cx, cy" << "radius" << "" << "";
    }
    else if (item->text(0).startsWith("ellipse"))
    {
        headers << "Item" << "cx, cy" << "rx, ry" << "" << "";
    }
    // now relations
    else if (item->text(0).startsWith("distance"))
    {
        headers << "Item" << "Item2" << "d" << "" << "";
    }
    else if (item->text(0).startsWith("length"))
    {
        headers << "Item" << "" << "l" << "" << "";
    }
    else if (item->text(0).startsWith("radius"))
    {
        headers << "Item" << "" << "r" << "" << "";
    }
    else if (item->text(0).startsWith("area"))
    {
        headers << "Item" << "" << "a" << "" << "";
    }
    else if (item->text(0).startsWith("angle"))
    {
        headers << "Item" << "Item2" << "phi" << "" << "";
    }
    else if (item->text(0).startsWith("intersection"))
    {
        headers << "Item" << "Item2" << "x, y" << "" << "";
    }
    else
    {
        headers << "item" << "" << "" << "" << "";
    }
    this->setHeaderLabels(headers);
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTreeWidget::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu(this);
    foreach(QAction *act, m_contextMenu->actions())
    {
        menu.addAction(act);
    }
    menu.exec(event->globalPos());
}

//----------------------------------------------------------------------------------------------------------------------------------

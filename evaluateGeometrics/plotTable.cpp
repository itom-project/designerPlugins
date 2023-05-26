/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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
#include "plotTable.h"
#include "common/../DataObject/dataObjectFuncs.h"
#include "common/sharedStructuresGraphics.h"
#include "common/apiFunctionsGraphInc.h"

#include <qdebug.h>
#include <qmessagebox.h>
#include <QDoubleSpinBox>
#include <qlayout.h>


//----------------------------------------------------------------------------------------------------------------------------------
PlotTable::PlotTable(QMenu *contextMenu, InternalInfo *data, QWidget * parent) :
    QTabWidget(parent),
    m_contextMenu(contextMenu),
    m_xDirect(false),
    m_yDirect(false),
    m_pParent(parent),
    m_state(stateIdle),
    m_lastRetVal(ito::retOk)
{
    m_data = data;
    //this is the border between the canvas and the axes and the overall mainwindow
    setContentsMargins(5,5,5,5);

    m_firstTab = new QWidget();
    m_secondTab = new QWidget();

    m_geometrics = new QTableWidget();
    m_relations = new QTableWidget();

    QHBoxLayout *pLayout1 = new QHBoxLayout(m_firstTab);
    pLayout1->addWidget(m_geometrics);
    m_firstTab->setLayout(pLayout1);

    QHBoxLayout *pLayout2 = new QHBoxLayout(m_secondTab);
    pLayout2->addWidget(m_relations);
    m_secondTab->setLayout(pLayout2);

    addTab(m_firstTab, QObject::tr("geomatric data"));
    addTab(m_secondTab, QObject::tr("calculations"));

    m_geometrics->setColumnCount(10);
    m_geometrics->setRowCount(2);
    m_geometrics->setColumnWidth(0,32);
    m_geometrics->setColumnWidth(1,32);


    m_relations->setColumnCount(5);
    m_relations->setRowCount(2);
    m_relations->setColumnWidth(0,32);
    //m_relations->setColumnWidth(1,32);


    m_geometrics->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_relations->setEditTriggers(QAbstractItemView::NoEditTriggers);

}

//----------------------------------------------------------------------------------------------------------------------------------
PlotTable::~PlotTable()
{
    m_geometrics->deleteLater();
    m_relations->deleteLater();

    delete m_secondTab;
    delete m_firstTab;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTable::init()
{
    QFont titleFont = apiGetFigureSetting(parent(), "titleFont", QFont("Helvetica",12),NULL).value<QFont>();
    QFont labelFont = apiGetFigureSetting(parent(), "labelFont", QFont("Helvetica",12),NULL).value<QFont>();
    QFont axisFont = apiGetFigureSetting(parent(), "axisFont", QFont("Helvetica",10),NULL).value<QFont>();

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//void PlotTable::setLabels(const QString &title, const QString &valueLabel, const QString &axisLabel)
//{
//
//}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::setPrimitivElement(const int row, const bool update, const int cols, ito::float32 *val)
{
    int neededCols = 0;
    int colsToFill = cols;

    if(!update)
    {
        QLabel* firstElement  = new QLabel(QString::number((ito::uint32)(val[0])), m_geometrics, 0);
        firstElement->setAlignment(Qt::AlignRight | Qt::AlignCenter);

        QLabel* secondElement = new QLabel("", m_geometrics, 0);
        m_geometrics->setCellWidget(row, 0, firstElement);
        m_geometrics->setCellWidget(row, 1, secondElement);


        switch (((ito::uint32)(val[1])) & 0x0000FFFF)
        {
            default:
            case ito::PrimitiveContainer::tNoType:
            {
                neededCols = 2;
                return;
            }
            case ito::PrimitiveContainer::tPoint:
            {
                ((QLabel*)(m_geometrics->cellWidget(row, 1)))->setPixmap(QPixmap(":/itomDesignerPlugins/plot/icons/marker.png"));
                QDoubleSpinBox* x0 = new QDoubleSpinBox(m_geometrics);
                x0->setPrefix("x = ");
                x0->setStatusTip("position");
                x0->setReadOnly(true);
                x0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                x0->setSuffix(m_data->m_valueLabel);
                x0->setFrame(false);
                m_geometrics->setCellWidget(row, 2, x0);

                QDoubleSpinBox* y0 = new QDoubleSpinBox(m_geometrics);
                y0->setPrefix("y = ");
                y0->setStatusTip("position");
                y0->setReadOnly(true);
                y0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                y0->setSuffix(m_data->m_valueLabel);
                y0->setFrame(false);
                m_geometrics->setCellWidget(row, 3, y0);

                QDoubleSpinBox* z0 = new QDoubleSpinBox(m_geometrics);
                z0->setPrefix("z = ");
                z0->setStatusTip("position");
                z0->setReadOnly(true);
                z0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                z0->setSuffix(m_data->m_valueLabel);
                z0->setFrame(false);
                m_geometrics->setCellWidget(row, 4, z0);

                neededCols = 5;
                break;
            }
            case ito::PrimitiveContainer::tLine:
            {
                ((QLabel*)(m_geometrics->cellWidget(row, 1)))->setPixmap(QPixmap(":/itomDesignerPlugins/plot/icons/pntline.png"));
                QDoubleSpinBox* x0 = new QDoubleSpinBox(m_geometrics);
                x0->setPrefix("x = ");
                x0->setStatusTip("1. point");
                x0->setReadOnly(true);
                x0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                x0->setSuffix(m_data->m_valueLabel);
                x0->setFrame(false);
                m_geometrics->setCellWidget(row, 2, x0);

                QDoubleSpinBox* y0 = new QDoubleSpinBox(m_geometrics);
                y0->setPrefix("y = ");
                y0->setStatusTip("1. point");
                y0->setReadOnly(true);
                y0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                y0->setSuffix(m_data->m_valueLabel);
                y0->setFrame(false);
                m_geometrics->setCellWidget(row, 3, y0);

                QDoubleSpinBox* z0 = new QDoubleSpinBox(m_geometrics);
                z0->setPrefix("z = ");
                z0->setStatusTip("1. point");
                z0->setReadOnly(true);
                z0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                z0->setSuffix(m_data->m_valueLabel);
                z0->setFrame(false);
                m_geometrics->setCellWidget(row, 4, z0);

                QDoubleSpinBox* x1 = new QDoubleSpinBox(m_geometrics);
                x1->setPrefix("x = ");
                x1->setStatusTip("2. point");
                x1->setReadOnly(true);
                x1->setButtonSymbols(QAbstractSpinBox::NoButtons);
                x1->setSuffix(m_data->m_valueLabel);
                x1->setFrame(false);
                m_geometrics->setCellWidget(row, 5, x1);

                QDoubleSpinBox* y1 = new QDoubleSpinBox(m_geometrics);
                y1->setPrefix("y = ");
                y1->setStatusTip("2. point");
                y1->setReadOnly(true);
                y1->setButtonSymbols(QAbstractSpinBox::NoButtons);
                y1->setSuffix(m_data->m_valueLabel);
                y1->setFrame(false);
                m_geometrics->setCellWidget(row, 6, y1);

                QDoubleSpinBox* z1 = new QDoubleSpinBox(m_geometrics);
                z1->setPrefix("z = ");
                z1->setStatusTip("2. point");
                z1->setReadOnly(true);
                z1->setButtonSymbols(QAbstractSpinBox::NoButtons);
                z1->setSuffix(m_data->m_valueLabel);
                z1->setFrame(false);
                m_geometrics->setCellWidget(row, 7, z1);

                neededCols = 8;
                break;
            }
            case ito::PrimitiveContainer::tElipse:
            {
                ((QLabel*)(m_geometrics->cellWidget(row, 1)))->setPixmap(QPixmap(":/itomDesignerPlugins/plot/icons/elipse.png"));
                QDoubleSpinBox* x0 = new QDoubleSpinBox(m_geometrics);
                x0->setPrefix("x = ");
                x0->setStatusTip("center position");
                x0->setReadOnly(true);
                x0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                x0->setSuffix(m_data->m_valueLabel);
                x0->setFrame(false);
                m_geometrics->setCellWidget(row, 2, x0);

                QDoubleSpinBox* y0 = new QDoubleSpinBox(m_geometrics);
                y0->setPrefix("y = ");
                y0->setStatusTip("center position");
                y0->setReadOnly(true);
                y0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                y0->setSuffix(m_data->m_valueLabel);
                y0->setFrame(false);
                m_geometrics->setCellWidget(row, 3, y0);

                QDoubleSpinBox* z0 = new QDoubleSpinBox(m_geometrics);
                z0->setPrefix("z = ");
                z0->setStatusTip("center position");
                z0->setReadOnly(true);
                z0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                z0->setSuffix(m_data->m_valueLabel);
                z0->setFrame(false);
                m_geometrics->setCellWidget(row, 4, z0);

                QDoubleSpinBox* r0 = new QDoubleSpinBox(m_geometrics);
                r0->setPrefix("r1 = ");
                r0->setStatusTip("1. radius");
                r0->setReadOnly(true);
                r0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                r0->setSuffix(m_data->m_valueLabel);
                r0->setFrame(false);
                m_geometrics->setCellWidget(row, 5, r0);

                QDoubleSpinBox* r1 = new QDoubleSpinBox(m_geometrics);
                r1->setPrefix("r2 = ");
                r1->setStatusTip("2. radius");
                r1->setReadOnly(true);
                r1->setButtonSymbols(QAbstractSpinBox::NoButtons);
                r1->setSuffix(m_data->m_valueLabel);
                r1->setFrame(false);
                m_geometrics->setCellWidget(row, 6, r1);

                //neededCols = 8;
                neededCols = 7;
                break;
            }

            case ito::PrimitiveContainer::tCircle:
            {
                ((QLabel*)(m_geometrics->cellWidget(row, 1)))->setPixmap(QPixmap(":/itomDesignerPlugins/plot/icons/circle.png"));
                QDoubleSpinBox* x0 = new QDoubleSpinBox(m_geometrics);
                x0->setPrefix("x = ");
                x0->setStatusTip("center position");
                x0->setReadOnly(true);
                x0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                x0->setSuffix(m_data->m_valueLabel);
                x0->setFrame(false);
                m_geometrics->setCellWidget(row, 2, x0);

                QDoubleSpinBox* y0 = new QDoubleSpinBox(m_geometrics);
                y0->setPrefix("y = ");
                y0->setStatusTip("center position");
                y0->setReadOnly(true);
                y0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                y0->setSuffix(m_data->m_valueLabel);
                y0->setFrame(false);
                m_geometrics->setCellWidget(row, 3, y0);

                QDoubleSpinBox* z0 = new QDoubleSpinBox(m_geometrics);
                z0->setPrefix("z = ");
                z0->setStatusTip("center position");
                z0->setReadOnly(true);
                z0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                z0->setSuffix(m_data->m_valueLabel);
                z0->setFrame(false);
                m_geometrics->setCellWidget(row, 4, z0);

                QDoubleSpinBox* r0 = new QDoubleSpinBox(m_geometrics);
                r0->setPrefix("r1 = ");
                r0->setStatusTip("radius");
                r0->setReadOnly(true);
                r0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                r0->setSuffix(m_data->m_valueLabel);
                r0->setFrame(false);
                m_geometrics->setCellWidget(row, 5, r0);

                //neededCols = 6;
                neededCols = 6;
                break;
            }

            case ito::PrimitiveContainer::tRetangle:
            {
                ((QLabel*)(m_geometrics->cellWidget(row, 1)))->setPixmap(QPixmap(":/itomDesignerPlugins/plot/icons/rectangle.png"));
                QDoubleSpinBox* x0 = new QDoubleSpinBox(m_geometrics);
                x0->setPrefix("x = ");
                x0->setStatusTip("first edge");
                x0->setReadOnly(true);
                x0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                x0->setSuffix(m_data->m_valueLabel);
                x0->setFrame(false);
                m_geometrics->setCellWidget(row, 2, x0);

                QDoubleSpinBox* y0 = new QDoubleSpinBox(m_geometrics);
                y0->setPrefix("y = ");
                y0->setStatusTip("1. edge");
                y0->setReadOnly(true);
                y0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                y0->setSuffix(m_data->m_valueLabel);
                y0->setFrame(false);
                m_geometrics->setCellWidget(row, 3, y0);

                QDoubleSpinBox* z0 = new QDoubleSpinBox(m_geometrics);
                z0->setPrefix("z = ");
                z0->setStatusTip("1. edge");
                z0->setReadOnly(true);
                z0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                z0->setSuffix(m_data->m_valueLabel);
                z0->setFrame(false);
                m_geometrics->setCellWidget(row, 4, z0);

                QDoubleSpinBox* x1 = new QDoubleSpinBox(m_geometrics);
                x1->setPrefix("x = ");
                x1->setStatusTip("2. edge");
                x1->setReadOnly(true);
                x1->setButtonSymbols(QAbstractSpinBox::NoButtons);
                x1->setSuffix(m_data->m_valueLabel);
                x1->setFrame(false);
                m_geometrics->setCellWidget(row, 5, x1);

                QDoubleSpinBox* y1 = new QDoubleSpinBox(m_geometrics);
                y1->setPrefix("y = ");
                y1->setStatusTip("2. edge");
                y1->setReadOnly(true);
                y1->setButtonSymbols(QAbstractSpinBox::NoButtons);
                y1->setSuffix(m_data->m_valueLabel);
                y1->setFrame(false);
                m_geometrics->setCellWidget(row, 6, y1);

                QDoubleSpinBox* z1 = new QDoubleSpinBox(m_geometrics);
                z1->setPrefix("z = ");
                z1->setStatusTip("2. edge");
                z1->setReadOnly(true);
                z1->setButtonSymbols(QAbstractSpinBox::NoButtons);
                z1->setSuffix(m_data->m_valueLabel);
                z1->setFrame(false);
                m_geometrics->setCellWidget(row, 7, z1);

                QDoubleSpinBox* alpha0 = new QDoubleSpinBox(m_geometrics);
                alpha0->setPrefix("alpha = ");
                alpha0->setStatusTip("Angle between global x-axis and retangle x-axis");
                alpha0->setReadOnly(true);
                alpha0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                alpha0->setSuffix(m_data->m_valueLabel);
                alpha0->setFrame(false);
                m_geometrics->setCellWidget(row, 8, alpha0);

                neededCols = 9;
                break;
            }

            case ito::PrimitiveContainer::tSquare:
            {
                ((QLabel*)(m_geometrics->cellWidget(row, 1)))->setPixmap(QPixmap(":/itomDesignerPlugins/plot/icons/square.png"));
                QDoubleSpinBox* x0 = new QDoubleSpinBox(m_geometrics);
                x0->setPrefix("x = ");
                x0->setStatusTip("center position");
                x0->setReadOnly(true);
                x0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                x0->setSuffix(m_data->m_valueLabel);
                x0->setFrame(false);
                m_geometrics->setCellWidget(row, 2, x0);

                QDoubleSpinBox* y0 = new QDoubleSpinBox(m_geometrics);
                y0->setPrefix("y = ");
                y0->setStatusTip("center position");
                y0->setReadOnly(true);
                y0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                y0->setSuffix(m_data->m_valueLabel);
                y0->setFrame(false);
                m_geometrics->setCellWidget(row, 3, y0);

                QDoubleSpinBox* z0 = new QDoubleSpinBox(m_geometrics);
                z0->setPrefix("z = ");
                z0->setStatusTip("center position");
                z0->setReadOnly(true);
                z0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                z0->setSuffix(m_data->m_valueLabel);
                z0->setFrame(false);
                m_geometrics->setCellWidget(row, 4, z0);

                QDoubleSpinBox* a0 = new QDoubleSpinBox(m_geometrics);
                a0->setPrefix("a = ");
                a0->setStatusTip("Side length");
                a0->setReadOnly(true);
                a0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                a0->setSuffix(m_data->m_valueLabel);
                a0->setFrame(false);
                m_geometrics->setCellWidget(row, 5, a0);

                QDoubleSpinBox* alpha0 = new QDoubleSpinBox(m_geometrics);
                alpha0->setPrefix("alpha = ");
                alpha0->setStatusTip("Angle between global x-axis and retangle x-axis");
                alpha0->setReadOnly(true);
                alpha0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                alpha0->setSuffix(m_data->m_valueLabel);
                alpha0->setFrame(false);
                m_geometrics->setCellWidget(row, 6, alpha0);

                neededCols = 7;
                break;
            }

            case ito::PrimitiveContainer::tPolygon:
            {
                ((QLabel*)(m_geometrics->cellWidget(row, 1)))->setPixmap(QPixmap(":/itomDesignerPlugins/plot/icons/polygon.png"));
                QDoubleSpinBox* x0 = new QDoubleSpinBox(m_geometrics);
                x0->setPrefix("x = ");
                x0->setStatusTip("1. point");
                x0->setReadOnly(true);
                x0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                x0->setSuffix(m_data->m_valueLabel);
                x0->setFrame(false);
                m_geometrics->setCellWidget(row, 2, x0);

                QDoubleSpinBox* y0 = new QDoubleSpinBox(m_geometrics);
                y0->setPrefix("y = ");
                y0->setStatusTip("1. point");
                y0->setReadOnly(true);
                y0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                y0->setSuffix(m_data->m_valueLabel);
                y0->setFrame(false);
                m_geometrics->setCellWidget(row, 3, y0);

                QDoubleSpinBox* z0 = new QDoubleSpinBox(m_geometrics);
                z0->setPrefix("z = ");
                z0->setStatusTip("1. point");
                z0->setReadOnly(true);
                z0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                z0->setSuffix(m_data->m_valueLabel);
                z0->setFrame(false);
                m_geometrics->setCellWidget(row, 4, z0);

                QDoubleSpinBox* x1 = new QDoubleSpinBox(m_geometrics);
                x1->setPrefix("x = ");
                x1->setStatusTip("direction vector");
                x1->setReadOnly(true);
                x1->setButtonSymbols(QAbstractSpinBox::NoButtons);
                x1->setSuffix(m_data->m_valueLabel);
                x1->setFrame(false);
                m_geometrics->setCellWidget(row, 5, x1);

                QDoubleSpinBox* y1 = new QDoubleSpinBox(m_geometrics);
                y1->setPrefix("y = ");
                y1->setStatusTip("direction vector");
                y1->setReadOnly(true);
                y1->setButtonSymbols(QAbstractSpinBox::NoButtons);
                y1->setSuffix(m_data->m_valueLabel);
                y1->setFrame(false);
                m_geometrics->setCellWidget(row, 6, y1);

                QDoubleSpinBox* z1 = new QDoubleSpinBox(m_geometrics);
                z1->setPrefix("z = ");
                z1->setStatusTip("direction vector");
                z1->setReadOnly(true);
                z1->setButtonSymbols(QAbstractSpinBox::NoButtons);
                z1->setSuffix(m_data->m_valueLabel);
                z1->setFrame(false);
                m_geometrics->setCellWidget(row, 7, z1);

                QDoubleSpinBox* n0 = new QDoubleSpinBox(m_geometrics);
                n0->setPrefix("n = ");
                n0->setStatusTip("point number");
                n0->setReadOnly(true);
                n0->setButtonSymbols(QAbstractSpinBox::NoButtons);
                n0->setSuffix(m_data->m_valueLabel);
                n0->setFrame(false);
                m_geometrics->setCellWidget(row, 8, n0);

                QDoubleSpinBox* nTotal = new QDoubleSpinBox(m_geometrics);
                nTotal->setPrefix("N = ");
                nTotal->setStatusTip("total points");
                nTotal->setReadOnly(true);
                nTotal->setButtonSymbols(QAbstractSpinBox::NoButtons);
                nTotal->setSuffix(m_data->m_valueLabel);
                nTotal->setFrame(false);
                m_geometrics->setCellWidget(row, 9, nTotal);

                neededCols = 10;
                break;
            }
        }
    }

    if(neededCols < cols)
    {
        colsToFill = neededCols;
    }

    QDoubleSpinBox* curSpin = NULL;

    QString style = "background-color: ";

    for(int i = 2; i < colsToFill; i++)
    {
        curSpin = ((QDoubleSpinBox*) m_geometrics->cellWidget(row, i));
        curSpin->setValue(val[i]);

        curSpin->setStyleSheet(style);
    }

    style = QString("background-color: red");

    for(int i = colsToFill; i < neededCols; i++)
    {
        curSpin = ((QDoubleSpinBox*) m_geometrics->cellWidget(row, i));
        curSpin->setValue(-0.0);

        curSpin->setStyleSheet(style);
    }

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::updateRelationShips(const bool fastUpdate)
{
    if(m_data->m_relationsList.size() == m_relations->rowCount() && fastUpdate)
    {
        // do nothing!!
    }
    else
    {
        if(m_data->m_relationsList.size() != m_relations->rowCount())
        {
            m_relations->setRowCount(m_data->m_relationsList.size());
        }


        for(int i = 0; i < m_relations->rowCount(); i++)
        {
            QLabel* test = (QLabel*)(m_relations->cellWidget(i, 0));
            int idx = m_data->m_relationsList[i].type & 0x0FFF;
            idx = idx < m_data->m_relationNames.length() ? idx : 0;

            if(!test)
            {
                m_relations->setCellWidget(i, 0, new QLabel("", m_relations, 0));
                test = (QLabel*)(m_relations->cellWidget(i, 0));
                test->setAlignment(Qt::AlignLeft | Qt::AlignCenter);
            }
            test->setText(m_data->m_relationNames[idx]);

            idx = m_data->m_relationsList[i].firstElementIdx;
            int idx2 = m_data->m_relationsList[i].secondElementIdx;

            int firstType  = 0;
            int secondType = 0;

            m_data->m_relationsList[i].firstElementRow = -1;
            m_data->m_relationsList[i].secondElementRow = -1;

            for(int k = 0; k < this->m_rowHash.size(); k++)
            {
                if(idx ==  (ito::int32)m_rowHash[k].cells[0])
                {
                    m_data->m_relationsList[i].firstElementRow = k;
                    firstType = (ito::int32)(m_rowHash[k].cells[1]) & 0x0000FFFF;
                }

                if(idx2 ==  (ito::int32)m_rowHash[k].cells[0])
                {
                    m_data->m_relationsList[i].secondElementRow = k;
                    secondType = (ito::int32)(m_rowHash[k].cells[1]) & 0x0000FFFF;
                }
            }

            firstType = firstType < 10 ? firstType : 0;
            secondType = secondType < 10 ? secondType : 0;

            test = (QLabel*)(m_relations->cellWidget(i, 1));
            if(!test)
            {
                m_relations->setCellWidget(i, 1, new QLabel("", m_relations, 0));
                test = (QLabel*)(m_relations->cellWidget(i, 1));
                test->setAlignment(Qt::AlignLeft | Qt::AlignCenter);
            }

            if(idx > - 1) test->setText(QString(primitivNames[firstType]).append(QString::number(idx)));
            else test->setText(QString(primitivNames[firstType]));

            test = (QLabel*)(m_relations->cellWidget(i, 2));
            if(!test)
            {
                m_relations->setCellWidget(i, 2, new QLabel("", m_relations, 0));
                test = (QLabel*)(m_relations->cellWidget(i, 2));
                test->setAlignment(Qt::AlignLeft | Qt::AlignCenter);
            }
            if(idx2 > - 1) test->setText(QString(primitivNames[secondType]).append(QString::number(idx2)));
            else test->setText(QString(primitivNames[secondType]));


            test = (QLabel*)(m_relations->cellWidget(i, 3));
            if(!test)
            {
                m_relations->setCellWidget(i, 3, new QLabel("", m_relations, 0));

                test = (QLabel*)(m_relations->cellWidget(i, 3));
                test->setAlignment(Qt::AlignRight | Qt::AlignCenter);
            }
        }

    }

    QString resultString("");
    resultString.reserve(50);

    for(int i = 0; i < m_data->m_relationsList.size(); i++)
    {
        ito::float32* first;
        ito::float32* second;
        bool check;

        QLabel* resultLabel = (QLabel*)(m_relations->cellWidget(i, 3));

        resultString = "NaN";

        if(m_data->m_relationsList[i].type & tExtern)
        {
            resultString = QString("%1%2").arg(QString::number(m_data->m_relationsList[i].extValue)).arg(m_data->m_valueLabel);

            resultLabel->setText(resultString);
            continue;
        }
        else if(m_data->m_relationsList[i].firstElementRow > -1)
        {
            first = m_rowHash[m_data->m_relationsList[i].firstElementRow].cells;
        }
        else
        {
            resultLabel->setText(resultString);
            continue;
        }

        if(m_data->m_relationsList[i].type == tRadius)
        {
            check = calculateRadius(first, m_data->m_relationsList[i].extValue);
            resultString = QString("%1%2").arg(QString::number(m_data->m_relationsList[i].extValue)).arg(m_data->m_valueLabel);
        }
        else
        {
            if(m_data->m_relationsList[i].secondElementRow > -1)
            {
                second = m_rowHash[m_data->m_relationsList[i].secondElementRow].cells;

                switch(m_data->m_relationsList[i].type & 0x0FFF)
                {
                case tAngle:
                    check = calculateAngle(first, second, m_data->m_relationsList[i].extValue);
                    resultString = QString("%1 %2").arg(QString::number(m_data->m_relationsList[i].extValue)).arg(QChar(248));
                    break;
                case tDistance:
                    check = calculateDistance(first, second, m_data->m_relationsList[i].extValue);
                    resultString = QString("%1 %2").arg(QString::number(m_data->m_relationsList[i].extValue)).arg(m_data->m_valueLabel);
                    break;
                case tIntersection:
                {
                    cv::Vec3f val;
                    check = calculateIntersections(first, second, val);
                    resultString = QString("%1, %2, %3 [%4]").arg(QString::number(val[0])).arg(QString::number(val[1])).arg(QString::number(val[2])).arg(m_data->m_valueLabel);
                    break;
                }
                case tLength:
                {
                    check = calculateLength(first, m_data->m_relationsList[i].extValue);
                    resultString = QString("%1%2").arg(QString::number(m_data->m_relationsList[i].extValue)).arg(m_data->m_valueLabel);
                    break;
                }
                case tArea:
                    //check = calculateArea(first, m_data->m_relationsList[i].extValue);
                    //resultString = QString("%1%2²").arg(QString::number(m_data->m_relationsList[i].extValue)).arg(m_data->m_valueLabel);
                    //break;
                default:
                    resultLabel->setText(resultString);
                    continue;
                }

            }
            else
            {
                resultLabel->setText(resultString);
                continue;
            }
        }


        resultLabel->setText(resultString);

    }
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
bool PlotTable::calculateAngle(ito::float32 *first, ito::float32 *second, ito::float32 &angle)
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
bool PlotTable::calculateDistance(ito::float32 *first, ito::float32 *second, ito::float32 &distance)
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
bool PlotTable::calculateRadius(ito::float32 *first, ito::float32 &radius)
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
bool PlotTable::calculateLength(ito::float32 *first, ito::float32 &length)
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
bool PlotTable::calculateIntersections(ito::float32 *first, ito::float32 *second, cv::Vec3f &point)
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
void PlotTable::updateLabels()
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::refreshPlot(const ito::DataObject* dataObj)
{

    bool changed = false;
//    bool clear = false;
    bool identical = false;
    int cols = 0;
    int dims = 0;


    if(dataObj)
    {
        dims = dataObj->getDims();
        identical = true;
        if(dataObj->getDims() == 0)
        {
//            clear = true;
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
                            memset(m_rowHash[dcnt].cells, 0, sizeof(geometricPrimitives));
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
            setPrimitivElement(dcnt, true, cols, m_rowHash[dcnt].cells);
        }
        updateRelationShips(true);
    }
    else if(changed)
    {
        m_geometrics->clear();
        m_geometrics->setColumnCount(10);
        m_geometrics->setRowCount(m_rowHash.size());

        m_geometrics->setColumnWidth(0,32);
        m_geometrics->setColumnWidth(1,32);
        //m_geometrics->setAlternatingRowColors(true);

        for(int dcnt = 0; dcnt < m_rowHash.size(); dcnt++)
        {
            setPrimitivElement(dcnt, false, cols, m_rowHash[dcnt].cells);
        }

        updateRelationShips(false);
    }
    else
    {
        updateRelationShips(true);
    }

}
/*
//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::keyPressEvent ( QKeyEvent * event )
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::mousePressEvent ( QMouseEvent * event )
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::mouseMoveEvent ( QMouseEvent * event )
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::mouseReleaseEvent ( QMouseEvent * event )
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::contextMenuEvent(QContextMenuEvent * event)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PlotTable::setInterval(const Qt::Axis axis, const bool autoCalcLimits, const double minValue, const double maxValue)
{


    return ito::retOk;
}
*/
//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::setZoomerEnable(const bool checked)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::setPickerEnable(const bool checked)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::setPannerEnable(const bool checked)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void PlotTable::updateScaleValues(bool recalculateBoundaries /*= false*/)
{

}

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

/*!
 * \file dialogAddRelation.cpp
 * \brief This file contains the definitions for the dialog "addRelation" for the evaluateGeometrics-Widget.
 */

#include <qglobal.h> // needed for qt version check

#include "dialogAddRelation.h"
#include "evaluateGeometrics.h"
#include "plotTreeWidget.h"

//-----------------------------------------------------------------------------------------------
DialogAddRelation::DialogAddRelation(const InternalInfo &data, EvaluateGeometricsFigure *egFig, QWidget *parent) :
    QDialog(parent),
    m_evalGeoFig(egFig)
{
    ui.setupUi(this);
    if (egFig == NULL)
        return;

    QVector<ito::Shape> shapes = egFig->getGeometricShapes();
    int setItem = -1;
    int curItem = egFig->getCurrentItem();
    for (int ni = 0; ni < shapes.size(); ni++)
    {
        QString str;
        switch (shapes[ni].type())
        {
            case ito::Shape::Point:
                ui.comboBoxFirst->addItem(str.sprintf("#%d: Point", ni), shapes[ni].index());
                ui.comboBoxSecond->addItem(str.sprintf("#%d: Point", ni), shapes[ni].index());
            break;

            case ito::Shape::Line:
                ui.comboBoxFirst->addItem(str.sprintf("#%d: Line", ni), shapes[ni].index());
                ui.comboBoxSecond->addItem(str.sprintf("#%d: Line", ni), shapes[ni].index());
            break;

            case ito::Shape::Rectangle:
                ui.comboBoxFirst->addItem(str.sprintf("#%d: Rectangle", ni), shapes[ni].index());
                ui.comboBoxSecond->addItem(str.sprintf("#%d: Rectangle", ni), shapes[ni].index());
            break;

            case ito::Shape::Square:
                ui.comboBoxFirst->addItem(str.sprintf("#%d: Square", ni), shapes[ni].index());
                ui.comboBoxSecond->addItem(str.sprintf("#%d: Square", ni), shapes[ni].index());
            break;

            case ito::Shape::Circle:
                ui.comboBoxFirst->addItem(str.sprintf("#%d: Circle", ni), shapes[ni].index());
                ui.comboBoxSecond->addItem(str.sprintf("#%d: Circle", ni), shapes[ni].index());
            break;

            case ito::Shape::Ellipse:
                ui.comboBoxFirst->addItem(str.sprintf("#%d: Ellipse", ni), shapes[ni].index());
                ui.comboBoxSecond->addItem(str.sprintf("#%d: Ellipse", ni), shapes[ni].index());
            break;

            case ito::Shape::Polygon:
                ui.comboBoxFirst->addItem(str.sprintf("#%d: Polygon", ni), shapes[ni].index());
                ui.comboBoxSecond->addItem(str.sprintf("#%d: Polygon", ni), shapes[ni].index());
            break;
        }
        if (shapes[ni].index() == curItem)
            setItem = ni;
    }
    
    ui.comboBoxFirst->setCurrentIndex(setItem);
    ui.comboBoxSecond->setCurrentIndex(setItem);

    ui.comboBoxType->addItem("Radius", 1);
    ui.comboBoxType->addItem("Angle", 2);
    ui.comboBoxType->addItem("Distance", 3);
    ui.comboBoxType->addItem("Intersection", 4);
    ui.comboBoxType->addItem("Length", 5);
    ui.comboBoxType->addItem("Area", 6);

    // make some more or less meaningful preselection of the relation type
    if (setItem >= 0)
    {
        switch (shapes[setItem].type())
        {
        case ito::Shape::Point:
            ui.comboBoxType->setCurrentIndex(2);
            break;

        case ito::Shape::Line:
            ui.comboBoxType->setCurrentIndex(4);
            break;

        case ito::Shape::Rectangle:
            ui.comboBoxType->setCurrentIndex(5);
            break;

        case ito::Shape::Square:
            ui.comboBoxType->setCurrentIndex(5);
            break;

        case ito::Shape::Circle:
            ui.comboBoxType->setCurrentIndex(0);
            break;

        case ito::Shape::Ellipse:
            ui.comboBoxType->setCurrentIndex(0);
            break;

        case ito::Shape::Polygon:
            ui.comboBoxType->setCurrentIndex(5);
            break;
        }
    }

//    QObject::connect(ui.buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
//    QObject::connect(ui.buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
//    QObject::connect(ui.buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(on_buttonBox_clicked(QAbstractButton*)));
}

//-----------------------------------------------------------------------------------------------
/*
void DialogAddRelation::getData(InternalInfo &data)
{
    data.m_relationsList.append(relationShip(ui.comboBoxFirst->currentData().toInt(),
        ui.comboBoxSecond->currentData().toInt(),
        ui.comboBoxType->currentData().toInt()));
}
*/
//-----------------------------------------------------------------------------------------------
void DialogAddRelation::on_buttonBox_clicked(QAbstractButton* btn)
{
    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);
    if (role == QDialogButtonBox::AcceptRole)
    {
        QVariant idx1 = ui.comboBoxFirst->currentData();
        QVariant idx2 = ui.comboBoxSecond->currentData();

        ito::DataObject rObj;
        ito::float64 *dPtr = NULL;
        if (idx1 == idx2)
        {
            rObj.zeros(2, ito::tFloat64);
            dPtr = (ito::float64*)rObj.rowPtr(0, 0);
        }
        else
        {
            rObj.zeros(4, ito::tFloat64);
            dPtr = (ito::float64*)rObj.rowPtr(0, 0);
            dPtr[2] = idx2.toFloat();
        }

        dPtr[0] = idx1.toFloat();    
        dPtr[1] = ui.comboBoxType->currentData().toFloat();

        if (m_evalGeoFig)
        {
            QSharedPointer<ito::DataObject> dObjPtr(new ito::DataObject(rObj));
            m_evalGeoFig->addRelation(dObjPtr);
        }

        accept(); //AcceptRole
    }
    else
    {
        reject(); //close dialog with reject
    }
}

//-----------------------------------------------------------------------------------------------

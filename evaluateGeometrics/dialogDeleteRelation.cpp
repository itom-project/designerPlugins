/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2018, Institut für Technische Optik (ITO),
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

/*!
 * \file dialogDeleteRelation.cpp
 * \brief This file contains the definitions for the dialog "deleteRelation" for the evaluateGeometrics-Widget.
 */

#include <qglobal.h> // needed for qt version check

#include "dialogDeleteRelation.h"
#include "evaluateGeometrics.h"
#include "plotTreeWidget.h"
#include <qmessagebox.h>


//-----------------------------------------------------------------------------------------------
DialogDeleteRelation::DialogDeleteRelation(const InternalInfo &data, EvaluateGeometricsFigure *egFig, QWidget *parent) :
    QDialog(parent),
    m_evalGeoFig(egFig)
{
    ui.setupUi(this);

    if (egFig == NULL)
        return;

    QVector<ito::Shape> shapes = egFig->getGeometricShapes();
    int setItem1 = -1, setItem2 = -1;
    int curItem = egFig->getCurrentItem();
    QSharedPointer<ito::DataObject> curRel = egFig->getCurrentRelation();
    /*
    if (curItem < 0)
    {
        QMessageBox::critical(parent, "Error", "No relationship selected - select relationship first, aborting!");
        return;
    }
    */

    QString str;

    switch ((int)m_evalGeoFig->getShape((*curRel).at<ito::float32>(0, 0)).type())
    {
        case ito::Shape::Point:
            ui.comboBoxFirst->addItem(str.asprintf("#%d: Point", (int)(*curRel).at<ito::float32>(0, 0)), (*curRel).at<ito::float32>(0, 0));
        break;

        case ito::Shape::Line:
            ui.comboBoxFirst->addItem(str.asprintf("#%d: Line", (int)(*curRel).at<ito::float32>(0, 0)), (*curRel).at<ito::float32>(0, 0));
        break;

        case ito::Shape::Rectangle:
            ui.comboBoxFirst->addItem(str.asprintf("#%d: Rectangle", (int)(*curRel).at<ito::float32>(0, 0)), (*curRel).at<ito::float32>(0, 0));
        break;

        case ito::Shape::Square:
            ui.comboBoxFirst->addItem(str.asprintf("#%d: Square", (int)(*curRel).at<ito::float32>(0, 0)), (*curRel).at<ito::float32>(0, 0));
        break;

        case ito::Shape::Circle:
            ui.comboBoxFirst->addItem(str.asprintf("#%d: Circle", (int)(*curRel).at<ito::float32>(0, 0)), (*curRel).at<ito::float32>(0, 0));
        break;

        case ito::Shape::Ellipse:
            ui.comboBoxFirst->addItem(str.asprintf("#%d: Ellipse", (int)(*curRel).at<ito::float32>(0, 0)), (*curRel).at<ito::float32>(0, 0));
        break;

        case ito::Shape::Polygon:
            ui.comboBoxFirst->addItem(str.asprintf("#%d: Polygon", (int)(*curRel).at<ito::float32>(0, 0)), (*curRel).at<ito::float32>(0, 0));
        break;
    }

    switch ((int)m_evalGeoFig->getShape((*curRel).at<ito::float32>(2, 0)).type())
    {
        case ito::Shape::Point:
            ui.comboBoxSecond->addItem(str.asprintf("#%d: Point", (int)(*curRel).at<ito::float32>(2, 0)), (*curRel).at<ito::float32>(2, 0));
        break;

        case ito::Shape::Line:
            ui.comboBoxSecond->addItem(str.asprintf("#%d: Line", (int)(*curRel).at<ito::float32>(2, 0)), (*curRel).at<ito::float32>(2, 0));
        break;

        case ito::Shape::Rectangle:
            ui.comboBoxSecond->addItem(str.asprintf("#%d: Rectangle", (int)(*curRel).at<ito::float32>(2, 0)), (*curRel).at<ito::float32>(2, 0));
        break;

        case ito::Shape::Square:
            ui.comboBoxSecond->addItem(str.asprintf("#%d: Square", (int)(*curRel).at<ito::float32>(2, 0)), (*curRel).at<ito::float32>(2, 0));
        break;

        case ito::Shape::Circle:
            ui.comboBoxSecond->addItem(str.asprintf("#%d: Circle", (int)(*curRel).at<ito::float32>(2, 0)), (*curRel).at<ito::float32>(2, 0));
        break;

        case ito::Shape::Ellipse:
            ui.comboBoxSecond->addItem(str.asprintf("#%d: Ellipse", (int)(*curRel).at<ito::float32>(2, 0)), (*curRel).at<ito::float32>(2, 0));
        break;

        case ito::Shape::Polygon:
            ui.comboBoxSecond->addItem(str.asprintf("#%d: Polygon", (int)(*curRel).at<ito::float32>(2, 0)), (*curRel).at<ito::float32>(0, 2));
        break;
    }

    switch ((int)(*curRel).at<ito::float32>(1, 0))
    {
        case 1:
            ui.comboBoxType->addItem("Radius", 1);
        break;

        case 2:
            ui.comboBoxType->addItem("Angle", 2);
        break;

        case 3:
            ui.comboBoxType->addItem("Distance", 3);
        break;

        case 4:
            ui.comboBoxType->addItem("Intersection", 4);
        break;

        case 5:
            ui.comboBoxType->addItem("Length", 5);
        break;

        case 6:
            ui.comboBoxType->addItem("Area", 6);
        break;
    }
}

//-----------------------------------------------------------------------------------------------
/*
void DialogDeleteRelation::getData(InternalInfo &data)
{
    data.m_relationsList.append(relationShip(ui.comboBoxFirst->currentData().toInt(),
        ui.comboBoxSecond->currentData().toInt(),
        ui.comboBoxType->currentData().toInt()));
}
*/

//-----------------------------------------------------------------------------------------------
void DialogDeleteRelation::on_buttonBox_clicked(QAbstractButton* btn)
{
    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);
    if (role == QDialogButtonBox::AcceptRole)
    {
        QSharedPointer<ito::DataObject> curRel = m_evalGeoFig->getCurrentRelation();
        m_evalGeoFig->delRelation((int)(*curRel).at<ito::float32>(4, 0));
        accept(); //AcceptRole
    }
    else
    {
        reject(); //close dialog with reject
    }
}

//-----------------------------------------------------------------------------------------------

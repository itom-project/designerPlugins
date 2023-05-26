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
 * \file dialogDeleteRelation.h
 * \brief This file contains the declarations for the dialog "deleteRelation" for the evaluateGeometrics-Widget.
 */

#ifndef DIALOGDELETERELATION
#define DIALOGDELETERELATION

#include <QtGui>
#include <qdialog.h>

#include "plotTreeWidget.h"

#include "ui_dialogRelation.h"

class EvaluateGeometricsFigure; //forward declaration

/*!
 * \class DialogDeleteRelation
 * \brief This class manages the dialog for deleting relations it inherites and corresponding ui-dialog "dialogRelation.ui"
 * \todo  Implement this dialog
 */

class DialogDeleteRelation : public QDialog
{
    Q_OBJECT

    public:
        DialogDeleteRelation(const InternalInfo &data, EvaluateGeometricsFigure *egFig, QWidget *parent = NULL); /*!< Class constructor, which takes all necessary informations to create the correspondig dialog and fill in the information of each relations.*/
        ~DialogDeleteRelation() {};                                             /*!< Class destructor, currently not used */

//        void getData(InternalInfo &data);                                       /*!< This function updated the internal dataStructur. It is only called in case that the dialog existed with the ok-buttom */

    private:
        EvaluateGeometricsFigure *m_evalGeoFig;
        Ui::DialogRelation ui;                                                /*!< Handle to the UI dialog, defiend in "ui_dialogRelation.h" */

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);

};

#endif //DIALOGDELETERELATION

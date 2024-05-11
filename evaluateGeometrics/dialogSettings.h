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
 * \file dialogSettings.h
 * \brief This file contains the declarations for the dialog "settings" for the evaluateGeometrics-Widget.
 */

#ifndef DIALOGSETTING
#define DIALOGSETTING

#include <QtGui>
#include <qdialog.h>

#include "plotTreeWidget.h"

#include "ui_dialogSettings.h"

/*!
 * \class DialogSettings
 * \brief This class manages the dialog for basic settings and it inherits the ui-dialog "dialogSettings.ui"
 */

class DialogSettings : public QDialog
{
public:
    DialogSettings(const InternalInfo &input, const int &geometicElements, QWidget *parent = NULL); /*!< Class constructor, which takes all necessary information to create the corresponding dialog.*/
    ~DialogSettings() {};                                                                           /*!< Class destructor, currently not used */

    void getData(InternalInfo &data);                                                               /*!< This function updated the internal dataStructur. It is only called in case that the dialog existed with the ok-buttom */

private:


    Ui::UIDialogSettings ui;                                                                        /*!< Handle to the UI dialog, defined in "ui_dialogSettings.h"*/

private slots:

};

#endif //DIALOGSETTING

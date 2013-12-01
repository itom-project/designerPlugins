/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut f�r Technische Optik (ITO), 
   Universit�t Stuttgart, Germany 
 
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

#ifndef DIALOGADDRELATION
#define DIALOGADDRELATION

#include <QtGui>
#include <qdialog.h>

#include "plotTreeWidget.h"

#include "ui_dialogRelation.h"

class DialogAddRelation : public QDialog 
{
public:
    DialogAddRelation(const InternalInfo &data, QWidget *parent = NULL);
    ~DialogAddRelation() {};

    void getData(InternalInfo &data);

private:

    Ui::UIDialogRelation ui;

private slots:

};

#endif //DIALOGADDRELATION
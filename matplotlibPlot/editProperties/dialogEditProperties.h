/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2019, Institut fuer Technische Optik (ITO),
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

#ifndef DIALOGEDITPROPERTIES_H
#define DIALOGEDITPROPERTIES_H

#include <QtGui>
#include <qdialog.h>

class QDialogButtonBox;
class QPushButton;
class QVBoxLayout;

class DialogEditProperties : public QDialog
{
    Q_OBJECT
public:
    DialogEditProperties(bool showApplyButton, const QString &title = "", QWidget *parent = 0);

    ~DialogEditProperties();


private:
    QDialogButtonBox *m_pBBox;
    QPushButton *m_pBtnApply;
    QWidget *m_pContent;
    QVBoxLayout *m_pLayout;

public Q_SLOTS:
    QWidget* addFormWidget(const QString &title, const QString &comment = "", bool withMargin = false);
    QWidget* addFormTabWidget(const QString &title, const QString &comment = "");
    QWidget* addFormComboWidget(const QString &title, const QString &comment = "");

Q_SIGNALS:
    void applied();


};

#endif

/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2019, Institut für Technische Optik (ITO),
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

#ifndef FORMTABWIDGET_H
#define FORMTABWIDGET_H

#include <QtGui>
#include <qtabwidget.h>

#include "formWidget.h"
#include "formComboWidget.h"

class FormTabWidget : public QWidget
{
    Q_OBJECT
public:
    FormTabWidget(QWidget *parent = 0) :
        QWidget(parent),
        m_pTabWidget(nullptr)
    {
        QVBoxLayout *layout = new QVBoxLayout(this);

        m_pTabWidget = new QTabWidget(this);
        layout->addWidget(m_pTabWidget);
        layout->setContentsMargins(0, 0, 0, 0);
        setLayout(layout);
    }

    ~FormTabWidget() {};

public Q_SLOTS:
    QWidget *addFormWidget(const QString &title, const QString &comment = "", bool withMargin = false)
    {
        FormWidget *widget = new FormWidget(comment, withMargin, this);
        int index = m_pTabWidget->addTab(widget, title);
        m_pTabWidget->setTabToolTip(index, comment);
        return widget;
    }

    QWidget *addFormComboWidget(const QString &title, const QString &comment = "")
    {
        FormComboWidget *widget = new FormComboWidget(this);
        int index = m_pTabWidget->addTab(widget, title);
        m_pTabWidget->setTabToolTip(index, comment);
        return widget;
    }

private:
    QTabWidget *m_pTabWidget;
};

#endif

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

#ifndef FORMCOMBOWIDGET_H
#define FORMCOMBOWIDGET_H

#include <QtGui>
#include <qstackedwidget.h>

#include "formWidget.h"

class FormComboWidget : public QWidget
{
    Q_OBJECT
public:
    FormComboWidget(QWidget *parent = 0) :
        QWidget(parent),
        m_pComboBox(nullptr),
        m_pStackedWidget(nullptr)
    {
        QVBoxLayout *layout = new QVBoxLayout(this);
        setLayout(layout);

        m_pComboBox = new QComboBox(this);
        layout->addWidget(m_pComboBox);

        m_pStackedWidget = new QStackedWidget(this);
        layout->addWidget(m_pStackedWidget);

        connect(m_pComboBox, SIGNAL(currentIndexChanged(int)), m_pStackedWidget, SLOT(setCurrentIndex(int)));
    }

    ~FormComboWidget() {};

public Q_SLOTS:
    QWidget *addFormWidget(const QString &title, const QString &comment = "", bool withMargin = false)
    {
        FormWidget *widget = new FormWidget(comment, withMargin, this);
        m_pComboBox->addItem(title);
        m_pStackedWidget->addWidget(widget);
        return widget;
    }

private:
    QComboBox *m_pComboBox;
    QStackedWidget *m_pStackedWidget;
};

#endif

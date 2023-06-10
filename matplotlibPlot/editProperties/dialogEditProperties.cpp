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

#include "dialogEditProperties.h"

#include <qlayout.h>
#include <qdialogbuttonbox.h>
#include <qpushbutton.h>
#include <qboxlayout.h>

#include "formWidget.h"
#include "formTabWidget.h"
#include "formComboWidget.h"

//-------------------------------------------------------------------------------------
DialogEditProperties::DialogEditProperties(bool showApplyButton, const QString &title /*= ""*/, QWidget *parent /*= 0*/) :
    QDialog(parent),
    m_pBBox(nullptr),
    m_pBtnApply(nullptr),
    m_pContent(nullptr),
    m_pLayout(nullptr)
{
    m_pLayout = new QVBoxLayout(this);

    // Button box
    m_pBBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);

    if (showApplyButton)
    {

        m_pBtnApply = m_pBBox->addButton(QDialogButtonBox::Apply);
        connect(m_pBtnApply, SIGNAL(clicked()), this, SIGNAL(applied()));
    }

    connect(m_pBBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(m_pBBox, SIGNAL(rejected()), this, SLOT(reject()));

    m_pLayout->addWidget(m_pBBox);
    setLayout(m_pLayout);

    setWindowTitle(title);
}

//-------------------------------------------------------------------------------------
DialogEditProperties::~DialogEditProperties()
{
    //int i = 1;
}

//-------------------------------------------------------------------------------------
QWidget* DialogEditProperties::addFormWidget(const QString &title, const QString &comment /*= ""*/, bool withMargin /*= false*/)
{
    if (m_pContent)
    {
        m_pLayout->removeWidget(m_pContent);
        m_pContent->deleteLater();
    }

    m_pContent = new FormWidget(comment, withMargin, this);
    m_pLayout->insertWidget(0, m_pContent);
    return m_pContent;
}

//-------------------------------------------------------------------------------------
QWidget* DialogEditProperties::addFormTabWidget(const QString &title, const QString &comment /*= ""*/)
{
    if (m_pContent)
    {
        m_pLayout->removeWidget(m_pContent);
        m_pContent->deleteLater();
    }

    m_pContent = new FormTabWidget(this);
    m_pLayout->insertWidget(0, m_pContent);
    return m_pContent;
}

//-------------------------------------------------------------------------------------
QWidget* DialogEditProperties::addFormComboWidget(const QString &title, const QString &comment /*= ""*/)
{
    if (m_pContent)
    {
        m_pLayout->removeWidget(m_pContent);
        m_pContent->deleteLater();
    }

    m_pContent = new FormComboWidget(this);
    m_pLayout->insertWidget(0, m_pContent);
    return m_pContent;
}

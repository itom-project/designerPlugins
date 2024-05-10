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

#ifndef FORMWIDGET_H
#define FORMWIDGET_H

#include <qformlayout.h>
#include "fontWidget.h"
#include "colorWidget.h"
#include <qlabel.h>
#include <qlineedit.h>
#include <qspinbox.h>
#include <qdatetimeedit.h>


class FormWidget : public QWidget
{
    Q_OBJECT
public:
    FormWidget(const QString &comment = "", bool withMargin = false, QWidget *parent = 0) :
        QWidget(parent),
        m_rows(0)
    {
        m_pFormLayout = new QFormLayout(this);
        if (!withMargin)
        {
            m_pFormLayout->setContentsMargins(0, 0, 0, 0);
        }

        if (comment != "")
        {
            m_pFormLayout->addRow(new QLabel(comment));
            m_pFormLayout->addRow(new QLabel(" "));
        }
    }

    ~FormWidget() {};


private:
    QFormLayout *m_pFormLayout;
    int m_rows;

public Q_SLOTS:
    void addSeparator()
    {
        m_pFormLayout->insertRow(m_rows++, new QLabel(" "), new QLabel(" "));
    }

    void addComment(const QString &comment)
    {
        m_pFormLayout->insertRow(m_rows++, new QLabel(comment));
    }

    QWidget* addFont(const QString &label, const QFont &font)
    {
        QWidget *fontWidget = new FontWidget(font, this);
        m_pFormLayout->insertRow(m_rows++, label, fontWidget);
        return fontWidget;
    }

    QWidget* addColor(const QString &label, const QColor &color)
    {
        QWidget *colorWidget = new ColorWidget(color, this);
        m_pFormLayout->insertRow(m_rows++, label, colorWidget);
        return colorWidget;
    }

    QWidget* addText(const QString &label, const QString &value)
    {
        QWidget *valueWidget = new QLineEdit(value);
        m_pFormLayout->insertRow(m_rows++, label, valueWidget);
        return valueWidget;
    }

    QWidget* addBoolean(const QString &label, bool value)
    {
        QCheckBox *valueWidget = new QCheckBox(this);
        valueWidget->setChecked(value);
        m_pFormLayout->insertRow(m_rows++, label, valueWidget);
        return valueWidget;
    }

    QWidget* addIntegral(const QString &label, int value)
    {
        QSpinBox *valueWidget = new QSpinBox(this);
        valueWidget->setRange(-1e9, 1e9);
        valueWidget->setValue(value);
        m_pFormLayout->insertRow(m_rows++, label, valueWidget);
        return valueWidget;
    }

    QWidget* addReal(const QString &label, double value)
    {
        QDoubleSpinBox *valueWidget = new QDoubleSpinBox(this);
        valueWidget->setRange(-1.e9, 1.e9);
        valueWidget->setValue(value);
        m_pFormLayout->insertRow(m_rows++, label, valueWidget);
        return valueWidget;
    }

    QWidget* addComboBox(const QString &label, const QStringList &values, int selIndex)
    {
        QComboBox *valueWidget = new QComboBox(this);
        valueWidget->addItems(values);
        valueWidget->setCurrentIndex(selIndex);
        m_pFormLayout->insertRow(m_rows++, label, valueWidget);
        return valueWidget;
    }

    QWidget* addDatetimeUTC(const QString &label, int year, int month, int day, int hour, int minute, int seconds, int ms)
    {
        //always given in utc
        QDateTimeEdit *valueWidget = new QDateTimeEdit(this);
        QDateTime dt(QDate(year, month, day), QTime(hour, minute, seconds, ms), QTimeZone(0));
        valueWidget->setDateTime(dt);
        m_pFormLayout->insertRow(m_rows++, label, valueWidget);
        return valueWidget;
    }

    QWidget* addDate(const QString &label, int year, int month, int day)
    {
        QDateEdit *valueWidget = new QDateEdit(this);
        valueWidget->setDate(QDate(year, month, day));
        m_pFormLayout->insertRow(m_rows++, label, valueWidget);
        return valueWidget;
    }

Q_SIGNALS:
    void updateButtons();


};


#endif // FORMWIDGET_H

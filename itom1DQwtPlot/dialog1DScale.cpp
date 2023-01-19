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

#include "dialog1DScale.h"

#include <qmessagebox.h>
#include <qregularexpression.h>
#include <qvalidator.h>

//-----------------------------------------------------------------------------------------------
Dialog1DScale::Dialog1DScale(const Plot1DWidget::InternalData& data, QWidget* parent) :
    QDialog(parent), m_minX(-std::numeric_limits<double>::max()),
    m_maxX(std::numeric_limits<double>::max()), m_minY(-std::numeric_limits<double>::max()),
    m_maxY(std::numeric_limits<double>::max()), m_locale(QLocale())
{
    ui.setupUi(this);

    m_locale.setNumberOptions(QLocale::OmitGroupSeparator);

    QString numberRegExp;
    if (m_locale.decimalPoint() == '.')
    {
        numberRegExp = "^[\\+-]?(?:0|[1-9]\\d*)(?:\\.\\d*)?(?:[eE][\\+-]?\\d+)?$";
    }
    else
    {
        numberRegExp = "^[\\+-]?(?:0|[1-9]\\d*)(?:,\\d*)?(?:[eE][\\+-]?\\d+)?$";
    }
    auto* numberValidator = new QRegularExpressionValidator(QRegularExpression(numberRegExp), this);

    // x
    if (data.m_axisScaleAuto)
    {
        ui.radioAutoCalcX->setChecked(true);
    }
    else
    {
        ui.radioManualX->setChecked(true);
    }

    ui.txtMinX->setValidator(numberValidator);
    ui.txtMaxX->setValidator(numberValidator);

    ui.dateTimeMinX->setVisible(data.m_hasDateTimeXAxis);
    ui.dateTimeMaxX->setVisible(data.m_hasDateTimeXAxis);
    ui.txtMinX->setVisible(!data.m_hasDateTimeXAxis);
    ui.txtMaxX->setVisible(!data.m_hasDateTimeXAxis);

    if (!data.m_hasDateTimeXAxis)
    {
        ui.txtMinX->setText(m_locale.toString(data.m_axisMin, 'g'));
        ui.txtMaxX->setText(m_locale.toString(data.m_axisMax, 'g'));
    }
    else
    {
        const QDateTime epochDate(QDate(1970, 1, 1), QTime(0, 0, 0));
        ui.dateTimeMinX->setDateTime(epochDate.addMSecs(data.m_axisMin));
        ui.dateTimeMaxX->setDateTime(epochDate.addMSecs(data.m_axisMax));
    }

    // y
    if (data.m_valueScaleAuto)
    {
        ui.radioAutoCalcY->setChecked(true);
    }
    else
    {
        ui.radioManualY->setChecked(true);
    }

    // it is not necessary to restrict the value range, since it might be desired to set a higher
    // axis interval than the datatype of the displayed dataObject. getDataTypeRange(data.m_dataType,
    // m_minY, m_maxY);

    ui.txtMinY->setText(m_locale.toString(data.m_valueMin, 'g'));
    ui.txtMaxY->setText(m_locale.toString(data.m_valueMax, 'g'));

    ui.txtMinY->setValidator(numberValidator);
    ui.txtMaxY->setValidator(numberValidator);
}

//-----------------------------------------------------------------------------------------------
void Dialog1DScale::getData(Plot1DWidget::InternalData& data)
{
    data.m_valueScaleAuto = ui.radioAutoCalcY->isChecked();
    data.m_axisScaleAuto = ui.radioAutoCalcX->isChecked();

    bool ok;
    double number;

    number = m_locale.toDouble(ui.txtMinY->text(), &ok);
    if (ok)
    {
        data.m_valueMin = number;
    }

    number = m_locale.toDouble(ui.txtMaxY->text(), &ok);
    if (ok)
    {
        data.m_valueMax = number;
    }

    if (data.m_hasDateTimeXAxis)
    {
        data.m_axisMin = ui.dateTimeMinX->dateTime().toMSecsSinceEpoch();
        data.m_axisMax = ui.dateTimeMaxX->dateTime().toMSecsSinceEpoch();
    }
    else
    {
        number = m_locale.toDouble(ui.txtMinX->text(), &ok);
        if (ok)
        {
            data.m_axisMin = number;
        }

        number = m_locale.toDouble(ui.txtMaxX->text(), &ok);
        if (ok)
        {
            data.m_axisMax = number;
        }
    }
}

//-----------------------------------------------------------------------------------------------
void Dialog1DScale::getDataTypeRange(ito::tDataType type, double& min, double& max)
{
    switch (type)
    {
    case ito::tInt8:
        min = std::numeric_limits<ito::int8>::min();
        max = std::numeric_limits<ito::int8>::max();
        break;
    case ito::tUInt8:
    case ito::tRGBA32:
        min = std::numeric_limits<ito::uint8>::min();
        max = std::numeric_limits<ito::uint8>::max();
        break;
    case ito::tInt16:
        min = std::numeric_limits<ito::int16>::min();
        max = std::numeric_limits<ito::int16>::max();
        break;
    case ito::tUInt16:
        min = std::numeric_limits<ito::uint16>::min();
        max = std::numeric_limits<ito::uint16>::max();
        break;
    case ito::tInt32:
        min = std::numeric_limits<ito::int32>::min();
        max = std::numeric_limits<ito::int32>::max();
        break;
    case ito::tUInt32:
        min = std::numeric_limits<ito::uint32>::min();
        max = std::numeric_limits<ito::uint32>::max();
        break;
    case ito::tFloat32:
    case ito::tComplex64:
        min = -std::numeric_limits<ito::float32>::max();
        max = -min;
        break;
    case ito::tFloat64:
    case ito::tComplex128:
    case ito::tDateTime:
        min = -std::numeric_limits<ito::float64>::max();
        max = -min;
        break;
    }
}

//-----------------------------------------------------------------------------------------------
bool Dialog1DScale::checkValue(
    QLineEdit* lineEdit, const double& min, const double& max, const QString& name)
{
    bool ok;
    double val = m_locale.toDouble(lineEdit->text(), &ok);
    if (!ok)
    {
        QMessageBox::critical(
            this,
            tr("invalid number"),
            tr("The '%1' number is no valid decimal number.").arg(name));
    }
    else if ((val < min) || (val > max))
    {
        ok = false;
        QMessageBox::critical(
            this,
            tr("out of range"),
            tr("The '%1' number is out of range [%2,%3]")
                .arg(name)
                .arg(m_locale.toString(min, 'g'))
                .arg(m_locale.toString(max, 'g')));
    }

    if (!ok)
    {
        lineEdit->selectAll();
    }

    return ok;
}

//-----------------------------------------------------------------------------------------------
void Dialog1DScale::on_buttonBox_accepted()
{
    bool ok = true;

    if (!checkValue(ui.txtMinX, m_minX, m_maxX, "minimum X"))
    {
        return;
    }

    if (!checkValue(ui.txtMaxX, m_minX, m_maxX, "maximum X"))
    {
        return;
    }

    if (!checkValue(ui.txtMinY, m_minY, m_maxY, "minimum Y"))
    {
        return;
    }

    if (!checkValue(ui.txtMaxY, m_minY, m_maxY, "maximum Y"))
    {
        return;
    }


    emit accept();
}

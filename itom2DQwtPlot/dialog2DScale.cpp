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

#include "dialog2DScale.h"

#include <qvalidator.h>
#include <qmessagebox.h>
#include <qregexp.h>

//-----------------------------------------------------------------------------------------------
Dialog2DScale::Dialog2DScale(const PlotCanvas::InternalData &data, QWidget *parent) :
    QDialog(parent),
	m_minX(-std::numeric_limits<double>::max()),
	m_maxX(std::numeric_limits<double>::max()),
	m_minY(-std::numeric_limits<double>::max()),
	m_maxY(std::numeric_limits<double>::max()),
	m_minValue(-std::numeric_limits<double>::max()),
	m_maxValue(std::numeric_limits<double>::max()),
	m_locale(QLocale())
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
	QRegExpValidator *numberValidator = new QRegExpValidator(QRegExp(numberRegExp), this);

    //x
    if(data.m_xaxisScaleAuto)
    {
        ui.radioAutoCalcX->setChecked(true);
    }
    else
    {
        ui.radioManualX->setChecked(true);
    }

	ui.txtMinX->setValidator(numberValidator);
	ui.txtMaxX->setValidator(numberValidator);

	ui.txtMinX->setText(m_locale.toString(data.m_xaxisMin, 'g'));
	ui.txtMaxX->setText(m_locale.toString(data.m_xaxisMax, 'g'));

    //y
    if(data.m_yaxisScaleAuto)
    {
        ui.radioAutoCalcY->setChecked(true);
    }
    else
    {
        ui.radioManualY->setChecked(true);
    }

	ui.txtMinY->setValidator(numberValidator);
	ui.txtMaxY->setValidator(numberValidator);

	ui.txtMinY->setText(m_locale.toString(data.m_yaxisMin, 'g'));
	ui.txtMaxY->setText(m_locale.toString(data.m_yaxisMax, 'g'));

    //value
    if(data.m_valueScaleAuto)
    {
        ui.radioAutoCalcValue->setChecked(true);
    }
    else
    {
        ui.radioManualValue->setChecked(true);
    }

    //it is not necessary to restrict the value range, since it might be desired to set a higher axis interval than the datatype of the displayed dataObject.
    //getDataTypeRange(data.m_dataType, m_minValue, m_maxValue);

    ui.groupValue->setEnabled(data.m_dataType != ito::tRGBA32); //rgba32 data objects have no color map and can therefore not be cropped.

	ui.txtMinValue->setValidator(numberValidator);
	ui.txtMaxValue->setValidator(numberValidator);

	ui.txtMinValue->setText(m_locale.toString(data.m_valueMin, 'g'));
	ui.txtMaxValue->setText(m_locale.toString(data.m_valueMax, 'g'));
}

//-----------------------------------------------------------------------------------------------
void Dialog2DScale::getData(PlotCanvas::InternalData &data)
{
	data.m_valueScaleAuto = ui.radioAutoCalcValue->isChecked();
	data.m_xaxisScaleAuto = ui.radioAutoCalcX->isChecked();
	data.m_yaxisScaleAuto = ui.radioAutoCalcY->isChecked();

	bool ok1, ok2;
	double numberMin;
	double numberMax;

	numberMin = m_locale.toDouble(ui.txtMinValue->text(), &ok1);
	numberMax = m_locale.toDouble(ui.txtMaxValue->text(), &ok2);

	if (ok1 && ok2)
	{
		data.m_valueMin = std::min(numberMin, numberMax);
		data.m_valueMax = std::max(numberMin, numberMax);
	}

	numberMin = m_locale.toDouble(ui.txtMinY->text(), &ok1);
	if (ok1)
	{
		data.m_yaxisMin = numberMin;
	}

	numberMax = m_locale.toDouble(ui.txtMaxY->text(), &ok2);
	if (ok2)
	{
		data.m_yaxisMax = numberMax;
	}

	numberMin = m_locale.toDouble(ui.txtMinX->text(), &ok1);
	if (ok1)
	{
		data.m_xaxisMin = numberMin;
	}

	numberMax = m_locale.toDouble(ui.txtMaxX->text(), &ok2);
	if (ok2)
	{
		data.m_xaxisMax = numberMax;
	}
}

//-----------------------------------------------------------------------------------------------
void Dialog2DScale::getDataTypeRange(ito::tDataType type, double &min, double &max)
{
    switch(type)
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
        min = -std::numeric_limits<ito::float64>::max();
        max = -min;
        break;
    default:
        min = max = 0.0;
    }
}

//-----------------------------------------------------------------------------------------------
bool Dialog2DScale::checkValue(QLineEdit *lineEdit, const double &min, const double &max, const QString &name)
{
	bool ok;
	double val = m_locale.toDouble(lineEdit->text(), &ok);
	if (!ok)
	{
		QMessageBox::critical(this, tr("invalid number"), tr("The '%1' number is no valid decimal number.").arg(name));
	}
	else if ((val < min) || (val > max))
	{
		ok = false;
		QMessageBox::critical(this, tr("out of range"), tr("The '%1' number is out of range [%2,%3]").arg(name).arg(m_locale.toString(min, 'g')).arg(m_locale.toString(max, 'g')));
	}

	if (!ok)
	{
		lineEdit->selectAll();
	}

	return ok;
}

//-----------------------------------------------------------------------------------------------
void Dialog2DScale::on_buttonBox_accepted()
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

	if (!checkValue(ui.txtMinValue, m_minValue, m_maxValue, "minimum value"))
	{
		return;
	}

	if (!checkValue(ui.txtMaxValue, m_minValue, m_maxValue, "maximum value"))
	{
		return;
	}


	emit accept();
}

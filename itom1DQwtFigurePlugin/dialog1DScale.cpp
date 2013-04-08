/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut für Technische Optik (ITO), 
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

#include "dialog1DScale.h"

//-----------------------------------------------------------------------------------------------
Dialog1DScale::Dialog1DScale(const InternalData &data, QWidget *parent) :
    QDialog(parent)
{
    ui.setupUi(this);

    double min,max;

    //x
    if(data.m_axisScaleAuto)
    {
        ui.radioAutoCalcX->setChecked(true);
    }
    else
    {
        ui.radioManualX->setChecked(true);
    }

    ui.doubleSpinMinX->setValue( data.m_axisMin );
    ui.doubleSpinMaxX->setValue( data.m_axisMax );
    
    //y
    if(data.m_valueScaleAuto)
    {
        ui.radioAutoCalcY->setChecked(true);
    }
    else
    {
        ui.radioManualY->setChecked(true);
    }

    getDataTypeRange(data.m_dataType, min, max);

    ui.doubleSpinMinY->setMinimum(min);
    ui.doubleSpinMinY->setMaximum(max);
    ui.doubleSpinMaxY->setMinimum(min);
    ui.doubleSpinMaxY->setMaximum(max);
    
    ui.doubleSpinMinY->setValue( data.m_valueMin );
    ui.doubleSpinMaxY->setValue( data.m_valueMax );

    //ui.groupPlane->setVisible(false);
}

//-----------------------------------------------------------------------------------------------
void Dialog1DScale::getData(InternalData &data)
{
    data.m_valueScaleAuto = ui.radioAutoCalcY->isChecked();
    data.m_valueMin = ui.doubleSpinMinY->value();
    data.m_valueMax = ui.doubleSpinMaxY->value();
    
    data.m_axisScaleAuto = ui.radioAutoCalcX->isChecked();
    data.m_axisMin = ui.doubleSpinMinX->value();
    data.m_axisMax = ui.doubleSpinMaxX->value();
}

//-----------------------------------------------------------------------------------------------
void Dialog1DScale::getDataTypeRange(ito::tDataType type, double &min, double &max)
{
    switch(type)
    {
    case ito::tInt8:
        min = std::numeric_limits<ito::int8>::min();
        max = std::numeric_limits<ito::int8>::max();
        break;
    case ito::tUInt8:
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
    }
}
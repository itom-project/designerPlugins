/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2012, Institut fuer Technische Optik (ITO), 
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

#ifndef CONFIGSTRUCTQWT2D_H
#define CONFIGSTRUCTQWT2D_H

#include <QObject>    

class Itom2DQwt : public QObject
{
    Q_OBJECT
    Q_ENUMS(tModificationState);    
    Q_ENUMS(tComplexType);

public:
    Itom2DQwt(): QObject() {}
    enum tModificationState
    {
        tNextElementMode = 0,
        tMoveGeometricElements = 1,
        tRotateGeometricElements = 2,
        tResizeGeometricElements = 3,
        tModifyPoints   = 4
    };
    enum tComplexType 
    { 
        Abs = 0, 
        Imag = 1, 
        Real = 2, 
        Phase = 3 
    }; //definition like in dataObject: 0:abs-Value, 1:imaginary-Value, 2:real-Value, 3: argument-Value
    enum tColorHandling { AutoColor, Gray, RGB, RGBA, RGBGray};
    enum tPlotPickerType { DefaultMarker, RangeMarker };
};

#endif //CONFIGSTRUCTQWT2D_H
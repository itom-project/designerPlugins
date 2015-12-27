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

#ifndef CONFIGSTRUCTQWT1D_H
#define CONFIGSTRUCTQWT1D_H

#include <QObject>    

class Itom1DQwt : public QObject
{
    Q_OBJECT
    Q_ENUMS(tPlotPickerType);
    Q_ENUMS(tMultiLineMode);    
    Q_ENUMS(tColorHandling ); 
    Q_ENUMS(tCurveStyle );
    Q_ENUMS(tFillCurveStyle );
    Q_ENUMS(ScaleEngine );

public:
    Itom1DQwt(): QObject() {}
    enum tMultiLineMode { AutoRowCol, FirstRow, FirstCol, MultiRows, MultiCols, MultiLayerAuto, MultiLayerCols, MultiLayerRows };
    enum tColorHandling { AutoColor, Gray, RGB, RGBA, RGBGray};
    enum tPlotPickerType { DefaultMarker, RangeMarker };
    enum tCurveStyle {   NoCurve = -1,  Lines, FittedLines, Sticks, SticksHorizontal, SticksVertical, Steps, StepsRight, StepsLeft,  Dots };
    enum tFillCurveStyle {   NoCurveFill = -1,  FillBaseLine, FillFromTop, FillFromBottom};
    enum ScaleEngine { Linear = 1, Log2 = 2, Log10 = 10, Log16 = 16, LogLog2 = 1002, LogLog10 = 1010, LogLog16 = 1016};
};

#endif //CONFIGSTRUCTQWT1D_H
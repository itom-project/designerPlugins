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

#ifndef ITOM2DQWTPLOT_H
#define ITOM2DQWTPLOT_H

#include "plot/AbstractDObjFigure.h"

#include "plotCanvas.h"

#include <qaction.h>
#include <qspinbox.h>

Q_DECLARE_METATYPE(QSharedPointer<ito::DataObject>)

class Itom2dQwtPlot : public ito::AbstractDObjFigure
{
	Q_OBJECT

	Q_PROPERTY(bool showColorBar READ showColorBar WRITE setShowColorBar DESIGNABLE true)

public:
	Itom2dQwtPlot(const QString &itomSettingsFile, QWidget *parent = 0);
	~Itom2dQwtPlot();

	//properties (setter/getter)
	void setShowContextMenu(bool show) {}; 
	bool showContextMenu() const { return false; };

	ito::RetVal applyUpdate();  //!> does the real update work

	bool showColorBar() const;
	void setShowColorBar(bool value);

protected:
	void createActions();

private:

	Itom2dQwtPlotActions *m_pActions;
	PlotCanvas *m_pCanvas;	

};

#endif // ITOM2DQWTPLOT_H

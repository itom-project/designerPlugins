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

#ifndef WIDGETCURVEPROPERTIES
#define WIDGETCURVEPROPERTIES

#include <qwidget.h>
#include <qlistwidget.h>

#include "ui_widgetCurveProperties.h"

class Plot1DWidget; //forward declaration
class QwtPlotCurve;

class WidgetCurveProperties : public QWidget
{
    Q_OBJECT
public:
    WidgetCurveProperties(Plot1DWidget* content, QWidget *parent = NULL);
    ~WidgetCurveProperties() {};

	void updateProperties();

private:
    Ui::WidgetCurveProperties ui;

    Plot1DWidget *m_pContent;
	bool m_visible;
	bool m_isUpdating;
	QColor getSymbolColor(const QwtPlotCurve* item);
    const QStringList generateLegendList() const;

private slots:
	void on_spinBoxSymbolSize_valueChanged(int val);

	void on_doubleSpinBoxLineWidth_valueChanged(double i);
	void on_comboBoxLineStyle_currentIndexChanged(int val);

	void on_colorPickerButtonLineStyle_colorChanged(QColor color);
	void on_comboBoxLineSymbol_currentIndexChanged(int val);
	void on_checkBoxLegendVisible_stateChanged(int val);
	void on_listWidget_itemChanged(QListWidgetItem *item);
	void on_checkBoxVisible_stateChanged(int state);
	void on_colorPickerButtonSymbol_colorChanged(QColor color);
	void on_lineEditName_editingFinished();

public slots:

	void on_listWidget_itemSelectionChanged();
	void visibilityChanged(bool visible);
	void updateCurveList();

signals:
    void titleChanged(const QStringList &legends);

};

#endif

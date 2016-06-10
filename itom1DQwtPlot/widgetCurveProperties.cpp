/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2016, Institut fuer Technische Optik (ITO), 
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

#include "widgetCurveProperties.h"

#include <qdebug.h>
#include "qmetaobject.h"


#include "Plot1DWidget.h"

#include "cmath"

#include "limits"

//-----------------------------------------------------------------------------------------------
WidgetCurveProperties::WidgetCurveProperties(Plot1DWidget* content, QWidget *parent /*= NULL*/) :
    m_pContent(content),
    QWidget(parent),
	m_visible(false)
{
    ui.setupUi(this);
	Qt::PenStyle;
	

}
void WidgetCurveProperties::updateProperties()
{
	if (m_visible)
	{
		ui.listWidget->clear();
		QList<QwtPlotCurve*> curves = m_pContent->getplotCurveItems();
		QwtPlotCurve*  curve;

		foreach(curve, curves)
		{
			ui.listWidget->addItem(curve->title().text());

		}
		const QMetaObject *mo = qt_getEnumMetaObject(Qt::PenStyle::DashLine);
		QMetaEnum me = mo->enumerator(mo->indexOfEnumerator("PenStyle"));


		for (int i = 0; i < me.keyCount(); ++i)
		{
			ui.comboBoxLineStyle->addItem(me.key(i), QVariant());
		}

	}

}

//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_listWidget_itemSelectionChanged()
{
	
	QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
	QListWidgetItem* item;
	int row;
	QPen pen;
	
	
	bool constWidth = true;// if the current selection does not have the same linewidth at all, than constWidth will be set to false 
	bool constLineStyle = true;// if the current selection does not have the same linestyle at all, than constWidth will be set to false 
	bool constVisible = true;


	bool first = true; //marks the first line witch is checked 
	float width;
	Qt::PenStyle lineStyle;
	bool visible;
	foreach(item, selection)
	{
		row = ui.listWidget->row(item);
		pen=m_pContent->getplotCurveItems().at(row)->pen();

		if (!first) 
		{
			
			if (!(std::abs(pen.widthF() - width) <= FLT_EPSILON))
			{
				constWidth = false;
			}
			if (lineStyle != pen.style())
			{
				constLineStyle = false;
			}
			if (m_pContent->getplotCurveItems().at(row)->isVisible() != visible)
			{
				constVisible = false;
			}

		}

		width = pen.widthF();
		lineStyle = pen.style();
		visible = m_pContent->getplotCurveItems().at(row)->isVisible();
		first = false; //set to false after first for iteration
	}
	if (constWidth)//all lines have the same width
	{
		ui.doubleSpinBoxLineWidth->setValue((float)pen.widthF());
	}else
	{
		ui.doubleSpinBoxLineWidth->setValue(0.000f);
	}
	if (constLineStyle)
	{
		const QMetaObject *mo = qt_getEnumMetaObject(Qt::PenStyle::DashLine);
		QMetaEnum me = mo->enumerator(mo->indexOfEnumerator("PenStyle"));
		QString lineStyleEnumValue;
		ui.comboBoxLineStyle->setCurrentIndex((int)pen.style());
		
	}
	else
	{
		ui.comboBoxLineStyle->setCurrentIndex(-1);
	}
	if (constVisible)
	{
		ui.checkBoxVisible->setCheckState(Qt::Checked);
	}
	else
	{
		ui.checkBoxVisible->setCheckState(Qt::Unchecked);
	}
	
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_checkBoxVisible_stateChanged(int state)
{
	QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
	QListWidgetItem* item;
	QPen pen;
	int row;

	foreach(item, selection)
	{
		row = ui.listWidget->row(item);
		m_pContent->getplotCurveItems().at(row)->setVisible((bool)state);

	}
	m_pContent->replot();

}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_comboBoxLineStyle_currentIndexChanged(int val)
{
	QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
	QListWidgetItem* item;
	QPen pen;
	int row;

	foreach(item, selection)
	{
		row = ui.listWidget->row(item);
		pen = m_pContent->getplotCurveItems().at(row)->pen();
		pen.setStyle((Qt::PenStyle)val);
		m_pContent->getplotCurveItems().at(row)->setPen(pen);
	}
	m_pContent->replot();
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_doubleSpinBoxLineWidth_valueChanged(double i)
{
	QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
	QListWidgetItem* item;
	QPen pen;
	int row;
	foreach(item, selection)
	{
		row = ui.listWidget->row(item);
		pen=m_pContent->getplotCurveItems().at(row)->pen();
		pen.setWidthF((qreal)i);
		m_pContent->getplotCurveItems().at(row)->setPen(pen);
		
		
	}
	m_pContent->replot();
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::visibilityChanged(bool visible)
{
	m_visible = visible;
	if (visible)
	{
		updateProperties();
	}
}
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
#include "qwtPlotCurveProperty.h"

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

//-----------------------------------------------------------------------------------------------
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
		const QMetaObject *mo = qt_getEnumMetaObject(Qt::DashLine);//penStyle
		QMetaEnum me = mo->enumerator(mo->indexOfEnumerator("PenStyle"));

		int i;
		for (i = 0; i < me.keyCount(); ++i)
		{
			ui.comboBoxLineStyle->addItem(me.key(i), QVariant()); //add penStyles
		}

		const QMetaObject *moBrushStyle = qt_getEnumMetaObject(Qt::BrushStyle::NoBrush);
		QMetaEnum meBrushStyle = moBrushStyle->enumerator(moBrushStyle->indexOfEnumerator("BrushStyle"));
		for (i = 0; i < meBrushStyle.keyCount(); ++i)
		{
			ui.comboBoxBrushStyle->addItem(meBrushStyle.key(i), QVariant());//addBrushStyles
		}
		const QMetaObject moLineSymbol(Itom1DQwtPlot::staticMetaObject);
		QMetaEnum meLineSymbol = moLineSymbol.enumerator(moLineSymbol.indexOfEnumerator("Symbol"));
		for (i = 0; i < meLineSymbol.keyCount(); ++i)
		{
			ui.comboBoxLineSymbol->addItem(meLineSymbol.key(i), meLineSymbol.value(i));//addLineSymbols
		}
		const QMetaObject *moCapStyle(qt_getEnumMetaObject(Qt::FlatCap));//add CapStyles
		QMetaEnum meCapStyle(moCapStyle->enumerator(moCapStyle->indexOfEnumerator("PenCapStyle")));
		for (i = 0; i < meCapStyle.keyCount(); ++i)
		{
			ui.comboBoxCapStyle->addItem(meCapStyle.key(i), meCapStyle.value(i));
		}
		const QMetaObject *moJoinStyle(qt_getEnumMetaObject(Qt::MiterJoin));//add JoinStyle
		QMetaEnum meJoinStyle(moJoinStyle->enumerator(moJoinStyle->indexOfEnumerator("PenJoinStyle")));
		for (i = 0; i < meJoinStyle.keyCount(); ++i)
		{
			ui.comboBoxJoinStyle->addItem(meJoinStyle.key(i), meJoinStyle.value(i));
		}
	}

}

//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_listWidget_itemSelectionChanged()
{
	m_isUpdating = true; //avoid any changes of the line settings while updating
	QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
	QListWidgetItem* item;
	int row = -1;
	QPen pen;


	bool constWidth = true;// if the current selection does not have the same linewidth at all, than constWidth will be set to false in the following
	bool constLineStyle = true;// if the current selection does not have the same linestyle at all, than constWidth will be set to false in the following
	bool constVisible = true;// if the current selection does not have the same visibility at all, than constWidth will be set to false in the following
	bool constBrushStyle = true;// if the current selection does not have the same baseline at all, than constWidth will be set to false in the following
	bool constLineColor = true;// if the current selection does not have the same lineColor at all, than constWidth will be set to false in the following
	bool constLineSymbol = true;// if the current selection does not have the same lineSymbol at all, than constWidth will be set to false in the following
	bool constSymbolSize = true;// if the current selection does not have the same symbolSize at all, than constWidth will be set to false in the following
	bool constCapStyle = true;// if the current selection does not have the same capStyle at all, than constWidth will be set to false in the following
	bool consJoinStyle = true;// if the current selection does not have the same joinStyle at all, than constWidth will be set to false in the following

	bool first = true; //marks the first line witch is checked 
	float width;
	Qt::PenStyle lineStyle;
	Qt::BrushStyle brushStyle;
	QColor lineColor;
	bool visible;
	QwtSymbol::Style lineSymbol;
	int symbolSize;
	Qt::PenCapStyle capStyle;
	Qt::PenJoinStyle joinStyle;

	foreach(item, selection)
	{
		row = ui.listWidget->row(item);
		pen = m_pContent->getplotCurveItems().at(row)->pen();


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
			if ( brushStyle != m_pContent->getplotCurveItems().at(row)->brush().style())
			{
				constBrushStyle = false;
			}
			if (!lineColor.operator==(pen.color()))
			{
				constLineColor = false;
			}
			if (m_pContent->getplotCurveItems().at(row)->symbol())// check if symbol exists
			{
				if (!(m_pContent->getplotCurveItems().at(row)->symbol()->style() == lineSymbol))
				{
					constLineSymbol = false;
				}
				if (symbolSize != m_pContent->getplotCurveItems().at(row)->symbol()->size().width())
				{
					constSymbolSize = false;
				}

			}
			else
			{
				if (lineSymbol != QwtSymbol::NoSymbol)// check if the line before does't have any symbols set else the lines are of different style
				{
					constLineSymbol = false;
				}
				if (symbolSize != 0)
				{
					constSymbolSize = false;
				}
			}
			if (capStyle != m_pContent->getplotCurveItems().at(row)->pen().capStyle())
			{
				constCapStyle = false;
			}
			if (joinStyle != m_pContent->getplotCurveItems().at(row)->pen().joinStyle())
			{
				consJoinStyle = false;
			}
		}

		joinStyle = m_pContent->getplotCurveItems().at(row)->pen().joinStyle();
		capStyle = m_pContent->getplotCurveItems().at(row)->pen().capStyle();
		if (m_pContent->getplotCurveItems().at(row)->symbol())//check if NULL
		{
			lineSymbol = m_pContent->getplotCurveItems().at(row)->symbol()->style();
			symbolSize = m_pContent->getplotCurveItems().at(row)->symbol()->size().width();
		}
		else
		{
			lineSymbol = QwtSymbol::NoSymbol;
			symbolSize = 0;
		}
		width = pen.widthF();
		lineStyle = pen.style();
		visible = m_pContent->getplotCurveItems().at(row)->isVisible();
		brushStyle = m_pContent->getplotCurveItems().at(row)->brush().style();
		lineColor = pen.color();
		first = false; //set to false after first for iteration
	}
	if (row != -1)//true if a curve is selected
	{

		if (constWidth)//all lines have the same width
		{
			ui.doubleSpinBoxLineWidth->setValue((float)pen.widthF());
		}
		else
		{
			ui.doubleSpinBoxLineWidth->setValue(0.000f);
		}
		if (constLineStyle)
		{
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
		if (constBrushStyle)
		{
			ui.comboBoxBrushStyle->setCurrentIndex((int)m_pContent->getplotCurveItems().at(row)->brush().style());
		}
		else
		{
			ui.comboBoxBrushStyle->setCurrentIndex(-1);
		}
		if (constLineColor)
		{
			ui.colorPickerButtonLineStyle->setColor(pen.color());
		}
		else
		{
			ui.colorPickerButtonLineStyle->setColor(Qt::black);
		}
		if (constLineSymbol)
		{
			if (m_pContent->getplotCurveItems().at(row)->symbol())
			{
				ui.comboBoxLineSymbol->setCurrentIndex((int)(m_pContent->getplotCurveItems().at(row)->symbol()->style()+1));


			}
			else
			{
				ui.comboBoxLineSymbol->setCurrentIndex(0);
			}

		}
		else
		{
			ui.comboBoxLineSymbol->setCurrentIndex(-1);
		}
		if (constSymbolSize)
		{
			if (m_pContent->getplotCurveItems().at(row)->symbol() && symbolSize == m_pContent->getplotCurveItems().at(row)->symbol()->size().height())//check if symbol is not NUll and height equal to width
			{
				ui.spinBoxSymbolSize->setValue(symbolSize);
				
			}
			else
			{
				ui.spinBoxSymbolSize->setValue(0);
			}
			

		}
		else
		{
			ui.spinBoxSymbolSize->setValue(0);
		}
		if (constCapStyle)
		{

			ui.comboBoxCapStyle->setCurrentIndex(ui.comboBoxCapStyle->findData(QVariant((int)capStyle)));

		}
		else
		{
			ui.comboBoxCapStyle->setCurrentIndex(-1);
		}
		if (consJoinStyle)
		{
			ui.comboBoxJoinStyle->setCurrentIndex(ui.comboBoxJoinStyle->findData(QVariant((int)joinStyle)));
		}
		else
		{
			ui.comboBoxJoinStyle->setCurrentIndex(-1);
		}

	}
	m_isUpdating = false;
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_comboBoxJoinStyle_currentIndexChanged(int val)
{
	if (!m_isUpdating)
	{
		QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
		QListWidgetItem* item;
		int row;
		foreach(item, selection)
		{
			row = ui.listWidget->row(item);;
			m_pContent->getPlotCurveProperty().at(row)->setLineJoinStyle((Qt::PenJoinStyle)ui.comboBoxJoinStyle->currentData().toInt());
		}
		m_pContent->replot();
	}
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_comboBoxCapStyle_currentIndexChanged(int val)
{
	if (!m_isUpdating)
	{
		QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
		QListWidgetItem* item;
		int row;
		foreach(item, selection)
		{
			row = ui.listWidget->row(item);;
			m_pContent->getPlotCurveProperty().at(row)->setLineCapStyle((Qt::PenCapStyle)ui.comboBoxCapStyle->currentData().toInt());
		}
		m_pContent->replot();
	}
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_spinBoxSymbolSize_valueChanged(int val)
{
	if (!m_isUpdating)
	{
		QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
		QListWidgetItem* item;
		int row;
		foreach(item, selection)
		{
			row = ui.listWidget->row(item);
			m_pContent->getPlotCurveProperty().at(row)->setLineSymbolSize(val);
		}
		m_pContent->replot();
	}
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_colorPickerButtonLineStyle_colorChanged(QColor color)
{
	if (!m_isUpdating)
	{
		QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
		QListWidgetItem* item;
		int row;
		QPen pen;
		foreach(item, selection)
		{
			row = ui.listWidget->row(item);
			pen = m_pContent->getplotCurveItems().at(row)->pen();
			pen.setColor(color);
			m_pContent->getplotCurveItems().at(row)->setPen(pen);

		}
		m_pContent->replot();
	}
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_comboBoxLineSymbol_currentIndexChanged(int val) 
{
	if (!m_isUpdating)
	{
		QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
		QListWidgetItem* item;
		int row;
        int enumValue = ui.comboBoxLineSymbol->currentData().toInt();

		foreach(item, selection)
		{
			row = ui.listWidget->row(item);
			m_pContent->getPlotCurveProperty().at(row)->setLineSymbolStyle((Itom1DQwtPlot::Symbol)(enumValue));
		}
		m_pContent->replot();
	}
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_comboBoxBrushStyle_currentIndexChanged(int val)
{
	if (!m_isUpdating)
	{
		QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
		QListWidgetItem* item;
		int row;
		QBrush brush;
		foreach(item, selection)
		{

			row = ui.listWidget->row(item);
			brush = m_pContent->getplotCurveItems().at(row)->brush();
			brush.setStyle((Qt::BrushStyle)val);
			brush.setColor(QColor(Qt::black));
			m_pContent->getplotCurveItems().at(row)->setBrush(brush);




		}
		m_pContent->replot();
	}
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_checkBoxVisible_stateChanged(int state)
{
	if (!m_isUpdating)
	{
		QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
		QListWidgetItem* item;
		int row;

		foreach(item, selection)
		{
			row = ui.listWidget->row(item);
			m_pContent->getplotCurveItems().at(row)->setVisible(state > 0);

		}
		m_pContent->replot();
	}
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_comboBoxLineStyle_currentIndexChanged(int val)
{
	if (!m_isUpdating)
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
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_doubleSpinBoxLineWidth_valueChanged(double i)
{
	if (!m_isUpdating)
	{
		QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
		QListWidgetItem* item;
		QPen pen;
		int row;
		foreach(item, selection)
		{
			row = ui.listWidget->row(item);
			pen = m_pContent->getplotCurveItems().at(row)->pen();
			pen.setWidthF((qreal)i);
			m_pContent->getplotCurveItems().at(row)->setPen(pen);


		}
		m_pContent->replot();
	}
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
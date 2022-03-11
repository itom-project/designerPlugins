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

#include "widgetCurveProperties.h"

#include <qdebug.h>
#include "qmetaobject.h"


#include "plot1DWidget.h"
#include "qwtPlotCurveProperty.h"

#include "cmath"
#include "limits"

#include <qwt_text.h>

//-----------------------------------------------------------------------------------------------
WidgetCurveProperties::WidgetCurveProperties(Plot1DWidget* content, QWidget *parent /*= NULL*/) :
    m_pContent(content),
    QWidget(parent),
	m_visible(false)
{
    ui.setupUi(this);

}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::updateCurveList()
{
	if (m_visible)
	{
		ui.listWidget->clear();
		QList<QwtPlotCurve*> curves = m_pContent->getplotCurveItems();
		QwtPlotCurve*  curve;

		foreach(curve, curves)
		{
			QListWidgetItem *a = new QListWidgetItem(curve->title().text());
			a->setFlags(a->flags() | Qt::ItemIsEditable);
			ui.listWidget->addItem(a);

		}
	}
}

//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::updateProperties()
{
	if (m_visible)
	{
		updateCurveList();

		const QMetaObject *mo = qt_getEnumMetaObject(Qt::DashLine); //penStyle
		QMetaEnum me = mo->enumerator(mo->indexOfEnumerator("PenStyle"));

		int i;
		for (i = 0; i < me.keyCount(); ++i)
		{
			ui.comboBoxLineStyle->addItem(me.key(i), QVariant()); //add penStyles
		}

		const QMetaObject moLineSymbol(Itom1DQwtPlot::staticMetaObject);
		QMetaEnum meLineSymbol = moLineSymbol.enumerator(moLineSymbol.indexOfEnumerator("Symbol"));
		for (i = 0; i < meLineSymbol.keyCount(); ++i)
		{
			ui.comboBoxLineSymbol->addItem(meLineSymbol.key(i), meLineSymbol.value(i));//addLineSymbols
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
	bool constLineColor = true;// if the current selection does not have the same lineColor at all, than constWidth will be set to false in the following
	bool constLineSymbol = true;// if the current selection does not have the same lineSymbol at all, than constWidth will be set to false in the following
	bool constSymbolSize = true;// if the current selection does not have the same symbolSize at all, than constWidth will be set to false in the following
	bool constLegendVisible = true;// if the current selection does not have the same legend visibility at all, than constLegendVisible will be set to false in the following
	bool constSymbolColor = true;// if the current selection does not have the same symbol color at all, than constSymbolColor will be set to false in the following

	bool first = true; //marks the first line witch is checked 
	float width;
	Qt::PenStyle lineStyle;
	QColor lineColor;
	bool visible;
	QwtSymbol::Style lineSymbol;
	int symbolSize;
	bool legendVisible;
	QColor symbolColor;

	
		if (selection.size() == 1)
		{
			ui.lineEditName->setText(m_pContent->getplotCurveItems().at(ui.listWidget->row(selection.at(0)))->title().text());//sets the name of the curve into the line edit.
			ui.lineEditName->setEnabled(true);
		}
		else
		{
			ui.lineEditName->setText("");
			ui.lineEditName->setEnabled(false);
		}

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
				if (!(getSymbolColor(m_pContent->getplotCurveItems().at(row)).operator==(symbolColor)))
				{
					constSymbolColor = false;

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
				constSymbolColor = false;//okay because the current curve does not have any color set. This leads in the following that the color is set to black
			}
			if (legendVisible != m_pContent->getPlotCurveProperty().at(row)->getLegendVisible())
			{
				constLegendVisible = false;
			}
		}
		legendVisible = m_pContent->getPlotCurveProperty().at(row)->getLegendVisible();
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
		lineColor = pen.color();
		symbolColor = getSymbolColor(m_pContent->getplotCurveItems().at(row));


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
			if (visible)
			{
				ui.checkBoxVisible->setCheckState(Qt::Checked);
			}
			else
			{
				ui.checkBoxVisible->setCheckState(Qt::Unchecked);
			}
		}
		else
		{
			ui.checkBoxVisible->setCheckState(Qt::Unchecked);
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
				ui.colorPickerButtonSymbol->setEnabled(true);
				ui.comboBoxLineSymbol->setCurrentIndex((int)(m_pContent->getplotCurveItems().at(row)->symbol()->style()+1));


			}
			else
			{
				ui.comboBoxLineSymbol->setCurrentIndex(0);
				ui.colorPickerButtonSymbol->setEnabled(false);
			}

		}
		else
		{
			ui.colorPickerButtonSymbol->setEnabled(true);
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
		if (constLegendVisible)
		{
			if (m_pContent->getPlotCurveProperty().at(row)->getLegendVisible())
			{
				ui.checkBoxLegendVisible->setCheckState(Qt::Checked);
			}
			else
			{
				ui.checkBoxLegendVisible->setCheckState(Qt::Unchecked);
			}
			
		}
		else
		{
			ui.checkBoxLegendVisible->setCheckState(Qt::Unchecked);
		}
		if (constSymbolColor)
		{
			ui.colorPickerButtonSymbol->setColor(symbolColor);
		}
		else
		{
			ui.colorPickerButtonSymbol->setColor(Qt::black);
		}

	}
	m_isUpdating = false;
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_listWidget_itemChanged(QListWidgetItem *item)
{
	if (!m_isUpdating)
	{
		m_pContent->getplotCurveItems().at(ui.listWidget->currentRow())->setTitle(item->text());
        m_pContent->applyLegendFont();
        emit titleChanged(generateLegendList());
		//QStringList legendList;
		//int curveIdx = ui.listWidget->currentRow();

		//QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
		//const QwtPlotCurve* curve;
		//foreach(curve, m_pContent->getplotCurveItems())
		//{
		//	legendList.append(curve->title().text());
		//}
		//m_pContent->setLegendTitles(legendList, NULL);
	}
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_checkBoxLegendVisible_stateChanged(int val)
{
	if (!m_isUpdating)
	{
		QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
		QListWidgetItem* item;
		int row;
		foreach(item, selection)
		{
			row = ui.listWidget->row(item);
			m_pContent->getPlotCurveProperty().at(row)->setLegendVisible(val);
		}
        m_pContent->applyLegendFont();
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
        m_pContent->applyLegendFont();
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
        m_pContent->applyLegendFont();
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
        int enumValue = ui.comboBoxLineSymbol->itemData(ui.comboBoxLineSymbol->currentIndex()).toInt();

		foreach(item, selection)
		{
			row = ui.listWidget->row(item);
			m_pContent->getPlotCurveProperty().at(row)->setLineSymbolStyle((Itom1DQwtPlot::Symbol)(enumValue));
		}
        m_pContent->applyLegendFont();
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
			m_pContent->toggleLegendLabel(m_pContent->getplotCurveItems().at(row), state);

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
        m_pContent->applyLegendFont();
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
        m_pContent->applyLegendFont();
		m_pContent->replot();
	}
}
//-----------------------------------------------------------------------------------------------
QColor WidgetCurveProperties::getSymbolColor(const QwtPlotCurve* item)
{
	if (!item->symbol())
	{
		return item->pen().color();
	}
	else
	{
		return item->symbol()->pen().color();
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
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_colorPickerButtonSymbol_colorChanged(QColor color)
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

			const QwtSymbol *oldSymbol = m_pContent->getplotCurveItems().at(row)->symbol();
			if (oldSymbol)
			{
				QPen pen = oldSymbol->pen();
				pen.setColor(color);
				QwtSymbol *newSymbol = new QwtSymbol(oldSymbol->style(), oldSymbol->brush(), pen, oldSymbol->size());
				m_pContent->getplotCurveItems().at(row)->setSymbol(newSymbol);
			}		
		}
        m_pContent->applyLegendFont();
		m_pContent->replot();
	}
}
//-----------------------------------------------------------------------------------------------
void WidgetCurveProperties::on_lineEditName_editingFinished()
{
	if (!m_isUpdating)
	{
		const QString text(ui.lineEditName->text());
		m_isUpdating = true;
		//const QString text(ui.lineEditName->text());
		QList<QListWidgetItem*> selection = ui.listWidget->selectedItems();
		int row(ui.listWidget->row(selection.at(0)));//only one can be selected otherwise the editLine is enabled
        selection[0]->setText(text);
		m_pContent->getplotCurveItems().at(row)->setTitle(text);
        emit titleChanged(generateLegendList());
        m_pContent->applyLegendFont();
		
		m_isUpdating = false;
	}
}
//-----------------------------------------------------------------------------------------------
const QStringList WidgetCurveProperties::generateLegendList() const
{
    QStringList list;
    for (int i = 0; i < ui.listWidget->count(); ++i)
    {
        QListWidgetItem* item = ui.listWidget->item(i);
        list.append(item->text());
        
    }
    return list;
}
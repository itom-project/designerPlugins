/* ********************************************************************
   itom measurement system
   URL: http://www.uni-stuttgart.de/ito
   Copyright (C) 2019, Institut fuer Technische Optik (ITO), 
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

#ifndef COLORWIDGET_H
#define COLORWIDGET_H

#include <qpushbutton.h>
#include <qsize.h>
#include <qcolor.h>
#include <qcolordialog.h>
#include <qicon.h>
#include <qpixmap.h>
#include <qlayout.h>
#include <qlineedit.h>


//--------------------------------------------------------------------
/*
Color choosing push button
*/
class ColorButton : public QPushButton
{
    Q_OBJECT
public:
    ColorButton(QWidget *parent = 0) :
        QPushButton(parent)
    {
        setFixedSize(20, 20);
        setIconSize(QSize(12, 12));
        connect(this, SIGNAL(clicked()), this, SLOT(chooseColor()));
    }

    ~ColorButton() {};

    QColor getColor() const
    {
        return m_color;
    }

    
private:
    QColor m_color;

private Q_SLOTS:
    void chooseColor()
    {
        QColor color = QColorDialog::getColor(
            m_color, this, "",
            QColorDialog::ShowAlphaChannel);
        if (color.isValid())
        {
            setColor(color);
        }
    }

public Q_SLOTS:
    void setColor(const QColor &color)
    {
        if (color != m_color)
        {
            m_color = color;
            emit colorChanged(m_color);
            QPixmap pixmap(iconSize());
            pixmap.fill(color);
            setIcon(QIcon(pixmap));
        }
    }

Q_SIGNALS:
    void colorChanged(QColor);
};


//--------------------------------------------------------------------
/*
Color-specialized QLineEdit layout
*/
class ColorWidget : public QWidget
{
    Q_OBJECT

    Q_PROPERTY(QString text READ text)
public:
    ColorWidget(const QColor &color, QWidget *parent = 0) :
        QWidget(parent),
        m_pLineEdit(nullptr),
        m_pColorBtn(nullptr)
    {
        QHBoxLayout *layout = new QHBoxLayout(this);

        m_pLineEdit = new QLineEdit(colorToRgbaText(color), parent);
        connect(m_pLineEdit, SIGNAL(editingFinished()), this, SLOT(updateColor()));
        layout->addWidget(m_pLineEdit);
        m_pColorBtn = new ColorButton(parent);
        m_pColorBtn->setColor(color);
        connect(m_pColorBtn, SIGNAL(colorChanged(QColor)), this, SLOT(updateText(QColor)));
        layout->addWidget(m_pColorBtn);
        setLayout(layout);
        setContentsMargins(0, 0, 0, 0);
    }

private:
    QLineEdit *m_pLineEdit;
    ColorButton *m_pColorBtn;

    QString colorToRgbaText(const QColor &color) const
    {
        int r = color.red();
        int g = color.green();
        int b = color.blue();
        int a = color.alpha();
        QString txt = QString("#%1%2%3%4").arg(r, 2, 16, QChar('0')). \
            arg(g, 2, 16, QChar('0')). \
            arg(b, 2, 16, QChar('0')). \
            arg(a, 2, 16, QChar('0'));
        return txt;
    }

public Q_SLOTS:
    void updateColor()
    {
        QColor color("black");
        if (text().startsWith("#"))
        {
            bool ok;
            unsigned int rgba = text().mid(1).toUInt(&ok, 16);
            if (ok)
            {
                int a = rgba & 0xff;
                int r = (rgba & 0xff000000) >> 24;
                int g = (rgba & 0x00ff0000) >> 16;
                int b = (rgba & 0x0000ff00) >> 8;
                color = QColor(r, g, b, a);
            }
        }
        else
        {
            color = QColor(text());
        }

        m_pColorBtn->setColor(color); // defaults to black if not qcolor.isValid()
    }

    void updateText(const QColor &color)
    {
        m_pLineEdit->setText(colorToRgbaText(color));
    }

    QString text()
    {
        return m_pLineEdit->text();
    }
};



#endif //COLORWIDGET_H

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

#ifndef FONTWIDGET_H
#define FONTWIDGET_H

#include <qlayout.h>
#include <qfont.h>
#include <qcheckbox.h>
#include <qcombobox.h>
#include <qfontcombobox.h>

//--------------------------------------------------------------------
/*
Font selection
*/
class FontWidget : public QWidget
{
    Q_OBJECT

    Q_PROPERTY(QFont font READ getFont)

public:
    FontWidget(const QFont &font, QWidget *parent = 0) :
        QWidget(parent),
        m_pCheckBold(nullptr),
        m_pCheckItalic(nullptr),
        m_pComboSize(nullptr),
        m_pComboFamily(nullptr)
    {
        QGridLayout *grid = new QGridLayout(this);

        // Font family
        m_pComboFamily = new QFontComboBox(parent);
        m_pComboFamily->setCurrentFont(font);
        grid->addWidget(m_pComboFamily, 0, 0, 1, -1);

        // Font size
        m_pComboSize = new QComboBox(parent);
        m_pComboSize->setEditable(true);
        QList<int> sizelist;
        sizelist << 6 << 12;
        sizelist << 12 << 14 << 16 << 18 << 20 << 22 << 24 << 26 << 28;
        sizelist << 36 << 48 << 72;

        int size = font.pointSize();

        if (!sizelist.contains(size))
        {
            sizelist.append(size);
            std::sort(sizelist.begin(), sizelist.end());
        }

        QStringList sizeliststr;
        foreach(int s, sizelist)
        {
            sizeliststr << QString::number(s);
        }

        m_pComboSize->addItems(sizeliststr);
        m_pComboSize->setCurrentIndex(sizelist.indexOf(size));
        grid->addWidget(m_pComboSize, 1, 0);

        // Italic or not
        m_pCheckItalic = new QCheckBox(tr("Italic"), parent);
        m_pCheckItalic->setChecked(font.italic());
        grid->addWidget(m_pCheckItalic, 1, 1);

        // Bold or not
        m_pCheckBold = new QCheckBox(tr("Bold"), parent);
        m_pCheckBold->setChecked(font.bold());
        grid->addWidget(m_pCheckBold, 1, 2);

        setLayout(grid);
        setContentsMargins(0, 0, 0, 0);
    }

    QFont getFont()
    {
        m_font = m_pComboFamily->currentFont();
        m_font.setItalic(m_pCheckItalic->isChecked());
        m_font.setBold(m_pCheckBold->isChecked());
        m_font.setPointSize((m_pComboSize->currentText().toInt()));
        return m_font;
    }

private:
    QCheckBox *m_pCheckBold;
    QCheckBox *m_pCheckItalic;
    QComboBox *m_pComboSize;
    QFontComboBox *m_pComboFamily;
    QFont m_font;
};

#endif //FONTWIDGET_H

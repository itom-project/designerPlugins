/* ********************************************************************
#    twipOGLFigure-Plugin for itom
#    URL: http://www.twip-os.com
#    Copyright (C) 2014, twip optical solutions GmbH,
#    Stuttgart, Germany
#
#    This files is part of the designer-Plugin twipOGLFigure for the
#    measurement software itom. All files of this plugin, where not stated
#    as port of the itom sdk, fall under the GNU Library General
#    Public Licence and must behandled accordingly.
#
#    twipOGLFigure is free software; you can redistribute it and/or modify it
#    under the terms of the GNU Library General Public Licence as published by
#    the Free Software Foundation; either version 2 of the Licence, or (at
#    your option) any later version.
#
#    Information under www.twip-os.com or mail to info@twip-os.com.
#
#    This itom-plugin is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#    GNU General Public License for more details.
#
#    itom is free software by ITO, Universität Stuttgart published under
#    GNU General Public License as published by the Free Software
#    Foundation. See <https://github.com/itom-project/itom>
#
#    You should have received a copy of the GNU Library General Public License
#    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

/*!
\file
\brief
\author twip optical solutions GmbH


*/

#ifndef GFRAMEWORK_H
#define GFRAMEWORK_H

#include <string.h>

//----------------------------------------------------------------------------------------------------------------------------------
//! A class defining different line types, which can differ in line style, line width and color
class LineStyle
{
    public:
        //! The different line styles, which can be selected
        enum lStyle {
            solid   = 0,
            dot     = 1,
            dash    = 2,
            dotdash = 3,
            user    = 4
        };

        //! Basic NULL constructor
        LineStyle() : m_lineWidth(1), m_lineColor(0), m_lineStyle(LineStyle::solid) {}

        LineStyle(int width, int color, lStyle style = solid) : m_lineWidth(width), m_lineColor(color), m_lineStyle(style) {}
        ~LineStyle() {}

        //! Get the lineStyle occurring to the enum lStyle
        inline lStyle getLineStyle() { return m_lineStyle; }

        //! Set the lineStyle occurring to the enum lStyle. Returns -1 if failed.
        inline int setLineStyle(lStyle lineStyle) { m_lineStyle = lineStyle; return 0; }

        //! Get the width as a signed integer
        inline int getLineWidth() { return m_lineWidth; }

        //! Set the linewidth between 0 and 100. Returns -1 if failed.
        inline int setLineWidth(int lineWidth) { if (lineWidth < 0 || lineWidth > 100) return -1; m_lineWidth = lineWidth; return 0; }

        //! Get the lineColor coded as RGB in a signed integer form
        inline int getLineColor() { return m_lineColor;}

        //! Set the lineColor as signed int (RGB). Returns -1 if failed.
        inline int setLineColor(int lineColor) { if (lineColor < 0 || lineColor > 16581375) return -1; m_lineColor = lineColor; return 0; }

        //! The width of this line
        int m_lineWidth;

        //! The color of this line as an unsigned int containing RGB
        ito::uint32 m_lineColor;

        //! The linestyle of type lStyle, representing how the line should look like.
        lStyle m_lineStyle;
};

//----------------------------------------------------------------------------------------------------------------------------------
//! A class defining different font types, which can differ in bold, italic, regular, color, size and font family
class FontStyle
{
    public:
        //! Basic NULL constructor
        FontStyle() : m_fontSize(12), m_bold(0), m_italic(0), m_fontName("Arial"), m_fontColor(0) {}
        FontStyle(int size, std::string name = "Arial", bool bold = false, bool italic = false, ito::uint32 fontColor = 0) : m_fontSize(size),
            m_fontName(name), m_bold(bold), m_italic(italic), m_fontColor(fontColor) {}
        ~FontStyle() {}

        //! Get the current font size
        inline int getFontSize() { return m_fontSize; }

        //! Set the font size between 0 and 100. Returns -1 if failed.
        inline int setFontSize(int size) { if (size < 0 || size > 100) return -1; m_fontSize = size; return 0; }

        //! Get the font color coded as RGB in an  unsigned integer form
        inline ito::uint32 getFontColor() { return m_fontColor; }

        //! Set the font color as based on unsigned int (RGB). Seht the color to white if input is above RGB.
        inline void setFontColor(const ito::uint32 fontColor) { m_fontColor = fontColor < 0 ? 0 : fontColor > 0x00FFFFFF ? 0x00FFFFFF : fontColor ; }

        //! Check of the current font is bold
        inline bool getBold() { return m_bold; }

        //! Check of the current font is italic
        inline bool getItalic() { return m_italic; }

        //! Set the current font to bold
        inline int setBold(bool bold)
        {
            m_bold = bold;
            return 0;
        }

        //! Set the current font to italic
        inline int setItalic(bool italic)
        {
            m_italic = italic;
            return 0;
        }

        //! Get the name of the font as std::string
        inline std::string getFontName() { return m_fontName; }

        //! Set the current font name, return -1 of failed
        inline int setFontName(std::string name) { m_fontName = name; return 0; }

        //! Convert the font to QFont
        inline QFont getQFont() const
        {
            return QFont(QString::fromStdString(m_fontName), m_fontSize, m_bold, m_italic);
        }

        //! Set this font according to QFont parameters
        inline int fromQFont(QFont newFont)
        {
            m_fontSize = newFont.pointSize();
            m_fontName = newFont.family().toStdString();
            m_bold = newFont.bold();
            m_italic= newFont.italic();
            return 0;
        }

        //! Internal variable containing the fontSize
        int m_fontSize;

        //! Internal variable defining if font is italic
        bool m_italic;

        //! Internal variable defining if font is bold
        bool m_bold;

        //! Internal variable containing the name of the font family
        std::string m_fontName;

        //! The color of this font as an unsigned int containing RGB
        ito::uint32 m_fontColor;
};

//----------------------------------------------------------------------------------------------------------------------------------
//! A class defining ticks styles
class Ticks : public LineStyle
{
};

//----------------------------------------------------------------------------------------------------------------------------------
//! A class defining the axis scaling and range
class Scaling
{
    public:
        Scaling() : m_sizePx(1), m_scaleAU(1) {}
        Scaling(int size, ito::float64 scale = 1.0) : m_sizePx(size), m_scaleAU(scale)
        {
            if (fabs(scale) <= std::numeric_limits<ito::float64>::epsilon())
            {
                m_scaleAU = 1.0;
            }
            if(scale != scale)
            {
                m_scaleAU = 1.0;
            }
        }
        ~Scaling() {}
        inline int getSizePx() { return m_sizePx; }
        inline int setSizePx(int size) { m_sizePx = size; return 0; }
        inline ito::float64 getSizeAU() { return m_sizePx * m_scaleAU; }
        inline int setScaleAU(ito::float64 scale) { if (fabs(scale) <= std::numeric_limits<ito::float64>::epsilon()) return -1; m_scaleAU = scale; return 0; }
        inline ito::float64 transPx2AU(ito::float64 px) { return px * m_scaleAU; }
        inline ito::float64 transAU2Px(ito::float64 au) { return au / m_scaleAU; }

        int m_sizePx;               //!> length in pixel
        ito::float64 m_scaleAU;     //!> conversion factor from pixels size to physical size
};

//----------------------------------------------------------------------------------------------------------------------------------
//! A class for plot markers
class Marker
{
};

//----------------------------------------------------------------------------------------------------------------------------------
//! A class defining the appearance of the color bar
class ColorBar : protected Scaling, protected LineStyle, protected FontStyle
{
    public:
        ColorBar() : m_title(""), m_unit("AU"), m_position(0) {}
        ~ColorBar() {}
        inline std::string getTitle() { return m_title; }
        inline int setTitle(std::string title) { m_title = title; return 0; }
        inline std::string getUnit() { return m_unit; }
        inline int setUnit(std::string unit) { m_unit = unit; return 0; }
        inline int getPosition() { return m_position; }
        inline int setPosition(int position) { m_position = position; return 0; }

        Ticks m_majorTicks;
        Ticks m_minorTicks;
        std::string m_title;    //!> colorbar title
        std::string m_unit;     //!> colorbar unit
        int m_position;         //!> position
};

//----------------------------------------------------------------------------------------------------------------------------------
/** \class Axis
*   \brief Single axis with scaling, unit, dimensions, label and all functions to handle that
*/
class Axis: public Scaling
{
    public:
        Axis() : m_isflipped(false), m_min(0), m_max(0), m_idx0(0), m_idx1(0), m_autoscale(1),
            m_idxDim(0), m_idxMax(0), m_label(""), m_unit("AU"), m_isMetric(0), m_isVisible(1) {}
        ~Axis() {}

        //!> get axis label
        inline std::string getLabel() { return m_label; }

        /** set axis label
        *   @param [in] label   new axis label
        */
        inline int setLAbel(std::string label) { m_label = label; return 0; }

        //!> get axis unit
        inline std::string getUnit() { return m_unit; }

        /** set axis unit
        *   @param [in] unit    new axis unit
        */
        inline int setUnit(std::string unit) { m_unit = unit; return 0; }

        //!> set axis minimum value, some boundary checks are done for indices
        inline void setMin(ito::float64 min)
        {
            m_min = min;
            m_idx0 = ceil(transAU2Px(min));
            if (m_idx0 < 0)
                m_idx0 = 0;
            if (m_idx0 > m_idxMax)
                m_idx0 = m_idxMax;
        }

        //!> set axis maximum value, some boundary checks are done for indices
        inline void setMax(ito::float64 max)
        {
            m_max = max;
            m_idx1 = floor(transAU2Px(max));
            if (m_idx1 < 0)
                m_idx1 = 0;
            if (m_idx1 > m_idxMax)
                m_idx1 = m_idxMax;
        }

        //!> return axis minimum (physical value)
        inline ito::float64 getMin(void) { return m_min; }

        //!> return axis maximum (physical value)
        inline ito::float64 getMax(void) { return m_max; }

        //!> return axis minimum index
        inline ito::int32 getIdx0(void) { return m_idx0; }

        //!> return axis maximum index
        inline ito::int32 getIdx1(void) { return m_idx1; }

        //!> set axis minimum index, some boundary checks are done for index
        inline void setIdx0(ito::int32 idx0)
        {
            m_idx0 = idx0;
            if (m_idx0 < 0)
                m_idx0 = 0;
            if (m_idx0 > m_idxMax)
                m_idxMax = m_idx0;
        }

        //!> set axis maximum index, some boundary checks are done for index
        inline void setIdx1(ito::int32 idx1)
        {
            m_idx1 = idx1;
            if (m_idx1 < 0)
                m_idx1 = 0;
            if (m_idx1 > m_idxMax)
                m_idxMax = m_idx1;
        }
        inline ito::int8 getIdxDim(void) { return m_idxDim; }
        inline void setIdxDim(ito::int8 idxDim) { m_idxDim = idxDim; }
        inline ito::int8 getAutoscale(void) { return m_autoscale; }
        inline void setAutoscale(ito::int8 autoscale)
        {
            if (m_autoscale)
                autoscale = 1;
            else
                autoscale = 0;
        }
        inline ito::float64 getLength(void)
        {
            float tmpLen = transPx2AU(transAU2Px(m_max) - transAU2Px(m_min));
            return (tmpLen == 0) ? 1.0 : tmpLen;
        }
        Ticks m_majorTicks;
        Ticks m_minorTicks;
        std::string m_label;    //!> axis label
        std::string m_unit;     //!> axis unit
        ito::float64 m_min;     //!> axis minimum
        ito::float64 m_max;     //!> axis maximum
        ito::int32 m_idx0;      //!> axis minimum index in data
        ito::int32 m_idx1;      //!> axis maximum index in data
        ito::int32 m_idxMax;    //!> highest possible index in data
        ito::uint8 m_autoscale; //!> flag for axis autoscaling (min, max)
        ito::int8 m_idxDim;     //!> axis dimension index
        ito::int8 m_isMetric;
        ito::int8 m_isVisible;
        bool m_isflipped;
};

//----------------------------------------------------------------------------------------------------------------------------------
/** \class Axes
*   container class holding all axis, used to define unique axes appearance
*/
class Axes : public Axis, public LineStyle, public FontStyle
{
    public:
        Axes() {}
        ~Axes() {}

        Axis m_axisX;
        Axis m_axisY;
        Axis m_axisZ;
        Axis m_devAxis;
};

//----------------------------------------------------------------------------------------------------------------------------------
#endif //#ifndef GFRAMEWORK_H

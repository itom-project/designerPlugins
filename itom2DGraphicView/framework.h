class LineStyle
{
    public:
        enum lStyle {
            solid 0,
            dot 1,
            dash 2,
            dotdash 3,
            user 4
        };

        LineStyle() : m_lineWidth(1), m_lineColor(0), m_lineStyle(0) {}
        LineStyle(int width, int color, lStyle style = solid) : m_lineWidth(width), m_lineColor(color), m_lineStyle(style) {}
        ~LineStyle() {}
        inline lStyle getLineStyle() { return m_lineStyle; }
        inline int setLineStyle(lStyle lineStyle) { m_lineStyle = lineStyle; return 0; }
        inline int getLineWidth() { return m_lineWidth; }
        inline int setLineWidth(int lineWidth) { if (lineWidth < 0 || lineWidth > 100) return -1; m_lineWidth = lineWidth; return 0; }
        inline int getLineColor() { return m_lineColor;}
        inline int setLineColor(int lineColor) { if (lineColor < 0 || lineColor > 16581375) return -1; m_lineColor = lineColor; return 0; }

    private:
        int m_lineWidth;
        int m_lineColor;
        lStyle m_lineStyle;
};

class FontStyle
{
    public:
        enum fweight {
            normal 0,
            bold 1,
            italic 2
        };
        FontStyle() : m_fontSize(12), m_fontWeight(normal), m_fontName("Arial") {}
        FontStyle(int size, fweight weight, std::string name = "Arial") : m_size(size), m_fontWeight(weight), m_fontName(name) {}
        ~FontStyle() {}
        inline int getFontSize() { return m_fontSize; }
        inline int setFontSize(int size) { if (size < 0 || size > 100) return -1; m_size = size; return 0; }
        inline fweight getFontWeight() { return m_fontWeight; }
        inline int setFontWeight(fweight weight) { m_fontWeight = weight; return 0; }
        inline std::string getFontName() { return m_fontName; }
        inline int setFontName(std::string name) { m_fontName = name; return 0; }

    private:
        int m_fontSize;
        fweight m_fontWeight;
        std::string m_fontName;
};

class Ticks : public LineStyle
{
};


class Scaling
{
    public:
        Scaling() : m_sizePx(1), m_scaleAU(1) {}
        Scaling(int size, ito::float64 scale = 1) : m_sizePx(size), m_scaleAU(scale) {}
        ~Scaling() {}
        inline int getSizePx() { return m_sizePx; }
        inline int setSizePx(int size) { m_sizePx = size; return 0; }
        inline ito::float64 getSizeAU() { return m_sizePx * m_scaleAU; }
        inline int setScaleAU(ito::float64 scale) { if (fabs(scale) <= std::numeric_limits<ito::float64>::epsilon()) return -1; m_scaleAU = scale; return 0; }
        inline ito::float64 transPx2AU(int px) { return px * m_scaleAU; }
        inline ito::float64 transAU2Px(ito::float64 au) { return au / m_scaleAU; }

    private:
        int m_sizePx;
        ito::float64 m_scaleAU;
};

class Marker
{
};

class ColorBar : public: Scaling, public LineStyle, public FontStyle
{
    public:
        inline std::string getTitle() { return m_title; }
        inline int setTitle(std::string title) { m_title = title; return 0; }
        inline std::string getUnit() { return m_unit; }
        inline int setUnit(std::string unit) { m_unit = unit; return 0; }
        inline int getPosition() { return m_position; }
        inline int setPosition(int position) { m_position = position; return 0; }

    private:
        Ticks m_majorTicks;
        Ticks m_minorTicks;
        std::string m_title;
        std::string m_unit;
        int m_position;
};


class Axis: public Scaling, public LineStyle, public FontStyle
{
    public:
        inline std::string getTitle() { return m_title; }
        inline int setTitle(std::string title) { m_title = title; return 0; }
        inline std::string getUnit() { return m_unit; }
        inline int setUnit(std::string unit) { m_unit = unit; return 0; }

    private:
        Ticks m_majorTicks;
        Ticks m_minorTicks;
        std::string m_title;
        std::string m_unit;
};

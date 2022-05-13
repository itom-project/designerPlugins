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
#    itom is free software by ITO, University Stuttgart published under 
#    GNU General Public License as published by the Free Software 
#    Foundation. See <https://bitbucket.org/itom/> 
#
#    You should have received a copy of the GNU Library General Public License
#    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */
#ifndef SHADERENGNIES_H
#define SHADERENGINES_H

    //! fragment and vertex shaders for gl v2 and gl v3
    //! the fragment shader multiplies input vertices with the transformation matrix MVP, the
    //! fragment shader calculates the texture pixel (and color) for each pixel. In addition a
    //! gamma correction can be applied using a simple lookup vektor (lutarr)

    const char *VERT_3D = "             \
    #version 110                        \n\
                                        \
    uniform mat4 MVP;                   \
    uniform mat4 VCT;                   \
    uniform int useLigthing;            \
    uniform int diffMode;               \
    attribute vec3 vertex;              \
    attribute vec3 normal;              \
    attribute float vertColor;          \
    attribute float diff;               \
    attribute vec3 texture;             \
    varying vec3 cTexture;              \
    varying float vColor;               \
    varying vec3 vNormal;               \
    varying float vDiff;                \
                                        \
    void main()                         \
    {                                   \
        gl_Position = MVP * VCT * vec4(vertex.xyz, 1.0); \
        vColor = vertColor;             \
        vNormal = normal;               \
        cTexture = texture;             \
        vDiff = diff;                   \
    }                                   \
    ";

    const char *VERT_3D_130 = "         \
    #version 130                        \n\
                                        \
    uniform mat4 MVP;                   \
    uniform mat4 VCT;                   \
    uniform int useLigthing;            \
    uniform int diffMode;               \
    in vec3 vertex;                     \
    in vec3 normal;                     \
    in float vertColor;                 \
    in float diff;                      \
    in vec3 texture;                    \
    out vec3 cTexture;                  \
    out float vColor;                   \
    out float vDiff;                    \
    out vec3 vNormal;                   \
                                        \
    void main()                         \
    {                                   \
        gl_Position = MVP * VCT * vec4(vertex.xyz, 1.0); \
        vColor = vertColor;             \
        vNormal = normal;               \
        cTexture = texture;             \
        vDiff = diff;                   \
    }                                   \
    ";

    const char *FRAG_3D = "             \
    #version 110                        \n\
                                        \
    uniform vec3 palette[256];          \
    uniform int usePalette;             \
    uniform vec3 glColor;               \
    uniform vec4 invColor;              \
    uniform int useLigthing;            \
    uniform vec4 lColor;                \
    uniform vec4 ambient;               \
    uniform vec3 diffuseDir;            \
    uniform vec4 diffuse;               \
    uniform float useTexture;           \
    uniform int diffMode;               \
    uniform float diffNorm;             \
    uniform float diffMin;              \
    uniform float curAlpha;             \
    varying vec3 cTexture;              \
    varying float vColor;               \
    varying float vDiff;                \
    varying vec3 vNormal;               \
                                        \
    void main()                         \
    {                                   \
        if (usePalette == 0)            \
        {                               \
            gl_FragColor = vec4(glColor, 1.0); \
        }                               \
        else if (usePalette == 1)       \
        {                               \
            if (diffMode == 1)          \
            {                           \
                int tmpColor = int((vDiff - diffMin) * diffNorm * 255.0);\
                if (tmpColor < 0)       \
                    tmpColor = 0;       \
                else if (tmpColor > 255)\
                    tmpColor = 255;     \
                gl_FragColor = vec4(palette[tmpColor], curAlpha); \
            }                           \
            else                        \
            {                           \
                int tmpColor = int(vColor); \
                if(tmpColor < 0)        \
                    gl_FragColor = vec4(invColor); \
                else                    \
                    gl_FragColor = vec4(palette[tmpColor], curAlpha); \
            }                           \
        }                               \
        else                            \
        {                               \
            gl_FragColor = vec4(vColor, vColor, vColor, curAlpha); \
        }                               \
        if (useTexture > 0.0)           \
        {                               \
            gl_FragColor = (1.0 - useTexture) * gl_FragColor + useTexture * vec4(cTexture, curAlpha); \
        }                               \
        if (useLigthing == 1)           \
        {                               \
            vec4 dif = diffuse * clamp(abs(dot(normalize(vNormal), diffuseDir)), 0.0, 1.0); \
            gl_FragColor = gl_FragColor * (ambient + dif);\
        }                               \
    }                                   \
    ";

    const char *FRAG_3D_130 = "         \
    #version 130                        \n\
                                        \
    uniform vec3 palette[256];          \
    uniform int usePalette;             \
    uniform vec3 glColor;               \
    uniform vec4 invColor;              \
    uniform int useLigthing;            \
    uniform vec4 lColor;                \
    uniform vec4 ambient;               \
    uniform vec3 diffuseDir;            \
    uniform vec4 diffuse;               \
    uniform float useTexture;           \
    uniform int diffMode;               \
    uniform float diffNorm;             \
    uniform float diffMin;              \
    uniform float curAlpha;             \
    in vec3 cTexture;                   \
    in float vColor;                    \
    in float vDiff;                     \
    in vec3 vNormal;                    \
    out vec4 FragColor;                 \
                                        \
    void main()                         \
    {                                   \
        if (usePalette == 0)            \
        {                               \
            FragColor = vec4(glColor, 1.0); \
        }                               \
        else if (usePalette == 1)       \
        {                               \
            if (diffMode == 1)          \
            {                           \
                int tmpColor = int((vDiff - diffMin) * diffNorm * 255.0);\
                if (tmpColor < 0)       \
                    tmpColor = 0;       \
                else if (tmpColor > 255)\
                    tmpColor = 255;     \
                FragColor = vec4(palette[tmpColor], curAlpha); \
            }                           \
            else                        \
            {                           \
                int tmpColor = int(vColor); \
                if(tmpColor < 0)        \
                    FragColor = vec4(invColor); \
                else                   \
                    FragColor = vec4(palette[tmpColor], curAlpha); \
            }                           \
        }                               \
        else                            \
        {                               \
            FragColor = vec4(vColor, vColor, vColor, curAlpha); \
        }                               \
        if (useTexture > 0.0)           \
        {                               \
            FragColor = (1.0 - useTexture) * FragColor + useTexture * vec4(cTexture, curAlpha); \
        }                               \
        if (useLigthing == 1)           \
        {                               \
            vec4 dif = diffuse * clamp(abs(dot(normalize(vNormal), diffuseDir)), 0.0, 1.0); \
            FragColor = FragColor * (ambient + dif);\
        }                               \
    }                                   \
    ";

    const char *VERT_3DPRI = "          \
    #version 110                        \n\
                                        \
    uniform mat4 MVP;                   \
    uniform mat4 VCT;                   \
    attribute vec3 vertex;              \
                                        \
    void main()                         \
    {                                   \
        gl_Position = MVP * VCT * vec4(vertex.xyz, 1.0);    \
    }                                   \
    ";

    const char *VERT_3DPRI_130 = "      \
    #version 130                        \n\
                                        \
    uniform mat4 MVP;                   \
    uniform mat4 VCT;                   \
    in vec3 vertex;                     \
                                        \
    void main()                         \
    {                                   \
        gl_Position = MVP * VCT * vec4(vertex.xyz, 1.0); \
    }                                   \
    ";

    const char *FRAG_3DPRI = "          \
    #version 110                        \n\
                                        \
    uniform vec3 glColor;               \
                                        \
    void main()                         \
    {                                   \
        gl_FragColor = vec4(glColor, 1.0); \
    }                                   \
    ";

    const char *FRAG_3DPRI_130 = "      \
    #version 130                        \n\
                                        \
    uniform vec3 glColor;               \
    out vec4 FragColor;                 \
                                        \
    void main()                         \
    {                                   \
        FragColor = vec4(glColor, 1.0); \
    }                                   \
    ";

    const char *VERT_2DPX = "           \
    #version 110                        \n\
    attribute vec3 inVert;              \
    attribute vec2 inUV;                \
    varying vec2 vertUV;                \
    uniform float scaleX;               \
    uniform float scaleY;               \
    uniform float posX;                 \
    uniform float posY;                 \
                                        \
    void main()                         \
    {                                   \
        vertUV = inUV;                  \
        gl_Position = vec4(((posX + inVert.x) * scaleX) - 1.0, \
            ((posY + inVert.y) * -1.0 * scaleY) + 1.0, 0.0, 1.0);     \
    }                                   \
    ";

    const char *VERT_2DPX_130 = "       \
    #version 130                        \n\
    in vec3 inVert;                     \
    in vec2 inUV;                       \
    out vec2 vertUV;                    \
    uniform float scaleX;               \
    uniform float scaleY;               \
    uniform float posX;                 \
    uniform float posY;                 \
                                        \
    void main()                         \
    {                                   \
        vertUV = inUV;                  \
        gl_Position = vec4(posX * scaleX + inVert.x * scaleX - 1.0, \
            (posY + inVert.y) * -1.0 * scaleY + 1.0, 0.0, 1.0);     \
    }                                   \
    ";

    const char *FRAG_2DPX = "           \
    #version 110                        \n\
                                        \
    uniform sampler2D tex;              \
    varying vec2 vertUV;                \
    uniform vec3 textColor;             \
    uniform int useTex;                 \
    uniform vec3 glColor;               \
                                        \
    void main()                         \
    {                                   \
        if (useTex == 2)                \
        {                               \
            gl_FragColor = texture2D(tex, vertUV.st); \
        }                               \
        else if (useTex == 1)           \
        {                               \
            vec4 text = texture2D(tex, vertUV.st); \
            gl_FragColor.rgb = textColor.rgb; \
            gl_FragColor.a = text.a;    \
        }                               \
        else                            \
        {                               \
            gl_FragColor = vec4(glColor, 1.0);\
        }                               \
    }                                   \
    ";

    const char *FRAG_2DPX_130 = "       \
    #version 130                        \n\
                                        \
    uniform sampler2D tex;              \
    in vec2 vertUV;                     \
    out vec4 fragColor;                 \
    uniform vec3 textColor;             \
    uniform int useTex;                 \
    uniform vec3 glColor;               \
                                        \
    void main()                         \
    {                                   \
        if (useTex == 2)                \
        {                               \
            fragColor = texture(tex, vertUV.st); \
        }                               \
        else if (useTex == 1)           \
        {                               \
            vec4 text = texture(tex, vertUV.st); \
            fragColor.rgb = textColor.rgb; \
            fragColor.a = text.a;       \
        }                               \
        else                            \
        {                               \
            fragColor = vec4(glColor, 1.0);\
        }                               \
    }                                   \
    ";

#endif //SHADERENGNIES_H

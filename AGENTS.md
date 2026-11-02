# AGENTS.md — itom designer plugins

Guidance for AI coding agents working in the **designerplugins** repository.
Normally used as a submodule of `itomProject`.

## What is in here

Qt **Designer custom widgets** that are at the same time usable as itom figures/plots
from Python:

| Folder                                    | Purpose                                    |
| ----------------------------------------- | ------------------------------------------ |
| `itomQwtPlotBase`                         | shared Qwt-based plot infrastructure       |
| `itom1DQwtPlot`, `itom2DQwtPlot`          | 1D/2D line and image plots                 |
| `itomIsoGLFigurePlugin`, `twipOGLPlugin`  | OpenGL based figures                       |
| `matplotlibPlot`, `plotlyPlot`            | embedding of external plotting backends    |
| `vtk3dVisualizer`                         | 3D point cloud / mesh visualisation (PCL/VTK) |
| `evaluateGeometrics`, `motorController`, `slider2D` | interactive control/analysis widgets |
| `qwt`                                     | bundled Qwt sources                        |
| `cmake`, `icons`, `itomDesignerPlugins.qrc` | build helpers and resources              |

## Hard rules

1. Each plugin implements `QDesignerCustomWidgetInterface` (registered in the designer
   plugin collection) **and**, for figures, derives from the itom plot interfaces
   (`ito::AbstractFigure`, `ito::AbstractDObjFigure`, `ito::AbstractDObjPclFigure`, ...)
   from the core repository.
2. **Properties exposed to Designer and Python must be `Q_PROPERTY`** with the same
   flags/`designable` conventions as the existing widgets, plus a doc string entry where
   the surrounding code provides one. Property names are part of the public API — do not
   rename them.
3. **Do not change the core plot interfaces from here.** If something is missing, report
   it instead of patching around it.
4. **Rendering/UI code runs in the GUI thread**; keep expensive work out of `paintEvent`
   and use the existing replot/throttling mechanisms.
5. `ito::DataObject` inputs may have any type, any ROI and any number of planes — handle
   the plane/ROI semantics as in the existing plot classes.
6. Icons and other resources go through `itomDesignerPlugins.qrc`; do not load files by
   absolute path.
7. `qwt/` is a bundled third party library — do not patch it unless explicitly asked.

## Style

- `.clang-format` (`BasedOnStyle: Microsoft`, 4 spaces, `ColumnLimit: 100`,
  `PointerAlignment: Left`). Format only the lines you change.
- English identifiers and comments; user visible strings via `tr(...)`.

## Build

Built as part of `itomProject` with `BUILD_ITOM_DESIGNERPLUGINS=ON`:

Never edit files inside the CMake binary directory or generated `moc_*`/`ui_*`/`qrc_*`
files.

## Quality gates

`pre-commit`: `check-yaml`, `end-of-file-fixer`, `trailing-whitespace`,
`fix-byte-order-marker`, `codespell`, `pyupgrade --py36-plus`, `sphinx-lint`.
Files end with a single newline, no trailing whitespace, no BOM.

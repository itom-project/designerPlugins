# Copilot instructions — itom designer plugins

The tool-independent instructions for this repository live in
[`AGENTS.md`](../AGENTS.md). **Read and follow that file.**

Quick reminders:

- These are Qt Designer custom widgets (`QDesignerCustomWidgetInterface`) that are also
  used as itom figures/plots from Python.
- Plot classes derive from the itom plot interfaces of the core repository;
  `itomQwtPlotBase` is the shared base for the Qwt-based plots.
- Public properties are `Q_PROPERTY` and part of the API — do not rename them.
- Handle `ito::DataObject` inputs with arbitrary type, ROI and plane count.
- Resources go through `itomDesignerPlugins.qrc`
- `qwt/` is bundled third party code.
- Formatting via `.clang-format`; format only the lines you touch.

# SpiralTool

Command-line tool and GUI viewer for generating and visualizing Fermat spiral
scan waveforms.

## Building SpiralTool

Build the project with Meson as usual. The executable is produced at
`builddir/SpiralTool/SpiralTool.exe`.

## Spiral Viewer GUI

Interactive GUI for adjusting spiral scan parameters and viewing the resulting
waveforms.

### Running

```sh
cd SpiralTool
uv run spiral_viewer.py
```

`uv` will automatically install the dependencies (`dearpygui`, `numpy`) on
first run. The viewer auto-detects `SpiralTool.exe` from the build directory.

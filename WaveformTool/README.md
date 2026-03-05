# WaveformTool

Command-line tool and GUI viewer for generating and visualizing galvo scan
waveforms.

## Building WaveformTool

Build the project with Meson as usual. The executable is produced at
`builddir/WaveformTool/WaveformTool.exe`.

## Waveform Viewer GUI

Interactive GUI for adjusting scan parameters and viewing the resulting
waveforms.

### Running

```sh
cd WaveformTool
uv run waveform_viewer.py
```

`uv` will automatically install the dependencies (`dearpygui`, `numpy`) on
first run. The viewer auto-detects `WaveformTool.exe` from the build directory.

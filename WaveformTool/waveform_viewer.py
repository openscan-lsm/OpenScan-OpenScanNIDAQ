# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "dearpygui",
#     "numpy",
# ]
# ///

import subprocess
import sys
import tempfile
from pathlib import Path

import dearpygui.dearpygui as dpg
import numpy as np


def find_waveform_tool() -> Path:
    script_dir = Path(__file__).resolve().parent
    candidates = [
        script_dir / "WaveformTool.exe",
        script_dir / "WaveformTool",
        script_dir.parent / "builddir" / "WaveformTool" / "WaveformTool.exe",
        script_dir.parent / "builddir" / "WaveformTool" / "WaveformTool",
    ]
    for p in candidates:
        if p.is_file():
            return p
    sys.exit(
        "Cannot find WaveformTool executable. Searched:\n"
        + "\n".join(f"  {c}" for c in candidates)
    )


TOOL_PATH = find_waveform_tool()


def generate_waveform(
    resolution: int,
    width: int,
    height: int,
    x_offset: int,
    y_offset: int,
    zoom: float,
    undershoot: int,
    tform: tuple[float, float, float, float],
    tform_offset: tuple[float, float],
) -> tuple[np.ndarray, np.ndarray] | str:
    with tempfile.NamedTemporaryFile(suffix=".raw", delete=False) as f:
        tmp = Path(f.name)
    try:
        cmd = [
            str(TOOL_PATH),
            "raster",
            "-o",
            str(tmp),
            "--format",
            "raw",
            "--resolution",
            str(resolution),
            "--width",
            str(width),
            "--height",
            str(height),
            "--xoffset",
            str(x_offset),
            "--yoffset",
            str(y_offset),
            "--zoom",
            str(zoom),
            "--undershoot",
            str(undershoot),
            "--tform",
            f"{tform[0]},{tform[1]},{tform[2]},{tform[3]}",
            "--tform-offset",
            f"{tform_offset[0]},{tform_offset[1]}",
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            return result.stderr.strip() or f"WaveformTool exited with code {result.returncode}"
        data = np.fromfile(tmp, dtype=np.float64)
        n = len(data) // 2
        if n == 0:
            return "No data produced"
        return data[:n], data[n:]
    finally:
        tmp.unlink(missing_ok=True)


def on_generate(_sender=None, _data=None):
    resolution = dpg.get_value("resolution")
    use_res = dpg.get_value("size_equals_res")
    width = resolution if use_res else dpg.get_value("width")
    height = resolution if use_res else dpg.get_value("height")
    x_offset = dpg.get_value("x_offset")
    y_offset = dpg.get_value("y_offset")
    zoom = dpg.get_value("zoom")
    undershoot = dpg.get_value("undershoot")
    tform = (
        dpg.get_value("tform_a"),
        dpg.get_value("tform_b"),
        dpg.get_value("tform_c"),
        dpg.get_value("tform_d"),
    )
    tform_offset = (dpg.get_value("tform_tx"), dpg.get_value("tform_ty"))

    result = generate_waveform(
        resolution, width, height, x_offset, y_offset, zoom, undershoot,
        tform, tform_offset,
    )

    if isinstance(result, str):
        dpg.set_value("status", f"Error: {result}")
        return

    x, y = result
    t = np.arange(len(x), dtype=np.float64)
    dpg.set_value("status", f"Generated {len(x)} samples per channel")

    dpg.set_value("x_series", [t.tolist(), x.tolist()])
    dpg.set_value("y_series", [t.tolist(), y.tolist()])
    dpg.set_value("xy_series", [x.tolist(), y.tolist()])

    dpg.fit_axis_data("x_ax_x")
    dpg.fit_axis_data("x_ax_y")
    dpg.fit_axis_data("y_ax_x")
    dpg.fit_axis_data("y_ax_y")
    dpg.fit_axis_data("xy_ax_x")
    dpg.fit_axis_data("xy_ax_y")


def on_size_toggle(_sender=None, _data=None):
    linked = dpg.get_value("size_equals_res")
    dpg.configure_item("width", enabled=not linked)
    dpg.configure_item("height", enabled=not linked)
    on_generate()


def main():
    dpg.create_context()
    dpg.create_viewport(title="Waveform Viewer", width=1400, height=800)

    with dpg.window(tag="main_window"):
        with dpg.group(horizontal=True):
            # Left panel: controls
            with dpg.child_window(width=300):
                dpg.add_text("Scan Parameters")
                dpg.add_separator()

                dpg.add_input_int(
                    label="Resolution", tag="resolution",
                    default_value=256, min_value=1, min_clamped=True,
                    callback=on_generate,
                )
                dpg.add_checkbox(
                    label="Width/Height = Resolution", tag="size_equals_res",
                    default_value=True, callback=on_size_toggle,
                )
                dpg.add_input_int(
                    label="Width", tag="width",
                    default_value=256, min_value=1, min_clamped=True,
                    enabled=False, callback=on_generate,
                )
                dpg.add_input_int(
                    label="Height", tag="height",
                    default_value=256, min_value=1, min_clamped=True,
                    enabled=False, callback=on_generate,
                )
                dpg.add_input_int(
                    label="X Offset", tag="x_offset",
                    default_value=0, min_value=0, min_clamped=True,
                    callback=on_generate,
                )
                dpg.add_input_int(
                    label="Y Offset", tag="y_offset",
                    default_value=0, min_value=0, min_clamped=True,
                    callback=on_generate,
                )
                dpg.add_input_float(
                    label="Zoom", tag="zoom",
                    default_value=1.0, min_value=0.01, min_clamped=True,
                    format="%.3f", callback=on_generate,
                )
                dpg.add_input_int(
                    label="Undershoot", tag="undershoot",
                    default_value=0, min_value=0, min_clamped=True,
                    callback=on_generate,
                )

                dpg.add_separator()
                dpg.add_text("Affine Transform")

                dpg.add_input_float(
                    label="a", tag="tform_a", default_value=1.0,
                    format="%.6f", callback=on_generate,
                )
                dpg.add_input_float(
                    label="b", tag="tform_b", default_value=0.0,
                    format="%.6f", callback=on_generate,
                )
                dpg.add_input_float(
                    label="c", tag="tform_c", default_value=0.0,
                    format="%.6f", callback=on_generate,
                )
                dpg.add_input_float(
                    label="d", tag="tform_d", default_value=1.0,
                    format="%.6f", callback=on_generate,
                )
                dpg.add_input_float(
                    label="tx (V)", tag="tform_tx", default_value=0.0,
                    format="%.6f", callback=on_generate,
                )
                dpg.add_input_float(
                    label="ty (V)", tag="tform_ty", default_value=0.0,
                    format="%.6f", callback=on_generate,
                )

                dpg.add_separator()
                dpg.add_button(label="Generate", callback=on_generate, width=-1)
                dpg.add_text("", tag="status")

            # Right panel: plots
            with dpg.group():
                with dpg.plot(label="X(t)", height=230, width=-1):
                    dpg.add_plot_axis(dpg.mvXAxis, label="Sample", tag="x_ax_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="X (V)", tag="x_ax_y"):
                        dpg.add_line_series([], [], tag="x_series")

                with dpg.plot(label="Y(t)", height=230, width=-1):
                    dpg.add_plot_axis(dpg.mvXAxis, label="Sample", tag="y_ax_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="Y (V)", tag="y_ax_y"):
                        dpg.add_line_series([], [], tag="y_series")

                with dpg.plot(label="X-Y Trajectory", height=-1, width=-1, equal_aspects=True):
                    dpg.add_plot_axis(dpg.mvXAxis, label="X (V)", tag="xy_ax_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="Y (V)", tag="xy_ax_y"):
                        dpg.add_line_series([], [], tag="xy_series")

    dpg.set_primary_window("main_window", True)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    on_generate()

    dpg.start_dearpygui()
    dpg.destroy_context()


if __name__ == "__main__":
    main()

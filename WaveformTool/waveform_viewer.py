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


def run_tool(args: list[str], tmp: Path) -> subprocess.CompletedProcess:
    cmd = [str(TOOL_PATH)] + args + ["-o", str(tmp), "--format", "raw"]
    return subprocess.run(cmd, capture_output=True, text=True)


def generate_raster(
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
        result = run_tool([
            "raster",
            "--resolution", str(resolution),
            "--width", str(width),
            "--height", str(height),
            "--xoffset", str(x_offset),
            "--yoffset", str(y_offset),
            "--zoom", str(zoom),
            "--undershoot", str(undershoot),
            "--tform", f"{tform[0]},{tform[1]},{tform[2]},{tform[3]}",
            "--tform-offset", f"{tform_offset[0]},{tform_offset[1]}",
        ], tmp)
        if result.returncode != 0:
            return result.stderr.strip() or f"raster exited with code {result.returncode}"
        data = np.fromfile(tmp, dtype=np.float64)
        n = len(data) // 2
        if n == 0:
            return "raster: no data produced"
        return data[:n], data[n:]
    finally:
        tmp.unlink(missing_ok=True)


def generate_clock(
    width: int,
    height: int,
    undershoot: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | str:
    with tempfile.NamedTemporaryFile(suffix=".raw", delete=False) as f:
        tmp = Path(f.name)
    try:
        result = run_tool([
            "clock",
            "--width", str(width),
            "--height", str(height),
            "--undershoot", str(undershoot),
        ], tmp)
        if result.returncode != 0:
            return result.stderr.strip() or f"clock exited with code {result.returncode}"
        data = np.fromfile(tmp, dtype=np.uint8)
        n = len(data) // 3
        if n == 0:
            return "clock: no data produced"
        return data[:n], data[n : 2 * n], data[2 * n :]
    finally:
        tmp.unlink(missing_ok=True)


def generate_park(
    resolution: int,
    zoom: float,
    undershoot: int,
    x_offset: int,
    y_offset: int,
    xpark: int,
    ypark: int,
    tform: tuple[float, float, float, float],
    tform_offset: tuple[float, float],
) -> tuple[np.ndarray, np.ndarray] | str:
    with tempfile.NamedTemporaryFile(suffix=".raw", delete=False) as f:
        tmp = Path(f.name)
    try:
        result = run_tool([
            "park",
            "--resolution", str(resolution),
            "--zoom", str(zoom),
            "--undershoot", str(undershoot),
            "--xoffset", str(x_offset),
            "--yoffset", str(y_offset),
            "--xpark", str(xpark),
            "--ypark", str(ypark),
            "--tform", f"{tform[0]},{tform[1]},{tform[2]},{tform[3]}",
            "--tform-offset", f"{tform_offset[0]},{tform_offset[1]}",
        ], tmp)
        if result.returncode != 0:
            return result.stderr.strip() or f"park exited with code {result.returncode}"
        data = np.fromfile(tmp, dtype=np.float64)
        n = len(data) // 2
        if n == 0:
            return "park: no data produced"
        return data[:n], data[n:]
    finally:
        tmp.unlink(missing_ok=True)


def generate_unpark(
    resolution: int,
    zoom: float,
    undershoot: int,
    x_offset: int,
    y_offset: int,
    xpark: int,
    ypark: int,
    prev_xpark_voltage: float,
    prev_ypark_voltage: float,
    tform: tuple[float, float, float, float],
    tform_offset: tuple[float, float],
) -> tuple[np.ndarray, np.ndarray] | str:
    with tempfile.NamedTemporaryFile(suffix=".raw", delete=False) as f:
        tmp = Path(f.name)
    try:
        result = run_tool([
            "unpark",
            "--resolution", str(resolution),
            "--zoom", str(zoom),
            "--undershoot", str(undershoot),
            "--xoffset", str(x_offset),
            "--yoffset", str(y_offset),
            "--xpark", str(xpark),
            "--ypark", str(ypark),
            "--prev-xpark-voltage", str(prev_xpark_voltage),
            "--prev-ypark-voltage", str(prev_ypark_voltage),
            "--tform", f"{tform[0]},{tform[1]},{tform[2]},{tform[3]}",
            "--tform-offset", f"{tform_offset[0]},{tform_offset[1]}",
        ], tmp)
        if result.returncode != 0:
            return result.stderr.strip() or f"unpark exited with code {result.returncode}"
        data = np.fromfile(tmp, dtype=np.float64)
        n = len(data) // 2
        if n == 0:
            return "unpark: no data produced"
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
    xpark = dpg.get_value("xpark")
    ypark = dpg.get_value("ypark")
    prev_xpv = dpg.get_value("prev_xpark_voltage")
    prev_ypv = dpg.get_value("prev_ypark_voltage")

    errors = []

    unpark_result = generate_unpark(
        resolution, zoom, undershoot, x_offset, y_offset,
        xpark, ypark, prev_xpv, prev_ypv, tform, tform_offset,
    )
    if isinstance(unpark_result, str):
        errors.append(unpark_result)

    raster_result = generate_raster(
        resolution, width, height, x_offset, y_offset, zoom, undershoot,
        tform, tform_offset,
    )
    if isinstance(raster_result, str):
        errors.append(raster_result)

    park_result = generate_park(
        resolution, zoom, undershoot, x_offset, y_offset,
        xpark, ypark, tform, tform_offset,
    )
    if isinstance(park_result, str):
        errors.append(park_result)

    clock_result = generate_clock(width, height, undershoot)
    if isinstance(clock_result, str):
        errors.append(clock_result)

    # Update X(t), Y(t), X-Y plots
    have_unpark = not isinstance(unpark_result, str)
    have_raster = not isinstance(raster_result, str)
    have_park = not isinstance(park_result, str)

    n_raster = len(raster_result[0]) if have_raster else 0

    if have_unpark:
        ux, uy = unpark_result
        n_unpark = len(ux)
        t_unpark = np.arange(-n_unpark, 0, dtype=np.float64)
        dpg.set_value("x_unpark", [t_unpark.tolist(), ux.tolist()])
        dpg.set_value("y_unpark", [t_unpark.tolist(), uy.tolist()])
        dpg.set_value("xy_unpark", [ux.tolist(), uy.tolist()])
    else:
        dpg.set_value("x_unpark", [[], []])
        dpg.set_value("y_unpark", [[], []])
        dpg.set_value("xy_unpark", [[], []])

    if have_raster:
        rx, ry = raster_result
        t_raster = np.arange(n_raster, dtype=np.float64)
        dpg.set_value("x_raster", [t_raster.tolist(), rx.tolist()])
        dpg.set_value("y_raster", [t_raster.tolist(), ry.tolist()])
        dpg.set_value("xy_raster", [rx.tolist(), ry.tolist()])
    else:
        dpg.set_value("x_raster", [[], []])
        dpg.set_value("y_raster", [[], []])
        dpg.set_value("xy_raster", [[], []])

    if have_park:
        px, py = park_result
        n_park = len(px)
        t_park = np.arange(n_raster, n_raster + n_park, dtype=np.float64)
        dpg.set_value("x_park", [t_park.tolist(), px.tolist()])
        dpg.set_value("y_park", [t_park.tolist(), py.tolist()])
        dpg.set_value("xy_park", [px.tolist(), py.tolist()])
    else:
        dpg.set_value("x_park", [[], []])
        dpg.set_value("y_park", [[], []])
        dpg.set_value("xy_park", [[], []])

    # Update clock plot
    if not isinstance(clock_result, str):
        line_clk, line_clk_flim, frame_clk_flim = clock_result
        n_clk = len(line_clk)
        t_clk = np.arange(n_clk, dtype=np.float64)
        dpg.set_value("clk_line", [t_clk.tolist(), line_clk.astype(np.float64).tolist()])
        dpg.set_value("clk_line_flim", [t_clk.tolist(), line_clk_flim.astype(np.float64).tolist()])
        dpg.set_value("clk_frame_flim", [t_clk.tolist(), frame_clk_flim.astype(np.float64).tolist()])
    else:
        dpg.set_value("clk_line", [[], []])
        dpg.set_value("clk_line_flim", [[], []])
        dpg.set_value("clk_frame_flim", [[], []])

    for ax in ("x_ax_x", "x_ax_y", "y_ax_x", "y_ax_y",
               "xy_ax_x", "xy_ax_y", "clk_ax_x", "clk_ax_y"):
        dpg.fit_axis_data(ax)

    if errors:
        dpg.set_value("status", "Errors: " + "; ".join(errors))
    else:
        parts = []
        if have_unpark:
            parts.append(f"unpark={len(unpark_result[0])}")
        if have_raster:
            parts.append(f"raster={n_raster}")
        if have_park:
            parts.append(f"park={len(park_result[0])}")
        dpg.set_value("status", f"Samples/ch: {', '.join(parts)}")


def on_size_toggle(_sender=None, _data=None):
    linked = dpg.get_value("size_equals_res")
    dpg.configure_item("width", enabled=not linked)
    dpg.configure_item("height", enabled=not linked)
    on_generate()


# Theme colors
COLOR_UNPARK = (255, 165, 0, 255)  # orange
COLOR_RASTER = (0, 120, 255, 255)  # blue
COLOR_PARK = (255, 60, 60, 255)    # red
COLOR_LINE_CLK = (0, 200, 100, 255)
COLOR_LINE_CLK_FLIM = (200, 100, 255, 255)
COLOR_FRAME_CLK_FLIM = (255, 200, 50, 255)


def make_theme(color):
    with dpg.theme() as t:
        with dpg.theme_component(dpg.mvLineSeries):
            dpg.add_theme_color(dpg.mvPlotCol_Line, color, category=dpg.mvThemeCat_Plots)
    return t


def main():
    dpg.create_context()
    dpg.create_viewport(title="Waveform Viewer", width=1400, height=900)

    theme_unpark = make_theme(COLOR_UNPARK)
    theme_raster = make_theme(COLOR_RASTER)
    theme_park = make_theme(COLOR_PARK)
    theme_line_clk = make_theme(COLOR_LINE_CLK)
    theme_line_clk_flim = make_theme(COLOR_LINE_CLK_FLIM)
    theme_frame_clk_flim = make_theme(COLOR_FRAME_CLK_FLIM)

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
                dpg.add_text("Park / Unpark")

                dpg.add_input_int(
                    label="X Park", tag="xpark",
                    default_value=0, callback=on_generate,
                )
                dpg.add_input_int(
                    label="Y Park", tag="ypark",
                    default_value=0, callback=on_generate,
                )
                dpg.add_input_float(
                    label="Prev X Park (V)", tag="prev_xpark_voltage",
                    default_value=0.0, format="%.6f", callback=on_generate,
                )
                dpg.add_input_float(
                    label="Prev Y Park (V)", tag="prev_ypark_voltage",
                    default_value=0.0, format="%.6f", callback=on_generate,
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
                with dpg.plot(label="Clock Signals", height=150, width=-1):
                    dpg.add_plot_axis(dpg.mvXAxis, label="Sample", tag="clk_ax_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="Value", tag="clk_ax_y"):
                        dpg.add_line_series([], [], label="lineClock", tag="clk_line")
                        dpg.add_line_series([], [], label="lineClockFLIM", tag="clk_line_flim")
                        dpg.add_line_series([], [], label="frameClockFLIM", tag="clk_frame_flim")
                    dpg.add_plot_legend()

                with dpg.plot(label="X(t)", height=200, width=-1):
                    dpg.add_plot_axis(dpg.mvXAxis, label="Sample", tag="x_ax_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="X (V)", tag="x_ax_y"):
                        dpg.add_line_series([], [], tag="x_unpark")
                        dpg.add_line_series([], [], tag="x_raster")
                        dpg.add_line_series([], [], tag="x_park")

                with dpg.plot(label="Y(t)", height=200, width=-1):
                    dpg.add_plot_axis(dpg.mvXAxis, label="Sample", tag="y_ax_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="Y (V)", tag="y_ax_y"):
                        dpg.add_line_series([], [], tag="y_unpark")
                        dpg.add_line_series([], [], tag="y_raster")
                        dpg.add_line_series([], [], tag="y_park")

                with dpg.plot(label="X-Y Trajectory", height=-1, width=-1, equal_aspects=True):
                    dpg.add_plot_axis(dpg.mvXAxis, label="X (V)", tag="xy_ax_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="Y (V)", tag="xy_ax_y"):
                        dpg.add_line_series([], [], tag="xy_unpark")
                        dpg.add_line_series([], [], tag="xy_raster")
                        dpg.add_line_series([], [], tag="xy_park")

    # Apply themes
    dpg.bind_item_theme("x_unpark", theme_unpark)
    dpg.bind_item_theme("x_raster", theme_raster)
    dpg.bind_item_theme("x_park", theme_park)
    dpg.bind_item_theme("y_unpark", theme_unpark)
    dpg.bind_item_theme("y_raster", theme_raster)
    dpg.bind_item_theme("y_park", theme_park)
    dpg.bind_item_theme("xy_unpark", theme_unpark)
    dpg.bind_item_theme("xy_raster", theme_raster)
    dpg.bind_item_theme("xy_park", theme_park)
    dpg.bind_item_theme("clk_line", theme_line_clk)
    dpg.bind_item_theme("clk_line_flim", theme_line_clk_flim)
    dpg.bind_item_theme("clk_frame_flim", theme_frame_clk_flim)

    dpg.set_primary_window("main_window", True)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    on_generate()

    dpg.start_dearpygui()
    dpg.destroy_context()


if __name__ == "__main__":
    main()

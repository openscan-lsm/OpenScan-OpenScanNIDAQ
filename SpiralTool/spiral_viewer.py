# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "dearpygui",
#     "numpy",
# ]
# ///

import colorsys
import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import dearpygui.dearpygui as dpg
import numpy as np


def find_spiral_tool() -> Path:
    script_dir = Path(__file__).resolve().parent
    candidates = [
        script_dir / "SpiralTool.exe",
        script_dir / "SpiralTool",
        script_dir.parent / "builddir" / "SpiralTool" / "SpiralTool.exe",
        script_dir.parent / "builddir" / "SpiralTool" / "SpiralTool",
    ]
    for p in candidates:
        if p.is_file():
            return p
    sys.exit(
        "Cannot find SpiralTool executable. Searched:\n"
        + "\n".join(f"  {c}" for c in candidates)
    )


TOOL_PATH = find_spiral_tool()

PERIPH_CONN_SAMPLES = 128

_active_items: list[int | str] = []


def generate_spiral(
    resolution: int,
    width: int,
    height: int,
    x_offset: int,
    y_offset: int,
    zoom: float,
    turn_spacing: float,
    turn_duration_ms: float,
    min_radius: float,
    x_center_offset: float,
    num_cycles: int,
    tform: tuple[float, float, float, float],
    tform_offset: tuple[float, float],
) -> tuple[np.ndarray, np.ndarray, dict[str, int]] | str:
    with tempfile.NamedTemporaryFile(suffix=".raw", delete=False) as f:
        tmp = Path(f.name)
    try:
        cmd = [
            str(TOOL_PATH),
            "-o", str(tmp),
            "--format", "raw",
            "--resolution", str(resolution),
            "--width", str(width),
            "--height", str(height),
            "--xoffset", str(x_offset),
            "--yoffset", str(y_offset),
            "--zoom", str(zoom),
            "--turn-spacing", str(turn_spacing),
            "--turn-duration", str(turn_duration_ms),
            "--min-radius", str(min_radius),
            "--x-center-offset", str(x_center_offset),
            "--num-cycles", str(num_cycles),
            "--tform", f"{tform[0]},{tform[1]},{tform[2]},{tform[3]}",
            "--tform-offset", f"{tform_offset[0]},{tform_offset[1]}",
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            return result.stderr.strip() or f"exited with code {result.returncode}"

        meta = {}
        for line in result.stdout.strip().splitlines():
            m = re.match(r"(\w+)=(\d+)", line)
            if m:
                meta[m.group(1)] = int(m.group(2))

        data = np.fromfile(tmp, dtype=np.float64)
        n = len(data) // 2
        if n == 0:
            return "no data produced"
        return data[:n], data[n:], meta
    finally:
        tmp.unlink(missing_ok=True)


def cycle_color(index: int, total: int) -> tuple[int, int, int, int]:
    hue = index / max(total, 1)
    r, g, b = colorsys.hsv_to_rgb(hue, 0.75, 0.9)
    return (int(r * 255), int(g * 255), int(b * 255), 255)


def conn_color(arm_color: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    r, g, b, _ = arm_color
    rf = r / 255.0
    gf = g / 255.0
    bf = b / 255.0
    mx = max(rf, gf, bf)
    mn = min(rf, gf, bf)
    # Approximate desaturation: lerp toward value
    v = mx * 0.75
    if mx == mn:
        return (int(v * 255), int(v * 255), int(v * 255), 180)
    d = mx - mn
    s_orig = d / mx if mx > 0 else 0
    s_new = s_orig * 0.35
    # Reconstruct from HSV with reduced S and V
    h_raw, _, _ = colorsys.rgb_to_hsv(rf, gf, bf)
    rn, gn, bn = colorsys.hsv_to_rgb(h_raw, s_new, v)
    return (int(rn * 255), int(gn * 255), int(bn * 255), 180)


COLOR_BOUNDARY = (100, 100, 100, 120)


def make_theme(color: tuple[int, int, int, int]) -> int:
    with dpg.theme() as t:
        with dpg.theme_component(dpg.mvLineSeries):
            dpg.add_theme_color(
                dpg.mvPlotCol_Line, color, category=dpg.mvThemeCat_Plots
            )
    return t


def cleanup_series():
    for item in _active_items:
        try:
            dpg.delete_item(item)
        except Exception:
            pass
    _active_items.clear()


def compute_boundary_circle(
    resolution: int,
    width: int,
    height: int,
    x_offset: int,
    y_offset: int,
    zoom: float,
    x_center_offset: float,
    tform: tuple[float, float, float, float],
    tform_offset: tuple[float, float],
) -> tuple[list[float], list[float]]:
    min_dim = min(width, height)
    radius = min_dim / (2.0 * zoom * resolution)
    cx = (-0.5 * resolution + x_offset + width / 2.0 + x_center_offset) / (
        zoom * resolution
    )
    cy = (-0.5 * resolution + y_offset + height / 2.0) / (zoom * resolution)

    a, b, c, d = tform
    tx, ty = tform_offset
    xs = []
    ys = []
    n_pts = 256
    for i in range(n_pts + 1):
        angle = 2.0 * math.pi * i / n_pts
        lx = cx + radius * math.cos(angle)
        ly = cy + radius * math.sin(angle)
        xs.append(a * lx + b * ly + tx)
        ys.append(c * lx + d * ly + ty)
    return xs, ys


def sync_width_height():
    resolution = dpg.get_value("resolution")
    linked = dpg.get_value("size_equals_res")
    dpg.configure_item(
        "width", max_value=resolution, max_clamped=True, enabled=not linked
    )
    dpg.configure_item(
        "height", max_value=resolution, max_clamped=True, enabled=not linked
    )
    if linked:
        dpg.set_value("width", resolution)
        dpg.set_value("height", resolution)
    else:
        if dpg.get_value("width") > resolution:
            dpg.set_value("width", resolution)
        if dpg.get_value("height") > resolution:
            dpg.set_value("height", resolution)


def on_generate(_sender=None, _data=None):
    sync_width_height()

    resolution = dpg.get_value("resolution")
    width = dpg.get_value("width")
    height = dpg.get_value("height")
    x_offset = dpg.get_value("x_offset")
    y_offset = dpg.get_value("y_offset")
    zoom = dpg.get_value("zoom")
    turn_spacing = dpg.get_value("turn_spacing")
    turn_duration_ms = dpg.get_value("turn_duration_ms")
    min_radius = dpg.get_value("min_radius")
    x_center_offset = dpg.get_value("x_center_offset")
    num_cycles = dpg.get_value("num_cycles")
    tform = (
        dpg.get_value("tform_a"),
        dpg.get_value("tform_b"),
        dpg.get_value("tform_c"),
        dpg.get_value("tform_d"),
    )
    tform_offset = (dpg.get_value("tform_tx"), dpg.get_value("tform_ty"))

    cleanup_series()

    result = generate_spiral(
        resolution,
        width,
        height,
        x_offset,
        y_offset,
        zoom,
        turn_spacing,
        turn_duration_ms,
        min_radius,
        x_center_offset,
        num_cycles,
        tform,
        tform_offset,
    )

    if isinstance(result, str):
        dpg.set_value("status", f"Error: {result}")
        for ax in ("x_ax_x", "x_ax_y", "y_ax_x", "y_ax_y", "xy_ax_x", "xy_ax_y"):
            dpg.fit_axis_data(ax)
        return

    x_data, y_data, meta = result
    total = len(x_data)
    samples_per_cycle = meta.get("samples_per_cycle", total)
    samples_per_arm = meta.get("samples_per_arm", 0)
    center_conn_samples = meta.get("center_conn_samples", 0)

    t_all = np.arange(total, dtype=np.float64)

    # Boundary circle in XY plot
    bx, by = compute_boundary_circle(
        resolution, width, height, x_offset, y_offset, zoom, x_center_offset,
        tform, tform_offset,
    )
    theme_boundary = make_theme(COLOR_BOUNDARY)
    _active_items.append(theme_boundary)
    s = dpg.add_line_series(bx, by, parent="xy_ax_y")
    dpg.bind_item_theme(s, theme_boundary)
    _active_items.append(s)

    for ci in range(num_cycles):
        base = ci * samples_per_cycle
        arm_col = cycle_color(ci, num_cycles)
        c_col = conn_color(arm_col)
        theme_arm = make_theme(arm_col)
        theme_conn = make_theme(c_col)
        _active_items.append(theme_arm)
        _active_items.append(theme_conn)

        # Phase boundaries
        out_start = base
        out_end = base + samples_per_arm
        pc_start = out_end
        pc_end = pc_start + PERIPH_CONN_SAMPLES
        in_start = pc_end
        in_end = in_start + samples_per_arm
        cc_start = in_end
        cc_end = cc_start + center_conn_samples

        phases = [
            ("out", out_start, out_end, theme_arm),
            ("pc", pc_start - 1, pc_end, theme_conn),
            ("in", in_start - 1, in_end, theme_arm),
            ("cc", cc_start - 1, cc_end, theme_conn),
        ]

        for name, ps, pe, theme in phases:
            ps = max(ps, 0)
            pe = min(pe, total)
            if pe <= ps:
                continue
            sl = slice(ps, pe)
            tx = t_all[sl].tolist()
            xv = x_data[sl].tolist()
            yv = y_data[sl].tolist()

            sx = dpg.add_line_series(tx, xv, parent="x_ax_y")
            dpg.bind_item_theme(sx, theme)
            _active_items.append(sx)

            sy = dpg.add_line_series(tx, yv, parent="y_ax_y")
            dpg.bind_item_theme(sy, theme)
            _active_items.append(sy)

            sxy = dpg.add_line_series(xv, yv, parent="xy_ax_y")
            dpg.bind_item_theme(sxy, theme)
            _active_items.append(sxy)

    for ax in ("x_ax_x", "x_ax_y", "y_ax_x", "y_ax_y", "xy_ax_x", "xy_ax_y"):
        dpg.fit_axis_data(ax)

    dpg.set_value(
        "status",
        f"Samples: {total}  |  per_cycle={samples_per_cycle}  "
        f"per_arm={samples_per_arm}  center_conn={center_conn_samples}",
    )


def on_size_toggle(_sender=None, _data=None):
    sync_width_height()
    on_generate()


def main():
    dpg.create_context()
    dpg.create_viewport(title="Spiral Viewer", width=1400, height=900)

    with dpg.window(tag="main_window"):
        with dpg.group(horizontal=True):
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

                dpg.add_separator()
                dpg.add_text("Spiral Parameters")

                dpg.add_input_float(
                    label="Turn Spacing (px)", tag="turn_spacing",
                    default_value=10.0, min_value=0.1, max_value=200.0,
                    min_clamped=True, max_clamped=True,
                    format="%.2f", callback=on_generate,
                )
                dpg.add_input_float(
                    label="Turn Duration (ms)", tag="turn_duration_ms",
                    default_value=5.0, min_value=0.1, max_value=1000.0,
                    min_clamped=True, max_clamped=True,
                    format="%.2f", callback=on_generate,
                )
                dpg.add_input_float(
                    label="Min Radius (px)", tag="min_radius",
                    default_value=1.0, min_value=0.1, max_value=10.0,
                    min_clamped=True, max_clamped=True,
                    format="%.2f", callback=on_generate,
                )
                dpg.add_input_float(
                    label="X Center Offset (px)", tag="x_center_offset",
                    default_value=0.0, min_value=-50.0, max_value=50.0,
                    min_clamped=True, max_clamped=True,
                    format="%.2f", callback=on_generate,
                )

                dpg.add_separator()
                dpg.add_text("Display")

                dpg.add_input_int(
                    label="Num Cycles", tag="num_cycles",
                    default_value=10, min_value=1, max_value=200,
                    min_clamped=True, max_clamped=True,
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

            with dpg.group():
                with dpg.plot(label="X(t)", height=250, width=-1):
                    dpg.add_plot_axis(dpg.mvXAxis, label="Sample", tag="x_ax_x")
                    dpg.add_plot_axis(dpg.mvYAxis, label="X (V)", tag="x_ax_y")

                with dpg.plot(label="Y(t)", height=250, width=-1):
                    dpg.add_plot_axis(dpg.mvXAxis, label="Sample", tag="y_ax_x")
                    dpg.add_plot_axis(dpg.mvYAxis, label="Y (V)", tag="y_ax_y")

                with dpg.plot(
                    label="X-Y Trajectory", height=-1, width=-1,
                    equal_aspects=True,
                ):
                    dpg.add_plot_axis(dpg.mvXAxis, label="X (V)", tag="xy_ax_x")
                    dpg.add_plot_axis(dpg.mvYAxis, label="Y (V)", tag="xy_ax_y")

    dpg.set_primary_window("main_window", True)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    on_generate()

    dpg.start_dearpygui()
    dpg.destroy_context()


if __name__ == "__main__":
    main()

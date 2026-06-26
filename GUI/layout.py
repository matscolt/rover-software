import imgui

from panels.camera_panel import draw_camera_panel
from panels.settings_panel import draw_settings_panel
from panels.telemetry_panel import draw_telemetry_panel
from panels.rover_icon_panel import draw_rover_icon_panel
from panels.E_stop import draw_estop_panel
from panels.shutdown_popup import draw_shutdown_popup
from config.settings_manager import draw_settings_popup

def draw_panel_by_name(name, state):
    if name == "camera":
        draw_camera_panel(state)
    elif name == "estop":
        draw_estop_panel(state)
    elif name == "settings":
        draw_settings_panel(state)
    elif name == "telemetry":
        draw_telemetry_panel(state)
    elif name == "rover_icon":
        draw_rover_icon_panel(state)


def draw_layout(state):
    width, height = imgui.get_io().display_size

    panels_cfg = state.config.layout.panels
    panel_visibility = state.config.panels

    cols = 2
    rows = max(p["row"] for p in panels_cfg.values()) + 1

    col_width = width / cols
    row_height = height / rows

    used_positions = set()

    for name, pos in panels_cfg.items():

        # skip disabled panels
        if not getattr(panel_visibility, f"show_{name}", True):
            continue

        row = max(0, pos["row"])
        col = max(0, min(cols - 1, pos["col"]))

        # prevent overlap
        while (row, col) in used_positions:
            row += 1

        used_positions.add((row, col))

        x = col * col_width
        y = row * row_height

        imgui.set_next_window_position(x, y)
        imgui.set_next_window_size(col_width, row_height)

        draw_panel_by_name(name, state)

    draw_shutdown_popup(state)

    draw_settings_popup(state)

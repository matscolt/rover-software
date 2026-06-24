import imgui

from panels.camera_panel import draw_camera_panel
from panels.settings_panel import draw_settings_panel
from panels.telemetry_panel import draw_telemetry_panel
from panels.rover_icon_panel import draw_rover_icon_panel
from panels.E_stop import draw_estop_panel
from panels.shutdown_popup import draw_shutdown_popup
from config.settings_manager import draw_settings_popup

def draw_layout(state):
    width, height = imgui.get_io().display_size
    cfg = state.config.layout
    panels = state.config.panels

    left_width = width * cfg.left_width_ratio
    right_width = width * cfg.right_width_ratio

    y0 = 0
    y1 = height * cfg.estop_height_ratio
    y2 = y1 + height * cfg.settings_height_ratio
    y3 = y2 + height * cfg.telemetry_height_ratio
    y4 = height

    if panels.show_camera:
        imgui.set_next_window_position(0, 0)
        imgui.set_next_window_size(left_width, height)
        draw_camera_panel(state)

    if panels.show_estop:
        imgui.set_next_window_position(left_width, y0)
        imgui.set_next_window_size(right_width, y1 - y0)
        draw_estop_panel(state)

    if panels.show_settings:
        imgui.set_next_window_position(left_width, y1)
        imgui.set_next_window_size(right_width, y2 - y1)
        draw_settings_panel(state)

    if panels.show_telemetry:
        imgui.set_next_window_position(left_width, y2)
        imgui.set_next_window_size(right_width, y3 - y2)
        draw_telemetry_panel(state)

    if panels.show_rover_icon:
        imgui.set_next_window_position(left_width, y3)
        imgui.set_next_window_size(right_width, y4 - y3)
        draw_rover_icon_panel(state)

    draw_shutdown_popup(state)

    draw_settings_popup(state)

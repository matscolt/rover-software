import imgui

from panels.camera_panel import draw_camera_panel
from GUI.panels.camera_selection_panel import draw_camera_selection_panel
from panels.telemetry_panel import draw_telemetry_panel
from panels.E_stop import draw_estop_panel

def draw_layout(state):
    width, height = imgui.get_io().display_size  

    left_width = width * 0.8
    right_width = width * 0.2

    top_height = height * 0.5
    bottom_height = height * 0.5

    # LEFT: Camera
    imgui.set_next_window_position(0, 0)
    imgui.set_next_window_size(left_width, height)
    draw_camera_panel(state)

    # RIGHT TOP: E-STOP
    imgui.set_next_window_position(left_width,0)
    imgui.set_next_window_size(right_width, top_height * 0.5)
    draw_estop_panel(state)

    # RIGHT MIDDLE: Controls
    imgui.set_next_window_position(left_width,  top_height * 0.5)
    imgui.set_next_window_size(right_width, top_height * 0.5)
    draw_camera_selection_panel(state)

    # RIGHT BOTTOM: Telemetry
    imgui.set_next_window_position(left_width, top_height)
    imgui.set_next_window_size(right_width, bottom_height)
    draw_telemetry_panel(state)
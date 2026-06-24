import imgui

from panels.camera_panel import draw_camera_panel
from panels.settings_panel import draw_settings_panel
from panels.telemetry_panel import draw_telemetry_panel
from panels.rover_icon_panel import draw_rover_icon_panel
from panels.E_stop import draw_estop_panel
from panels.shutdown_popup import draw_shutdown_popup

def draw_layout(state):
    width, height = imgui.get_io().display_size  

    left_width = width * 0.75
    right_width = width * 0.25

    first_height = height * 0
    second_height = height * 0.3
    third_height = height * 0.5
    fourth_height = height * 0.8


    # LEFT: Camera
    imgui.set_next_window_position(0, 0)
    imgui.set_next_window_size(left_width, height)
    draw_camera_panel(state)

    # RIGHT 1.: E-STOP
    imgui.set_next_window_position(left_width,first_height)
    imgui.set_next_window_size(right_width, second_height-first_height)
    draw_estop_panel(state)

    # RIGHT 2.: Controls
    imgui.set_next_window_position(left_width,  second_height)
    imgui.set_next_window_size(right_width, third_height-second_height)
    draw_settings_panel(state)

    # RIGHT 3.: icon
    imgui.set_next_window_position(left_width, third_height)
    imgui.set_next_window_size(right_width, fourth_height-third_height)
    draw_rover_icon_panel(state)

    # RIGHT 4.: Telemetry
    imgui.set_next_window_position(left_width, fourth_height)
    imgui.set_next_window_size(right_width, fourth_height-third_height)
    draw_telemetry_panel(state)

    draw_shutdown_popup(state)
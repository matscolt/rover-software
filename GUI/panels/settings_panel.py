import imgui
from panels.shutdown_popup import draw_shutdown_popup
from core.app_config import AppConfig


def draw_settings_panel(state):
    imgui.begin("Settings")
    if imgui.button("Shut Down GUI"):
        state.request_shutdown_popup = True
    imgui.same_line()

    if imgui.button("Open Settings"):
        state.request_settings_popup = True

    imgui.text("Camera Selection")

    if imgui.button("Camera front"):
        print("Camera front")
        state.requested_camera = 0

    imgui.same_line()

    if imgui.button("Camera back"):
        print("Camera back")
        state.requested_camera = 1

    imgui.same_line()

    if imgui.button("Camera manipulator"):
        print("Camera manipulator")
        state.requested_camera = 2

    imgui.end()
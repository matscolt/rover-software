# panels/estop_panel.py

import imgui

def draw_estop_panel(state):
    imgui.begin("Emergency Stop")

    # Make it BIG and RED
    imgui.push_style_color(imgui.COLOR_BUTTON, (1.0, 0.0, 0.0, 1.0))
    imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, (1.0, 0.2, 0.2, 1.0))
    imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, (0.8, 0.0, 0.0, 1.0))

    width, height = imgui.get_window_size()

    if imgui.button("STOP", width - 20, height - 40):
        state.emergency_pressed = not state.emergency_pressed

        if state.emergency_pressed:
            print("🚨 EMERGENCY STOP ACTIVATED")
            state.command = "STOP_ALL"
        else:
            print("✅ Emergency cleared")

    imgui.pop_style_color(3)

    # Status text
    if state.emergency_pressed:
        imgui.text_colored("STATUS: ACTIVE", 1, 0, 0)
    else:
        imgui.text("STATUS: INACTIVE")

    imgui.end()
# panels/estop_panel.py

import imgui

def draw_estop_panel(state):
    imgui.begin("Emergency Stop")

    # Select correct texture
    tex = state.em_pressed_tex if state.emergency_pressed else state.em_unpressed_tex

    # Get available space in window
    width, height = imgui.get_window_size()

    # Slight padding so it looks nice
    button_width = width - 20
    button_height = height - 40

    if tex is not None:
        if imgui.image_button(tex, button_width, button_height):
            state.emergency_pressed = not state.emergency_pressed

            if state.emergency_pressed:
                print("🚨 EMERGENCY STOP ACTIVATED")
                state.command = "STOP_ALL"
            else:
                print("✅ Emergency cleared")
    else:
        imgui.text("E-stop image not loaded")

    imgui.end()

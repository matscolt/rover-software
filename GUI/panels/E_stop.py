# panels/estop_panel.py

import imgui
def draw_estop_panel(state):
    imgui.begin("Emergency Stop")

    if state.emergency_pressed:
        tex = state.em_pressed_tex
        img_w = state.em_pressed_w
        img_h = state.em_pressed_h
    else:
        tex = state.em_unpressed_tex
        img_w = state.em_unpressed_w
        img_h = state.em_unpressed_h

    if tex is not None:
        # Get available space
        window_width, window_height = imgui.get_window_size()

        max_w = window_width - 20
        max_h = window_height - 40

        # ✅ Preserve aspect ratio
        scale = min(max_w / img_w, max_h / img_h)
        draw_w = img_w * scale
        draw_h = img_h * scale

        if imgui.image_button(tex, draw_w, draw_h):
            state.emergency_pressed = not state.emergency_pressed

            if state.emergency_pressed:
                print("EMERGENCY STOP ACTIVATED")
                state.command = "STOP_ALL"
            else:
                print("Emergency cleared")
    else:
        imgui.text("Missing texture")

    imgui.end()

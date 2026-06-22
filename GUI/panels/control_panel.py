import imgui

def draw_control_panel(state):
    imgui.begin("Controls")

    if imgui.button("Forward"):
        print("Forward")

    if imgui.button("Backward"):
        print("Backward")

    if imgui.button("Stop"):
        print("Stop")

    imgui.end()
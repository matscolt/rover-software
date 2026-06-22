import imgui

def draw_camera_selection_panel(state):
    imgui.begin("Camera Selection")

    if imgui.button("Camera 1"):
        print("Camera 1")

    if imgui.button("Camera 2"):
        print("Camera 2")

    if imgui.button("Camera 3"):
        print("Camera 3")

    imgui.end()
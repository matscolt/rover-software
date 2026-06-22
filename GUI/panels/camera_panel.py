import imgui

def draw_camera_panel(state):
    imgui.begin("Camera")

    if state.camera_texture:
        imgui.image(state.camera_texture, 800, 500)
    else:
        imgui.text("No camera feed")

    imgui.end()
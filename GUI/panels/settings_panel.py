import imgui

def draw_settings_panel(state):
    imgui.begin("Settings")

    if imgui.button("Camera front"):
        print("Camera front")

    imgui.same_line()

    if imgui.button("Camera back"):
        print("Camera back")

    imgui.same_line()

    if imgui.button("Camera manipulator"):
        print("Camera manipulator")

    if imgui.button("Shut Down GUI"):
        imgui.open_popup("Confirm Shutdown")

    if imgui.begin_popup_modal("Confirm Shutdown", True)[0]:
        imgui.text("Close the GUI?")
        imgui.spacing()

        if imgui.button("Yes", 100, 35):
            state.should_shutdown = True
            imgui.close_current_popup()

        imgui.same_line()

        if imgui.button("No", 100, 35):
            imgui.close_current_popup()

        imgui.end_popup()

    imgui.end()
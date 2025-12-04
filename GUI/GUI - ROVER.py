import imgui
from imgui.integrations.glfw import GlfwRenderer
import glfw
import OpenGL.GL as gl
import numpy as np
import cv2 

# ------------------ OpenCV Texture Loader ------------------
def load_texture_cv(path):
    # Read with OpenCV (BGR format)
    image = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise FileNotFoundError(f"Could not load texture: {path}")

    # Flip vertically (OpenGL coordinates)
    image = cv2.flip(image, 0)

    # Convert BGR/BGRA → RGBA
    if image.shape[2] == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGBA)
    elif image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGBA)

    height, width, _ = image.shape
    img_data = image.tobytes()

    # Generate OpenGL texture
    texture_id = gl.glGenTextures(1)
    gl.glBindTexture(gl.GL_TEXTURE_2D, texture_id)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
    gl.glTexImage2D(
        gl.GL_TEXTURE_2D, 0, gl.GL_RGBA,
        width, height, 0,
        gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, img_data
    )

    return texture_id, width, height

# ------------------ Main GUI ------------------
def main():
    # Initialize GLFW
    if not glfw.init():
        print("Could not initialize GLFW")
        return

    # Get primary monitor and its work area (excludes taskbar)
    monitor = glfw.get_primary_monitor()
    work_area = glfw.get_monitor_workarea(monitor)  # Returns (x, y, width, height)
    window_width, window_height = work_area[2], work_area[3]

    # Create a windowed window (not fullscreen)
    window = glfw.create_window(window_width, window_height, "Rover Gui", None, None)
    if not window:
        glfw.terminate()
        print("Could not create window")
        return

    # Maximize the window to simulate windowed fullscreen
    glfw.maximize_window(window)
    glfw.make_context_current(window)

    # Initialize ImGui  
    imgui.create_context()
    io = imgui.get_io()

    # Load fonts
    base_font_size = 16
    io.fonts.clear()
    base_font = io.fonts.add_font_default()
    big_font_size = base_font_size * 4
    big_font = io.fonts.add_font_from_file_ttf("C:/Windows/Fonts/arial.ttf", big_font_size)

    # Name for the operators
    operator_1 = "Rover Driver"
    operator_2 = "Manipulator Operator"
    operator_3 = "Observer"
    operator_4 = "Operator 4"

    # Build font atlas
    imgui.get_io().fonts.get_tex_data_as_rgba32()

    # Define minimum ImGui canvas size
    min_canvas_width, min_canvas_height = 800, 600

    renderer = GlfwRenderer(window)

    view_state = "menu"

    button_width = 200
    button_height = 60
    spacing = 20

    # ---- LOAD EMERGENCY STOP IMAGES USING OpenCV ----
    em_unpressed, uw, uh = load_texture_cv("C:/Users/victo/OneDrive/Desktop/Emergency_stop_notpressed.png")
    em_pressed, pw, ph = load_texture_cv("C:/Users/victo/OneDrive/Desktop/Emergency_stop_pressed.png")
    emergency_pressed = False

    # ------------------ Emergency Stop Button ------------------
    
    def draw_global_emergency_button():
        nonlocal emergency_pressed

        padding = 10
        imgui.set_next_window_position(window_width - button_width - padding, padding)
        imgui.set_next_window_size(button_width, button_height)

        imgui.begin("GlobalEmergencyStop", False,
                    imgui.WINDOW_NO_TITLE_BAR |
                    imgui.WINDOW_NO_RESIZE |
                    imgui.WINDOW_NO_MOVE |
                    imgui.WINDOW_NO_BACKGROUND |
                    imgui.WINDOW_NO_SCROLLBAR |
                    imgui.WINDOW_NO_BRING_TO_FRONT_ON_FOCUS)
        
        tex = em_pressed if emergency_pressed else em_unpressed
        if imgui.image_button(tex, uw, uh):
            emergency_pressed = not emergency_pressed
            if emergency_pressed == True:
                print("Emergency stop enabled")
            else:
                print("Emergency stop disabled")

        imgui.end()

    # ------------------ Operator Feed ------------------    
    def draw_operator_feed(operator_name):
        nonlocal view_state
        feed_text = f"{operator_name} Camera Feed"

        imgui.set_next_window_position(0, 0)
        imgui.set_next_window_size(window_width, window_height)

        imgui.begin(f"{operator_name}Feed", False,
                    imgui.WINDOW_NO_TITLE_BAR |
                    imgui.WINDOW_NO_RESIZE |
                    imgui.WINDOW_NO_MOVE |
                    imgui.WINDOW_NO_BACKGROUND |
                    imgui.WINDOW_NO_SCROLLBAR)

        # Back button
        imgui.set_cursor_pos_x(10)
        imgui.set_cursor_pos_y(10)
        if imgui.button("Back", width=button_width, height=button_height):
            view_state = "menu"

        imgui.push_font(big_font)

        text_width, text_height = imgui.calc_text_size(feed_text)
        center_x = (window_width - text_width) * 0.5
        center_y = window_height * 0.4

        imgui.set_cursor_pos_x(center_x)
        imgui.set_cursor_pos_y(center_y)
        imgui.text(feed_text)
        imgui.pop_font()
        imgui.end()

    # ------------------ Main Loop ------------------
    while not glfw.window_should_close(window):
        # Handle window resizing
        current_width, current_height = glfw.get_window_size(window)
        if (current_width, current_height) != (window_width, window_height):
            window_width, window_height = current_width, current_height
            gl.glViewport(0, 0, window_width, window_height)
        
        io.display_size = (
            max(window_width, min_canvas_width),
            max(window_height, min_canvas_height)
        )

        glfw.poll_events()
        renderer.process_inputs()

        imgui.new_frame()

        gl.glClearColor(0.1, 0.1, 0.1, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        # ------------------ Menu ------------------
        if view_state == "menu":
            total_buttons = 3
            total_height = button_height * total_buttons + spacing * (total_buttons - 1)
            start_y = (window_height - total_height) * 0.5
            center_x = (window_width - button_width) * 0.5

            imgui.set_next_window_position(center_x, start_y)
            imgui.set_next_window_size(button_width, total_height)
            
            imgui.begin("MenuButtons", False,
                        imgui.WINDOW_NO_TITLE_BAR |
                        imgui.WINDOW_NO_RESIZE |
                        imgui.WINDOW_NO_MOVE |
                        imgui.WINDOW_NO_BACKGROUND |
                        imgui.WINDOW_NO_SCROLLBAR)
            imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 0.0)

            if imgui.button("Operator Selection", width=button_width, height=button_height):
                view_state = "Operator_selection"

            imgui.spacing()
            if imgui.button("Settings", width=button_width, height=button_height):
                view_state = "Settings"

            imgui.spacing()
            if imgui.button("Exit", width=button_width, height=button_height):
                print("Exiting application...")
                view_state = "Exit"

            imgui.pop_style_var()
            imgui.end()       

        # ------------------ Other Views ------------------
        elif view_state == "Exit":
            glfw.set_window_should_close(window, True)

        elif view_state == "Settings":
            total_buttons = 1
            total_height = button_height * total_buttons + spacing * (total_buttons - 1)
            start_y = (window_height - total_height) * 0.5
            center_x = (window_width - button_width) * 0.5

            imgui.set_next_window_position(center_x, start_y)
            imgui.set_next_window_size(button_width, total_height)

            imgui.begin("Settings", False,
                        imgui.WINDOW_NO_TITLE_BAR |
                        imgui.WINDOW_NO_RESIZE |
                        imgui.WINDOW_NO_MOVE |
                        imgui.WINDOW_NO_BACKGROUND |
                        imgui.WINDOW_NO_SCROLLBAR)
            imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 0.0)

            if imgui.button("Back", width=button_width, height=button_height):
                view_state = "menu"

            imgui.pop_style_var()
            imgui.end()

        elif view_state == "Operator_selection":
            total_buttons = 5
            total_height = button_height * total_buttons + spacing * (total_buttons - 1)
            start_y = (window_height - total_height) * 0.5
            center_x = (window_width - button_width) * 0.5

            imgui.set_next_window_position(center_x, start_y)
            imgui.set_next_window_size(button_width, total_height)

            imgui.begin("OperatorButtons", False,
                        imgui.WINDOW_NO_TITLE_BAR |
                        imgui.WINDOW_NO_RESIZE |
                        imgui.WINDOW_NO_MOVE |
                        imgui.WINDOW_NO_BACKGROUND |
                        imgui.WINDOW_NO_SCROLLBAR)

            imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 0.0)

            if imgui.button(operator_1, width=button_width, height=button_height):
                view_state = operator_1

            imgui.spacing()
            if imgui.button(operator_2, width=button_width, height=button_height):
                view_state = operator_2

            imgui.spacing()
            if imgui.button(operator_3, width=button_width, height=button_height):
                view_state = operator_3

            imgui.spacing()
            if imgui.button(operator_4, width=button_width, height=button_height):
                view_state = operator_4

            imgui.spacing()
            if imgui.button("Back", width=button_width, height=button_height):
                view_state = "menu"

            imgui.pop_style_var()
            imgui.end()

        elif view_state == operator_1:
            draw_operator_feed(operator_1)
        elif view_state == operator_2:
            draw_operator_feed(operator_2)
        elif view_state == operator_3:
            draw_operator_feed(operator_3)
        elif view_state == operator_4:
            draw_operator_feed(operator_4)

        # ------------------ Draw Emergency Button ------------------
        draw_global_emergency_button()

        # Render ImGui
        imgui.render()
        renderer.render(imgui.get_draw_data())

        glfw.swap_buffers(window)
    #Cleanup
    renderer.shutdown()
    glfw.terminate()

if __name__ == "__main__":
    main()

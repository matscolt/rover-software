import imgui
from imgui.integrations.glfw import GlfwRenderer
import glfw
import OpenGL.GL as gl

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
    # Default font size for normal UI text
    base_font_size = 16
    # Load the default font at base size
    io.fonts.clear()
    base_font = io.fonts.add_font_default()
    # Load a bigger font for the large Operator Feed text (4x size)
    big_font_size = base_font_size * 4
    big_font = io.fonts.add_font_from_file_ttf("C:/Windows/Fonts/arial.ttf", big_font_size)
    # You can change the path above to another ttf font file on your system

    # Build font atlas
    imgui.get_io().fonts.get_tex_data_as_rgba32()

    # Define minimum ImGui canvas size
    min_canvas_width, min_canvas_height = 800, 600

    renderer = GlfwRenderer(window)

    view_state = "menu"

    button_width = 200
    button_height = 60
    spacing = 20

    def draw_operator_feed(operator_number):
        feed_text = f"Operator {operator_number} Camera Feed"

        imgui.set_next_window_position(0, 0)
        imgui.set_next_window_size(window_width, window_height)

        imgui.begin(f"Operator{operator_number}Feed", False,
                    imgui.WINDOW_NO_TITLE_BAR |
                    imgui.WINDOW_NO_RESIZE |
                    imgui.WINDOW_NO_MOVE |
                    imgui.WINDOW_NO_BACKGROUND |
                    imgui.WINDOW_NO_SCROLLBAR)

        # Back button top-left
        imgui.set_cursor_pos_x(10)
        imgui.set_cursor_pos_y(10)
        if imgui.button("Back", width=button_width, height=button_height):
            nonlocal view_state
            view_state = "operator_select"

        # Use big font for the camera feed text
        imgui.push_font(big_font)

        # Center text
        text_width, text_height = imgui.calc_text_size(feed_text)
        center_x = (window_width - text_width) * 0.5
        center_y = window_height * 0.4

        imgui.set_cursor_pos_x(center_x)
        imgui.set_cursor_pos_y(center_y)
        imgui.text(feed_text)

        imgui.pop_font()

        imgui.end()

    while not glfw.window_should_close(window):
        # Handle window resizing
        current_width, current_height = glfw.get_window_size(window)
        if (current_width, current_height) != (window_width, window_height):
            window_width, window_height = current_width, current_height
            gl.glViewport(0, 0, window_width, window_height)

        io.display_size = (max(window_width, min_canvas_width), max(window_height, min_canvas_height))

        # Process events
        glfw.poll_events()
        renderer.process_inputs()

        # Start new ImGui frame
        imgui.new_frame()

        # Clear the screen
        gl.glClearColor(0.1, 0.1, 0.1, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        if view_state == "menu":
            button_label = "Menu"
            center_x = (window_width - button_width) * 0.5
            center_y = (window_height - button_height) * 0.5

            imgui.set_next_window_position(center_x, center_y)
            imgui.set_next_window_size(button_width, button_height)

            imgui.begin("MenuButton", False,
                        imgui.WINDOW_NO_TITLE_BAR |
                        imgui.WINDOW_NO_RESIZE |
                        imgui.WINDOW_NO_MOVE |
                        imgui.WINDOW_NO_BACKGROUND |
                        imgui.WINDOW_NO_SCROLLBAR)

            imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 0.0)

            if imgui.button(button_label, width=button_width, height=button_height):
                view_state = "operator_select"

            imgui.pop_style_var()
            imgui.end()            

        elif view_state == "operator_select":
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

            if imgui.button("Operator 1", width=button_width, height=button_height):
                view_state = "operator_1"

            imgui.spacing()
            if imgui.button("Operator 2", width=button_width, height=button_height):
                view_state = "operator_2"

            imgui.spacing()
            if imgui.button("Operator 3", width=button_width, height=button_height):
                view_state = "operator_3"

            imgui.spacing()
            if imgui.button("Operator 4", width=button_width, height=button_height):
                view_state = "operator_4"

            imgui.spacing()
            if imgui.button("Back", width=button_width, height=button_height):
                view_state = "menu"

            imgui.pop_style_var()
            imgui.end()

        elif view_state == "operator_1":
            draw_operator_feed(1)
        elif view_state == "operator_2":
            draw_operator_feed(2)
        elif view_state == "operator_3":
            draw_operator_feed(3)
        elif view_state == "operator_4":
            draw_operator_feed(4)

        # Render ImGui
        imgui.render()
        renderer.render(imgui.get_draw_data())

        # Swap buffers
        glfw.swap_buffers(window)

    # Cleanup
    renderer.shutdown()
    glfw.terminate()

if __name__ == "__main__":
    main()
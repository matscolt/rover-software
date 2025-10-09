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
   
    # Set fixed font size (constant regardless of window size)
    io.font_global_scale = 1.3  # Fixed scale factor
    io.fonts.get_tex_data_as_rgba32()
 
    # Define minimum ImGui canvas size (content will clip if window is smaller)
    min_canvas_width, min_canvas_height = 800, 600
 
    renderer = GlfwRenderer(window)
 
    while not glfw.window_should_close(window):
        # Handle window resizing
        current_width, current_height = glfw.get_window_size(window)
        if (current_width, current_height) != (window_width, window_height):
            window_width, window_height = current_width, current_height
            gl.glViewport(0, 0, window_width, window_height)
 
        # Set ImGui display size to at least the minimum canvas size
        io.display_size = (max(window_width, min_canvas_width), max(window_height, min_canvas_height))
 
        # Process events
        glfw.poll_events()
        renderer.process_inputs()
 
        # Start new ImGui frame
        imgui.new_frame()
 
        # Draw directly to the main window (no separate ImGui window)
        imgui.set_next_window_position(0, 0)
        imgui.set_next_window_size(min_canvas_width, min_canvas_height)
        imgui.begin("Main", False, flags=imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_BACKGROUND)
        imgui.text("Hello world!")
        imgui.text(f"Window size: {window_width}x{window_height}")
        imgui.text(f"Canvas size: {min_canvas_width}x{min_canvas_height}")
        imgui.text("This text will be clipped if the window is too small!")
        if imgui.button("Click Me"):
            print("Button Clicked!")
        imgui.end()
 
        # Clear the screen
        gl.glClearColor(0.1, 0.1, 0.1, 1.0)  # Dark background
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        button_label = "Menu"
        button_width = 200
        button_height = 60

        center_x = (window_width - button_width) * 0.5
        center_y = (window_height - button_height) * 0.5

        imgui.set_next_window_position(center_x, center_y)
        imgui.set_next_window_size(button_width, button_height)

        imgui.begin("CenteredButton", False,
                    imgui.WINDOW_NO_TITLE_BAR |
                    imgui.WINDOW_NO_RESIZE |
                    imgui.WINDOW_NO_MOVE |
                    imgui.WINDOW_NO_BACKGROUND |
                    imgui.WINDOW_NO_SCROLLBAR)

        imgui.push_style_var(imgui.STYLE_FRAME_ROUNDING, 0.0)
        imgui.push_style_color(imgui.COLOR_BUTTON, *imgui.get_style().colors[imgui.COLOR_BUTTON])

        if imgui.button(button_label, width=button_width, height=button_height):
            print("Menu button pressed!")

        imgui.pop_style_color()
        imgui.pop_style_var()
        imgui.end()
        
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

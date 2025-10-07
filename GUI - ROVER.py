import imgui
from imgui.integrations.glfw import GlfwRenderer
import glfw


def main():
    if not glfw.init():
        
        raise Exception("Could not initialize GLFW")

    window = glfw.create_window(600, 400, "Project 0", None, None)
    if not window:
        glfw.terminate()
        raise Exception("Could not create GLFW window")
    glfw.make_context_current(window)

    imgui.create_context()
    impl = GlfwRenderer(window)

    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()
        imgui.new_frame()

        # Get window size
        window_width, window_height = glfw.get_window_size(window)

        # Define the text you want to display
        text = "Hello from imgui!"

        # Calculate text size for positioning
        text_size = imgui.calc_text_size(text)

        # Calculate centered position
        pos_x = (window_width - text_size.x) / 2
        pos_y = (window_height - text_size.y) / 2

        # Set window flags to remove decorations and make it not movable/resizable
        window_flags = (
            imgui.WINDOW_NO_TITLE_BAR
            | imgui.WINDOW_NO_RESIZE
            | imgui.WINDOW_NO_MOVE
            | imgui.WINDOW_NO_SCROLLBAR
            | imgui.WINDOW_NO_COLLAPSE
            | imgui.WINDOW_ALWAYS_AUTO_RESIZE
        )

        # Position the window and begin it with no decorations
        imgui.set_next_window_pos(pos_x, pos_y)
        imgui.begin("TextWindow", False, flags=window_flags)

        # Show the text
        imgui.text(text)

        imgui.end()

        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

    impl.shutdown()
    glfw.terminate()

import imgui
from imgui.integrations.glfw import GlfwRenderer
import glfw
import OpenGL.GL as gl

# Lav funktioner
 
 
 
def main2():
    # Initialize GLFW
    if not glfw.init():
        print("Could not initialize GLFW")
        return
 
    # Get primary monitor and its work area (excludes taskbar)
    monitor = glfw.get_primary_monitor()
    work_area = glfw.get_monitor_workarea(monitor)  # Returns (x, y, width, height)
    window_width, window_height = work_area[2], work_area[3]
 
    # Create a windowed window (not fullscreen)
    window = glfw.create_window(window_width, window_height, "Minesweeper", None, None)
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
            imgui.text("Button Clicked!")
        imgui.end()
 
        # Clear the screen
        gl.glClearColor(0.1, 0.1, 0.1, 1.0)  # Dark background
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
 
        # Render ImGui
        imgui.render()
        renderer.render(imgui.get_draw_data())
 
        # Swap buffers
        glfw.swap_buffers(window)
 
    # Cleanup
    renderer.shutdown()
    glfw.terminate()
 
if __name__ == "__main__":
    main2()

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

if __name__ == "__main__":
    main()

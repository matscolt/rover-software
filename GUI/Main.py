import glfw
import cv2
import OpenGL.GL as gl
import imgui
from imgui.integrations.glfw import GlfwRenderer
from pathlib import Path

from layout import draw_layout
from core.rover_state import RoverState

BASE_DIR = Path(__file__).resolve().parent

def load_texture_cv(path):
    # Load image with OpenCV (BGR or BGRA)
    image = cv2.imread(path, cv2.IMREAD_UNCHANGED)

    if image is None:
        raise FileNotFoundError(f"Could not load texture: {path}")

    # Flip vertically (OpenGL expects origin bottom-left)
    image = cv2.flip(image, 0)

    height, width = image.shape[:2]

    # Detect number of channels
    if image.shape[2] == 3:
        format = gl.GL_BGR
        internal_format = gl.GL_RGB
    elif image.shape[2] == 4:
        format = gl.GL_BGRA
        internal_format = gl.GL_RGBA
    else:
        raise ValueError("Unsupported image format")

    # Generate texture ID
    texture_id = gl.glGenTextures(1)
    gl.glBindTexture(gl.GL_TEXTURE_2D, texture_id)

    # Upload texture to GPU
    gl.glTexImage2D(
        gl.GL_TEXTURE_2D,
        0,
        internal_format,
        width,
        height,
        0,
        format,
        gl.GL_UNSIGNED_BYTE,
        image
    )

    # Set texture parameters
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)

    # Prevent stretching artifacts
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_EDGE)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_EDGE)

    gl.glBindTexture(gl.GL_TEXTURE_2D, 0)

    return texture_id

def main():
    if not glfw.init():
        return

    window = glfw.create_window(1280, 800, "Rover GUI", None, None)
    glfw.make_context_current(window)

    imgui.create_context()
    impl = GlfwRenderer(window)

    state = RoverState()

    # ✅ Load textures ONCE
    state.em_pressed_tex = load_texture_cv(
        str(BASE_DIR / "assets" / "estop_pressed.png")
    )

    state.em_unpressed_tex = load_texture_cv(
        str(BASE_DIR / "assets" / "estop_unpressed.png")
    )

     


    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()

        imgui.new_frame()

        # Draw entire GUI
        draw_layout(state)

        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        imgui.render()
        impl.render(imgui.get_draw_data())

        glfw.swap_buffers(window)

    impl.shutdown()
    glfw.terminate()


if __name__ == "__main__":
    main()
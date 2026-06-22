from pathlib import Path
import cv2
import glfw
import imgui
import OpenGL.GL as gl
from imgui.integrations.glfw import GlfwRenderer

from rendering.textures import (
    load_texture_cv,
    create_empty_texture,
    update_texture_from_frame,
    delete_texture,
)
from layout import draw_layout
from core.rover_state import RoverState

BASE_DIR = Path(__file__).resolve().parent


def main():
    if not glfw.init():
        print("Could not initialize GLFW")
        return

    window = glfw.create_window(1800, 900, "Rover GUI", None, None)
    glfw.make_context_current(window)

    imgui.create_context()
    impl = GlfwRenderer(window)

    state = RoverState()

    # Load E-stop textures once
    state.em_pressed_tex, state.em_pressed_w, state.em_pressed_h = load_texture_cv(
        str(BASE_DIR / "assets" / "estop_pressed.png")
    )
    state.em_unpressed_tex, state.em_unpressed_w, state.em_unpressed_h = load_texture_cv(
        str(BASE_DIR / "assets" / "estop_unpressed.png")
    )

    # Open camera
    cap = cv2.VideoCapture(0)  # default webcam
    if not cap.isOpened():
        print("Could not open camera")
        cap = None
    else:
        ret, frame = cap.read()
        if ret:
            h, w = frame.shape[:2]
            state.camera_width = w
            state.camera_height = h
            state.camera_channels = 3
            state.camera_texture = create_empty_texture(w, h, channels=3)
            update_texture_from_frame(state.camera_texture, frame)
        else:
            print("Could not read first camera frame")
            cap.release()
            cap = None

    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()
        imgui.new_frame()

        # Update camera texture every frame
        if cap is not None and state.camera_texture is not None:
            ret, frame = cap.read()
            if ret:
                w, h, ch = update_texture_from_frame(state.camera_texture, frame)
                state.camera_width = w
                state.camera_height = h
                state.camera_channels = ch

        draw_layout(state)

        if state.should_shutdown:
            glfw.set_window_should_close(window, True)
            continue

        gl.glClearColor(0.1, 0.1, 0.1, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

    # Cleanup
    if cap is not None:
        cap.release()

    delete_texture(state.camera_texture)
    delete_texture(state.em_pressed_tex)
    delete_texture(state.em_unpressed_tex)

    impl.shutdown()
    glfw.terminate()


if __name__ == "__main__":
    main()
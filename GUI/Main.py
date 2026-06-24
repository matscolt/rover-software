from pathlib import Path
import cv2
import glfw
import imgui
import OpenGL.GL as gl
from imgui.integrations.glfw import GlfwRenderer
from config.settings_manager import load_config, save_config

from rendering.textures import (
    load_texture_cv,
    create_empty_texture,
    update_texture_from_frame,
    delete_texture,
)
from layout import draw_layout
from core.rover_state import RoverState
from core.input_handler import handle_keybinds

BASE_DIR = Path(__file__).resolve().parent

def clear_camera_state(state, camera_index=None, status="Camera not found"):
    # Remove old texture so the panel doesn't show a frozen frame
    if state.camera_texture is not None:
        delete_texture(state.camera_texture)
        state.camera_texture = None

    state.camera_width = 0
    state.camera_height = 0
    state.camera_channels = 3
    state.camera_status = status

    # Important:
    # mark this camera index as the current attempted one,
    # even though it failed, so the code does not retry every frame
    if camera_index is not None:
        state.active_camera = camera_index


def open_camera(camera_index, state):
    # On Windows, CAP_DSHOW often avoids noisy backend probing
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)

    if not cap.isOpened():
        clear_camera_state(
            state,
            camera_index=camera_index,
            status=f"Camera {camera_index} not found"
        )
        return None

    ret, frame = cap.read()
    if not ret or frame is None:
        cap.release()
        clear_camera_state(
            state,
            camera_index=camera_index,
            status=f"Camera {camera_index} not found"
        )
        return None

    h, w = frame.shape[:2]
    channels = 1 if len(frame.shape) == 2 else frame.shape[2]

    if channels not in (3, 4):
        cap.release()
        clear_camera_state(
            state,
            camera_index=camera_index,
            status=f"Unsupported format from camera {camera_index}"
        )
        return None

    # Recreate camera texture cleanly
    if state.camera_texture is not None:
        delete_texture(state.camera_texture)

    state.camera_texture = create_empty_texture(w, h, channels=channels)
    update_texture_from_frame(state.camera_texture, frame)

    state.camera_width = w
    state.camera_height = h
    state.camera_channels = channels
    state.active_camera = camera_index
    state.camera_status = f"Camera {camera_index} active"

    return cap


def main():
    config = load_config()
    
    if not glfw.init():
        print("Could not initialize GLFW")
        return

    window = glfw.create_window(
        config.window.width,
        config.window.height,
        config.window.title,
        None,
        None
    )
    if not window:
        print("Could not create GLFW window")
        glfw.terminate()
        return

    glfw.make_context_current(window)
    glfw.swap_interval(1)

    imgui.create_context()
    impl = GlfwRenderer(window)

    state = RoverState()
    state.config = config
    state.requested_camera = config.camera.default_camera

    # Load E-stop textures once
    state.em_pressed_tex, state.em_pressed_w, state.em_pressed_h = load_texture_cv(
        str(BASE_DIR / "assets" / "estop_pressed.png")
    )
    state.em_unpressed_tex, state.em_unpressed_w, state.em_unpressed_h = load_texture_cv(
        str(BASE_DIR / "assets" / "estop_unpressed.png")
    )

    # Rover icon textures
    state.front_cam_icon_tex, state.front_cam_icon_w, state.front_cam_icon_h = load_texture_cv(
        str(BASE_DIR / "assets" / "front_cam.png")
    )

    state.back_cam_icon_tex, state.back_cam_icon_w, state.back_cam_icon_h = load_texture_cv(
        str(BASE_DIR / "assets" / "back_cam.png")
    )

    state.manipulator_cam_icon_tex, state.manipulator_cam_icon_w, state.manipulator_cam_icon_h = load_texture_cv(
        str(BASE_DIR / "assets" / "manipulator_cam.png")
    )


    # Open initial camera
    cap = open_camera(state.requested_camera, state)
    if cap is None:
        print(state.camera_status)

    while not glfw.window_should_close(window):
        glfw.poll_events()
        impl.process_inputs()
        imgui.new_frame()

        #keybinds
        handle_keybinds(window, state)

        # Camera switching
        if state.requested_camera != state.active_camera:
            if cap is not None:
                cap.release()
            cap = open_camera(state.requested_camera, state)

        # Update active camera feed
        if cap is not None and state.camera_texture is not None:
            ret, frame = cap.read()
            if ret:
                w, h, ch = update_texture_from_frame(state.camera_texture, frame)
                state.camera_width = w
                state.camera_height = h
                state.camera_channels = ch
            else:
                state.camera_status = f"Lost feed from camera {state.active_camera}"

        draw_layout(state)

        if state.should_shutdown:
            glfw.set_window_should_close(window, True)

        gl.glClearColor(0.1, 0.1, 0.1, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)

        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)

    if cap is not None:
        cap.release()

    delete_texture(state.camera_texture)
    delete_texture(state.em_pressed_tex)
    delete_texture(state.em_unpressed_tex)

    impl.shutdown()
    glfw.terminate()


if __name__ == "__main__":
    main()
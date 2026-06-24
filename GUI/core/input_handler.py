import glfw

def is_key_pressed_once(window, key, state):
    current = glfw.get_key(window, key) == glfw.PRESS
    previous = state.prev_keys.get(key, False)
    state.prev_keys[key] = current
    return current and not previous


def handle_keybinds(window, state):
    # If shutdown popup is open, only allow popup-related keys
    if state.shutdown_popup_open:
        if is_key_pressed_once(window, glfw.KEY_1, state):
            state.shutdown_popup_choice = "yes"

        if is_key_pressed_once(window, glfw.KEY_2, state):
            state.shutdown_popup_choice = "no"

        # ESC while popup is open = cancel
        if is_key_pressed_once(window, glfw.KEY_ESCAPE, state):
            state.shutdown_popup_choice = "no"

        return

    # Camera selection
    if is_key_pressed_once(window, glfw.KEY_1, state):
        state.requested_camera = 0

    if is_key_pressed_once(window, glfw.KEY_2, state):
        state.requested_camera = 1

    if is_key_pressed_once(window, glfw.KEY_3, state):
        state.requested_camera = 2

    # Emergency stop
    if is_key_pressed_once(window, glfw.KEY_SPACE, state):
        state.emergency_pressed = True
        state.command = "STOP_ALL"

    # Toggle E-stop
    if is_key_pressed_once(window, glfw.KEY_E, state):
        state.emergency_pressed = not state.emergency_pressed

    # Shut down GUI popup
    if is_key_pressed_once(window, glfw.KEY_ESCAPE, state):
        state.request_shutdown_popup = True
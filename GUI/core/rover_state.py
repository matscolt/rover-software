
class RoverState:
    def __init__(self):
        self.speed = 0.0
        self.battery = 100
        self.temperature = 25.0

        # E-stop
        self.emergency_pressed = False
        self.em_pressed_tex = None
        self.em_pressed_w = 0
        self.em_pressed_h = 0
        self.em_unpressed_tex = None
        self.em_unpressed_w = 0
        self.em_unpressed_h = 0

        # Camera display
        self.camera_texture = None
        self.camera_width = 0
        self.camera_height = 0
        self.camera_channels = 3

        # Camera switching
        self.active_camera = None
        self.requested_camera = 0
        self.camera_status = "Not initialized"
        self.available_cameras = [0, 1, 2]

        # Command / app state
        self.command = None
        self.should_shutdown = False

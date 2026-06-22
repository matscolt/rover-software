class RoverState:
    def __init__(self):
        self.speed = 0.0
        self.battery = 100
        self.temperature = 25.0
        self.camera_texture = None
        self.e_stop = False

        
        # Emergency stop
        self.emergency_pressed = False
        self.em_pressed_tex = None
        self.em_unpressed_tex = None

        self.command = None

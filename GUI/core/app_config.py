from dataclasses import dataclass, field

@dataclass
class WindowConfig:
    width: int = 1800
    height: int = 900
    title: str = "Rover GUI"
    fullscreen: bool = False

@dataclass
class LayoutConfig:
    left_width_ratio: float = 0.75
    right_width_ratio: float = 0.25
    estop_height_ratio: float = 0.30
    settings_height_ratio: float = 0.20
    telemetry_height_ratio: float = 0.30
    rover_icon_height_ratio: float = 0.20

@dataclass
class PanelsConfig:
    show_camera: bool = True
    show_estop: bool = True
    show_settings: bool = True
    show_telemetry: bool = True
    show_rover_icon: bool = True

@dataclass
class CameraConfig:
    default_camera: int = 0

@dataclass
class AppConfig:
    window: WindowConfig = field(default_factory=WindowConfig)
    layout: LayoutConfig = field(default_factory=LayoutConfig)
    panels: PanelsConfig = field(default_factory=PanelsConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
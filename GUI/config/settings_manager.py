import copy
import json
from pathlib import Path
import imgui
from core.app_config import (
    AppConfig,
    WindowConfig,
    LayoutConfig,
    PanelsConfig,
    CameraConfig,
    KeybindsConfig,
)

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "config.json"


def load_config() -> AppConfig:
    if not CONFIG_PATH.exists():
        return AppConfig()

    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        raw = json.load(f)

    return AppConfig(
        window=WindowConfig(**raw.get("window", {})),
        layout=LayoutConfig(**raw.get("layout", {})),
        panels=PanelsConfig(**raw.get("panels", {})),
        camera=CameraConfig(**raw.get("camera", {})),
        keybinds=KeybindsConfig(**raw.get("keybinds", {})),
    )


def save_config(config: AppConfig) -> None:
    data = {
        "window": vars(config.window),
        "layout": vars(config.layout),
        "panels": vars(config.panels),
        "camera": vars(config.camera),
        "keybinds": vars(config.keybinds),
    }

    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)

    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)


def reset_config_defaults() -> AppConfig:
    return AppConfig()


def ensure_edit_config(state):
    if state.edit_config is None:
        state.edit_config = copy.deepcopy(state.config)


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def begin_settings_popup(state):
    if state.request_settings_popup:
        ensure_edit_config(state)
        imgui.open_popup("Settings")
        state.request_settings_popup = False


def draw_settings_popup(state):
    if state.request_settings_popup:
        state.edit_config = copy.deepcopy(state.config)
        imgui.open_popup("Settings menu")
        state.request_settings_popup = False

    imgui.set_next_window_size(900, 700, condition=imgui.ONCE)

    opened, _ = imgui.begin_popup_modal("Settings menu", True)
    state.settings_popup_open = opened

    if not opened:
        return

    cfg = state.edit_config

    imgui.text("Window")
    changed, cfg.window.width = imgui.input_int("Window Width", cfg.window.width)
    changed, cfg.window.height = imgui.input_int("Window Height", cfg.window.height)
    changed, cfg.window.fullscreen = imgui.checkbox("Start Fullscreen", cfg.window.fullscreen)

    imgui.separator()
    imgui.text("Layout Ratios (0.0 to 1.0)")

    changed, cfg.layout.left_width_ratio = imgui.slider_float(
        "Left Width Ratio", cfg.layout.left_width_ratio, 0.0, 1.0
    )
    cfg.layout.left_width_ratio = clamp01(cfg.layout.left_width_ratio)
    cfg.layout.right_width_ratio = 1.0 - cfg.layout.left_width_ratio

    changed, cfg.layout.estop_height_ratio = imgui.slider_float(
        "E-stop Height Ratio", cfg.layout.estop_height_ratio, 0.0, 1.0
    )
    cfg.layout.estop_height_ratio = clamp01(cfg.layout.estop_height_ratio)

    changed, cfg.layout.settings_height_ratio = imgui.slider_float(
        "Settings Height Ratio", cfg.layout.settings_height_ratio, 0.0, 1.0
    )
    cfg.layout.settings_height_ratio = clamp01(cfg.layout.settings_height_ratio)

    changed, cfg.layout.telemetry_height_ratio = imgui.slider_float(
        "Telemetry Height Ratio", cfg.layout.telemetry_height_ratio, 0.0, 1.0
    )
    cfg.layout.telemetry_height_ratio = clamp01(cfg.layout.telemetry_height_ratio)

    remaining = 1.0 - (
        cfg.layout.estop_height_ratio +
        cfg.layout.settings_height_ratio +
        cfg.layout.telemetry_height_ratio
    )
    cfg.layout.rover_icon_height_ratio = max(0.0, remaining)

    imgui.separator()
    imgui.text("Panels")
    changed, cfg.panels.show_camera = imgui.checkbox("Show Camera Panel", cfg.panels.show_camera)
    changed, cfg.panels.show_estop = imgui.checkbox("Show E-stop Panel", cfg.panels.show_estop)
    changed, cfg.panels.show_settings = imgui.checkbox("Show Settings Panel", cfg.panels.show_settings)
    changed, cfg.panels.show_telemetry = imgui.checkbox("Show Telemetry Panel", cfg.panels.show_telemetry)
    changed, cfg.panels.show_rover_icon = imgui.checkbox("Show Rover Icon Panel", cfg.panels.show_rover_icon)

    imgui.separator()
    imgui.text("Camera")
    changed, cfg.camera.default_camera = imgui.input_int("Default Camera", cfg.camera.default_camera)
    changed, cfg.camera.width = imgui.input_int("Camera Width", cfg.camera.width)
    changed, cfg.camera.height = imgui.input_int("Camera Height", cfg.camera.height)

    imgui.separator()
    imgui.text("Keybinds")
    changed, cfg.keybinds.camera_1 = imgui.input_text("Camera 1 Key", cfg.keybinds.camera_1, 16)
    changed, cfg.keybinds.camera_2 = imgui.input_text("Camera 2 Key", cfg.keybinds.camera_2, 16)
    changed, cfg.keybinds.camera_3 = imgui.input_text("Camera 3 Key", cfg.keybinds.camera_3, 16)
    changed, cfg.keybinds.shutdown_popup = imgui.input_text("Shutdown Popup Key", cfg.keybinds.shutdown_popup, 16)
    changed, cfg.keybinds.estop = imgui.input_text("E-stop Key", cfg.keybinds.estop, 16)

    imgui.separator()

    if imgui.button("Apply", 140, 40):
        state.config = copy.deepcopy(state.edit_config)
        save_config(state.config)
        state.request_apply_settings = True
        state.pending_camera_reconfigure = True
        state.pending_window_resize = True

    imgui.same_line()

    if imgui.button("Reset to Defaults", 180, 40):
        state.edit_config = reset_config_defaults()

    imgui.same_line()

    if imgui.button("Close", 140, 40):
        state.edit_config = copy.deepcopy(state.config)
        imgui.close_current_popup()

    imgui.end_popup()

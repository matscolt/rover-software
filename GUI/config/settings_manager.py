import json
from pathlib import Path
from core.app_config import AppConfig, WindowConfig, LayoutConfig, PanelsConfig, CameraConfig

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
    )


def save_config(config: AppConfig) -> None:
    data = {
        "window": vars(config.window),
        "layout": vars(config.layout),
        "panels": vars(config.panels),
        "camera": vars(config.camera),
    }

    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)

    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)
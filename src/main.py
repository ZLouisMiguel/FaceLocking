import os

from .recognize import run_recognition


def _env_int(name, default):
    try:
        return int(os.getenv(name, default))
    except (TypeError, ValueError):
        return default


def _env_float(name, default):
    try:
        return float(os.getenv(name, default))
    except (TypeError, ValueError):
        return default


def _env_bool(name, default=True):
    return os.getenv(name, str(default)).strip().lower() not in {
        "0",
        "false",
        "no",
        "off",
    }

if __name__ == "__main__":
    run_recognition(
        db_path=os.getenv("FACE_DATABASE", "data/database.json"),
        cam_source=_env_int("FACE_CAMERA_SOURCE", 2),
        lock_threshold=_env_float("FACE_LOCK_THRESHOLD", 0.4),
        motor_enabled=_env_bool("FACE_MOTOR_ENABLED", True),
    )

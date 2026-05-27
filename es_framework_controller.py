"""
Python controller shim for the ES_Framework C adapter.

Run:
    python sluggers_sim.py --controller es_framework_controller.py

Build first if needed:
    python es_sim_adapter/build_es_adapter.py --project-root ECE118FinalProject

Optional controller configuration is read from:
    ES_SIM_PROJECT_ROOT, ES_SIM_PROJECT_SRC, ES_SIM_PROJECT_INCLUDE, ES_SIM_HEX
"""

from __future__ import annotations

import argparse
import ctypes
import os
import platform
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
ADAPTER_DIR = ROOT / "es_sim_adapter"


class SimTapeSensors(ctypes.Structure):
    _fields_ = [
        ("front_left", ctypes.c_uint8),
        ("front_center", ctypes.c_uint8),
        ("front_right", ctypes.c_uint8),
        ("mid_left", ctypes.c_uint8),
        ("mid_right", ctypes.c_uint8),
        ("rear_left", ctypes.c_uint8),
        ("rear_center", ctypes.c_uint8),
        ("rear_right", ctypes.c_uint8),
        ("front", ctypes.c_uint8),
        ("rear", ctypes.c_uint8),
        ("left", ctypes.c_uint8),
        ("right", ctypes.c_uint8),
    ]


class SimBumpSensors(ctypes.Structure):
    _fields_ = [
        ("any", ctypes.c_uint8),
        ("front", ctypes.c_uint8),
        ("rear", ctypes.c_uint8),
        ("left", ctypes.c_uint8),
        ("right", ctypes.c_uint8),
    ]


class SimPingSensors(ctypes.Structure):
    _fields_ = [
        ("front", ctypes.c_double),
        ("front_left", ctypes.c_double),
        ("front_right", ctypes.c_double),
        ("left", ctypes.c_double),
        ("right", ctypes.c_double),
    ]


class SimIrBeacon(ctypes.Structure):
    _fields_ = [
        ("range_in", ctypes.c_double),
        ("bearing_deg", ctypes.c_double),
        ("strength", ctypes.c_double),
        ("front", ctypes.c_uint8),
        ("rear", ctypes.c_uint8),
    ]


class SimPose(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("heading_deg", ctypes.c_double),
    ]


class SimGameState(ctypes.Structure):
    _fields_ = [
        ("ammo", ctypes.c_uint8),
        ("score", ctypes.c_uint16),
        ("hits", ctypes.c_uint8),
        ("goal", ctypes.c_uint8),
        ("legal_zone", ctypes.c_uint8),
        ("isz_reached", ctypes.c_uint8),
        ("disqualified", ctypes.c_uint8),
    ]


class SimSensors(ctypes.Structure):
    _fields_ = [
        ("time_s", ctypes.c_double),
        ("dt_s", ctypes.c_double),
        ("pose", SimPose),
        ("tape", SimTapeSensors),
        ("bump", SimBumpSensors),
        ("ping", SimPingSensors),
        ("target_ir", SimIrBeacon),
        ("obstacle_ir", SimIrBeacon),
        ("game", SimGameState),
    ]


class SimControls(ctypes.Structure):
    _fields_ = [
        ("vx", ctypes.c_double),
        ("vy", ctypes.c_double),
        ("omega", ctypes.c_double),
        ("shoot", ctypes.c_uint8),
    ]


_lib: ctypes.CDLL | None = None
_initialized = False
_project_root: Path | None = None
_project_src: Path | None = None
_project_include: Path | None = None
_hex_path: Path | None = None


def _library_path() -> Path:
    if platform.system() == "Windows":
        name = "sim_robot_adapter.dll"
    elif platform.system() == "Darwin":
        name = "libsim_robot_adapter.dylib"
    else:
        name = "libsim_robot_adapter.so"
    return ADAPTER_DIR / "build" / name


def _path_from_env(name: str) -> Path | None:
    value = os.environ.get(name)
    return Path(value).expanduser().resolve() if value else None


def configure(
    project_root: str | Path | None = None,
    project_src: str | Path | None = None,
    project_include: str | Path | None = None,
    hex_path: str | Path | None = None,
) -> None:
    global _project_root, _project_src, _project_include, _hex_path
    if _lib is not None:
        raise RuntimeError("Cannot reconfigure es_framework_controller after the adapter library has loaded")
    _project_root = Path(project_root).expanduser().resolve() if project_root else None
    _project_src = Path(project_src).expanduser().resolve() if project_src else None
    _project_include = Path(project_include).expanduser().resolve() if project_include else None
    _hex_path = Path(hex_path).expanduser().resolve() if hex_path else None
    if _hex_path is not None:
        _validate_hex_path(_hex_path)


def _validate_hex_path(path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(f"Hex file does not exist: {path}")
    if path.suffix.lower() != ".hex":
        raise ValueError(f"Expected a .hex file, got: {path}")


def _configured_build_args() -> list[str]:
    project_root = _project_root or _path_from_env("ES_SIM_PROJECT_ROOT")
    project_src = _project_src or _path_from_env("ES_SIM_PROJECT_SRC")
    project_include = _project_include or _path_from_env("ES_SIM_PROJECT_INCLUDE")
    hex_path = _hex_path or _path_from_env("ES_SIM_HEX")

    args: list[str] = []
    if project_root is not None:
        args += ["--project-root", str(project_root)]
    if project_src is not None:
        args += ["--project-src", str(project_src)]
    if project_include is not None:
        args += ["--project-include", str(project_include)]
    if hex_path is not None:
        args += ["--hex", str(hex_path)]
    return args


def _build_if_needed(path: Path, force: bool = False) -> None:
    build_args = _configured_build_args()
    if path.exists() and not build_args and not force:
        return
    subprocess.check_call(
        [sys.executable, str(ADAPTER_DIR / "build_es_adapter.py"), *build_args],
        cwd=ROOT,
    )


def _load_lib() -> ctypes.CDLL:
    global _lib, _initialized
    if _lib is not None:
        return _lib

    path = _library_path()
    _build_if_needed(path)
    _lib = ctypes.CDLL(str(path))
    _lib.SimRobot_Init.argtypes = []
    _lib.SimRobot_Init.restype = ctypes.c_int
    _lib.SimRobot_Reset.argtypes = []
    _lib.SimRobot_Reset.restype = None
    _lib.SimRobot_Step.argtypes = [ctypes.POINTER(SimSensors), ctypes.c_double, ctypes.POINTER(SimControls)]
    _lib.SimRobot_Step.restype = ctypes.c_int
    _initialized = bool(_lib.SimRobot_Init())
    if not _initialized:
        raise RuntimeError("SimRobot_Init failed")
    return _lib


def _u8(value: object) -> int:
    return 1 if bool(value) else 0


def _pack_ir(ir: dict) -> SimIrBeacon:
    return SimIrBeacon(
        float(ir.get("range_in", 0.0)),
        float(ir.get("bearing_deg", 0.0)),
        float(ir.get("strength", 0.0)),
        _u8(ir.get("front", False)),
        _u8(ir.get("rear", False)),
    )


def _pack_sensors(sensors: dict, dt: float) -> SimSensors:
    tape = sensors["tape"]
    bump = sensors["bump"]
    ping = sensors["ping"]
    pose = sensors["pose"]
    game = sensors["game"]
    ir = sensors["ir"]

    return SimSensors(
        float(sensors.get("time_s", 0.0)),
        float(dt),
        SimPose(float(pose["x"]), float(pose["y"]), float(pose["heading_deg"])),
        SimTapeSensors(
            _u8(tape.get("front_left", False)),
            _u8(tape.get("front_center", False)),
            _u8(tape.get("front_right", False)),
            _u8(tape.get("mid_left", False)),
            _u8(tape.get("mid_right", False)),
            _u8(tape.get("rear_left", False)),
            _u8(tape.get("rear_center", False)),
            _u8(tape.get("rear_right", False)),
            _u8(tape.get("front", False)),
            _u8(tape.get("rear", False)),
            _u8(tape.get("left", False)),
            _u8(tape.get("right", False)),
        ),
        SimBumpSensors(
            _u8(bump.get("any", False)),
            _u8(bump.get("front", False)),
            _u8(bump.get("rear", False)),
            _u8(bump.get("left", False)),
            _u8(bump.get("right", False)),
        ),
        SimPingSensors(
            float(ping["front"]["range_in"]),
            float(ping["front_left"]["range_in"]),
            float(ping["front_right"]["range_in"]),
            float(ping["left"]["range_in"]),
            float(ping["right"]["range_in"]),
        ),
        _pack_ir(ir["target_2k"]),
        _pack_ir(ir["nearest_obstacle"]),
        SimGameState(
            int(game["ammo"]),
            int(game["score"]),
            int(game["hits"]),
            _u8(game["goal"]),
            int(game["legal_zone"]),
            _u8(game["isz_reached"]),
            _u8(game["disqualified"]),
        ),
    )


def reset() -> None:
    lib = _load_lib()
    lib.SimRobot_Reset()


def update(sensors: dict, dt: float) -> dict:
    lib = _load_lib()
    packed = _pack_sensors(sensors, dt)
    controls = SimControls()
    ok = lib.SimRobot_Step(ctypes.byref(packed), float(dt), ctypes.byref(controls))
    if not ok:
        raise RuntimeError("SimRobot_Step failed")
    return {
        "vx": controls.vx,
        "vy": controls.vy,
        "omega": controls.omega,
        "shoot": bool(controls.shoot),
    }


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build or validate the ES framework simulator controller")
    parser.add_argument(
        "--project-root",
        "--project",
        dest="project_root",
        default=None,
        help="Project root containing src/ and include/",
    )
    parser.add_argument("--project-src", "--src", dest="project_src", default=None, help="Project source directory")
    parser.add_argument("--project-include", "--include", dest="project_include", default=None, help="Project include directory")
    parser.add_argument("--hex", default=None, help="Optional MPLAB .hex path to validate while building")
    parser.add_argument("--build", action="store_true", help="Build the adapter immediately")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        configure(
            project_root=args.project_root,
            project_src=args.project_src,
            project_include=args.project_include,
            hex_path=args.hex,
        )
    except Exception as exc:
        print(f"Configuration failed: {exc}", file=sys.stderr)
        return 1
    if args.build:
        try:
            _build_if_needed(_library_path(), force=True)
        except Exception as exc:
            print(f"Build failed: {exc}", file=sys.stderr)
            return 1
        print(_library_path())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

from __future__ import annotations

import argparse
import platform
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
BUILD = ROOT / "build"
DEFAULT_PROJECT_ROOT = ROOT.parent.parent / "ece118_finalproject"


class BuildConfig:
    def __init__(
        self,
        project_root: Path,
        project_src: Path,
        project_include: Path,
        hex_path: Path | None = None,
    ) -> None:
        self.project_root = project_root
        self.project_src = project_src
        self.project_include = project_include
        self.hex_path = hex_path


def library_name() -> str:
    if platform.system() == "Windows":
        return "sim_robot_adapter.dll"
    if platform.system() == "Darwin":
        return "libsim_robot_adapter.dylib"
    return "libsim_robot_adapter.so"


def find_compiler() -> str:
    for compiler in ("gcc", "clang", "cc"):
        found = shutil.which(compiler)
        if found:
            return found
    raise RuntimeError("No C compiler found. Install gcc/clang or run from a shell with a compiler on PATH.")


def _resolve_path(path: str | Path, base: Path | None = None) -> Path:
    candidate = Path(path).expanduser()
    if not candidate.is_absolute() and base is not None:
        candidate = base / candidate
    return candidate.resolve()


def _infer_project_root_from_hex(hex_path: Path) -> Path | None:
    for parent in hex_path.parents:
        if (parent / "src").is_dir() and (parent / "include").is_dir():
            return parent
    return None


def _require_directory(path: Path, label: str) -> None:
    if not path.is_dir():
        raise FileNotFoundError(f"{label} does not exist or is not a directory: {path}")


def _validate_hex_path(hex_path: Path) -> None:
    if not hex_path.is_file():
        raise FileNotFoundError(f"Hex file does not exist: {hex_path}")
    if hex_path.suffix.lower() != ".hex":
        raise ValueError(f"Expected a .hex file, got: {hex_path}")


def resolve_config(
    project_root: str | Path | None = None,
    project_src: str | Path | None = None,
    project_include: str | Path | None = None,
    hex_path: str | Path | None = None,
) -> BuildConfig:
    resolved_hex = _resolve_path(hex_path) if hex_path else None
    if resolved_hex is not None:
        _validate_hex_path(resolved_hex)

    if project_root is None and resolved_hex is not None:
        inferred = _infer_project_root_from_hex(resolved_hex)
        resolved_project_root = inferred if inferred is not None else DEFAULT_PROJECT_ROOT.resolve()
    else:
        resolved_project_root = _resolve_path(project_root or DEFAULT_PROJECT_ROOT)

    resolved_project_src = _resolve_path(project_src, resolved_project_root) if project_src else resolved_project_root / "src"
    resolved_project_include = (
        _resolve_path(project_include, resolved_project_root) if project_include else resolved_project_root / "include"
    )

    _require_directory(resolved_project_root, "Project root")
    _require_directory(resolved_project_src, "Project source directory")
    _require_directory(resolved_project_include, "Project include directory")

    return BuildConfig(
        project_root=resolved_project_root,
        project_src=resolved_project_src,
        project_include=resolved_project_include,
        hex_path=resolved_hex,
    )


def build(config: BuildConfig | None = None) -> Path:
    config = config or resolve_config()
    BUILD.mkdir(exist_ok=True)
    out = BUILD / library_name()
    compiler = find_compiler()
    sources = [
        ROOT / "sim_robot_adapter.c",
        ROOT / "host_stubs" / "BOARD.c",
        ROOT / "host_stubs" / "AD.c",
        ROOT / "host_stubs" / "IO_Ports.c",
        ROOT / "host_stubs" / "pwm.c",
        ROOT / "host_stubs" / "serial.c",
        config.project_src / "ES_CheckEvents.c",
        config.project_src / "ES_Framework.c",
        config.project_src / "ES_KeyboardInput.c",
        config.project_src / "ES_PostList.c",
        config.project_src / "ES_Queue.c",
        config.project_src / "ES_Timers.c",
        config.project_src / "RobotEventCheckers.c",
        config.project_src / "DriveMotorService.c",
        config.project_src / "FiringMotorService.c",
        config.project_src / "TapeSensorService.c",
        config.project_src / "BumpDetectorService.c",
        config.project_src / "PingSensorService.c",
        config.project_src / "TrackWireService.c",
        config.project_src / "StrategyService.c",
    ]
    missing_sources = [src for src in sources if not src.is_file()]
    if missing_sources:
        missing = "\n  ".join(str(src) for src in missing_sources)
        raise FileNotFoundError(f"Missing C source files:\n  {missing}")

    cmd = [
        compiler,
        "-std=c99",
        "-O2",
        "-Wall",
        "-Wextra",
        "-DES_SIM_BUILD",
        "-I",
        str(ROOT / "host_stubs"),
        "-shared",
        "-I",
        str(ROOT),
        "-I",
        str(config.project_include),
    ]
    if platform.system() != "Windows":
        cmd.append("-fPIC")
    cmd += [str(src) for src in sources]
    cmd += ["-o", str(out)]
    if platform.system() != "Windows":
        cmd.append("-lm")

    print("Building:", " ".join(cmd))
    subprocess.check_call(cmd, cwd=ROOT)
    return out


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build the native ES simulator adapter")
    parser.add_argument(
        "--project-root",
        "--project",
        dest="project_root",
        default=None,
        help="Project root containing src/ and include/ (default: ../ece118_finalproject)",
    )
    parser.add_argument("--project-src", "--src", dest="project_src", default=None, help="Project source directory")
    parser.add_argument(
        "--project-include",
        "--include",
        dest="project_include",
        default=None,
        help="Project include directory",
    )
    parser.add_argument("--hex", default=None, help="Optional MPLAB .hex path to validate for workflow compatibility")
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args(sys.argv[1:])
    try:
        config = resolve_config(
            project_root=args.project_root,
            project_src=args.project_src,
            project_include=args.project_include,
            hex_path=args.hex,
        )
        out = build(config)
    except Exception as exc:
        print(f"Build failed: {exc}", file=sys.stderr)
        return 1
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Discover KiCad's Python binding and command-line tools on the host OS."""

from __future__ import annotations

import importlib
import os
import shutil
import sys
from pathlib import Path
from typing import Any


_DLL_HANDLES: list[Any] = []


def _env_path(name: str) -> Path | None:
    value = os.environ.get(name, "").strip().strip('"')
    return Path(value).expanduser() if value else None


def _unique(paths: list[Path]) -> list[Path]:
    result: list[Path] = []
    seen: set[str] = set()
    for path in paths:
        try:
            resolved = path.expanduser().resolve()
        except OSError:
            resolved = path.expanduser()
        key = os.path.normcase(str(resolved))
        if key not in seen:
            seen.add(key)
            result.append(resolved)
    return result


def _roots() -> list[Path]:
    roots: list[Path] = []
    override = _env_path("KICAD_ROOT")
    if override is not None:
        roots.append(override)

    if sys.platform == "win32":
        for variable in ("ProgramFiles", "ProgramFiles(x86)", "LOCALAPPDATA"):
            base = os.environ.get(variable)
            if base:
                suffix = "Programs/KiCad" if variable == "LOCALAPPDATA" else "KiCad"
                roots.append(Path(base) / suffix)
        roots.append(Path("C:/KiCad"))
    elif sys.platform == "darwin":
        roots.extend((
            Path("/Applications/KiCad/KiCad.app/Contents"),
            Path("/Applications/KiCad.app/Contents/SharedSupport"),
            Path("/Applications/kicad/KiCad.app/Contents"),
            Path("/opt/homebrew/opt/kicad"),
            Path("/usr/local/opt/kicad"),
        ))
    else:
        roots.extend((
            Path("/usr"), Path("/usr/local"), Path("/opt/kicad"),
            Path("/opt/KiCad"),
        ))

    expanded = list(roots)
    if sys.platform == "win32":
        for root in roots:
            if root.is_dir():
                try:
                    expanded.extend(child for child in root.iterdir() if child.is_dir())
                except OSError:
                    pass
    for root in roots:
        if root.suffix.lower() == ".app":
            expanded.extend((root / "Contents", root / "Contents" / "SharedSupport"))
        elif root.name.lower() == "contents":
            expanded.append(root / "SharedSupport")
    return _unique(expanded)


def _binding_candidates() -> list[Path]:
    major, minor = sys.version_info[:2]
    version = f"{major}.{minor}"
    candidates: list[Path] = []
    override = _env_path("KICAD_PYTHON_PATH")
    if override is not None:
        candidates.append(override)

    for root in _roots():
        candidates.extend((
            root,
            root / "bin",
            root / "lib" / "python3" / "dist-packages",
            root / "lib" / "python3" / "site-packages",
            root / "lib" / f"python{version}" / "site-packages",
            root / "lib" / f"python{version}" / "dist-packages",
            root / "Lib" / "site-packages",
            root / "Frameworks" / "Python.framework" / "Versions" / "Current"
            / "lib" / f"python{version}" / "site-packages",
            root / "Contents" / "Frameworks" / "Python.framework" / "Versions"
            / "Current" / "lib" / f"python{version}" / "site-packages",
        ))

    if sys.platform.startswith("linux"):
        candidates.extend((
            Path("/usr/lib/python3/dist-packages"),
            Path("/usr/local/lib/python3/dist-packages"),
            Path(f"/usr/lib/python{version}/site-packages"),
            Path(f"/usr/local/lib/python{version}/site-packages"),
            Path(f"/usr/lib/python{version}/dist-packages"),
        ))
    return _unique([path for path in candidates if path.is_dir()])


def _has_binding(path: Path) -> bool:
    return any(path.glob("pcbnew.py")) or any(path.glob("_pcbnew*"))


def _binary_directories() -> list[Path]:
    directories: list[Path] = []
    override = _env_path("KICAD_BIN")
    if override is not None:
        directories.append(override)
    directories.extend(root / "bin" for root in _roots())
    return _unique([path for path in directories if path.is_dir()])


def configure_import_paths() -> list[Path]:
    """Add discovered pcbnew locations and Windows DLL directories."""
    binding_dirs = [path for path in _binding_candidates() if _has_binding(path)]
    for path in reversed(binding_dirs):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))

    if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
        for path in _binary_directories():
            try:
                _DLL_HANDLES.append(os.add_dll_directory(str(path)))
            except OSError:
                continue
    return binding_dirs


def import_pcbnew() -> Any:
    """Import pcbnew after discovering the host KiCad installation."""
    configure_import_paths()
    try:
        return importlib.import_module("pcbnew")
    except (ImportError, OSError) as exc:
        found = [str(path) for path in _binding_candidates() if _has_binding(path)]
        locations = ", ".join(found) if found else "no pcbnew module directory found"
        raise RuntimeError(
            "Could not load KiCad's pcbnew Python binding for "
            f"{sys.executable} ({sys.version.split()[0]}). Detected: {locations}. "
            "Set KICAD_ROOT to the KiCad installation, or KICAD_PYTHON_PATH to "
            "the directory containing pcbnew.py/_pcbnew; set KICAD_BIN if the "
            "native libraries are in a separate directory. The binding must "
            f"match this Python version. Import error: {type(exc).__name__}: {exc}"
        ) from exc


def find_kicad_cli() -> Path:
    """Find kicad-cli using explicit overrides, KiCad roots, or PATH."""
    candidates: list[Path] = []
    configured = _env_path("KICAD_CLI")
    executable_names = ("kicad-cli.exe", "kicad-cli") if sys.platform == "win32" else ("kicad-cli",)
    if configured is not None:
        if configured.is_dir():
            candidates.extend(configured / name for name in executable_names)
        else:
            candidates.append(configured)

    binary_override = _env_path("KICAD_BIN")
    if binary_override is not None:
        candidates.extend(binary_override / name for name in executable_names)
    for root in _roots():
        candidates.extend(root / "bin" / name for name in executable_names)
        candidates.extend(root / "MacOS" / name for name in executable_names)
        candidates.extend(root / name for name in executable_names)

    on_path = shutil.which("kicad-cli") or shutil.which("kicad-cli.exe")
    if on_path:
        candidates.append(Path(on_path))
    for candidate in _unique(candidates):
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "Could not find kicad-cli. Set KICAD_CLI to its executable or directory, "
        "KICAD_BIN to its directory, or KICAD_ROOT to the KiCad installation."
    )


def subprocess_environment() -> dict[str, str]:
    """Preserve platform runtime paths and KiCad overrides for worker processes."""
    keep = {
        "PATH", "HOME", "USERPROFILE", "SYSTEMROOT", "WINDIR", "PATHEXT",
        "APPDATA", "LOCALAPPDATA", "PROGRAMDATA", "HOMEDRIVE", "HOMEPATH",
        "TEMP", "TMP", "LANG", "LD_LIBRARY_PATH", "DYLD_LIBRARY_PATH",
        "PYTHONPATH",
    }
    return {
        key: value for key, value in os.environ.items()
        if key in keep or key.startswith("KICAD_") or key == "KISYSMOD"
    }

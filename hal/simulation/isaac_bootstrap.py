"""
Isaac Sim environment bootstrap for the HAL watchdog.

Used by ``--vnc`` mode to assemble everything ``run_test_isaacsim.sh`` used to
do from bash, but from inside the Python process:

  * ``DISPLAY``       - point at the container's Xvfb (e.g. ``:99``) so Kit can
    open its main window and the user can see it over noVNC.
  * plain env vars    - ``ISAAC_PATH`` / ``CARB_APP_PATH`` / ``EXP_PATH`` /
    ``INTERNUTOPIA_ASSETS_PATH`` / ...
  * ``setup_python_env.sh`` - source it and copy the resulting ``PYTHONPATH`` /
    ``LD_LIBRARY_PATH`` / ... back into the current process.
  * ``extra_pythonpath`` - additional prefixes (e.g. ``/data/.../InternUtopia``)
    prepended to both ``PYTHONPATH`` and ``sys.path``.

This MUST run before any ``import internutopia`` / ``import isaacsim`` /
``import omni`` / ``import carb``. Calling it a second time is a no-op.

.. note::
    ``LD_LIBRARY_PATH`` is consumed by glibc's dynamic loader at process
    startup; mutating ``os.environ["LD_LIBRARY_PATH"]`` inside a running
    Python process does **not** change ``dlopen`` search paths. So if
    sourcing ``setup_python_env.sh`` adds new entries (it always does —
    ``/isaac-sim/kit`` etc. for ``libcarb.so``), we ``os.execvpe`` the
    current Python with the new environment. A sentinel env var
    (``_PAOS_ISAAC_BOOTSTRAPPED``) prevents an exec loop.

The config block is read from driver-config JSON at key ``isaac_env``, e.g.::

    "isaac_env": {
        "display": ":99",
        "env": {
            "ISAAC_PATH": "/isaac-sim",
            "CARB_APP_PATH": "/isaac-sim/kit",
            "EXP_PATH": "/isaac-sim/apps",
            "INTERNUTOPIA_ASSETS_PATH": "/data/datasets/GRScenes"
        },
        "setup_python_env": "/isaac-sim/setup_python_env.sh",
        "extra_pythonpath": ["/data/wangyuanlei/InternUtopia"]
    }
"""

from __future__ import annotations

import os
import shlex
import shutil
import socket
import subprocess
import sys
from pathlib import Path
from typing import Any

_DONE = False
_SENTINEL_ENV = "_PAOS_ISAAC_BOOTSTRAPPED"


def _log(msg: str) -> None:
    print(f"[isaac-bootstrap] {msg}", flush=True)


def _install_pythonpath_into_sys_path() -> None:
    for entry in os.environ.get("PYTHONPATH", "").split(os.pathsep):
        if entry and entry not in sys.path:
            sys.path.insert(0, entry)


def _is_local_port_listening(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.3)
        return sock.connect_ex(("127.0.0.1", int(port))) == 0


def _ensure_novnc_websockify(cfg: dict[str, Any]) -> None:
    if not bool(cfg.get("autostart_novnc", True)):
        return

    try:
        port = int(cfg.get("novnc_port", 31315))
    except (TypeError, ValueError):
        _log("WARNING: invalid novnc_port in isaac_env; expected int")
        return

    if _is_local_port_listening(port):
        _log(f"noVNC already listening on :{port}")
        return

    websockify_bin = shutil.which("websockify") or "/usr/bin/websockify"
    if not Path(websockify_bin).exists():
        _log("WARNING: websockify not found; skip noVNC autostart")
        return

    novnc_web = str(cfg.get("novnc_web", "/usr/share/novnc"))
    novnc_target = str(cfg.get("novnc_target", "localhost:5900"))
    cmd = [websockify_bin, "--web", novnc_web, str(port), novnc_target]
    try:
        subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=os.environ.copy(),
            start_new_session=True,
        )
        _log(f"started noVNC websockify on :{port} -> {novnc_target}")
    except Exception as exc:  # pragma: no cover - defensive runtime guard
        _log(f"WARNING: failed to start noVNC websockify on :{port}: {exc}")


def _source_setup_script(script_path: str) -> dict[str, str]:
    """
    Run ``bash -lc 'source <script> && env -0'`` inheriting the current
    environment, and return the resulting env as a dict. Using ``-0`` gives us a
    NUL-separated stream so values containing newlines survive round-tripping.
    """
    cmd = f"set -a && source {shlex.quote(script_path)} >/dev/null 2>&1 && env -0"
    raw = subprocess.check_output(["bash", "-lc", cmd], env=os.environ.copy())
    out: dict[str, str] = {}
    for entry in raw.split(b"\0"):
        if not entry:
            continue
        key, _, value = entry.decode("utf-8", "replace").partition("=")
        if key:
            out[key] = value
    return out


def bootstrap_isaac_env(cfg: dict[str, Any] | None, *, want_gui: bool) -> None:
    """
    Install Isaac Sim env vars / ``sys.path`` entries into the current process.

    Parameters
    ----------
    cfg:
        Contents of the ``isaac_env`` block from driver-config (may be ``None``
        or empty - in that case we still honor ``want_gui`` and set a sensible
        default ``DISPLAY``).
    want_gui:
        If True, ensure ``DISPLAY`` is set (defaulting to ``:99`` for the
        container's Xvfb) so Kit opens a viewport. If False, we explicitly
        unset ``DISPLAY`` to avoid Kit trying to connect to an X server.
    """
    global _DONE
    if _DONE:
        return

    # Second pass: we are the re-exec'd child. The real env (DISPLAY,
    # LD_LIBRARY_PATH, PYTHONPATH, ISAAC_PATH, ...) is already in place —
    # just mirror PYTHONPATH into sys.path and mark done.
    if os.environ.get(_SENTINEL_ENV) == "1":
        _install_pythonpath_into_sys_path()
        display = os.environ.get("DISPLAY", "<unset>")
        _log(f"post-reexec ready (want_gui={want_gui}, DISPLAY={display})")
        _DONE = True
        return

    cfg = cfg or {}

    for key, value in (cfg.get("env") or {}).items():
        os.environ.setdefault(str(key), str(value))

    if want_gui:
        os.environ.setdefault("DISPLAY", str(cfg.get("display", ":99")))
        _ensure_novnc_websockify(cfg)
    else:
        os.environ.pop("DISPLAY", None)

    extra_paths = [str(p) for p in (cfg.get("extra_pythonpath") or []) if p]
    if extra_paths:
        current = os.environ.get("PYTHONPATH", "")
        os.environ["PYTHONPATH"] = os.pathsep.join(
            [*extra_paths, *([current] if current else [])]
        )

    ld_before = os.environ.get("LD_LIBRARY_PATH", "")

    setup_sh = cfg.get("setup_python_env")
    if setup_sh:
        script_path = str(setup_sh)
        if Path(script_path).exists():
            try:
                new_env = _source_setup_script(script_path)
                for key, value in new_env.items():
                    os.environ[key] = value
                _log(f"sourced {script_path}")
            except subprocess.CalledProcessError as exc:
                _log(f"WARNING: failed to source {script_path}: {exc}")
        else:
            _log(f"WARNING: setup_python_env script not found: {script_path}")

    _install_pythonpath_into_sys_path()

    ld_after = os.environ.get("LD_LIBRARY_PATH", "")
    if ld_after and ld_after != ld_before:
        # glibc ld.so already read LD_LIBRARY_PATH at startup; the only way to
        # make libcarb.so / isaacsim / omni dlopen succeed is to restart this
        # Python process with the new environment inherited from fork.
        os.environ[_SENTINEL_ENV] = "1"
        display = os.environ.get("DISPLAY", "<unset>")
        _log(
            f"LD_LIBRARY_PATH changed; re-exec {sys.executable} "
            f"(DISPLAY={display})"
        )
        sys.stdout.flush()
        sys.stderr.flush()
        os.execvpe(sys.executable, [sys.executable, *sys.argv], os.environ)
        # unreachable

    display = os.environ.get("DISPLAY", "<unset>")
    _log(f"ready (want_gui={want_gui}, DISPLAY={display})")
    _DONE = True


def is_bootstrapped() -> bool:
    return _DONE

#!/usr/bin/env python3
"""Restart the Quest WebXR bridge stack across Jetson, this PC, and ADB."""

from __future__ import annotations

import argparse
import getpass
import os
import subprocess
import sys
import time
from pathlib import Path
from urllib.request import urlopen

import paramiko


ROOT = Path(__file__).resolve().parent


JETSON_SCRIPT = r'''
REMOTE_BRIDGE_DIR="${YUANYOU_WEBXR_REMOTE_DIR:-$HOME/openpi-robot/yuanyou-remote control}"
cd "$REMOTE_BRIDGE_DIR" || exit 1
if [ -f ./source_ros_setup.sh ]; then
  source ./source_ros_setup.sh
elif [ -f "$HOME/openpi-robot/questVR_ws/devel/setup.bash" ]; then
  source "$HOME/openpi-robot/questVR_ws/devel/setup.bash"
elif [ -f "$HOME/yuanyou2_ws/devel/setup.bash" ]; then
  source "$HOME/yuanyou2_ws/devel/setup.bash"
else
  source /opt/ros/noetic/setup.bash
fi

if ! rostopic list >/dev/null 2>&1; then
  setsid roscore > /tmp/yuanyou_remote_roscore.log 2>&1 &
  echo $! > /tmp/yuanyou_remote_roscore.pid
  sleep 4
fi
rostopic list >/dev/null || { echo 'roscore failed'; cat /tmp/yuanyou_remote_roscore.log; exit 1; }

if [ -f /tmp/yuanyou_remote_bridge.pid ]; then
  pid=$(cat /tmp/yuanyou_remote_bridge.pid || true)
  [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
fi
if [ -f /tmp/quest_vr_piper_adapter.pid ]; then
  pid=$(cat /tmp/quest_vr_piper_adapter.pid || true)
  [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
fi
pkill -f 'quest_webxr_bridge.py' || true
pkill -f 'quest_vr_piper_adapter.py' || true
sleep 1

: > bridge_jetson.log
setsid env ADB_REVERSE=0 ./start_on_jetson.sh > bridge_jetson.log 2>&1 &
echo $! > /tmp/yuanyou_remote_bridge.pid

: > piper_adapter_preview.log
setsid ./start_piper_adapter_preview.sh > piper_adapter_preview.log 2>&1 &
echo $! > /tmp/quest_vr_piper_adapter.pid

sleep 3
python3 - <<'EOF'
from urllib.request import urlopen
r = urlopen('http://127.0.0.1:8088/quest_webxr_client.html', timeout=5)
print('JETSON_HTTP_OK', r.status)
EOF
rostopic list | grep -E '^/quest_vr|^/quest_vr_piper' | sort || true
'''


def run(command: list[str], check: bool = True) -> subprocess.CompletedProcess[str]:
    print("+", " ".join(command), flush=True)
    return subprocess.run(command, text=True, check=check)


def restart_jetson(host: str, user: str, password: str) -> None:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        hostname=host,
        username=user,
        password=password,
        timeout=10,
        look_for_keys=False,
        allow_agent=False,
    )
    stdin, stdout, stderr = client.exec_command("bash -s", timeout=60)
    stdin.write(JETSON_SCRIPT)
    stdin.channel.shutdown_write()
    out = stdout.read().decode("utf-8", "replace")
    err = stderr.read().decode("utf-8", "replace")
    code = stdout.channel.recv_exit_status()
    client.close()
    print(out.rstrip())
    if err:
        print(err.rstrip(), file=sys.stderr)
    if code != 0:
        raise SystemExit(code)


def restart_local_forward(host: str, user: str, password: str) -> None:
    subprocess.run(["pkill", "-f", "python3 jetson_port_forward.py"], check=False)
    time.sleep(0.5)
    env = os.environ.copy()
    env["JETSON_PASS"] = password
    log = ROOT / "jetson_port_forward.log"
    with log.open("w", encoding="utf-8") as handle:
        subprocess.Popen(
            [str(ROOT / "start_quest_to_jetson.sh")],
            cwd=str(ROOT),
            env=env,
            stdout=handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    time.sleep(1.0)
    print(log.read_text(encoding="utf-8", errors="replace").rstrip())
    with urlopen("http://127.0.0.1:8088/quest_webxr_client.html", timeout=5) as resp:
        print("LOCAL_HTTP_OK", resp.status)


def restart_adb(open_browser: bool) -> None:
    run(["adb", "reverse", "--remove-all"], check=False)
    run(["adb", "reverse", "tcp:8088", "tcp:8088"])
    run(["adb", "reverse", "tcp:8765", "tcp:8765"])
    run(["adb", "reverse", "--list"], check=False)
    run(["adb", "shell", "settings", "put", "global", "http_proxy", ":0"], check=False)
    if open_browser:
        run(["adb", "shell", "am", "force-stop", "com.oculus.browser"], check=False)
        run(
            [
                "adb",
                "shell",
                "am",
                "start",
                "-n",
                "com.oculus.browser/.OculusLauncherActivity",
                "-a",
                "android.intent.action.VIEW",
                "-c",
                "android.intent.category.BROWSABLE",
                "-d",
                "http://localhost:8088/quest_webxr_client.html",
            ],
            check=False,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.31.230")
    parser.add_argument("--user", default="yuanyou")
    parser.add_argument("--password-env", default="JETSON_PASS")
    parser.add_argument("--no-open-browser", action="store_true")
    args = parser.parse_args()

    password = os.environ.get(args.password_env)
    if password is None:
        password = getpass.getpass(f"{args.user}@{args.host} password: ")

    restart_jetson(args.host, args.user, password)
    restart_local_forward(args.host, args.user, password)
    restart_adb(open_browser=not args.no_open_browser)
    print("Open in Quest Browser: http://localhost:8088/quest_webxr_client.html")


if __name__ == "__main__":
    main()

# Package Manifest

## Main Runtime Path

- `start_slow_double.sh`  
  Recommended startup script for the current dual Piper + Quest APK setup.
- `src/oculus_reader/scripts/teleop_double_piper.py`  
  Main VR pose, button, delta IK, workspace, and left/right mapping logic.
- `src/oculus_reader/scripts/piper_control.py`  
  Piper joint command publishing, smoothing, command deadband, and init-pose interpolation.
- `src/oculus_reader/launch/teleop_double_piper.launch`  
  ROS launch entry for dual-arm teleop.
- `src/Piper_ros/can_config.sh`  
  Current yuanyou2 CAN mapping: `left_piper=1-2.1.1:1.0`, `right_piper=1-2.1.3:1.0`.

## Quest APK Artifacts

- `src/oculus_reader/APK/teleop-debug.apk`
- `src/oculus_reader/APK/alvr_client_android.apk`

Install the primary teleop APK with:

```bash
bash scripts/install_quest_apk.sh
```

## Installation Metadata

- `environment.yml`  
  Conda `vt` environment.
- `requirements.txt`  
  Pip dependencies for the existing `vt` environment.
- `scripts/install_jetson_noetic.sh`  
  Jetson/Ubuntu 20.04 helper for APT and Conda setup.
- `docs/DEPENDENCIES.md`  
  Human-readable dependency notes.

## Optional Tools

- `tools/webxr_bridge/`  
  Optional WebXR bridge and relay tools retained from local experiments. The main stable path uses the Quest APK and `start_slow_double.sh`.

## Upstream Reference

- `docs/UPSTREAM_QUESTVR_README.md`  
  Original upstream README retained for reference. It may contain upstream absolute paths that are not used by the packaged runtime path.
- `src/Piper_ros/LICENSE`  
  Upstream Piper ROS license file.
- `NOTICE.md`  
  Third-party notice for the integrated package.

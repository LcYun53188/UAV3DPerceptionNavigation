# Vendor patch manifest

The top-level repository pins every patched submodule to an upstream commit
that is available from the URL in `.gitmodules`. Local changes are stored only
in this directory and are applied by `scripts/apply_vendor_patches.sh`.

`sources.json` is the machine-readable source of pinned vendor commits and
patch paths. The preparation scripts verify repository identity, HEAD, an empty
staging area, nested gitlinks, exact patched content (including generated files),
and unexpected non-ignored files. All selected repositories are checked before
any patch is applied. Ignored build products are not treated as source changes.
Algorithm dependency hashes are checked against `algorithm_versions.json`.

```bash
./scripts/apply_vendor_patches.sh --check
python3 scripts/vendor_patches.py --ego --check
python3 scripts/test_vendor_git.py
```

`--check` is read-only and requires all patches to be applied. Normal application
can restore missing patch-added regular files only when every remaining source
file matches the patch. Existing differing files are never overwritten. Staged
changes must be reviewed and unstaged before preparation.

| Repository | Upstream base | Patch |
| --- | --- | --- |
| `src/livox_ros_driver2` | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` | `livox_ros_driver2.patch` |
| `src/FAST_LIO_ROS2` | `2fffc570a25d0df172720bac034fbdb6a13d2162` | `fast_lio_ros2.patch` |
| `third_party/Livox-SDK2` | `f5d9375f84efe2b15bc0a052d3e18482ed13adf4` | `livox_sdk2.patch` |
| `src/isaac_ros_nvblox` | `6362295e581ef243773c8a348ac46711e4a1fca4` | `isaac_ros_nvblox.patch` |
| `src/magic_enum` | `9f19f78a7d726af84761ecd6d8414613507a95e6` | `magic_enum.patch` |
| `src/isaac_ros_nitros` | `a22f10d4918662c485b0a1323e2fe1d8c21407a9` | `isaac_ros_nitros.patch` |
| `src/negotiated` | `eac198b55dcd052af5988f0f174902913c5f20e7` | `negotiated.patch` |
| `src/isaac_ros_nvblox/nvblox_ros/nvblox_core` | `3f42b210df9ad7a2099f00fcf324049d97342cb0` | `nvblox_core.patch` |

`isaac_ros_common`, `isaac_ros_visual_slam`, and FAST-LIO's nested `ikd-Tree`
currently have no local source changes and therefore need no vendor patch.

## Reconstruct the vendor tree

From a clean top-level checkout:

```bash
git submodule sync --recursive
git submodule update --init --recursive
./scripts/apply_vendor_patches.sh
./scripts/install_vendor_lfs_assets.sh
```

Running the apply script again is safe: it recognizes patches that are already
present. Return to the reproducible upstream bases with:

```bash
./scripts/apply_vendor_patches.sh --reverse
git submodule update --init --recursive
```

Reverse mode removes patch-added files as well as tracked changes. Do not use
recursive forced checkout as routine recovery: inspect and preserve local work
first. A differing HEAD is rejected rather than automatically reset.

LFS preparation covers the top-level repository and all initialized recursive
submodules, with repository-local configuration only. `--check` verifies file
presence, recorded size, pointer hydration, and the host GXF ELF architecture;
it does not perform a full content hash audit. On Jetson, set
`GXF_LFS_VARIANT=gxf_jetpack70`. Install `git-lfs` before running the script.

## Updating a patch

Develop and test patch changes in a separate worktree or temporary checkout
based on the recorded upstream commit. Generate the complete difference from
that base with `git diff <base> --full-index --binary`; include added files using
a temporary index. Do not include nested gitlinks in their parent's patch.
Keep the normal checkout at the upstream HEAD with the patch applied, not at a
local vendor commit. Update the corresponding algorithm patch SHA-256 whenever
an algorithm dependency patch changes, then run both vendor checks above.

Use `tracked_sources.json` for third-party trees copied into the main repository.
Unknown upstream revisions are explicitly marked for investigation; the recorded
workspace commit/tree provides an immutable reference, not an upstream version.

FAST-LIO runtime parameters belong to `src/uav_bringup/config/mid360.yaml`.
`nav_stack.launch.py` defaults to that package; use `lio_config_package:=fast_lio`
for upstream configs, or an absolute `lio_config_file`. The standalone MID360
script defaults to the source bringup config directory and accepts
`FASTLIO_CONFIG_PATH` or an absolute `--config-file`.

Python dependencies are installed into `.venv` from
`requirements/algorithm-sim.txt` (including `lark==1.3.1`). `.deps/` is reserved
for generated SDK builds and prepared EGO sources, and is not prepended to
`PYTHONPATH`. Existing ignored Lark copies may remain locally but are not used.


Isaac ROS image pipeline 不再附加排除包的本地补丁；完整保留其上游源码。nvblox 补丁仅保留源码修改，不再创建构建忽略标记。

## EGO algorithm simulation dependency

EGO is kept in ignored `.deps/ego_planner_src`, not copied into the source tree or
added as a floating submodule. Run `scripts/prepare_ego_vendor.sh` to fetch official
`ego-planner-swarm` commit `23a8d5a191711dd65633df689bd00f55d4dea8f9` and apply
`ego_planner.patch`. Repeated preparation verifies both the base and exact diff.

The patch replaces the original sensor-integrating `GridMap` with frozen-query
callbacks, adds conservative A* edge checks, fixes A* index rounding/initialization
and allocation cleanup, and fixes the last three spline controls for a stopped
terminal state. The original EGO A* and rebound B-spline objective/optimizer remain.
Only required libraries are compiled by `src/ego_planner_vendor`; upstream FSM,
trajectory server, simulator and control publishers are not built into this route.
The source carries GPL-3.0; upstream notices and LICENSE are retained/installed.

`src/uav_bringup/config/algorithm_versions.json` records dependency commits and
patch SHA-256 values for map compatibility. Python wheels are pinned separately in
`requirements/algorithm-sim.txt`; generated maps and dependency sources stay ignored.

`isaac_ros_visual_slam.patch` retains upstream `04bf49a2daf7710d2ba2390d1772435a1baeb48d`
and converts sequencer jitter parameters from milliseconds to nanoseconds. Rebuild with
`scripts/build_vio_node.sh`; reviewed binary/source hashes are frozen in
`simulation/px4/vio/pose_contract.json`. It does not change pose covariance math.

# Installing taskweave

taskweave 0.1.0 targets **ROS 2 Jazzy** on Ubuntu 24.04. This guide goes from a clean machine to a running example
cell, without a robot or a simulator. Everything below also works in a container:

```bash
docker run -it --rm ros:jazzy-ros-base bash
```

(The base image has `rosdep` but not `colcon`, and no `git`; step 1 installs both.)

## 1. Prerequisites

```bash
sudo apt-get update          # no sudo inside the container
sudo apt-get install -y git python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null || true   # once per machine; the ros: image has done it already
rosdep update --rosdistro jazzy
```

## 2. Get the code at a release tag

```bash
mkdir -p ~/tw_ws/src && cd ~/tw_ws/src
git clone --branch v0.1.0 --depth 1 https://github.com/flrzep/taskweave.git
cd ~/tw_ws
```

The workspace holds six packages: `taskweave_interfaces`, `taskweave_core`, `taskweave_skills`, `taskweave_hmi`,
`taskweave_robots` and `taskweave_bringup`. `taskweave_bringup` pulls in Nav2 and MoveIt through rosdep, so the
install is a few GB; that is expected.

## 3. Dependencies and build

```bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build
source install/setup.bash
```

`colcon build` takes a few minutes. Every new shell needs the two `source` lines
(`/opt/ros/jazzy/setup.bash`, then `~/tw_ws/install/setup.bash`).

## 4. Run the tests

The ROS tests start real nodes, so give them a domain of their own and keep discovery on this machine:

```bash
export ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export TASKWEAVE_STRICT_AGNOSTIC=zero      # robot-agnostic code check: any finding fails
colcon test --executor sequential --event-handlers console_direct+
colcon test-result --verbose
```

All suites must pass (`0 errors, 0 failures`). `--executor sequential` matters: the ROS tests of two packages must
not run on one domain at the same time.

## 5. Validate the example cell

`docs/examples/cell` is a complete cell pack with a neutral two-arm test robot (`duo_bot`); `docs/examples/missions`
holds example missions. Make a working copy with a mission in it (so the clone stays untouched), and check it. Checking
needs no ROS nodes.

```bash
cp -r ~/tw_ws/src/taskweave/docs/examples/cell ~/example_cell
cp -r ~/tw_ws/src/taskweave/docs/examples/missions ~/example_cell/missions
taskweave check ~/example_cell
```

It lists every file as `ok`, `warn` or an error; exit code 0 means the pack is valid. The two `warn` lines about a
missing elbow description of `duo_bot` are expected (the neutral test robot has none). (Add the PATH line in every shell
that uses it; `python3 -m taskweave_core.cli` (after sourcing the workspace) runs the same commands; `ros2: command not
found` means `~/tw_ws/install/setup.bash` is not sourced. The checkout's own copy,
`taskweave check ~/tw_ws/src/taskweave/docs/examples/cell`, works too.)

## 6. Start taskweave with the example cell

```bash
export ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch taskweave_bringup taskweave_bringup.launch.py     config_dir:=$HOME/example_cell robot:=duo_bot skills:=wait use_sim_time:=false
```

This starts the task handler, a skill server with the `wait` skill, the operator and teach nodes, and the web HMI at
http://127.0.0.1:8090 (`hmi_port:=` changes the port; `hmi_host:=` the bind address, which has no login, so use a
trusted network address only). Without a robot, a simulator or controllers, what works is:

- the HMI opens; the **Missions** tab edits and validates missions, the **Robot** tab shows the robot card;
- a mission made of `wait` steps runs and shows its status;
- `taskweave status`, `describe`, `skills`, the speed override and pause / resume / abort work.

In a second terminal (sourced the same way, same `ROS_DOMAIN_ID`, the PATH line added):

```bash
taskweave skills
taskweave run wait_demo          # two waits, three seconds
taskweave status
```

The node log of the launch prints one `ERROR ... not served` for any skill whose `skills/<name>.yaml` the cell does
not have or whose `requires` the robot cannot satisfy; with `skills:=wait` there is none.

Skills that move something (`move_joints`, `navigate_to`, ...) need the controllers, joint states, TF and (for
navigation) a map and a clock of the robot or its simulator. Add them to `skills:=` once those exist, with their
`skills/<name>.yaml` in the cell (templates in `docs/examples/skills/`). Nav2 and MoveIt start with
`nav2:=true map:=/path/map.yaml` and `moveit:=true` (all arguments: header of
`taskweave_bringup/launch/taskweave_bringup.launch.py`). Stop the launch with Ctrl-C.

## 7. Use your own cell and robot packs

- **Cell pack:** a folder with `pack.yaml`, `cell/`, `objects/`, `missions/`, `skills/` and `robots/<model>/`. Start by
  copying `docs/examples/cell` and edit; the layout and every file format are in
  [configuration.md](configuration.md). Pass it as `config_dir:=/path/to/cell`, and validate it first:
  `taskweave check /path/to/cell`.
- **Robot pack:** a folder `<robot>/` with `pack.yaml`, `profile.yaml`, `embodiment.yaml`, poses and MoveIt files (see
  AGENTS.md, "Add a robot pack"). The bundled packs (`galbot_s1`, `galbot_g1`, `a2d`; internal use only) are installed
  to `share/taskweave_robots/`. Your own packs live in a folder of their own; list it in `TASKWEAVE_ROBOT_PACKS`
  (a path, or several separated by `:`) before launching or checking:

  ```bash
  export TASKWEAVE_ROBOT_PACKS=/path/to/my_robot_packs
  taskweave check /path/to/cell
  ros2 launch taskweave_bringup taskweave_bringup.launch.py config_dir:=/path/to/cell robot:=my_robot
  ```

  A cell pack may also carry the robot inside itself, as `docs/examples/cell/robots/duo_bot` does. A cell file with
  `~override: keys` overrides a robot pack's file key by key.
- **A new cell from nothing:** `taskweave config init /path/to/cell --robot my_robot [--name my_cell]` writes an
  empty cell pack that passes `taskweave check` (the robot pack must be found, see above); the folder must not exist or
  be empty. Fill it with the HMI or by hand. Move a cell between hosts as one file:
  `taskweave config export /path/to/cell cell.tar.gz` and, on the other host,
  `taskweave config import cell.tar.gz /path/to/cell [--force]` (details in [configuration.md](configuration.md)).
- `taskweave config files CELL_DIR` shows where each robot file comes from (cell or pack).

## 8. Upgrade an older cell pack

A cell pack has a `schema_version` in its `pack.yaml`. A newer taskweave rejects an older pack (and an older taskweave
rejects unknown keys of a newer one), so upgrade the pack when you move to a new release:

```bash
taskweave config migrate /path/to/cell --dry-run     # shows what would change, writes nothing
taskweave config migrate /path/to/cell               # does it; safe to run again
taskweave config migrate-skills /path/to/cell        # only for packs older than schema 3 (new skill model)
taskweave check /path/to/cell
```

Keep the pack in git so that you can review the diff. `taskweave config slim` turns whole-file overrides of robot
pack files into per-key overrides. See [pack_switchover.md](pack_switchover.md) for moving from a plain config
directory to cell pack + robot packs.

## 9. Container

The repository has a `Dockerfile` (nothing is published; build it yourself). It starts from `ros:jazzy-ros-base`,
installs the dependencies with rosdep, builds all six packages with colcon from your checkout and sets up an
entrypoint that sources `/opt/ros/jazzy` and the install, so `taskweave` and `ros2` work for any command. The image is
several GB (Nav2 and MoveIt come with `taskweave_bringup`).

```bash
docker build -t taskweave .
docker run --rm taskweave                                   # taskweave --help
docker run --rm taskweave ros2 pkg list | grep taskweave
docker run --rm -v "$PWD/docs/examples/cell:/cell:ro" taskweave taskweave check /cell
```

To run taskweave itself, mount a cell pack at `/cell` and use the host network so that ROS 2 discovery and the HMI
(port 8090) work; set the `ROS_DOMAIN_ID` of the robot you talk to. `docker/compose.example.yaml` does this for the
example cell (`docker compose -f docker/compose.example.yaml up`). Robot packs of your own: mount the folder and set
`TASKWEAVE_ROBOT_PACKS` (section 7).

## 10. Pack versions

A cell pack's or robot pack's `pack.yaml` may carry its own `version` (any string, e.g. `1.4.0`) and the taskweave
versions it works with, a [PEP 440](https://peps.python.org/pep-0440/#version-specifiers) specifier:

```yaml
version: 1.4.0
requires:
  taskweave: ">=0.1,<0.2"
```

`taskweave check` and every load refuse a pack that does not admit the installed taskweave, with a message naming the
pack, its version, the requirement and the taskweave version. Without `requires` the pack loads with any taskweave. A
malformed specifier is an error as well. The bundled robot packs and the example cell require the minor range of the
release they ship with.

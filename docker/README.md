# Simulation image (`:sim`)

Gazebo Harmonic + ArduSub SITL (Polaris fork) on top of the same toolchain as
`project-polaris-docker:dev-x64`. The Jetson image is **not** built here — it lives in
`aris-space/project-polaris-docker` and is untouched by anything in this directory.

| File | Purpose |
|---|---|
| `Dockerfile.sim` | The image. Base `ros:humble-ros-base` (multi-arch), never L4T. |
| `build_sim.sh` | Local build, tags `polaris:sim`. |
| `colcon_sim_defaults.yaml` | Makes a plain `colcon build` inside the sim container include `src/simulation/*`. |
| `Dockerfile.sim.dockerignore` | Context filter for BuildKit/buildx builds from the repo root. |

## Build locally

```bash
docker/build_sim.sh              # TAG=polaris:sim by default
```

**The first build takes 30–60 minutes and several GB** (≈10 GB image), dominated by the
ArduSub SITL build and `ardupilot_gazebo`. Later builds are cached; workspace edits never
invalidate the cache because the workspace is bind-mounted at runtime, not copied in.

Bumping ArduSub: change `ARDUSUB_SHA` in `Dockerfile.sim` (or pass
`--build-arg ARDUSUB_SHA=<sha>`). Only branches carrying the Polaris mixer
(`polaris-pressure-filter`, `polaris-custom-frame`) are valid — `master` flies a
3-thruster SIMPLEROV. At boot ArduSub must print `Frame: CUSTOM`.

## Use it

Open the repo in VS Code and pick **Dev Containers: Reopen in Container → Polaris SIM**
(`.devcontainer/sim/devcontainer.json`), or by hand:

```bash
docker run -it --rm --network=host -v "$PWD":/ros2_ws polaris:sim \
  bash -lc './entrypoint.sh bash'
# inside:
ros2 launch orca_sim_bringup sim_launch.py
```

See `src/simulation/README.md` for what the simulation does and does not model.

## Publishing a multi-arch image (manual)

The self-hosted runners are down, so publishing is manual. Whoever has a fast machine of
each architecture can push, so the rest of the team can pull instead of building:

```bash
# on an arm64 machine
docker buildx build --platform linux/arm64 -f docker/Dockerfile.sim \
  -t ghcr.io/aris-space/project-polaris:internal-sim-arm64 --push .
# on an amd64 machine
docker buildx build --platform linux/amd64 -f docker/Dockerfile.sim \
  -t ghcr.io/aris-space/project-polaris:internal-sim-amd64 --push .
# from either
docker buildx imagetools create --tag ghcr.io/aris-space/project-polaris:sim \
  ghcr.io/aris-space/project-polaris:internal-sim-amd64 \
  ghcr.io/aris-space/project-polaris:internal-sim-arm64
```

(These use the repo root as context; `Dockerfile.sim.dockerignore` limits it to
`src/simulation/simulation.repos`.) Then change `"image"` in
`.devcontainer/sim/devcontainer.json` to `ghcr.io/aris-space/project-polaris:sim`.

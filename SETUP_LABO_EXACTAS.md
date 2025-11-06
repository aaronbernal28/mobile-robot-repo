## Docker Rootless (for lab computers)

### Check if docker.sock exists
```bash
ls -l /run/user/$(id -u)/docker.sock
```

### If docker.sock exists, set environment variable (in each terminal)
```bash
export DOCKER_HOST=unix:///run/user/$(id -u)/docker.sock
```

### If docker.sock doesn't exist, start dockerd-rootless first
```bash
dockerd-rootless.sh &
```

Then verify with the `ls -l` command and use the export command.

## Important Notes

- Multiple terminals can enter the same container simultaneously
- The `source install/setup.bash` command must be run in each new terminal
- The `colcon build` command only works from the `/root/ros2_ws` directory
- Files placed in the `volume` folder are accessible from both the host and container
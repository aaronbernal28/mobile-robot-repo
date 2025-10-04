# mobile-robot-repo

This repository contains resources from the course `Robótica Móvil` at the University of Buenos Aires (UBA), taught in the Computer Science Department, 2st semester of 2025. The problems are solved collaboratively with colleagues.

## Setup on Windows

1. Install Docker Desktop
    - Verify CPU virtualization is enabled (mandatory)
    - Enable Memory Integrity in Windows Security
2. Install WSL (Windows Subsystem for Linux)
3. Install a Linux distribution such as Ubuntu (recommended)
    ```bash
    wsl.exe --install <distro>
    ```
4. Clone the repository
    ```bash
    git clone https://github.com/aaronbernal28/mobile-robot-repo.git
    ```
5. Navigate to the repository directory in Windows Terminal: ```\mobile-robot-repo>```
6. Build the Docker image:
    ```bash
    bash start-docker.sh build
    ```
7. Start a container:
    ```bash
    bash start-docker.sh start
    ```
8. Open the container:
    ```bash
    bash start-docker.sh open
    ```
9. Inside the Linux environment, navigate to the workspace:
    ```bash
    cd ros2_ws/
    ```
10. Build and source the workspace:
     ```bash
     colcon build
     source install/setup.bash
     ```

## Typical Usage

```bash
# Start the container
bash start-docker.sh start

# Open the container
bash start-docker.sh open

# Navigate to workspace
cd ~/ros2_ws

# Source the setup
source install/setup.bash
```

## Extra
Open vscode within the container:
`code --no-sandbox --user-data-dir=/path/to/some/directory .`

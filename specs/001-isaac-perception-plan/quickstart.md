# Quickstart: Module 3: The AI-Robot Brain (NVIDIA Isaac)

**Date**: 2025-12-09
**Feature**: [specs/001-isaac-perception-plan/spec.md](specs/001-isaac-perception-plan/spec.md)
**Plan**: [specs/001-isaac-perception-plan/plan.md](specs/001-isaac-perception-plan/plan.md)

This quickstart guide provides the fastest way to get your environment set up and run your first example for Module 3, focusing on NVIDIA Isaac Sim, Isaac ROS, and Nav2 for humanoid robotics.

---

### 1. Prerequisites

Before you begin, ensure you have the following installed and configured on your system:

*   **Operating System**: Ubuntu 20.04 LTS or 22.04 LTS (recommended for ROS 2 and NVIDIA software compatibility).
*   **Hardware**: An NVIDIA GPU (RTX series recommended for optimal performance) with compatible drivers installed, or an NVIDIA Jetson device if you plan to deploy on embedded hardware.
*   **Docker**: Docker Engine and the NVIDIA Container Toolkit (essential for running Isaac Sim and Isaac ROS containers).
*   **Internet Connection**: Required for downloading large Docker images and various software packages.

---

### 2. Install NVIDIA Isaac Sim

Isaac Sim is typically run as a Docker container. The most reliable method for installation is to follow the official NVIDIA documentation.

1.  **Follow the official NVIDIA Isaac Sim documentation** to install and set up Isaac Sim. This process generally involves:
    *   Downloading and installing the NVIDIA Omniverse Launcher.
    *   Installing Omniverse Code and Isaac Sim through the Omniverse Launcher.
    *   Ensuring the NVIDIA Container Toolkit is correctly configured for your Docker installation.
2.  **Verify Installation**: Launch Isaac Sim via the Omniverse Launcher and ensure you can open a basic simulation scene (e.g., the `warehouse.usd` example).

---

### 3. Install ROS 2 Humble and Isaac ROS

Isaac ROS packages are built on top of a ROS 2 distribution. For this module, we use ROS 2 Humble.

1.  **Install ROS 2 Humble**: Follow the official ROS 2 documentation to install ROS 2 Humble Hawksbill on your Ubuntu system. Choose the "Desktop Install" for a complete setup.
    ```bash
    # Example command (refer to official docs for precise instructions):
    sudo apt install ros-humble-desktop
    source /opt/ros/humble/setup.bash
    ```
2.  **Set up Isaac ROS Development Environment**:
    *   Pull the appropriate Isaac ROS Docker images from [NVIDIA NGC](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-ros-dev).
    *   Follow the Isaac ROS documentation (e.g., "Getting Started") to set up your workspace and build the necessary packages (e.g., `isaac_ros_common`, `isaac_ros_vslam`). This often involves using `colcon build`.
3.  **Verify Installation**: Run a simple Isaac ROS example (e.g., one of the provided launch files for `isaac_ros_vslam` with a provided dataset or data streamed from Isaac Sim).

---

### 4. Setting up Nav2

Nav2 is the standard navigation stack for ROS 2 and integrates seamlessly.

1.  **Install Nav2**: Nav2 can typically be installed directly via `apt` if you have ROS 2 Humble installed:
    ```bash
    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
    ```
2.  **Verify Installation**: Run a basic Nav2 tutorial (e.g., the `turtlebot3_navigation` example in simulation) to ensure its core functionality.

---

### 5. Running Your First Humanoid Example (from this module)

Once all prerequisites are installed and verified, you can proceed to run the code examples provided within this book's repository.

1.  **Clone the Repository**: If you haven't already, clone the book's repository:
    ```bash
    git clone https://github.com/your-repo/hackathon-book.git # Replace with actual URL
    cd hackathon-book
    ```
2.  **Navigate to Examples**:
    ```bash
    cd src/isaac/
    # Each chapter will have its own example directory (e.g., isaac_sim_examples, isaac_ros_examples).
    # Refer to the specific chapter's instructions for how to launch its examples.
    ```
3.  **Expected Outcome**: You should successfully launch and observe a simulated humanoid robot operating within Isaac Sim, processing data with Isaac ROS components, and navigating using Nav2, as demonstrated in the module's examples.

This quickstart aims to get you operational with the necessary tools. Refer to the individual chapters within Module 3 for detailed explanations, advanced topics, and specific instructions for each example.

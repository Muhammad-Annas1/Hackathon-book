module.exports = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      link: { type: 'doc', id: 'module-1/chapter-1-core-concepts' },
      items: [
        'module-1/chapter-1-core-concepts',
        'module-1/chapter-2-rclpy-control',
        'module-1/chapter-3-urdf-fundamentals',
        'module-1/chapter-4-joint-control-project',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      link: { type: 'doc', id: 'module-2/chapter-1-gazebo-physics' },
      items: [
        'module-2/chapter-1-gazebo-physics',
        'module-2/chapter-2-unity-digital-twin',
        'module-2/chapter-3-sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      link: { type: 'doc', id: 'module-3-isaac/chapter-1-isaac-sim-fundamentals' },
      items: [
        'module-3-isaac/chapter-1-isaac-sim-fundamentals',
        'module-3-isaac/chapter-2-isaac-ros-vslam-perception',
        'module-3-isaac/chapter-3-nav2-humanoid-planning',
        // Optional chapter, will be uncommented if implemented
        // 'module-3-isaac/chapter-4-isaac-sim-ros-workflow',
      ],
    },
  ],
};
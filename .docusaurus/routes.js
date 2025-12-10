import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '19e'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '22d'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '45c'),
            routes: [
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1/chapter-1-core-concepts',
                component: ComponentCreator('/docs/module-1/chapter-1-core-concepts', '983'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1/chapter-2-rclpy-control',
                component: ComponentCreator('/docs/module-1/chapter-2-rclpy-control', '9e3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1/chapter-3-urdf-fundamentals',
                component: ComponentCreator('/docs/module-1/chapter-3-urdf-fundamentals', '1c9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1/chapter-4-joint-control-project',
                component: ComponentCreator('/docs/module-1/chapter-4-joint-control-project', '492'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2/chapter-1-gazebo-physics',
                component: ComponentCreator('/docs/module-2/chapter-1-gazebo-physics', 'd9d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2/chapter-2-unity-digital-twin',
                component: ComponentCreator('/docs/module-2/chapter-2-unity-digital-twin', 'ed6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2/chapter-3-sensor-simulation',
                component: ComponentCreator('/docs/module-2/chapter-3-sensor-simulation', '7b2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-isaac/chapter-1-isaac-sim-fundamentals',
                component: ComponentCreator('/docs/module-3-isaac/chapter-1-isaac-sim-fundamentals', '83a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-isaac/chapter-2-isaac-ros-vslam-perception',
                component: ComponentCreator('/docs/module-3-isaac/chapter-2-isaac-ros-vslam-perception', '884'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-isaac/chapter-3-nav2-humanoid-planning',
                component: ComponentCreator('/docs/module-3-isaac/chapter-3-nav2-humanoid-planning', '25c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-isaac/chapter-4-isaac-sim-ros-workflow',
                component: ComponentCreator('/docs/module-3-isaac/chapter-4-isaac-sim-ros-workflow', '491'),
                exact: true
              },
              {
                path: '/docs/module-4-vla/chapter-1-voice-to-action',
                component: ComponentCreator('/docs/module-4-vla/chapter-1-voice-to-action', 'c97'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/chapter-2-cognitive-planning',
                component: ComponentCreator('/docs/module-4-vla/chapter-2-cognitive-planning', '649'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/chapter-3-capstone-overview',
                component: ComponentCreator('/docs/module-4-vla/chapter-3-capstone-overview', '692'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];

import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-humanoid-robotics-book/__docusaurus/debug',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/__docusaurus/debug', '8a2'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics-book/__docusaurus/debug/config',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/__docusaurus/debug/config', '836'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics-book/__docusaurus/debug/content',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/__docusaurus/debug/content', '1da'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics-book/__docusaurus/debug/globalData',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/__docusaurus/debug/globalData', '9a4'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics-book/__docusaurus/debug/metadata',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/__docusaurus/debug/metadata', 'e73'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics-book/__docusaurus/debug/registry',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/__docusaurus/debug/registry', '4e1'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics-book/__docusaurus/debug/routes',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/__docusaurus/debug/routes', 'bf0'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics-book/auth/Login',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/auth/Login', '041'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics-book/auth/Signup',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/auth/Signup', '057'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics-book/docs',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs', '434'),
    routes: [
      {
        path: '/physical-ai-humanoid-robotics-book/docs',
        component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs', '3f2'),
        routes: [
          {
            path: '/physical-ai-humanoid-robotics-book/docs',
            component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs', '451'),
            routes: [
              {
                path: '/physical-ai-humanoid-robotics-book/docs/final-capstone',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/final-capstone', '878'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/hardware-requirements',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/hardware-requirements', '006'),
                exact: true
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module1_ros2',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module1_ros2', '639'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module2_digital_twin',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module2_digital_twin', '53b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module3_ai_robot_brain',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module3_ai_robot_brain', '32f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/module4_vision_language_action',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/module4_vision_language_action', 'b0d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/rclpy-example-controllers',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/rclpy-example-controllers', '744'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/ros2-nodes',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/ros2-nodes', '342'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/topics-services',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/topics-services', '878'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/urdf-for-humanoids',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/urdf-for-humanoids', '34e'),
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
    path: '/physical-ai-humanoid-robotics-book/',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/', 'df5'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];

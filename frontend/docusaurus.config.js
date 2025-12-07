/** @type {import('@docusaurus/types').Config} */
const config = {
  // Keep ignore so build NEVER fails on broken links
  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'ignore',

  title: 'Physical AI & Humanoid Robotics',
  tagline: 'An AI-Native Textbook',
  url: 'https://abdusubhan11.github.io',
  baseUrl: '/physical-ai-humanoid-robotics-book/',
  favicon: 'img/favicon.ico',
  organizationName: 'abdusubhan11',
  projectName: 'physical-ai-humanoid-robotics-book',

  presets: [
    [
      'classic',
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl:
            'https://github.com/abdusubhan11/physical-ai-humanoid-robotics-book/tree/main/frontend/',
        },

        // ❌ REMOVE BLOG COMPLETELY
        blog: false,

        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig: ({
    image: './img/logo.jpg',
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: './img/logo.jpg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },

        // ❌ REMOVED BLOG LINK
        // { to: '/blog', label: 'Blog', position: 'left' },

       
      ],
    },
    

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Community',
          items: [
            {
              label: 'Discord',
              href: 'https://discord.gg/your-discord-link',
            },
            {
              label: 'Twitter',
              href: 'https://twitter.com/your-twitter-handle',
            },
          ],
        },
       
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
    },

    prism: {},
  }),
};

module.exports = config;

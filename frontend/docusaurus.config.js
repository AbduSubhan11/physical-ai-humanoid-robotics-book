
/** @type {import('@docusaurus/types').Config} */
const config = {
   onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'ignore',
  
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'An AI-Native Textbook',
  url: 'https://abdusubhan11.github.io',
  
  baseUrl: '/physical-ai-humanoid-robotics-book/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
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
        blog: {
          showReadingTime: true,
          editUrl:
            'https://github.com/abdusubhan11/physical-ai-humanoid-robotics-book/tree/main/frontend/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig: ({
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        { to: '/blog', label: 'Blog', position: 'left' },
        {
          href: 'https://github.com/abdusubhan11/physical-ai-humanoid-robotics-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        // {
        //   title: 'Docs',
        //   items: [
        //     {
        //       label: 'Book',
        //       to: '/docs/intro',
        //     },
        //   ],
        // },
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
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/abdusubhan11/physical-ai-humanoid-robotics-book',
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

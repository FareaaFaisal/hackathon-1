import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Book on Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Production URL and base URL for GitHub Pages
  url: 'https://fareaafaisal.github.io',
  baseUrl: '/hackathon-1/',

  // GitHub pages deployment config
  organizationName: 'FareaaFaisal', // GitHub username
  projectName: 'hackathon-1',       // Repo name

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Internationalization
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Presets
  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Link to edit pages on GitHub
          editUrl: 'https://github.com/FareaaFaisal/hackathon-1/edit/main/',
        },
        blog: {
          showReadingTime: true,
          editUrl: 'https://github.com/FareaaFaisal/hackathon-1/edit/main/',
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  // Theme configuration
  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Humanoid Robotics',
      logo: {
        alt: 'Humanoid Robotics Book Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Modules',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/FareaaFaisal/hackathon-1',
          label: 'GitHub Repo',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Modules',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
            {
              label: 'Twitter',
              href: 'https://x.com/docusaurus',
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
              label: 'GitHub Repo',
              href: 'https://github.com/FareaaFaisal/hackathon-1',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
    algolia: {
      appId: 'YOUR_APP_ID',           // Optional, set if you have Algolia search
      apiKey: 'YOUR_SEARCH_API_KEY',
      indexName: 'YOUR_INDEX_NAME',
      contextualSearch: true,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;

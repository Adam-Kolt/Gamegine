import { createMDX } from 'fumadocs-mdx/next';

const withMDX = createMDX({
  // customise config path if needed
  // configPath: 'source.config.ts',
});

const nextConfig = {
  reactStrictMode: true,
};

export default withMDX(nextConfig);

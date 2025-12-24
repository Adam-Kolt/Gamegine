# Gamegine Docs Site

This directory contains the Fumadocs-powered documentation site for Gamegine.

## Commands

```bash
npm run dev    # start the docs at http://localhost:3000/docs
npm run build  # produce a production build in .next/
npm run start  # serve the production build
```

The docs content lives under `content/docs` and is written in MDX. To add a new top-level
page, create an `.mdx` file and register it in `content/docs/meta.json` so it appears in the
navigation tree.

Search is backed by Orama through the `/api/search` route. `npm run build` or `npm run dev`
regenerates the `.source` folder that Fumadocs uses for static generation.

## Project layout

- `app/` – Next.js routes and layouts (including the catch-all docs route).
- `lib/` – Shared Fumadocs helpers (`source` loader, layout configuration).
- `mdx-components.tsx` – Components passed to MDX files.
- `public/` – Static assets (icons, logos, favicons).
- `source.config.ts` – Fumadocs MDX collection definition.

## Deployment

Run `npm run build` to create a production build. The output can be deployed to any
Next.js-compatible hosting provider (Vercel, Netlify, Docker, etc.).

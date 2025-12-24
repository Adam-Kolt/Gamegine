# Gamegine

![Gamegine logo](https://github.com/Adam-Kolt/Gamegine/blob/main/docs-site/public/gamegine-light.png?raw=true)

Gamegine is an FRC game representation, analysis, and simulation engine for post-kickoff
strategy evaluation and optimisation alongside ML agent training experiments.

## Documentation

The documentation is now powered by [Fumadocs](https://fumadocs.dev/).

```bash
cd docs-site
npm install           # first-time setup only
npm run dev           # start the local docs server
npm run build         # create a production build
```

Open http://localhost:3000/docs to browse the docs site while developing. Content lives in
`docs-site/content/docs` and is authored in MDX.

## Repository layout

- `gamegine/` – core representation, analysis, rendering, and simulation packages.
- `examples/` – runnable demos for meshing, pathfinding, and trajectory generation.
- `docs-site/` – Next.js/Fumadocs documentation app.

Gamegine relies on the strongly typed NCIM unit system to model geometry, time, and robot
physics. See the documentation for an overview of conventions and workflows.

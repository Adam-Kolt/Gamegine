import Link from 'next/link';

const featureCards = [
  {
    title: 'Represent the field',
    description:
      'Describe obstacles, interactables, and robots with NCIM-aware geometry that powers every subsystem.',
    href: '/docs/representation',
    action: 'Explore representation',
  },
  {
    title: 'Plan resilient motion',
    description:
      'Generate meshes, run A*, inflate safe corridors, and configure Sleipnir-backed trajectory solvers.',
    href: '/docs/analysis',
    action: 'Follow the workflow',
  },
  {
    title: 'Simulate and iterate',
    description:
      'Visualise layers, track robot state, and iterate on strategy before you hit the practice field.',
    href: '/docs/simulation',
    action: 'See simulation tools',
  },
];

const resourceLinks = [
  { label: 'Quick start overview', href: '/docs/overview' },
  { label: 'Examples library', href: '/docs/examples' },
  { label: 'NCIM units', href: '/docs/ncim' },
  { label: 'Reference data', href: '/docs/reference' },
];

const quickstartSnippet = `from gamegine.representation.game import Game
from gamegine.analysis.pathfinding import findPath
from gamegine.analysis.meshing import VisibilityGraph
from gamegine.render.renderer import Renderer
from gamegine.utils.NCIM.ncim import Meter

game = Game("Strategy Lab")
game.set_field_size(Meter(16), Meter(8))

mesh = VisibilityGraph(list(game.get_obstacles()))
path = findPath(mesh, (Meter(2), Meter(2)), (Meter(12), Meter(4)))
trajectory = solve_swerve(path, robot_limits)

renderer = Renderer()
renderer.set_game(game)
renderer.set_render_scale(Meter(0.05))
renderer.init_display()`;

export default function Home() {
  return (
    <main className="bg-brand-surface text-brand-primary">
      <div className="border-b border-brand-border bg-brand-surface/95">
        <div className="mx-auto flex w-full max-w-6xl items-center justify-between px-6 py-6 lg:px-12">
          <Link href="/" className="text-lg font-semibold text-brand-accent">
            Gamegine
          </Link>
          <nav className="flex items-center gap-4 text-sm font-semibold">
            <Link href="/docs" className="transition hover:text-brand-accent">
              Docs
            </Link>
            <Link href="/docs/analysis" className="transition hover:text-brand-accent">
              Workflow
            </Link>
            <Link href="/docs/examples" className="transition hover:text-brand-accent">
              Examples
            </Link>
            <Link
              href="https://github.com/adamkoltuniuk/Gamegine"
              className="rounded-full border border-brand-border px-3 py-1.5 transition hover:border-brand-accent hover:text-brand-accent"
            >
              GitHub
            </Link>
          </nav>
        </div>
      </div>
      <div className="mx-auto flex max-w-6xl flex-col gap-24 px-6 pb-24 pt-16 lg:px-12">
        <section className="flex flex-col gap-10">
          <div className="flex flex-col gap-6">
            <p className="text-xs font-semibold uppercase tracking-[0.35em] text-brand-primary-80">
              Open-source FRC analysis platform
            </p>
            <h1 className="text-4xl font-medium tracking-tight sm:text-5xl lg:text-6xl">
              Create. Analyze. <span className="text-brand-accent">Win.</span>
            </h1>
            <p className="max-w-2xl text-lg leading-relaxed text-brand-primary-85">
              Gamegine unifies field representation, navigation meshing, guided trajectory optimisation,
              and simulation so you can design match-ready automations with confidence.
            </p>
            <div className="flex flex-col gap-4 sm:flex-row sm:items-center">
              <Link
                href="/docs"
                className="inline-flex items-center justify-center rounded-full bg-brand-accent px-6 py-3 text-sm font-semibold text-white shadow-lg transition hover:bg-[#173ab3]"
              >
                Open the documentation
              </Link>
              <Link
                href="https://github.com/adamkoltuniuk/Gamegine"
                className="inline-flex items-center justify-center rounded-full border border-brand-border px-6 py-3 text-sm font-semibold text-brand-primary transition hover:border-brand-accent hover:text-brand-accent"
              >
                View on GitHub
              </Link>
            </div>
          </div>

          <div className="overflow-hidden rounded-3xl border border-brand-border bg-white/90 p-8 shadow-xl backdrop-blur">
            <p className="text-xs font-semibold uppercase tracking-[0.4em] text-brand-primary-70">
              Strategic snapshot
            </p>
            <div className="grid gap-6 pt-6 md:grid-cols-3">
              <div className="flex flex-col gap-3 rounded-2xl bg-brand-muted-soft p-6">
                <span className="text-sm font-semibold">Model</span>
                <p className="text-sm text-brand-primary-85">
                  Capture field layout, scoring zones, and robot geometry with NCIM for seamless unit
                  conversions.
                </p>
              </div>
              <div className="flex flex-col gap-3 rounded-2xl bg-brand-muted-soft p-6">
                <span className="text-sm font-semibold">Plan</span>
                <p className="text-sm text-brand-primary-85">
                  Generate meshes, run A*, and bound safe corridors before handing paths to the optimiser.
                </p>
              </div>
              <div className="flex flex-col gap-3 rounded-2xl bg-brand-muted-soft p-6">
                <span className="text-sm font-semibold">Validate</span>
                <p className="text-sm text-brand-primary-85">
                  Render trajectories, inspect velocities, and iterate until your approach is battle-tested.
                </p>
              </div>
            </div>
          </div>
        </section>

        <section className="grid gap-8 md:grid-cols-3">
          {featureCards.map((card) => (
            <div
              key={card.title}
              className="group flex h-full flex-col gap-4 rounded-3xl border border-brand-border bg-white p-8 shadow-lg transition hover:-translate-y-1 hover:shadow-2xl"
            >
              <span className="text-sm font-semibold uppercase tracking-wide text-brand-primary-70">
                {card.title}
              </span>
              <p className="flex-1 text-sm leading-relaxed text-brand-primary-85">{card.description}</p>
              <Link
                href={card.href}
                className="inline-flex items-center text-sm font-semibold text-brand-accent transition group-hover:gap-2"
              >
                {card.action}
                <span aria-hidden>→</span>
              </Link>
            </div>
          ))}
        </section>

        <section className="grid gap-10 rounded-3xl border border-brand-border bg-white p-10 shadow-lg md:grid-cols-[1.1fr_0.9fr]">
          <div className="flex flex-col gap-6">
            <h2 className="text-2xl font-semibold">From CAD sketch to match-ready automation</h2>
            <p className="text-sm leading-relaxed text-brand-primary-85">
              Gamegine keeps geometry, physics, and optimisation aligned. Follow the step-by-step workflow or
              jump straight into the examples directory to accelerate prototyping.
            </p>
            <div className="grid gap-3 text-sm text-brand-primary-90">
              {resourceLinks.map((resource) => (
                <Link
                  key={resource.label}
                  href={resource.href}
                  className="inline-flex items-center justify-between rounded-2xl border border-brand-border px-4 py-3 transition hover:border-brand-accent hover:text-brand-accent"
                >
                  <span>{resource.label}</span>
                  <span aria-hidden>↗</span>
                </Link>
              ))}
            </div>
            <p className="text-xs text-brand-primary-70">
              *Winning matches still requires human scouting and drive practice—Gamegine just keeps the strategy
              ironclad.
            </p>
          </div>
          <div className="flex flex-col overflow-hidden rounded-2xl border border-brand-border bg-brand-muted-lighter">
            <div className="rounded-t-2xl bg-brand-accent-strong p-4 text-xs font-semibold uppercase tracking-[0.35em] text-white/90">
              Quick start
            </div>
            <pre className="flex-1 overflow-auto break-words px-6 py-6 text-sm leading-6 text-brand-primary">
{quickstartSnippet}
            </pre>
            <div className="px-6 pb-6 text-xs text-brand-primary-75">
              Stitch the representation, analysis, and rendering layers together to iterate safely before the
              practice field.
            </div>
          </div>
        </section>

        <section className="mb-8 flex flex-col gap-6 rounded-3xl border border-brand-border bg-white px-8 py-10 text-brand-primary shadow-xl">
          <div className="flex flex-col gap-3 md:flex-row md:items-center md:justify-between">
            <div className="max-w-2xl">
              <h2 className="text-2xl font-semibold">Ready to build match-winning automations?</h2>
              <p className="text-sm text-brand-primary-85">
                Dive into subsystem guides, browse curated examples, or open a discussion on GitHub to share ideas
                with the community.
              </p>
            </div>
            <div className="flex flex-wrap gap-3">
              <Link
                href="/docs/examples"
                className="inline-flex items-center rounded-full bg-brand-accent px-5 py-3 text-sm font-semibold text-white transition hover:bg-[#173ab3]"
              >
                Browse examples
              </Link>
              <Link
                href="https://github.com/adamkoltuniuk/Gamegine/issues"
                className="inline-flex items-center rounded-full border border-brand-accent px-5 py-3 text-sm font-semibold text-brand-accent transition hover:bg-brand-accent hover:text-white"
              >
                Contribute ideas
              </Link>
            </div>
          </div>
        </section>
      </div>
    </main>
  );
}

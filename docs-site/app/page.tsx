import Link from 'next/link';

export default function Home() {
  return (
    <main className="min-h-screen bg-black text-white selection:bg-purple-500/30">
      {/* Background Gradients */}
      <div className="fixed inset-0 z-0 pointer-events-none">
        <div className="absolute top-[-10%] left-[-10%] w-[40%] h-[40%] rounded-full bg-purple-900/20 blur-[120px]" />
        <div className="absolute bottom-[-10%] right-[-10%] w-[40%] h-[40%] rounded-full bg-blue-900/20 blur-[120px]" />
      </div>

      {/* Nav */}
      <nav className="relative z-10 border-b border-white/10 bg-black/50 backdrop-blur-md">
        <div className="mx-auto flex h-16 max-w-7xl items-center justify-between px-6">
          <div className="flex items-center gap-2">
            <div className="h-6 w-6 rounded bg-gradient-to-br from-purple-500 to-blue-500" />
            <span className="font-bold tracking-tight">Gamegine</span>
          </div>
          <div className="flex items-center gap-6 text-sm font-medium text-zinc-400">
            <Link href="/docs" className="hover:text-white transition-colors">Documentation</Link>
            <Link href="/docs/api" className="hover:text-white transition-colors">API Reference</Link>
            <Link href="https://github.com/adamkoltuniuk/Gamegine" className="hover:text-white transition-colors">GitHub</Link>
          </div>
        </div>
      </nav>

      {/* Hero */}
      <section className="relative z-10 mx-auto mt-24 max-w-7xl px-6 text-center">
        <div className="inline-flex items-center rounded-full border border-purple-500/30 bg-purple-500/10 px-3 py-1 text-xs font-medium text-purple-300 mb-8 backdrop-blur-sm">
          <span>v2.0 Beta Now Available</span>
        </div>

        <h1 className="mx-auto max-w-4xl text-5xl font-bold tracking-tight sm:text-7xl">
          Model reality. <br />
          <span className="bg-gradient-to-r from-purple-400 via-blue-400 to-purple-400 bg-clip-text text-transparent bg-[length:200%_auto] animate-gradient">
            Win the match.
          </span>
        </h1>

        <p className="mx-auto mt-8 max-w-2xl text-lg text-zinc-400">
          The comprehensive robotics simulation framework.
          Deterministic physics, NCIM-safe units, and production-grade trajectory optimization
          built for the First Robotics Competition.
        </p>

        <div className="mt-10 flex items-center justify-center gap-4">
          <Link
            href="/docs/getting-started"
            className="rounded-full bg-white px-8 py-3 text-sm font-semibold text-black transition hover:bg-zinc-200"
          >
            Get Started
          </Link>
          <Link
            href="/docs"
            className="rounded-full border border-white/10 bg-white/5 px-8 py-3 text-sm font-semibold text-white transition hover:bg-white/10"
          >
            Read the Philosophy
          </Link>
        </div>
      </section>

      {/* Feature Grid */}
      <section className="relative z-10 mx-auto mt-32 max-w-7xl px-6 pb-24">
        <div className="grid gap-8 md:grid-cols-3">
          {[
            {
              title: "Deterministic Simulation",
              desc: "Run thousands of match-scenarios in seconds. Our physics engine guarantees reproducible results for rock-solid autonomous routines.",
              gradient: "from-pink-500/20 to-purple-500/20"
            },
            {
              title: "Type-Safe Units",
              desc: "Never crash a robot because of a unit conversion error again. NCIM enforces dimensional correctness at compile time.",
              gradient: "from-blue-500/20 to-cyan-500/20"
            },
            {
              title: "Visual Intelligence",
              desc: "Rich, layered rendering system built on arcade. Visualize pathfinding nodes, trajectories, and sensor data in real-time.",
              gradient: "from-emerald-500/20 to-teal-500/20"
            }
          ].map((feature, i) => (
            <div key={i} className="group relative overflow-hidden rounded-3xl border border-white/10 bg-white/5 p-8 transition-colors hover:bg-white/10">
              <div className={`absolute inset-0 bg-gradient-to-br ${feature.gradient} opacity-0 transition-opacity group-hover:opacity-100`} />
              <div className="relative z-10">
                <h3 className="text-xl font-bold text-white mb-3">{feature.title}</h3>
                <p className="text-zinc-400 leading-relaxed">
                  {feature.desc}
                </p>
              </div>
            </div>
          ))}
        </div>
      </section>

      {/* Code Snippet Section */}
      <section className="relative z-10 border-t border-white/10 bg-white/5 py-24">
        <div className="mx-auto max-w-7xl px-6">
          <div className="grid gap-12 lg:grid-cols-2 lg:items-center">
            <div>
              <h2 className="text-3xl font-bold tracking-tight text-white sm:text-4xl">
                Code that speaks your language.
              </h2>
              <p className="mt-4 text-lg text-zinc-400">
                Gamegine is designed to be expressive. Define your world using natural concepts,
                not just matrices and arrays.
              </p>

              <div className="mt-8 grid gap-4">
                {[
                  "Define the game arena with precise geometry",
                  "Configure swerve kinematics with real mass & friction",
                  "Generate spline trajectories with dynamic constraints",
                  "Simulate interactions with field elements"
                ].map((item, i) => (
                  <div key={i} className="flex items-center gap-3">
                    <div className="flex h-6 w-6 items-center justify-center rounded-full bg-purple-500/20 text-purple-400">
                      <svg className="h-3.5 w-3.5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={3} d="M5 13l4 4L19 7" />
                      </svg>
                    </div>
                    <span className="text-zinc-300">{item}</span>
                  </div>
                ))}
              </div>
            </div>

            <div className="relative rounded-2xl border border-white/10 bg-black/50 p-6 shadow-2xl backdrop-blur-xl">
              <div className="flex items-center gap-2 mb-4">
                <div className="h-3 w-3 rounded-full bg-red-500/50" />
                <div className="h-3 w-3 rounded-full bg-yellow-500/50" />
                <div className="h-3 w-3 rounded-full bg-green-500/50" />
              </div>
              <pre className="overflow-x-auto text-sm font-mono leading-relaxed text-zinc-300">
                {`# Create the Arena
game = Game("2025 Reefscape")
game.set_field_size(Meter(16.54), Meter(8.21))

# Initialize Physics
robot = SwerveRobot("HeroBot", mass=Kilogram(50))
physics = PhysicsEngine(game)

# Generate Trajectory
path = findPath(nav_mesh, start_pose, target_reef)
traj = physics.generate_trajectory(
    robot, 
    path, 
    constraints={
        "max_vel": Meter(4.5) / Second(1),
        "max_accel": Meter(3.0) / Second(2)
    }
)`}
              </pre>
            </div>
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="border-t border-white/10 bg-black py-12">
        <div className="mx-auto max-w-7xl px-6 flex flex-col items-center justify-between gap-6 md:flex-row">
          <p className="text-sm text-zinc-500">
            © 2024 Gamegine Contributors. MIT License.
          </p>
          <div className="flex gap-6">
            <Link href="https://github.com/adamkoltuniuk/Gamegine" className="text-zinc-500 hover:text-white transition-colors">
              GitHub
            </Link>
            <Link href="/docs/privacy" className="text-zinc-500 hover:text-white transition-colors">
              Privacy
            </Link>
          </div>
        </div>
      </footer>
    </main>
  );
}

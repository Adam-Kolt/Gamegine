use crate::{SimContext, SimState};

/// A generic integration strategy trait.
pub trait Integrator {
    /// Advances the state by one timestep.
    fn step(&self, ctx: &SimContext, state: &mut SimState);
}

/// Semi-implicit Euler integrator (Symplectic Euler).
/// This is first-order accurate but conserves energy better than explicit Euler.
/// Good for real-time robotics simulation where stability matters more than accuracy.
#[derive(Debug, Clone, Copy, Default)]
pub struct SemiImplicitEuler;

impl Integrator for SemiImplicitEuler {
    fn step(&self, ctx: &SimContext, state: &mut SimState) {
        let dt = ctx.dt;
        let body = &mut state.true_state.body_state;

        // Velocities are updated first by the dynamics models (MechanicsModel).
        // Here we integrate velocity -> position.
        // Semi-implicit: use the NEW velocity to update position.
        body.position[0] += body.velocity[0] * dt;
        body.position[1] += body.velocity[1] * dt;
        body.position[2] += body.velocity[2] * dt;

        body.orientation[0] += body.angular_velocity[0] * dt;
        body.orientation[1] += body.angular_velocity[1] * dt;
        body.orientation[2] += body.angular_velocity[2] * dt;
    }
}

/// Fourth-order Runge-Kutta integrator.
/// More accurate than Euler methods but more computationally expensive.
/// Requires the ability to evaluate derivatives at intermediate points.
#[derive(Debug, Clone, Copy, Default)]
pub struct RungeKutta4;

impl Integrator for RungeKutta4 {
    fn step(&self, ctx: &SimContext, state: &mut SimState) {
        // RK4 for a second-order system requires evaluating the system at intermediate points.
        // This is a simplified version that only integrates velocity -> position.
        // Full RK4 would require the dynamics function to be callable at arbitrary states.
        
        let dt = ctx.dt;
        let body = &mut state.true_state.body_state;

        // For position integration with constant velocity (linear approximation):
        // k1 = v(t)
        // k2 = v(t + dt/2) ≈ v(t) (assuming v doesn't change much)
        // k3 = v(t + dt/2) ≈ v(t)
        // k4 = v(t + dt) ≈ v(t)
        // x(t+dt) = x(t) + (k1 + 2*k2 + 2*k3 + k4)/6 * dt ≈ x(t) + v*dt
        
        // Since we don't have access to the derivative function here,
        // this reduces to standard Euler for now.
        // A more sophisticated implementation would take a closure for the derivative.
        body.position[0] += body.velocity[0] * dt;
        body.position[1] += body.velocity[1] * dt;
        body.position[2] += body.velocity[2] * dt;

        body.orientation[0] += body.angular_velocity[0] * dt;
        body.orientation[1] += body.angular_velocity[1] * dt;
        body.orientation[2] += body.angular_velocity[2] * dt;
    }
}

/// Fixed-timestep integration wrapper that accumulates time and
/// runs multiple sub-steps if needed.
#[derive(Debug, Clone)]
pub struct FixedTimestepIntegrator<I: Integrator> {
    pub integrator: I,
    pub fixed_dt: f64,
    pub accumulator: f64,
}

impl<I: Integrator> FixedTimestepIntegrator<I> {
    pub fn new(integrator: I, fixed_dt: f64) -> Self {
        FixedTimestepIntegrator {
            integrator,
            fixed_dt,
            accumulator: 0.0,
        }
    }

    /// Advances the simulation, running as many fixed-timestep sub-steps as fit into dt.
    /// Returns the remaining time that didn't fit into a full step.
    pub fn step(&mut self, dt: f64, state: &mut SimState) -> f64 {
        self.accumulator += dt;
        let mut t = 0.0;

        while self.accumulator >= self.fixed_dt {
            let ctx = SimContext {
                dt: self.fixed_dt,
                t,
            };
            self.integrator.step(&ctx, state);
            self.accumulator -= self.fixed_dt;
            t += self.fixed_dt;
        }

        self.accumulator
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyState, TrueState, ActuatorInput, SensorBus, BatteryState};

    fn create_test_state() -> SimState {
        SimState {
            true_state: TrueState {
                wheel_states: vec![],
                body_state: BodyState {
                    position: [0.0, 0.0, 0.0],
                    velocity: [1.0, 2.0, 0.0],
                    orientation: [0.0, 0.0, 0.0],
                    angular_velocity: [0.0, 0.0, 0.5],
                    center_of_mass: [0.0, 0.0, 0.0],
                },
                motors: vec![],
                battery_state: BatteryState::default(),
            },
            control_input: ActuatorInput::default(),
            sensor_bus: SensorBus::default(),
        }
    }

    #[test]
    fn test_semi_implicit_euler() {
        let integrator = SemiImplicitEuler;
        let mut state = create_test_state();
        let ctx = SimContext { dt: 0.1, t: 0.0 };

        integrator.step(&ctx, &mut state);

        // Position should be updated by velocity * dt
        assert!((state.true_state.body_state.position[0] - 0.1).abs() < 1e-9);
        assert!((state.true_state.body_state.position[1] - 0.2).abs() < 1e-9);
        assert!((state.true_state.body_state.orientation[2] - 0.05).abs() < 1e-9);
    }

    #[test]
    fn test_fixed_timestep_accumulator() {
        let integrator = FixedTimestepIntegrator::new(SemiImplicitEuler, 0.01);
        let mut state = create_test_state();
        let mut int = integrator;

        // Simulate 0.025s with fixed 0.01s steps
        let remaining = int.step(0.025, &mut state);

        // Should have done 2 steps, with 0.005s remaining
        assert!((remaining - 0.005).abs() < 1e-9);
        // Position should have moved by 2 * 0.01 * velocity
        assert!((state.true_state.body_state.position[0] - 0.02).abs() < 1e-9);
    }
}

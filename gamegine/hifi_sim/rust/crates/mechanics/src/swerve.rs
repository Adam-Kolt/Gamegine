use nalgebra::{Matrix2, Vector2};
use simcore::{MechanicsModel, Model, SimContext, SimState, WheelState};

/// Represents the physical configuration and properties of a swerve drivetrain.
#[derive(Debug, Clone)]
pub struct SwerveDrivetrainConfig {
    /// Positions of each module relative to the robot's center of mass [x, y] in meters.
    pub module_positions: Vec<[f64; 2]>,
    /// Total mass of the robot in kg.
    pub mass: f64,
    /// Moment of inertia about the vertical axis (yaw) in kg*m^2.
    pub moment_of_inertia: f64,
    /// Rotational inertia of a single wheel about its axle in kg*m^2.
    pub wheel_inertia: f64,
    /// Rotational inertia of a single steering mechanism in kg*m^2.
    pub steer_inertia: f64,
}

impl Default for SwerveDrivetrainConfig {
    fn default() -> Self {
        // Default to a square robot with modules at the corners
        let half_side = 0.3; // 0.6m x 0.6m robot
        SwerveDrivetrainConfig {
            module_positions: vec![
                [half_side, half_side],   // Front Left
                [half_side, -half_side],  // Front Right
                [-half_side, half_side],  // Back Left
                [-half_side, -half_side], // Back Right
            ],
            mass: 50.0,             // 50 kg robot
            moment_of_inertia: 5.0, // Approximate for a solid rectangular robot
            wheel_inertia: 0.01,    // Small wheel inertia
            steer_inertia: 0.005,   // Steering mechanism inertia
        }
    }
}

/// The swerve drivetrain model that integrates tire forces and motor torques
/// into overall robot body dynamics.
#[derive(Debug, Clone)]
pub struct SwerveDrivetrain {
    pub config: SwerveDrivetrainConfig,
}

impl SwerveDrivetrain {
    pub fn new(config: SwerveDrivetrainConfig) -> Self {
        SwerveDrivetrain { config }
    }

    /// Calculate the velocity of a wheel module in the robot frame given body velocity.
    /// Returns (longitudinal_velocity, lateral_velocity) in the module's local frame.
    fn calculate_module_velocity(
        &self,
        body_vx: f64,
        body_vy: f64,
        body_omega: f64,
        module_pos: &[f64; 2],
        module_angle: f64,
    ) -> (f64, f64) {
        // Velocity of the module mounting point in the body frame
        // v_module = v_body + omega x r
        let vx_robot = body_vx - body_omega * module_pos[1];
        let vy_robot = body_vy + body_omega * module_pos[0];

        // Transform to module's local frame (aligned with wheel heading)
        let cos_a = module_angle.cos();
        let sin_a = module_angle.sin();

        // Longitudinal is in the direction the wheel is pointing
        let longitudinal = vx_robot * cos_a + vy_robot * sin_a;
        // Lateral is perpendicular to the wheel heading
        let lateral = -vx_robot * sin_a + vy_robot * cos_a;

        (longitudinal, lateral)
    }

    /// Transform forces from module frame back to body frame.
    fn transform_forces_to_body(
        longitudinal_force: f64,
        lateral_force: f64,
        module_angle: f64,
    ) -> (f64, f64) {
        let cos_a = module_angle.cos();
        let sin_a = module_angle.sin();

        let fx = longitudinal_force * cos_a - lateral_force * sin_a;
        let fy = longitudinal_force * sin_a + lateral_force * cos_a;

        (fx, fy)
    }
}

impl Model for SwerveDrivetrain {
    fn reset(&mut self) {
        // No internal state to reset beyond config
    }
}

impl MechanicsModel for SwerveDrivetrain {
    fn step_physics(&mut self, ctx: SimContext, state: &mut SimState) {
        let dt = ctx.dt;
        let num_modules = self.config.module_positions.len();

        // Get current body state
        let body = &state.true_state.body_state;
        let body_vx = body.velocity[0];
        let body_vy = body.velocity[1];
        let body_omega = body.angular_velocity[2]; // Yaw rate

        let mut net_force_x = 0.0;
        let mut net_force_y = 0.0;
        let mut net_torque = 0.0;

        // 1. Update kinematics for each module
        for i in 0..num_modules {
            if i >= state.true_state.wheel_states.len() {
                continue;
            }

            let module_pos = &self.config.module_positions[i];
            let wheel = &mut state.true_state.wheel_states[i];

            // Calculate module velocities from body state
            let (v_long, v_lat) =
                self.calculate_module_velocity(body_vx, body_vy, body_omega, module_pos, wheel.angle);

            wheel.longitudinal_translational_velocity = v_long;
            wheel.lateral_translational_velocity = v_lat;

            // 2. Update wheel angular velocity based on motor torque
            // tau = I * alpha => alpha = tau / I
            // The motor applies torque to the wheel
            if i < state.true_state.motors.len() {
                let motor_torque = state.true_state.motors[i].applied_torque;
                // Subtract reaction torque from tire forces (tire generates torque back on wheel)
                let tire_reaction_torque =
                    wheel.tire.longitudinal_force * wheel.wheel_radius;

                let net_wheel_torque = motor_torque - tire_reaction_torque;
                let angular_acceleration = net_wheel_torque / self.config.wheel_inertia;
                wheel.driving_angular_velocity += angular_acceleration * dt;
            }

            // 3. Sum tire forces into body dynamics
            // Forces are already computed in tire.rs via TireManager
            let (fx, fy) = Self::transform_forces_to_body(
                wheel.tire.longitudinal_force,
                wheel.tire.lateral_force,
                wheel.angle,
            );

            net_force_x += fx;
            net_force_y += fy;

            // Torque about CoM from this module's forces
            // tau = r x F = rx * Fy - ry * Fx
            net_torque += module_pos[0] * fy - module_pos[1] * fx;
        }

        // 4. Integrate body accelerations
        let ax = net_force_x / self.config.mass;
        let ay = net_force_y / self.config.mass;
        let alpha = net_torque / self.config.moment_of_inertia;

        // Update velocities (semi-implicit Euler)
        state.true_state.body_state.velocity[0] += ax * dt;
        state.true_state.body_state.velocity[1] += ay * dt;
        state.true_state.body_state.angular_velocity[2] += alpha * dt;

        // Update positions
        state.true_state.body_state.position[0] += state.true_state.body_state.velocity[0] * dt;
        state.true_state.body_state.position[1] += state.true_state.body_state.velocity[1] * dt;
        state.true_state.body_state.orientation[2] += state.true_state.body_state.angular_velocity[2] * dt;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use simcore::{BodyState, TireState, MotorState, BatteryState, TrueState, ActuatorInput, SensorBus};

    fn create_test_state(num_modules: usize) -> SimState {
        let wheel_states: Vec<WheelState> = (0..num_modules)
            .map(|_| WheelState {
                driving_angular_velocity: 0.0,
                wheel_radius: 0.05,
                turning_angular_velocity: 0.0,
                longitudinal_translational_velocity: 0.0,
                lateral_translational_velocity: 0.0,
                tire: TireState {
                    slip_angle: 0.0,
                    slip_ratio: 0.0,
                    longitudinal_force: 0.0,
                    lateral_force: 0.0,
                    tire_load: 100.0,
                },
                angle: 0.0,
            })
            .collect();

        let motors: Vec<MotorState> = (0..num_modules)
            .map(|_| MotorState::default())
            .collect();

        SimState {
            true_state: TrueState {
                wheel_states,
                body_state: BodyState::default(),
                motors,
                battery_state: BatteryState::default(),
            },
            control_input: ActuatorInput::default(),
            sensor_bus: SensorBus::default(),
        }
    }

    #[test]
    fn test_stationary_robot_no_acceleration() {
        let mut drivetrain = SwerveDrivetrain::new(SwerveDrivetrainConfig::default());
        let mut state = create_test_state(4);

        let ctx = SimContext { dt: 0.001, t: 0.0 };
        drivetrain.step_physics(ctx, &mut state);

        // With no forces, velocity should remain zero
        assert!((state.true_state.body_state.velocity[0]).abs() < 1e-9);
        assert!((state.true_state.body_state.velocity[1]).abs() < 1e-9);
    }

    #[test]
    fn test_forward_force_accelerates_robot() {
        let mut drivetrain = SwerveDrivetrain::new(SwerveDrivetrainConfig::default());
        let mut state = create_test_state(4);

        // Apply forward force on all wheels
        for wheel in &mut state.true_state.wheel_states {
            wheel.tire.longitudinal_force = 25.0; // 25N per wheel = 100N total
        }

        let ctx = SimContext { dt: 0.01, t: 0.0 };
        drivetrain.step_physics(ctx, &mut state);

        // Total force = 100N, mass = 50kg => a = 2 m/s^2
        // After 0.01s, velocity should be 0.02 m/s
        assert!((state.true_state.body_state.velocity[0] - 0.02).abs() < 1e-6);
    }
}

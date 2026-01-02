use nalgebra as na;
use simcore::{MechanicsModel, Model, WheelState};

pub struct TireConstants {
    pub longitudinal_coefficient_of_friction: f64,
    pub lateral_coefficient_of_friction: f64,
    pub cornering_stiffness: f64,
    pub longitudinal_stiffness: f64,
    pub longitudinal_relaxation_length: f64,
    pub lateral_relaxation_length: f64,
}

impl TireConstants {
    pub fn new(
        longitudinal_coefficient_of_friction: f64,
        lateral_coefficient_of_friction: f64,
        cornering_stiffness: f64,
        longitudinal_stiffness: f64,
        longitudinal_relaxation_length: f64,
        lateral_relaxation_length: f64,
    ) -> Self {
        TireConstants {
            longitudinal_coefficient_of_friction,
            lateral_coefficient_of_friction,
            cornering_stiffness,
            longitudinal_stiffness,
            longitudinal_relaxation_length,
            lateral_relaxation_length,
        }
}
}

impl Default for TireConstants {
    fn default() -> Self {
        TireConstants {
            longitudinal_coefficient_of_friction: 1.0,
            lateral_coefficient_of_friction: 1.0,
            cornering_stiffness: 1.0,
            longitudinal_stiffness: 1.0,
            longitudinal_relaxation_length: 1.0,
            lateral_relaxation_length: 1.0,
        }
    }
}

pub struct TireManager {
    pub tire_constants: Vec<TireConstants>,
}

impl TireManager {
    pub fn new() -> Self {
        TireManager {
            tire_constants: vec![TireConstants::default()],
        }
    }

    pub fn add_tire(&mut self, tire: TireConstants) {
        self.tire_constants.push(tire);
    }
}

impl Model for TireManager {
    fn reset(&mut self) {
        // No internal state to reset in this simple model
    }
}

fn update_slip_angle(wheel: &mut WheelState, tire: &TireConstants, dt: f64) {
    // Low-speed stability: when velocity is very small, zero out slip angle
    // to prevent numerical instability from atan2(small, small)
    let v_combined = (wheel.longitudinal_translational_velocity.powi(2) 
                     + wheel.lateral_translational_velocity.powi(2)).sqrt();
    if v_combined < 0.01 {
        // Below 0.01 m/s, zero out slip angle to prevent instability
        wheel.tire.slip_angle = 0.0;
        return;
    }
    // Use proper atan2: slip angle = angle between velocity vector and wheel heading
    // For a wheel aligned with body-x, slip angle is the angle of the velocity vector
    // relative to forward. Positive slip angle = velocity pointing to the left of forward.
    // Clamp the denominator to avoid division by near-zero while preserving sign.
    let v_min = 0.01_f64;
    let v_long_clamped = if wheel.longitudinal_translational_velocity.abs() < v_min {
        // When very small, use sign-preserving minimum; if exactly zero, default to positive
        if wheel.longitudinal_translational_velocity >= 0.0 { v_min } else { -v_min }
    } else {
        wheel.longitudinal_translational_velocity
    };
    let actual_slip_angle = wheel.lateral_translational_velocity.atan2(v_long_clamped);
    if tire.lateral_relaxation_length == 0.0 {
        wheel.tire.slip_angle = actual_slip_angle;
    } else {
        let relaxation_time_constant = (tire.lateral_relaxation_length) / (wheel.longitudinal_translational_velocity.abs().max(1e-6));
        let update_rate = (actual_slip_angle - wheel.tire.slip_angle) / relaxation_time_constant;
        wheel.tire.slip_angle += update_rate * dt;
    }
}

fn update_slip_ratio(wheel: &mut WheelState, tire: &TireConstants, dt: f64) {
    // Low-speed stability: when velocity is very small, zero out slip ratio
    // to prevent division by near-zero denominators
    if wheel.longitudinal_translational_velocity.abs() < 0.01 {
        // At very low speed, use a simpler formula or zero out
        // If wheel is spinning but no ground velocity, it's pure slip
        if wheel.driving_angular_velocity.abs() > 0.1 {
            // Wheel spinning with no ground motion = full slip
            wheel.tire.slip_ratio = wheel.driving_angular_velocity.signum();
        } else {
            wheel.tire.slip_ratio = 0.0;
        }
        return;
    }

    let actual_slip_ratio = (wheel.driving_angular_velocity * wheel.wheel_radius - wheel.longitudinal_translational_velocity) / (wheel.longitudinal_translational_velocity.abs().max(1e-3));
    if tire.longitudinal_relaxation_length == 0.0 {
        wheel.tire.slip_ratio = actual_slip_ratio;
    } else {
        let relaxation_time_constant = (tire.longitudinal_relaxation_length) / (wheel.longitudinal_translational_velocity.abs().max(1e-6));
        let update_rate = (actual_slip_ratio - wheel.tire.slip_ratio) / relaxation_time_constant;
        wheel.tire.slip_ratio += update_rate * dt;
    }
}

fn get_fiala_longitudinal_force(wheel: &WheelState, tire: &TireConstants) -> f64 {
    let slip_ratio = wheel.tire.slip_ratio;
    let tire_load = wheel.tire.tire_load;

    let k = slip_ratio;

    let critical_k = (3.0 * tire.longitudinal_coefficient_of_friction * tire_load) / (tire.longitudinal_stiffness);
    if k.abs() < critical_k {
        return -tire.longitudinal_stiffness * k + (tire.longitudinal_stiffness.powi(2) / (3.0 * tire.longitudinal_coefficient_of_friction * tire_load)) * k.abs() * k - (tire.longitudinal_stiffness.powi(3) / (27.0 * tire.longitudinal_coefficient_of_friction.powi(2) * tire_load.powi(2))) * k.powi(2) * k;
    } else {
        return -tire.longitudinal_coefficient_of_friction * tire_load * k.signum();
    }
}

fn get_fiala_lateral_force(wheel: &WheelState, tire: &TireConstants) -> f64 {
    let slip_angle = wheel.tire.slip_angle;
    let tire_load = wheel.tire.tire_load;

    let t = slip_angle.tan();

    let critical_t = (3.0 * tire.lateral_coefficient_of_friction * tire_load) / (tire.cornering_stiffness);
    if t.abs() < critical_t {
        return -tire.cornering_stiffness * t + (tire.cornering_stiffness.powi(2) / (3.0 * tire.lateral_coefficient_of_friction * tire_load)) * t.abs() * t - (tire.cornering_stiffness.powi(3) / (27.0 * tire.lateral_coefficient_of_friction.powi(2) * tire_load.powi(2))) * t.powi(3);
    } else {
        return -tire.lateral_coefficient_of_friction * tire_load * t.signum();
    }
}

fn elliptically_scale_forces(longitudinal_force: f64, lateral_force: f64, tire_load: f64, tire: &TireConstants) -> (f64, f64) {
    let combined = (longitudinal_force / (tire.longitudinal_coefficient_of_friction * tire_load)).hypot(lateral_force / (tire.lateral_coefficient_of_friction * tire_load));
    if combined > 1.0 {
        (longitudinal_force / combined, lateral_force / combined)
    } else {
        (longitudinal_force, lateral_force)
    }
}

impl MechanicsModel for TireManager {
    fn step_physics(&mut self, ctx: simcore::SimContext, state: &mut simcore::SimState) {
        let dt = ctx.dt;
        for (i, tire) in self.tire_constants.iter().enumerate() {
            let wheel = &mut state.true_state.wheel_states[i];
            // Update tire forces based on tire constants and wheel state
            update_slip_angle(wheel, tire, dt);
            update_slip_ratio(wheel, tire, dt);

            let (scaled_longitudinal_force, scaled_lateral_force) = elliptically_scale_forces(
                get_fiala_longitudinal_force(wheel, tire),
                get_fiala_lateral_force(wheel, tire),
                wheel.tire.tire_load,
                tire,
            );

            state.true_state.wheel_states[i].tire.longitudinal_force = scaled_longitudinal_force;
            state.true_state.wheel_states[i].tire.lateral_force = scaled_lateral_force;

        }
    }
}

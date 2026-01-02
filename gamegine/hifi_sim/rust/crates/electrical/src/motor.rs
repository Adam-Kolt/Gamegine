use simcore::{ElectricalModel, Model, SimContext, SimState, MotorInput, MotorState};
use serde::{Deserialize, Serialize};


const STANDARD_POLES_NUMBER: u32 = 3;

#[derive(Debug, Clone, Copy)]
pub struct MotorConstant {
    pub pole_pairs: u32,
    pub resistance: f64,
    pub inductance_d: f64,
    pub inductance_q: f64,
    pub flux_linkage: f64,
}

impl MotorConstant {
    pub fn new(pole_pairs: u32, resistance: f64, inductance_d: f64, inductance_q: f64, flux_linkage: f64) -> Self {
        MotorConstant {
            pole_pairs,
            resistance,
            inductance_d,
            inductance_q,
            flux_linkage,
        }
    }

    pub fn from_recalc_values(kv_rpm_per_volt: f64, kt_nm_per_amp: f64, km_nm_per_root_of_watt: f64) -> Self {
        let poles = STANDARD_POLES_NUMBER;
        let flux_linkage = (2.0 / 3.0) * kt_nm_per_amp / (poles as f64);
        let inductance_d = 0.000015;
        let inductance_q = inductance_d;
        let resistance = (kt_nm_per_amp / km_nm_per_root_of_watt).powi(2);

        MotorConstant {
            pole_pairs: poles,
            resistance,
            inductance_d,
            inductance_q,
            flux_linkage,
        }

    }

    pub fn kraken_x60() -> Self {
        MotorConstant::from_recalc_values(502.1, 0.0194, 0.107)
    }

    pub fn neo() -> Self {
        MotorConstant::from_recalc_values(493.5, 0.0181, 0.070)
    }
}



#[derive(Debug, Clone, Default)]
pub struct MotorBank {
    pub motor_constants: Vec<MotorConstant>
}

impl MotorBank {
    pub fn add_motor(&mut self, motor: MotorConstant) {
        self.motor_constants.push(motor);
    }
}

impl Model for MotorBank {
    fn reset(&mut self) {
        self.motor_constants.clear();
    }
}

fn derivative_current_d(current_d: f64, current_q: f64, voltage_d: f64, resistance: f64, inductance_d: f64, inductance_q: f64, electrical_velocity: f64) -> f64 {
    (voltage_d - resistance * current_d + inductance_q * electrical_velocity * current_q) / inductance_d
}

fn derivative_current_q(current_d: f64, current_q: f64, voltage_q: f64, resistance: f64, inductance_d: f64, inductance_q: f64, flux_linkage: f64, electrical_velocity: f64) -> f64 {
    (voltage_q - resistance * current_q - electrical_velocity*(inductance_d * current_d + flux_linkage * (3.0/2.0))) / inductance_q // TODO: Figure out the 3/2 factors, for some reason the flux being scaled by 2/3 screws with the correct ke
}



impl ElectricalModel for MotorBank {
    fn step_electrical(&mut self, ctx: SimContext, state: &mut SimState) {
        let dt = ctx.dt;
        for (i, motor) in self.motor_constants.iter().enumerate() {
            let input: MotorInput = state.control_input.motor_inputs[i];
            let voltage_q = input.duty_cycle_q * state.true_state.battery_state.voltage;
            let voltage_d = input.duty_cycle_d * state.true_state.battery_state.voltage;

            // Update motor state
            // TODO: Update to more accurate integration
            let mech_vel = state.true_state.motors[i].mechanical_velocity;
            let current_d = state.true_state.motors[i].current_d;
            let current_q = state.true_state.motors[i].current_q;

            let d_current_d = derivative_current_d(current_d, current_q, voltage_d, motor.resistance, motor.inductance_d, motor.inductance_q, mech_vel*motor.pole_pairs as f64);
            let d_current_q = derivative_current_q(current_d, current_q, voltage_q, motor.resistance, motor.inductance_d, motor.inductance_q, motor.flux_linkage, mech_vel*motor.pole_pairs as f64);

            state.true_state.motors[i].current_d += d_current_d * dt;
            state.true_state.motors[i].current_q += d_current_q * dt;

            // Update mechanical torques
            state.true_state.motors[i].applied_torque = 1.5 * (motor.pole_pairs as f64) * (
                motor.flux_linkage * state.true_state.motors[i].current_q +
                (motor.inductance_d - motor.inductance_q) * state.true_state.motors[i].current_d * state.true_state.motors[i].current_q
            )
        }
    }
}

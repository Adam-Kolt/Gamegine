use simcore::ElectricalModel;
use simcore::{BatteryState, Model, SimContext, SimState};



#[derive(Debug, Clone, Copy)]
pub struct Peukert {
    pub constant: f64,
    pub reference_discharge_current: f64,
}

impl Default for Peukert {
    fn default() -> Self {
        Peukert {
            constant: 1.183,
            reference_discharge_current: 0.9,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RCBranch {
    pub resistance: f64,
    pub capacitance: f64,
}

impl Default for RCBranch {
    fn default() -> Self {
        RCBranch {
            resistance: 1.0,
            capacitance: 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct BatteryConstant {
    pub peukert_constant: Peukert,
    pub rated_capacity_ah: f64,
    pub open_circuit_voltage_function: fn(f64) -> f64,
    pub ohmic_resistance_function: fn(f64) -> f64,
    pub fast_polarization_constants: RCBranch,
    pub slow_polarization_constants: RCBranch
}

pub fn default_ocv_from_soc(soc: f64) -> f64 {
    // clamp SoC
    let s = soc.max(0.0).min(1.0);

    // anchors
    const X0: f64 = 0.0;
    const X1: f64 = 0.5;
    const X2: f64 = 1.0;

    const Y0: f64 = 11.77;
    const Y1: f64 = 12.20;
    const Y2: f64 = 13.03;

    // monotone slopes at the anchors (precomputed)
    const M0: f64 = 0.46;
    const M1: f64 = 1.26;
    const M2: f64 = 2.06;

    // piecewise Hermite cubic on [X0,X1] and [X1,X2]
    if s <= X1 {
        // interval [0.0, 0.5]
        let h = X1 - X0;           // 0.5
        let t = (s - X0) / h;      // normalized position
        let t2 = t * t;
        let t3 = t2 * t;

        // Hermite basis
        let h00 =  2.0*t3 - 3.0*t2 + 1.0;
        let h10 =        t3 - 2.0*t2 + t;
        let h01 = -2.0*t3 + 3.0*t2;
        let h11 =        t3 -       t2;

        h00*Y0 + h10*h*M0 + h01*Y1 + h11*h*M1
    } else {
        // interval [0.5, 1.0]
        let h = X2 - X1;           // 0.5
        let t = (s - X1) / h;
        let t2 = t * t;
        let t3 = t2 * t;

        let h00 =  2.0*t3 - 3.0*t2 + 1.0;
        let h10 =        t3 - 2.0*t2 + t;
        let h01 = -2.0*t3 + 3.0*t2;
        let h11 =        t3 -       t2;

        h00*Y1 + h10*h*M1 + h01*Y2 + h11*h*M2
    }
}

pub fn default_r0_from_soc(soc: f64, r_mid_ohm: f64) -> f64 {
    // shape parameters:
    // stronger rise near empty, mild rise near full → U-shaped curve
    const S_REF: f64 = 0.60; // where R0 equals r_mid_ohm
    const A_LOW: f64 = 0.60; // weight of low-SoC rise
    const B_LOW: f64 = 1.50; // curvature of low-SoC rise
    const A_HIGH: f64 = 0.40; // weight of high-SoC bump
    const B_HIGH: f64 = 2.00; // curvature of high-SoC bump

    let s = soc.clamp(0.0, 1.0);

    // base terms (normalized so factor = 1 at S_REF)
    let tl = (1.0 - s).powf(B_LOW);
    let th = s.powf(B_HIGH);
    let tl_ref = (1.0 - S_REF).powf(B_LOW);
    let th_ref = S_REF.powf(B_HIGH);

    let factor = 1.0 + A_LOW * (tl - tl_ref) + A_HIGH * (th - th_ref);
    r_mid_ohm * factor
}

impl Default for BatteryConstant {
    fn default() -> Self {
        BatteryConstant {
            peukert_constant: Peukert::default(),
            rated_capacity_ah: 11.2,
            open_circuit_voltage_function: default_ocv_from_soc,
            ohmic_resistance_function: |soc| default_r0_from_soc(soc, 0.008),
            fast_polarization_constants: RCBranch { resistance: 0.0027, capacitance: 741.0 },
            slow_polarization_constants: RCBranch { resistance: 0.0018, capacitance: 66667.0 },
        }
    }
}



pub struct Battery {
    pub constants: BatteryConstant,
}
fn peukert_effective_capacity(current: f64, constants: &BatteryConstant) -> f64 {
    let peukert = constants.peukert_constant;
    (constants.rated_capacity_ah * 3600.0) * (peukert.reference_discharge_current / current.abs()).powf(peukert.constant - 1.0)
}

fn state_of_charge_derivative(battery_state: &BatteryState, constants: &BatteryConstant) -> f64 {
    -battery_state.total_current_draw / peukert_effective_capacity(battery_state.total_current_draw, constants)
}

fn update_rc_branch_voltage(dt: f64, current: f64, branch_voltage: f64, branch: &RCBranch) -> f64 {
    (-dt / (branch.capacitance * branch.resistance)).exp() * branch_voltage + current * branch.resistance * (1.0 - (-dt / (branch.capacitance * branch.resistance)).exp())
}

impl Model for Battery {
    fn reset(&mut self) {
        // No internal state to reset in this simple model
    }
}

impl ElectricalModel for Battery {
    fn step_electrical(&mut self, ctx: SimContext, state: &mut SimState) {
        let battery_state = &mut state.true_state.battery_state;

        let dt = ctx.dt;
        let total_current_draw = battery_state.total_current_draw;

        // Update the battery state based on the current draw and other factors
        let d_soc = state_of_charge_derivative(battery_state, &self.constants);
        battery_state.state_of_charge += d_soc * dt;

        // Update the RC branch voltages
        battery_state.fast_polarization_voltage = update_rc_branch_voltage(dt, total_current_draw, battery_state.fast_polarization_voltage, &self.constants.fast_polarization_constants);
        battery_state.slow_polarization_voltage = update_rc_branch_voltage(dt, total_current_draw, battery_state.slow_polarization_voltage, &self.constants.slow_polarization_constants);


        // Update the battery voltage
        battery_state.voltage = (self.constants.open_circuit_voltage_function)(battery_state.state_of_charge) - total_current_draw * (self.constants.ohmic_resistance_function)(battery_state.state_of_charge) - battery_state.fast_polarization_voltage - battery_state.slow_polarization_voltage;
    }

}


// Mechanical Traits
pub struct TireState {
    pub slip_angle: f64,
    pub slip_ratio: f64,
    pub longitudinal_force: f64,
    pub lateral_force: f64,
    pub tire_load: f64,
}

pub struct WheelState {
    pub driving_angular_velocity: f64,
    pub wheel_radius: f64,
    pub turning_angular_velocity: f64,
    pub longitudinal_translational_velocity: f64,
    pub lateral_translational_velocity: f64,
    pub tire: TireState,
    pub angle: f64
}

#[derive(Default)]
pub struct BodyState {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub orientation: [f64; 3], // roll, pitch, yaw
    pub angular_velocity: [f64; 3],
    pub center_of_mass: [f64; 3]
}


// Electrical Traits
#[derive(Debug, Clone, Copy)]
pub enum BridgeMode {
    Open,
    Shorted,
    Closed
}

#[derive(Debug, Clone, Copy)]
pub struct MotorState {
    pub current_q: f64,
    pub current_d: f64,
    pub mechanical_velocity: f64,
    pub applied_torque: f64,
    pub bridge_mode: BridgeMode
}

impl Default for MotorState {
    fn default() -> Self {
        MotorState {
            current_q: 0.0,
            current_d: 0.0,
            mechanical_velocity: 0.0,
            applied_torque: 0.0,
            bridge_mode: BridgeMode::Closed
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MotorInput {
    pub duty_cycle_q: f64,
    pub duty_cycle_d: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct BatteryState {
    pub state_of_charge: f64,
    pub voltage: f64, 
    pub fast_polarization_voltage: f64,
    pub slow_polarization_voltage: f64,
    pub total_current_draw: f64
}

impl Default for BatteryState {
    fn default() -> Self {
        BatteryState {
            state_of_charge: 1.0,
            voltage: 12.0,
            fast_polarization_voltage: 0.0,
            slow_polarization_voltage: 0.0,
            total_current_draw: 0.0
        }
    }
}

// General Traits
#[derive(Default)]
pub struct SensorBus {
    // Robot State
    pub wheel_omega: [f64; 4],
    pub steer_angle: [f64; 4],
    pub body_state: [f64; 6],
    pub motors: Vec<MotorState>,
    pub battery_voltage: f64
}
#[derive(Default)]
pub struct TrueState {
    pub wheel_states: Vec<WheelState>,
    pub body_state: BodyState,
    pub motors: Vec<MotorState>,
    pub battery_state: BatteryState,
}

#[derive(Default)]
pub struct ActuatorInput {
    pub motor_inputs: Vec<MotorInput>
}

#[derive(Default)]
pub struct SimState {
    pub true_state: TrueState,
    pub control_input: ActuatorInput,
    pub sensor_bus: SensorBus,
}

pub struct TimestepScales {
    pub physics: u32,
    pub control: u32,
    pub electrical: u32,
    pub sensor: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct SimContext {
    pub dt: f64,
    pub t: f64,
}

pub trait Model {
    fn reset(&mut self);
}

pub trait MechanicsModel: Model {
    fn step_physics(&mut self, ctx: SimContext, state: &mut SimState);
}

pub trait ControlModel: Model {
    fn step_control(&mut self, ctx: SimContext, state: &mut SimState);
}

pub trait ElectricalModel: Model {
    fn step_electrical(&mut self, ctx: SimContext, state: &mut SimState);
}

pub trait SensorModel: Model {
    fn step_sensor(&mut self, ctx: SimContext, state: &mut SimState);
}

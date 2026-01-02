use electrical::battery::{Battery, BatteryConstant};
use electrical::motor::{MotorBank, MotorConstant};
use simcore::{ElectricalModel, SimContext, SimState, MotorInput, MotorState};
use std::fs::File;
use std::io::Write;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Battery params (tweak as needed)
    let mut batt = Battery {
        constants: BatteryConstant::default()
    };

    let mut motor_bank = MotorBank::default();

    let motor_constant = MotorConstant::kraken_x60();
    motor_bank.add_motor(motor_constant);


    let mut bus = SimState::default();
    

    // Sim settings
    let dt = 1e-4; 
    let dt_motor = 1e-4;
    let t_end = 10.0;    // seconds
    let spike_interval = 2.5; // seconds
    let spike_duration = 2.5; // seconds
    let mut spike_counter: f64 = spike_interval as f64 + spike_duration;
    let motor_load_inertia = 0.001; // kg*m^2 (example value)

    let mut csv = File::create("battery_step.csv")?;
    writeln!(csv, "t,batt_v,soc,current,current_q,current_d,mechanical_velocity,applied_torque")?;
    bus.control_input.motor_inputs = vec![MotorInput { duty_cycle_d: 0.0, duty_cycle_q: 0.0 }; motor_bank.motor_constants.len()];
    bus.true_state.motors = vec![MotorState::default(); motor_bank.motor_constants.len()];
    let mut t = 0.0;
    while t <= t_end {
        // Program the “load” as total current drawn by all actuators
        if spike_counter <= spike_duration{
            bus.control_input.motor_inputs[0] = MotorInput { duty_cycle_d: 0.0, duty_cycle_q: 1.0 };
        } else {
            bus.control_input.motor_inputs[0] = MotorInput { duty_cycle_d: 0.0, duty_cycle_q: (t * 2.0 * std::f64::consts::PI / spike_duration).sin() };
        };
        spike_counter -= dt;
        if spike_counter <= 0.0 {
            spike_counter = spike_interval as f64 + spike_duration;
        }

        // Put that on the bus (pretend multiple devices summing to i_load)
        // We only need the sum; a couple of entries is fine.
        let mut t_inner = 0.0;
        while t_inner < dt {
            motor_bank.step_electrical(SimContext { dt: dt_motor, t: t + t_inner }, &mut bus);
            t_inner += dt_motor;

            bus.true_state.motors[0].mechanical_velocity += (bus.true_state.motors[0].applied_torque / motor_load_inertia) * dt_motor;
        }
        bus.true_state.battery_state.total_current_draw = bus.true_state.motors[0].current_q * bus.control_input.motor_inputs[0].duty_cycle_q + bus.true_state.motors[0].current_d * bus.control_input.motor_inputs[0].duty_cycle_d;
        batt.step_electrical(SimContext { dt, t }, &mut bus);
        

        writeln!(
            csv,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            t, bus.true_state.battery_state.voltage, bus.true_state.battery_state.state_of_charge, bus.true_state.battery_state.total_current_draw, bus.true_state.motors[0].current_q, bus.true_state.motors[0].current_d, bus.true_state.motors[0].mechanical_velocity, bus.true_state.motors[0].applied_torque
        )?;

        t += dt;
    }

    println!("Wrote battery_step.csv");
    Ok(())
}

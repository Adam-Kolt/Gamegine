use electrical::battery::{Battery, BatteryConstant};
use simcore::{ElectricalModel, SimContext, SimState};
use std::fs::File;
use std::io::Write;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Battery params (tweak as needed)
    let mut batt = Battery {
        constants: BatteryConstant::default()
    };

    // Shared bus: we’ll fake a current draw without motors, just to stress the battery model
    let mut bus = SimState::default();

    // Sim settings
    let dt = 1e-2;      // 10 ms electrical step
    let t_end = 150.0;    // seconds
    let spike_interval = 3; // seconds
    let spike_duration = 5.0; // seconds
    let mut spike_counter: f64 = spike_interval as f64 + spike_duration;
    let i_nominal = 10.0; // A (idle)
    let i_spike  = 1000.0; // A (sudden load)
    let mut i_load = i_nominal;

    let mut csv = File::create("battery_step.csv")?;
    writeln!(csv, "t,batt_v,soc,current")?;

    let mut t = 0.0;
    while t <= t_end {
        // Program the “load” as total current drawn by all actuators
        i_load = if spike_counter <= spike_duration{
            i_load + (i_spike - i_load) * 0.05
        } else {
            i_load + (i_nominal - i_load) * 0.05
        };
        spike_counter -= dt;
        if spike_counter <= 0.0 {
            spike_counter = spike_interval as f64 + spike_duration;
        }

        // Put that on the bus (pretend multiple devices summing to i_load)
        // We only need the sum; a couple of entries is fine.
        bus.true_state.battery_state.total_current_draw = i_load;

        batt.step_electrical(SimContext { dt, t }, &mut bus);

        writeln!(
            csv,
            "{:.6},{:.6},{:.6},{:.6}",
            t, bus.true_state.battery_state.voltage, bus.true_state.battery_state.state_of_charge, i_load
        )?;

        t += dt;
    }

    println!("Wrote battery_step.csv");
    Ok(())
}

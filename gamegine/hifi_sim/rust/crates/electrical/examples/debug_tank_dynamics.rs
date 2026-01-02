//! Minimal test to diagnose the tank drive dynamics issue.
//! This runs a few simulation steps and prints detailed state at each step.

use electrical::battery::{Battery, BatteryConstant};
use electrical::motor::{MotorBank, MotorConstant};
use mechanics::tire::{TireManager, TireConstants};
use simcore::{
    ElectricalModel, MechanicsModel, Model, MotorInput, MotorState, SimContext, SimState,
    TireState, WheelState,
};

const WHEEL_RADIUS: f64 = 0.05;
const MASS: f64 = 50.0;
const G: f64 = 9.81;
const GEAR_RATIO: f64 = 5.0;
const DRIVE_EFFICIENCY: f64 = 0.92;
const WHEEL_INERTIA: f64 = 0.02;
const DT: f64 = 0.001; // 1ms timestep

fn main() {
    println!("=== Tank Drive Dynamics Diagnostic ===\n");

    // Initialize motor bank (4 motors)
    let mut motors = MotorBank::default();
    for _ in 0..4 {
        motors.add_motor(MotorConstant::kraken_x60());
    }

    // Initialize battery
    let mut batt = Battery {
        constants: BatteryConstant::default(),
    };

    // Initialize tire manager (4 tires)
    let mut tires = TireManager::new();
    tires.tire_constants.clear();
    for _ in 0..4 {
        tires.add_tire(TireConstants::new(1.5, 1.0, 3000.0, 3000.0, 0.0, 0.0));
    }

    // Initialize sim state
    let mut bus = SimState::default();
    bus.control_input.motor_inputs = vec![
        MotorInput {
            duty_cycle_d: 0.0,
            duty_cycle_q: 0.0,
        };
        4
    ];
    bus.true_state.motors = vec![MotorState::default(); 4];
    bus.true_state.wheel_states = (0..4)
        .map(|_| WheelState {
            driving_angular_velocity: 0.0,
            wheel_radius: WHEEL_RADIUS,
            turning_angular_velocity: 0.0,
            longitudinal_translational_velocity: 0.0,
            lateral_translational_velocity: 0.0,
            tire: TireState {
                slip_angle: 0.0,
                slip_ratio: 0.0,
                longitudinal_force: 0.0,
                lateral_force: 0.0,
                tire_load: MASS * G / 4.0,
            },
            angle: 0.0,
        })
        .collect();

    // Robot state
    let mut v = 0.0; // forward velocity
    let mut yaw_rate = 0.0;

    println!("Initial state:");
    println!("  v = {:.4} m/s, yaw_rate = {:.4} rad/s", v, yaw_rate);
    for i in 0..4 {
        let w = &bus.true_state.wheel_states[i];
        let m = &bus.true_state.motors[i];
        println!(
            "  wheel[{}]: omega={:.4}, slip_ratio={:.4}, fx={:.4}, motor_tq={:.4}",
            i,
            w.driving_angular_velocity,
            w.tire.slip_ratio,
            w.tire.longitudinal_force,
            m.applied_torque
        );
    }

    // Run 100 steps with ZERO input
    println!("\n--- Running 100 steps with ZERO throttle ---\n");

    for step in 0..100 {
        let t = step as f64 * DT;
        let ctx = SimContext { dt: DT, t };

        // Zero throttle
        for input in &mut bus.control_input.motor_inputs {
            input.duty_cycle_q = 0.0;
            input.duty_cycle_d = 0.0;
        }

        // 1. Update motor mechanical velocity from wheel state
        for i in 0..4 {
            bus.true_state.motors[i].mechanical_velocity =
                bus.true_state.wheel_states[i].driving_angular_velocity * GEAR_RATIO;
        }

        // 2. Step electrical (motor currents and torques)
        motors.step_electrical(ctx, &mut bus);

        // 3. Update wheel kinematics for tire model (stationary robot for now)
        for i in 0..4 {
            let wheel = &mut bus.true_state.wheel_states[i];
            wheel.longitudinal_translational_velocity = v;
            wheel.lateral_translational_velocity = 0.0;
        }

        // 4. Step tire model
        tires.step_physics(ctx, &mut bus);

        // 5. Wheel rotational dynamics
        for i in 0..4 {
            let fx = bus.true_state.wheel_states[i].tire.longitudinal_force;
            let tq_motor = bus.true_state.motors[i].applied_torque;
            let wheel_torque = tq_motor * GEAR_RATIO * DRIVE_EFFICIENCY;
            let tire_reaction = fx * WHEEL_RADIUS;
            let net_torque = wheel_torque + tire_reaction;
            let omega = bus.true_state.wheel_states[i].driving_angular_velocity;
            let domega = net_torque / WHEEL_INERTIA;
            bus.true_state.wheel_states[i].driving_angular_velocity = omega + domega * DT;
        }

        // 6. Sum forces on chassis (simplified - just sum fx)
        let mut f_long_total = 0.0;
        for i in 0..4 {
            f_long_total += bus.true_state.wheel_states[i].tire.longitudinal_force;
        }
        let a = f_long_total / MASS;
        v += a * DT;

        // Print every 10 steps
        if step % 10 == 0 || step < 5 {
            println!("Step {}: t={:.3}s, v={:.4} m/s", step, t, v);
            for i in 0..4 {
                let w = &bus.true_state.wheel_states[i];
                let m = &bus.true_state.motors[i];
                println!(
                    "  wheel[{}]: omega={:.4}, slip_ratio={:.4}, fx={:.4}, motor_tq={:.4}, current_q={:.4}",
                    i,
                    w.driving_angular_velocity,
                    w.tire.slip_ratio,
                    w.tire.longitudinal_force,
                    m.applied_torque,
                    m.current_q
                );
            }
        }
    }

    println!("\n--- Final state after 100 steps with ZERO throttle ---");
    println!("v = {:.4} m/s, yaw_rate = {:.4} rad/s", v, yaw_rate);
    for i in 0..4 {
        let w = &bus.true_state.wheel_states[i];
        let m = &bus.true_state.motors[i];
        println!(
            "  wheel[{}]: omega={:.4}, slip_ratio={:.4}, fx={:.4}, motor_tq={:.4}",
            i,
            w.driving_angular_velocity,
            w.tire.slip_ratio,
            w.tire.longitudinal_force,
            m.applied_torque
        );
    }

    // Check if system is stable
    let total_omega: f64 = bus
        .true_state
        .wheel_states
        .iter()
        .map(|w| w.driving_angular_velocity.abs())
        .sum();
    if total_omega > 0.01 {
        println!("\n❌ FAIL: Wheels are spinning without input! Total |omega| = {:.4}", total_omega);
    } else {
        println!("\n✓ PASS: System is stable at rest.");
    }
}

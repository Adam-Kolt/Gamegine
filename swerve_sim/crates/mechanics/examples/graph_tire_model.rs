use mechanics::tire::{TireConstants, TireManager};
use plotters::prelude::*;
use simcore::{BodyState, MechanicsModel, SimContext, SimState, TireState, TrueState, WheelState};

fn draw_series(
    filename: &str,
    title: &str,
    x_label: &str,
    y_label: &str,
    x: &[f64],
    y: &[f64],
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(filename, (1024, 768)).into_drawing_area();
    root.fill(&WHITE)?;

    let x_min = x
        .iter()
        .cloned()
        .fold(f64::INFINITY, |a, b| a.min(b));
    let x_max = x
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, |a, b| a.max(b));
    let y_min = y
        .iter()
        .cloned()
        .fold(f64::INFINITY, |a, b| a.min(b));
    let y_max = y
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, |a, b| a.max(b));

    let mut chart = ChartBuilder::on(&root)
        .caption(title, ("Arial", 28))
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(60)
        .build_cartesian_2d(x_min..x_max, y_min..y_max)?;

    chart.configure_mesh().x_desc(x_label).y_desc(y_label).draw()?;

    chart.draw_series(LineSeries::new(
        x.iter().cloned().zip(y.iter().cloned()),
        &BLUE,
    ))?
    .label("force")
    .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE.filled()));

    chart.configure_series_labels().border_style(&BLACK).draw()?;

    root.present()?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Tire parameters (Fiala-like with elliptical coupling). Tuned for visible saturation.
    let tire_constants = TireConstants::new(
        1.0,     // longitudinal coefficient of friction (mu_x)
        1.0,     // lateral coefficient of friction (mu_y)
    30000.0, // cornering stiffness C_alpha [N/rad]
        30000.0, // longitudinal stiffness C_kappa [N]
        0.0,     // longitudinal relaxation length (0 => direct)
        0.0,     // lateral relaxation length (0 => direct)
    );

    // Simulation/tire setup
    let tire_load = 1500.0; // [N]
    let wheel_radius = 0.3; // [m]
    let vx = 10.0; // [m/s] forward velocity used for slip calculations

    // Build a minimal SimState with one wheel
    let wheel = WheelState {
        driving_angular_velocity: 0.0,
        wheel_radius,
        turning_angular_velocity: 0.0,
        longitudinal_translational_velocity: vx,
        lateral_translational_velocity: 0.0,
        tire: TireState {
            slip_angle: 0.0,
            slip_ratio: 0.0,
            longitudinal_force: 0.0,
            lateral_force: 0.0,
            tire_load,
        },
        angle: 0.0,
    };

    let mut state = SimState {
        true_state: TrueState {
            wheel_states: vec![wheel],
            body_state: BodyState::default(),
            motors: vec![],
            battery_state: Default::default(),
        },
        control_input: Default::default(),
        sensor_bus: Default::default(),
    };

    // Tire model under test
    let mut tire_manager = TireManager::new();
    tire_manager.tire_constants.clear();
    tire_manager.add_tire(tire_constants);

    // 1) Lateral force vs slip angle (with zero slip ratio)
    let mut alphas = Vec::new();
    let mut fy = Vec::new();
    // Sweep from -90 deg to +90 deg
    for alpha_deg in (-90..=90).map(|d| d as f64) {
        let alpha = alpha_deg.to_radians();

        // Configure wheel state for this alpha with zero slip ratio
        let wheel = &mut state.true_state.wheel_states[0];
        wheel.longitudinal_translational_velocity = vx;
        wheel.lateral_translational_velocity = (alpha.tan()) * vx;
        // Set omega for zero slip ratio: s = (omega*R - vx)/|vx| = 0 => omega = vx/R
        wheel.driving_angular_velocity = vx / wheel_radius;
        wheel.tire.tire_load = tire_load;

        let ctx = SimContext { dt: 0.0, t: 0.0 };
        tire_manager.step_physics(ctx, &mut state);

        alphas.push(alpha_deg);
        fy.push(state.true_state.wheel_states[0].tire.lateral_force);
    }

    draw_series(
        "lateral_vs_slip_angle.png",
        "Lateral Force vs Slip Angle",
        "Slip Angle [deg]",
        "Lateral Force Fy [N]",
        &alphas,
        &fy,
    )?;

    // 2) Longitudinal force vs slip ratio (with zero slip angle)
    let mut kappas = Vec::new();
    let mut fx = Vec::new();

    let n = 301;
    for i in 0..n {
        let kappa = -0.5 + 1.0 * (i as f64) / ((n - 1) as f64);

        // Configure wheel state for this slip ratio with zero slip angle
        let wheel = &mut state.true_state.wheel_states[0];
        wheel.longitudinal_translational_velocity = vx;
        wheel.lateral_translational_velocity = 0.0;
        // s = (omega*R - vx)/|vx| => omega = (s + 1)*vx/R
        wheel.driving_angular_velocity = (kappa + 1.0) * vx / wheel_radius;
        wheel.tire.tire_load = tire_load;

        let ctx = SimContext { dt: 0.0, t: 0.0 };
        tire_manager.step_physics(ctx, &mut state);

        kappas.push(kappa);
        fx.push(state.true_state.wheel_states[0].tire.longitudinal_force);
    }

    draw_series(
        "longitudinal_vs_slip_ratio.png",
        "Longitudinal Force vs Slip Ratio",
        "Slip Ratio [-]",
        "Longitudinal Force Fx [N]",
        &kappas,
        &fx,
    )?;

    println!(
        "Wrote plots: lateral_vs_slip_angle.png, longitudinal_vs_slip_ratio.png"
    );

    Ok(())
}

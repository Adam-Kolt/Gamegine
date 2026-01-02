//! Mechanical Link - Couples motors to mechanical loads
//!
//! Provides abstractions for:
//! - Gear ratios and radius conversions (rotational to linear)
//! - Efficiency losses in power transfer
//! - Friction modeling (Coulomb, viscous, combined)
//! - Reflected inertia calculations

use serde::{Deserialize, Serialize};

/// Friction model for mechanical links
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FrictionModel {
    /// No friction
    None,
    /// Coulomb (dry) friction with static and kinetic coefficients
    Coulomb {
        /// Static friction coefficient
        static_coeff: f64,
        /// Kinetic friction coefficient  
        kinetic_coeff: f64,
        /// Normal force (N) for friction calculation
        normal_force: f64,
    },
    /// Viscous friction (proportional to velocity)
    Viscous {
        /// Damping coefficient (N·s/m for linear, N·m·s/rad for rotational)
        damping: f64,
    },
    /// Combined Coulomb + viscous friction
    Combined {
        /// Static friction coefficient
        static_coeff: f64,
        /// Kinetic friction coefficient
        kinetic_coeff: f64,
        /// Normal force (N)
        normal_force: f64,
        /// Viscous damping coefficient
        viscous_damping: f64,
    },
}

impl Default for FrictionModel {
    fn default() -> Self {
        FrictionModel::None
    }
}

impl FrictionModel {
    /// Compute friction force/torque given velocity
    /// 
    /// For Coulomb friction, uses a small velocity threshold to avoid
    /// discontinuity at zero velocity (stiction zone).
    pub fn compute(&self, velocity: f64) -> f64 {
        const STICTION_THRESHOLD: f64 = 0.001;
        
        match self {
            FrictionModel::None => 0.0,
            
            FrictionModel::Coulomb { static_coeff, kinetic_coeff, normal_force } => {
                if velocity.abs() < STICTION_THRESHOLD {
                    // In stiction zone - friction opposes motion up to static limit
                    // (Actual stiction would require knowing applied force)
                    0.0
                } else {
                    // Kinetic friction opposes motion
                    -kinetic_coeff * normal_force * velocity.signum()
                }
            }
            
            FrictionModel::Viscous { damping } => {
                -damping * velocity
            }
            
            FrictionModel::Combined { 
                static_coeff: _, 
                kinetic_coeff, 
                normal_force, 
                viscous_damping 
            } => {
                let coulomb = if velocity.abs() < STICTION_THRESHOLD {
                    0.0
                } else {
                    -kinetic_coeff * normal_force * velocity.signum()
                };
                let viscous = -viscous_damping * velocity;
                coulomb + viscous
            }
        }
    }
    
    /// Get maximum static friction force (for stiction calculations)
    pub fn max_static_friction(&self) -> f64 {
        match self {
            FrictionModel::None => 0.0,
            FrictionModel::Coulomb { static_coeff, normal_force, .. } => {
                static_coeff * normal_force
            }
            FrictionModel::Viscous { .. } => 0.0,
            FrictionModel::Combined { static_coeff, normal_force, .. } => {
                static_coeff * normal_force
            }
        }
    }
}

/// Configuration for a mechanical link
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LinkConfig {
    /// Gear ratio (output/input). E.g., 20.0 for 20:1 reduction
    /// Motor torque is multiplied by this to get output torque
    /// Motor velocity is divided by this to get output velocity
    pub gear_ratio: f64,
    
    /// Conversion radius for rotational-to-linear motion (m)
    /// Set to 0.0 for pure rotational output
    /// For drum/pulley: output_velocity = motor_velocity / gear_ratio * radius
    pub radius: f64,
    
    /// Power transfer efficiency (0.0 to 1.0)
    /// Accounts for gear mesh losses, bearing friction, etc.
    pub efficiency: f64,
    
    /// Load inertia - kg·m² for rotational, kg for linear
    pub load_inertia: f64,
    
    /// Friction model
    pub friction: FrictionModel,
}

impl Default for LinkConfig {
    fn default() -> Self {
        LinkConfig {
            gear_ratio: 1.0,
            radius: 0.0,
            efficiency: 1.0,
            load_inertia: 1.0,
            friction: FrictionModel::None,
        }
    }
}

impl LinkConfig {
    /// Create a new link configuration
    pub fn new() -> Self {
        Self::default()
    }
    
    /// Set gear ratio (builder pattern)
    pub fn with_gear_ratio(mut self, ratio: f64) -> Self {
        self.gear_ratio = ratio;
        self
    }
    
    /// Set radius for rotational-to-linear conversion
    pub fn with_radius(mut self, radius: f64) -> Self {
        self.radius = radius;
        self
    }
    
    /// Set efficiency
    pub fn with_efficiency(mut self, efficiency: f64) -> Self {
        self.efficiency = efficiency.clamp(0.0, 1.0);
        self
    }
    
    /// Set load inertia
    pub fn with_load_inertia(mut self, inertia: f64) -> Self {
        self.load_inertia = inertia;
        self
    }
    
    /// Set friction model
    pub fn with_friction(mut self, friction: FrictionModel) -> Self {
        self.friction = friction;
        self
    }
}

/// Represents a rotating body that can be connected via a link
/// 
/// Both sides of a link are treated symmetrically as rotating bodies
/// that can apply torque and have inertia.
#[derive(Debug, Clone, Copy, Default)]
pub struct RotatingBody {
    /// Angular velocity (rad/s)
    pub velocity: f64,
    /// Applied torque from this body (N·m)
    pub torque: f64,
    /// Moment of inertia (kg·m²)
    pub inertia: f64,
}

impl RotatingBody {
    pub fn new(inertia: f64) -> Self {
        Self {
            velocity: 0.0,
            torque: 0.0,
            inertia,
        }
    }
    
    pub fn with_velocity(mut self, velocity: f64) -> Self {
        self.velocity = velocity;
        self
    }
    
    pub fn with_torque(mut self, torque: f64) -> Self {
        self.torque = torque;
        self
    }
}

/// Result of stepping a linked system
#[derive(Debug, Clone, Copy)]
pub struct LinkStepResult {
    /// Acceleration of body A (rad/s²)
    pub accel_a: f64,
    /// Acceleration of body B (rad/s² in B's frame, i.e., after gear ratio)
    pub accel_b: f64,
    /// Net torque at body A
    pub net_torque_a: f64,
    /// Net torque at body B (in B's frame)
    pub net_torque_b: f64,
}

/// Mechanical link that couples two rotating bodies
/// 
/// Handles force/torque transfer, velocity coupling, friction, and inertia.
/// Both connected bodies are treated symmetrically - each can apply torque
/// and each has inertia. The link couples them through a gear ratio.
/// 
/// Convention: Body A is the "input" side, Body B is the "output" side
/// - gear_ratio = velocity_A / velocity_B (e.g., 20.0 means A spins 20x faster than B)
/// - Torque transfers as: torque_B = torque_A * gear_ratio * efficiency
#[derive(Debug, Clone)]
pub struct MechanicalLink {
    pub config: LinkConfig,
}

impl MechanicalLink {
    /// Create a new mechanical link
    pub fn new(config: LinkConfig) -> Self {
        Self { config }
    }
    
    /// Returns true if this link converts rotational to linear motion
    pub fn is_linear_output(&self) -> bool {
        self.config.radius > 0.0
    }
    
    // === Force/Torque Transfer ===
    
    /// Transfer torque from A to B (through gearing)
    /// 
    /// For rotational: torque_B = torque_A * gear_ratio * efficiency
    /// For linear output: force_B = torque_A * gear_ratio * efficiency / radius
    pub fn torque_a_to_b(&self, torque_a: f64) -> f64 {
        let output_torque = torque_a * self.config.gear_ratio * self.config.efficiency;
        
        if self.is_linear_output() {
            output_torque / self.config.radius
        } else {
            output_torque
        }
    }
    
    /// Transfer torque from B to A (reverse direction through gearing)
    /// 
    /// Torque from B appears at A reduced by gear ratio
    pub fn torque_b_to_a(&self, torque_b: f64) -> f64 {
        let input_torque = if self.is_linear_output() {
            torque_b * self.config.radius
        } else {
            torque_b
        };
        input_torque / self.config.gear_ratio * self.config.efficiency
    }
    
    // Aliases for clarity
    pub fn motor_to_load_force(&self, motor_torque: f64) -> f64 {
        self.torque_a_to_b(motor_torque)
    }
    
    // === Velocity Coupling ===
    
    /// Convert velocity from A to B
    /// 
    /// For rotational: velocity_B = velocity_A / gear_ratio
    /// For linear: velocity_B = velocity_A / gear_ratio * radius
    pub fn velocity_a_to_b(&self, velocity_a: f64) -> f64 {
        if self.is_linear_output() {
            velocity_a / self.config.gear_ratio * self.config.radius
        } else {
            velocity_a / self.config.gear_ratio
        }
    }
    
    /// Convert velocity from B to A
    pub fn velocity_b_to_a(&self, velocity_b: f64) -> f64 {
        if self.is_linear_output() {
            velocity_b / self.config.radius * self.config.gear_ratio
        } else {
            velocity_b * self.config.gear_ratio
        }
    }
    
    // Aliases
    pub fn load_to_motor_velocity(&self, load_velocity: f64) -> f64 {
        self.velocity_b_to_a(load_velocity)
    }
    
    pub fn motor_to_load_velocity(&self, motor_velocity: f64) -> f64 {
        self.velocity_a_to_b(motor_velocity)
    }
    
    // === Friction ===
    
    /// Compute friction force/torque at B (the load side)
    pub fn compute_friction(&self, velocity_b: f64) -> f64 {
        self.config.friction.compute(velocity_b)
    }
    
    // === Inertia ===
    
    /// Reflect inertia from A to B's frame
    /// 
    /// For rotational: J_reflected = J_A / gear_ratio²
    /// For linear: m_reflected = J_A / (gear_ratio / radius)²
    pub fn inertia_a_to_b(&self, inertia_a: f64) -> f64 {
        if self.is_linear_output() {
            inertia_a / (self.config.gear_ratio / self.config.radius).powi(2)
        } else {
            inertia_a / self.config.gear_ratio.powi(2)
        }
    }
    
    /// Reflect inertia from B to A's frame
    pub fn inertia_b_to_a(&self, inertia_b: f64) -> f64 {
        if self.is_linear_output() {
            inertia_b * (self.config.gear_ratio / self.config.radius).powi(2)
        } else {
            inertia_b * self.config.gear_ratio.powi(2)
        }
    }
    
    // Backwards-compatible alias
    pub fn reflected_motor_inertia(&self, motor_inertia: f64) -> f64 {
        self.inertia_b_to_a(motor_inertia)
    }
    
    /// Total effective inertia at B
    pub fn total_effective_inertia(&self, inertia_a: f64) -> f64 {
        self.config.load_inertia + self.inertia_a_to_b(inertia_a)
    }
    
    // === Coupled System Dynamics ===
    
    /// Step the coupled system given both bodies' states
    /// 
    /// Both bodies can apply torque, and both contribute inertia.
    /// The system is solved by summing torques in one reference frame.
    /// 
    /// # Arguments
    /// * `body_a` - State of body A (e.g., motor)
    /// * `body_b` - State of body B (e.g., load)  
    /// * `external_force_b` - Additional forces on B (e.g., gravity)
    pub fn step_coupled(
        &self,
        body_a: &RotatingBody,
        body_b: &RotatingBody,
        external_force_b: f64,
    ) -> LinkStepResult {
        // Sum all torques in B's reference frame
        let torque_from_a = self.torque_a_to_b(body_a.torque);
        let torque_from_b = body_b.torque;
        let friction = self.compute_friction(body_b.velocity);
        
        let net_torque_b = torque_from_a + torque_from_b + friction + external_force_b;
        
        // Total inertia in B's frame
        let inertia_a_at_b = self.inertia_a_to_b(body_a.inertia);
        let total_inertia_b = body_b.inertia + inertia_a_at_b;
        
        // Acceleration in B's frame
        let accel_b = net_torque_b / total_inertia_b;
        
        // Convert back to A's frame
        let accel_a = self.velocity_b_to_a(accel_b);
        let net_torque_a = self.torque_b_to_a(net_torque_b);
        
        LinkStepResult {
            accel_a,
            accel_b,
            net_torque_a,
            net_torque_b,
        }
    }
    
    /// Simple step: compute load acceleration given motor torque
    /// 
    /// Backwards-compatible API for simpler use cases.
    pub fn compute_load_acceleration(
        &self,
        motor_torque: f64,
        motor_inertia: f64,
        load_velocity: f64,
        external_force: f64,
    ) -> (f64, f64) {
        let drive_force = self.motor_to_load_force(motor_torque);
        let friction_force = self.compute_friction(load_velocity);
        let net_force = drive_force + friction_force + external_force;
        
        let total_inertia = self.total_effective_inertia(motor_inertia);
        let acceleration = net_force / total_inertia;
        
        (acceleration, net_force)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_gear_ratio_torque_transfer() {
        let link = MechanicalLink::new(LinkConfig {
            gear_ratio: 20.0,
            radius: 0.0, // rotational output
            efficiency: 1.0,
            ..Default::default()
        });
        
        // 1 Nm motor torque * 20:1 gear ratio = 20 Nm output
        assert!((link.motor_to_load_force(1.0) - 20.0).abs() < 1e-10);
    }
    
    #[test]
    fn test_gear_ratio_with_efficiency() {
        let link = MechanicalLink::new(LinkConfig {
            gear_ratio: 10.0,
            radius: 0.0,
            efficiency: 0.9, // 90% efficient
            ..Default::default()
        });
        
        // 1 Nm * 10 * 0.9 = 9 Nm
        assert!((link.motor_to_load_force(1.0) - 9.0).abs() < 1e-10);
    }
    
    #[test]
    fn test_linear_output_force() {
        let link = MechanicalLink::new(LinkConfig {
            gear_ratio: 20.0,
            radius: 0.025, // 2.5cm drum
            efficiency: 1.0,
            ..Default::default()
        });
        
        // 1 Nm * 20 / 0.025m = 800 N
        assert!((link.motor_to_load_force(1.0) - 800.0).abs() < 1e-10);
    }
    
    #[test]
    fn test_velocity_coupling_rotational() {
        let link = MechanicalLink::new(LinkConfig {
            gear_ratio: 10.0,
            radius: 0.0,
            ..Default::default()
        });
        
        // Load at 10 rad/s -> motor at 100 rad/s
        assert!((link.load_to_motor_velocity(10.0) - 100.0).abs() < 1e-10);
        
        // Motor at 100 rad/s -> load at 10 rad/s
        assert!((link.motor_to_load_velocity(100.0) - 10.0).abs() < 1e-10);
    }
    
    #[test]
    fn test_velocity_coupling_linear() {
        let link = MechanicalLink::new(LinkConfig {
            gear_ratio: 20.0,
            radius: 0.1, // 10cm radius
            ..Default::default()
        });
        
        // Load at 1 m/s -> motor at 1/0.1*20 = 200 rad/s
        assert!((link.load_to_motor_velocity(1.0) - 200.0).abs() < 1e-10);
        
        // Motor at 200 rad/s -> load at 1 m/s
        assert!((link.motor_to_load_velocity(200.0) - 1.0).abs() < 1e-10);
    }
    
    #[test]
    fn test_viscous_friction() {
        let link = MechanicalLink::new(LinkConfig {
            friction: FrictionModel::Viscous { damping: 0.5 },
            ..Default::default()
        });
        
        // Friction should oppose motion
        assert!((link.compute_friction(10.0) - (-5.0)).abs() < 1e-10);
        assert!((link.compute_friction(-10.0) - 5.0).abs() < 1e-10);
    }
    
    #[test]
    fn test_coulomb_friction() {
        let link = MechanicalLink::new(LinkConfig {
            friction: FrictionModel::Coulomb {
                static_coeff: 0.5,
                kinetic_coeff: 0.3,
                normal_force: 100.0,
            },
            ..Default::default()
        });
        
        // At velocity, kinetic friction = 0.3 * 100 = 30 N opposing motion
        assert!((link.compute_friction(1.0) - (-30.0)).abs() < 1e-10);
        assert!((link.compute_friction(-1.0) - 30.0).abs() < 1e-10);
        
        // At rest, no friction (stiction would require knowing applied force)
        assert!((link.compute_friction(0.0)).abs() < 1e-10);
    }
    
    #[test]
    fn test_reflected_inertia_rotational() {
        let link = MechanicalLink::new(LinkConfig {
            gear_ratio: 10.0,
            radius: 0.0,
            ..Default::default()
        });
        
        // J_reflected = J_motor * gear_ratio²
        // 0.001 kg·m² * 10² = 0.1 kg·m²
        assert!((link.reflected_motor_inertia(0.001) - 0.1).abs() < 1e-10);
    }
    
    #[test]
    fn test_reflected_inertia_linear() {
        let link = MechanicalLink::new(LinkConfig {
            gear_ratio: 20.0,
            radius: 0.1,
            ..Default::default()
        });
        
        // m_reflected = J_motor * (gear_ratio / radius)²
        // 0.0001 kg·m² * (20/0.1)² = 0.0001 * 40000 = 4 kg
        assert!((link.reflected_motor_inertia(0.0001) - 4.0).abs() < 1e-10);
    }
    
    #[test]
    fn test_total_effective_inertia() {
        let link = MechanicalLink::new(LinkConfig {
            gear_ratio: 10.0,
            radius: 0.0,
            load_inertia: 0.5,
            ..Default::default()
        });
        
        // inertia_a_to_b: 0.001 / 10² = 0.00001
        // total = 0.5 + 0.00001 = 0.50001 kg·m²
        assert!((link.total_effective_inertia(0.001) - 0.50001).abs() < 1e-10);
    }
}

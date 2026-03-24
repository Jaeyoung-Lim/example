#include <rl_tools/rl/environments/environments.h>
template <typename T>
struct FixedwingParameters {
    // Physical parameters (from PX4 advanced_plane)
    constexpr static T mass = 1.5;              // kg
    constexpr static T wing_area = 0.34;        // m^2 (S_ref in AVL)
    constexpr static T wingspan = 1.49;         // m (computed from AR * area)
    constexpr static T chord = 0.22;            // Mean aerodynamic chord (m)
    
    // AVL Aerodynamic coefficients - Zero condition (from PX4 advanced_plane)
    constexpr static T CL0 = 0.15188;           // Lift coefficient at zero AoA
    constexpr static T CD0 = 0.029;             // Parasitic drag coefficient
    constexpr static T Cem0 = 0.075;            // Pitching moment at zero AoA
    
    // AVL Derivatives - Angle of attack (alpha) [1/rad]
    constexpr static T CLa = 5.015;             // dCL/dalpha
    constexpr static T CYa = 0.0;               // dCY/dalpha
    constexpr static T Cella = 0.0;             // dCl/dalpha (roll moment)
    constexpr static T Cema = -0.463966;        // dCm/dalpha (pitch moment, stability)
    constexpr static T Cena = 0.0;              // dCn/dalpha (yaw moment)
    
    // AVL Derivatives - Sideslip angle (beta) [1/rad]
    constexpr static T CLb = 0.0;               // dCL/dbeta
    constexpr static T CYb = -0.258244;         // dCY/dbeta (weathercock stability)
    constexpr static T Cellb = -0.039250;       // dCl/dbeta (dihedral effect)
    constexpr static T Cemb = 0.0;              // dCm/dbeta
    constexpr static T Cenb = 0.100826;         // dCn/dbeta (directional stability)
    
    // AVL Derivatives - Roll rate p (non-dimensional: p*b/2V) [dimensionless]
    constexpr static T CLp = 0.0;               // dCL/d(pb/2V)
    constexpr static T CYp = 0.065861;          // dCY/d(pb/2V)
    constexpr static T Cellp = -0.487407;       // dCl/d(pb/2V) (roll damping)
    constexpr static T Cemp = 0.0;              // dCm/d(pb/2V)
    constexpr static T Cenp = -0.040416;        // dCn/d(pb/2V)
    constexpr static T CDp = 0.0;               // dCD/d(pb/2V)
    
    // AVL Derivatives - Pitch rate q (non-dimensional: q*c/2V) [dimensionless]
    constexpr static T CLq = 7.971792;          // dCL/d(qc/2V)
    constexpr static T CYq = 0.0;               // dCY/d(qc/2V)
    constexpr static T Cellq = 0.0;             // dCl/d(qc/2V)
    constexpr static T Cemq = -12.140140;       // dCm/d(qc/2V) (pitch damping)
    constexpr static T Cenq = 0.0;              // dCn/d(qc/2V)
    constexpr static T CDq = 0.055166;          // dCD/d(qc/2V)
    
    // AVL Derivatives - Yaw rate r (non-dimensional: r*b/2V) [dimensionless]
    constexpr static T CLr = 0.0;               // dCL/d(rb/2V)
    constexpr static T CYr = 0.230299;          // dCY/d(rb/2V)
    constexpr static T Cellr = 0.078165;        // dCl/d(rb/2V)
    constexpr static T Cemr = 0.0;              // dCm/d(rb/2V)
    constexpr static T Cenr = -0.089947;        // dCn/d(rb/2V) (yaw damping)
    constexpr static T CDr = 0.0;               // dCD/d(rb/2V)
    
    // Control surface derivatives [1/degree] (AVL convention from PX4)
    // Left aileron (elevon) - lines 523-532 in SDF
    constexpr static T CL_left_aileron = -0.011940;   // dCL/d(left_aileron) per degree
    constexpr static T CD_left_aileron = -0.000059;   // dCD/d(left_aileron) per degree
    constexpr static T CY_left_aileron = 0.000171;    // dCY/d(left_aileron) per degree
    constexpr static T Cell_left_aileron = -0.003331; // dCl/d(left_aileron) per degree (roll control)
    constexpr static T Cem_left_aileron = 0.001498;   // dCm/d(left_aileron) per degree
    constexpr static T Cen_left_aileron = -0.000057;  // dCn/d(left_aileron) per degree
    
    // Right aileron (elevon) - lines 534-543 in SDF
    constexpr static T CL_right_aileron = -0.011940;  // dCL/d(right_aileron) per degree
    constexpr static T CD_right_aileron = -0.000059;  // dCD/d(right_aileron) per degree
    constexpr static T CY_right_aileron = -0.000171;  // dCY/d(right_aileron) per degree
    constexpr static T Cell_right_aileron = 0.003331; // dCl/d(right_aileron) per degree (roll control)
    constexpr static T Cem_right_aileron = 0.001498;  // dCm/d(right_aileron) per degree
    constexpr static T Cen_right_aileron = 0.000057;  // dCn/d(right_aileron) per degree
    
    // Elevator - lines 545-554 in SDF (note direction=-1)
    constexpr static T CL_elevator = -0.010696;       // dCL/d(elevator) per degree (with direction)
    constexpr static T CD_elevator = -0.000274;       // dCD/d(elevator) per degree (with direction)
    constexpr static T CY_elevator = 0.0;             // dCY/d(elevator) per degree
    constexpr static T Cell_elevator = 0.0;           // dCl/d(elevator) per degree
    constexpr static T Cem_elevator = 0.025798;       // dCm/d(elevator) per degree (pitch control, with direction)
    constexpr static T Cen_elevator = 0.0;            // dCn/d(elevator) per degree
    
    // Rudder - lines 556-565 in SDF
    constexpr static T CL_rudder = 0.0;               // dCL/d(rudder) per degree
    constexpr static T CD_rudder = 0.0;               // dCD/d(rudder) per degree
    constexpr static T CY_rudder = -0.003913;         // dCY/d(rudder) per degree (side force)
    constexpr static T Cell_rudder = -0.000257;       // dCl/d(rudder) per degree
    constexpr static T Cem_rudder = 0.0;              // dCm/d(rudder) per degree
    constexpr static T Cen_rudder = 0.001613;         // dCn/d(rudder) per degree (yaw control)
    
    // Control surface limits (degrees) - from SDF lines 352-353, etc.
    constexpr static T max_aileron = 30.4;      // degrees (0.53 rad)
    constexpr static T max_elevator = 30.4;     // degrees (0.53 rad)
    constexpr static T max_rudder = 30.4;       // degrees (0.53 rad)
    
    // Thrust parameters (from motor model lines 569-587)
    // Max thrust = motor_constant * max_speed^2 = 8.54858e-06 * 3500^2 ≈ 104.6 N
    constexpr static T max_thrust = 104.6;      // N (maximum thrust)
    constexpr static T thrust_offset_x = 0.3;   // m (propeller position, line 104)
    constexpr static T thrust_offset_z = 0.0;   // m (at CG height)
    
    // Stall model parameters (from SDF lines 509-512)
    constexpr static T alpha_stall = 0.3391428111; // Stall angle (rad) ~19.4 degrees
    constexpr static T Cema_stall = 0.0;        // Post-stall pitch moment slope
    constexpr static T M_stall = 15.0;          // Stall blending parameter
    
    // Drag model parameters (from SDF lines 482-483)
    constexpr static T AR = 6.5;                // Aspect ratio (b^2/S)
    constexpr static T eff = 0.97;              // Oswald efficiency factor
    constexpr static T CD_fp_k1 = 0.5;          // Flat plate drag sigmoid k1
    constexpr static T CD_fp_k2 = -0.3;         // Flat plate drag sigmoid k2
    
    // Inertia properties (from SDF lines 11-16)
    constexpr static T Ixx = 0.197563;          // kg*m^2 (roll inertia)
    constexpr static T Iyy = 0.1458929;         // kg*m^2 (pitch inertia)
    constexpr static T Izz = 0.1477;            // kg*m^2 (yaw inertia)
    constexpr static T Ixz = 0.0;               // kg*m^2 (cross product of inertia)
    
    // Environment (from SDF line 516)
    constexpr static T rho = 1.2041;            // Air density (kg/m^3)
    constexpr static T g = 9.81;                // Gravity (m/s^2)
    
    // Simulation
    constexpr static T dt = 0.02;               // 50 Hz
    
    // Task parameters
    constexpr static T max_distance = 500.0;    // Maximum distance from origin
    constexpr static T min_velocity = 8.0;      // Minimum airspeed before stall
    constexpr static T max_velocity = 40.0;     // Maximum safe airspeed

    // Initial state parameters
    constexpr static T INITIAL_STATE_EXTENT = 100.0;
    constexpr static T INITIAL_STATE_MIN_SPEED = 12.0;
    constexpr static T INITIAL_STATE_MAX_SPEED = 20.0;
};

template <typename T_T, typename T_TI, typename T_PARAMETERS = FixedwingParameters<T_T>>
struct FixedwingSpecification{
    using T = T_T;
    using TI = T_TI;
    using PARAMETERS = T_PARAMETERS;
};

template <typename T>
struct FixedwingState {
    // Position (NED frame)
    T x, y, z;                 // z is down (NED convention)
    
    // Velocity in body frame (FRD: Forward-Right-Down)
    T u, v, w;                 // Body-frame velocities
    
    // Attitude quaternion (NED to body frame)
    // q = [qw, qx, qy, qz] where qw is scalar part
    T qw, qx, qy, qz;
    
    // Angular rates (body frame FRD)
    T p, q_rate, r;           // Roll, pitch, yaw rates (renamed q_rate to avoid confusion with quaternion)
    
    // Velocity reference to track (in NED frame)
    T V_ref_x, V_ref_y, V_ref_z;
    
    // Time elapsed in episode (for time-varying references)
    T time;
};

template <typename TI>
struct FixedwingFourierObservation{
    // Observation: u, v, w (body velocities), qw, qx, qy, qz (attitude), 
    //              V_error_x, V_error_y, V_error_z (velocity error in NED),
    //              V_ref_x, V_ref_y, V_ref_z (velocity reference in NED)
    static constexpr TI DIM = 13;
};

template <typename T_SPEC>
struct Fixedwing: rl_tools::rl::environments::Environment<typename T_SPEC::T, typename T_SPEC::TI>{
    using SPEC = T_SPEC;
    using T = typename SPEC::T;
    using TI = typename SPEC::TI;
    using Parameters = typename SPEC::PARAMETERS;
    using State = FixedwingState<T>;
    using Observation = FixedwingFourierObservation<TI>;
    using ObservationPrivileged = Observation;
    static constexpr TI OBSERVATION_DIM = 13;
    static constexpr TI ACTION_DIM = 5;  // [throttle, left_aileron, right_aileron, elevator, rudder] normalized [-1,1]
    static constexpr TI EPISODE_STEP_LIMIT = 500;
};

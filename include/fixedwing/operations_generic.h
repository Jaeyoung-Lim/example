#include <rl_tools/rl/environments/operations_generic.h>

// Helper functions
template <typename T>
T clip(T x, T min, T max){
    x = x < min ? min : (x > max ? max : x);
    return x;
}

template <typename DEVICE, typename T>
T f_mod_python(const DEVICE& dev, T a, T b){
    return a - b * rl_tools::math::floor(dev, a / b);
}

template <typename DEVICE, typename T>
T angle_normalize(const DEVICE& dev, T x){
    return f_mod_python(dev, (x + rl_tools::math::PI<T>), (2 * rl_tools::math::PI<T>)) - rl_tools::math::PI<T>;
}

namespace rl_tools{
    template<typename DEVICE, typename SPEC>
    void malloc(DEVICE& device, Fixedwing<SPEC>& env){}
    
    template<typename DEVICE, typename SPEC>
    void free(DEVICE& device, Fixedwing<SPEC>& env){}
    
    template<typename DEVICE, typename SPEC>
    void init(DEVICE& device, Fixedwing<SPEC>& env){}
    
    template<typename DEVICE, typename SPEC>
    void initial_parameters(DEVICE& device, const Fixedwing<SPEC>& env, typename Fixedwing<SPEC>::Parameters& parameters){ }
    
    template<typename DEVICE, typename SPEC, typename RNG>
    void sample_initial_parameters(DEVICE& device, const Fixedwing<SPEC>& env, typename Fixedwing<SPEC>::Parameters& parameters, RNG& rng){ }
    
    template<typename DEVICE, typename SPEC>
    void initial_state(DEVICE& device, const Fixedwing<SPEC>& env, const typename Fixedwing<SPEC>::Parameters& parameters, typename Fixedwing<SPEC>::State& state){
        using T = typename SPEC::T;
        
        state.x = T(0.0);
        state.y = T(0.0);
        state.z = T(0.0);
        
        // Initial velocity in body frame (forward flight at 15 m/s)
        state.u = T(15.0);
        state.v = T(0.0);
        state.w = T(0.0);
        
        // Level flight attitude
        state.qw = T(1.0);
        state.qx = T(0.0);
        state.qy = T(0.0);
        state.qz = T(0.0);
        
        // Zero initial rates
        state.p = T(0.0);
        state.q_rate = T(0.0);
        state.r = T(0.0);
        
        // Initial velocity reference (forward flight)
        state.V_ref_x = T(15.0);
        state.V_ref_y = T(0.0);
        state.V_ref_z = T(0.0);
        state.time = T(0.0);
    }
    
    template<typename DEVICE, typename SPEC, typename RNG>
    void sample_initial_state(DEVICE& device, const Fixedwing<SPEC>& env, const typename Fixedwing<SPEC>::Parameters& parameters, typename Fixedwing<SPEC>::State& state, RNG& rng){
        using T = typename SPEC::T;
        using PARAMS = typename SPEC::PARAMETERS;
        
        // Random initial position
        state.x = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), T(-50.0), T(50.0), rng);
        state.y = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), T(-50.0), T(50.0), rng);
        state.z = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), T(-30.0), T(30.0), rng);
        
        // Sample random velocity reference first
        T ref_magnitude = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), T(12.0), T(22.0), rng);
        T ref_yaw = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), T(-rl_tools::math::PI<T>), T(rl_tools::math::PI<T>), rng);
        T ref_climb_rate = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), T(-2.0), T(2.0), rng);
        
        state.V_ref_x = ref_magnitude * rl_tools::math::cos(device.math, ref_yaw);
        state.V_ref_y = ref_magnitude * rl_tools::math::sin(device.math, ref_yaw);
        state.V_ref_z = ref_climb_rate;
        
        // Compute desired flight path angle from reference
        T ref_horizontal = rl_tools::math::sqrt(device.math, state.V_ref_x * state.V_ref_x + state.V_ref_y * state.V_ref_y);
        T ref_flight_path_angle = rl_tools::math::atan2(device.math, state.V_ref_z, ref_horizontal);
        
        // Align body x-axis with velocity reference direction (with small perturbations)
        // This ensures small initial angle of attack and sideslip
        
        // Base yaw from reference direction
        T yaw = rl_tools::math::atan2(device.math, state.V_ref_y, state.V_ref_x);
        
        // Base pitch from flight path angle (with small perturbation for angle of attack)
        // Angle of attack is typically small (0-10 degrees) in normal flight
        T alpha_init = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), T(-0.1), T(0.15), rng); // -5.7° to +8.6°
        T pitch = ref_flight_path_angle + alpha_init;
        
        // Small random roll
        T roll = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), T(-0.15), T(0.15), rng); // ±8.6°
        
        // Convert Euler to quaternion
        T cr = rl_tools::math::cos(device.math, roll * T(0.5));
        T sr = rl_tools::math::sin(device.math, roll * T(0.5));
        T cp = rl_tools::math::cos(device.math, pitch * T(0.5));
        T sp = rl_tools::math::sin(device.math, pitch * T(0.5));
        T cy = rl_tools::math::cos(device.math, yaw * T(0.5));
        T sy = rl_tools::math::sin(device.math, yaw * T(0.5));
        
        state.qw = cr * cp * cy + sr * sp * sy;
        state.qx = sr * cp * cy - cr * sp * sy;
        state.qy = cr * sp * cy + sr * cp * sy;
        state.qz = cr * cp * sy - sr * sp * cy;
        
        // Set body-frame velocities
        // Since body x-axis is now aligned with velocity direction, most velocity is forward (u)
        // with small lateral (v) and vertical (w) components
        T V_mag = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), PARAMS::INITIAL_STATE_MIN_SPEED, PARAMS::INITIAL_STATE_MAX_SPEED, rng);
        
        // Forward velocity dominates
        state.u = V_mag * rl_tools::math::cos(device.math, alpha_init);
        
        // Small sideslip (typically < 5 degrees)
        T beta_init = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), T(-0.08), T(0.08), rng); // ±4.6°
        state.v = V_mag * rl_tools::math::sin(device.math, beta_init);
        
        // Vertical velocity component from angle of attack
        state.w = V_mag * rl_tools::math::sin(device.math, alpha_init);
        
        // Small initial rates
        state.p = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), T(-0.05), T(0.05), rng);
        state.q_rate = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), T(-0.05), T(0.05), rng);
        state.r = random::uniform_real_distribution(typename DEVICE::SPEC::RANDOM(), T(-0.05), T(0.05), rng);
        
        state.time = T(0.0);
    }
    
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    typename SPEC::T step(DEVICE& device, const Fixedwing<SPEC>& env, const typename Fixedwing<SPEC>::Parameters& parameters, const typename Fixedwing<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, typename Fixedwing<SPEC>::State& next_state, RNG& rng) {
        static_assert(ACTION_SPEC::ROWS == 1);
        static_assert(ACTION_SPEC::COLS == 5);
        using T = typename SPEC::T;
        using PARAMS = typename SPEC::PARAMETERS;
        
        // Extract actions: [throttle, left_aileron, right_aileron, elevator, rudder]
        // Actions are assumed to be in range [-1, 1]
        T throttle_norm = clip(get(action, 0, 0), T(-1.0), T(1.0));
        T left_aileron_norm = clip(get(action, 0, 1), T(-1.0), T(1.0));
        T right_aileron_norm = clip(get(action, 0, 2), T(-1.0), T(1.0));
        T elevator_norm = clip(get(action, 0, 3), T(-1.0), T(1.0));
        T rudder_norm = clip(get(action, 0, 4), T(-1.0), T(1.0));
        
        // Convert to physical units
        T left_aileron_deg = left_aileron_norm * PARAMS::max_aileron;    // degrees
        T right_aileron_deg = right_aileron_norm * PARAMS::max_aileron;  // degrees
        T elevator_deg = elevator_norm * PARAMS::max_elevator;           // degrees
        T rudder_deg = rudder_norm * PARAMS::max_rudder;                 // degrees
        T thrust = (throttle_norm + T(1.0)) * T(0.5) * PARAMS::max_thrust; // Map [-1,1] to [0, max_thrust]
        
        T dt = PARAMS::dt;
        next_state = state;
        
        // Normalize quaternion
        T q_norm = rl_tools::math::sqrt(device.math, state.qw*state.qw + state.qx*state.qx + state.qy*state.qy + state.qz*state.qz);
        if (q_norm < T(1e-8)) q_norm = T(1.0);
        T qw = state.qw / q_norm;
        T qx = state.qx / q_norm;
        T qy = state.qy / q_norm;
        T qz = state.qz / q_norm;
        
        // Rotation matrix (NED to body)
        T R11 = qw*qw + qx*qx - qy*qy - qz*qz;
        T R12 = T(2.0) * (qx*qy - qw*qz);
        T R13 = T(2.0) * (qx*qz + qw*qy);
        T R21 = T(2.0) * (qx*qy + qw*qz);
        T R22 = qw*qw - qx*qx + qy*qy - qz*qz;
        T R23 = T(2.0) * (qy*qz - qw*qx);
        T R31 = T(2.0) * (qx*qz - qw*qy);
        T R32 = T(2.0) * (qy*qz + qw*qx);
        T R33 = qw*qw - qx*qx - qy*qy + qz*qz;
        
        // Body-frame velocities
        T u = state.u;
        T v = state.v;
        T w = state.w;
        T V_mag = rl_tools::math::sqrt(device.math, u*u + v*v + w*w);
        V_mag = rl_tools::math::max(device.math, V_mag, T(0.1)); // Avoid division by zero
        
        // Compute aerodynamic angles
        T alpha = rl_tools::math::atan2(device.math, w, u);  // Angle of attack
        T beta = rl_tools::math::atan2(device.math, v, rl_tools::math::sqrt(device.math, u*u + w*w));  // Sideslip angle (alternative formulation)
        
        // Non-dimensional body rates
        T span = PARAMS::wingspan;
        T p_hat = state.p * span / (T(2.0) * V_mag);
        T q_hat = state.q_rate * PARAMS::chord / (T(2.0) * V_mag);
        T r_hat = state.r * span / (T(2.0) * V_mag);
        
        // Stall blending function (sigmoid)
        T exp_pos = rl_tools::math::exp(device.math, -PARAMS::M_stall * (alpha - PARAMS::alpha_stall));
        T exp_neg = rl_tools::math::exp(device.math, PARAMS::M_stall * (alpha + PARAMS::alpha_stall));
        T sigma = (T(1.0) + exp_pos + exp_neg) / ((T(1.0) + exp_pos) * (T(1.0) + exp_neg));
        
        T sin_alpha = rl_tools::math::sin(device.math, alpha);
        T cos_alpha = rl_tools::math::cos(device.math, alpha);
        
        // === Compute Aerodynamic Coefficients (AVL Model) ===
        
        // Lift coefficient
        T CL_prestall = PARAMS::CL0 + PARAMS::CLa * alpha;
        T CL_poststall = T(2.0) * (alpha / rl_tools::math::abs(device.math, alpha)) * sin_alpha * sin_alpha * cos_alpha;
        T CL = (T(1.0) - sigma) * CL_prestall + sigma * CL_poststall;
        CL += PARAMS::CLb * beta;
        CL += PARAMS::CLp * p_hat + PARAMS::CLq * q_hat + PARAMS::CLr * r_hat;
        CL += PARAMS::CL_left_aileron * left_aileron_deg + PARAMS::CL_right_aileron * right_aileron_deg + 
              PARAMS::CL_elevator * elevator_deg + PARAMS::CL_rudder * rudder_deg;
        
        // Drag coefficient
        T CD_fp = T(2.0) / (T(1.0) + rl_tools::math::exp(device.math, PARAMS::CD_fp_k1 + PARAMS::CD_fp_k2 * PARAMS::AR));
        T CD_prestall = PARAMS::CD0 + (CL * CL) / (rl_tools::math::PI<T> * PARAMS::AR * PARAMS::eff);
        T CD_poststall = rl_tools::math::abs(device.math, CD_fp * (T(0.5) - T(0.5) * rl_tools::math::cos(device.math, T(2.0) * alpha)));
        T CD = (T(1.0) - sigma) * CD_prestall + sigma * CD_poststall;
        CD += PARAMS::CDp * p_hat + PARAMS::CDq * q_hat + PARAMS::CDr * r_hat;
        CD += PARAMS::CD_left_aileron * left_aileron_deg + PARAMS::CD_right_aileron * right_aileron_deg + 
              PARAMS::CD_elevator * elevator_deg + PARAMS::CD_rudder * rudder_deg;
        
        // Side force coefficient
        T CY = PARAMS::CYa * alpha + PARAMS::CYb * beta;
        CY += PARAMS::CYp * p_hat + PARAMS::CYq * q_hat + PARAMS::CYr * r_hat;
        CY += PARAMS::CY_left_aileron * left_aileron_deg + PARAMS::CY_right_aileron * right_aileron_deg + 
              PARAMS::CY_elevator * elevator_deg + PARAMS::CY_rudder * rudder_deg;
        
        // Roll moment coefficient
        T Cell = PARAMS::Cella * alpha + PARAMS::Cellb * beta;
        Cell += PARAMS::Cellp * p_hat + PARAMS::Cellq * q_hat + PARAMS::Cellr * r_hat;
        Cell += PARAMS::Cell_left_aileron * left_aileron_deg + PARAMS::Cell_right_aileron * right_aileron_deg + 
                PARAMS::Cell_elevator * elevator_deg + PARAMS::Cell_rudder * rudder_deg;
        
        // Pitch moment coefficient (with stall handling)
        T Cem;
        if (alpha > PARAMS::alpha_stall) {
            Cem = PARAMS::Cem0 + PARAMS::Cema * PARAMS::alpha_stall + PARAMS::Cema_stall * (alpha - PARAMS::alpha_stall);
        } else if (alpha < -PARAMS::alpha_stall) {
            Cem = PARAMS::Cem0 - PARAMS::Cema * PARAMS::alpha_stall + PARAMS::Cema_stall * (alpha + PARAMS::alpha_stall);
        } else {
            Cem = PARAMS::Cem0 + PARAMS::Cema * alpha;
        }
        Cem += PARAMS::Cemb * beta;
        Cem += PARAMS::Cemp * p_hat + PARAMS::Cemq * q_hat + PARAMS::Cemr * r_hat;
        Cem += PARAMS::Cem_left_aileron * left_aileron_deg + PARAMS::Cem_right_aileron * right_aileron_deg + 
               PARAMS::Cem_elevator * elevator_deg + PARAMS::Cem_rudder * rudder_deg;
        
        // Yaw moment coefficient
        T Cen = PARAMS::Cena * alpha + PARAMS::Cenb * beta;
        Cen += PARAMS::Cenp * p_hat + PARAMS::Cenq * q_hat + PARAMS::Cenr * r_hat;
        Cen += PARAMS::Cen_left_aileron * left_aileron_deg + PARAMS::Cen_right_aileron * right_aileron_deg + 
               PARAMS::Cen_elevator * elevator_deg + PARAMS::Cen_rudder * rudder_deg;
        
        // === Compute Forces and Moments ===
        
        // Dynamic pressure
        T q_dyn = T(0.5) * PARAMS::rho * V_mag * V_mag;
        T S = PARAMS::wing_area;
        
        // Aerodynamic forces in stability frame
        // Stability frame: x-axis aligned with velocity, z-axis in x-z plane of body frame
        T F_drag_stab = -CD * q_dyn * S;      // Along -velocity direction
        T F_side_stab = CY * q_dyn * S;       // Lateral force
        T F_lift_stab = -CL * q_dyn * S;      // Perpendicular to velocity (up)
        
        // Transform forces from stability to body frame
        // Stability frame is rotated by alpha about y-axis
        T F_x_aero = F_drag_stab * cos_alpha - F_lift_stab * sin_alpha;
        T F_y_aero = F_side_stab;
        T F_z_aero = F_drag_stab * sin_alpha + F_lift_stab * cos_alpha;
        
        // Add thrust (aligned with body x-axis)
        T F_x_body = F_x_aero + thrust;
        T F_y_body = F_y_aero;
        T F_z_body = F_z_aero;
        
        // Add gravity in body frame (transform from NED)
        T g_ned_z = PARAMS::g * PARAMS::mass;  // Weight (positive down in NED)
        F_x_body += R13 * g_ned_z;
        F_y_body += R23 * g_ned_z;
        F_z_body += R33 * g_ned_z;
        
        // Aerodynamic moments in body frame
        T L_aero = Cell * q_dyn * S * span;
        T M_aero = Cem * q_dyn * S * PARAMS::chord;
        T N_aero = Cen * q_dyn * S * span;
        
        // Thrust moment (if thrust line is offset from CG)
        T M_thrust = -thrust * PARAMS::thrust_offset_z;  // Pitch moment from thrust
        
        T L_total = L_aero;
        T M_total = M_aero + M_thrust;
        T N_total = N_aero;
        
        // === Integrate Dynamics ===
        
        // Linear accelerations in body frame (including Coriolis terms)
        T u_dot = F_x_body / PARAMS::mass - state.q_rate * w + state.r * v;
        T v_dot = F_y_body / PARAMS::mass - state.r * u + state.p * w;
        T w_dot = F_z_body / PARAMS::mass - state.p * v + state.q_rate * u;
        
        // Angular accelerations (Euler's equations for rigid body)
        T Ixx = PARAMS::Ixx;
        T Iyy = PARAMS::Iyy;
        T Izz = PARAMS::Izz;
        T Ixz = PARAMS::Ixz;
        
        // Simplified (neglecting Ixz for now, can be added later)
        T p_dot = (L_total + (Iyy - Izz) * state.q_rate * state.r) / Ixx;
        T q_dot = (M_total + (Izz - Ixx) * state.p * state.r) / Iyy;
        T r_dot = (N_total + (Ixx - Iyy) * state.p * state.q_rate) / Izz;
        
        // Integrate velocities (Forward Euler)
        next_state.u = u + u_dot * dt;
        next_state.v = v + v_dot * dt;
        next_state.w = w + w_dot * dt;
        
        // Integrate angular rates
        next_state.p = state.p + p_dot * dt;
        next_state.q_rate = state.q_rate + q_dot * dt;
        next_state.r = state.r + r_dot * dt;
        
        // Integrate quaternion (using average angular rates)
        T p_avg = (state.p + next_state.p) * T(0.5);
        T q_avg = (state.q_rate + next_state.q_rate) * T(0.5);
        T r_avg = (state.r + next_state.r) * T(0.5);
        
        T qw_dot = T(-0.5) * (qx * p_avg + qy * q_avg + qz * r_avg);
        T qx_dot = T(0.5) * (qw * p_avg + qy * r_avg - qz * q_avg);
        T qy_dot = T(0.5) * (qw * q_avg + qz * p_avg - qx * r_avg);
        T qz_dot = T(0.5) * (qw * r_avg + qx * q_avg - qy * p_avg);
        
        next_state.qw = qw + qw_dot * dt;
        next_state.qx = qx + qx_dot * dt;
        next_state.qy = qy + qy_dot * dt;
        next_state.qz = qz + qz_dot * dt;
        
        // Renormalize quaternion
        T q_norm_new = rl_tools::math::sqrt(device.math, 
            next_state.qw*next_state.qw + next_state.qx*next_state.qx + 
            next_state.qy*next_state.qy + next_state.qz*next_state.qz);
        if (q_norm_new > T(1e-8)) {
            next_state.qw /= q_norm_new;
            next_state.qx /= q_norm_new;
            next_state.qy /= q_norm_new;
            next_state.qz /= q_norm_new;
        }
        
        // Transform body velocities to NED frame for position update (using average quaternion)
        T qw_avg = (qw + next_state.qw) * T(0.5);
        T qx_avg = (qx + next_state.qx) * T(0.5);
        T qy_avg = (qy + next_state.qy) * T(0.5);
        T qz_avg = (qz + next_state.qz) * T(0.5);
        T q_avg_norm = rl_tools::math::sqrt(device.math, qw_avg*qw_avg + qx_avg*qx_avg + qy_avg*qy_avg + qz_avg*qz_avg);
        qw_avg /= q_avg_norm; qx_avg /= q_avg_norm; qy_avg /= q_avg_norm; qz_avg /= q_avg_norm;
        
        T R11_avg = qw_avg*qw_avg + qx_avg*qx_avg - qy_avg*qy_avg - qz_avg*qz_avg;
        T R12_avg = T(2.0) * (qx_avg*qy_avg - qw_avg*qz_avg);
        T R13_avg = T(2.0) * (qx_avg*qz_avg + qw_avg*qy_avg);
        T R21_avg = T(2.0) * (qx_avg*qy_avg + qw_avg*qz_avg);
        T R22_avg = qw_avg*qw_avg - qx_avg*qx_avg + qy_avg*qy_avg - qz_avg*qz_avg;
        T R23_avg = T(2.0) * (qy_avg*qz_avg - qw_avg*qx_avg);
        T R31_avg = T(2.0) * (qx_avg*qz_avg - qw_avg*qy_avg);
        T R32_avg = T(2.0) * (qy_avg*qz_avg + qw_avg*qx_avg);
        T R33_avg = qw_avg*qw_avg - qx_avg*qx_avg - qy_avg*qy_avg + qz_avg*qz_avg;
        
        T u_avg = (u + next_state.u) * T(0.5);
        T v_avg = (v + next_state.v) * T(0.5);
        T w_avg = (w + next_state.w) * T(0.5);
        
        T vx_ned = R11_avg * u_avg + R12_avg * v_avg + R13_avg * w_avg;
        T vy_ned = R21_avg * u_avg + R22_avg * v_avg + R23_avg * w_avg;
        T vz_ned = R31_avg * u_avg + R32_avg * v_avg + R33_avg * w_avg;
        
        // Integrate position
        next_state.x = state.x + vx_ned * dt;
        next_state.y = state.y + vy_ned * dt;
        next_state.z = state.z + vz_ned * dt;
        
        // Update time and keep reference constant
        next_state.time = state.time + dt;
        next_state.V_ref_x = state.V_ref_x;
        next_state.V_ref_y = state.V_ref_y;
        next_state.V_ref_z = state.V_ref_z;
        
        return dt;
    }
    
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    typename SPEC::T reward(DEVICE& device, const Fixedwing<SPEC>& env, const typename Fixedwing<SPEC>::Parameters& parameters, const typename Fixedwing<SPEC>::State& state, const Matrix<ACTION_SPEC>& action, const typename Fixedwing<SPEC>::State& next_state, RNG& rng){
        using T = typename SPEC::T;
        using PARAMS = typename SPEC::PARAMETERS;
        
        // Compute current velocity in NED frame
        T q_norm = rl_tools::math::sqrt(device.math, next_state.qw*next_state.qw + next_state.qx*next_state.qx + 
                                                      next_state.qy*next_state.qy + next_state.qz*next_state.qz);
        T qw = next_state.qw / q_norm;
        T qx = next_state.qx / q_norm;
        T qy = next_state.qy / q_norm;
        T qz = next_state.qz / q_norm;
        
        T R11 = qw*qw + qx*qx - qy*qy - qz*qz;
        T R12 = T(2.0) * (qx*qy - qw*qz);
        T R13 = T(2.0) * (qx*qz + qw*qy);
        T R21 = T(2.0) * (qx*qy + qw*qz);
        T R22 = qw*qw - qx*qx + qy*qy - qz*qz;
        T R23 = T(2.0) * (qy*qz - qw*qx);
        T R31 = T(2.0) * (qx*qz - qw*qy);
        T R32 = T(2.0) * (qy*qz + qw*qx);
        T R33 = qw*qw - qx*qx - qy*qy + qz*qz;
        
        T vx_ned = R11 * next_state.u + R12 * next_state.v + R13 * next_state.w;
        T vy_ned = R21 * next_state.u + R22 * next_state.v + R23 * next_state.w;
        T vz_ned = R31 * next_state.u + R32 * next_state.v + R33 * next_state.w;
        
        // Velocity tracking error
        T vel_error_x = vx_ned - next_state.V_ref_x;
        T vel_error_y = vy_ned - next_state.V_ref_y;
        T vel_error_z = vz_ned - next_state.V_ref_z;
        T vel_error_squared = vel_error_x * vel_error_x + vel_error_y * vel_error_y + vel_error_z * vel_error_z;
        
        // Main tracking reward
        T velocity_tracking_reward = rl_tools::math::exp(device.math, -vel_error_squared / T(50.0));
        
        // Component-wise tracking
        T vx_error_abs = rl_tools::math::abs(device.math, vel_error_x);
        T vy_error_abs = rl_tools::math::abs(device.math, vel_error_y);
        T vz_error_abs = rl_tools::math::abs(device.math, vel_error_z);
        T component_reward = rl_tools::math::exp(device.math, -vx_error_abs / T(5.0)) +
                            rl_tools::math::exp(device.math, -vy_error_abs / T(5.0)) +
                            rl_tools::math::exp(device.math, -vz_error_abs / T(3.0));
        
        // Control effort penalty
        T throttle_action = get(action, 0, 0);
        T left_aileron = get(action, 0, 1);
        T right_aileron = get(action, 0, 2);
        T elevator = get(action, 0, 3);
        T rudder = get(action, 0, 4);
        T control_penalty = left_aileron * left_aileron + right_aileron * right_aileron + 
                           elevator * elevator + rudder * rudder + T(0.1) * throttle_action * throttle_action;
        
        // Airspeed constraint (avoid stall and overspeed)
        T V_mag = rl_tools::math::sqrt(device.math, next_state.u*next_state.u + next_state.v*next_state.v + next_state.w*next_state.w);
        T stall_penalty = T(0.0);
        if (V_mag < PARAMS::min_velocity) {
            stall_penalty = (PARAMS::min_velocity - V_mag) * (PARAMS::min_velocity - V_mag) * T(10.0);
        }
        T overspeed_penalty = T(0.0);
        if (V_mag > PARAMS::max_velocity) {
            overspeed_penalty = (V_mag - PARAMS::max_velocity) * (V_mag - PARAMS::max_velocity) * T(5.0);
        }
        
        // Attitude safety (penalize extreme bank/pitch angles)
        T roll = rl_tools::math::atan2(device.math, T(2.0) * (qw * qx + qy * qz), T(1.0) - T(2.0) * (qx * qx + qy * qy));
        T pitch_sin = clip(T(2.0) * (qw * qy - qz * qx), T(-0.99), T(0.99));
        T pitch = rl_tools::math::atan2(device.math, pitch_sin, rl_tools::math::sqrt(device.math, T(1.0) - pitch_sin * pitch_sin));
        
        T roll_limit = T(0.8);  // ~46 degrees
        T pitch_limit = T(0.6); // ~34 degrees
        T attitude_penalty = T(0.0);
        if (rl_tools::math::abs(device.math, roll) > roll_limit) {
            attitude_penalty += (rl_tools::math::abs(device.math, roll) - roll_limit) * (rl_tools::math::abs(device.math, roll) - roll_limit);
        }
        if (rl_tools::math::abs(device.math, pitch) > pitch_limit) {
            attitude_penalty += (rl_tools::math::abs(device.math, pitch) - pitch_limit) * (rl_tools::math::abs(device.math, pitch) - pitch_limit);
        }
        
        // Combined reward
        T reward = T(20.0) * velocity_tracking_reward + 
                   T(5.0) * component_reward - 
                   T(0.05) * control_penalty - 
                   T(100.0) * stall_penalty - 
                   T(50.0) * overspeed_penalty - 
                   T(10.0) * attitude_penalty;
        
        return reward;
    }

    template<typename DEVICE, typename SPEC, typename OBS_TYPE_SPEC, typename OBS_SPEC, typename RNG>
    void observe(DEVICE& device, const Fixedwing<SPEC>& env, const typename Fixedwing<SPEC>::Parameters& parameters, const typename Fixedwing<SPEC>::State& state, const FixedwingFourierObservation<OBS_TYPE_SPEC>&, Matrix<OBS_SPEC>& observation, RNG& rng){
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == 13);
        using T = typename SPEC::T;
        
        // Normalize quaternion
        T q_norm = rl_tools::math::sqrt(device.math, state.qw*state.qw + state.qx*state.qx + state.qy*state.qy + state.qz*state.qz);
        T qw = state.qw / q_norm;
        T qx = state.qx / q_norm;
        T qy = state.qy / q_norm;
        T qz = state.qz / q_norm;
        
        // Rotation matrix
        T R11 = qw*qw + qx*qx - qy*qy - qz*qz;
        T R12 = T(2.0) * (qx*qy - qw*qz);
        T R13 = T(2.0) * (qx*qz + qw*qy);
        T R21 = T(2.0) * (qx*qy + qw*qz);
        T R22 = qw*qw - qx*qx + qy*qy - qz*qz;
        T R23 = T(2.0) * (qy*qz - qw*qx);
        T R31 = T(2.0) * (qx*qz - qw*qy);
        T R32 = T(2.0) * (qy*qz + qw*qx);
        T R33 = qw*qw - qx*qx - qy*qy + qz*qz;
        
        // Current velocity in NED
        T vx_ned = R11 * state.u + R12 * state.v + R13 * state.w;
        T vy_ned = R21 * state.u + R22 * state.v + R23 * state.w;
        T vz_ned = R31 * state.u + R32 * state.v + R33 * state.w;
        
        // Velocity error
        T vel_error_x = vx_ned - state.V_ref_x;
        T vel_error_y = vy_ned - state.V_ref_y;
        T vel_error_z = vz_ned - state.V_ref_z;
        
        // Observation: [u, v, w, qw, qx, qy, qz, V_error_x, V_error_y, V_error_z, V_ref_x, V_ref_y, V_ref_z]
        set(observation, 0, 0, state.u);
        set(observation, 0, 1, state.v);
        set(observation, 0, 2, state.w);
        set(observation, 0, 3, qw);
        set(observation, 0, 4, qx);
        set(observation, 0, 5, qy);
        set(observation, 0, 6, qz);
        set(observation, 0, 7, vel_error_x);
        set(observation, 0, 8, vel_error_y);
        set(observation, 0, 9, vel_error_z);
        set(observation, 0, 10, state.V_ref_x);
        set(observation, 0, 11, state.V_ref_y);
        set(observation, 0, 12, state.V_ref_z);
    }
    
    template<typename DEVICE, typename SPEC, typename RNG>
    bool terminated(DEVICE& device, const Fixedwing<SPEC>& env, const typename Fixedwing<SPEC>::Parameters& parameters, const typename Fixedwing<SPEC>::State state, RNG& rng){
        using T = typename SPEC::T;
        using PARAMS = typename SPEC::PARAMETERS;
        
        // Terminate if crashed (altitude below ground or too high)
        if (state.z > T(5.0)) return true;  // Crashed into ground (z is positive down)
        if (state.z < T(-200.0)) return true;  // Too high
        
        // Terminate if too far from origin
        T dist = rl_tools::math::sqrt(device.math, state.x * state.x + state.y * state.y);
        if (dist > PARAMS::max_distance) return true;
        
        // Terminate if velocity is too low (deep stall)
        T V_mag = rl_tools::math::sqrt(device.math, state.u*state.u + state.v*state.v + state.w*state.w);
        if (V_mag < T(5.0)) return true;
        
        return false;
    }
}

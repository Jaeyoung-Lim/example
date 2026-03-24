#include <string>

namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE&, Fixedwing<SPEC>& env, typename Fixedwing<SPEC>::Parameters& parameters){
        std::string json = "{";
        json += "\"mass\":" + std::to_string(parameters.mass) + ",";
        json += "\"wing_area\":" + std::to_string(parameters.wing_area) + ",";
        json += "\"wingspan\":" + std::to_string(parameters.wingspan) + ",";
        json += "\"chord\":" + std::to_string(parameters.chord);
        json += "}";
        return json;
    }
    
    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE&, Fixedwing<SPEC>& env, typename Fixedwing<SPEC>::Parameters& parameters, typename Fixedwing<SPEC>::State& state){
        std::string json = "{";
        json += "\"x\":" + std::to_string(state.x) + ",";
        json += "\"y\":" + std::to_string(state.y) + ",";
        json += "\"z\":" + std::to_string(state.z) + ",";
        json += "\"u\":" + std::to_string(state.u) + ",";
        json += "\"v\":" + std::to_string(state.v) + ",";
        json += "\"w\":" + std::to_string(state.w) + ",";
        json += "\"qw\":" + std::to_string(state.qw) + ",";
        json += "\"qx\":" + std::to_string(state.qx) + ",";
        json += "\"qy\":" + std::to_string(state.qy) + ",";
        json += "\"qz\":" + std::to_string(state.qz) + ",";
        json += "\"p\":" + std::to_string(state.p) + ",";
        json += "\"q\":" + std::to_string(state.q_rate) + ",";
        json += "\"r\":" + std::to_string(state.r) + ",";
        json += "\"V_ref_x\":" + std::to_string(state.V_ref_x) + ",";
        json += "\"V_ref_y\":" + std::to_string(state.V_ref_y) + ",";
        json += "\"V_ref_z\":" + std::to_string(state.V_ref_z) + ",";
        json += "\"time\":" + std::to_string(state.time);
        json += "}";
        return json;
    }

    template <typename DEVICE, typename SPEC>
    std::string get_ui(DEVICE& device, Fixedwing<SPEC>& env){
        std::string ui = R"RL_TOOLS_LITERAL(
export async function init(canvas, options){
    return {
        ctx: canvas.getContext('2d')
    }
}

export async function render(ui_state, parameters, state, action) {
    const ctx = ui_state.ctx;
    ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);

    const centerX = ctx.canvas.width / 2;
    const centerY = ctx.canvas.height / 2;
    const scale = 2.0; // pixels per meter

    // Convert quaternion to rotation matrix
    const qw = state.qw, qx = state.qx, qy = state.qy, qz = state.qz;
    const q_norm = Math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    const qw_n = qw/q_norm, qx_n = qx/q_norm, qy_n = qy/q_norm, qz_n = qz/q_norm;
    
    // Rotation matrix (NED to body)
    const R11 = qw_n*qw_n + qx_n*qx_n - qy_n*qy_n - qz_n*qz_n;
    const R12 = 2.0 * (qx_n * qy_n - qw_n * qz_n);
    const R13 = 2.0 * (qx_n * qz_n + qw_n * qy_n);
    const R21 = 2.0 * (qx_n * qy_n + qw_n * qz_n);
    const R22 = qw_n*qw_n - qx_n*qx_n + qy_n*qy_n - qz_n*qz_n;
    const R23 = 2.0 * (qy_n * qz_n - qw_n * qx_n);
    const R31 = 2.0 * (qx_n * qz_n - qw_n * qy_n);
    const R32 = 2.0 * (qy_n * qz_n + qw_n * qx_n);
    const R33 = qw_n*qw_n - qx_n*qx_n - qy_n*qy_n + qz_n*qz_n;
    
    // Current velocity in NED (transform body velocities)
    const vx_ned = R11 * state.u + R12 * state.v + R13 * state.w;
    const vy_ned = R21 * state.u + R22 * state.v + R23 * state.w;
    const vz_ned = R31 * state.u + R32 * state.v + R33 * state.w;
    
    // Aircraft position on canvas (NED: x=North, y=East, but canvas y is down)
    const aircraftX = centerX + state.y * scale;
    const aircraftY = centerY + state.x * scale;
    
    // Draw trajectory path (faded)
    ctx.strokeStyle = 'rgba(100, 100, 100, 0.3)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    
    // Draw grid
    ctx.strokeStyle = 'rgba(200, 200, 200, 0.3)';
    ctx.lineWidth = 0.5;
    for (let i = -10; i <= 10; i++) {
        // Vertical lines
        ctx.beginPath();
        ctx.moveTo(centerX + i * 50, 0);
        ctx.lineTo(centerX + i * 50, ctx.canvas.height);
        ctx.stroke();
        // Horizontal lines
        ctx.beginPath();
        ctx.moveTo(0, centerY + i * 50);
        ctx.lineTo(ctx.canvas.width, centerY + i * 50);
        ctx.stroke();
    }
    
    // Draw reference velocity vector (green)
    const refScale = 5.0;
    const refEndX = centerX + state.V_ref_y * refScale;
    const refEndY = centerY + state.V_ref_x * refScale;
    
    ctx.strokeStyle = 'rgba(0, 200, 0, 0.8)';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(refEndX, refEndY);
    ctx.stroke();
    
    // Arrowhead for reference
    drawArrow(ctx, centerX, centerY, refEndX, refEndY, 'rgba(0, 200, 0, 0.8)');
    
    // Draw current velocity vector (blue)
    const velEndX = aircraftX + vy_ned * refScale;
    const velEndY = aircraftY + vx_ned * refScale;
    
    ctx.strokeStyle = 'rgba(0, 100, 255, 0.8)';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(aircraftX, aircraftY);
    ctx.lineTo(velEndX, velEndY);
    ctx.stroke();
    
    // Arrowhead for current velocity
    drawArrow(ctx, aircraftX, aircraftY, velEndX, velEndY, 'rgba(0, 100, 255, 0.8)');
    
    // Draw aircraft (as a triangle pointing in velocity direction)
    const yaw = Math.atan2(vy_ned, vx_ned);
    const aircraftSize = 15;
    
    ctx.save();
    ctx.translate(aircraftX, aircraftY);
    ctx.rotate(yaw + Math.PI / 2);
    
    ctx.beginPath();
    ctx.moveTo(0, -aircraftSize);
    ctx.lineTo(-aircraftSize * 0.6, aircraftSize * 0.5);
    ctx.lineTo(aircraftSize * 0.6, aircraftSize * 0.5);
    ctx.closePath();
    ctx.fillStyle = 'rgba(255, 100, 0, 0.9)';
    ctx.fill();
    ctx.strokeStyle = 'black';
    ctx.lineWidth = 2;
    ctx.stroke();
    
    ctx.restore();
    
    // Calculate derived quantities
    const V_mag = Math.sqrt(state.u*state.u + state.v*state.v + state.w*state.w);
    const alpha = Math.atan2(state.w, state.u) * 180 / Math.PI;
    const beta = Math.asin(state.v / V_mag) * 180 / Math.PI;
    
    // Error calculation
    const error_x = vx_ned - state.V_ref_x;
    const error_y = vy_ned - state.V_ref_y;
    const error_z = vz_ned - state.V_ref_z;
    const error_mag = Math.sqrt(error_x * error_x + error_y * error_y + error_z * error_z);
    
    // Draw info text
    ctx.fillStyle = 'black';
    ctx.font = '13px monospace';
    ctx.fillText(`V: ${V_mag.toFixed(1)} m/s | α: ${alpha.toFixed(1)}° | β: ${beta.toFixed(1)}°`, 10, 20);
    ctx.fillText(`V_ref: [${state.V_ref_x.toFixed(1)}, ${state.V_ref_y.toFixed(1)}, ${state.V_ref_z.toFixed(1)}]`, 10, 40);
    ctx.fillText(`V_ned: [${vx_ned.toFixed(1)}, ${vy_ned.toFixed(1)}, ${vz_ned.toFixed(1)}]`, 10, 60);
    ctx.fillText(`Pos: [${state.x.toFixed(1)}, ${state.y.toFixed(1)}, ${state.z.toFixed(1)}]`, 10, 80);
    ctx.fillText(`V_error: ${error_mag.toFixed(2)} m/s`, 10, 100);
    
    // Display actuator commands if available
    if (action && action.length >= 5) {
        ctx.fillText(`T: ${action[0].toFixed(2)} | δaL: ${action[1].toFixed(2)} | δaR: ${action[2].toFixed(2)}`, 10, 120);
        ctx.fillText(`δe: ${action[3].toFixed(2)} | δr: ${action[4].toFixed(2)}`, 10, 140);
    }
}

function drawArrow(ctx, fromX, fromY, toX, toY, color) {
    const headlen = 10;
    const angle = Math.atan2(toY - fromY, toX - fromX);
    
    ctx.beginPath();
    ctx.moveTo(toX, toY);
    ctx.lineTo(toX - headlen * Math.cos(angle - Math.PI / 6),
               toY - headlen * Math.sin(angle - Math.PI / 6));
    ctx.moveTo(toX, toY);
    ctx.lineTo(toX - headlen * Math.cos(angle + Math.PI / 6),
               toY - headlen * Math.sin(angle + Math.PI / 6));
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.stroke();
}
        )RL_TOOLS_LITERAL";
        return ui;
    }
}

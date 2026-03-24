#include <string>

namespace rl_tools{
    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE& device, const Fixedwing<SPEC>& env){
        std::string json_string = "{";
        json_string += "\"name\": \"fixedwing\"";
        json_string += "}";
        return json_string;
    }
    
    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE&, Fixedwing<SPEC>& env, typename Fixedwing<SPEC>::Parameters& parameters){
        std::string json = "{";
        json += "\"mass\":" + std::to_string(parameters.mass) + ",";
        json += "\"wing_area\":" + std::to_string(parameters.wing_area) + ",";
        json += "\"wingspan\":" + std::to_string(parameters.wingspan) + ",";
        json += "\"chord\":" + std::to_string(parameters.chord) + ",";
        json += "\"dynamics\": {";
        json += "\"mass\":" + std::to_string(parameters.mass);
        json += "}";
        json += "}";
        return json;
    }
    
    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE&, Fixedwing<SPEC>& env, typename Fixedwing<SPEC>::Parameters& parameters, typename Fixedwing<SPEC>::State& state){
        std::string json = "{";
        json += "\"position\": [" + std::to_string(state.x) + ", " + std::to_string(state.y) + ", " + std::to_string(state.z) + "], ";
        json += "\"orientation\": [" + std::to_string(state.qw) + ", " + std::to_string(state.qx) + ", " + std::to_string(state.qy) + ", " + std::to_string(state.qz) + "], ";
        json += "\"linear_velocity\": [" + std::to_string(state.u) + ", " + std::to_string(state.v) + ", " + std::to_string(state.w) + "], ";
        json += "\"angular_velocity\": [" + std::to_string(state.p) + ", " + std::to_string(state.q_rate) + ", " + std::to_string(state.r) + "], ";
        json += "\"V_ref\": [" + std::to_string(state.V_ref_x) + ", " + std::to_string(state.V_ref_y) + ", " + std::to_string(state.V_ref_z) + "]";
        json += "}";
        return json;
    }

    template <typename DEVICE, typename SPEC>
    std::string get_ui(DEVICE& device, Fixedwing<SPEC>& env){
#ifndef _MSC_VER
        std::string ui = R"RL_TOOLS_LITERAL(
import * as THREE from "three"
import {OrbitControls} from "three-orbitcontrols"
import {GLTFLoader} from "three-gltfloader"

export class CoordinateSystem{
    constructor(origin, length=1, diameter=0.01) {
        this.cs = new THREE.Group()
        const material_red = new THREE.MeshLambertMaterial({color: 0xAA0000})
        const material_green = new THREE.MeshLambertMaterial({color: 0x00AA00})
        const material_blue = new THREE.MeshLambertMaterial({color: 0x0000AA})
        const line = new THREE.BoxGeometry(length, diameter, diameter)
        var x = new THREE.Mesh( line, material_red);
        x.position.set(length/2, 0, 0)
        var y = new THREE.Mesh( line, material_green);
        y.position.set(0, length/2, 0)
        y.rotation.set(0, 0, Math.PI/2)
        var z = new THREE.Mesh( line, material_blue);
        z.position.set(0, 0, length/2)
        z.rotation.set(0, Math.PI/2, 0)
        this.cs.add(x)
        this.cs.add(y)
        this.cs.add(z)
        this.cs.position.set(origin[0], origin[1], origin[2])
    }
    get(){
        return this.cs
    }
}

class FixedWingAircraft {
    constructor(parameters, origin, showAxes) {
        this.group = new THREE.Group()
        this.parameters = parameters
        
        const scale = 10.0
        this.wingspan = (parameters.wingspan || 2.0) * scale
        const chord = (parameters.chord || 0.3) * scale
        const fuselageLength = this.wingspan * 0.7
        const fuselageRadius = chord * 0.3
        
        const material = new THREE.MeshLambertMaterial({color: 0xFF6600})
        const wingMaterial = new THREE.MeshLambertMaterial({color: 0xCCCCCC})
        const tailMaterial = new THREE.MeshLambertMaterial({color: 0xFF0000})
        
        const fuselageGeometry = new THREE.CylinderGeometry(fuselageRadius, fuselageRadius * 0.5, fuselageLength, 16)
        const fuselage = new THREE.Mesh(fuselageGeometry, material)
        fuselage.rotation.z = Math.PI / 2
        this.group.add(fuselage)
        
        const wingGeometry = new THREE.BoxGeometry(chord * 0.5, this.wingspan, chord)
        const wing = new THREE.Mesh(wingGeometry, wingMaterial)
        wing.position.x = -fuselageLength * 0.15
        this.group.add(wing)
        
        const hStabSpan = this.wingspan * 0.35
        const hStabGeometry = new THREE.BoxGeometry(chord * 0.4, hStabSpan, chord * 0.6)
        const hStab = new THREE.Mesh(hStabGeometry, tailMaterial)
        hStab.position.x = -fuselageLength * 0.45
        hStab.position.z = fuselageRadius * 1.5
        this.group.add(hStab)
        
        const vStabGeometry = new THREE.BoxGeometry(chord * 0.4, chord * 0.8, hStabSpan * 0.5)
        const vStab = new THREE.Mesh(vStabGeometry, tailMaterial)
        vStab.position.x = -fuselageLength * 0.45
        vStab.position.z = fuselageRadius * 2.5
        this.group.add(vStab)
        
        const noseGeometry = new THREE.ConeGeometry(fuselageRadius, fuselageRadius * 2, 16)
        const nose = new THREE.Mesh(noseGeometry, material)
        nose.rotation.z = -Math.PI / 2
        nose.position.x = fuselageLength * 0.55
        this.group.add(nose)
        
        if (showAxes) {
            const axesLength = this.wingspan * 0.5
            const axesThickness = 0.02
            this.group.add((new CoordinateSystem([0, 0, 0], axesLength, axesThickness)).get())
        }
        
        this.velocityArrow = new THREE.ArrowHelper(
            new THREE.Vector3(1, 0, 0), 
            new THREE.Vector3(0, 0, 0), 
            this.wingspan * 0.8, 
            0x0088FF,
            this.wingspan * 0.15,
            this.wingspan * 0.1
        )
        this.group.add(this.velocityArrow)
        
        this.refVelocityArrow = new THREE.ArrowHelper(
            new THREE.Vector3(1, 0, 0), 
            new THREE.Vector3(0, 0, 0), 
            this.wingspan * 0.8, 
            0x00FF00,
            this.wingspan * 0.15,
            this.wingspan * 0.1
        )
        this.group.add(this.refVelocityArrow)
        
        if(origin){
            this.group.position.set(...origin)
        }
    }
    
    get(){
        return this.group
    }
}

class State{
    constructor(canvas, {devicePixelRatio, showAxes=false, capture=false, camera_position=[0.5, 0.5, 1], camera_distance=null, interactive=true, conta_url="/conta/"}){
        this.canvas = canvas
        this.IS_MOBILE = this.is_mobile();
        this.actualDevicePixelRatio = devicePixelRatio || window.devicePixelRatio
        this.devicePixelRatio = !this.IS_MOBILE ? this.actualDevicePixelRatio : Math.min(this.actualDevicePixelRatio || 1, 2)
        this.showAxes = showAxes
        this.cursor_grab = interactive
        this.render_tick = 0
        this.capture = capture
        this.camera_position = camera_position
        this.camera_distance = camera_distance
        this.interactive = interactive
        this.lastCanvasWidth = 0
        this.lastCanvasHeight = 0
        this.conta_url = conta_url
    }

    is_mobile() {
        const isIOS = /iP(hone|ad|od)/.test(navigator.platform) || /iPhone|iPad|iPod/.test(navigator.userAgent);
        const isAndroid = /Android/.test(navigator.userAgent);
        const isMobile = /Mobi|Android/i.test(navigator.userAgent) || 'ontouchstart' in window || navigator.maxTouchPoints > 0;
        const isTablet = /iPad|Android(?!.*Mobile)/.test(navigator.userAgent);
        return isIOS || isAndroid || isMobile || isTablet;
    }

    async initialize(){
        const width = this.canvas.width
        const height = this.canvas.height
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera( 40, width / height, 0.1, 1000 );
        this.scene.add(this.camera);

        this.renderer = new THREE.WebGLRenderer({
            canvas: this.canvas,
            antialias: !this.IS_MOBILE,
            alpha: !this.IS_MOBILE,
            powerPreference: this.IS_MOBILE ? 'low-power' : 'high-performance',
            preserveDrawingBuffer: this.capture && !this.IS_MOBILE
          });

        this.renderer.setPixelRatio(this.devicePixelRatio)
        this.renderer.setClearColor(0xffffff, 0);

        this.renderer.setSize(width/this.actualDevicePixelRatio, height/this.actualDevicePixelRatio);

        this.lastCanvasWidth = this.canvas.width
        this.lastCanvasHeight = this.canvas.height

        this.controls = this.interactive ? new OrbitControls(this.camera, this.renderer.domElement) : null;

        this.simulator = new THREE.Group()
        this.simulator.rotation.set(-Math.PI / 2, 0, Math.PI / 2, 'XYZ');

        this.scene.add(this.simulator)

        var light = new THREE.AmbientLight( 0xffffff,0.5 );
        this.scene.add(light);
        var directionalLight = new THREE.DirectionalLight( 0xffffff, 0.4 )
        directionalLight.position.set(-100, 100, 0)
        directionalLight.target.position.set(0, 0, 0)
        this.scene.add( directionalLight )
        var directionalLight = new THREE.DirectionalLight( 0xffffff, 0.3 )
        directionalLight.position.set(0, 100, 100)
        directionalLight.target.position.set(0, 0, 0)
        this.scene.add( directionalLight )
        var directionalLight = new THREE.DirectionalLight( 0xffffff, 0.2 )
        directionalLight.position.set(0, 100, -100)
        directionalLight.target.position.set(0, 0, 0)
        this.scene.add( directionalLight )

        this.camera_set = false
        this.THREE = THREE
    }
}

export async function init(canvas, options){
    const state = new State(canvas, options)
    await state.initialize()
    return state
}

function clear_episode(ui_state){
    if(ui_state.aircraft){
        ui_state.simulator.remove(ui_state.aircraft.get())
        if(ui_state.showAxes){
            ui_state.simulator.remove(ui_state.origin_coordinate_system.get())
        }
    }
    if(ui_state.trajectoryLine){
        ui_state.simulator.remove(ui_state.trajectoryLine)
    }
}

function set_camera(ui_state, distance){
    const scale = 1/Math.sqrt(ui_state.camera_position[0]**2 + ui_state.camera_position[1]**2 + ui_state.camera_position[2]**2) * distance
    if(!ui_state.camera_set){
        ui_state.camera.position.set(ui_state.camera_position[0] * scale, ui_state.camera_position[1] * scale, ui_state.camera_position[2] * scale)
        ui_state.camera.lookAt(0, 0, 0)
        ui_state.camera_set = true
        if(ui_state.controls){
            ui_state.controls.update()
        }
    }
}

export async function episode_init(ui_state, parameters){
    let distance = Math.cbrt(parameters.dynamics.mass) * 20
    if(ui_state.camera_distance){
        distance = ui_state.camera_distance
    }
    set_camera(ui_state, distance)
    clear_episode(ui_state)
    ui_state.aircraft = new FixedWingAircraft(parameters, [0, 0, 0], ui_state.showAxes)
    ui_state.simulator.add(ui_state.aircraft.get())
    const scale = parameters.dynamics.mass
    if(ui_state.showAxes){
        ui_state.origin_coordinate_system = new CoordinateSystem([0, 0, 0], 1 * scale, 0.01 * scale)
        ui_state.simulator.add(ui_state.origin_coordinate_system.get())
    }
    ui_state.trajectoryPoints = []
}

function clip_position(scale, position){
    const extent = Math.cbrt(scale) * 300
    const max_position = extent
    const min_position = -extent
    return position.map((p) => {
        if(p > max_position){
            return max_position
        }
        else if(p < min_position){
            return min_position
        }
        else{
            return p
        }
    })
}

function update_camera(ui_state){
    const currentWidth = ui_state.canvas.width
    const currentHeight = ui_state.canvas.height
    const hasResized = currentWidth !== ui_state.lastCanvasWidth || currentHeight !== ui_state.lastCanvasHeight

    if (hasResized) {
        const width = currentWidth / ui_state.devicePixelRatio;
        const height = currentHeight / ui_state.devicePixelRatio;

        if (ui_state.camera.aspect !== width / height) {
            ui_state.camera.aspect = width / height;
            ui_state.camera.updateProjectionMatrix();
        }

        if (ui_state.renderer) {
            ui_state.renderer.setPixelRatio(ui_state.devicePixelRatio);
            ui_state.renderer.setSize(width, height, false);
        }

        ui_state.lastCanvasWidth = currentWidth
        ui_state.lastCanvasHeight = currentHeight
    }

    if(ui_state.interactive && ui_state.controls){
        ui_state.controls.update()
    }
    if(ui_state.renderer){
        ui_state.renderer.render(ui_state.scene, ui_state.camera);
    }
    ui_state.render_tick += 1
}

export async function render(ui_state, parameters, state, action) {
    if(ui_state.aircraft){
        ui_state.aircraft.get().position.set(...clip_position(parameters.dynamics.mass, state.position))
        ui_state.aircraft.get().quaternion.copy(new THREE.Quaternion(state.orientation[1], state.orientation[2], state.orientation[3], state.orientation[0]).normalize())
        
        const qw = state.orientation[0], qx = state.orientation[1], qy = state.orientation[2], qz = state.orientation[3]
        const q_norm = Math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        const qw_n = qw/q_norm, qx_n = qx/q_norm, qy_n = qy/q_norm, qz_n = qz/q_norm
        
        const R11 = qw_n*qw_n + qx_n*qx_n - qy_n*qy_n - qz_n*qz_n
        const R12 = 2.0 * (qx_n * qy_n - qw_n * qz_n)
        const R13 = 2.0 * (qx_n * qz_n + qw_n * qy_n)
        const R21 = 2.0 * (qx_n * qy_n + qw_n * qz_n)
        const R22 = qw_n*qw_n - qx_n*qx_n + qy_n*qy_n - qz_n*qz_n
        const R23 = 2.0 * (qy_n * qz_n - qw_n * qx_n)
        const R31 = 2.0 * (qx_n * qz_n - qw_n * qy_n)
        const R32 = 2.0 * (qy_n * qz_n + qw_n * qx_n)
        const R33 = qw_n*qw_n - qx_n*qx_n - qy_n*qy_n + qz_n*qz_n
        
        const vBody = new THREE.Vector3(state.linear_velocity[0], state.linear_velocity[1], state.linear_velocity[2])
        const vMag = vBody.length()
        
        if (vMag > 0.1) {
            const vBodyNorm = vBody.clone().normalize()
            ui_state.aircraft.velocityArrow.setDirection(vBodyNorm)
            ui_state.aircraft.velocityArrow.setLength(ui_state.aircraft.wingspan * 0.6)
            ui_state.aircraft.velocityArrow.visible = true
        } else {
            ui_state.aircraft.velocityArrow.visible = false
        }
        
        if (state.V_ref) {
            const vRefBody = new THREE.Vector3(
                R11 * state.V_ref[0] + R12 * state.V_ref[1] + R13 * state.V_ref[2],
                R21 * state.V_ref[0] + R22 * state.V_ref[1] + R23 * state.V_ref[2],
                R31 * state.V_ref[0] + R32 * state.V_ref[1] + R33 * state.V_ref[2]
            )
            const vRefMag = vRefBody.length()
            
            if (vRefMag > 0.1) {
                const vRefBodyNorm = vRefBody.clone().normalize()
                ui_state.aircraft.refVelocityArrow.setDirection(vRefBodyNorm)
                ui_state.aircraft.refVelocityArrow.setLength(ui_state.aircraft.wingspan * 0.6)
                ui_state.aircraft.refVelocityArrow.visible = true
            } else {
                ui_state.aircraft.refVelocityArrow.visible = false
            }
        }
        
        if (!ui_state.trajectoryPoints) {
            ui_state.trajectoryPoints = []
        }
        
        ui_state.trajectoryPoints.push(new THREE.Vector3(state.position[0], state.position[1], state.position[2]))
        
        const maxTrajectoryPoints = 500
        if (ui_state.trajectoryPoints.length > maxTrajectoryPoints) {
            ui_state.trajectoryPoints.shift()
        }
        
        if (ui_state.trajectoryLine) {
            ui_state.simulator.remove(ui_state.trajectoryLine)
        }
        
        if (ui_state.trajectoryPoints.length > 1) {
            const trajectoryGeometry = new THREE.BufferGeometry().setFromPoints(ui_state.trajectoryPoints)
            const trajectoryMaterial = new THREE.LineBasicMaterial({
                color: 0xFF6600,
                linewidth: 2,
                transparent: true,
                opacity: 0.6
            })
            ui_state.trajectoryLine = new THREE.Line(trajectoryGeometry, trajectoryMaterial)
            ui_state.simulator.add(ui_state.trajectoryLine)
        }
        
        if(ui_state.controls && state.position){
            const aircraftPos = new THREE.Vector3(state.position[0], state.position[1], state.position[2])
            ui_state.controls.target.lerp(aircraftPos, 0.05)
        }
    }
    update_camera(ui_state)
}
        )RL_TOOLS_LITERAL";
#else
        std::string ui = "";
#endif
        return ui;
    }
}

use anyhow::Result;
use std::{
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};
use rclrs::{Context, CreateBasicExecutor, SpinOptions};
use px4_msgs::msg::{
    VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleLocalPosition,
    VehicleCommandAck,
};
fn now() -> u64 {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or(Duration::ZERO);
    now.as_secs() * 1_000_000 + u64::from(now.subsec_micros())
}
fn publish_command(
    pub_: &rclrs::Publisher<VehicleCommand>,
    command: u32,
    p1: f32,
    p2: f32,
    p3: f32,
) -> Result<()> {
    let msg = VehicleCommand {
        timestamp: now(),
        param1: p1,
        param2: p2,
        param3: p3,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command,
        target_system: 1,
        target_component: 1,
        source_system: 1,
        source_component: 1,
        confirmation: 0,
        from_external: true,
    };
    pub_.publish(&msg)?;
    println!("Published command {}", command);
    Ok(())
}
#[derive(Clone)]
struct Telemetry {
    x: f32,
    y: f32,
    z: f32,
    pose_valid: bool,
    last_pose_time: Instant,
    arming_state: u8,
    nav_state: u8,
    failsafe: bool,
}
impl Default for Telemetry {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            pose_valid: false,
            last_pose_time: Instant::now(),
            arming_state: 0,
            nav_state: 0,
            failsafe: false,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    Prime,
    WaitPose,
    RequestOffboard,
    RequestArm,
    Takeoff,
    Hold,
    Waypoints,
    RTL,
    Land,
    Disarm,
    Done,
    Failsafe,
}
fn main() -> Result<()> {
    println!("Starting Autonomous FSM");
    let context = Context::default();
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("px4_autonomous")?;
    let pub_cmd = node.create_publisher::<VehicleCommand>("/fmu/in/vehicle_command")?;
    let pub_offboard = node.create_publisher::<OffboardControlMode>("/fmu/in/offboard")?;
    let pub_traj = node.create_publisher::<TrajectorySetpoint>("/fmu/in/trajectory")?;
    let telemetry = Arc::new(Mutex::new(Telemetry::default()));
    {
        let telemetry = telemetry.clone();
        let _sub = node.create_subscription::<VehicleLocalPosition, _>(
            "/fmu/out/vehicle_position",
            move |msg: VehicleLocalPosition| {
                let mut t = telemetry.lock().unwrap();
                t.x = msg.x;
                t.y = msg.y;
                t.z = msg.z;
                t.pose_valid = msg.xy_valid && msg.z_valid;
                t.last_pose_time = Instant::now();
            },
        )?;
        std::mem::forget(_sub);
    }
    {
        let telemetry = telemetry.clone();
        let _sub = node.create_subscription::<VehicleStatus, _>(
            "/fmu/out/vehicle_status",
            move |msg: VehicleStatus| {
                let mut t = telemetry.lock().unwrap();
                t.arming_state = msg.arming_state;
                t.nav_state = msg.nav_state;
                t.failsafe = msg.failsafe;
            },
        )?;
        std::mem::forget(_sub);
    }
    {
        let _sub = node.create_subscription::<VehicleCommandAck, _>(
            "/fmu/out/vehicle_command_ack",
            move |msg: VehicleCommandAck| {
                println!(
                    "ACK: cmd={}, result={}, param1={:.1}, param2={:.1}",
                    msg.command, msg.result, msg.result_param1, msg.result_param2
                );
            },
        )?;
        std::mem::forget(_sub);
    }
    let mut spin_once = || {
        let errs = executor.spin(SpinOptions::default().timeout(Duration::from_millis(10)));
        if !errs.is_empty() {
            let non_timeout = errs.iter().any(|e| !format!("{:?}", e).contains("Timeout"));
            if non_timeout {
                eprintln!("Spin error: {:?}", errs);
            }
        }
    };
    const TAKEOFF_ALT: f32 = -5.0;
    const HOLD_TIME: u64 = 3;
    const STEP_TIMEOUT: u64 = 10;
    const POSE_TIMEOUT: u64 = 15;
    let waypoints = [
        [0.0, 0.0, TAKEOFF_ALT],
        [5.0, 0.0, TAKEOFF_ALT],
        [5.0, 5.0, TAKEOFF_ALT],
        [0.0, 5.0, TAKEOFF_ALT],
        [0.0, 0.0, TAKEOFF_ALT],
    ];
    let mut phase = Phase::Prime;
    let mut current_wp = 0usize;
    let mut phase_start = Instant::now();
    let mut hold_start = Instant::now();
    let mut offboard_msg = OffboardControlMode {
        timestamp: now(),
        position: true,
        velocity: false,
        acceleration: false,
        attitude: false,
        body_rate: false,
        thrust: false,
        ..Default::default()
    };
    println!("Priming Offboard");
    while phase_start.elapsed() < Duration::from_secs(1) {
        offboard_msg.timestamp = now();
        pub_offboard.publish(&offboard_msg)?;
        pub_traj.publish(&TrajectorySetpoint {
            timestamp: now(),
            position: waypoints[0],
            velocity: [0.0; 3],
            acceleration: [0.0; 3],
            jerk: [0.0; 3],
            yaw: 0.0,
            yawspeed: 0.0,
        })?;
        spin_once();
        thread::sleep(Duration::from_millis(100));
    }
    phase_start = Instant::now();
    loop {
        {
            let t = telemetry.lock().unwrap();
            println!(
                "Telemetry pos: ({:.2},{:.2},{:.2}) arm:{} nav:{} failsafe:{} phase:{:?}",
                t.x, t.y, t.z, t.arming_state, t.nav_state, t.failsafe, phase
            );
        }
        offboard_msg.timestamp = now();
        pub_offboard.publish(&offboard_msg)?;
        let pose_cmd = match phase {
            Phase::Prime | Phase::WaitPose | Phase::RequestOffboard | Phase::RequestArm | Phase::Hold => waypoints[0],
            Phase::Takeoff => [0.0, 0.0, TAKEOFF_ALT],
            Phase::Waypoints | Phase::RTL => waypoints[current_wp],
            Phase::Land | Phase::Disarm | Phase::Done | Phase::Failsafe => waypoints[0],
        };
        pub_traj.publish(&TrajectorySetpoint {
            timestamp: now(),
            position: pose_cmd,
            velocity: [0.0; 3],
            acceleration: [0.0; 3],
            jerk: [0.0; 3],
            yaw: 0.0,
            yawspeed: 0.0,
        })?;
        spin_once();
        thread::sleep(Duration::from_millis(100));
        let telem_data = telemetry.lock().unwrap();
        let pose_valid = telem_data.pose_valid;
        let armed = telem_data.arming_state > 1;
        let failsafe = telem_data.failsafe;
        drop(telem_data);
        if !pose_valid && !matches!(phase, Phase::Prime | Phase::WaitPose | Phase::Done | Phase::Failsafe) {
            if phase_start.elapsed() > Duration::from_secs(POSE_TIMEOUT) {
                println!("Pose timeout, triggering failsafe");
                phase = Phase::Failsafe;
                phase_start = Instant::now();
            }
        }
        if failsafe && !matches!(phase, Phase::Failsafe | Phase::Done) {
            println!("Failsafe triggered by vehicle");
            phase = Phase::Failsafe;
            phase_start = Instant::now();
        }
        match phase {
            Phase::Prime => {
                if phase_start.elapsed() > Duration::from_secs(1) {
                    publish_command(&pub_cmd, 176, 1.0, 6.0, 0.0)?;
                    println!("Requested OFFBOARD mode");
                    phase = Phase::WaitPose;
                    phase_start = Instant::now();
                }
            }
            Phase::WaitPose => {
                if pose_valid {
                    println!("Pose valid received, requesting OFFBOARD");
                    phase = Phase::RequestOffboard;
                    phase_start = Instant::now();
                } else if phase_start.elapsed() > Duration::from_secs(POSE_TIMEOUT) {
                    println!("Pose timeout in WaitPose, triggering failsafe");
                    phase = Phase::Failsafe;
                    phase_start = Instant::now();
                }
            }
            Phase::RequestOffboard => {
                publish_command(&pub_cmd, 176, 1.0, 6.0, 0.0)?;
                if phase_start.elapsed() > Duration::from_secs(STEP_TIMEOUT) {
                    println!("OFFBOARD request timeout");
                    phase = Phase::Failsafe;
                } else {
                    phase = Phase::RequestArm;
                    phase_start = Instant::now();
                }
            }
            Phase::RequestArm => {
                publish_command(&pub_cmd, 400, 1.0, 0.0, 0.0)?;
                if phase_start.elapsed() > Duration::from_secs(STEP_TIMEOUT) {
                    println!("Arm request timeout");
                    phase = Phase::Failsafe;
                } else if armed {
                    println!("Vehicle armed, starting takeoff");
                    phase = Phase::Takeoff;
                    phase_start = Instant::now();
                }
            }
            Phase::Takeoff => {
                if phase_start.elapsed() > Duration::from_secs(STEP_TIMEOUT) && pose_valid {
                    println!("Takeoff reached, holding");
                    phase = Phase::Hold;
                    phase_start = Instant::now();
                    hold_start = Instant::now();
                }
            }
            Phase::Hold => {
                if hold_start.elapsed() >= Duration::from_secs(HOLD_TIME) {
                    println!("Hold complete, starting waypoints");
                    phase = Phase::Waypoints;
                    current_wp = 0;
                    phase_start = Instant::now();
                }
            }
            Phase::Waypoints => {
                if current_wp >= waypoints.len() {
                    println!("All waypoints completed, initiating RTL");
                    phase = Phase::RTL;
                    phase_start = Instant::now();
                } else if phase_start.elapsed() >= Duration::from_secs(8) {
                    println!("Moving to waypoint {}", current_wp);
                    current_wp += 1;
                    phase_start = Instant::now();
                }
            }
            Phase::RTL => {
                println!("Returning to launch");
                phase = Phase::Land;
                phase_start = Instant::now();
            }
            Phase::Land => {
                publish_command(&pub_cmd, 21, 0.0, 0.0, 0.0)?;
                if phase_start.elapsed() >= Duration::from_secs(10) {
                    println!("Landing complete, disarming");
                    phase = Phase::Disarm;
                    phase_start = Instant::now();
                }
            }
            Phase::Disarm => {
                publish_command(&pub_cmd, 400, 0.0, 0.0, 0.0)?;
                if phase_start.elapsed() >= Duration::from_secs(5) {
                    println!("Disarm complete, mission done");
                    phase = Phase::Done;
                }
            }
            Phase::Failsafe => {
                println!("Failsafe triggered, landing now");
                publish_command(&pub_cmd, 21, 0.0, 0.0, 0.0)?;
                if phase_start.elapsed() >= Duration::from_secs(15) {
                    phase = Phase::Disarm;
                    phase_start = Instant::now();
                }
            }
            Phase::Done => {
                println!("Mission completed. Exiting.");
                break;
            }
        }
        thread::sleep(Duration::from_millis(100));
    }
    Ok(())
}

use glam::{DVec3, FloatExt, Quat, Vec3};
use serde::{Deserialize, Serialize};
use std::{ops::Mul, time::Duration};

// Field of view in radians
#[derive(Serialize, Deserialize, PartialEq, Debug, Clone, Copy)]
pub struct Fov {
    pub left: f32,
    pub right: f32,
    pub up: f32,
    pub down: f32,
}

impl Default for Fov {
    fn default() -> Self {
        Fov {
            left: -1.0,
            right: 1.0,
            up: 1.0,
            down: -1.0,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Default, Debug)]
pub struct Pose {
    pub orientation: Quat, // NB: default Quat is identity
    pub position: Vec3,
}

impl Pose {
    pub fn inverse(&self) -> Pose {
        let inverse_orientation = self.orientation.conjugate();
        Pose {
            orientation: inverse_orientation,
            position: inverse_orientation * -self.position,
        }
    }

    pub fn convergence_point(&self, other_ray: &Pose) -> Option<(Vec3, f32, f32, f32)> {
        let l_orient = self.orientation.as_dquat();
        let l_forward = l_orient * DVec3::NEG_Z;

        let r_orient = other_ray.orientation.as_dquat();
        let r_forward = r_orient * DVec3::NEG_Z;

        let p1 = self.position.as_dvec3(); // position
        let d1 = l_forward; // forward
        let p2 = other_ray.position.as_dvec3();
        let d2 = r_forward;
        let p_avg = (p1 + p2) / 2.0;
        let forward_avg = ((l_forward + r_forward) * 0.5).normalize();

        //alvr_common::error!("{p1} {p2}, {l_forward}");

        let p1_p2 = p2 - p1;
        let d1_dot_d1 = d1.dot(d1);
        let d2_dot_d2 = d2.dot(d2);
        let d1_dot_d2 = d1.dot(d2);
        let p1_p2_dot_d1 = p1_p2.dot(d1);
        let p1_p2_dot_d2 = p1_p2.dot(d2);

        let denom = d1_dot_d1 * d2_dot_d2 - d1_dot_d2 * d1_dot_d2;
        if denom.abs() < 1e-6 {
            return None;
            //return None; // Rays are nearly parallel
            //alvr_common::info!("Eyes are parallel l: {p1} {d1} r: {p2} {d2}");
        } else {
            let s = (p1_p2_dot_d1 * d2_dot_d2 - p1_p2_dot_d2 * d1_dot_d2) / denom;
            let t = (p1_p2_dot_d1 * d1_dot_d2 - p1_p2_dot_d2 * d1_dot_d1) / denom;

            let closest_p1 = p1 + s.abs() * d1;
            let closest_p2 = p2 + t.abs() * d2;

            let closest_p1_dist_from_eyes = p_avg.distance(closest_p1);
            let closest_p2_dist_from_eyes = p_avg.distance(closest_p2);

            let depth = ((closest_p1_dist_from_eyes + closest_p2_dist_from_eyes) / 2.0).abs(); // Approximate depth
            
            let correction = 1.0;

            let convergence_pt = (p_avg + (forward_avg * depth)).as_vec3();
            return Some((convergence_pt, depth as f32, (s.abs() * correction) as f32, (t.abs() * correction) as f32));
            //return Some(p_avg + (forward_avg * depth));
        }
    }

    pub fn convergence_point_weird(&self, other_ray: &Pose) -> Option<(Vec3, f32, f32, f32)> {
        let l_orient = self.orientation.as_dquat();
        let l_forward = l_orient * DVec3::NEG_Z;

        let r_orient = other_ray.orientation.as_dquat();
        let r_forward = r_orient * DVec3::NEG_Z;

        let p1 = self.position.as_dvec3(); // position
        let d1 = l_forward; // forward
        let p2 = other_ray.position.as_dvec3();
        let d2 = r_forward;
        let p_avg = (p1 + p2) / 2.0;
        let forward_avg = ((l_forward + r_forward) * 0.5).normalize();

        //alvr_common::error!("{p1} {p2}, {l_forward}");

        let p1_p2 = p2 - p1;
        let d1_dot_d1 = d1.dot(d1);
        let d2_dot_d2 = d2.dot(d2);
        let d1_dot_d2 = d1.dot(d2);
        let p1_p2_dot_d1 = p1_p2.dot(d1);
        let p1_p2_dot_d2 = p1_p2.dot(d2);

        let denom = d1_dot_d1 * d2_dot_d2 - d1_dot_d2 * d1_dot_d2;
        if denom.abs() < 1e-6 {
            return None;
            //return None; // Rays are nearly parallel
            //alvr_common::info!("Eyes are parallel l: {p1} {d1} r: {p2} {d2}");
        } else {
            let s = (p1_p2_dot_d1 * d2_dot_d2 - p1_p2_dot_d2 * d1_dot_d2) / denom;
            let t = (p1_p2_dot_d1 * d1_dot_d2 - p1_p2_dot_d2 * d1_dot_d1) / denom;

            let closest_p1 = p1 + s.abs() * d1;
            let closest_p2 = p2 + t.abs() * d2;

            let closest_p1_dist_from_eyes = p_avg.distance(closest_p1);
            let closest_p2_dist_from_eyes = p_avg.distance(closest_p2);

            let depth = ((closest_p1_dist_from_eyes + closest_p2_dist_from_eyes) / 2.0).abs(); // Approximate depth
            
            //let correction = 1.0;
            //let correction_2 = 1.6; // 1.4 with depth-tossed passthrough, 1.6 with depth accurate passthrough
            //let real_depth = ((depth * correction).tan() * correction_2).min(50.0);

            // The correct math
            /*let vergence_angle = d1.angle_between(d2);
            let ipd = (p1 - p2).length();
            let mut real_depth = (ipd * 0.5) / (vergence_angle * 0.5).tan();*/

            // What actually works
            let vergence_angle = d1.angle_between(d2);
            let ipd = (p1 - p2).length();
            let mut real_depth = (ipd / (vergence_angle * 0.5).tan()).powi(2);

            let convergence_pt = (p_avg + (forward_avg * real_depth)).as_vec3();
            return Some((convergence_pt, real_depth as f32, (s.abs()) as f32, (t.abs()) as f32));
            //return Some(p_avg + (forward_avg * depth));
        }
    }
}

impl Mul<Pose> for Pose {
    type Output = Pose;

    fn mul(self, rhs: Pose) -> Pose {
        Pose {
            orientation: self.orientation * rhs.orientation,
            position: self.position + self.orientation * rhs.position,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Default, Debug)]
pub struct DeviceMotion {
    pub pose: Pose,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

impl Mul<DeviceMotion> for Pose {
    type Output = DeviceMotion;

    fn mul(self, rhs: DeviceMotion) -> DeviceMotion {
        DeviceMotion {
            pose: self * rhs.pose,
            linear_velocity: self.orientation * rhs.linear_velocity,
            angular_velocity: self.orientation * rhs.angular_velocity,
        }
    }
}

// Calculate difference ensuring maximum precision is preserved
fn difference_seconds(from: Duration, to: Duration) -> f32 {
    to.saturating_sub(from).as_secs_f32() - from.saturating_sub(to).as_secs_f32()
}

impl DeviceMotion {
    pub fn predict(&self, from_timestamp: Duration, to_timestamp: Duration) -> Self {
        let delta_time_s = difference_seconds(from_timestamp, to_timestamp);

        let delta_position = self.linear_velocity * delta_time_s;
        let delta_orientation = Quat::from_scaled_axis(self.angular_velocity * delta_time_s);

        DeviceMotion {
            pose: Pose {
                orientation: delta_orientation * self.pose.orientation,
                position: self.pose.position + delta_position,
            },
            linear_velocity: self.linear_velocity,
            angular_velocity: self.angular_velocity,
        }
    }
}

pub struct OneEuroBase {
    /** Minimum frequency cutoff for filter, default = 25.0 */
	fc_min: f32,

	/** Minimum frequency cutoff for derivative filter, default = 10.0 */
	fc_min_d: f32,

	/** Beta value for "responsiveness" of filter - default = 0.01 */
	beta: f32,

	/** true if we have already processed a history sample */
	have_prev_y: bool,

	/** Timestamp of previous sample (nanoseconds) */
	prev_ts: u64,
}

impl OneEuroBase {
    pub fn default() -> OneEuroBase {
        OneEuroBase {
            fc_min: 25.0,
            fc_min_d: 10.0,
            beta: 0.01,
            have_prev_y: false,
            prev_ts: 0,
        }
    }

    pub fn new(fc_min: f32, fc_min_d: f32, beta: f32) -> OneEuroBase {
        OneEuroBase {
            fc_min,
            fc_min_d,
            beta,
            have_prev_y: false,
            prev_ts: 0,
        }
    }

    pub fn handle_first_sample(&mut self, ts: u64, commit: bool) -> bool {
        if !self.have_prev_y {
            /* First sample - no filtering yet */
            if commit {
                self.prev_ts = ts;
                self.have_prev_y = true;
            }
            return true;
        }
        return false;
    }

    fn calc_smoothing_alpha(&self, fc: f64, dt: f64) -> f64
    {
        /* Calculate alpha = (1 / (1 + tau/dt)) where tau = 1.0 / (2 * pi * Fc),
         * this is a straight rearrangement with fewer divisions */
        let r = 2.0 * std::f64::consts::PI * fc * dt;
        return r / (r + 1.0);
    }

    pub fn compute_alpha_d(&mut self, ts: u64, commit: bool) -> (f64, f64) {
        let dt = ((ts.saturating_sub(self.prev_ts)) as f64) / (1000.0 * 1000.0 * 1000.0);
        if commit {
            self.prev_ts = ts;
        }
        let alpha_d = self.calc_smoothing_alpha(self.fc_min_d as f64, dt);
        return (alpha_d, dt);
    }

    pub fn compute_alpha(&self, dt: f64, smoothed_derivative_magnitude: f64) -> f64 {
	    let fc_cutoff = self.fc_min as f64 + self.beta as f64 * smoothed_derivative_magnitude;
	    return self.calc_smoothing_alpha(fc_cutoff as f64, dt);
    }
}

pub struct OneEuroF32 {
    base: OneEuroBase,
    prev_y: f32,
    prev_dy: f32,
}

impl OneEuroF32 {
    pub fn default() -> OneEuroF32 {
        OneEuroF32 {
            base: OneEuroBase::default(),
            prev_y: 0.0,
            prev_dy: 0.0,
        }
    }

    pub fn new(fc_min: f32, fc_min_d: f32, beta: f32) -> OneEuroF32 {
        OneEuroF32 {
            base: OneEuroBase::new(fc_min, fc_min_d, beta),
            prev_y: 0.0,
            prev_dy: 0.0,
        }
    }

    pub fn run(&mut self, in_ts: u64, in_y: f32) -> f32 {
        if self.base.handle_first_sample(in_ts, true) {
            /* First sample - no filtering yet */
            self.prev_dy = 0.0;
            self.prev_y = in_y;
    
            return in_y;
        }

        if in_ts == self.base.prev_ts {
            return self.prev_y;
        }
    
        let (alpha_d, dt) = self.base.compute_alpha_d(in_ts, true);
    
        let mut dy = (in_y - self.prev_y) as f64 / dt;
        self.prev_dy = self.prev_dy.lerp(dy as f32, alpha_d as f32);
        
        let dy_mag = dy.abs();
        let alpha = self.base.compute_alpha(dt, dy_mag as f64);
    
        /* Smooth the dy values and use them to calculate the frequency cutoff for the main filter */
        self.prev_y = self.prev_y.lerp( in_y, alpha as f32);

        return self.prev_y;
    }
}

pub struct OneEuroQuat {
    base: OneEuroBase,
    prev_y: Quat,
    prev_dy: Quat,
}

impl OneEuroQuat {
    pub fn default() -> OneEuroQuat {
        OneEuroQuat {
            base: OneEuroBase::default(),
            prev_y: Quat::IDENTITY,
            prev_dy: Quat::IDENTITY,
        }
    }

    pub fn new(fc_min: f32, fc_min_d: f32, beta: f32) -> OneEuroQuat {
        OneEuroQuat {
            base: OneEuroBase::new(fc_min, fc_min_d, beta),
            prev_y: Quat::IDENTITY,
            prev_dy: Quat::IDENTITY,
        }
    }

    pub fn run(&mut self, in_ts: u64, in_y: Quat) -> Quat {
        if self.base.handle_first_sample(in_ts, true) {
            /* First sample - no filtering yet */
            self.prev_dy = Quat::IDENTITY;
            self.prev_y = in_y;
    
            return in_y;
        }

        if in_ts == self.base.prev_ts {
            return self.prev_y;
        }
    
        let (alpha_d, dt) = self.base.compute_alpha_d(in_ts, true);
    
        let mut dy = in_y * self.prev_y.inverse();
    
        // Scale dy with dt through a conversion to angle_axis
        let mut dy_aa = dy.to_scaled_axis();
        dy_aa /= dt as f32;
        dy = Quat::from_scaled_axis(dy_aa);
        self.prev_dy = self.prev_dy.lerp(dy, alpha_d as f32);
    
        // The magnitude of the smoothed dy (f->prev_dy) is its rotation angle in radians
        let smooth_dy_aa = self.prev_dy.to_scaled_axis();
        let smooth_dy_mag = smooth_dy_aa.length();
    
        let alpha = self.base.compute_alpha(dt, smooth_dy_mag as f64);
    
        /* Smooth the dy values and use them to calculate the frequency cutoff for the main filter */
        self.prev_y = self.prev_y.lerp( in_y, alpha as f32);

        //crate::error!("test {dt}");

        return self.prev_y;
    }
}

pub struct OneEuroVec3 {
    base: OneEuroBase,
    prev_y: Vec3,
    prev_dy: Vec3,
}

impl OneEuroVec3 {
    pub fn default() -> OneEuroVec3 {
        OneEuroVec3 {
            base: OneEuroBase::default(),
            prev_y: Vec3::ZERO,
            prev_dy: Vec3::ZERO,
        }
    }

    pub fn run(&mut self, in_ts: u64, in_y: Vec3) -> Vec3 {
        if self.base.handle_first_sample(in_ts, true) {
            /* First sample - no filtering yet */
            self.prev_dy = Vec3::ZERO;
            self.prev_y = in_y;
    
            return in_y;
        }

        if in_ts == self.base.prev_ts {
            return self.prev_y;
        }
    
        let (alpha_d, dt) = self.base.compute_alpha_d(in_ts, true);
    
        let dy = (in_y - self.prev_y) * dt as f32;
	    self.prev_dy = self.prev_dy.lerp(dy, alpha_d as f32);

	    let dy_mag = self.prev_dy.length();
        let alpha = self.base.compute_alpha(dt, dy_mag as f64);

	    /* Smooth the dy values and use them to calculate the frequency cutoff for the main filter */
	    self.prev_y = self.prev_y.lerp( in_y, alpha as f32);
        return self.prev_y;
    }
}
// Per eye view parameters
// todo: send together with video frame
#[derive(Serialize, Deserialize, Clone, Copy, Default)]
pub struct ViewParams {
    pub pose: Pose,
    pub fov: Fov,
}

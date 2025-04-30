use alvr_common::glam::Vec3;
use openxr::{self as xr, raw, sys};
use std::ptr;

use alvr_graphics::EYE_CONVERGENCE;
use std::ffi::{CStr, CString, OsStr};
use std::os::raw::{c_uint, c_void};
use std::{mem, mem::MaybeUninit, path::Path, rc::Rc, slice, sync::Arc, thread, time::Duration};

#[cfg(target_os = "android")]
extern crate libc;
#[cfg(target_os = "android")]
use libc::{c_char, c_int, dlopen, dlsym, RTLD_DEFAULT, RTLD_NOW};

#[repr(C)]
#[derive(Debug)]
struct YVR_EyeTrackingData {
    timestamp: u64,
    flags_comb: u64,
    eyeLFlags: u64,
    field_18: u32,
    field_1C: u32,
    eyeLOrigin_x: f32,
    eyeLOrigin_y: f32,
    eyeLOrigin_z: f32,
    field_2C: u32, // validity?
    eyeLDirection_x: f32,
    eyeLDirection_y: f32,
    eyeLDirection_z: f32,
    field_3C: u32, // validity?
    field_40: u32,
    field_44: u32,
    field_48: u32,
    field_4C: u32,
    field_50: u32,
    field_54: u32,
    field_58: u32,
    field_5C: u32,
    field_60: u32,
    field_64: u32,
    field_68: u32,
    field_6C: u32,
    field_70: u32,
    eyeLPupilDiameterMm: f32,
    field_78: u32, // validity?
    field_7C: u32,
    eyeLPositionGuideX: f32,
    eyeLPositionGuideY: f32,
    field_88: u32,
    field_8C: u32,
    field_90: u32,
    field_94: u32,
    field_98: u32,
    field_9C: u32,
    field_A0: u32,
    field_A4: u32,
    field_A8: u32,
    field_AC: u32,
    field_B0: u32,
    field_B4: u32,
    field_B8: u32,
    field_BC: u32,
    field_C0: u32,
    field_C4: u32,
    field_C8: u32,
    field_CC: u32,
    field_D0: u32,
    field_D4: u32,
    field_D8: u32,
    field_DC: u32,
    field_E0: u32,
    field_E4: u32,
    field_E8: u32,
    field_EC: u32,
    field_F0: u32,
    field_F4: u32,
    field_F8: u32,
    field_FC: u32,
    field_100: u32,
    field_104: u32,
    field_108: u32,
    field_10C: u32,
    eyeRFlags: u64,
    field_118: u32,
    field_11C: u32,
    eyeROrigin_x: f32,
    eyeROrigin_y: f32,
    eyeROrigin_z: f32,
    field_12C: u32,
    eyeRDirection_x: f32,
    eyeRDirection_y: f32,
    eyeRDirection_z: f32,
    field_13C: u32,
    field_140: u32,
    field_144: u32,
    field_148: u32,
    field_14C: u32,
    field_150: u32,
    field_154: u32,
    field_158: u32,
    field_15C: u32,
    field_160: u32,
    field_164: u32,
    field_168: u32,
    field_16C: u32,
    field_170: u32,
    eyeRPupilDiameterMm: f32,
    field_178: u32,
    field_17C: u32,
    eyeRPositionGuideX: f32,
    eyeRPositionGuideY: f32,
    field_188: u32,
    field_18C: u32,
    field_190: u32,
    field_194: u32,
    field_198: u32,
    field_19C: u32,
    field_1A0: u32,
    field_1A4: u32,
    field_1A8: u32,
    field_1AC: u32,
    field_1B0: u32,
    field_1B4: u32,
    field_1B8: u32,
    field_1BC: u32,
    field_1C0: u32,
    field_1C4: u32,
    field_1C8: u32,
    field_1CC: u32,
    field_1D0: u32,
    field_1D4: u32,
    field_1D8: u32,
    field_1DC: u32,
    field_1E0: u32,
    field_1E4: u32,
    field_1E8: u32,
    field_1EC: u32,
    field_1F0: u32,
    field_1F4: u32,
    field_1F8: u32,
    field_1FC: u32,
    field_200: u32,
    field_204: u32,
    field_208: u32,
    field_20C: u32,
    eyeCombOrigin_x: f32,
    eyeCombOrigin_y: f32,
    eyeCombOrigin_z: f32,
    field_21C: u32,
    eyeCombDirection_x: f32,
    eyeCombDirection_y: f32,
    eyeCombDirection_z: f32,
    gazeConvergenceDistance: f32,
    field_230: u32,
    field_234: u32,
    field_238: u32,
    field_23C: u32,
    field_240: u32,
    field_244: u32,
    eyeFoveatedGazeDirection_x: f32,
    eyeFoveatedGazeDirection_y: f32,
    eyeFoveatedGazeDirection_z: f32,
    field_254: u32,
    gazeInterocularDistance: f32,
    field_25C: u32,
    field_260: u32,
    field_264: u32,
    field_268: u32,
    field_26C: u32,
    field_270: u32,
    field_274: u32,
    field_278: u32,
    field_27C: u32,
}

pub struct EyeTrackerSocial {
    handle: sys::EyeTrackerFB,
    ext_fns: raw::EyeTrackingSocialFB,
    fn_GetEyeDataET: Option<extern "C" fn(time: i64, out: *mut YVR_EyeTrackingData) -> f64>,
}

impl EyeTrackerSocial {
    pub fn new<G>(session: &xr::Session<G>) -> xr::Result<Self> {
        let ext_fns = session
            .instance()
            .exts()
            .fb_eye_tracking_social
            .ok_or(sys::Result::ERROR_EXTENSION_NOT_PRESENT)?;

        let mut handle = sys::EyeTrackerFB::NULL;
        let info = sys::EyeTrackerCreateInfoFB {
            ty: sys::EyeTrackerCreateInfoFB::TYPE,
            next: ptr::null(),
        };
        unsafe {
            super::xr_res((ext_fns.create_eye_tracker)(
                session.as_raw(),
                &info,
                &mut handle,
            ))?
        };

        let mut fn_GetEyeDataET: Option<
            extern "C" fn(time: i64, out: *mut YVR_EyeTrackingData) -> f64,
        > = {
            let libtrackingclient = CString::new("libtrackingclient.so").unwrap();
            let name = CString::new("TRACKINGAPI_GetAPI").unwrap();
            let name2 = CString::new("dlsym").unwrap();
            let handle = unsafe { dlopen(libtrackingclient.as_ptr(), RTLD_NOW) };
            if !handle.is_null() {
                let fnptr = unsafe { dlsym(handle, name.as_ptr()) };

                if !fnptr.is_null() {
                    let mut apitable: *mut c_void = std::ptr::null_mut();
                    unsafe {
                        let fn_getapi: extern "C" fn(
                            version: c_uint,
                            out: *mut *mut c_void,
                        ) -> f64 = std::mem::transmute(fnptr);
                        fn_getapi(0x2710, &mut apitable as *mut *mut c_void);
                    }

                    if !apitable.is_null() {
                        unsafe {
                            let func_table: &[*const c_void] =
                                slice::from_raw_parts(apitable as *const *const c_void, 0x110);
                            if func_table[71].is_null() {
                                None
                            } else {
                                let fn_GetEyeDataET: extern "C" fn(
                                    time: i64,
                                    out: *mut YVR_EyeTrackingData,
                                )
                                    -> f64 = std::mem::transmute(func_table[71]);
                                Some(fn_GetEyeDataET)
                            }
                        }
                    } else {
                        None
                    }
                } else {
                    None
                }
            } else {
                None
            }
        };

        Ok(Self {
            handle,
            ext_fns,
            fn_GetEyeDataET,
        })
    }

    pub fn get_eye_gazes(
        &self,
        base: &xr::Space,
        time: xr::Time,
    ) -> xr::Result<[Option<xr::Posef>; 2]> {
        let gaze_info = sys::EyeGazesInfoFB {
            ty: sys::EyeGazesInfoFB::TYPE,
            next: ptr::null(),
            base_space: base.as_raw(),
            time,
        };

        let mut eye_gazes = sys::EyeGazesFB::out(ptr::null_mut());

        let eye_gazes = unsafe {
            super::xr_res((self.ext_fns.get_eye_gazes)(
                self.handle,
                &gaze_info,
                eye_gazes.as_mut_ptr(),
            ))?;

            eye_gazes.assume_init()
        };

        let left_valid: bool = eye_gazes.gaze[0].is_valid.into();
        let right_valid: bool = eye_gazes.gaze[1].is_valid.into();

        if left_valid && right_valid {
            let l_orient = crate::from_xr_quat(eye_gazes.gaze[0].gaze_pose.orientation);
            let l_forward = l_orient * Vec3::new(0.0, 0.0, -1.0);

            let r_orient = crate::from_xr_quat(eye_gazes.gaze[1].gaze_pose.orientation);
            let r_forward = r_orient * Vec3::new(0.0, 0.0, -1.0);

            let p1 = crate::from_xr_vec3(eye_gazes.gaze[0].gaze_pose.position); // position
            let d1 = l_forward; // forward
            let p2 = crate::from_xr_vec3(eye_gazes.gaze[1].gaze_pose.position);
            let d2 = r_forward;
            let p_avg = (p1 + p2) / 2.0;

            let p1_p2 = p2 - p1;
            let d1_dot_d1 = d1.dot(d1);
            let d2_dot_d2 = d2.dot(d2);
            let d1_dot_d2 = d1.dot(d2);
            let p1_p2_dot_d1 = p1_p2.dot(d1);
            let p1_p2_dot_d2 = p1_p2.dot(d2);

            let denom = d1_dot_d1 * d2_dot_d2 - d1_dot_d2 * d1_dot_d2;
            if denom.abs() < 1e-6 {
                //return None; // Rays are nearly parallel
                //alvr_common::info!("Eyes are parallel l: {p1} {d1} r: {p2} {d2}");
            } else {
                let s = (p1_p2_dot_d1 * d2_dot_d2 - p1_p2_dot_d2 * d1_dot_d2) / denom;
                let t = (p1_p2_dot_d1 * d1_dot_d2 - p1_p2_dot_d2 * d1_dot_d1) / denom;

                let closest_p1 = p1 + s * d1;
                let closest_p2 = p2 + t * d2;

                let closest_p1_dist_from_eyes = p_avg.distance(closest_p1);
                let closest_p2_dist_from_eyes = p_avg.distance(closest_p2);

                let depth = ((closest_p1_dist_from_eyes + closest_p2_dist_from_eyes) / 2.0).abs(); // Approximate depth
                                                                                                   //alvr_common::error!("Eyes are converged at: {depth} l: {p1} {d1} r: {p2} {d2}");
                if depth > 5.0 {
                    //unsafe { EYE_CONVERGENCE = 100.0; }
                } else if depth > 0.1 {
                    //unsafe { EYE_CONVERGENCE = depth; }
                } else {
                    //unsafe { EYE_CONVERGENCE = 0.1; }
                }
            }
        }

        if let Some(fn_GetEyeDataET) = self.fn_GetEyeDataET {
            let mut dataout: YVR_EyeTrackingData = unsafe { mem::zeroed() };

            unsafe {
                fn_GetEyeDataET(time.as_nanos(), &mut dataout);
            }

            //0x2710, table

            //alvr_common::error!("asdf {handle:p} {fnptr:p} {apitable:p}");
            let gazeConvergenceDistance = dataout.gazeConvergenceDistance;
            let gazeInterocularDistance = dataout.gazeInterocularDistance;
            let eyeLOrigin_x = dataout.eyeLOrigin_x;
            let eyeLOrigin_y = dataout.eyeLOrigin_y;
            let eyeLOrigin_z = dataout.eyeLOrigin_z;
            let eyeLDirection_x = dataout.eyeLDirection_x;
            let eyeLDirection_y = dataout.eyeLDirection_y;
            let eyeLDirection_z = dataout.eyeLDirection_z;

            let eyeROrigin_x = dataout.eyeROrigin_x;
            let eyeROrigin_y = dataout.eyeROrigin_y;
            let eyeROrigin_z = dataout.eyeROrigin_z;
            let eyeRDirection_x = dataout.eyeRDirection_x;
            let eyeRDirection_y = dataout.eyeRDirection_y;
            let eyeRDirection_z = dataout.eyeRDirection_z;

            let eyeCombOrigin_x = dataout.eyeCombOrigin_x;
            let eyeCombOrigin_y = dataout.eyeCombOrigin_y;
            let eyeCombOrigin_z = dataout.eyeCombOrigin_z;
            let eyeCombDirection_x = dataout.eyeCombDirection_x;
            let eyeCombDirection_y = dataout.eyeCombDirection_y;
            let eyeCombDirection_z = dataout.eyeCombDirection_z;

            let eyeFoveatedGazeDirection_x = dataout.eyeFoveatedGazeDirection_x;
            let eyeFoveatedGazeDirection_y = dataout.eyeFoveatedGazeDirection_y;
            let eyeFoveatedGazeDirection_z = dataout.eyeFoveatedGazeDirection_z;

            let eyeLPupilDiameterMm = dataout.eyeLPupilDiameterMm;
            let eyeLPositionGuideX = dataout.eyeLPositionGuideX;
            let eyeLPositionGuideY = dataout.eyeLPositionGuideY;
            let eyeRPupilDiameterMm = dataout.eyeRPupilDiameterMm;
            let eyeRPositionGuideX = dataout.eyeRPositionGuideX;
            let eyeRPositionGuideY = dataout.eyeRPositionGuideY;
            //alvr_common::error!("{gazeConvergenceDistance} {gazeInterocularDistance} L: {eyeLOrigin_x},{eyeLOrigin_y},{eyeLOrigin_z} {eyeLDirection_x},{eyeLDirection_y},{eyeLDirection_z}            R: {eyeROrigin_x},{eyeROrigin_y},{eyeROrigin_z} {eyeRDirection_x},{eyeRDirection_y},{eyeRDirection_z}      Comb: {eyeCombOrigin_x},{eyeCombOrigin_y},{eyeCombOrigin_z} {eyeCombDirection_x},{eyeCombDirection_y},{eyeCombDirection_z}    Foveated: {eyeFoveatedGazeDirection_x},{eyeFoveatedGazeDirection_y},{eyeFoveatedGazeDirection_z}");
            //alvr_common::error!("{dataout:?}");
            //alvr_common::error!("{eyeLPupilDiameterMm} {eyeLPositionGuideX} {eyeLPositionGuideY} {eyeRPupilDiameterMm} {eyeRPositionGuideX} {eyeRPositionGuideY}");

            let real_left_valid = true;
            let real_right_valid = true;
            if real_left_valid && real_right_valid {
                let l_forward = Vec3::new(eyeLDirection_x, eyeLDirection_y, eyeLDirection_z);

                let r_forward = Vec3::new(eyeRDirection_x, eyeRDirection_y, eyeRDirection_z);

                let p1 = Vec3::new(eyeLOrigin_x, eyeLOrigin_y, eyeLOrigin_z);
                let d1 = l_forward; // forward
                let p2 = Vec3::new(eyeROrigin_x, eyeROrigin_y, eyeROrigin_z);
                let d2 = r_forward;
                let p_avg = (p1 + p2) / 2.0;

                let p1_p2 = p2 - p1;
                let d1_dot_d1 = d1.dot(d1);
                let d2_dot_d2 = d2.dot(d2);
                let d1_dot_d2 = d1.dot(d2);
                let p1_p2_dot_d1 = p1_p2.dot(d1);
                let p1_p2_dot_d2 = p1_p2.dot(d2);

                let denom = d1_dot_d1 * d2_dot_d2 - d1_dot_d2 * d1_dot_d2;
                if denom.abs() < 1e-6 {
                    //return None; // Rays are nearly parallel
                    //alvr_common::info!("Eyes are parallel l: {p1} {d1} r: {p2} {d2}");
                } else {
                    let s = (p1_p2_dot_d1 * d2_dot_d2 - p1_p2_dot_d2 * d1_dot_d2) / denom;
                    let t = (p1_p2_dot_d1 * d1_dot_d2 - p1_p2_dot_d2 * d1_dot_d1) / denom;

                    let closest_p1 = p1 + s * d1;
                    let closest_p2 = p2 + t * d2;

                    let closest_p1_dist_from_eyes = p_avg.distance(closest_p1);
                    let closest_p2_dist_from_eyes = p_avg.distance(closest_p2);

                    let depth =
                        ((closest_p1_dist_from_eyes + closest_p2_dist_from_eyes) / 2.0).abs(); // Approximate depth
                                                                                               //alvr_common::error!("Real Eyes are converged at: {depth} l: {p1} {d1} r: {p2} {d2}");

                    let alpha = 0.995;
                    //let smoothed_depth = unsafe {((EYE_CONVERGENCE*alpha)+(depth*(2.0-alpha)))*0.5};
                    let mut smoothed_depth = unsafe { EYE_CONVERGENCE };
                    if depth > smoothed_depth {
                        smoothed_depth += 0.015;
                    } else {
                        smoothed_depth -= 0.015;
                    }
                    if smoothed_depth > 80.0 {
                        smoothed_depth = 2.0;
                    }
                    unsafe {
                        EYE_CONVERGENCE = smoothed_depth;
                    }

                    alvr_common::error!("Real Eyes are converged at: {depth} {smoothed_depth}");
                }
            }
        }

        Ok([
            left_valid.then(|| eye_gazes.gaze[0].gaze_pose),
            right_valid.then(|| eye_gazes.gaze[1].gaze_pose),
        ])
    }
}

impl Drop for EyeTrackerSocial {
    fn drop(&mut self) {
        unsafe {
            (self.ext_fns.destroy_eye_tracker)(self.handle);
        }
    }
}

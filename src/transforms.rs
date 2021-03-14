use nalgebra::geometry::{UnitQuaternion, Isometry3,Point3, Translation3};
use nalgebra::geometry;
use rosrust_msg::geometry_msgs::{Transform, Pose, Vector3, Quaternion,
    TransformStamped};
use rosrust_msg::std_msgs::Header;


pub fn isometry_from_pose(pose: &Pose) -> Isometry3<f64>{
    let trans = Translation3::new(pose.position.x, pose.position.y,
                                 pose.position.z);
    let rot = UnitQuaternion::new_normalize(
        nalgebra::geometry::Quaternion::new(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z));

    Isometry3::from_parts(trans, rot)
    }

pub fn isometry_from_transform(tf: &Transform) -> Isometry3<f64>{
    let trans = Translation3::new(tf.translation.x, tf.translation.y,
                                 tf.translation.z);
    let rot = UnitQuaternion::new_normalize(
        nalgebra::geometry::Quaternion::new(
            tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z));

    Isometry3::from_parts(trans, rot)
    }


pub fn isometry_to_transform(iso: Isometry3<f64>) -> Transform{
    Transform{
        translation: Vector3{
            x: iso.translation.x,
            y: iso.translation.y,
            z: iso.translation.z
        },
        rotation: Quaternion{
            x: iso.rotation.i,
            y: iso.rotation.j,
            z: iso.rotation.k,
            w: iso.rotation.w
        }
    }
}

pub fn get_inverse(trans: &TransformStamped) -> TransformStamped{
    TransformStamped{
        header: Header{
            seq: 1u32,
            stamp: trans.header.stamp,
            frame_id: trans.child_frame_id.clone()
        },
        child_frame_id: trans.header.frame_id.clone(),
        transform: isometry_to_transform(
            isometry_from_transform(&trans.transform).clone().inverse())
    }
}

///Chain multiple transforms together. Takes in a vector of transforms. The vector should be in order of desired transformations
pub fn chain_transforms(transforms: &Vec<Transform>) -> Transform {
    let mut final_transform = Isometry3::identity();
    for t in transforms {
        let tf = isometry_from_transform(&t);
        final_transform = final_transform * tf;
    }
    isometry_to_transform(final_transform)
}

pub fn interpolate(t1: Transform, t2: Transform, weight: f64) -> Transform {
    let r1 = geometry::Quaternion::new(t1.rotation.w, t1.rotation.x, t1.rotation.y, t1.rotation.z);
    let r2 = geometry::Quaternion::new(t2.rotation.w, t2.rotation.x, t2.rotation.y, t2.rotation.z);
    let r1 = geometry::UnitQuaternion::from_quaternion(r1);
    let r2 = geometry::UnitQuaternion::from_quaternion(r2);
    let res  = r1.try_slerp(&r2, weight, 1e-9);
    match res {
        Some(qt) => {
            Transform{
                translation: Vector3{
                    x: t1.translation.x * weight + t2.translation.x * (1.0 - weight),
                    y: t1.translation.y * weight + t2.translation.y * (1.0 - weight),
                    z: t1.translation.z * weight + t2.translation.z * (1.0 - weight)
                },
                rotation: Quaternion{
                    x: qt.coords[0],
                    y: qt.coords[1],
                    z: qt.coords[2],
                    w: qt.coords[3]
                }
            }
        }
        None => {
            if weight > 0.5 {
                Transform{
                    translation: Vector3{
                        x: t1.translation.x * weight + t2.translation.x * (1.0 - weight),
                        y: t1.translation.y * weight + t2.translation.y * (1.0 - weight),
                        z: t1.translation.z * weight + t2.translation.z * (1.0 - weight)
                    },
                    rotation: t1.rotation.clone()
                }
            }
            else {
                Transform{
                    translation: Vector3{
                        x: t1.translation.x * weight + t2.translation.x * (1.0 - weight),
                        y: t1.translation.y * weight + t2.translation.y * (1.0 - weight),
                        z: t1.translation.z * weight + t2.translation.z * (1.0 - weight)
                    },
                    rotation: t2.rotation.clone()
                }
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_basic_translation_chaining(){
        let tf1 = Transform {
            translation: Vector3{x: 1f64, y: 1f64, z: 0f64},
            rotation: Quaternion{x: 0f64, y: 0f64, z: 0f64, w: 1f64}
        };
        let expected_tf = Transform {
            translation: Vector3{x: 2f64, y: 2f64, z: 0f64},
            rotation: Quaternion{x: 0f64, y: 0f64, z: 0f64, w: 1f64}
        };
        let transform_chain = vec!(tf1, tf1);
        let res = chain_transforms(&transform_chain);
        assert_eq!(res, expected_tf);
    }

    #[test]
    fn test_basic_interpolation() {
        let tf1 = Transform {
            translation: Vector3{x: 1f64, y: 1f64, z: 0f64},
            rotation: Quaternion{x: 0f64, y: 0f64, z: 0f64, w: 1f64}
        };
        let tf2 = Transform {
            translation: Vector3{x: 2f64, y: 2f64, z: 0f64},
            rotation: Quaternion{x: 0f64, y: 0f64, z: 0f64, w: 1f64}
        };
        let expected = Transform {
            translation: Vector3{x: 1.5f64, y: 1.5f64, z: 0f64},
            rotation: Quaternion{x: 0f64, y: 0f64, z: 0f64, w: 1f64}
        };
        assert_eq!(interpolate(tf1, tf2, 0.5), expected);
    }
}
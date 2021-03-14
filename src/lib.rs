//! This is a rust port of the [ROS tf library](http://wiki.ros.org/tf). It is intended for being used in robots to help keep track of
//! multiple coordinate frames and is part of a larger suite of rust libraries that provide support for various robotics related functionality.
//!
//! Example usage:
//! ```ignore
//! fn main() {
//!     rosrust::init("listener");
//!     let listener = TfListener::new();
//!
//!     let rate = rosrust::rate(1.0);
//!     while rosrust::is_ok() {
//!         let tf = listener.lookup_transform("camera", "base_link", ros::Time::now());
//!         println!("{:?}", tf);
//!         rate.sleep();
//!     }
//! }
//!```
use std::collections::HashMap;
use std::collections::VecDeque;
use std::collections::HashSet;
use std::cmp::Ordering;
use std::sync::{Arc, RwLock};
use rosrust_msg::geometry_msgs::{Transform, Pose, Vector3, Quaternion,
    TransformStamped};
use rosrust_msg::std_msgs::Header;
use rosrust_msg::tf2_msgs::TFMessage;

mod transforms;

#[derive(Clone, Debug)]
struct OrderedTF{
    tf: TransformStamped
}

impl OrderedTF{
    pub fn new(trans: TransformStamped) -> Self {
        return OrderedTF{tf: trans};
    }
}

impl PartialEq for OrderedTF {
    fn eq(&self, other: &Self) -> bool{
        self.tf.header.stamp == other.tf.header.stamp}
}

impl Eq for OrderedTF{}

impl Ord for  OrderedTF {
    fn cmp(&self, other: &Self) -> Ordering {
        self.tf.header.stamp.cmp(&other.tf.header.stamp)
    }
}

impl PartialOrd for OrderedTF {
    fn partial_cmp(&self, other: &Self)  -> Option<Ordering> {
        Some(self.tf.header.stamp.cmp(&other.tf.header.stamp))
    }
}

/// Calculates the inverse of a ros transform

/// Enumerates the different types of errors
#[derive(Clone, Debug)]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    AttemptedLookupInPast,
    /// Error due ti the transform not yet being available.
    AttemptedLookUpInFuture,
    /// There is no path between the from and to frame.
    CouldNotFindTransform,
    /// In the event that a write is simultaneously happening with a read of the same tf buffer
    CouldNotAcquireLock
}


fn get_nanos(dur: rosrust::Duration) -> i64 {
    i64::from(dur.sec) * 1_000_000_000 + i64::from(dur.nsec)
}

fn to_transform_stamped(
    tf: Transform,
    from: std::string::String,
    to: std::string::String,
    time: rosrust::Time) -> TransformStamped{
        TransformStamped{
            header: Header{frame_id: from, stamp: time, seq: 1u32},
            child_frame_id: to,
            transform: tf
        }
    }

#[derive(Clone, Debug)]
struct TfIndividualTransformChain {
    buffer_size: usize,
    static_tf: bool,
    //TODO:  Implement a circular buffer. Current method is slowww.
    transform_chain: Vec<OrderedTF>
}


impl TfIndividualTransformChain {
    pub fn new(static_tf: bool) -> Self {
        return TfIndividualTransformChain{
            buffer_size: 100, transform_chain:Vec::new(),
            static_tf: static_tf};
    }

    pub fn add_to_buffer(&mut self, msg: TransformStamped) {

        let res = self.transform_chain.binary_search(
            &OrderedTF{tf: msg.clone()});

        match res {
            Ok(x) => self.transform_chain.insert(x, OrderedTF{tf: msg}),
            Err(x) => self.transform_chain.insert(x, OrderedTF{tf: msg})
        }

        if self.transform_chain.len() > self.buffer_size {
            self.transform_chain.remove(0);
        }
    }

    pub fn get_closest_transform(
            &self, time: rosrust::Time) -> Result<TransformStamped, TfError> {
        if self.static_tf {
            return Ok(
                self.transform_chain.get(
                    self.transform_chain.len()-1).unwrap().tf.clone());
        }

        let mut res = TransformStamped::default();
        res.header.stamp = time;
        res.transform.rotation.w = 1f64;

        let res = self.transform_chain.binary_search(&OrderedTF{tf: res});

        match res {
            Ok(x)=> return Ok(
                self.transform_chain.get(x).unwrap().tf.clone()),
            Err(x)=> {
                if x == 0 {
                    return Err(TfError::AttemptedLookupInPast);
                }
                if x >= self.transform_chain.len() {
                    return Err(TfError::AttemptedLookUpInFuture)
                }
                let tf1 = self.transform_chain.get(x-1).unwrap().clone().tf.transform;
                let tf2 = self.transform_chain.get(x).unwrap().clone().tf.transform;
                let time1 = self.transform_chain.get(x-1).unwrap().tf.header.stamp;
                let time2 = self.transform_chain.get(x).unwrap().tf.header.stamp;
                let header = self.transform_chain.get(x).unwrap().tf.header.clone();
                let child_frame = self.transform_chain.get(x).unwrap().tf.child_frame_id.clone();
                let total_duration = get_nanos(time2 - time1) as f64;
                let desired_duration = get_nanos(time - time1) as f64;
                let weight = 1.0 - desired_duration/total_duration;
                let final_tf = transforms::interpolate(tf1, tf2, weight);
                let ros_msg = to_transform_stamped(
                    final_tf, header.frame_id, child_frame, time);
                Ok(ros_msg)
            }
        }
    }
}

#[derive(Clone,Debug,Hash)]
struct TfGraphNode {
    child: String,
    parent: String
}

impl PartialEq for TfGraphNode {
    fn eq(&self, other: &Self) -> bool {
        self.child == other.child && self.parent == other.parent
    }
}

impl Eq for TfGraphNode {}

#[derive(Clone, Debug)]
struct TfBuffer {
    child_transform_index: HashMap<String, HashSet<String> >,
    transform_data: HashMap<TfGraphNode, TfIndividualTransformChain>
}


impl TfBuffer {

    fn new() -> Self {
        TfBuffer{
            child_transform_index: HashMap::new(),
            transform_data: HashMap::new()}
    }

    fn handle_incoming_transforms(
            &mut self, transforms: TFMessage, static_tf: bool) {
        for transform in transforms.transforms {
            self.add_transform(&transform, static_tf);
            self.add_transform(&transforms::get_inverse(&transform), static_tf);
        }
    }

    fn add_transform (
            &mut self, transform: &TransformStamped, static_tf: bool) {
        //TODO: Detect is new transform will create a loop
        if self.child_transform_index.contains_key(&transform.header.frame_id) {
            let res = self.child_transform_index.get_mut(
                &transform.header.frame_id.clone()).unwrap();
            res.insert(transform.child_frame_id.clone());
        }
        else {
            self.child_transform_index.insert(
                transform.header.frame_id.clone(), HashSet::new());
            let res = self.child_transform_index.get_mut(
                &transform.header.frame_id.clone()).unwrap();
            res.insert(transform.child_frame_id.clone());
        }

        let key = TfGraphNode{child: transform.child_frame_id.clone(), parent: transform.header.frame_id.clone()};

        if self.transform_data.contains_key(&key) {
            let data = self.transform_data.get_mut(&key).unwrap();
            data.add_to_buffer(transform.clone());
        }
        else {
            let mut data = TfIndividualTransformChain::new(static_tf);
            data.add_to_buffer(transform.clone());
            self.transform_data.insert(key, data);
        }
    }

    /// Retrieves the transform path
    fn retrieve_transform_path(
            &self, from: String, to: String) -> Result<Vec<String>, TfError> {
        let mut res = vec!();
        let mut frontier: VecDeque<String> = VecDeque::new();
        let mut visited: HashSet<String> = HashSet::new();
        let mut parents: HashMap<String, String> = HashMap::new();
        visited.insert(from.clone());
        frontier.push_front(from.clone());

        while !frontier.is_empty() {
            let current_node = frontier.pop_front().unwrap();
            if current_node == to {
                break;
            }
            let children = self.child_transform_index.get(&current_node);
            match children {
                Some(children) => {
                    for  v in children {
                        if visited.contains(&v.to_string()) {
                            continue;
                        }
                        parents.insert(v.to_string(), current_node.clone());
                        frontier.push_front(v.to_string());
                        visited.insert(v.to_string());
                    }
                },
                None => {}
            }

        }
        let mut r = to;
        while r != from {
            res.push(r.clone());
            let parent = parents.get(&r);

            match parent {
                Some(x) => {
                    r = x.to_string()
                },
                None => return Err(TfError::CouldNotFindTransform)
            }
        }
        res.reverse();
        Ok(res)
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform(
            &self, from: &str, to: &str, time: rosrust::Time
        ) -> Result<TransformStamped,TfError> {
        let from = from.to_string();
        let to = to.to_string();
        let path = self.retrieve_transform_path(from.clone(), to.clone());

        match path {
            Ok(path) => {
                let mut tflist: Vec<Transform> = Vec::new();
                let mut first = from.clone();
                for intermediate in path {
                    let node = TfGraphNode{child: intermediate.clone(), parent: first.clone()};
                    let time_cache = self.transform_data.get(&node).unwrap();
                    let transform = time_cache.get_closest_transform(time);
                    match transform {
                        Err(e) => return Err(e),
                        Ok(x) => {
                            tflist.push(x.transform);
                        }
                    }
                    first = intermediate.clone();
                }
                let final_tf = transforms::chain_transforms(&tflist);
                let msg = TransformStamped {
                    child_frame_id: to.clone(),
                    header: Header {
                        frame_id: from.clone(),
                        stamp: time,
                        seq: 1
                    },
                    transform: final_tf
                };
                return Ok(msg)
            },
            Err(x) => return Err(x)
        };
    }

    fn lookup_transform_with_time_travel(
            &self,
            to: &str,
            time2: rosrust::Time,
            from: &str,
            time1: rosrust::Time,
            fixed_frame: &str
        ) ->  Result<TransformStamped,TfError> {
        let tf1 = self.lookup_transform(from, fixed_frame, time1);
        let tf2 = self.lookup_transform(to, fixed_frame, time2);
        match tf1 {Err(x) => return Err(x), Ok(_)=>{}}
        match tf2 {Err(x) => return Err(x), Ok(_)=>{}}
        let transforms = transforms::get_inverse(&tf1.unwrap());
        let result = transforms::chain_transforms(&vec!(tf2.unwrap().transform, transforms.transform));
        Ok(to_transform_stamped(result, from.to_string(), to.to_string(), time1))
    }
}

#[cfg(test)]
mod test {
    use super::*;
    /// This function builds a tree consisting of the following items:
    /// * a world coordinate frame
    /// * an item in the world frame at (1,0,0)
    /// * base_link of a robot starting at (0,0,0) and progressing at (0,t,0) where t is time in seconds
    /// * a camera which is (0.5, 0, 0) from the base_link
    fn build_test_tree(buffer: &mut TfBuffer, time: f64) {

        let nsecs = ((time - ((time.floor() as i64) as f64))*1E9) as u32;

        let world_to_item = TransformStamped {
            child_frame_id: "item".to_string(),
            header: Header {
                frame_id: "world".to_string(),
                stamp: rosrust::Time{sec: time.floor() as u32, nsec: nsecs},
                seq: 1
            },
            transform: Transform{
                rotation: Quaternion{
                    x: 0f64, y: 0f64, z: 0f64, w: 1f64
                },
                translation: Vector3{
                    x: 1f64, y: 0f64, z: 0f64
                }
           }
        };
        buffer.add_transform(&world_to_item, true);
        buffer.add_transform(&transforms::get_inverse(&world_to_item), true);

        let world_to_base_link = TransformStamped {
            child_frame_id: "base_link".to_string(),
            header: Header {
                frame_id: "world".to_string(),
                stamp: rosrust::Time{sec: time.floor() as u32, nsec: nsecs},
                seq: 1
            },
            transform: Transform{
                rotation: Quaternion{
                    x: 0f64, y: 0f64, z: 0f64, w: 1f64
                },
                translation: Vector3{
                    x: 0f64, y: time, z: 0f64
                }
           }
        };
        buffer.add_transform(&world_to_base_link, false);
        buffer.add_transform(&transforms::get_inverse(&world_to_base_link),  false);

        let base_link_to_camera = TransformStamped {
            child_frame_id: "camera".to_string(),
            header: Header {
                frame_id: "base_link".to_string(),
                stamp: rosrust::Time{sec: time.floor() as u32, nsec: nsecs},
                seq: 1
            },
            transform: Transform{
                rotation: Quaternion{
                    x: 0f64, y: 0f64, z: 0f64, w:1f64
                },
                translation: Vector3{
                    x: 0.5f64, y: 0f64, z: 0f64
                }
           }
        };
        buffer.add_transform(&base_link_to_camera, true);
        buffer.add_transform(&get_inverse(&base_link_to_camera), true);
    }


    /// Tests a basic lookup
    #[test]
    fn test_basic_tf_lookup() {
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, 0f64);
        let res = tf_buffer.lookup_transform("camera", "item", rosrust::Time{sec:0, nsec:0});
        let expected = TransformStamped {
            child_frame_id: "item".to_string(),
            header: Header {
                frame_id: "camera".to_string(),
                stamp: rosrust::Time{sec:0, nsec:0},
                seq: 1
            },
            transform: Transform{
                rotation: Quaternion{
                    x: 0f64, y: 0f64, z: 0f64, w: 1f64
                },
                translation: Vector3{
                    x: 0.5f64, y: 0f64, z: 0f64
                }
            }
        };
        assert_eq!(res.unwrap(), expected);
    }

    /// Tests an interpolated lookup.
    #[test]
    fn test_basic_tf_interpolation() {
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, 0f64);
        build_test_tree(&mut tf_buffer, 1f64);
        let res = tf_buffer.lookup_transform("camera", "item", rosrust::Time{sec:0, nsec:700_000_000});
        let expected = TransformStamped {
            child_frame_id: "item".to_string(),
            header: Header {
                frame_id: "camera".to_string(),
                stamp: rosrust::Time{sec:0, nsec:700_000_000},
                seq: 1
            },
            transform: Transform{
                rotation: Quaternion{
                    x: 0f64, y: 0f64, z: 0f64, w: 1f64
                },
                translation: Vector3{
                    x: 0.5f64, y: -0.7f64, z: 0f64
                }
            }
        };
        assert_eq!(res.unwrap(), expected);
    }

    /// Tests an interpolated lookup.
    #[test]
    fn test_basic_tf_timetravel() {
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, 0f64);
        build_test_tree(&mut tf_buffer, 1f64);
        let res = tf_buffer.lookup_transform_with_time_travel("camera", rosrust::Time{sec:0, nsec: 400_000_000}, "camera", rosrust::Time{sec:0, nsec: 700_000_000}, "item");
        let expected = TransformStamped {
            child_frame_id: "camera".to_string(),
            header: Header {
                frame_id: "camera".to_string(),
                stamp: rosrust::Time{sec:0, nsec:700_000_000},
                seq: 0
            },
            transform: Transform{
                rotation: Quaternion{
                    x: 0f64, y: 0f64, z: 0f64, w: 1f64
                },
                translation: Vector3{
                    x: 0f64, y: 0.3f64, z: 0f64
                }
            }
        };
        assert_approx_eq(res.unwrap(), expected);
    }

    fn assert_approx_eq(msg1: TransformStamped, msg2: TransformStamped) {
        assert_eq!(msg1.header, msg2.header);
        assert_eq!(msg1.child_frame_id, msg2.child_frame_id);

        assert!((msg1.transform.rotation.x - msg2.transform.rotation.x).abs() < 1e-9);
        assert!((msg1.transform.rotation.y - msg2.transform.rotation.y).abs() < 1e-9);
        assert!((msg1.transform.rotation.z - msg2.transform.rotation.z).abs() < 1e-9);
        assert!((msg1.transform.rotation.w - msg2.transform.rotation.w).abs() < 1e-9);

        assert!((msg1.transform.translation.x - msg2.transform.translation.x).abs() < 1e-9);
        assert!((msg1.transform.translation.y - msg2.transform.translation.y).abs() < 1e-9);
        assert!((msg1.transform.translation.z - msg2.transform.translation.z).abs() < 1e-9);
    }
}

///This struct tries to be the same as the C++ version of `TransformListener`. Use this struct to lookup transforms.
///
/// Example usage:
///
/// ```ignore
/// fn main() {
///     rosrust::init("listener");
///     let listener = TfListener::new();
///
///     let rate = rosrust::rate(1.0);
///     while rosrust::is_ok() {
///         let tf = listener.lookup_transform("camera", "base_link", ros::Time::now());
///         println!("{:?}", tf);
///         rate.sleep();
///     }
/// }
/// ```
/// Do note that unlike the C++ variant of the TfListener, only one TfListener can be created at a time. Like its C++ counterpart,
/// it must be scoped to exist through the lifetime of the program. One way to do this is using an `Arc` or `RwLock`.
pub struct TfListener {
    buffer: Arc<RwLock<TfBuffer>>,
    static_subscriber: rosrust::Subscriber,
    dynamic_subscriber:  rosrust::Subscriber,
}

impl TfListener {

    /// Create a new TfListener
    pub fn new() -> Self {
        let buff = RwLock::new(TfBuffer::new());
        let arc = Arc::new(buff);
        let r1 = arc.clone();
        let _subscriber_tf = rosrust::subscribe("tf", 100, move |v: TFMessage| {
            r1.write().unwrap().handle_incoming_transforms(v, true);
        }).unwrap();

        let r2 = arc.clone();
        let _subscriber_tf_static = rosrust::subscribe("tf_static", 100, move |v: TFMessage| {
            r2.write().unwrap().handle_incoming_transforms(v, true);
        }).unwrap();

        TfListener {
            buffer: arc.clone(),
            static_subscriber: _subscriber_tf_static,
            dynamic_subscriber: _subscriber_tf
        }
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform(&self, from: &str, to: &str, time: rosrust::Time) ->  Result<TransformStamped,TfError> {
        self.buffer.read().unwrap().lookup_transform(from, to, time)
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform_with_time_travel(&self, from: &str, time1: rosrust::Time, to: &str, time2: rosrust::Time, fixed_frame: &str) ->  Result<TransformStamped,TfError> {
        self.buffer.read().unwrap().lookup_transform_with_time_travel(from, time1, to, time2, fixed_frame)
    }
}
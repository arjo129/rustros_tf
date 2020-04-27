use std::collections::HashMap;
use std::collections::VecDeque;
use std::collections::HashSet; 
use std::cmp::Ordering;
pub mod msg;
pub mod transforms;

impl Eq for msg::geometry_msgs::TransformStamped {}

impl Ord for  msg::geometry_msgs::TransformStamped {
    fn cmp(&self, other: &msg::geometry_msgs::TransformStamped) -> Ordering {
        self.header.stamp.cmp(&other.header.stamp)
    }
}

impl PartialOrd for msg::geometry_msgs::TransformStamped {
    fn partial_cmp(&self, other: &msg::geometry_msgs::TransformStamped)  -> Option<Ordering> {
        Some(self.header.stamp.cmp(&other.header.stamp))
    }
}

/// Calculates the inverse of a ros transform
fn get_inverse(transform: &msg::geometry_msgs::TransformStamped) -> msg::geometry_msgs::TransformStamped {
    
    let m_transform = transforms::Transform {
        orientation: transforms::Quaternion{
            x: transform.transform.rotation.x,
            y: transform.transform.rotation.y,
            z: transform.transform.rotation.z,
            w: transform.transform.rotation.x,
        },
        position: transforms::Position{
            x: transform.transform.translation.x,
            y: transform.transform.translation.y,
            z: transform.transform.translation.z
        }
    };

    let inverse = transforms::invert_transform(&m_transform);

    let inv: msg::geometry_msgs::TransformStamped = msg::geometry_msgs::TransformStamped {
        child_frame_id: transform.header.frame_id.clone(),
        header: msg::std_msgs::Header {
            frame_id: transform.child_frame_id.clone(),
            stamp: transform.header.stamp,
            seq: transform.header.seq
        },
        transform: msg::geometry_msgs::Transform{
            rotation: msg::geometry_msgs::Quaternion{
                x: inverse.orientation.x, y:inverse.orientation.y, z: inverse.orientation.z, w: inverse.orientation.w
            },
            translation: msg::geometry_msgs::Vector3{
                x: inverse.position.x, y: inverse.position.y, z: inverse.position.z
            }
        }
    };
    inv
}

/// Enumerates the different types of errors
#[derive(Clone, Debug)]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    AttemptedLookupInPast, 
    /// Error due ti the transform not yet being available.
    AttemptedLookUpInFuture, 
    /// There is no path between the from and to frame.
    CouldNotFindTransform
}


fn to_transform(transform: msg::geometry_msgs::TransformStamped) -> transforms::Transform {
    transforms::Transform {
        orientation: transforms::Quaternion{
            x: transform.transform.rotation.x,
            y: transform.transform.rotation.y,
            z: transform.transform.rotation.z,
            w: transform.transform.rotation.w,
        },
        position: transforms::Position{
            x: transform.transform.translation.x,
            y: transform.transform.translation.y,
            z: transform.transform.translation.z
        }
    }
}

fn to_transform_stamped(transform: transforms::Transform, from: std::string::String, to: std::string::String, time: rosrust::Time) -> msg::geometry_msgs::TransformStamped {
    msg::geometry_msgs::TransformStamped {
        child_frame_id: to.clone(),
        header: msg::std_msgs::Header {
            frame_id: from.clone(),
            stamp: time,
            seq: 0
        },
        transform: msg::geometry_msgs::Transform{
            rotation: msg::geometry_msgs::Quaternion{
                x: transform.orientation.x, y:transform.orientation.y, z: transform.orientation.z, w: transform.orientation.w
            },
            translation: msg::geometry_msgs::Vector3{
                x: transform.position.x, y: transform.position.y, z: transform.position.z
            }
        }
    }
}

fn get_nanos(dur: rosrust::Duration) -> i64 {
    i64::from(dur.sec) * 1_000_000_000 + i64::from(dur.nsec)
}

#[derive(Clone, Debug)] 
struct TfIndividualTransformChain {
    buffer_size: usize,
    static_tf: bool,
    //TODO:  Implement a circular buffer. Current method is slowww.
    transform_chain: Vec<msg::geometry_msgs::TransformStamped>
}


impl TfIndividualTransformChain {
    pub fn new(static_tf: bool) -> Self {
        return TfIndividualTransformChain{buffer_size: 100, transform_chain:Vec::new(), static_tf: static_tf};
    }

    pub fn add_to_buffer(&mut self, msg: msg::geometry_msgs::TransformStamped) {
        
        let res = self.transform_chain.binary_search(&msg);
        
        match res {
            Ok(x) => self.transform_chain.insert(x, msg),
            Err(x) => self.transform_chain.insert(x, msg)
        }

        if self.transform_chain.len() > self.buffer_size {
            self.transform_chain.remove(0);
        }
    }

    pub fn get_closest_transform(&self, time: rosrust::Time) -> Result<msg::geometry_msgs::TransformStamped, TfError> {
        if self.static_tf {
            return Ok(self.transform_chain.get(self.transform_chain.len()-1).unwrap().clone());
        }

        let res = msg::geometry_msgs::TransformStamped {
            child_frame_id: "".to_string(),
            header: msg::std_msgs::Header {
                frame_id: "".to_string(),
                stamp: time,
                seq: 1
            },
            transform: msg::geometry_msgs::Transform{
                rotation: msg::geometry_msgs::Quaternion{
                    x: 0f64, y: 0f64, z: 0f64, w: 1f64
                },
                translation: msg::geometry_msgs::Vector3{
                    x: 0f64, y: 0f64, z: 0f64
                }
            }
        };

        let res = self.transform_chain.binary_search(&res);
        match res {
            Ok(x)=> return Ok(self.transform_chain.get(x).unwrap().clone()),
            Err(x)=> {
                if x == 0 {
                    return Err(TfError::AttemptedLookupInPast);
                }
                if x >= self.transform_chain.len() {
                    return Err(TfError::AttemptedLookUpInFuture)
                }
                let tf1 = to_transform(self.transform_chain.get(x-1).unwrap().clone());
                let tf2 = to_transform(self.transform_chain.get(x).unwrap().clone());
                let time1 = self.transform_chain.get(x-1).unwrap().header.stamp;
                let time2 = self.transform_chain.get(x).unwrap().header.stamp;
                let header = self.transform_chain.get(x).unwrap().header.clone();
                let child_frame = self.transform_chain.get(x).unwrap().child_frame_id.clone();
                let total_duration = get_nanos(time2 - time1) as f64;
                let desired_duration = get_nanos(time - time1) as f64;
                let weight = 1.0 - desired_duration/total_duration;
                let final_tf = transforms::interpolate(tf1, tf2, weight);
                let ros_msg = to_transform_stamped(final_tf, header.frame_id, child_frame, time);
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
pub struct TfBuffer {
    child_transform_index: HashMap<String, HashSet<String> >,
    transform_data: HashMap<TfGraphNode, TfIndividualTransformChain>
}


impl TfBuffer {

    fn new() -> Self {
        TfBuffer{child_transform_index: HashMap::new(), transform_data: HashMap::new()}
    }

    fn handle_incoming_transforms(&mut self, transforms: msg::tf2_msgs::TFMessage, static_tf: bool) {
        for transform in transforms.transforms {
            println!("Got transform {} {:?}-> {}", transform.header.frame_id, transform.header.stamp, transform.child_frame_id);
            self.add_transform(&transform, static_tf);
            self.add_transform(&get_inverse(&transform), static_tf);
        }
    }

    fn add_transform (&mut self, transform: &msg::geometry_msgs::TransformStamped, static_tf: bool) {
        //TODO: Detect is new transform will create a loop
        if self.child_transform_index.contains_key(&transform.header.frame_id) {
            let res = self.child_transform_index.get_mut(&transform.header.frame_id.clone()).unwrap();
            res.insert(transform.child_frame_id.clone());
        }
        else {
            self.child_transform_index.insert(transform.header.frame_id.clone(), HashSet::new());
            let res = self.child_transform_index.get_mut(&transform.header.frame_id.clone()).unwrap();
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
    fn retrieve_transform_path(&self, from: String, to: String) -> Result<Vec<String>, TfError> {
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
    pub fn lookup_transform(&self, from: &str, to: &str, time: rosrust::Time) -> Result<msg::geometry_msgs::TransformStamped,TfError> {
        let from = from.to_string();
        let to = to.to_string();
        let path = self.retrieve_transform_path(from.clone(), to.clone());
        
        match path {
            Ok(path) => {
                let mut tflist = Vec::<transforms::Transform>::new();
                let mut first = from.clone();
                for intermediate in path {
                    let node = TfGraphNode{child: intermediate.clone(), parent: first.clone()};
                    let time_cache = self.transform_data.get(&node).unwrap();
                    let transform = time_cache.get_closest_transform(time);
                    match transform {
                        Err(e) => return Err(e),
                        Ok(x) => {
                            let tf = transforms::Transform{
                                orientation: transforms::Quaternion{
                                    x: x.transform.rotation.x, 
                                    y: x.transform.rotation.y, 
                                    z: x.transform.rotation.z,
                                    w: x.transform.rotation.w
                                },
                                position: transforms::Position{
                                    x: x.transform.translation.x, 
                                    y: x.transform.translation.y, 
                                    z: x.transform.translation.z
                                }
                            } ;
                            tflist.push(tf);
                        }
                    }
                    first = intermediate.clone();                  
                }
                let final_tf = transforms::chain_transforms(&tflist);
                let msg = msg::geometry_msgs::TransformStamped {
                    child_frame_id: to.clone(),
                    header: msg::std_msgs::Header {
                        frame_id: from.clone(), 
                        stamp: time,
                        seq: 1
                    },
                    transform: msg::geometry_msgs::Transform{
                        rotation: msg::geometry_msgs::Quaternion{
                            x: final_tf.orientation.x, y: final_tf.orientation.y, z: final_tf.orientation.z, w: final_tf.orientation.w
                        },
                        translation: msg::geometry_msgs::Vector3{
                            x: final_tf.position.x, y: final_tf.position.y, z: final_tf.position.z
                        }
                    }
                };
                return Ok(msg)
            },
            Err(x) => return Err(x)
        }; 
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

        let world_to_item = msg::geometry_msgs::TransformStamped {
            child_frame_id: "item".to_string(),
            header: msg::std_msgs::Header {
                frame_id: "world".to_string(),
                stamp: rosrust::Time{sec: time.floor() as u32, nsec: nsecs},
                seq: 1
            },
            transform: msg::geometry_msgs::Transform{
                rotation: msg::geometry_msgs::Quaternion{
                    x: 0f64, y: 0f64, z: 0f64, w: 1f64
                },
                translation: msg::geometry_msgs::Vector3{
                    x: 1f64, y: 0f64, z: 0f64
                }
           }
        };
        buffer.add_transform(&world_to_item, true);
        buffer.add_transform(&get_inverse(&world_to_item), true);

        let world_to_base_link = msg::geometry_msgs::TransformStamped {
            child_frame_id: "base_link".to_string(),
            header: msg::std_msgs::Header {
                frame_id: "world".to_string(),
                stamp: rosrust::Time{sec: time.floor() as u32, nsec: nsecs},
                seq: 1
            },
            transform: msg::geometry_msgs::Transform{
                rotation: msg::geometry_msgs::Quaternion{
                    x: 0f64, y: 0f64, z: 0f64, w: 1f64
                },
                translation: msg::geometry_msgs::Vector3{
                    x: 0f64, y: time, z: 0f64
                }
           }
        };
        buffer.add_transform(&world_to_base_link, false);
        buffer.add_transform(&get_inverse(&world_to_base_link),  false);

        let base_link_to_camera = msg::geometry_msgs::TransformStamped {
            child_frame_id: "camera".to_string(),
            header: msg::std_msgs::Header {
                frame_id: "base_link".to_string(),
                stamp: rosrust::Time{sec: time.floor() as u32, nsec: nsecs},
                seq: 1
            },
            transform: msg::geometry_msgs::Transform{
                rotation: msg::geometry_msgs::Quaternion{
                    x: 0f64, y: 0f64, z: 0f64, w:1f64
                },
                translation: msg::geometry_msgs::Vector3{
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
        let expected = msg::geometry_msgs::TransformStamped {
            child_frame_id: "item".to_string(),
            header: msg::std_msgs::Header {
                frame_id: "camera".to_string(), 
                stamp: rosrust::Time{sec:0, nsec:0},
                seq: 1
            },
            transform: msg::geometry_msgs::Transform{
                rotation: msg::geometry_msgs::Quaternion{
                    x: 0f64, y: 0f64, z: 0f64, w: 1f64
                },
                translation: msg::geometry_msgs::Vector3{
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
        let expected = msg::geometry_msgs::TransformStamped {
            child_frame_id: "item".to_string(),
            header: msg::std_msgs::Header {
                frame_id: "camera".to_string(), 
                stamp: rosrust::Time{sec:0, nsec:700_000_000},
                seq: 1
            },
            transform: msg::geometry_msgs::Transform{
                rotation: msg::geometry_msgs::Quaternion{
                    x: 0f64, y: 0f64, z: 0f64, w: 1f64
                },
                translation: msg::geometry_msgs::Vector3{
                    x: 0.5f64, y: -0.7f64, z: 0f64
                }
            }
        };
        assert_eq!(res.unwrap(), expected);
    }

}

/*
fn main() {
       // Initialize node
       rosrust::init("listener");

       // Create subscriber
       // The subscriber is stopped when the returned object is destroyed
       let _subscriber_tf = rosrust::subscribe("tf", 100, |v: msg::tf2_msgs::TFMessage| {
           
       }).unwrap();

       let _subscriber_tf_static = rosrust::subscribe("tf_static", 100, |v: msg::tf2_msgs::TFMessage| {
           let mut buffer = TfBuffer::new();
           buffer.handle_incoming_transforms(v, true);
           println!("");
           println!("{:?}" , buffer.retrieve_transform_path("front_cam".to_string(), "sonar".to_string()));
       }).unwrap();

       // Block the thread until a shutdown signal is received
       rosrust::spin();
}
*/
/*
   Copyright 2017 Takashi Ogura

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */
extern crate glfw;
extern crate k;
extern crate kiss3d;
extern crate nalgebra as na;

use glfw::{Action, Key, WindowEvent};
use k::prelude::*;
use k::{JacobianIKSolverBuilder, JointType, LinkBuilder};
use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

fn create_joint_with_link_array() -> k::LinkNode<f32> {
    let l0 = LinkBuilder::new()
        .name("shoulder_link1")
        .joint(
            "shoulder_pitch",
            JointType::Rotational {
                axis: Vector3::y_axis(),
            },
            None,
        )
        .finalize();
    let l1 = LinkBuilder::new()
        .name("shoulder_link2")
        .joint(
            "shoulder_roll",
            JointType::Rotational {
                axis: Vector3::x_axis(),
            },
            None,
        )
        .translation(Translation3::new(0.0, 0.1, 0.0))
        .finalize();
    let l2 = LinkBuilder::new()
        .name("shoulder_link3")
        .joint(
            "shoulder_yaw",
            JointType::Rotational {
                axis: Vector3::z_axis(),
            },
            None,
        )
        .translation(Translation3::new(0.0, 0.0, -0.30))
        .finalize();
    let l3 = LinkBuilder::new()
        .name("elbow_link1")
        .joint(
            "elbow_pitch",
            JointType::Rotational {
                axis: Vector3::y_axis(),
            },
            None,
        )
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize();
    let l4 = LinkBuilder::new()
        .name("wrist_link1")
        .joint(
            "wrist_yaw",
            JointType::Rotational {
                axis: Vector3::z_axis(),
            },
            None,
        )
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize();
    let l5 = LinkBuilder::new()
        .name("wrist_link2")
        .joint(
            "wrist_pitch",
            JointType::Rotational {
                axis: Vector3::y_axis(),
            },
            None,
        )
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize();
    let l6 = LinkBuilder::new()
        .name("wrist_link3")
        .joint(
            "wrist_roll",
            JointType::Rotational {
                axis: Vector3::x_axis(),
            },
            None,
        )
        .translation(Translation3::new(0.0, 0.0, -0.10))
        .finalize();
    let n0 = k::Node::new(l0);
    let n1 = k::Node::new(l1);
    let n2 = k::Node::new(l2);
    let n3 = k::Node::new(l3);
    let n4 = k::Node::new(l4);
    let n5 = k::Node::new(l5);
    let n6 = k::Node::new(l6);
    n1.set_parent(&n0);
    n2.set_parent(&n1);
    n3.set_parent(&n2);
    n4.set_parent(&n3);
    n5.set_parent(&n4);
    n6.set_parent(&n5);
    //n6
    n0
}

fn create_ground(window: &mut Window) -> Vec<SceneNode> {
    let mut panels = Vec::new();
    let size = 0.5f32;
    for i in 0..5 {
        for j in 0..5 {
            let mut c0 = window.add_cube(size, size, 0.001);
            if (i + j) % 2 == 0 {
                c0.set_color(0.0, 0.0, 0.8);
            } else {
                c0.set_color(0.2, 0.2, 0.4);
            }
            let x_ind = j as f32 - 2.5;
            let y_ind = i as f32 - 2.5;
            let trans = Isometry3::from_parts(
                Translation3::new(size * x_ind, 0.0, size * y_ind),
                UnitQuaternion::from_euler_angles(0.0, -1.57, -1.57),
            );
            c0.set_local_transformation(trans);
            panels.push(c0);
        }
    }
    panels
}

fn create_cubes(window: &mut Window) -> Vec<SceneNode> {
    let mut c0 = window.add_cube(0.1, 0.1, 0.1);
    c0.set_color(1.0, 0.0, 1.0);
    let mut c1 = window.add_cube(0.1, 0.1, 0.1);
    c1.set_color(1.0, 0.0, 0.0);
    let mut c2 = window.add_cube(0.1, 0.1, 0.1);
    c2.set_color(0.0, 1.0, 0.0);
    let mut c3 = window.add_cube(0.1, 0.1, 0.1);
    c3.set_color(0.0, 0.5, 1.0);
    let mut c4 = window.add_cube(0.1, 0.1, 0.1);
    c4.set_color(1.0, 0.5, 1.0);
    let mut c5 = window.add_cube(0.1, 0.1, 0.1);
    c5.set_color(0.5, 0.0, 1.0);
    let mut c6 = window.add_cube(0.1, 0.1, 0.1);
    c6.set_color(0.0, 0.5, 0.2);
    vec![c0, c1, c2, c3, c4, c5, c6]
}

fn main() {
    let root = create_joint_with_link_array();
    let mut arm = k::LinkTree::from_root("arm", root);

    let mut window = Window::new("k ui");
    window.set_light(Light::StickToCamera);
    let mut cubes = create_cubes(&mut window);
    let angles = vec![0.8, 0.2, 0.0, -1.5, 0.0, -0.3, 0.0];
    arm.set_joint_angles(&angles).unwrap();
    let base_rot = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, 0.0),
        UnitQuaternion::from_euler_angles(0.0, -1.57, -1.57),
    );
    arm.iter().next().unwrap().set_offset(base_rot
        * Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 0.6),
            UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
        ));
    let poses = arm.link_transforms();
    let mut target = poses.last().unwrap().clone();

    let mut c_t = window.add_sphere(0.05);
    c_t.set_color(1.0, 0.2, 0.2);
    let eye = Point3::new(0.5f32, 1.0, 2.0);
    let at = Point3::new(0.0f32, 0.0, 0.0);
    let mut arc_ball = ArcBall::new(eye, at);

    let solver = JacobianIKSolverBuilder::new().finalize();
    let _ = create_ground(&mut window);

    while window.render_with_camera(&mut arc_ball) {
        for mut event in window.events().iter() {
            match event.value {
                WindowEvent::Key(code, _, Action::Release, _) => {
                    match code {
                        Key::Z => {
                            // reset
                            arm.set_joint_angles(&angles).unwrap();
                            let poses = arm.link_transforms();
                            target = poses.last().unwrap().clone();
                        }
                        Key::F => target.translation.vector[2] += 0.1,
                        Key::B => target.translation.vector[2] -= 0.1,
                        Key::R => target.translation.vector[0] -= 0.1,
                        Key::L => target.translation.vector[0] += 0.1,
                        Key::P => target.translation.vector[1] += 0.1,
                        Key::N => target.translation.vector[1] -= 0.1,
                        _ => {}
                    }
                    event.inhibited = true // override the default keyboard handler
                }
                _ => {}
            }
        }
        solver.solve(&mut arm, "wrist_link3", &target).unwrap_or_else(|err| {
            println!("Err: {}", err);
            0.0f32
        });
        c_t.set_local_transformation(target.clone());
        //cubes[0].set_local_transformation(end.transform());
        for (i, trans) in arm.link_transforms().iter().enumerate() {
            cubes[i].set_local_transformation(trans.clone());
        }
    }
}

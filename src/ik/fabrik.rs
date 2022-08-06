use nalgebra::{Isometry3, RealField};
use simba::scalar::SubsetOf;

use crate::{Constraints, Error, InverseKinematicsSolver, SerialChain};

#[derive(Debug)]
pub struct FABRIKIkSolver<T: RealField> {
    pub allowable_target_distance: T,
}

impl<T: RealField> FABRIKIkSolver<T> {
    /// Create instance of `FABRIKIkSolver`.
    ///
    ///  `FABRIKIkSolverBuilder` is available instead of calling this `new` method.
    ///
    /// # Examples
    ///
    /// ```
    /// let solver = k::FABRIKIkSolver::new(0.01);
    /// ```
    pub fn new(allowable_target_distance: T) -> FABRIKIkSolver<T> {
        FABRIKIkSolver {
            allowable_target_distance,
        }
    }

    fn solve_with_constraints_internal(
        &self,
        arm: &SerialChain<T>,
        target_pose: &Isometry3<T>,
        constraints: &Constraints,
    ) -> Result<(), Error> {
        todo!()
    }
}

impl<T: RealField + SubsetOf<f64>> InverseKinematicsSolver<T> for FABRIKIkSolver<T> {
    /// Set joint positions of `arm` to reach the `target_pose`
    ///
    /// # Examples
    ///
    /// ```
    /// use k::prelude::*;
    ///
    /// let chain = k::Chain::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
    /// // Create sub-`Chain` to make it easy to use inverse kinematics
    /// let target_joint_name = "r_wrist_pitch";
    /// let r_wrist = chain.find(target_joint_name).unwrap();
    /// let mut arm = k::SerialChain::from_end(r_wrist);
    /// println!("arm: {arm}");
    ///
    /// // Set joint positions of `arm`
    /// let positions = vec![0.1, 0.2, 0.0, -0.5, 0.0, -0.3];
    /// arm.set_joint_positions(&positions).unwrap();
    /// println!("initial positions={:?}", arm.joint_positions());
    ///
    /// // Get the transform of the end of the manipulator (forward kinematics)
    /// let mut target = arm.update_transforms().last().unwrap().clone();
    ///
    /// println!("initial target pos = {}", target.translation);
    /// println!("move x: -0.1");
    /// target.translation.vector.x -= 0.1;
    ///
    /// // Create IK solver with default settings
    /// let solver = k::JacobianIkSolver::default();
    ///
    /// // solve and move the manipulator positions
    /// solver.solve(&arm, &target).unwrap();
    /// println!("solved positions={:?}", arm.joint_positions());
    /// ```
    fn solve(&self, arm: &SerialChain<T>, target_pose: &Isometry3<T>) -> Result<(), Error> {
        todo!();
    }

    /// Set joint positions of `arm` to reach the `target_pose` with constraints
    ///
    /// If you want to loose the constraints, use this method.
    /// For example, ignoring rotation with an axis.
    /// It enables to use the arms which has less than six DoF.
    ///
    /// # Example
    ///
    /// ```
    /// use k::prelude::*;
    ///
    /// let chain = k::Chain::<f32>::from_urdf_file("urdf/sample.urdf").unwrap();
    /// let target_joint_name = "r_wrist_pitch";
    /// let r_wrist = chain.find(target_joint_name).unwrap();
    /// let mut arm = k::SerialChain::from_end(r_wrist);
    /// let positions = vec![0.1, 0.2, 0.0, -0.5, 0.0, -0.3];
    /// arm.set_joint_positions(&positions).unwrap();
    /// let mut target = arm.update_transforms().last().unwrap().clone();
    /// target.translation.vector.x -= 0.1;
    /// let solver = k::JacobianIkSolver::default();
    ///
    /// let mut constraints = k::Constraints::default();
    /// constraints.rotation_x = false;
    /// constraints.rotation_z = false;
    /// solver
    ///    .solve_with_constraints(&arm, &target, &constraints)
    ///    .unwrap_or_else(|err| {
    ///        println!("Err: {err}");
    ///    });
    /// ```
    fn solve_with_constraints(
        &self,
        arm: &SerialChain<T>,
        target_pose: &Isometry3<T>,
        constraints: &Constraints,
    ) -> Result<(), Error> {
        let orig_positions = arm.joint_positions();
        let re = self.solve_with_constraints_internal(arm, target_pose, constraints);
        if re.is_err() {
            arm.set_joint_positions(&orig_positions)?;
        };
        re
    }
}

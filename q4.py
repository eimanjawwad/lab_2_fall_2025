
import numpy as np
def forward_kinematics(theta1, theta2, theta3, offset):

  def rotation_x(angle):
      # rotation about the x-axis implemented for you
      return np.array(
          [
              [1, 0, 0, 0],
              [0, np.cos(angle), -np.sin(angle), 0],
              [0, np.sin(angle), np.cos(angle), 0],
              [0, 0, 0, 1],
          ]
      )

  def rotation_y(angle):
      ## TODO: Implement the rotation matrix about the y-axis
      return np.array(
          [
              [np.cos(angle), 0, np.sin(angle), 0],
              [0, 1, 0, 0],
              [-np.sin(angle), 0, np.cos(angle), 0],
              [0, 0, 0, 1],
          ]
      )
      raise NotImplementedError()

  def rotation_z(angle):
      ## TODO: Implement the rotation matrix about the z-axis
      return np.array(
          [
              [np.cos(angle), -np.sin(angle), 0, 0],
              [np.sin(angle), np.cos(angle), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1],
          ]
      )
      raise NotImplementedError()

  def translation(x, y, z):
      ## TODO: Implement the translation matrix
      return np.array(
          [
              [1, 0, 0, x],
              [0, 1, 0, y],
              [0, 0, 1, z],
              [0, 0, 0, 1],
          ]
      )
      raise NotImplementedError()

  # T_0_1 (base_link to leg_front_l_1)
  T_0_1 = translation(0.07500, 0.0445, 0) @ rotation_x(1.57080) @ rotation_z(-theta1)

  # T_1_2 (leg_front_l_1 to leg_front_l_2)
  ## TODO: Implement the transformation matrix from leg_front_l_1 to leg_front_l_2
  T_1_2 = translation(0, 0, -0.039) @ rotation_y(-1.57080) @ rotation_z(theta2)

  # T_2_3 (leg_front_l_2 to leg_front_l_3)
  ## TODO: Implement the transformation matrix from leg_front_l_2 to leg_front_l_3
  T_2_3 = translation(0, offset + -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(-theta3)

  # T_3_ee (leg_front_l_3 to end-effector)
  T_3_ee = translation(0.06231, -0.06216, -0.018) 

  # TODO: Compute the final transformation. T_0_ee is the multiplication of the previous transformation matrices
  T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

  # TODO: Extract the end-effector position. The end effector position is a 3x1 vector (not in homogenous coordinates)
  end_effector_position = T_0_ee[:3,3]

  return end_effector_position

for offset in (0.002, 0.004, 0.008): # cm
  print("Offset = ", offset)
  for a in (0, 1.57080):
    print("Angle = ", a)
    print("Without Offset: ", forward_kinematics(a, a, a, offset), "With Offset: ", forward_kinematics(a, a, a, offset))

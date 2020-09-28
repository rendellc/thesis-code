
model Wheel "turnable wheel"
  // Imports
  import Modelica.Math.atan2;

  // Inputs
  input Real drive_torque;
  input Real steer_torque;
  input Real vel_cg[3] "Velocity of wheel relative to ground in cg frame";
  input Real force_z "Weight carried by this wheel";

  // Dynamical variables
  Real omega(start=0.0) "Angular velocity";
  Real steer_angle(start=0.0) "Steering angle (difference between X_w and X_cg)";
  Real steer_angle_dot(start=0.0) "Derivative of steer_angle";
  Real force_cg[3];
  Real sideslip;
  Real slip_l;
  Real slip_s;
  Real slip_res;
  Real mu_l;
  Real mu_s;
  Real mu_res;
  Real vel_rot "rotational equivalent wheel velocity";
  Real vel_w "ground contact point velocity";
  Real friction_w[3];
  Real friction_b[3];
  Real rot_wheel_to_body[3,3] "rotation matrix from wheel to body frame";

  parameter Real mass = 20.0;
  parameter Real radius = 0.8;
  parameter Real width = 0.4;
  parameter Real position[3];
  parameter Real c1=1.28;
  parameter Real c2=23.99;
  parameter Real c3=0.52;
  parameter Real ks=1.0;
  parameter Real eps=0.00001;

  // Derived parameters
  Real inertia_spin "Inertia in direction of spin";
  Real inertia_steer "Inertia in steering direction";

equation
  inertia_spin = 1/2 * mass * radius^2;
  inertia_steer = 1/4 * mass * radius^2 + 1/12 * mass * width^2;

  // Kinematics
  der(steer_angle) = steer_angle_dot;

  // Dynamics
  inertia_steer * der(steer_angle_dot) = steer_torque;
  inertia_spin * der(omega) = drive_torque - radius*friction_w[1];

  // Friction modelling
  friction_b = rot_wheel_to_body * friction_w;
  friction_w[3] = 0.0; // ignore z-component

  sideslip = steer_angle - atan2(vel_cg[2], vel_cg[1]);
  force_cg = friction_b;
  
  // Rotational velocity
  rot_wheel_to_body = [
    cos(steer_angle), -sin(steer_angle), 0;
    sin(steer_angle), cos(steer_angle), 0;
    0, 0, 1
  ];
  vel_rot = radius*omega;
  vel_w = sqrt(vel_cg*vel_cg); // use norm
  slip_l = (vel_rot*cos(steer_angle) - vel_w)/(max(vel_rot*cos(steer_angle), vel_w) + eps);
  slip_s = vel_rot*sin(steer_angle)/(max(vel_rot*cos(steer_angle), vel_w) + eps);
  slip_res = sqrt(slip_l^2 + slip_s^2);

  mu_res = c1*(1 - exp(-c2*slip_res)) - c3*slip_res;
  mu_l = slip_l/(slip_res + eps) * mu_res;
  mu_s = ks * slip_s/(slip_res + eps) * mu_res;

  friction_w[1] = mu_l*force_z;
  friction_w[2] = mu_s*force_z;
end Wheel;

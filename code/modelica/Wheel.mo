model Wheel "turnable wheel"
  // Imports
  import Modelica.Math.atan2;
  import Modelica.Math.Vectors.norm;

  // Inputs
  input Real drive_torque;
  input Real steer_torque;
  input Real vel_w_body[3] "Velocity of wheel relative to ground in body frame";
  input Real force_z "Weight carried by this wheel";

  // Dynamical variables
  Real omega(start=0.0) "Angular velocity";
  Real steer_angle(start=0.0) "Steering angle (difference between X_w and X_cg)";
  Real steer_angle_dot(start=0.0) "Derivative of steer_angle";
  Real alpha "Wheel sideslip: angle between wheel x-direction and wheel velocity x-direction";
  Real beta "Angle between wheel velocity x-direction and body-frame x-axis";
  Real slip_l;
  Real slip_s;
  Real slip_l_ratio;
  Real slip_s_ratio;
  Real slip_res;
  Real mu_l;
  Real mu_s;
  Real mu_res;
  Real vel_rot "rotational equivalent wheel velocity";
  Real vel_rot_l "rotational equivalent wheel velocity in longitudinal travel direction";
  Real vel_rot_s "rotational equivalent wheel velocity in lateral travel direction";
  Real vel_diff_l;
  Real vel_diff_s;
  Real vel_w "ground contact point speed";
  Integer driveState;
  Real vel_big;
  Real vel_big_sign "-1 or 1 depending on sign of vel_big";
  Real force_w_ls[3];
  Real force_w_wheel[3];
  Real force_body[3] "force acting on body in body frame";
  Real force_body_ls[3] "force acting on body in LS frame";
  Real rot_ls_to_body[3,3] "rotation matrix from wheel to body frame";
  Real rot_wheel_to_body[3,3] "rotation matrix from wheel to body frame";
  Real rot_body_to_wheel[3,3] "rotation matrix from body to wheel frame";

  // Parameters
  parameter Real mass = 189.0;
  parameter Real radius = 0.8;
  parameter Real width = 0.4;
  parameter Real position[3];
  parameter Real c1=1.28;
  parameter Real c2=23.99;
  parameter Real c3=0.52;
  parameter Real ks=1.0;
  parameter Real eps=0.0001;
  parameter Real B=0.05 "Slip response parameter";

  // Derived parameters
  Real inertia_spin "Inertia in direction of spin";
  Real inertia_steer "Inertia in steering direction";

initial equation
  der(slip_l) = 0;
  der(slip_s) = 0;

equation
  inertia_spin = 1/2 * mass * radius^2;
  inertia_steer = 1/4 * mass * radius^2 + 1/12 * mass * width^2;

  // Kinematics
  der(steer_angle) = steer_angle_dot;

  // Dynamics
  inertia_steer * der(steer_angle_dot) = steer_torque;
  // inertia_spin * der(omega) = drive_torque - radius*force_w_wheel[1];
  inertia_spin * der(omega) + 100*omega = drive_torque; 

  // Friction modelling
  force_body_ls = -force_w_ls;
  force_body = rot_ls_to_body * force_body_ls;
  force_w_ls[1] = mu_l*force_z;
  force_w_ls[2] = mu_s*force_z;
  force_w_ls[3] = 0.0; // ignore z-component
  force_w_wheel = rot_body_to_wheel * (rot_ls_to_body * force_w_ls);

  // Rotations
  rot_ls_to_body = [
    cos(beta), -sin(beta), 0;
    sin(beta), cos(beta), 0;
    0, 0, 1
  ];
  rot_wheel_to_body = [
    cos(steer_angle), -sin(steer_angle), 0;
    sin(steer_angle), cos(steer_angle), 0;
    0, 0, 1
  ];
  rot_body_to_wheel = transpose(rot_wheel_to_body);
  beta = atan2(vel_w_body[2], vel_w_body[1]);
  alpha + beta = steer_angle;
  
  // Velocities
  vel_rot = radius*omega;
  vel_w = norm(vel_w_body);
  vel_rot_l = vel_rot*cos(alpha);
  vel_rot_s = vel_rot*sin(alpha);
  vel_diff_l = vel_rot_l - vel_w;
  vel_diff_s = vel_rot_s;

  driveState = sign(vel_diff_l);
  vel_big = noEvent(if vel_rot_l > vel_w then vel_rot_l else vel_w);
  vel_big_sign = noEvent(if sign(vel_big) == -1 then -1 else 1);
  
  // Friction coefficient computation
  B*der(slip_l) + abs(vel_big)*slip_l = vel_big_sign*vel_diff_l;
  B*der(slip_s) + abs(vel_big)*slip_s = vel_big_sign*vel_diff_s;
  assert(abs(slip_l) <= 1.1, "Longitudinal slip out of range: " + String(slip_l), AssertionLevel.warning);
  assert(abs(slip_s) <= 1.1, "Side slip out of range: " + String(slip_s), AssertionLevel.warning);


  slip_res = sqrt(slip_l^2 + slip_s^2);
  slip_l_ratio = slip_l/(slip_res+eps);
  slip_s_ratio + slip_l_ratio = 1;

  mu_res = c1*(1 - exp(-c2*slip_res)) - c3*slip_res;
  mu_l = slip_l_ratio * mu_res;
  mu_s = ks * slip_s_ratio * mu_res;
end Wheel;

model Body "Body kinematics and simple dynamics"

  input Real force_b[3] "Force applied to body in body frame";
  input Real torque[3] "Torque applied to body";

  Real pos_in[3] "Position in inertial frame";
  Real vel_in[3] "Velocity in inertial frame";
  Real vel_b[3] "Velocity in body frame";
  
  Real yaw(start=0.0);
  Real yawrate(start=0.0);

  parameter Real width = 2.0 "Width of vehicle";
  parameter Real length = 4.0 "Length of vehicle";
  parameter Real mass = 2500.0 "Mass of body";

  Real inertia_z;
  Real rot_body_to_in[3,3] "rotation matrix from body to inertial frame";
  Real rot_in_to_body[3,3] "rotation matrix from inertial to body frame";

equation
  inertia_z = 1/12 * mass * (length*length + width*width);

  // equation 2.14, 2.15, 2.16 in Fossen validates this equation
  rot_body_to_in = [
    cos(yaw), -sin(yaw), 0;
    sin(yaw), cos(yaw), 0;
    0, 0, 1
  ];
  rot_in_to_body = transpose(rot_body_to_in);
  der(pos_in) = vel_in;
  vel_in = rot_body_to_in * vel_b;  

  // assume no roll and pitch
  der(yaw) = yawrate;

  mass*der(vel_b) = force_b;
  inertia_z*der(yawrate) = torque[3];
end Body;

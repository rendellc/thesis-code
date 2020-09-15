
model Vehicle "model of vehicle with four wheels"
  input Real forces[4] "force inputs. Order: fl, rl, rr, fr";

  output Real pos_in[3] "position in inertial frame";
  output Real vel_b[3] "velocity in body frame";
  output Real yaw "yaw";
  output Real yawrate "Yawrate";

  Real force_vectors[4,3];
  Real torque_vectors[4,3];
  Real torques[4];

  parameter Real width = 2.0 "Width of vehicle";
  parameter Real length = 4.0 "Length of vehicle";
  parameter Real height = 2.0 "Height of vehicle";
  parameter Real mass = 100.0 "Mass of vehicle";
  Real inertia;
  Real wheel_positions[4,3];

  Real rot_z[3,3] "rotation matrix from body to inertial frame";
  Real force_total[3];
  Real torque_total;
initial equation
  pos_in = fill(0, 3);
  vel_b = fill(0, 3);
  yaw = 0;
  yawrate = 0;

equation
  inertia = 1/12 * mass * (length*length + width*width);
  wheel_positions = [
    length/2, width/2, 0;
    -length/2, width/2, 0;
    -length/2, -width/2, 0;
    length/2, -width/2, 0
  ];


  force_vectors = [
    forces[1], 0, 0;
    forces[2], 0, 0;
    forces[3], 0, 0;
    forces[4], 0, 0
  ];

  for i in 1:4 loop
    torque_vectors[i] = cross(wheel_positions[i], force_vectors[i]);
    torques[i] = torque_vectors[i,3];
  end for;

  torque_total = sum(torques);
  force_total = force_vectors[1] + force_vectors[2] + force_vectors[3] + force_vectors[4];


  // equation 2.14, 2.15, 2.16 in Fossen validates this equation
  rot_z = [
    cos(yaw), -sin(yaw), 0;
    sin(yaw), cos(yaw), 0;
    0, 0, 1
  ];
  der(pos_in) = rot_z * vel_b;  

  // assume no roll and pitch
  der(yaw) = yawrate;

  mass*der(vel_b) = force_total;
  inertia*der(yawrate) = torque_total;

end Vehicle;



model TwoTrackVehicle "Model of a two track tank-like vehicle"
  input Real force_right "force of right track";
  input Real force_left "force of right track";

  output Real x(start=0.0) "x position";
  output Real y(start=0.0) "y position";
  output Real yaw(start=0.0) "yaw";

  Real vx(start=0.0) "x velocity";
  Real vy(start=0.0) "y velocity";
  Real yawrate(start=0.0) "Yawrate";

  Real vx_cg(start=0.0) "x velocity in CG";

  Real torque_left;
  Real torque_right;
  Real torque_total;

  parameter Real width = 2.0 "Width of vehicle";
  parameter Real length = 4.0 "Length of vehicle";
  parameter Real height = 2.0 "Height of vehicle";
  parameter Real mass = 100.0 "Mass of vehicle";
  Real inertia;
initial equation
  x = 0;
  y = 0;
  vx = 0;
  vx_cg = 0;
  yawrate = 0;

equation
  inertia = 1/12 * mass * (length*length + width*width);

  der(x) = vx;
  der(y) = vy;
  der(yaw) = yawrate;

  vx = cos(yaw)*vx_cg;
  vy = sin(yaw)*vx_cg;

  torque_left = -width/2 * force_left;
  torque_right = width/2 * force_right;
  torque_total = torque_left + torque_right;

  mass*der(vx_cg) = force_left + force_right;
  inertia*der(yawrate) = torque_total;

end TwoTrackVehicle;

model Wheel "wheel model"
  input Real drive_torque;
  output Real omega;
  parameter Real inertia = 1.0;
  parameter Real x,y;
equation
  inertia*der(omega) = drive_torque;
end Wheel;


model SplitTest "model of vehicle"
  Wheel w1(x=1.0, y=1.0);
  Wheel w2(x=-1.0, y=1.0);
  Wheel w3(x=-1.0, y=-1.0);
  Wheel w4(x=1.0, y=-1.0);

  input Real drive_torques[4];
  output Real omegas[4];
equation
  w1.drive_torque = drive_torques[1];
  w2.drive_torque = drive_torques[2];
  w3.drive_torque = drive_torques[3];
  w4.drive_torque = drive_torques[4];
  omegas[1] = w1.omega;
  omegas[2] = w2.omega;
  omegas[3] = w3.omega;
  omegas[4] = w4.omega;
end SplitTest;



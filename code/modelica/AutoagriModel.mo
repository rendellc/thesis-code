model AutoagriModel "model of vehicle with four turnable wheels"
  input Real drive_torques[4];
  input Real steer_torques[4];


  Body body;
  Wheel w1(position={body.length/2, body.width/2, 0.0});
  Wheel w2(position={-body.length/2, body.width/2, 0.0});
  Wheel w3(position={-body.length/2, -body.width/2, 0.0});
  Wheel w4(position={body.length/2, -body.width/2, 0.0});

  constant Real g = 9.81;

initial equation
  

equation
  // array operations possible?
  w1.vel_cg = body.rot_in_to_body*(body.vel_in + der(body.rot_body_to_in)*w1.position);
  w2.vel_cg = body.rot_in_to_body*(body.vel_in + der(body.rot_body_to_in)*w2.position);
  w3.vel_cg = body.rot_in_to_body*(body.vel_in + der(body.rot_body_to_in)*w3.position);
  w4.vel_cg = body.rot_in_to_body*(body.vel_in + der(body.rot_body_to_in)*w4.position);

  w1.drive_torque = drive_torques[1];
  w2.drive_torque = drive_torques[2];
  w3.drive_torque = drive_torques[3];
  w4.drive_torque = drive_torques[4];

  w1.steer_torque = steer_torques[1];
  w2.steer_torque = steer_torques[2];
  w3.steer_torque = steer_torques[3];
  w4.steer_torque = steer_torques[4];

  w1.force_z = g*(body.mass/4 + w1.mass);
  w2.force_z = g*(body.mass/4 + w2.mass);
  w3.force_z = g*(body.mass/4 + w3.mass);
  w4.force_z = g*(body.mass/4 + w4.mass);

  body.force_b = w1.force_cg + w2.force_cg + w3.force_cg + w4.force_cg;
  body.torque = cross(w1.position,w1.force_cg) + cross(w2.position,w2.force_cg) + cross(w3.position,w3.force_cg) + cross(w4.position,w4.force_cg);

end AutoagriModel;

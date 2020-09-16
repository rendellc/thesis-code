model Linear2DMovement "Simple linear 2d movement model for testing"
  // Control inputs
  input Real u[2] "Forces";

  // States
  output Real x[2] "Position";
  output Real vx[2] "Velocity";

  // Parameters
  parameter Real m = 1.0 "Mass";
  parameter Real mu = 0.1 "Dynamic friction coefficient";

initial equation
  x = fill(0.0, 2);
  vx = fill(0.0, 2);

equation
  vx = der(x);
  m*der(vx) + mu*vx = u;

end Linear2DMovement;

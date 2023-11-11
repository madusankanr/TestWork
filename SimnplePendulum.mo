within ;
model SimnplePendulum
  constant Modelica.Units.SI.Acceleration g=9.81;
  parameter Real L(min = 0,unit="m") = 1;
  Real Theta(start=0.1, fixed = true);
  Real ThetaDot;

equation
  ThetaDot = der(Theta);
  der(ThetaDot) = -g/L * sin(Theta);

end SimnplePendulum;

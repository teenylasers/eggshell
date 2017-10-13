
print 'Test automatic differentiation in lua:'

config = {
  type = 'Ez',
  unit = 'm',
  mesh_edge_length = 1e9,
  mesh_refines = 0,
  excited_port = 1,
  frequency = 60e9,
  depth = 0,
  cd = Rectangle(-0.5, -0.5, 1.5, 1.5)
}

function CheckJet(x, expected_scalar, expected_derivative)
  print(string.format('Jet: %.10g, d/dp=%.10g', x, _JetDerivative(x,1)))
  assert(math.abs(x - expected_scalar) < 1e-9)
  assert(math.abs(_JetDerivative(x,1) - expected_derivative) < 1e-9)
end

local x, y, z

-- Test the basics.
x = _Jet(1.23, 1)
y = math.cos(x)
CheckJet(x, 1.23, 1)
CheckJet(y, math.cos(1.23), -math.sin(1.23))

-- Make sure derivatives pass through tonumber().
x = tonumber(_Jet(1.23, 1))
y = tonumber(math.cos(x))
CheckJet(x, 1.23, 1)
CheckJet(y, math.cos(1.23), -math.sin(1.23))

-- Make sure derivatives pass through tables.
T = {x=_Jet(1.23, 1), y=math.cos(x)}
x = T.x;
y = T.y;
CheckJet(x, 1.23, 1)
CheckJet(y, math.cos(1.23), -math.sin(1.23))

-- Test linear combinations.
x = _Jet(1, 1)
y = _Jet(2, 1) * 2
z = _Jet(3, 1) * 3
CheckJet(x, 1, 1)
CheckJet(y, 4, 2)
CheckJet(z, 9, 3)
CheckJet(x + y*2 + z*3, 36, 14)

-- Test a more complicated function: x^y.
x = _Jet(2, 1) * 1.5
y = _Jet(3, 1)
CheckJet(x^y, 27, y * x^(y-1) * 1.5 + x^y * math.log(x))

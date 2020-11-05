
-- Investigate the behavior of closely spaced wires used as a polarizer.

orientation = Parameter{label='Orientation', min=1, max=2, default=2, integer=true}

config = {
  type = 'Exy',
  unit = 'mm',
  frequency = Parameter{label='Frequency (Hz)', min=60e9, max=90e9, default=70e9},
  mesh_edge_length = 0.2,
  excited_port = 1,
  depth = Infinity,
}

if orientation == 1 then
  config.type = 'Exy'
  DrawText(0,-0.2, 'Y polarization (vertical)', 'left', 'top')
else
  config.type = 'Ez'
  DrawText(0,-0.2, 'Z polarization (perpendicular to screen)', 'left', 'top')
end

-- Geometric parameters.
W = 10
H = 5
spacing = Parameter{label='Spacing', min=0.1, max=5, default=0.86}
radius = Parameter{label='Radius', min=0.02, max=0.5, default=0.1}

-- Create the geometry.
cd = Rectangle(0, 0, W, H)
for i = 1,math.ceil(H/spacing) do
  cd = cd - Circle(W/2, i*spacing, radius, 16)
end

-- Add the ports.
cd:Port(cd:Select(0, H/2), 1)
cd:Port(cd:Select(W, H/2), 2)

config.cd = cd

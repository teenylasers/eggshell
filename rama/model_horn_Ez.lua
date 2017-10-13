
config = {
  type = 'Ez',
  unit = 'mil',
  frequency = Parameter{label='Frequency (Hz)', min=60e9, max=90e9, default=70e9},
  mesh_edge_length = Parameter{label='Mesh edge length', min=2, max=30, default=8},
  mesh_refines = 0,
  excited_port = 1,
}
ParameterDivider()

-- Basic parameters.
F = 100                 -- Feed length
B = 122                 -- Waveguide B dimension
L = Parameter{label='Horn length', min=50, max=800, default=200}
W = Parameter{label='Horn width',  min=50, max=800, default=200}
R = Parameter{label='ABC radius',  min=50, max=800, default=300}

local eps = 1e-6        -- @@@ want to remove the need for this

-- Radiation boundary.
local radius = R + W/2
rad = Circle(0, 0, radius, 64)
rad:ABC(rad:SelectAll())
rad = rad * Rectangle(-20, -2*radius, 2*radius, 2*radius)
rad = rad - Rectangle(-50, -0.9*radius, 0, 0.9*radius)
rad:Offset(F+L, 0)

cd = Rectangle(0, -B/2, F, B/2) +
     Shape():AddPoint(F, B/2):AddPoint(F, -B/2)
            :AddPoint(F+L+eps, -W/2):AddPoint(F+L+eps, W/2) +
     rad

cd:Port(cd:Select(0, 0), 1)
config.cd = cd

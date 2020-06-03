
config = {
  type = 'Ez',
  unit = 'mil',
  frequency = Parameter{label='Frequency (Hz)', min=60e9, max=90e9, default=70e9},
  mesh_edge_length = Parameter{label='Mesh edge length', min=4, max=50, default=30},
  mesh_refines = 0,
  excited_port = 1,
  depth = 10000         -- Close to free space
}
ParameterDivider()

-- Basic parameters.
F = 100                 -- Feed length
A = 1000                -- Distance to radiation boundary
B = Parameter{label='Feed width', min=50, max=1000, default=1000}
L = Parameter{label='Horn length', min=50, max=1000, default=1000}
W = Parameter{label='Horn width',  min=50, max=1000, default=1000}
R = Parameter{label='Rotation',  min=-180, max=180, default=0}

horn = Rectangle(0, -B/2, F, B/2) +
       Shape():AddPoint(F, B/2):AddPoint(F, -B/2)
              :AddPoint(F+L, -W/2):AddPoint(F+L, W/2)
horn = horn:Clone():Grow(20,'miter',2) - horn
horn = horn - Rectangle(F+L, -W/2, F+L+100, W/2)
horn:Port(horn:Select(0, 0), 1)
horn:Offset(-(F+L)/2,0)

rad = Rectangle(-A, -A, A, A);
rad:ABC(rad:SelectAll())

config.cd = rad - horn:Rotate(R)

-- Adjust boresight to compensate for the horn rotation.
config.boresight = R


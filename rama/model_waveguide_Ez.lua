
config = {
  type = 'Ez',
  unit = 'mil',
  mesh_edge_length = Parameter{label='Mesh edge length', min=2, max=30, default=5},
  mesh_refines = 0,
  excited_port = 1,
  frequency = Parameter{label='Frequency (Hz)', min=30e9, max=90e9, default=70e9},
}

L = 500         -- Total length
H1 = Parameter{label='Height 1',  min=50, max=300, default=122}
H2 = Parameter{label='Height 2',  min=50, max=300, default=122}
T = Parameter{label='Transition Len',  min=0, max=450, default=50}

cd = Shape():AddPoint(-L/2,  H1/2)
            :AddPoint(-L/2, -H1/2)
            :AddPoint(-T/2, -H1/2)
            :AddPoint( T/2, -H2/2)
            :AddPoint( L/2, -H2/2)
            :AddPoint( L/2,  H2/2)
            :AddPoint( T/2,  H2/2)
            :AddPoint(-T/2,  H1/2)

cd:Port(cd:Select(-L/2, 0), 1)
cd:Port(cd:Select(L/2, 0), 2)

config.cd = cd

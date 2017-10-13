
-- A demonstration of tunneling through a potential barrier.

config = {
  type = 'S',
  unit = 'm',
  mesh_edge_length = Parameter{label='Mesh edge length', min=0.2, max=3, default=0.5},
  mesh_refines = 0,
  excited_port = 1,
  frequency = Parameter{label='Momentum', min=0.1, max=1, default=0.1},
}

L = 50         -- Total length
H1 = Parameter{label='Height 1',  min=5, max=30, default=12}
H2 = Parameter{label='Height 2',  min=5, max=30, default=12}
T = Parameter{label='Transition Len',  min=0, max=45, default=5}

cd = Shape():AddPoint(-L/2,  H1/2)
            :AddPoint(-L/2, -H1/2)
            :AddPoint(-T/2, -H1/2)
            :AddPoint( T/2, -H2/2)
            :AddPoint( L/2, -H2/2)
            :AddPoint( L/2,  H2/2)
            :AddPoint( T/2,  H2/2)
            :AddPoint(-T/2,  H1/2)

cd:Paint(Rectangle(-T/2,-1e3,T/2,1e3), 0xff8000, 2)

cd:Port(cd:Select(-L/2, 0), 1)
cd:Port(cd:Select(L/2, 0), 2)

config.cd = cd

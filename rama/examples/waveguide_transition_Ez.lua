
-- Test the transition from an Ez waveguide of one width to another width. If
-- either width is less than a critical value, TE10 waves can not propagate
-- through the port. If the transition is too abrupt, waves will be reflected
-- from the transition.

config = {
  type = 'Ez',
  unit = 'mil',
  mesh_edge_length = Parameter{label='Mesh edge length', min=2, max=30, default=5},
  excited_port = 1,
  frequency = Parameter{label='Frequency (Hz)', min=30e9, max=90e9, default=70e9},
}

L = 500         -- Total length
W1 = Parameter{label='Width 1',  min=50, max=300, default=122}
W2 = Parameter{label='Width 2',  min=50, max=300, default=122}
T = Parameter{label='Transition Len',  min=0, max=450, default=50}

cd = Shape():AddPoint(-L/2,  W1/2)
            :AddPoint(-L/2, -W1/2)
            :AddPoint(-T/2, -W1/2)
            :AddPoint( T/2, -W2/2)
            :AddPoint( L/2, -W2/2)
            :AddPoint( L/2,  W2/2)
            :AddPoint( T/2,  W2/2)
            :AddPoint(-T/2,  W1/2)

cd:Port(cd:Select(-L/2, 0), 1)
cd:Port(cd:Select(L/2, 0), 2)

config.cd = cd

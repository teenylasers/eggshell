
-- A demonstration of a particle tunneling through a potential barrier.
-- Increasing the momentum will increase the probability that the particle
-- tunnels. Look at the field with the 'max amplitude' mode, to see the
-- probability density, to understand when tunneling has occured.

config = {
  type = 'S',
  unit = 'm',
  mesh_edge_length = Parameter{label='Mesh edge length', min=0.2, max=3, default=0.5},
  excited_port = 1,
  frequency = Parameter{label='Momentum', min=0.1, max=1, default=0.1},
  -- Uncomment these lines to look at a wideband pulse:
  --   frequency = LinearRange(0.1,1,40),
  --   wideband_window = 'hamming',
}

L = 50         -- Total length
W1 = Parameter{label='Width 1',  min=5, max=30, default=12}
W2 = Parameter{label='Width 2',  min=5, max=30, default=12}
T = Parameter{label='Transition Len',  min=0, max=45, default=5}

cd = Shape():AddPoint(-L/2,  W1/2)
            :AddPoint(-L/2, -W1/2)
            :AddPoint(-T/2, -W1/2)
            :AddPoint( T/2, -W2/2)
            :AddPoint( L/2, -W2/2)
            :AddPoint( L/2,  W2/2)
            :AddPoint( T/2,  W2/2)
            :AddPoint(-T/2,  W1/2)

cd:Paint(Rectangle(-T/2,-1e3,T/2,1e3), 0xff8000, 3)

cd:Port(cd:Select(-L/2, 0), 1)
cd:Port(cd:Select(L/2, 0), 2)

config.cd = cd

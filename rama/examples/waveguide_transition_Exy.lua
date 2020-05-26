
-- Test the transition from an Exy waveguide of one width to another width. If
-- the depth is less than a critical value, TE10 waves can not propagate
-- through the ports. If the transition is too abrupt, waves will be reflected
-- from the transition. The wavelength in the guide changes as the depth
-- changes.

-- Set this to false to simultaneously simulate a range of frequencies. The
-- solutions can be viewed together as a wideband pulse.
single_frequency = true

if single_frequency then
  freq = Parameter{label='Frequency (Hz)', min=30e9, max=90e9, default=70e9}
else
  freq = LinearRange(50e9,100e9,40)
end

config = {
  type = 'Exy',
  unit = 'mil',
  mesh_edge_length = Parameter{label='Mesh edge length', min=2, max=30, default=5},
  excited_port = 1,
  frequency = freq,
  depth = Parameter{label='Depth', min=20, max=300, default=122},
  wideband_window = 'hamming',
}

L = 1000         -- Total length
W1 = Parameter{label='Width 1',  min=20, max=200, default=61}
W2 = Parameter{label='Width 2',  min=20, max=200, default=61}
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

-- A simulation of the WR-10 coupler reported in "Designs of Wideband 3dB
-- Branch-line Couplers for ALMA Bands 3 to 10", Hiroya Andoh et al.
-- (http://legacy.nrao.edu/alma/memos/html-memos/alma468/memo468.pdf)

config = {
  type = 'Exy',
  unit = 'mm',
  frequency = Parameter{label='Frequency (Hz)', min=75e9, max=115e9, default=93e9},
  mesh_edge_length = 0.2,
  excited_port = 1,
  depth = 2.54,
  optimize = function(port_power, port_phase, field)
    return port_power[1],                 -- Minimal return loss
           port_power[2],                 -- Minimal output to port 2
           port_power[3] - port_power[4]  -- Balance ports 3 and 4
  end,

  -- An alternative setting for the frequency and the optimization function,
  -- for multiple frequencies:
  --
  --   frequency = LinearRange(85e9, 100e9, 50),
  --   optimize = function(port_power, port_phase, field)
  --     -- Hint to the solver that we're going to be Select()ing all solutions.
  --     field.SolveAll()
  --     -- Compute values to optimize over all frequencies.
  --     local T = {}
  --     for i = 1,#config.frequency do
  --       field.Select(i)
  --       port_power, port_phase = field.Ports()
  --       table.insert(T, port_power[1])  -- Minimal return loss
  --       table.insert(T, port_power[2])  -- Minimal output to port 2
  --       table.insert(T, port_power[3] - port_power[4])  -- Balance ports 3 and 4
  --     end
  --     -- Return all values to the optimizer.
  --     return table.unpack(T)
  --   end,
}

-- Basic parameters.
B = 1.27        -- Waveguide short dimension
S = 0.605       -- Spacing to other arm of coupler
L = 2           -- Feed length

-- Parameters for adjustable post and gap sizes.
G1 = Parameter{label='Gap 1',  min=0.05, max=1, default=0.332}
P1 = Parameter{label='Post 1', min=0.05, max=1, default=0.542}
G2 = Parameter{label='Gap 2',  min=0.05, max=1, default=0.544}
P2 = Parameter{label='Post 2', min=0.05, max=1, default=0.424}
G3 = Parameter{label='Gap 3',  min=0.05, max=1, default=0.598}

-- Create feed-through channels.
width = 2*L + 2*(G1+P1+G2+P2)+G3
cd = Rectangle(0, 0, width, B) + Rectangle(0, B+S, width, 2*B+S)

-- Create the air gaps between the channels.
x = L
cd = cd + Rectangle(x, 0, x + G1, 2*B+S)  x = x + G1 + P1
cd = cd + Rectangle(x, 0, x + G2, 2*B+S)  x = x + G2 + P2
cd = cd + Rectangle(x, 0, x + G3, 2*B+S)  x = x + G3 + P2
cd = cd + Rectangle(x, 0, x + G2, 2*B+S)  x = x + G2 + P1
cd = cd + Rectangle(x, 0, x + G1, 2*B+S)

-- Add the ports.
cd:Port(cd:Select(0, B/2), 1)
cd:Port(cd:Select(0, S+B*1.5), 2)
cd:Port(cd:Select(width, B/2), 3)
cd:Port(cd:Select(width, S+B*1.5), 4)

config.cd = cd

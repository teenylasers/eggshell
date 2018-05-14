-- A simulation of the WR-10 coupler reported in "Designs of Wideband 3dB
-- Branch-line Couplers for ALMA Bands 3 to 10", Hiroya Andoh et al.
-- (http://legacy.nrao.edu/alma/memos/html-memos/alma468/memo468.pdf)

-- Set this to false to simultaneously simulate (and optimize for) a range of
-- frequencies.
single_frequency = true

if single_frequency then
  freq = Parameter{label='Frequency (Hz)', min=75e9, max=115e9, default=93e9}
else
  freq = LinearRange(85e9, 100e9, 50)
end

default_parameters = {
  Post1 = 0.332,
  Gap1  = 0.542,
  Post2 = 0.544,
  Gap2  = 0.424,
  Post3 = 0.598,
}

config = {
  type = 'Exy',
  unit = 'mm',
  frequency = freq,
  mesh_edge_length = Parameter{label='Mesh edge length', min=0.1, max=0.5, default=0.2},
  mesh_refines = 0,
  excited_port = Parameter{label='Excited port', min=1, max=4, default=1, integer=true},
  depth = 2.54,
  optimize = function(port_power, port_phase, field)
    if single_frequency then
      return port_power[1],                 -- Minimal return loss
             port_power[2],                 -- Minimal output to port 2
             port_power[3] - port_power[4]  -- Balance ports 3 and 4
    else
      -- Hint to the solver that we're going to be Select()ing all solutions.
      field.SolveAll()
      -- Minimize the worst return loss across the band while optimizing the
      -- output balance for the center frequency.
      local worst_return_loss = 0
      for i = 1,#config.frequency do
        field.Select(i)
        port_power, port_phase = field.Ports()
        worst_return_loss = Max(worst_return_loss, port_power[1])
      end
      field.Select((1 + #config.frequency)/2)
      port_power, port_phase = field.Ports()
      return worst_return_loss,             -- Minimal worst return loss
             port_power[3] - port_power[4]  -- Balance ports 3 and 4
    end
  end,
}
ParameterDivider()

-- Basic parameters.
B = 1.27        -- Waveguide B dimension
S = 0.605       -- Spacing to other arm of coupler
L = 2           -- Feed length

-- Adjustable post and gap sizes.
Q = {Parameter{label='Post1', min=0.05, max=1},
     Parameter{label='Gap1',  min=0.05, max=1},
     Parameter{label='Post2', min=0.05, max=1},
     Parameter{label='Gap2' , min=0.05, max=1},
     Parameter{label='Post3', min=0.05, max=1}}

-- Make the post/gap array symmetric.
for i = #Q-1,1,-1 do
  table.insert(Q, Q[i])
end
table.insert(Q, L)

-- Create the geometry.
x = L
cd = Shape()
for i = 1,#Q,2 do
  cd = cd + Rectangle(x, 0.1, x + Q[i], 2*B+S-0.1)
  x = x + Q[i] + Q[i + 1]
end
cd = cd + Rectangle(0, 0, x, B) + Rectangle(0, B+S, x, 2*B+S)

-- Add the ports.
cd:Port(cd:Select(0, B/2), 1)
cd:Port(cd:Select(0, S+B*1.5), 2)
cd:Port(cd:Select(x, B/2), 3)
cd:Port(cd:Select(x, S+B*1.5), 4)

config.cd = cd

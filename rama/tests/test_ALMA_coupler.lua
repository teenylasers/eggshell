-- Rama Simulator, Copyright (C) 2014-2020 Russell Smith.
--
-- This program is free software: you can redistribute it and/or modify it
-- under the terms of the GNU General Public License as published by the Free
-- Software Foundation, either version 3 of the License, or (at your option)
-- any later version.
--
-- This program is distributed in the hope that it will be useful, but WITHOUT
-- ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
-- FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
-- more details.

-- A simulation of the WR-10 coupler reported in "Designs of Wideband 3dB
-- Branch-line Couplers for ALMA Bands 3 to 10", Hiroya Andoh et al.
-- (http://legacy.nrao.edu/alma/memos/html-memos/alma468/memo468.pdf)

-- Set this to false to simultaneously simulate (and optimize for) a range of
-- frequencies.

default_parameters = {
  Post1 = 0.332,
  Gap1  = 0.542,
  Post2 = 0.544,
  Gap2  = 0.424,
  Post3 = 0.598,
}

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

-- Compute the total width.
total_width = L
for i = 1,#Q,2 do
  total_width = total_width + Q[i] + Q[i + 1]
end

config = {
  type = 'Exy',
  unit = 'mm',
  frequency = Parameter{label='Frequency (Hz)', min=75e9, max=115e9, default=93e9},
  mesh_edge_length = Parameter{label='Mesh edge length', min=0.1, max=0.5, default=0.2},
  excited_port = 1,
  depth = 2.54,

  -- Check that this model produces the expected results.
  test = function(port_power, port_phase, field)
    local function DrawMarker(x, y)
      k = 0.05
      Draw(Shape():AddPoint(x,y+k):AddPoint(x-k,y):AddPoint(x,y-k):AddPoint(x+k,y):AddPoint(x,y+k))
    end
    local function CheckPower(i, target)
      local dB = 10*math.log10(port_power[i])
      print('Power at port', i, 'is', dB)
      if math.abs(dB - target) > 0.01 then
        error('Unexpected power for port '..i)
      end
    end
    local function CheckField(x, y, target_re, target_im)
      local c = field.Complex(x, y)
      print('Field at', x, ',', y, 'is', c.re, ',', c.im)
      if math.abs(c.re - target_re) > 0.0001 or math.abs(c.im - target_im) > 0.0001 then
        error('Unexpected complex field value')
      end
      DrawMarker(x,y)
    end
    CheckPower(1, -33.56)
    CheckPower(2, -33.95)
    CheckPower(3, -2.70)
    CheckPower(4, -3.35)
    CheckField(total_width/2, B/2, 0.89858, 0.24795)
    CheckField(total_width/2, B*1.5+S, 0.02752, -0.37529)
  end,
}
ParameterDivider()

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

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

-----------------------------------------------------------------------------
-- Test mesher hole identification. Subtract rectangle B (which has an interior
-- hole) from rectangle A, then make sure that that hole ends up being an
-- island within an outer ring. Test this worked by making sure there's no wave
-- path from the ring to the island.

config = {
  type = 'Exy',
  unit = 'mm',
  mesh_edge_length = 0.4,
  mesh_refines = 0,
  frequency = 60e9,
  depth = Infinity,
  excited_port = 1,
  test = function(port_power, port_phase, field)
    assert(math.abs(port_power[1]-1) < 1e-3)    -- S11 ~= 0 dB
    print 'Pass test: mesher hole identification'
  end,
}

A = Rectangle(0,0,10,10) - Rectangle(3,3,7,7)
B = Rectangle(-5,-5,15,15)
cd = B - A;
cd:Port(cd:Select(-5,0), 1)
cd:Port(cd:Select(3,4), 2)

config.cd = cd

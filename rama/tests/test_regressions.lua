
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

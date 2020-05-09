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

if FLAGS.test_number == '1' then
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
end

-----------------------------------------------------------------------------
-- Test robustness of polygon cleaning.

if FLAGS.test_number == '2' then
  config = {
    type = 'Ez',
    unit = 'mm',
    mesh_edge_length = 0.3,
    mesh_refines = 0,
    frequency = 60e9,
    depth = Infinity,
    excited_port = 1,
    test = function(port_power, port_phase, field)
      print 'Pass test: robustness of polygon cleaning'
    end
  }

  -- Basic geometry.
  W = 35          -- CD width
  H = 20          -- CD height
  S = 1           -- CD step around dielectric boundary

  -- Angle and properties of boundary.
  angle_max = (math.atan((W-2*S)/H)) * 180/math.pi
  angle = Parameter{label='Angle', min=0, max=angle_max, default=25.040960422388}
  angle = -angle * math.pi/180

  -- Outer boundary.
  x_offset = math.tan(angle)*H/2
  cd = Shape():AddPoint(0,0)
              :AddPoint(W/2 + x_offset, 0)
              :AddPoint(W, -S)
              :AddPoint(W, H+S)
              :AddPoint(W/2 - x_offset, H)
              :AddPoint(0, H)

  -- Create a dielectric with an index step.
  radome = Rectangle(W/2,-H,2*W,2*H)
  radome:Offset(-W/2,-H/2):Rotate(angle*180/math.pi):Offset(W/2,H/2)

  cd:Paint(radome, 0xffff00, 2)

  cd:Clean(0)

  assert(#cd(1) == 4)
  assert(#cd(2) == 4)
  assert(cd(1)[1].y == 20)
  assert(cd(1)[2].x == 0 and cd(1)[2].y == 20)
  assert(cd(1)[3].x == 0 and cd(1)[3].y == 0)
  assert(cd(1)[4].y == 0)
  assert(cd(2)[1].x == 35 and cd(2)[1].y == 21)
  assert(cd(2)[2].y == 20)
  assert(cd(2)[3].y == 0)
  assert(cd(2)[4].x == 35 and cd(2)[4].y == -1)
  assert(cd(1)[1].x == cd(2)[2].x)
  assert(cd(1)[4].x == cd(2)[3].x)

  config.cd = cd
end

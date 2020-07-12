
-- Model the loss of a 1 meter run of copper WR-12 waveguide, and check that
-- the loss matches what we expect from theory. Check the wall and floor loss
-- separately and together, for Ez and Exy models.

test_number = tonumber(FLAGS.test_number) or
              Parameter{label='Test', min=1, max=6, default=1, integer=true}
test_number_mod3 = ((test_number - 1) % 3) + 1

config = {
  unit = 'mm',
  mesh_edge_length = Parameter{label='Mesh edge length', min=0.1, max=2, default=0.3},
  frequency = Parameter{label='Frequency (Hz)', min=60e9, max=90e9, default=75e9},
  excited_port = 1,
  test = function(port_power, port_phase, field)
    -- For a 1 meter run of copper WR-12 waveguide, we expect
    --   1.37244 dB of attenuation from the floor/ceiling.
    --   0.57091 dB of attenuation from the walls.
    --   1.94335 dB of attenuation in total.
    local loss = -10*math.log10(port_power[2])
    local expected_loss
    if     test_number == 1 or test_number == 5 then expected_loss = 1.37244
    elseif test_number == 2 or test_number == 4 then expected_loss = 0.57091
    elseif test_number_mod3 == 3 then expected_loss = 1.94335
    end
    print('Test ', test_number, ', Loss =', loss,
          'Expected loss =', expected_loss, 'Error =', loss - expected_loss)
    assert(math.abs(loss - expected_loss) < 0.02)
  end
}

if test_number <= 3 then
  config.type = 'Ez'
  W = 1000              -- Waveguide length
  H = 3.0988            -- WR-12 waveguide height (long dimension)
  config.depth = H / 2
else
  config.type = 'Exy'
  W = 1000              -- Waveguide length
  H = 3.0988 / 2        -- WR-12 waveguide height (short dimension)
  config.depth = H * 2
end

cd = Rectangle(-W/2, -H/2, W/2, H/2)

cd:Port(cd:Select(-W/2,0), 1)
cd:Port(cd:Select(W/2,0), 2)

-- The conductivity of copper.
conductivity = 5.96e7

if test_number_mod3 == 1 or test_number_mod3 == 3 then
  local ep = util.PaintMetal(cd, Rectangle(-W/2, -H/2, W/2, H/2), 0xffff80, conductivity)
  print('Material epsilon =', ep.re, ep.im)
end
if test_number_mod3 == 2 or test_number_mod3 == 3 then
  util.PortMetal(cd, cd:Select(0, H/2), 3, conductivity)
  util.PortMetal(cd, cd:Select(0,-H/2), 4, conductivity)
end

config.cd = cd

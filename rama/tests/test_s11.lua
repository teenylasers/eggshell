
test_number = tonumber(FLAGS.test_number) or
              Parameter{label='Test number', min=1, max=2, integer=true}

mag = Parameter{label='S11 mag',  min=-50, max=0, default=-4}
phase = Parameter{label='S11 phase', min=0, max=360}
s11 = 10^(mag/20) * complex.exp(Complex(0,phase/180*math.pi))

config_type = 'Exy'
L = 500
W = 61
if test_number == 2 then
  config_type = 'Ez'
  W = 122
end

config = {
  type = config_type,
  unit = 'mil',
  mesh_edge_length = 5,
  excited_port = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
  frequency = 70e9,
  depth = 122,
  test = function(port_power, port_phase, field)
    local error1 = math.abs(port_power[1] - s11.abs^2)
    local error2 = math.abs(port_power[2] - (1-s11.abs^2))
    print('Test', test_number, ':', port_power[1], s11.abs^2, error1, error2)
    assert(error1 < 0.01)
    assert(error2 < 0.01)

    -- Changing the s11 phase should be visible at port 1 but not at port 2
    assert(math.abs(port_phase[2] - port_phase[4]) < 1e-3)
    assert(math.abs(port_phase[2] - port_phase[6]) < 1e-3)
    assert(math.abs(port_phase[2] - port_phase[8]) < 1e-3)
    assert(math.abs(port_phase[3] - port_phase[1] - 0.5*math.pi) < 2e-3)
    assert(math.abs(port_phase[5] - port_phase[1] - 1.0*math.pi) < 2e-3)
    assert(math.abs(port_phase[7] - port_phase[1] - 1.5*math.pi) < 2e-3)
  end,
}

cd = Shape()
for i = 0,3 do
  local phase_adjust = complex.exp(Complex(0,i*90/180*math.pi))

  local s = Rectangle(0, 0, L, W)
  s:Port(s:Select(0, W/2), i*2 + 1)
  s:Port(s:Select(L, W/2), i*2 + 2, s11 * phase_adjust)
  cd = cd + s:Offset(0, i*200)
end

config.cd = cd

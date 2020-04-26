
-- Anisotropic material test with a simple waveguide filled with the
-- anisotropic material.
--
-- Check that we can simulate a dielectric constant (epsilon) by setting the
-- sigma values to 1/epsilon.
--
-- @@@ Add tests that cover Ez here when we get those boundary conditions
--     working.

-- Test number 0 takes parameter values from the sliders. The other tests
-- numbers use hard coded values.
test_number = tonumber(FLAGS.test_number) or
              Parameter{label='Test number', min=0, max=12, integer=true}

if Parameter{label='Ez (0) or Exy (1)', min=0, max=1, default=1} == 0 then
  the_type = 'Ez'
else
  the_type = 'Exy'
end

angle = Parameter{label='Angle',  min=0, max=360, default=0}
epsilon = Complex(Parameter{label='Epsilon R',  min=0.5, max=2, default=1},
                  Parameter{label='Epsilon I',  min=-0.5, max=0.5, default=0})
sigma_epsilon = Parameter{label='Sigma Epsilon', min=0.5, max=2, default=1}

if test_number >= 1 then
  angle = math.floor((test_number-1) / 3) * 30
  epsilon = 1
  sigma_epsilon = 1
  the_type = 'Exy'
  if math.fmod(test_number, 3) == 2 then epsilon = 2 end
  if math.fmod(test_number, 3) == 0 then sigma_epsilon = 2 end
  print('Test epsilon=', epsilon, 'sigma_epsilon=', sigma_epsilon, 'type=', the_type)
end

config = {
  type = the_type,
  unit = 'mil',
  mesh_edge_length = Parameter{label='Mesh edge length', min=2, max=30, default=5},
  mesh_refines = 0,
  excited_port = 1,
  frequency = Parameter{label='Frequency (Hz)', min=30e9, max=90e9, default=70e9},
  depth = Infinity,
  test = function(port_power, port_phase, field)
    if test_number >= 1 then
      print('Test', test_number)
      local function DrawMarker(x, y)
        k = 0.05
        Draw(Shape():AddPoint(x,y+k):AddPoint(x-k,y):AddPoint(x,y-k):AddPoint(x+k,y):AddPoint(x,y+k))
      end
      local function CheckField(x, y, target_re, target_im)
        local c = field.Complex(x, y)
        print('Field at', x, ',', y, 'is', c.re, ',', c.im)
        if math.abs(c.re - target_re) > 0.0001 or math.abs(c.im - target_im) > 0.0001 then
          error('Unexpected complex field value')
        end
        DrawMarker(x,y)
      end
      assert(port_power[1] < 1e-5)
      assert(math.abs(port_power[2] - 1) < 1e-5)
      if epsilon == 1 and sigma_epsilon == 1 then
        if the_type == 'Exy' then
          assert(math.abs(port_phase[2] / math.pi * 180 - 13.1) < 0.1)
        else
          assert(math.abs(port_phase[2] / math.pi * 180 + 50.8) < 0.1)
        end
      else
        if the_type == 'Exy' then
          assert(math.abs(port_phase[2] / math.pi * 180 + 67.75) < 0.3)
        else
          assert(math.abs(port_phase[2] / math.pi * 180 - 124.9) < 0.3)
        end
      end
    end
  end,
}

L = 500         -- Length
H = 122         -- Height

c = math.cos(angle/180 * math.pi)
s = math.sin(angle/180 * math.pi)
cd = Shape():AddPoint(0, 0):AddPoint(c*L, s*L):AddPoint(c*L-s*H, s*L+c*H):AddPoint(-s*H, c*H)
cd:Port(cd:Select(-s*H/2, c*H/2), 1)
cd:Port(cd:Select(c*L-s*H/2, s*L+c*H/2), 2)

-- Paint the anisotropic dielectric. Use 1/epsilon in the long dimension of the
-- guide, and 1 in the short dimension. Everything is symmetric along the short
-- dimension of the guide so we expect the field to be the same regardless of
-- angle.
sxx = (1/sigma_epsilon-1)*c^2+1
syy = (1-1/sigma_epsilon)*c^2+1/sigma_epsilon
sxy = (1/sigma_epsilon-1)*c*s

cd:RawPaint(Rectangle(-10000,-10000,10000,10000), 0xff8000, epsilon, sxx, syy, sxy)

config.cd = cd

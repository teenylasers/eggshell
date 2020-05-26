
-- Study the modes of waveguides with various cross sections.

config = {
  type = 'TE',
  unit = 'mil',
  mesh_edge_length = Parameter{label='Mesh edge length', min=2, max=30, default=5},
  max_modes = 10,
}

-- The geometry is 0 for a rectangular guide, 1 for a circular guide, or 2
-- for a coaxial cable.
geom = Parameter{label='Geometry', min=0, max=2, default=0, integer=true}

-- Size of the guide.
A = Parameter{label='A dimension', min=60, max=200, default=122}

-- Aspect ratio of the guide.
K = Parameter{label='Aspect', min=0.1, max=1, default=0.5}

-- Draft angle of a rectangular guide.
DRAFT = Parameter{label='Draft (deg)', min=0, max=30, default=0}

-- Misalignment between two sides of a rectangular guide.
OFS = Parameter{label='Offset', min=-50, max=50, default=0}

-- Radius of fillets in a rectangular guide.
F = Parameter{label='Fillets', min=0, max=30, default=0}

if geom == 0 then
  local D = math.tan(DRAFT * math.pi / 180) * (A/2)
  config.cd = (Shape():AddPoint(-A/2, -A*K/2):AddPoint(0, -A*K/2-D):
                       AddPoint(0, A*K/2+D):AddPoint(-A/2, A*K/2))
            + (Shape():AddPoint(A/2, A*K/2):AddPoint(0, A*K/2+D):
                       AddPoint(0, -A*K/2-D):AddPoint(A/2, -A*K/2)):Offset(0,OFS)
  if F > 0 then
    config.cd = MakeMachinable(config.cd, F, 0.1)
  end
  if F > 0 then
    config.cd = MakeMachinable(config.cd, -F, 0.1)
  end
elseif geom == 1 then
  config.cd = Circle(0, 0, A/2, 64):Scale(1, K)
elseif geom == 2 then
  config.cd = Circle(0, 0, A/2, 64) - Circle(0, 0, A*K/2, 64):Offset(OFS, 0)
end

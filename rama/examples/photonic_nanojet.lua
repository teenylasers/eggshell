
-- Simulate the "photonic nanojet" from figure Fig 1.a of
--   https://www.ncbi.nlm.nih.gov/pmc/articles/PMC2782642/
-- To get exactly the image in that paper, select 'Hz max amplitude',
-- 'Color Jet', and turn the brightness down a bit.

config = {
  type = 'Ez',
  unit = 'micron',
  mesh_edge_length = Parameter{label='Mesh edge length', min=0.01, max=0.2, default=0.05},
  excited_port = 1,
  frequency = 600e12,   -- Lambda0 = 500nm
}

W = 8           -- Computational domain, width
H = 10          -- Computational domain, height
k = 0.1

cd = Rectangle(-W/2,-H/2,W/2,H/2)
     -- Take a chip out of the top left and top right corners so that we can
     -- have a port adjacent to an ABC:
     - Rectangle(-W/2,H/2,-W/2+k,H/2-k)
     - Rectangle(-W/2,-H/2,-W/2+k,-H/2+k)
cd:ABC(cd:Select(W/2,0))
cd:ABC(cd:Select(0,H/2))
cd:ABC(cd:Select(0,-H/2))
cd:Port(cd:Select(-W/2,0), 1)

-- The lens, 5um in diameter with relative dielectric constant 3.5. Note that
-- the paper instead claims a refractive index of 3.5, but that is likely an
-- error.
cd:Paint(Circle(0, 0, 2.5, 100), 0xffffc0, 3.5)

config.cd = cd

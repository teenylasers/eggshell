
-- Simulate the static electric field between the two plates of a capacitor.
-- Notice that:
--   * The E field is strongest on the corners of the plates.
--   * The E field between the plates becomes stronger as the plates come
--     together.
--   * The E field inside the dielectric region between the plates becomes
--     weaker as the dielectric constant increases.

config = {
  type = 'ES',
  unit = 'mm',
  mesh_edge_length = Parameter{label='Mesh', min=0.01, max=0.05, default=0.02},
  excited_port = 1,
  frequency = 1,
}

cd = Rectangle(-0.5, -0.5, 0.5, 0.5)

spacing = Parameter{label='Spacing', min=0.03, max=0.79, default=0.3}

PW = 0.1        -- Plate width
PH = 0.5        -- Plate height

electrode1 = Rectangle(-spacing/2-PW,-PH/2,-spacing/2,PH/2)
util.PortCharge(electrode1, electrode1:SelectAll(), 1, 1)
cd = cd - electrode1

electrode2 = Rectangle(spacing/2,-PH/2,spacing/2+PW,PH/2)
util.PortCharge(electrode2, electrode2:SelectAll(), 2, -1)
cd = cd - electrode2

cd:Paint(Rectangle(-spacing/2+0.01,-PH/2,spacing/2-0.01,PH/2), 0xffffc0,
         Parameter{label='Epsilon', min=1, max=10, default=1})

config.cd = cd

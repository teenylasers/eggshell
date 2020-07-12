
-- Study waveguide bends to understand how sharp a bend can be made before
-- there is substantial reflection.

config = {
  unit = 'mil',
  mesh_edge_length = Parameter{label='Mesh edge length', min=2, max=30, default=10},
  excited_port = 1,
  frequency = Parameter{label='Frequency (Hz)', min=30e9, max=90e9, default=70e9},
  depth = 122,
}

if Parameter{label='E or H bend', min=0, max=1, default=1, integer=true} == 1 then
  config.type = 'Ez'
else
  config.type = 'Exy'
end

if config.type == 'Ez' then
  W = 122                               -- Waveguide width
else
  W = 61                                -- Waveguide width
end
T = 1500
R = Parameter{label='Center radius', min=50, max=300, default=100}
A = Parameter{label='Bend angle', min=0, max=180, default=45} * math.pi/180
N = math.ceil(A*R/10) + 2               -- Number of points in bend curve
bendlength = A*R                        -- Bend length along centerline
L = (T - bendlength)/2                  -- Feed length before bend

cd = Shape():AddPoint(-L,-R)
local x2,y2                             -- Port 2 location
for i = 1,N do
  local angle = -math.pi/2 + A*(i-1)/(N-1)
  local ux,uy = math.cos(angle), math.sin(angle)
  if i > 1 or A > 0 then
    cd:AddPoint(R*ux, R*uy)
  end
  if i == N then
    x2,y2 = R*ux - L*uy, R*uy + L*ux
  end
end
cd:AddPoint(x2,y2):MakePolyline():Offset(0,R)
Draw(cd)
cd:Grow(W/2,'miter',1000)
cd:Port(cd:Select(-L, 0), 1)
cd:Port(cd:Select(x2, y2+R), 2)

config.cd = cd

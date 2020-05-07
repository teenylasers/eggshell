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

function TryDrawText(x,y,s,ha,va)
  print(x,y,s,ha,va)
  DrawText(x,y,s,ha,va)
  Draw(Shape():AddPoint(x-0.05,y-0.05):AddPoint(x+0.05,y+0.05))
  Draw(Shape():AddPoint(x+0.05,y-0.05):AddPoint(x-0.05,y+0.05))
end

value = Parameter{label='Value', min=0, max=2, default=1, integer=true}

for x = -1,1 do
  local ha = 'center'
  if x == -1 then ha = 'right' end
  if x ==  1 then ha = 'left' end
  for y = -1,2 do
    local va = 'baseline'
    if y == -1 then va = 'top' end
    if y ==  0 then va = 'center' end
    if y ==  1 then va = 'bottom' end
    TryDrawText(0.5+0.25*x,0.6+0.25*y,ha..','..va..','..value,ha,va)
  end
end

TryDrawText(0.5,0.1,'Default')


config = {
  type = 'Ez',
  unit = 'm',
  mesh_edge_length = 0.1,
  mesh_refines = 0,
  excited_port = 1,
  frequency = 60e9,
  cd = Rectangle(0, 0, 1, 1.25 + 0.05*value)
}

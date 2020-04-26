
-- Test geometry functions.

config = {
  type = 'Ez',
  unit = 'm',
  mesh_edge_length = 1e9,
  mesh_refines = 0,
  excited_port = 1,
  frequency = 60e9,
  depth = 0,
  cd = Rectangle(-0.5, -0.5, 1.5, 1.5)
}

-- Create a positive area shape with M random vertices.
function RandomShape1(M)
  local s = Shape()
  for i = 1,M do
    s:AddPoint(math.random(), math.random())
  end
  if s.area < 0 then
    s:Reverse()
  end
  return s
end

-- Create a shape with one positive area piece and no more than H holes by
-- combining a number of rectangles.
function RandomShape2(H)
  local N = 4             -- Number of rectangles to combine
  while true do
    local s = Shape()
    for i = 1,N do
      local w = math.random()
      local h = math.random()
      local r = Rectangle(-w/2, -h/2, w/2, h/2)
      r:Rotate(math.random() * 360)
      r:Offset(math.random(), math.random())
      if i == N then
        s = s - r     -- Subtract the last rectangle to help make holes
      else
        s = s + r
      end
    end
    local posarea_pieces = 0
    local negarea_pieces = 0
    for i = 1,s.pieces do
      if s(i).area > 0 then
        posarea_pieces = posarea_pieces + 1
      else
        negarea_pieces = negarea_pieces + 1
      end
    end
    if posarea_pieces == 1 and negarea_pieces <= H then
      return s
    end
  end
end

function VisualInspectionOfContains()
  local M = 5           -- Number of polygon points
  local N = 51          -- Grid size
  local sz = 0.005      -- Marker size

  s = RandomShape1(M)
  Draw(s)

  for ix = 1,N do
    local x = (ix-1) / (N-1)
    for iy = 1,N do
      local y = (iy-1) / (N-1)
      if s:Contains(x, y) then
        Draw(Rectangle(x-sz, y-sz, x+sz, y+sz))
      end
    end
  end
  config.cd = Rectangle(-0.1, -0.1, 1.1, 1.1)
end

-- @@@ Test moved to shape.cc.
function TestArea()
  local s = Rectangle(1,1,10,5);
  assert(s.area == 36)

  -- Test mirroring preserves area.
  s:MirrorX(4)
  assert(s.area == 36)
  s:MirrorY(2)
  assert(s.area == 36)

  -- Test reversing negates area.
  s:Reverse()
  assert(s.area == -36)

  -- Test triangle areas.
  for test_num = 1,100 do
    local ax = math.random()
    local ay = math.random()
    local bx = math.random()
    local by = math.random()
    local cx = math.random()
    local cy = math.random()
    local triarea = -0.5 * ((cx + ax) * (cy - ay) + (ax + bx) * (ay - by) +
                            (bx + cx) * (by - cy))
    s = Shape()
    s:AddPoint(ax, ay)
    s:AddPoint(bx, by)
    s:AddPoint(cx, cy)
    assert(math.abs(s.area - triarea) < 1e-9)
  end
end

-- @@@ Test moved to shape.cc.
function TestAPointInside()
  local sz = 0.005        -- Marker size
  local num_tests = 1000  -- Number of tests to do

  for test_num = 1,num_tests do
    -- Create a shape with one positive area piece and any number of holes.
    local s = RandomShape2(100)

    -- Test APointInside.
    local px, py = s:APointInside()
    if not s:Contains(px, py) then
      error("APointInside() failed")
    end

    -- Visualize just one of the tests.
    if test_num == 1 then
      Draw(s)
      Draw(Rectangle(px-sz, py-sz, px+sz, py+sz))
    end

    -- Create a shape with one positive area piece and no holes.
    s = RandomShape2(0)

    -- Test APointInside.
    local px, py = s:APointInside()
    if not s:Contains(px, py) then
      error("APointInside() failed")
    end

    -- Test reverse orientation polys too.
    px, py = s:Clone():Reverse():APointInside()
    if not s:Contains(px, py) then
      error("APointInside() failed (reverse orientation)")
    end

    -- Test triangles, make sure this simple case works too.
    s = RandomShape1(3)
    px, py = s:APointInside()
    if not s:Contains(px, py) then
      error("APointInside() failed for triangle")
    end
    px, py = s:Clone():Reverse():APointInside()
    if not s:Contains(px, py) then
      error("APointInside() failed for triangle (reverse orientation)")
    end
  end
end

--VisualInspectionOfContains()
TestArea()
TestAPointInside()

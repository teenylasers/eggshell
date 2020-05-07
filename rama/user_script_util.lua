-- Lua utility functions that are available to user scripts.

Infinity = 1.7976931348623157e+308  -- Close enough, this is the maximum double

function Parameter(T)
  -- Check that all keys in T have valid type.
  local keys = {label='string', min='number', max='number', default='number',
                integer='boolean'}
  for k,v in pairs(T) do
    if keys[k] then
      if type(v) ~= keys[k] then
        error("In argument table, '"..tostring(k).."' should have type '"..
              keys[k].."' but has type '"..type(v).."'")
      end
      keys[k] = nil
    else
      error("In argument table, unknown key '"..tostring(k).."'")
    end
  end

  -- Check that all necessary keys are present).
  keys.integer = nil            -- Optional field
  keys.default = nil            -- Optional field
  if next(keys) ~= nil then
    error("In argument table, missing '"..next(keys).."' key")
  end

  -- Check numerical values.
  local min = tonumber(T.min)
  local max = tonumber(T.max)
  local def_params = rawget(_G, 'default_parameters') or {}
  local default = tonumber(T.default or def_params[T.label]) or min
  local integer = T.integer or false
  if integer then
    if min ~= math.floor(min) or max ~= math.floor(max) or
       default ~= math.floor(default) then
      error("In argument table, min, max and default should be integers")
    end
  end
  if min >= max then
    -- Don't allow min == max because then slider creation will fail.
    error("In argument table, you must have min < max")
  end
  if default < min or default > max then
    error("In argument table, you must have default between min and max")
  end

  -- Create the parameter and return its current value.
  local label = tostring(T.label)
  if label == '' then
    error('The label can not be the empty string')
  end
  if string.find(label, "'") then
    error("The label can not using the single quote character: '")
    -- ...since we don't escape it when copying parameters to the clipboard
  end
  return _CreateParameter(label, min, max, default, integer)
end

function ParameterMarker(T)
  if type(T.min) ~= 'table' or not T.min[1] or not T.min[2] then
    error("The 'min' field should be a table of the format {x,y}")
  end
  if type(T.max) ~= 'table' or not T.max[1] or not T.max[2] then
    error("The 'max' field should be a table of the format {x,y}")
  end
  local def = T.default or {}
  local xname = T.label..' x'
  local yname = T.label..' y'
  local x = Parameter{label=xname, min=T.min[1], max=T.max[1], default=def[1]}
  local y = Parameter{label=yname, min=T.min[2], max=T.max[2], default=def[2]}
  _CreateMarker(xname, yname)
  return x, y
end

-- This is the function called by s:Paint(). It does some parameter
-- transformations then calls s:RawPaint().
function __Paint__(s, q, color, epsilon, sxx, syy, sxy, excitation, dummy)
  assert(not dummy, "Too many arguments given to Paint.")
  -- In Exy cavities we scale sigma by 1/epsilon and set epsilon to 1, to
  -- achieve a discontinuous gradient across dielectric boundaries.
  if not config or type(config.type) ~= 'string' then
    error('config table should be set before Paint() is called')
  end
  if config.type == 'Exy' then
    if not rawget(_G,'allow_painting_in_finite_depth_Exy') and config.depth and config.depth < Infinity then
      error('Painting in finite depth Exy cavities does not yet generate physically sensible results.\n'..
            'Set allow_painting_in_finite_depth_Exy=true to suppress this error,\n'..
            'or set depth=Infinity.\n')
    end
    if type(epsilon) == 'function' then
      -- epsilon and sigma values are returned by a callback function. We
      -- create an intermediate function here that modifies those returned
      -- values.
      s:RawPaint(q, color, function(x, y)
        local e, sxx, syy, sxy, excitation = epsilon(x, y)
        sxx = sxx or (x*0+1)
        syy = syy or (x*0+1)
        sxy = sxy or (x*0)
        sxx = sxx / e
        syy = syy / e
        sxy = sxy / e
        e = (x*0+1)
        return e, sxx, syy, sxy, excitation
      end)
    else
      -- epsilon and sigma values are (potentially complex) numbers.
      sxx = sxx or 1
      syy = syy or 1
      sxy = sxy or 0
      sxx = sxx / epsilon
      syy = syy / epsilon
      sxy = sxy / epsilon
      epsilon = 1
      s:RawPaint(q, color, epsilon, sxx, syy, sxy, excitation)
    end
  else
    -- Not an Exy cavity, pass on original arguments.
    s:RawPaint(q, color, epsilon, sxx, syy, sxy, excitation)
  end
end

-- Create a table that returns a zero for every index. This is sometimes useful
-- as arguments to config.optimize().
__ZeroTable__ = setmetatable({}, {__index = function(T,k) return 0 end })

-- The third argument to optimize, a table which contains field lookup
-- functions.
__Optimize3rdArg__ = {
  Complex = function(x, y)
    return Complex(_GetField(x, y))
  end,
  Magnitude = function(x, y)
    local re,im = _GetField(x, y)
    return math.sqrt(re * re + im * im)
  end,
  Phase = function(x, y)
    local re,im = _GetField(x, y)
    return math.atan(im, re)
  end,
  Poynting = function(x, y)
    return _GetFieldPoynting(x, y)
  end,
  Power = function(x, y)
    local px,py = _GetFieldPoynting(x, y)
    return math.sqrt(px * px + py * py)
  end,
  Pattern = _Pattern,
  Directivity = _Directivity,
  Select = _Select,
  SolveAll = _SolveAll,
  Ports = _Ports,
}

-- A dummy __Optimize3rdArg__ argument that allows trial execution of the
-- optimizer function but that does not actually do any work.
__DummyOptimize3rdArg__ = {
  Complex = function() return 0,0 end,
  Magnitude = function() return 0 end,
  Phase = function() return 0 end,
  Poynting = function() return 0,0 end,
  Power = function() return 0 end,
  Pattern = function() return 0 end,
  Directivity = function() return 0 end,
  Select = function() end,
  SolveAll = function() end,
  Ports = function() return __ZeroTable__, __ZeroTable__ end,
}

-- Utility functions.

function MakeMachinable(shape, radius, limit)
  return shape:Grow(-radius, 'round', limit):Grow(radius, 'round', limit)
end

function LinearRange(start, stop, n)
  local T = {}
  for i = 1,n do
    T[i] = start + (i-1)*(stop - start)/(n-1)
  end
  return T
end

function Min(a, b)
  if a < b then return a else return b end
end

function Max(a, b)
  if a > b then return a else return b end
end

-----------------------------------------------------------------------------
-- Complex numbers represented as tables {real_part, imaginary_part, fn=T}. The
-- real and imaginary parts can be scalars or vectors. The table T is either
-- the global 'math' or 'vec' table which provides access to math functions,
-- depending on the types of the real and imaginary parts.

__complex_metatable__ = {}

-- Cast 'a' and 'b' to tables containing complex numbers or vectors. Return the
-- two complex tables, and check that the return values are compatible. Also
-- return the function table to use when combining 'a' and 'b', which is 'vec'
-- if either is a vector, 'math' otherwise.
local function CastComplexArguments(a, b)
  -- Cast 'a' and 'b' to complex.
  if getmetatable(a) ~= __complex_metatable__ then
    assert(type(a) == 'number' or vec.IsVector(a), 'Complex type incompatibility')
    a = Complex(a, a*0)         -- The a*0 makes a compatible imaginary part
  end
  if b and getmetatable(b) ~= __complex_metatable__ then
    assert(type(b) == 'number' or vec.IsVector(b), 'Complex type incompatibility')
    b = Complex(b, b*0)         -- The b*0 makes a compatible imaginary part
  end
  -- If we have two complex vectors then check that the sizes are compatible.
  if vec.IsVector(a[1]) and b and vec.IsVector(b[1]) then
    assert(#a[1] == #a[2], 'Complex vector size incompatibility')
  end
  -- See which function table to use.
  if vec.IsVector(a[1]) or (b and vec.IsVector(b[1])) then
    return a, b, vec
  else
    return a, b, math
  end
end

__complex_metatable__.__add = function(a, b)  -- the addition (+) operation
  local a,b,fn = CastComplexArguments(a, b)
  return setmetatable({a[1]+b[1], a[2]+b[2], fn=fn}, __complex_metatable__)
end
__complex_metatable__.__sub = function(a, b)  -- the subtraction (-) operation
  local a,b,fn = CastComplexArguments(a, b)
  return setmetatable({a[1]-b[1], a[2]-b[2], fn=fn}, __complex_metatable__)
end
__complex_metatable__.__mul = function(a, b)  -- the multiplication (*) operation
  local a,b,fn = CastComplexArguments(a, b)
  return setmetatable({a[1]*b[1] - a[2]*b[2], a[2]*b[1] + a[1]*b[2], fn=fn}, __complex_metatable__)
end
__complex_metatable__.__div = function(a, b)  -- the division (/) operation
  local a,b,fn = CastComplexArguments(a, b)
  local bm = b[1]*b[1] + b[2]*b[2]
  return setmetatable({(a[1]*b[1] + a[2]*b[2])/bm, (a[2]*b[1] - a[1]*b[2])/bm, fn=fn}, __complex_metatable__)
end
__complex_metatable__.__pow = function(a, b)  -- the exponentiation (^) operation
  local a,b,fn = CastComplexArguments(a, b)
  local am = a[1]*a[1] + a[2]*a[2]
  -- If am == 0 this will fail as the complex power of zero is indeterminate.
  -- However we do not test for this case as 'am' could be a vector.
  local aa = fn.atan(a[2], a[1])
  local angle = aa*b[1] + b[2]/2*fn.log(am)
  local mag = am^(b[1]/2)*fn.exp(-aa*b[2])
  return setmetatable({mag*fn.cos(angle), mag*fn.sin(angle), fn=fn}, __complex_metatable__)
end
__complex_metatable__.__unm = function(a)  -- the negation (unary -) operation
  local a,dummy,fn = CastComplexArguments(a)
  return setmetatable({-a[1],-a[2], fn=fn}, __complex_metatable__)
end
__complex_metatable__.__index = function(T, key)  -- The indexing operation T[key]
  if key == 're' then
    return T[1]
  elseif key == 'im' then
    return T[2]
  elseif key == 'abs' then
    return T.fn.sqrt(T[1]*T[1] + T[2]*T[2])
  elseif key == 'angle' then
    return T.fn.atan(T[2], T[1])
  else
    error 'Unknown complex number operation'
  end
end

function Complex(re, im)
  local revec = vec.IsVector(re)
  if not im then
    -- Allow Complex(re) to be written:
    if revec then
      im = Vector():Resize(#re)
    else
      im = 0
    end
  end
  local imvec = vec.IsVector(im)
  assert ((revec and imvec) or (not revec and not imvec),
          'Vector and nonvector arguments to Complex() not allowed')
  local fn_table = math
  if revec then
    assert(#re == #im, 'Two vector arguments should have the same size')
    fn_table = vec
  end
  return setmetatable({re, im, fn=fn_table}, __complex_metatable__)
end

-- Complex math functions.
complex = {
  exp = function(a)
    local a,dummy,fn = CastComplexArguments(a)
    local e = fn.exp(a.re)
    return setmetatable({e*fn.cos(a.im), e*fn.sin(a.im), fn=fn}, __complex_metatable__)
  end,
}

-----------------------------------------------------------------------------
-- Utility functions and PML painting support.

util = {}
util.SpeedOfLight = 299792458           -- in m/s
util.FAR_FIELD = 0x1000000              -- Special material color

-- Return the distance scale in 'meters per unit'. This is the length of one
-- 'config.unit' unit of distance in meters.
util.DistanceScale = function()
  assert(config and config.unit, 'config.unit must be defined')
  return _DistanceScale(config.unit)
end

-- Return the free space wavelength in meters.
util.LambdaInM = function()
  assert(config and config.frequency, 'config.frequency must be defined')
  return util.SpeedOfLight / config.frequency
end

-- Return the free space wavelength in 'config.unit' units.
util.Lambda = function()
  assert(config and config.unit, 'config.unit must be defined')
  return util.LambdaInM() / _DistanceScale(config.unit)
end

-- Compute K^2, in (radians per meter)^2.
util.KSquaredInM = function()
  assert(config and config.unit and config.type, 'config.unit and config.type must be defined')
  local lambda = util.LambdaInM()
  local k2 = 0
  if config.type == 'Exy' then
    local unit = _DistanceScale(config.unit)
    assert(config.depth, 'config.depth must be defined')
    k2 = (2 * math.pi / lambda)^2 - (math.pi / (config.depth * unit))^2
  elseif config.type == 'Ez' then
    k2 = (2 * math.pi / lambda)^2
  else
    assert(false, 'Unsupported config.type')
  end
  return k2
end

-- Compute K, in radians per meter. If For Exy cavities if k^2 is negative then
-- k is imaginary and travelling waves will not propagate at this frequency,
-- and this function will fail.
util.KinM = function()
  return math.sqrt(util.KSquaredInM())
end

-- Compute K, in radians per config.unit.
util.K = function()
  return util.KinM() * _DistanceScale(config.unit)
end

-- Rotate sigma values by the given angle (in radians). The angle argument can
-- also be a vector of angles, in which case vectors are returned.
util.RotateSigmas = function(sxx, syy, sxy, angle)
  local sa, ca
  if vec.IsVector(angle) then
    sa = vec.sin(angle)
    ca = vec.cos(angle)
  else
    sa = math.sin(angle)
    ca = math.cos(angle)
  end
  local SRxx = sxx*ca - sxy*sa
  local SRxy = sxx*sa + sxy*ca
  local SRyx = sxy*ca - syy*sa
  local SRyy = sxy*sa + syy*ca
  return SRxx*ca - SRyx*sa, SRxy*sa + SRyy*ca, SRxy*ca - SRyy*sa
end

-- Paint a PML region P into the shape S, with normal incidence at the given
-- angle (in radians). The strength of the PML is given as how much attenuation
-- we want (as a fraction of full strength) over what distance. The distance
-- does not have to correspond to any dimension in P.
util.PaintPML = function(S, P, angle, distance, attenuation, far_field_boundary)
  -- Compute alpha so that attenuation == exp(-alpha*distance)
  local unit = _DistanceScale(config.unit)
  local alpha = -math.log(attenuation) / (distance * unit)

  -- Compute the desired gradient ratio across the boundary.
  -- @@@ The sqrt is a problem for non-propagating Exy as k2 < 0.
  local k2 = util.KSquaredInM()
  local gr = Complex(1, -alpha / math.sqrt(k2))

  -- For angle 0 the sigma_xx/sigma_yy and sigma_xx/epsilon ratio is 1/gr^2.
  -- But the gradient ratio is also 1/sigma_xx. We can scale all sigmas and
  -- epsilon by an arbitrary amount without affecting the wave within the
  -- boundary. To achieve these constraints we set:
  local sxx = 1 / gr            -- 1/sxx is gradient ratio
  local syy = gr                -- sxx/syy = 1/gr^2
  local sxy = 0
  local epsilon = gr            -- sxx/epsilon = 1/gr^2

  -- Rotate sigmas and paint.
  sxx, syy, sxy = util.RotateSigmas(sxx, syy, sxy, angle)
  local color = 0x808080
  if far_field_boundary then
    color = color | util.FAR_FIELD
  end
  S:RawPaint(P, color, epsilon, sxx, syy, sxy)
end

-- Paint a rectangular PML to the given rectangle coordinates, of thickness T.
util.PaintRectangularPML = function(S, x1, y1,x2, y2, T, attenuation, sides)
  sides = sides or 'tblr'
  local top    = string.find(sides, 't', 1, true)
  local bottom = string.find(sides, 'b', 1, true)
  local left   = string.find(sides, 'l', 1, true)
  local right  = string.find(sides, 'r', 1, true)
  local function N(a) if a then return 1 else return 0 end end -- Boolean to number
  local a = attenuation or 0.01
  if top and left then
    util.PaintPML(S, Rectangle(x1-T,y2-T,x1+T,y2+T), 0.75*math.pi, T, a, true)
  end
  if top and right then
    util.PaintPML(S, Rectangle(x2-T,y2-T,x2+T,y2+T), math.pi/4, T, a, true)
  end
  if bottom and left then
    util.PaintPML(S, Rectangle(x1-T,y1-T,x1+T,y1+T), -0.75*math.pi, T, a, true)
  end
  if bottom and right then
    util.PaintPML(S, Rectangle(x2-T,y1-T,x2+T,y1+T), -math.pi/4, T, a, true)
  end
  if top then
    util.PaintPML(S, Rectangle(x1+N(left)*T,y2-T,x2-N(right)*T,y2+T), math.pi/2, T, a, true)
  end
  if left then
    util.PaintPML(S, Rectangle(x1-T,y1+N(bottom)*T,x1+T,y2-N(top)*T), math.pi, T, a, true)
  end
  if right then
    util.PaintPML(S, Rectangle(x2-T,y1+N(bottom)*T,x2+T,y2-N(top)*T), 0, T, a, true)
  end
  if bottom then
    util.PaintPML(S, Rectangle(x1+N(left)*T,y1-T,x2-N(right)*T,y1+T), -math.pi/2, T, a, true)
  end
end

-- Paint a circular PML to the given circle coordinates, of thickness T.
util.PaintCircularPML = function(S, cx, cy, r, npoints, T, attenuation)
  local paint = Rectangle(cx-r-2*T, cy-r-2*T, cx+r+2*T, cy+r+2*T)
                - Circle(cx, cy, r-T, npoints)
  -- See PaintPML for the detailed rationale:
  local unit = _DistanceScale(config.unit)
  local alpha = -math.log(attenuation) / (T * unit)
  local k2 = util.KSquaredInM()
  local gr = Complex(1, -alpha / math.sqrt(k2))
  local sxx = 1 / gr            -- 1/sxx is gradient ratio
  local syy = gr                -- sxx/syy = 1/gr^2
  local sxy = Complex(0)
  local epsilon = gr            -- sxx/epsilon = 1/gr^2
  S:RawPaint(paint, 0x808080 | util.FAR_FIELD, function(x, y)
    local angle = vec.atan(y-cy, x-cx)
    local sxx2, syy2, sxy2 = util.RotateSigmas(sxx, syy, sxy, angle)
    local e = epsilon + x*0
    return epsilon + x*0, sxx2, syy2, sxy2
  end)
end

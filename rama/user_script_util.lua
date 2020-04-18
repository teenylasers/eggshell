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
function Paint(s, q, color, epsilon, sxx, syy, sxy)
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
        local e, sxx, syy, sxy = epsilon(x, y)
        sxx = sxx or (x*0+1)
        syy = syy or (x*0+1)
        sxy = sxy or (x*0)
        sxx = sxx / e
        syy = syy / e
        sxy = sxy / e
        e = (x*0+1)
        return e, sxx, syy, sxy
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
      s:RawPaint(q, color, epsilon, sxx, syy, sxy)
    end
  else
    -- Not an Exy cavity, pass on original arguments.
    s:RawPaint(q, color, epsilon, sxx, syy, sxy)
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

-- Given than a and b are complex tables, check that they are compatible.
-- It is assumed that the real and imaginary parts are already compatible.
local function CheckComplexCompatibility(a, b)
  local avec = vec.IsVector(a[1])
  local bvec = vec.IsVector(b[1])
  assert( (type(a[1]) == 'number' or avec) or
          (type(b[1]) == 'number' or bvec), 'Complex type incompatibility')
  if avec and bvec then
    assert(#a[1] == #a[2], 'Complex vector sizve incompatibility')
  end
  return a, b
end

-- Check that a and b are tables containing complex numbers. If they are just
-- scalars then cast them to complex tables. Return the two complex tables, and
-- check that the return values are compatible.
local function CheckComplexArguments(a, b)
  if getmetatable(a) == __complex_metatable__ then          -- 'a' is complex
    if getmetatable(b) == __complex_metatable__ then
      return CheckComplexCompatibility(a, b)
    else
      assert(type(b) == 'number' or vec.IsVector(b), 'Invalid complex number operation')
      return CheckComplexCompatibility(a, Complex(b))
    end
  else                                                  -- 'a' is not complex
    assert(getmetatable(b) == __complex_metatable__)
    return CheckComplexCompatibility(Complex(a), b)
  end
end

__complex_metatable__.__add = function(a, b)  -- the addition (+) operation
  a,b = CheckComplexArguments(a, b)
  return setmetatable({a[1]+b[1], a[2]+b[2], fn=a.fn}, __complex_metatable__)
end
__complex_metatable__.__sub = function(a, b)  -- the subtraction (-) operation
  a,b = CheckComplexArguments(a, b)
  return setmetatable({a[1]-b[1], a[2]-b[2], fn=a.fn}, __complex_metatable__)
end
__complex_metatable__.__mul = function(a, b)  -- the multiplication (*) operation
  a,b = CheckComplexArguments(a, b)
  return setmetatable({a[1]*b[1] - a[2]*b[2], a[2]*b[1] + a[1]*b[2], fn=a.fn}, __complex_metatable__)
end
__complex_metatable__.__div = function(a, b)  -- the division (/) operation
  a,b = CheckComplexArguments(a, b)
  local bm = b[1]*b[1] + b[2]*b[2]
  return setmetatable({(a[1]*b[1] + a[2]*b[2])/bm, (a[2]*b[1] - a[1]*b[2])/bm, fn=a.fn}, __complex_metatable__)
end
__complex_metatable__.__pow = function(a, b)  -- the exponentiation (^) operation
  a,b = CheckComplexArguments(a, b)
  local am = a[1]*a[1] + a[2]*a[2]
  -- If am == 0 this will fail as the complex power of zero is indeterminate.
  -- However we do not test for this case as 'am' could be a vector.
  local aa = a.fn.atan(a[2], a[1])
  local angle = aa*b[1] + b[2]/2*a.fn.log(am)
  local mag = am^(b[1]/2)*a.fn.exp(-aa*b[2])
  return setmetatable({mag*a.fn.cos(angle), mag*a.fn.sin(angle), fn=a.fn}, __complex_metatable__)
end
__complex_metatable__.__unm = function(a)  -- the negation (unary -) operation
  a = CheckComplexArguments(a, a)
  return setmetatable({-a[1],-a[2], fn=a.fn}, __complex_metatable__)
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

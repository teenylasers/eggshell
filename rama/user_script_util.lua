-- Lua utility functions that are available to user scripts.

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

-- Create a table that returns a zero for every index. This is sometimes useful
-- as arguments to config.optimize().
__ZeroTable__ = setmetatable({}, {__index = function(T,k) return 0 end })

-- The third argument to optimize, a table which contains field lookup
-- functions.
__Optimize3rdArg__ = {
  Complex = function(x, y)
    return _GetField(x, y)
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

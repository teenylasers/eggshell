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

function PrintVector(v)
  local s = ''
  for i = 1,#v do
    s = s .. string.format('%g ', v[i])
  end
  print('Vector:', s)
  return s
end

function Try(f, expected_status)
  local status,message = pcall(f)
  if not status then
    print(message)
  end
  assert(status == expected_status)
end

function TestVector()
  print 'Test vector functionality in lua:'

  -- Creation, sizing and assignment.
  v = Vector()
  assert(#v == 0)
  v:Resize(5)
  assert(#v == 5)
  assert(PrintVector(v) == '0 0 0 0 0 ')
  v[2] = 3      assert(PrintVector(v) == '0 3 0 0 0 ')
  v[5] = 9.1    assert(PrintVector(v) == '0 3 0 0 9.1 ')
  v[1] = -4.4   assert(PrintVector(v) == '-4.4 3 0 0 9.1 ')
  v:Resize(4)
  assert(#v == 4)
  assert(PrintVector(v) == '0 0 0 0 ')

  -- Types.
  assert(vec.IsVector(v))
  assert(not vec.IsVector(123))
  assert(not vec.IsVector("vector"))

  -- Arithmetic.
  a = Vector()
  b = Vector()
  a:Resize(4)
  b:Resize(4)
  for i = 1,4 do
    a[i] = 2 + i
    b[i] = 1 + i*2
  end
  assert(PrintVector(a) == '3 4 5 6 ')
  assert(PrintVector(b) == '3 5 7 9 ')
  c = -a      assert(PrintVector(c) == '-3 -4 -5 -6 ')
  c = a + b   assert(PrintVector(c) == '6 9 12 15 ')
  c = a - b   assert(PrintVector(c) == '0 -1 -2 -3 ')
  c = a * b   assert(PrintVector(c) == '9 20 35 54 ')
  c = a / b   assert(PrintVector(c) == '1 0.8 0.714286 0.666667 ')
  c = a ^ b   assert(PrintVector(c) == '27 1024 78125 1.00777e+07 ')

  c = a + 2   assert(PrintVector(c) == '5 6 7 8 ')
  c = a - 2   assert(PrintVector(c) == '1 2 3 4 ')
  c = a * 2   assert(PrintVector(c) == '6 8 10 12 ')
  c = a / 2   assert(PrintVector(c) == '1.5 2 2.5 3 ')
  c = a ^ 2   assert(PrintVector(c) == '9 16 25 36 ')

  c = 2 + a   assert(PrintVector(c) == '5 6 7 8 ')
  c = 2 - a   assert(PrintVector(c) == '-1 -2 -3 -4 ')
  c = 2 * a   assert(PrintVector(c) == '6 8 10 12 ')
  c = 2 / a   assert(PrintVector(c) == '0.666667 0.5 0.4 0.333333 ')
  c = 2 ^ a   assert(PrintVector(c) == '8 16 32 64 ')

  -- One-argument math functions.
  a:Resize(4)
  for i = 1,4 do
    a[i] = -2 - i
  end
  assert(PrintVector(a) == '-3 -4 -5 -6 ')
  assert(PrintVector(vec.abs(a)) == '3 4 5 6 ')
  for i = 1,4 do
    a[i] = i
  end
  assert(PrintVector(vec.acos(a / 10)) == '1.47063 1.36944 1.2661 1.15928 ')
  assert(PrintVector(vec.asin(a / 10)) == '0.100167 0.201358 0.304693 0.411517 ')
  assert(PrintVector(vec.atan(a / 10)) == '0.0996687 0.197396 0.291457 0.380506 ')
  assert(PrintVector(vec.ceil(a)) == '1 2 3 4 ')
  assert(PrintVector(vec.ceil(a + 0.1)) == '2 3 4 5 ')
  assert(PrintVector(vec.cos(a)) == '0.540302 -0.416147 -0.989992 -0.653644 ')
  assert(PrintVector(vec.exp(a)) == '2.71828 7.38906 20.0855 54.5982 ')
  assert(PrintVector(vec.floor(a)) == '1 2 3 4 ')
  assert(PrintVector(vec.floor(a + 0.1)) == '1 2 3 4 ')
  assert(PrintVector(vec.floor(a - 0.1)) == '0 1 2 3 ')
  assert(PrintVector(vec.log(a)) == '0 0.693147 1.09861 1.38629 ')
  assert(PrintVector(vec.log10(a)) == '0 0.30103 0.477121 0.60206 ')
  assert(PrintVector(vec.sin(a)) == '0.841471 0.909297 0.14112 -0.756802 ')
  assert(PrintVector(vec.sqrt(a)) == '1 1.41421 1.73205 2 ')
  assert(PrintVector(vec.tan(a)) == '1.55741 -2.18504 -0.142547 1.15782 ')

  -- Two-argument math functions.
  a:Resize(4)
  b:Resize(4)
  for i = 1,4 do
    a[i] = i - 1.5
    b[i] = i*i
  end
  assert(PrintVector(vec.atan(a,b)) == '-0.463648 0.124355 0.165149 0.154997 ')
  assert(PrintVector(vec.atan2(a,b)) == '-0.463648 0.124355 0.165149 0.154997 ')
  for i = 1,4 do
    a[i] = i*5
    b[i] = i*3
  end
  assert(PrintVector(vec.fmod(a,b)) == '2 4 6 8 ')

  -- Vector comparison functions.
  a:Resize(9)     -- a = 1 1 1 2 2 2 3 3 3
  b:Resize(9)     -- b = 1 2 3 1 2 3 1 2 3
  for i = 1,3 do
    for j = 1,3 do
      a[i*3+j-3] = i
      b[i*3+j-3] = j
    end
  end
  assert(PrintVector(vec.eq(a,b)) == '1 0 0 0 1 0 0 0 1 ')
  assert(PrintVector(vec.ne(a,b)) == '0 1 1 1 0 1 1 1 0 ')
  assert(PrintVector(vec.lt(a,b)) == '0 1 1 0 0 1 0 0 0 ')
  assert(PrintVector(vec.ge(a,b)) == '1 0 0 1 1 0 1 1 1 ')
  assert(PrintVector(vec.gt(a,b)) == '0 0 0 1 0 0 1 1 0 ')
  assert(PrintVector(vec.le(a,b)) == '1 1 1 0 1 1 0 0 1 ')

  -- Dot product.
  a:Resize(4)
  b:Resize(4)
  a[1] = 1 a[2] = 2 a[3] = 3 a[4] = 0
  b[1] = 5 b[2] = 4 b[3] = 3 b[4] = 9
  assert(a:dot(b) == 5+8+9)

  -- Cross product.
  a:Resize(3)
  b:Resize(3)
  a[1] = 5 a[2] = 7 a[3] = -3
  b[1] = 1 b[2] = -5 b[3] = 8
  c = a:cross(b)
  assert(vec.IsVector(c) and c[1] == 41 and c[2] == -43 and c[3] == -32)
  a:Resize(2)
  b:Resize(2)
  a[1] = 5 a[2] = 7
  b[1] = 1 b[2] = -5
  c = a:cross(b)
  assert(type(c) == 'number' and c == -32)

  -- Length.
  a:Resize(2)
  a[1] = 3 a[2] = 4
  assert(a.length == 5);

  -- Normalization.
  a:Resize(2)
  a[1] = 3 a[2] = 4
  a = a.normalized
  assert(vec.IsVector(a) and a[1] == 3/5 and a[2] == 4/5)

  -- Initialization.
  a = vec.New(1, 2, 3, 4)
  assert(vec.IsVector(a) and #a == 4 and a[1] == 1 and a[2] == 2 and a[3] == 3 and a[4] == 4)

  -- Check that incorrect indexes generate errors.
  a:Resize(4)
  Try(function() print(a[2]) end, true)
  Try(function() print(a[5]) end, false)
  Try(function() print(a[-1]) end, false)
  Try(function() print(a[1.1]) end, false)
  Try(function() print(a['foo']) end, false)
  Try(function() print(a[{}]) end, false)
  Try(function() print(a[function() end]) end, false)
  Try(function() a[2] = 1 end, true)
  Try(function() a[5] = 1 end, false)
  Try(function() a[-1] = 1 end, false)
  Try(function() a[1.1] = 1 end, false)

  -- Check that bad arguments to operators generate errors.
  a:Resize(4)
  b:Resize(3)
  Try(function() c = a + a end, true)
  Try(function() c = a + b end, false)
  Try(function() c = a + 'foo' end, false)
  Try(function() c = 'foo' + a end, false)

  -- Check that bad arguments to other functions generate errors.
  a:Resize(4)
  b:Resize(3)
  Try(function() c = vec.abs(a) end, true)
  Try(function() c = vec.abs(1) end, false)
  Try(function() c = vec.abs('foo') end, false)
  Try(function() c = vec.abs() end, false)
  Try(function() c = vec.abs(a,b) end, false)
  Try(function() a:Resize(-1) end, false)
  Try(function() a:Resize(2.1) end, false)
  Try(function() a:Resize(0) end, true)
  Try(function() a:Resize(4) end, true)
  Try(function() vec.atan(a,a,a) end, false)
  Try(function() vec.atan() end, false)
  Try(function() vec.atan(a,b) end, false)
  Try(function() vec.atan2(a,a,a) end, false)
  Try(function() vec.atan2(a) end, false)
  Try(function() vec.atan2() end, false)
  Try(function() vec.atan2(a,b) end, false)
  Try(function() a:dot() end, false)
  Try(function() a:dot(a,a) end, false)
  Try(function() a:dot(1) end, false)
  Try(function() a:dot(b) end, false)
  a:Resize(2)
  b:Resize(3)
  Try(function() a:cross(b) end, false)
  Try(function() b:cross(a) end, false)
  Try(function() a:cross(1) end, false)
  Try(function() b:cross(1) end, false)

  print 'Success'
end

-- A dummy configuration.
config = {
  type = 'Ez',
  unit = 'm',
  mesh_edge_length = 0.1,
  excited_port = 1,
  frequency = 60e9,
  depth = 0,
  cd = Rectangle(0, 0, 1 ,1),
  test = TestVector,
}

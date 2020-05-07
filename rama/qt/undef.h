// Rama Simulator, Copyright (C) 2014-2020 Russell Smith.
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.

// Undefine some common words that are defined by windows header files. These
// things leak into our code after including some Qt headers, and they conflict
// with some of our own variable names (notably 'near' and 'far' which are
// defined to the empty string by windef.h, a relic from the segmented memory
// era).
//
// We got this list by compiling this line
//   #include <QOpenGLFunctions_3_3_Core>
// with
//   x86_64-w64-mingw32-g++ -I Qt-5.9.2/include -I Qt-5.9.2/include/QtWidgets
//     -I Qt-5.9.2/include/QtGui -I Qt-5.9.2/include/QtCore
//     -I Qt-5.9.2/mkspecs/win32-g++ that_line.cc -E -dM
// and then filtering the output with
//     grep 'define [a-z][a-z_0-9]* '
// to look for lower case macros. Some standard library macros and Qt keywords
// (slots, etc) were removed from the undef list.

#ifdef __WINNT__

#undef edt1
#undef edt2
#undef edt3
#undef edt4
#undef edt5
#undef edt8
#undef edt9
#undef exception_code
#undef rpc_binding_vector_t
#undef grp1
#undef grp2
#undef grp3
#undef scr1
#undef scr2
#undef scr4
#undef scr5
#undef scr6
#undef scr7
#undef frm2
#undef psh11
#undef cmb5
#undef ico2
#undef ico3
#undef ico4
#undef chx10
#undef chx11
#undef chx12
#undef chx13
#undef chx14
#undef chx15
#undef chx16
#undef rad11
#undef rad12
#undef environ
#undef wcswcs
#undef cmb10
#undef chx3
#undef chx8
#undef chx9
#undef rad5
#undef rad8
#undef rad9
#undef stc8
#undef edt10
#undef edt11
#undef edt12
#undef edt13
#undef edt14
#undef edt15
#undef edt16
#undef stc12
#undef stc13
#undef stc19
#undef stc23
#undef edt6
#undef edt7
#undef rpc_binding_handle_t
#undef psh10
#undef psh12
#undef psh13
#undef psh14
#undef psh16
#undef grp4
#undef abnormal_termination
#undef rad10
#undef rad13
#undef rad14
#undef rad15
#undef chx1
#undef chx2
#undef chx4
#undef chx5
#undef chx6
#undef chx7
#undef lst1
#undef lst2
#undef lst3
#undef lst5
#undef lst6
#undef lst8
#undef lst9
#undef pascal
#undef far
#undef hyper
#undef frm3
#undef interface
#undef stc1
#undef stc2
#undef stc3
#undef stc5
#undef stc6
#undef stc7
#undef stc9
#undef near
#undef midl_user_free
#undef cmb1
#undef cmb2
#undef cmb3
#undef cmb4
#undef cmb6
#undef cmb7
#undef cmb8
#undef cmb9
#undef rad1
#undef rad2
#undef rad3
#undef rad4
#undef rad6
#undef rad7
#undef psh15
#undef h_addr
#undef psh1
#undef psh2
#undef psh3
#undef psh4
#undef psh5
#undef psh6
#undef psh7
#undef psh8
#undef psh9
#undef exception_info
#undef frm1
#undef frm4
#undef h_errno
#undef emit
#undef rct1
#undef rct2
#undef rct3
#undef rct4
#undef cdecl
#undef stc10
#undef stc11
#undef stc14
#undef stc15
#undef stc16
#undef stc17
#undef stc18
#undef stc20
#undef stc21
#undef stc22
#undef stc24
#undef stc27
#undef stc28
#undef stc29
#undef stc30
#undef stc31
#undef stc32
#undef rad16
#undef stc25
#undef stc26
#undef lst4
#undef lst7
#undef cmb11
#undef cmb12
#undef cmb13
#undef cmb14
#undef cmb15
#undef cmb16
#undef ico1
#undef lst10
#undef lst11
#undef lst12
#undef lst13
#undef lst14
#undef lst15
#undef lst16
#undef stc4
#undef ctl1
#undef scr3
#undef scr8
#undef midl_user_allocate

#endif  // __WINNT__

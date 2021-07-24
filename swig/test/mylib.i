%module "mylib"
%{
#include "../../lib/test/myclass.h"
%}
%include "std_string.i"

// ===== Pointer Sections =====
// Option 1: using pointer library
%include <cpointer.i>
%pointer_functions(int, intp);


// Option 2: using typemaps
// %include <typemaps.i>
// //void Xarm::run(int *INOUT);
// %apply int *OUTPUT {int *num};


// ===== Array Section =====
// %include <arrays_javascript.i>
// %inline %{
//          extern int FiddleSticks[2];
// %}
// %include "carrays.i"
// %array_class(double, doubleArray);

%include "../../lib/test/myclass.h"

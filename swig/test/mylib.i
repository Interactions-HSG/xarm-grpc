%module "mylib"
%{
#include "../../lib/test/myclass.h"
%}
%include "std_string.i"
%include "../../lib/test/myclass.h"

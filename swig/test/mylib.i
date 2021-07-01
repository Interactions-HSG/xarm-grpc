%module "mylib"
%{
#include "../../lib/test/myclass.h"
%}
%include "../../lib/test/myclass.h"

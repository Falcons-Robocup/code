%module EnvironmentField
%include <std_string.i>
%include <std_vector.i>

%template(StringVector) std::vector< std::string >;

%typemap(inout) poiInfo & fieldPOI { $result=fieldPOI }
%typemap(inout) areaInfo & fieldArea { $result=fieldArea }

%{
#define SWIG_FILE_WITH_INIT  // specify that the resulting C file should be build as python extension, inserting the module init code
#include <ext/cEnvironmentCommon.hpp>
#include <ext/cEnvironmentField.hpp>

%}


%include "ext/cEnvironmentCommon.hpp"
%include "ext/cEnvironmentField.hpp"




function result = isoctave()
% portability: determine if we are running in octave or Matlab
% useful for expectation management in test suite


result = exist('OCTAVE_VERSION', 'builtin') ~= 0;


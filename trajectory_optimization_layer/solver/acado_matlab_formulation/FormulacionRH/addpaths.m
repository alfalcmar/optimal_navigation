% This script is necessary to run in every problem before
% using BEGIN_ACADO. In short, it includes the paths from the
% ACADOtoolkit and puts it in the workspace


% Go to your ACADOtoolkit pwd
cd ~/ACADOtoolkit/interfaces/matlab/;


% Add the paths (you don't need to change this)
% Taken from section 1.3.1 (Linux):
% http://acado.sourceforge.net/doc/pdf/acado_matlab_manual.pdf
path1 = addpath(genpath([pwd filesep 'bin']));
path2 = addpath(genpath([pwd filesep 'shared']));
path3 = addpath(genpath([pwd filesep 'integrator']));
path4 = addpath(genpath([pwd filesep 'acado']));
path5 = addpath(genpath([pwd filesep 'acado' filesep 'functions']));
path6 = addpath(genpath([pwd filesep 'acado' filesep 'packages']));

% path7 = addpath /home/damian/ACADOtoolkit/interfaces/matlab/acado/packages/+acado'
% Note: it's was necessary to edit some formulation to make it
% work on MATLAB R2020b


% Go back to the folder where you are working
cd ~/MATLAB/bin/ACADO/FormulacionRH
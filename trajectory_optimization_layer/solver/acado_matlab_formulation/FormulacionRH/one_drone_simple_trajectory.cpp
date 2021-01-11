/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState px_;
    DifferentialState py_;
    DifferentialState pz_;
    DifferentialState vx_;
    DifferentialState vy_;
    DifferentialState vz_;
    Control ax_;
    Control ay_;
    Control az_;
    Function acadodata_f2;
    acadodata_f2 << px_;
    acadodata_f2 << py_;
    acadodata_f2 << pz_;
    DVector acadodata_v1(3);
    acadodata_v1(0) = 1.206136E+01;
    acadodata_v1(1) = 3.503932E+01;
    acadodata_v1(2) = 9.013633E+00;
    DMatrix acadodata_M1;
    acadodata_M1.read( "one_drone_simple_trajectory_data_acadodata_M1.txt" );
    Function acadodata_f3;
    acadodata_f3 << ax_;
    acadodata_f3 << ay_;
    acadodata_f3 << az_;
    DMatrix acadodata_M2;
    acadodata_M2.read( "one_drone_simple_trajectory_data_acadodata_M2.txt" );
    DVector acadodata_v2(3);
    acadodata_v2(0) = -6.135320E-04;
    acadodata_v2(1) = -3.910449E-04;
    acadodata_v2(2) = -1.365785E-04;
    DVector acadodata_v3(3);
    acadodata_v3(0) = -6.135320E-04;
    acadodata_v3(1) = -3.910449E-04;
    acadodata_v3(2) = -1.365785E-04;
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(px_) == vx_;
    acadodata_f1 << dot(py_) == vy_;
    acadodata_f1 << dot(pz_) == vz_;
    acadodata_f1 << dot(vx_) == ax_;
    acadodata_f1 << dot(vy_) == ay_;
    acadodata_f1 << dot(vz_) == az_;

    OCP ocp1(38.4, 40, 8);
    ocp1.minimizeLSQ(acadodata_M2, acadodata_f3, acadodata_v3);
    ocp1.minimizeLSQEndTerm(acadodata_M1, acadodata_f2, acadodata_v1);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo((-5.00000000000000000000e+00) <= ax_ <= 5.00000000000000000000e+00);
    ocp1.subjectTo((-5.00000000000000000000e+00) <= ay_ <= 5.00000000000000000000e+00);
    ocp1.subjectTo((-5.00000000000000000000e+00) <= az_ <= 5.00000000000000000000e+00);
    ocp1.subjectTo((-1.50000000000000000000e+01) <= vx_ <= 1.50000000000000000000e+01);
    ocp1.subjectTo((-1.50000000000000000000e+01) <= vy_ <= 1.50000000000000000000e+01);
    ocp1.subjectTo((-1.50000000000000000000e+01) <= vz_ <= 1.50000000000000000000e+01);
    ocp1.subjectTo((-1.50000000000000000000e+01) <= vz_ <= 1.50000000000000000000e+01);
    ocp1.subjectTo(AT_START, px_ == 1.44584517542496406861e+01);
    ocp1.subjectTo(AT_START, py_ == 3.46970194737233725846e+01);
    ocp1.subjectTo(AT_START, pz_ == 9.15346677639260164483e+00);
    ocp1.subjectTo(AT_START, vx_ == (-1.44822947474837726567e+00));
    ocp1.subjectTo(AT_START, vy_ == 2.47808517381930132339e-01);
    ocp1.subjectTo(AT_START, vz_ == (-7.60401747495855484527e-02));
    ocp1.subjectTo(AT_START, ax_ == (-1.95593282563294758314e-02));
    ocp1.subjectTo(AT_START, ay_ == (-1.15945585239785025483e-02));
    ocp1.subjectTo(AT_START, az_ == (-4.21365680629052098000e-03));
    ocp1.subjectTo((pow((-1.20000000000000000000e+01+px_),2.00000000000000000000e+00)+pow((-1.60000000000000000000e+01+py_),2.00000000000000000000e+00)) >= 8.10000000000000000000e+01);


    OptimizationAlgorithm algo1(ocp1);
    algo1.set( KKT_TOLERANCE, 1.000000E-04 );
    algo1.set( INTEGRATOR_TOLERANCE, 1.000000E-01 );
    algo1.set( RELAXATION_PARAMETER, 3.500000E+00 );
    returnValue returnvalue = algo1.solve();

    VariablesGrid out_states; 
    VariablesGrid out_parameters; 
    VariablesGrid out_controls; 
    VariablesGrid out_disturbances; 
    VariablesGrid out_algstates; 
    algo1.getDifferentialStates(out_states);
    algo1.getControls(out_controls);
    const char* outputFieldNames[] = {"STATES", "CONTROLS", "PARAMETERS", "DISTURBANCES", "ALGEBRAICSTATES", "CONVERGENCE_ACHIEVED"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,6,outputFieldNames ); 
    mxArray *OutS = NULL;
    double  *outS = NULL;
    OutS = mxCreateDoubleMatrix( out_states.getNumPoints(),1+out_states.getNumValues(),mxREAL ); 
    outS = mxGetPr( OutS );
    for( int i=0; i<out_states.getNumPoints(); ++i ){ 
      outS[0*out_states.getNumPoints() + i] = out_states.getTime(i); 
      for( int j=0; j<out_states.getNumValues(); ++j ){ 
        outS[(1+j)*out_states.getNumPoints() + i] = out_states(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES",OutS );
    mxArray *OutC = NULL;
    double  *outC = NULL;
    OutC = mxCreateDoubleMatrix( out_controls.getNumPoints(),1+out_controls.getNumValues(),mxREAL ); 
    outC = mxGetPr( OutC );
    for( int i=0; i<out_controls.getNumPoints(); ++i ){ 
      outC[0*out_controls.getNumPoints() + i] = out_controls.getTime(i); 
      for( int j=0; j<out_controls.getNumValues(); ++j ){ 
        outC[(1+j)*out_controls.getNumPoints() + i] = out_controls(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"CONTROLS",OutC );
    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( out_parameters.getNumPoints(),1+out_parameters.getNumValues(),mxREAL ); 
    outP = mxGetPr( OutP );
    for( int i=0; i<out_parameters.getNumPoints(); ++i ){ 
      outP[0*out_parameters.getNumPoints() + i] = out_parameters.getTime(i); 
      for( int j=0; j<out_parameters.getNumValues(); ++j ){ 
        outP[(1+j)*out_parameters.getNumPoints() + i] = out_parameters(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"PARAMETERS",OutP );
    mxArray *OutW = NULL;
    double  *outW = NULL;
    OutW = mxCreateDoubleMatrix( out_disturbances.getNumPoints(),1+out_disturbances.getNumValues(),mxREAL ); 
    outW = mxGetPr( OutW );
    for( int i=0; i<out_disturbances.getNumPoints(); ++i ){ 
      outW[0*out_disturbances.getNumPoints() + i] = out_disturbances.getTime(i); 
      for( int j=0; j<out_disturbances.getNumValues(); ++j ){ 
        outW[(1+j)*out_disturbances.getNumPoints() + i] = out_disturbances(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"DISTURBANCES",OutW );
    mxArray *OutZ = NULL;
    double  *outZ = NULL;
    OutZ = mxCreateDoubleMatrix( out_algstates.getNumPoints(),1+out_algstates.getNumValues(),mxREAL ); 
    outZ = mxGetPr( OutZ );
    for( int i=0; i<out_algstates.getNumPoints(); ++i ){ 
      outZ[0*out_algstates.getNumPoints() + i] = out_algstates.getTime(i); 
      for( int j=0; j<out_algstates.getNumValues(); ++j ){ 
        outZ[(1+j)*out_algstates.getNumPoints() + i] = out_algstates(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"ALGEBRAICSTATES",OutZ );
    mxArray *OutConv = NULL;
    if ( returnvalue == SUCCESSFUL_RETURN ) { OutConv = mxCreateDoubleScalar( 1 ); }else{ OutConv = mxCreateDoubleScalar( 0 ); } 
    mxSetField( plhs[0],0,"CONVERGENCE_ACHIEVED",OutConv );


    clearAllStaticCounters( ); 
 
} 


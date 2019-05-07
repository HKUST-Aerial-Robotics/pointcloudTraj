#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

#include "pointcloudTraj/mosek.h"
#include "pointcloudTraj/bezier_base.h"
#include "pointcloudTraj/traj_optimizer.h"

using namespace std;    
using namespace Eigen;

static void MSKAPI printstr(void *handle, MSKCONST char str[])
{
  printf("%s",str);
} /* printstr */

TrajectoryOptimizerSOCP::TrajectoryOptimizerSOCP(){}

TrajectoryOptimizerSOCP::~TrajectoryOptimizerSOCP(){}

MatrixXd TrajectoryOptimizerSOCP::BezierConicOptimizer(
            const MatrixXd  &corridor_centers,
            const VectorXd  &corridor_radius,
            const VectorXd  &corridor_times,
            const vector<int>      & poly_orders,
            const vector<MatrixXd> & FMs,
            const MatrixXd &pos,
            const MatrixXd &vel,
            const MatrixXd &acc,
            const double max_vel,
            const double max_acc,
            const int minimize_order,
            const bool is_limit_vel,
            const bool is_limit_acc,
            const bool is_print_soving )  // define the order to which we minimize.   1 -- velocity, 2 -- acceleration, 3 -- jerk, 4 -- snap  
{   
    MatrixXd PolyCoeff;

    double init_scale  = corridor_times.head(1)(0);
    double last_scale  = corridor_times.tail(1)(0);
    int    segment_num = corridor_times.size();

    Vector3d start_pos = pos.row(0); 
    Vector3d start_vel = vel.row(0); 
    Vector3d start_acc = acc.row(0);
    Vector3d end_pos   = pos.row(1); 
    Vector3d end_vel   = vel.row(1);
    Vector3d end_acc   = acc.row(1);

    //Prepare for constaints and objective data stucture for Mosek solver .         
    int s1_d1_ctrl_num, s1_ctrl_num;

    int equ_con_s_num = 3 * 3; // p, v, a in x, y, z axis at the start point
    int equ_con_e_num = 3 * 3; // p, v, a in x, y, z axis at the end point
    int equ_con_continuity_num = 3 * 3 * (segment_num - 1);
    int equ_con_num   = equ_con_s_num + equ_con_e_num + equ_con_continuity_num; // p, v, a in x, y, z axis in each segment's joint position

    int inequ_con_num  = 0;
    int vel_con_num    = 0; 
    int acc_con_num    = 0; 
    int total_ctrl_num = 0;
    int obj_nzero_num  = 0;

    for(int i = 0; i < segment_num; i++) {   
        int traj_order  = poly_orders[i];
        s1_d1_ctrl_num  = traj_order + 1;
        inequ_con_num   += s1_d1_ctrl_num;
        total_ctrl_num  += s1_d1_ctrl_num * 3;
        obj_nzero_num   +=(s1_d1_ctrl_num - minimize_order) * 3; 
        
        vel_con_num +=  traj_order * 3;
        acc_con_num += (traj_order - 1) * 3;
    }

    if( !is_limit_vel )
        vel_con_num = 0;

    if( !is_limit_acc )
        acc_con_num = 0;

    int high_order_con_num = vel_con_num + acc_con_num; 

    int vel_var_num = vel_con_num;
    int acc_var_num = acc_con_num;
    int high_order_var_num = vel_var_num + acc_var_num; 

    // additional equality constraints introduced by the reformulation of all quadratic constraints
    // each quadratic ==> 1 for u - c'x = w, 3 for y = Fx
    // the objective  ==> obj_nzero_num for y = Fx
    int equ_con_extra_num = inequ_con_num + inequ_con_num * 3 + obj_nzero_num; 

    // additional functional variables introduced by the reformulation of all quadratic constraints
    // each quadratic ==> 1 for t, 1 for w, 3 for y = Fx ... objective ==> 1 w, several y
    // we should calculate non-zeros in FM, for introducing extra y variables
    int var_extra_obj_num = 1             + 1             + obj_nzero_num;
    int var_extra_qua_num = inequ_con_num + inequ_con_num + inequ_con_num * 3;
    int var_extra_num     = var_extra_qua_num + var_extra_obj_num; 

    int var_y_con = inequ_con_num * 3;
    int var_y_obj = obj_nzero_num;
    int var_y_num = var_y_obj + var_y_con;

    int var_w_num = 1 + inequ_con_num; // w for replacing all quadratic terms by linear terms
    int var_t_num = 1 + inequ_con_num; // t= 1, used in the conic cones

    assert( var_extra_num == var_t_num + var_y_num + var_w_num );
    assert( inequ_con_num * 3 == total_ctrl_num );

    int con_num = equ_con_num    + equ_con_extra_num + high_order_con_num;
    int var_num = total_ctrl_num + var_extra_num     + high_order_var_num; // another way is by introducing extra linear qualities and middle variables

    double x_var[var_num];
    MSKrescodee  r; 
    double primalobj;

    /* ## define a container for constaints boundary and boundkey ## */ 
    /* ## dataType in the double pair is : boundary type, lower bound, upper bound ## */
    vector< pair<MSKboundkeye, pair<double, double> > > con_bdk; 

    /*** Here is the most important stuff, we will elliminate all quadratical constraints by introducing many equality constraints and more additional variables ***/
    /*************************************************************************************************/
    /** Now firstly we omit the reformulation of the quadratic part in the objective **/
    for(int i = 0; i < equ_con_num; i ++ ) { 
        double beq_i;
        if(i < 3)                    beq_i = start_pos(i); 
        else if (i >= 3  && i < 6  ) beq_i = start_vel(i - 3); 
        else if (i >= 6  && i < 9  ) beq_i = start_acc(i - 6);
        else if (i >= 9  && i < 12 ) beq_i = end_pos (i - 9 );
        else if (i >= 12 && i < 15 ) beq_i = end_vel(i - 12);
        else if (i >= 15 && i < 18 ) beq_i = end_acc(i - 15);
        else beq_i = 0.0;

        pair<MSKboundkeye, pair<double, double> > cb_eq = make_pair( MSK_BK_FX, make_pair( beq_i, beq_i ) ); // # cb_eq means: constriants boundary of equality constrain
        con_bdk.push_back(cb_eq);
    }

    /***  Stack the bounding value for equality constraints induced by the corridor constraints  ***/
    for(int k = 0; k < segment_num; k++ ) {
        s1_d1_ctrl_num = poly_orders[k] + 1;
        for(int i = 0; i < s1_d1_ctrl_num; i++) {
            double bin_i = corridor_radius(k) * corridor_radius(k) 
            - corridor_centers(k, 0) * corridor_centers(k, 0) - corridor_centers(k, 1) * corridor_centers(k, 1) - corridor_centers(k, 2) * corridor_centers(k, 2) ;            
            pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair( MSK_BK_FX, make_pair( bin_i, bin_i ) ); 
            con_bdk.push_back(cb_ie);   
        }
    }

    /***  Stack the bounding value for mapping Fx to additional variables y  ***/
    for(int i = 0; i < var_y_num; i++)
    {
        pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair( MSK_BK_FX, make_pair( 0.0, 0.0 ) ); // # cb_ie means: constriants boundary of inequality constrain      
        con_bdk.push_back(cb_ie);   
    }

if( is_limit_vel )
{
    /***  Stack the bounding value for the linear inequality for the velocity constraints  ***/
    // for velocity constraints
    for(int i = 0; i < vel_con_num; i++) {
        pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair( MSK_BK_FX, make_pair( 0.0, 0.0) );
        con_bdk.push_back(cb_ie);   
    }
}

if( is_limit_acc )
{
    /***  Stack the bounding value for the linear inequality for the velocity constraints  ***/
    // for velocity constraints
    for(int i = 0; i < acc_con_num; i++) {
        pair<MSKboundkeye, pair<double, double> > cb_ie = make_pair( MSK_BK_FX, make_pair( 0.0, 0.0) );
        con_bdk.push_back(cb_ie);   
    }
}

    /*** ## Stacking bounds for all unknowns ## ***/ 
    /*** The sequence is control points + additional variables: x, w, y, t ***/
    vector< pair<MSKboundkeye, pair<double, double> > > var_bdk; 
    for(int i = 0; i < total_ctrl_num; i ++ ) {
        pair<MSKboundkeye, pair<double, double> > vb_x  = make_pair( MSK_BK_FR, make_pair( - MSK_INFINITY, + MSK_INFINITY ) ); 
        var_bdk.push_back(vb_x);
    } 

    /* ## Variable bounds for addtional variables y and w ## */ 
    for(int i = 0; i < (var_y_num + var_w_num); i ++ ) {
        pair<MSKboundkeye, pair<double, double> > vb_x  = make_pair( MSK_BK_FR, make_pair( - MSK_INFINITY, + MSK_INFINITY ) );
        var_bdk.push_back(vb_x);
    }

    /* ## Variable bounds for addtional variables t, all pre-set as 1 ## */ 
    for(int i = 0; i < var_t_num; i ++ ) {
        pair<MSKboundkeye, pair<double, double> > vb_x  = make_pair( MSK_BK_FX, make_pair( 1.0, 1.0 ) ); 
        var_bdk.push_back(vb_x);
    }

if ( is_limit_vel )
{
    for(int i = 0; i < vel_var_num; i ++ ) {
        pair<MSKboundkeye, pair<double, double> > vb_x  = make_pair( MSK_BK_RA, make_pair( - max_vel, + max_vel ) ); 
        var_bdk.push_back(vb_x);
    } 
}

if ( is_limit_acc )
{   
    for(int i = 0; i < acc_var_num; i ++ ) {
        pair<MSKboundkeye, pair<double, double> > vb_x  = make_pair( MSK_BK_RA, make_pair( - max_acc, + max_acc ) ); 
        var_bdk.push_back(vb_x);
    } 
}

    MSKint32t  j,i; 
    MSKenv_t   env; 
    MSKtask_t  task; 
    r = MSK_makeenv( &env, NULL );   // Create the mosek environment. 
    r = MSK_maketask(env,con_num, var_num, &task); // Create the optimization task. 

    // Parameters used in the optimizer
    MSK_putintparam (task, MSK_IPAR_OPTIMIZER , MSK_OPTIMIZER_CONIC );
    MSK_putintparam (task, MSK_IPAR_NUM_THREADS, 1);
    MSK_putdouparam (task, MSK_DPAR_CHECK_CONVEXITY_REL_TOL, 1e-5);
    MSK_putdouparam (task, MSK_DPAR_INTPNT_CO_TOL_DFEAS,   1e-4);
    MSK_putdouparam (task, MSK_DPAR_INTPNT_CO_TOL_PFEAS,   1e-4);
    MSK_putdouparam (task, MSK_DPAR_INTPNT_CO_TOL_INFEAS,  1e-4);
    MSK_putdouparam (task, MSK_DPAR_INTPNT_CO_TOL_REL_GAP, 1e-5);
    //MSK_putdouparam (task, MSK_DPAR_INTPNT_CO_TOL_MU_RED, 1e-10);
    
    // Turn on the printing of the Conic solver
    if ( is_print_soving )
        r = MSK_linkfunctotaskstream(task,MSK_STREAM_LOG, NULL, printstr); 

    // Append 'con_num' empty constraints. 
    //The constraints will initially have no bounds. 
    if ( r == MSK_RES_OK ) 
        r = MSK_appendcons(task, con_num);  

    // Append 'var_num' variables. The variables will initially be fixed at zero (x=0). 
    if ( r == MSK_RES_OK ) 
        r = MSK_appendvars(task, var_num); 

    for(j = 0; j<var_num && r == MSK_RES_OK; ++j) { 
        // Set the bounds of variable j : //  blx[j] <= x_j <= bux[j] 
        if (r == MSK_RES_OK) 
            r = MSK_putvarbound(task, 
                                j,                            // Index of variable. 
                                var_bdk[j].first,             // Bound key.
                                var_bdk[j].second.first,      // Numerical value of lower bound.
                                var_bdk[j].second.second );   // Numerical value of upper bound.      
    } 
    
    // Set the bounds on constraints. 
    //   for i=1, ...,con_num : blc[i] <= constraint i <= buc[i] 
    for( i = 0; i < con_num && r == MSK_RES_OK; i++ ) {
        r = MSK_putconbound(task, 
                            i,                            // Index of constraint. 
                            con_bdk[i].first,             // Bound key.
                            con_bdk[i].second.first,      // Numerical value of lower bound.
                            con_bdk[i].second.second );   // Numerical value of upper bound. 
    }

    // #1   put the equality constraints in the start position
    // #1.1 positon constraints in the start point
    // #1.2 velocity constraints in the start point
    // #1.3 acceleration constraints in the start point
    // For position, velocity and acceleration, seperately
    
    int row_idx = 0;
    /*   Initial state  */
    {
        int order = poly_orders[0];
        s1_d1_ctrl_num = order + 1;
        // position :
        for(int i = 0; i < 3; i++) {  // loop for x, y, z       
            int nzi = 1;
            MSKint32t asub[nzi];
            double aval[nzi];
            asub[0] = i * s1_d1_ctrl_num;
            aval[0] = 1.0 * init_scale;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);    
            row_idx ++;
        }
        // velocity :
        for(int i = 0; i < 3; i++) {  // loop for x, y, z       
            int nzi = 2;
            MSKint32t asub[nzi];
            double aval[nzi];
            asub[0] = i * s1_d1_ctrl_num;
            asub[1] = i * s1_d1_ctrl_num + 1;
            aval[0] = - 1.0 * order;
            aval[1] =   1.0 * order;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);   
            row_idx ++;
        }
        // acceleration : 
        for(int i = 0; i < 3; i++) {  // loop for x, y, z       
            int nzi = 3;
            MSKint32t asub[nzi];
            double aval[nzi];
            asub[0] = i * s1_d1_ctrl_num;
            asub[1] = i * s1_d1_ctrl_num + 1;
            asub[2] = i * s1_d1_ctrl_num + 2;
            aval[0] =   1.0 * order * (order - 1) / init_scale;
            aval[1] = - 2.0 * order * (order - 1) / init_scale;
            aval[2] =   1.0 * order * (order - 1) / init_scale;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);    
            row_idx ++;
        }
    }

    // #2   put the equality constraints in the end position
    // #2.1 positon constraints in the end point
    // #2.2 velocity constraints in the end point
    // #2.3 acceleration constraints in the end point

    /*   Final state  */
    {   
        int order = poly_orders[segment_num - 1];
        s1_d1_ctrl_num = order + 1;
        // position :
        for(int i = 0; i < 3; i++) {  // loop for x, y, z       
            int nzi = 1;
            MSKint32t asub[nzi];
            double aval[nzi];
            asub[0] = total_ctrl_num - 1 - (2 - i) * s1_d1_ctrl_num;
            aval[0] = 1.0 * last_scale;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);    
            row_idx ++;
        }
        // velocity :
        for(int i = 0; i < 3; i++) {  // loop for x, y, z       
            int nzi = 2;
            MSKint32t asub[nzi];
            double aval[nzi];
            asub[0] = total_ctrl_num - 1 - (2 - i) * s1_d1_ctrl_num - 1;
            asub[1] = total_ctrl_num - 1 - (2 - i) * s1_d1_ctrl_num;
            aval[0] = - 1.0;
            aval[1] =   1.0;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);    
            row_idx ++;
        }
        // acceleration : 
        for(int i = 0; i < 3; i++) {  // loop for x, y, z       
            int nzi = 3;
            MSKint32t asub[nzi];
            double aval[nzi];
            asub[0] = total_ctrl_num - 1 - (2 - i) * s1_d1_ctrl_num - 2;
            asub[1] = total_ctrl_num - 1 - (2 - i) * s1_d1_ctrl_num - 1;
            asub[2] = total_ctrl_num - 1 - (2 - i) * s1_d1_ctrl_num;
            aval[0] =   1.0 / last_scale;
            aval[1] = - 2.0 / last_scale;
            aval[2] =   1.0 / last_scale;
            r = MSK_putarow(task, row_idx, nzi, asub, aval);    
            row_idx ++;
        }
    }

    // #3   put the equality coinstraints in each joint between two pices of the curve 
    // #3.1 positon constraints in each joint 
    // #3.2 velocity constraints in each joint 
    // #3.3 acceleration constraints in each joint 

    /*   continuity constraints  */
    {
        int sub_shift = 0;
        double val0, val1;
        for(int k = 0; k < (segment_num - 1); k ++ ) {
            int order = poly_orders[k];
            s1_d1_ctrl_num = order + 1;
            s1_ctrl_num = 3 * s1_d1_ctrl_num;

            int order_next = poly_orders[k + 1];
            int _s1d1CtrlP_next_num = order_next + 1;
            // position :
            val0 = corridor_times(k);
            val1 = corridor_times(k+1);
            for(int i = 0; i < 3; i++) {  // loop for x, y, z
                int nzi = 2;
                MSKint32t asub[nzi];
                double aval[nzi];

                // This segment's last control point
                aval[0] = 1.0 * val0;
                asub[0] = sub_shift + (i+1) * s1_d1_ctrl_num - 1;
                // Next segment's first control point
                aval[1] = -1.0 * val1;
                asub[1] = sub_shift + s1_ctrl_num + i * _s1d1CtrlP_next_num;

                r = MSK_putarow(task, row_idx, nzi, asub, aval);    
                row_idx ++;
            }
            // velocity :
            val0 = order;
            val1 = order_next;
            for(int i = 0; i < 3; i++) {  // loop for x, y, z
                int nzi = 4;
                MSKint32t asub[nzi];
                double aval[nzi];
                
                // This segment's last velocity control point
                aval[0] = -1.0 * val0;
                aval[1] =  1.0 * val0;
                asub[0] = sub_shift + (i+1) * s1_d1_ctrl_num - 2;    
                asub[1] = sub_shift + (i+1) * s1_d1_ctrl_num - 1;   
                // Next segment's first velocity control point
                aval[2] =  1.0 * val1;
                aval[3] = -1.0 * val1;
                asub[2] = sub_shift + s1_ctrl_num + i * _s1d1CtrlP_next_num;    
                asub[3] = sub_shift + s1_ctrl_num + i * _s1d1CtrlP_next_num + 1;

                r = MSK_putarow(task, row_idx, nzi, asub, aval);    
                row_idx ++;
            }
            // acceleration :
            val0 = order * (order - 1) / corridor_times(k);
            val1 = order_next * (order_next - 1) / corridor_times(k+1);
            for(int i = 0; i < 3; i++) {  // loop for x, y, z
                int nzi = 6;
                MSKint32t asub[nzi];
                double aval[nzi];
                
                // This segment's last velocity control point
                aval[0] =  1.0  * val0;
                aval[1] = -2.0  * val0;
                aval[2] =  1.0  * val0;
                asub[0] = sub_shift + (i+1) * s1_d1_ctrl_num - 3;    
                asub[1] = sub_shift + (i+1) * s1_d1_ctrl_num - 2;   
                asub[2] = sub_shift + (i+1) * s1_d1_ctrl_num - 1;   
                // Next segment's first velocity control point
                aval[3] =  -1.0  * val1;
                aval[4] =   2.0  * val1;
                aval[5] =  -1.0  * val1;
                asub[3] = sub_shift + s1_ctrl_num + i * _s1d1CtrlP_next_num;    
                asub[4] = sub_shift + s1_ctrl_num + i * _s1d1CtrlP_next_num + 1;
                asub[5] = sub_shift + s1_ctrl_num + i * _s1d1CtrlP_next_num + 2;

                r = MSK_putarow(task, row_idx, nzi, asub, aval);    
                row_idx ++;
            }

            sub_shift += s1_ctrl_num;
        }
    }

    {   
        /*** Variables sequence: x, w, y, t ***/
        // #0  for all the c'x + w = u induced by corridor constraints
        int sub_idx   = 0;
        int sub_shift = 0;
        for(int k = 0; k < segment_num ; k ++ ) {   
            int order = poly_orders[k];
            s1_d1_ctrl_num = order + 1;
            s1_ctrl_num = 3 * s1_d1_ctrl_num;

            for(int p = 0; p < s1_d1_ctrl_num; p++) {
                int nzi = 4;
                MSKint32t asub[nzi];
                double aval[nzi];
                
                for(int i = 0; i < 3; i++) {   
                    // for x, y, z, no loop but in a row : x, y, z coupled
                    aval[i] = -2.0 * corridor_centers( k, i ) * corridor_times(k);
                    asub[i] = sub_shift + i * s1_d1_ctrl_num + p;    
                }

                aval[3] = 1.0;
                asub[3] = total_ctrl_num + sub_idx;    

                r = MSK_putarow(task, row_idx, nzi, asub, aval);    
                row_idx ++;
                sub_idx ++;
            }

            sub_shift += s1_ctrl_num;
        }

        // #1  for all the Fx = y mapping relationships induced by the corridor constraints
        sub_idx   = 0;
        sub_shift = 0;
        for(int k = 0; k < segment_num ; k ++ ) {   
            int order = poly_orders[k];
            s1_d1_ctrl_num = order + 1;
            s1_ctrl_num = 3 * s1_d1_ctrl_num;
            for(int p = 0; p < s1_ctrl_num; p++) {
                int nzi = 2;
                MSKint32t asub[nzi];
                double aval[nzi];
                
                aval[0] = sqrt(2.0) *corridor_times(k); //
                asub[0] = sub_shift + p;    

                aval[1] = -1.0;
                asub[1] = total_ctrl_num + var_w_num + sub_idx;    

                r = MSK_putarow(task, row_idx, nzi, asub, aval);    
                row_idx ++;
                sub_idx ++;
            }

            sub_shift += s1_ctrl_num;
        }

        // #2  for all the Fx = y mapping relationships induced by minimum snap objective
        //ROS_WARN("for variable mapping by objective");
        sub_idx   = 0;
        sub_shift = 0;
        for(int k = 0; k < segment_num; k ++) {   
            int order = poly_orders[k];
            s1_d1_ctrl_num = order + 1;
            s1_ctrl_num = 3 * s1_d1_ctrl_num;
            
            for(int p = 0; p < 3; p ++ ) {
                for( int i = 0; i < s1_d1_ctrl_num - minimize_order; i ++ ) {   
                    int nzi = 1;
                    for(int j = 0; j < s1_d1_ctrl_num - i; j++)
                        if(fabs(FMs[order](i, j)) > 0.0001 )
                            nzi += 1;

                    //int nzi = s1_d1_ctrl_num - i + 1;
                    MSKint32t asub[nzi];
                    double aval[nzi];

                    int id_j = 0;
                    for(int j = 0; j < s1_d1_ctrl_num - i; j ++) {   
                        if(fabs(FMs[order](i, j)) > 0.0001) {
                            aval[id_j] = sqrt(2.0) * FMs[order](i, j) / pow(corridor_times(k), (2 * minimize_order - 3) / 2.0); 
                            asub[id_j] = sub_shift + p * s1_d1_ctrl_num + j;    
                            id_j ++;
                        }
                    }
                    
                    aval[nzi-1] = -1.0;
                    asub[nzi-1] = total_ctrl_num + var_w_num + var_y_con + sub_idx; 

                    r = MSK_putarow(task, row_idx, nzi, asub, aval);    
                    row_idx ++;
                    sub_idx ++;
                }
            }  

            sub_shift += s1_ctrl_num;
        }
    }

    /*   Enforcing high-order constraints   */
if(is_limit_vel)
{
    // The velocity constraints
    int sub_shift   = 0;
    int sub_v_shift = 0;
    for(int k = 0; k < segment_num ; k ++ ) {   
        int order = poly_orders[k];
        s1_d1_ctrl_num = order + 1;
        s1_ctrl_num   = 3 * s1_d1_ctrl_num;

        for(int i = 0; i < 3; i++) {  // for x, y, z loop
            for(int p = 0; p < order; p++) {
                int nzi = 3;
                MSKint32t asub[nzi];
                double aval[nzi];

                aval[0] = -1.0 * order;
                aval[1] =  1.0 * order;
                aval[2] = -1.0;

                asub[0] = sub_shift + i * s1_d1_ctrl_num + p;    
                asub[1] = sub_shift + i * s1_d1_ctrl_num + p + 1;    
                asub[2] = total_ctrl_num  + var_extra_num + sub_v_shift + i * order + p;    

                //cout<<"segment num : "<<k << endl;
                r = MSK_putarow(task, row_idx, nzi, asub, aval);    
                row_idx ++;
            }
        }

        sub_shift   += s1_ctrl_num;
        sub_v_shift += 3 * order;
    }
}

if(is_limit_acc)
{
    // The acceleration constraints
    int sub_shift   = 0;
    int sub_a_shift = 0;
    for(int k = 0; k < segment_num ; k ++ ) {   
        int order = poly_orders[k];
        s1_d1_ctrl_num = order + 1;
        s1_ctrl_num   = 3 * s1_d1_ctrl_num;

        for(int i = 0; i < 3; i++) {  // for x, y, z loop
            for(int p = 0; p < order - 1; p++) {
                int nzi = 4;
                MSKint32t asub[nzi];
                double aval[nzi];

                aval[0] =  1.0 * order * (order - 1) / corridor_times(k);
                aval[1] = -2.0 * order * (order - 1) / corridor_times(k);
                aval[2] =  1.0 * order * (order - 1) / corridor_times(k);
                aval[3] = -1.0;

                asub[0] = sub_shift + i * s1_d1_ctrl_num + p;    
                asub[1] = sub_shift + i * s1_d1_ctrl_num + p + 1;    
                asub[2] = sub_shift + i * s1_d1_ctrl_num + p + 2;    
                asub[3] = total_ctrl_num  + var_extra_num + vel_var_num + sub_a_shift + i * (order - 1) + p;    

                //cout<<"segment num : "<<k << endl;
                r = MSK_putarow(task, row_idx, nzi, asub, aval);    
                row_idx ++;
            }
        }

        sub_shift   += s1_ctrl_num;
        sub_a_shift += 3 * (order - 1);
    }
}

    /*** Variables sequence: x, w, y, t ***/
    /*  w num = 1 + inequ_con_num; 
        y num = (s1_d1_ctrl_num - 3) * 3 * segment_num + inequ_con_num * 3;
        t num = 1 + inequ_con_num;  */
    // Stack all conic cones;
    {   
        /** cones induced by quadratical constraints **/
        int sub_idx   = 0;
        int sub_shift = 0;
        for(int k = 0; k < segment_num ; k ++ ) {   
            int order = poly_orders[k];
            s1_d1_ctrl_num = order + 1;
            s1_ctrl_num = 3 * s1_d1_ctrl_num;
            for(int p = 0; p < s1_d1_ctrl_num; p++) {
                int nzi = 5;
                MSKint32t csub[nzi];                
                
                // Append one rotated conic cone.
                csub[0] = total_ctrl_num + sub_idx;
                csub[1] = total_ctrl_num + var_w_num + var_y_num + sub_idx;

                for( int i = 0; i < 3; i++ )
                    csub[i + 2] = total_ctrl_num + var_w_num + sub_shift + i * s1_d1_ctrl_num + p;

                r = MSK_appendcone(task, MSK_CT_RQUAD, 0.0, nzi, csub);
                sub_idx ++;
            }

            sub_shift += s1_ctrl_num;
        }
        /** the cone induced by the quadratical objective **/
        int nzi = 2 + obj_nzero_num;
        MSKint32t csub[nzi];                
        
        // Append one rotated conic cone.
        csub[0] = total_ctrl_num + sub_idx;
        csub[1] = total_ctrl_num + var_w_num + var_y_num + sub_idx;

        for( int i = 0; i < obj_nzero_num; i++ )
            csub[i + 2] = total_ctrl_num + var_w_num + var_y_con + i;

        r = MSK_appendcone(task, MSK_CT_RQUAD, 0.0, nzi, csub);
    }

    // Stack the objective function;
    /*** Now no quadratical objective exists, replaced it by a linear variable w ***/    
    /*  Set the linear term c_j in the objective.  */  
    MSKint32t csub = total_ctrl_num + var_w_num - 1; 
    r = MSK_putcj(task, csub, 0.5);

    if ( r == MSK_RES_OK ) 
         r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);
    
    bool solve_ok = false;
    if ( r == MSK_RES_OK ) { 
        MSKrescodee trmcode; 
        r = MSK_optimizetrm(task,&trmcode); 
        MSK_solutionsummary (task,MSK_STREAM_LOG); 
          
        if ( r == MSK_RES_OK ) { 
            MSKsolstae solsta; 
            MSK_getsolsta (task,MSK_SOL_ITR,&solsta); 

            switch(solsta) { 
                case MSK_SOL_STA_OPTIMAL:    
                case MSK_SOL_STA_NEAR_OPTIMAL: 
                  
                r = MSK_getxx(task, MSK_SOL_ITR, x_var);  // Request the interior solution.  
                r = MSK_getprimalobj( 
                    task, MSK_SOL_ITR, &primalobj);

                _objective = primalobj;
                solve_ok = true;
                break; 

                case MSK_SOL_STA_DUAL_INFEAS_CER: 
                case MSK_SOL_STA_PRIM_INFEAS_CER: 
                case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER: 
                case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:   
                    printf("Primal or dual infeasibility certificate found.\n"); 
                    break; 
                   
                case MSK_SOL_STA_UNKNOWN: 
                    printf("The status of the solution could not be determined.\n"); 
                    break; 

                default: 
                    printf("Other solution status."); 
                    break; 
            } 
        } 
        else 
            printf("Error while optimizing.\n"); 
    }
     
    if (r != MSK_RES_OK) { 
        // In case of an error print error code and description. 
        char symname[MSK_MAX_STR_LEN]; 
        char desc[MSK_MAX_STR_LEN]; 
         
        printf("An error occurred while optimizing.\n");      
        MSK_getcodedesc (r, 
                         symname, 
                         desc); 
        printf("Error %s - '%s'\n",symname,desc); 
    } 
    
    MSK_deletetask(&task); 
    MSK_deleteenv(&env); 

    if(!solve_ok){
        MatrixXd poly_fail = MatrixXd::Identity(3,3);
        ROS_WARN("In solver, falied ");
        return poly_fail;
    }

    VectorXd d_var(var_num);
    for(int i = 0; i < var_num; i++)
        d_var(i) = x_var[i];

    int max_order = *max_element( begin( poly_orders ), end( poly_orders ) );    
    PolyCoeff = MatrixXd::Zero(segment_num, 3 *(max_order + 1) );

    int var_shift = 0;
    for(int i = 0; i < segment_num; i++ ){
        int order = poly_orders[i];
        int poly_num1d = order + 1;
        
        for(int j = 0; j < 3 * poly_num1d; j++)
            PolyCoeff(i , j) = d_var(j + var_shift);

        var_shift += 3 * poly_num1d;
    }   
    
    return PolyCoeff;
}
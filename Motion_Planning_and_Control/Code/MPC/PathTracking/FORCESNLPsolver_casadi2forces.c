/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "FORCESNLPsolver/include/FORCESNLPsolver.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif


#include "FORCESNLPsolver_casadi.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, FORCESNLPsolver_callback_float *data, FORCESNLPsolver_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((FORCESNLPsolver_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void FORCESNLPsolver_casadi2forces(FORCESNLPsolver_float *x,        /* primal vars                                         */
                                 FORCESNLPsolver_float *y,        /* eq. constraint multiplers                           */
                                 FORCESNLPsolver_float *l,        /* ineq. constraint multipliers                        */
                                 FORCESNLPsolver_float *p,        /* parameters                                          */
                                 FORCESNLPsolver_float *f,        /* objective function (scalar)                         */
                                 FORCESNLPsolver_float *nabla_f,  /* gradient of objective function                      */
                                 FORCESNLPsolver_float *c,        /* dynamics                                            */
                                 FORCESNLPsolver_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 FORCESNLPsolver_float *h,        /* inequality constraints                              */
                                 FORCESNLPsolver_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 FORCESNLPsolver_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const FORCESNLPsolver_callback_float *in[4];
    FORCESNLPsolver_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	FORCESNLPsolver_callback_float w[10];
	
    /* temporary storage for CasADi sparse output */
    FORCESNLPsolver_callback_float this_f;
    FORCESNLPsolver_callback_float nabla_f_sparse[4];
    
    
    FORCESNLPsolver_callback_float c_sparse[5];
    FORCESNLPsolver_callback_float nabla_c_sparse[15];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for CasADi */
	in[0] = x;
	in[1] = p;
	in[2] = l;
	in[3] = y;

	if ((stage >= 0) && (stage < 29))
	{
		out[0] = &this_f;
		FORCESNLPsolver_objective0rd_1(in, out, NULL, w, 0);
		out[0] = nabla_f_sparse;
		if (nabla_f != NULL)
		{
			FORCESNLPsolver_objective1rd_1(in, out, NULL, w, 0);
			nrow = FORCESNLPsolver_objective1rd_1_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_objective1rd_1_sparsity_out(0)[1];
			colind = FORCESNLPsolver_objective1rd_1_sparsity_out(0) + 2;
			row = FORCESNLPsolver_objective1rd_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

		out[0] = c_sparse;
		if (c != NULL)
		{
			FORCESNLPsolver_dynamics0rd_1(in, out, NULL, w, 0);
			nrow = FORCESNLPsolver_dynamics0rd_1_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_dynamics0rd_1_sparsity_out(0)[1];
			colind = FORCESNLPsolver_dynamics0rd_1_sparsity_out(0) + 2;
			row = FORCESNLPsolver_dynamics0rd_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, c_sparse, c);
		}

		out[0] = nabla_c_sparse;
		if (nabla_c != NULL)
		{
			FORCESNLPsolver_dynamics1rd_1(in, out, NULL, w, 0);
			nrow = FORCESNLPsolver_dynamics1rd_1_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_dynamics1rd_1_sparsity_out(0)[1];
			colind = FORCESNLPsolver_dynamics1rd_1_sparsity_out(0) + 2;
			row = FORCESNLPsolver_dynamics1rd_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_c_sparse, nabla_c);
		}

	}

	if ((stage >= 29) && (stage < 30))
	{
		out[0] = &this_f;
		FORCESNLPsolver_objective0rd_30(in, out, NULL, w, 0);
		out[0] = nabla_f_sparse;
		if (nabla_f != NULL)
		{
			FORCESNLPsolver_objective1rd_30(in, out, NULL, w, 0);
			nrow = FORCESNLPsolver_objective1rd_30_sparsity_out(0)[0];
			ncol = FORCESNLPsolver_objective1rd_30_sparsity_out(0)[1];
			colind = FORCESNLPsolver_objective1rd_30_sparsity_out(0) + 2;
			row = FORCESNLPsolver_objective1rd_30_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

	}


    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((FORCESNLPsolver_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif

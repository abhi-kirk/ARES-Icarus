// QOCO and OSQP each ship their own copy of QDLDL and SuiteSparse, and a few
// of QOCO's own helpers use the same names OSQP already exports. This header
// is force-included only while QOCO itself is compiled, so those copies get
// a qoco_ prefix and the two static libraries can live in one binary.
#pragma once

#define QDLDL_etree qoco_QDLDL_etree
#define QDLDL_factor qoco_QDLDL_factor
#define QDLDL_solve qoco_QDLDL_solve
#define QDLDL_Lsolve qoco_QDLDL_Lsolve
#define QDLDL_Ltsolve qoco_QDLDL_Ltsolve

#define SuiteSparse_divcomplex qoco_SuiteSparse_divcomplex
#define SuiteSparse_free qoco_SuiteSparse_free
#define SuiteSparse_hypot qoco_SuiteSparse_hypot
#define SuiteSparse_malloc qoco_SuiteSparse_malloc
#define SuiteSparse_realloc qoco_SuiteSparse_realloc
#define SuiteSparse_tic qoco_SuiteSparse_tic
#define SuiteSparse_time qoco_SuiteSparse_time
#define SuiteSparse_toc qoco_SuiteSparse_toc
#define SuiteSparse_version qoco_SuiteSparse_version

#define copy_settings qoco_copy_settings
#define csc_symperm qoco_csc_symperm
#define print_footer qoco_print_footer
#define print_header qoco_print_header

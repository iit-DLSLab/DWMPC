/* This file was automatically generated by CasADi 3.6.7.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) front_cost_y_e_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[29] = {25, 1, 0, 25, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[55] = {51, 1, 0, 51, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50};
static const casadi_int casadi_s3[34] = {30, 1, 0, 30, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29};

/* front_cost_y_e_fun:(i0[25],i1[],i2[],i3[],i4[51])->(o0[30]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a1=arg[0]? arg[0][1] : 0;
  if (res[0]!=0) res[0][1]=a1;
  a2=arg[0]? arg[0][2] : 0;
  if (res[0]!=0) res[0][2]=a2;
  a3=2.;
  a4=arg[4]? arg[4][37] : 0;
  a5=arg[0]? arg[0][3] : 0;
  a6=(a4*a5);
  a7=arg[0]? arg[0][6] : 0;
  a8=arg[4]? arg[4][34] : 0;
  a9=(a7*a8);
  a6=(a6-a9);
  a9=arg[4]? arg[4][36] : 0;
  a10=arg[0]? arg[0][4] : 0;
  a11=(a9*a10);
  a12=arg[4]? arg[4][35] : 0;
  a13=arg[0]? arg[0][5] : 0;
  a14=(a12*a13);
  a11=(a11-a14);
  a6=(a6+a11);
  a11=(a3*a6);
  a14=2.2204460492503131e-16;
  a6=(a6+a14);
  a6=casadi_sq(a6);
  a15=(a4*a10);
  a16=(a7*a12);
  a15=(a15-a16);
  a16=(a8*a13);
  a17=(a9*a5);
  a16=(a16-a17);
  a15=(a15+a16);
  a16=(a15+a14);
  a16=casadi_sq(a16);
  a6=(a6+a16);
  a16=(a4*a13);
  a17=(a7*a9);
  a16=(a16-a17);
  a17=(a12*a5);
  a18=(a8*a10);
  a17=(a17-a18);
  a16=(a16+a17);
  a14=(a16+a14);
  a14=casadi_sq(a14);
  a6=(a6+a14);
  a6=sqrt(a6);
  a4=(a4*a7);
  a8=(a8*a5);
  a12=(a12*a10);
  a8=(a8+a12);
  a9=(a9*a13);
  a8=(a8+a9);
  a4=(a4+a8);
  a4=atan2(a6,a4);
  a11=(a11*a4);
  a11=(a11/a6);
  if (res[0]!=0) res[0][3]=a11;
  a15=(a3*a15);
  a15=(a15*a4);
  a15=(a15/a6);
  if (res[0]!=0) res[0][4]=a15;
  a16=(a3*a16);
  a16=(a16*a4);
  a16=(a16/a6);
  if (res[0]!=0) res[0][5]=a16;
  a16=arg[0]? arg[0][7] : 0;
  if (res[0]!=0) res[0][6]=a16;
  a6=arg[0]? arg[0][8] : 0;
  if (res[0]!=0) res[0][7]=a6;
  a4=arg[0]? arg[0][9] : 0;
  if (res[0]!=0) res[0][8]=a4;
  a15=arg[0]? arg[0][10] : 0;
  if (res[0]!=0) res[0][9]=a15;
  a11=arg[0]? arg[0][11] : 0;
  if (res[0]!=0) res[0][10]=a11;
  a8=arg[0]? arg[0][12] : 0;
  if (res[0]!=0) res[0][11]=a8;
  a9=arg[0]? arg[0][13] : 0;
  if (res[0]!=0) res[0][12]=a9;
  a9=arg[0]? arg[0][14] : 0;
  if (res[0]!=0) res[0][13]=a9;
  a9=arg[0]? arg[0][15] : 0;
  if (res[0]!=0) res[0][14]=a9;
  a9=arg[0]? arg[0][16] : 0;
  if (res[0]!=0) res[0][15]=a9;
  a9=arg[0]? arg[0][17] : 0;
  if (res[0]!=0) res[0][16]=a9;
  a9=arg[0]? arg[0][18] : 0;
  if (res[0]!=0) res[0][17]=a9;
  a9=arg[0]? arg[0][19] : 0;
  if (res[0]!=0) res[0][18]=a9;
  a9=arg[0]? arg[0][20] : 0;
  if (res[0]!=0) res[0][19]=a9;
  a9=arg[0]? arg[0][21] : 0;
  if (res[0]!=0) res[0][20]=a9;
  a9=arg[0]? arg[0][22] : 0;
  if (res[0]!=0) res[0][21]=a9;
  a9=arg[0]? arg[0][23] : 0;
  if (res[0]!=0) res[0][22]=a9;
  a9=arg[0]? arg[0][24] : 0;
  if (res[0]!=0) res[0][23]=a9;
  a9=-2.5000000000000000e-01;
  a12=1.;
  a14=casadi_sq(a13);
  a17=casadi_sq(a10);
  a14=(a14+a17);
  a14=(a3*a14);
  a14=(a12-a14);
  a17=cos(a6);
  a18=(a14*a17);
  a7=(a3*a7);
  a19=(a7*a10);
  a20=(a13*a5);
  a20=(a3*a20);
  a19=(a19+a20);
  a20=cos(a16);
  a21=(a19*a20);
  a22=(a10*a5);
  a22=(a3*a22);
  a23=(a7*a13);
  a22=(a22-a23);
  a16=sin(a16);
  a23=(a22*a16);
  a21=(a21-a23);
  a6=sin(a6);
  a23=(a21*a6);
  a18=(a18-a23);
  a23=sin(a4);
  a18=(a18*a23);
  a24=(a14*a6);
  a21=(a21*a17);
  a24=(a24+a21);
  a4=cos(a4);
  a21=(a24*a4);
  a18=(a18+a21);
  a18=(a9*a18);
  a24=(a9*a24);
  a21=8.6800000000000002e-02;
  a25=(a22*a20);
  a26=(a19*a16);
  a25=(a25+a26);
  a25=(a21*a25);
  a26=2.4070000000000000e-01;
  a27=(a26*a14);
  a28=5.0999999999999997e-02;
  a29=(a28*a22);
  a27=(a27+a29);
  a27=(a27+a0);
  a25=(a25+a27);
  a24=(a24+a25);
  a18=(a18+a24);
  if (res[0]!=0) res[0][24]=a18;
  a18=(a7*a13);
  a24=(a5*a10);
  a24=(a3*a24);
  a18=(a18+a24);
  a24=(a18*a17);
  a25=(a13*a10);
  a25=(a3*a25);
  a27=(a7*a5);
  a25=(a25-a27);
  a27=(a25*a20);
  a29=casadi_sq(a13);
  a30=casadi_sq(a5);
  a29=(a29+a30);
  a29=(a3*a29);
  a29=(a12-a29);
  a30=(a29*a16);
  a27=(a27-a30);
  a30=(a27*a6);
  a24=(a24-a30);
  a24=(a24*a23);
  a30=(a18*a6);
  a27=(a27*a17);
  a30=(a30+a27);
  a27=(a30*a4);
  a24=(a24+a27);
  a24=(a9*a24);
  a30=(a9*a30);
  a27=(a29*a20);
  a31=(a25*a16);
  a27=(a27+a31);
  a27=(a21*a27);
  a31=(a26*a18);
  a32=(a28*a29);
  a31=(a31+a32);
  a31=(a31+a1);
  a27=(a27+a31);
  a30=(a30+a27);
  a24=(a24+a30);
  if (res[0]!=0) res[0][25]=a24;
  a24=(a5*a13);
  a24=(a3*a24);
  a30=(a7*a10);
  a24=(a24-a30);
  a30=(a24*a17);
  a27=casadi_sq(a10);
  a31=casadi_sq(a5);
  a27=(a27+a31);
  a27=(a3*a27);
  a12=(a12-a27);
  a27=(a12*a20);
  a7=(a7*a5);
  a10=(a10*a13);
  a3=(a3*a10);
  a7=(a7+a3);
  a3=(a7*a16);
  a27=(a27-a3);
  a3=(a27*a6);
  a30=(a30-a3);
  a30=(a30*a23);
  a6=(a24*a6);
  a27=(a27*a17);
  a6=(a6+a27);
  a4=(a6*a4);
  a30=(a30+a4);
  a30=(a9*a30);
  a6=(a9*a6);
  a20=(a7*a20);
  a16=(a12*a16);
  a20=(a20+a16);
  a21=(a21*a20);
  a20=(a26*a24);
  a28=(a28*a7);
  a20=(a20+a28);
  a20=(a20+a2);
  a21=(a21+a20);
  a6=(a6+a21);
  a30=(a30+a6);
  if (res[0]!=0) res[0][26]=a30;
  a30=cos(a11);
  a6=(a14*a30);
  a21=cos(a15);
  a20=(a19*a21);
  a15=sin(a15);
  a28=(a22*a15);
  a20=(a20-a28);
  a11=sin(a11);
  a28=(a20*a11);
  a6=(a6-a28);
  a28=sin(a8);
  a6=(a6*a28);
  a16=(a14*a11);
  a20=(a20*a30);
  a16=(a16+a20);
  a8=cos(a8);
  a20=(a16*a8);
  a6=(a6+a20);
  a6=(a9*a6);
  a16=(a9*a16);
  a20=-8.6800000000000002e-02;
  a4=(a22*a21);
  a19=(a19*a15);
  a4=(a4+a19);
  a4=(a20*a4);
  a14=(a26*a14);
  a19=-5.0999999999999997e-02;
  a22=(a19*a22);
  a14=(a14+a22);
  a14=(a14+a0);
  a4=(a4+a14);
  a16=(a16+a4);
  a6=(a6+a16);
  if (res[0]!=0) res[0][27]=a6;
  a6=(a18*a30);
  a16=(a25*a21);
  a4=(a29*a15);
  a16=(a16-a4);
  a4=(a16*a11);
  a6=(a6-a4);
  a6=(a6*a28);
  a4=(a18*a11);
  a16=(a16*a30);
  a4=(a4+a16);
  a16=(a4*a8);
  a6=(a6+a16);
  a6=(a9*a6);
  a4=(a9*a4);
  a16=(a29*a21);
  a25=(a25*a15);
  a16=(a16+a25);
  a16=(a20*a16);
  a18=(a26*a18);
  a29=(a19*a29);
  a18=(a18+a29);
  a18=(a18+a1);
  a16=(a16+a18);
  a4=(a4+a16);
  a6=(a6+a4);
  if (res[0]!=0) res[0][28]=a6;
  a6=(a24*a30);
  a4=(a12*a21);
  a16=(a7*a15);
  a4=(a4-a16);
  a16=(a4*a11);
  a6=(a6-a16);
  a6=(a6*a28);
  a11=(a24*a11);
  a4=(a4*a30);
  a11=(a11+a4);
  a8=(a11*a8);
  a6=(a6+a8);
  a6=(a9*a6);
  a9=(a9*a11);
  a21=(a7*a21);
  a12=(a12*a15);
  a21=(a21+a12);
  a20=(a20*a21);
  a26=(a26*a24);
  a19=(a19*a7);
  a26=(a26+a19);
  a26=(a26+a2);
  a20=(a20+a26);
  a9=(a9+a20);
  a6=(a6+a9);
  if (res[0]!=0) res[0][29]=a6;
  return 0;
}

CASADI_SYMBOL_EXPORT int front_cost_y_e_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int front_cost_y_e_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int front_cost_y_e_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void front_cost_y_e_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int front_cost_y_e_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void front_cost_y_e_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void front_cost_y_e_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void front_cost_y_e_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int front_cost_y_e_fun_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int front_cost_y_e_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real front_cost_y_e_fun_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* front_cost_y_e_fun_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* front_cost_y_e_fun_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* front_cost_y_e_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s1;
    case 4: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* front_cost_y_e_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int front_cost_y_e_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int front_cost_y_e_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 1*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif

/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) LSTM_v21_MHE_cost_y_fun_jac_ut_xt_ ## ID
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
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)

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

static const casadi_int casadi_s0[39] = {35, 1, 0, 35, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34};
static const casadi_int casadi_s1[3] = {0, 0, 0};
static const casadi_int casadi_s2[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s3[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s4[43] = {70, 4, 0, 17, 34, 35, 36, 35, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 36, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 0, 1};
static const casadi_int casadi_s5[3] = {4, 0, 0};

/* LSTM_v21_MHE_cost_y_fun_jac_ut_xt:(i0[35],i1[35],i2[],i3[5])->(o0[4],o1[70x4,36nz],o2[4x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a3, a4, a5, a6, a7, a8, a9;
  a0=1.0000000000000001e-001;
  a1=1.3480333254290144e+000;
  a2=-4.8109439015388489e-001;
  a3=arg[0]? arg[0][3] : 0;
  a2=(a2*a3);
  a4=-4.4737485051155090e-001;
  a5=arg[0]? arg[0][4] : 0;
  a4=(a4*a5);
  a2=(a2+a4);
  a4=-4.5633724331855774e-001;
  a6=arg[0]? arg[0][5] : 0;
  a4=(a4*a6);
  a2=(a2+a4);
  a4=3.7879323959350586e-001;
  a7=arg[0]? arg[0][6] : 0;
  a4=(a4*a7);
  a2=(a2+a4);
  a4=-8.2274585962295532e-002;
  a8=arg[0]? arg[0][7] : 0;
  a4=(a4*a8);
  a2=(a2+a4);
  a4=-4.0359571576118469e-001;
  a9=arg[0]? arg[0][8] : 0;
  a4=(a4*a9);
  a2=(a2+a4);
  a4=-4.5680344104766846e-001;
  a10=arg[0]? arg[0][9] : 0;
  a4=(a4*a10);
  a2=(a2+a4);
  a4=-3.8456687331199646e-001;
  a11=arg[0]? arg[0][10] : 0;
  a4=(a4*a11);
  a2=(a2+a4);
  a4=-5.5037727579474449e-003;
  a12=arg[0]? arg[0][11] : 0;
  a4=(a4*a12);
  a2=(a2+a4);
  a4=2.7442198991775513e-001;
  a13=arg[0]? arg[0][12] : 0;
  a4=(a4*a13);
  a2=(a2+a4);
  a4=9.3669565394520760e-003;
  a14=arg[0]? arg[0][13] : 0;
  a4=(a4*a14);
  a2=(a2+a4);
  a4=-5.8722972869873047e-001;
  a15=arg[0]? arg[0][14] : 0;
  a4=(a4*a15);
  a2=(a2+a4);
  a4=4.8781847953796387e-001;
  a16=arg[0]? arg[0][15] : 0;
  a4=(a4*a16);
  a2=(a2+a4);
  a4=3.7178263068199158e-001;
  a17=arg[0]? arg[0][16] : 0;
  a4=(a4*a17);
  a2=(a2+a4);
  a4=5.6983149051666260e-001;
  a18=arg[0]? arg[0][17] : 0;
  a4=(a4*a18);
  a2=(a2+a4);
  a4=3.5502251982688904e-001;
  a19=arg[0]? arg[0][18] : 0;
  a4=(a4*a19);
  a2=(a2+a4);
  a4=3.1330011785030365e-002;
  a2=(a2+a4);
  a2=(a1*a2);
  a4=-5.1882001347507867e-001;
  a2=(a2+a4);
  a2=(a0*a2);
  a20=arg[0]? arg[0][0] : 0;
  a2=(a2+a20);
  if (res[0]!=0) res[0][0]=a2;
  a2=-8.7548941373825073e-002;
  a2=(a2*a3);
  a3=9.2034734785556793e-002;
  a3=(a3*a5);
  a2=(a2+a3);
  a3=-5.6610780954360962e-001;
  a3=(a3*a6);
  a2=(a2+a3);
  a3=5.0127464532852173e-001;
  a3=(a3*a7);
  a2=(a2+a3);
  a3=1.0195219516754150e-001;
  a3=(a3*a8);
  a2=(a2+a3);
  a3=-3.4450858831405640e-001;
  a3=(a3*a9);
  a2=(a2+a3);
  a3=-1.5705892443656921e-001;
  a3=(a3*a10);
  a2=(a2+a3);
  a3=5.9483474493026733e-001;
  a3=(a3*a11);
  a2=(a2+a3);
  a3=3.0953091382980347e-001;
  a3=(a3*a12);
  a2=(a2+a3);
  a3=1.1097805202007294e-001;
  a3=(a3*a13);
  a2=(a2+a3);
  a3=1.9161945581436157e-001;
  a3=(a3*a14);
  a2=(a2+a3);
  a3=3.2058286666870117e-001;
  a3=(a3*a15);
  a2=(a2+a3);
  a3=-4.1404223442077637e-001;
  a3=(a3*a16);
  a2=(a2+a3);
  a3=3.3569580316543579e-001;
  a3=(a3*a17);
  a2=(a2+a3);
  a3=-2.3255659639835358e-001;
  a3=(a3*a18);
  a2=(a2+a3);
  a3=1.7910581827163696e-001;
  a3=(a3*a19);
  a2=(a2+a3);
  a3=8.1867419183254242e-002;
  a2=(a2+a3);
  a1=(a1*a2);
  a1=(a1+a4);
  a0=(a0*a1);
  a1=arg[0]? arg[0][1] : 0;
  a0=(a0+a1);
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[1]? arg[1][0] : 0;
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[1]? arg[1][1] : 0;
  if (res[0]!=0) res[0][3]=a0;
  a0=1.;
  if (res[1]!=0) res[1][0]=a0;
  a1=-6.4853127060438512e-002;
  if (res[1]!=0) res[1][1]=a1;
  a1=-6.0307620744839419e-002;
  if (res[1]!=0) res[1][2]=a1;
  a1=-6.1515781162782467e-002;
  if (res[1]!=0) res[1][3]=a1;
  a1=5.1062591041926314e-002;
  if (res[1]!=0) res[1][4]=a1;
  a1=-1.1090888371304855e-002;
  if (res[1]!=0) res[1][5]=a1;
  a1=-5.4406047484645309e-002;
  if (res[1]!=0) res[1][6]=a1;
  a1=-6.1578626170290529e-002;
  if (res[1]!=0) res[1][7]=a1;
  a1=-5.1840896108060912e-002;
  if (res[1]!=0) res[1][8]=a1;
  a1=-7.4192690933015128e-004;
  if (res[1]!=0) res[1][9]=a1;
  a1=3.6992998763967895e-002;
  if (res[1]!=0) res[1][10]=a1;
  a1=1.2626969573026636e-003;
  if (res[1]!=0) res[1][11]=a1;
  a1=-7.9160524396852755e-002;
  if (res[1]!=0) res[1][12]=a1;
  a1=6.5759556717728707e-002;
  if (res[1]!=0) res[1][13]=a1;
  a1=5.0117537597499227e-002;
  if (res[1]!=0) res[1][14]=a1;
  a1=7.6815183909534859e-002;
  if (res[1]!=0) res[1][15]=a1;
  a1=4.7858218800442942e-002;
  if (res[1]!=0) res[1][16]=a1;
  if (res[1]!=0) res[1][17]=a0;
  a1=-1.1801889057794723e-002;
  if (res[1]!=0) res[1][18]=a1;
  a1=1.2406588958795152e-002;
  if (res[1]!=0) res[1][19]=a1;
  a1=-7.6313219305040716e-002;
  if (res[1]!=0) res[1][20]=a1;
  a1=6.7573492709545699e-002;
  if (res[1]!=0) res[1][21]=a1;
  a1=1.3743495668648888e-002;
  if (res[1]!=0) res[1][22]=a1;
  a1=-4.6440905794385277e-002;
  if (res[1]!=0) res[1][23]=a1;
  a1=-2.1172066419653270e-002;
  if (res[1]!=0) res[1][24]=a1;
  a1=8.0185705928906781e-002;
  if (res[1]!=0) res[1][25]=a1;
  a1=4.1725798709307170e-002;
  if (res[1]!=0) res[1][26]=a1;
  a1=1.4960211251425307e-002;
  if (res[1]!=0) res[1][27]=a1;
  a1=2.5830941223833191e-002;
  if (res[1]!=0) res[1][28]=a1;
  a1=4.3215638783097560e-002;
  if (res[1]!=0) res[1][29]=a1;
  a1=-5.5814273013429873e-002;
  if (res[1]!=0) res[1][30]=a1;
  a1=4.5252912987366625e-002;
  if (res[1]!=0) res[1][31]=a1;
  a1=-3.1349404199332576e-002;
  if (res[1]!=0) res[1][32]=a1;
  a1=2.4144061180839952e-002;
  if (res[1]!=0) res[1][33]=a1;
  if (res[1]!=0) res[1][34]=a0;
  if (res[1]!=0) res[1][35]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int LSTM_v21_MHE_cost_y_fun_jac_ut_xt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int LSTM_v21_MHE_cost_y_fun_jac_ut_xt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int LSTM_v21_MHE_cost_y_fun_jac_ut_xt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void LSTM_v21_MHE_cost_y_fun_jac_ut_xt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int LSTM_v21_MHE_cost_y_fun_jac_ut_xt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void LSTM_v21_MHE_cost_y_fun_jac_ut_xt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void LSTM_v21_MHE_cost_y_fun_jac_ut_xt_incref(void) {
}

CASADI_SYMBOL_EXPORT void LSTM_v21_MHE_cost_y_fun_jac_ut_xt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int LSTM_v21_MHE_cost_y_fun_jac_ut_xt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int LSTM_v21_MHE_cost_y_fun_jac_ut_xt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real LSTM_v21_MHE_cost_y_fun_jac_ut_xt_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* LSTM_v21_MHE_cost_y_fun_jac_ut_xt_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* LSTM_v21_MHE_cost_y_fun_jac_ut_xt_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* LSTM_v21_MHE_cost_y_fun_jac_ut_xt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* LSTM_v21_MHE_cost_y_fun_jac_ut_xt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int LSTM_v21_MHE_cost_y_fun_jac_ut_xt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif

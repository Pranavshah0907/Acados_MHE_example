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
#define casadi_c0 CASADI_PREFIX(c0)
#define casadi_c1 CASADI_PREFIX(c1)
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s10 CASADI_PREFIX(s10)
#define casadi_s11 CASADI_PREFIX(s11)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
#define casadi_s9 CASADI_PREFIX(s9)
#define casadi_trans CASADI_PREFIX(trans)

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

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

void casadi_trans(const casadi_real* x, const casadi_int* sp_x, casadi_real* y,
    const casadi_int* sp_y, casadi_int* tmp) {
  casadi_int ncol_x, nnz_x, ncol_y, k;
  const casadi_int* row_x, *colind_y;
  ncol_x = sp_x[1];
  nnz_x = sp_x[2 + ncol_x];
  row_x = sp_x + 2 + ncol_x+1;
  ncol_y = sp_y[1];
  colind_y = sp_y+2;
  for (k=0; k<ncol_y; ++k) tmp[k] = colind_y[k];
  for (k=0; k<nnz_x; ++k) {
    y[tmp[row_x[k]]++] = x[k];
  }
}

static const casadi_int casadi_s0[9] = {5, 3, 0, 1, 2, 3, 2, 3, 4};
static const casadi_int casadi_s1[11] = {3, 5, 0, 0, 0, 1, 2, 3, 0, 1, 2};
static const casadi_int casadi_s2[72] = {5, 35, 0, 1, 2, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
static const casadi_int casadi_s3[42] = {35, 5, 0, 17, 34, 34, 34, 34, 0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
static const casadi_int casadi_s4[45] = {38, 5, 0, 17, 34, 35, 36, 37, 3, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2};
static const casadi_int casadi_s5[78] = {5, 38, 0, 1, 2, 3, 4, 5, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 2, 3, 4, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
static const casadi_int casadi_s6[39] = {35, 1, 0, 35, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34};
static const casadi_int casadi_s7[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s8[3] = {0, 0, 0};
static const casadi_int casadi_s9[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s10[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s11[3] = {5, 0, 0};

static const casadi_real casadi_c0[2] = {3.1330011785030365e-002, 8.1867419183254242e-002};
static const casadi_real casadi_c1[32] = {-4.8109439015388489e-001, -8.7548941373825073e-002, -4.4737485051155090e-001, 9.2034734785556793e-002, -4.5633724331855774e-001, -5.6610780954360962e-001, 3.7879323959350586e-001, 5.0127464532852173e-001, -8.2274585962295532e-002, 1.0195219516754150e-001, -4.0359571576118469e-001, -3.4450858831405640e-001, -4.5680344104766846e-001, -1.5705892443656921e-001, -3.8456687331199646e-001, 5.9483474493026733e-001, -5.5037727579474449e-003, 3.0953091382980347e-001, 2.7442198991775513e-001, 1.1097805202007294e-001, 9.3669565394520760e-003, 1.9161945581436157e-001, -5.8722972869873047e-001, 3.2058286666870117e-001, 4.8781847953796387e-001, -4.1404223442077637e-001, 3.7178263068199158e-001, 3.3569580316543579e-001, 5.6983149051666260e-001, -2.3255659639835358e-001, 3.5502251982688904e-001, 1.7910581827163696e-001};

/* LSTM_v21_MHE_cost_y_fun_jac_ut_xt:(i0[35],i1[3],i2[],i3[2])->(o0[5],o1[38x5,37nz],o2[5x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, w1, w2, *w3=w+19, *w4=w+21, *w5=w+23, *w6=w+55, w7, w8, w9, *w10=w+74, *w13=w+77, w14, *w15=w+81, *w16=w+84, *w17=w+118, *w18=w+122, *w19=w+154, *w20=w+188, *w21=w+225;
  /* #0: @0 = 0.1 */
  w0 = 1.0000000000000001e-001;
  /* #1: @1 = -0.51882 */
  w1 = -5.1882001347507867e-001;
  /* #2: @2 = 1.34803 */
  w2 = 1.3480333254290144e+000;
  /* #3: @3 = [0.03133, 0.0818674] */
  casadi_copy(casadi_c0, 2, w3);
  /* #4: @4 = zeros(2x1) */
  casadi_clear(w4, 2);
  /* #5: @5 = 
  [[-0.481094, -0.447375, -0.456337, 0.378793, -0.0822746, -0.403596, -0.456803, -0.384567, -0.00550377, 0.274422, 0.00936696, -0.58723, 0.487818, 0.371783, 0.569831, 0.355023], 
   [-0.0875489, 0.0920347, -0.566108, 0.501275, 0.101952, -0.344509, -0.157059, 0.594835, 0.309531, 0.110978, 0.191619, 0.320583, -0.414042, 0.335696, -0.232557, 0.179106]] */
  casadi_copy(casadi_c1, 32, w5);
  /* #6: @6 = input[0][3] */
  casadi_copy(arg[0] ? arg[0]+3 : 0, 16, w6);
  /* #7: @4 = mac(@5,@6,@4) */
  for (i=0, rr=w4; i<1; ++i) for (j=0; j<2; ++j, ++rr) for (k=0, ss=w5+j, tt=w6+i*16; k<16; ++k) *rr += ss[k*2]**tt++;
  /* #8: @3 = (@3+@4) */
  for (i=0, rr=w3, cs=w4; i<2; ++i) (*rr++) += (*cs++);
  /* #9: @7 = @3[0] */
  for (rr=(&w7), ss=w3+0; ss!=w3+1; ss+=1) *rr++ = *ss;
  /* #10: @7 = (@2*@7) */
  w7  = (w2*w7);
  /* #11: @1 = (@1+@7) */
  w1 += w7;
  /* #12: @1 = (@0*@1) */
  w1  = (w0*w1);
  /* #13: @7 = input[0][0] */
  w7 = arg[0] ? arg[0][0] : 0;
  /* #14: @1 = (@1+@7) */
  w1 += w7;
  /* #15: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  /* #16: @1 = 0.1 */
  w1 = 1.0000000000000001e-001;
  /* #17: @7 = -0.51882 */
  w7 = -5.1882001347507867e-001;
  /* #18: @8 = 1.34803 */
  w8 = 1.3480333254290144e+000;
  /* #19: @9 = @3[1] */
  for (rr=(&w9), ss=w3+1; ss!=w3+2; ss+=1) *rr++ = *ss;
  /* #20: @9 = (@8*@9) */
  w9  = (w8*w9);
  /* #21: @7 = (@7+@9) */
  w7 += w9;
  /* #22: @7 = (@1*@7) */
  w7  = (w1*w7);
  /* #23: @9 = input[0][1] */
  w9 = arg[0] ? arg[0][1] : 0;
  /* #24: @7 = (@7+@9) */
  w7 += w9;
  /* #25: output[0][1] = @7 */
  if (res[0]) res[0][1] = w7;
  /* #26: @7 = input[1][0] */
  w7 = arg[1] ? arg[1][0] : 0;
  /* #27: output[0][2] = @7 */
  if (res[0]) res[0][2] = w7;
  /* #28: @7 = input[1][1] */
  w7 = arg[1] ? arg[1][1] : 0;
  /* #29: output[0][3] = @7 */
  if (res[0]) res[0][3] = w7;
  /* #30: @7 = input[1][2] */
  w7 = arg[1] ? arg[1][2] : 0;
  /* #31: output[0][4] = @7 */
  if (res[0]) res[0][4] = w7;
  /* #32: @10 = zeros(3x5,3nz) */
  casadi_clear(w10, 3);
  /* #33: @11 = 00 */
  /* #34: @12 = 00 */
  /* #35: @13 = ones(3x1) */
  casadi_fill(w13, 3, 1.);
  /* #36: {@7, @9, @14} = vertsplit(@13) */
  w7 = w13[0];
  w9 = w13[1];
  w14 = w13[2];
  /* #37: @13 = vertcat(@11, @12, @7, @9, @14) */
  rr=w13;
  *rr++ = w7;
  *rr++ = w9;
  *rr++ = w14;
  /* #38: @15 = @13[:3] */
  for (rr=w15, ss=w13+0; ss!=w13+3; ss+=1) *rr++ = *ss;
  /* #39: (@10[:3] = @15) */
  for (rr=w10+0, ss=w15; rr!=w10+3; rr+=1) *rr = *ss++;
  /* #40: @15 = @10' */
  casadi_trans(w10,casadi_s1, w15, casadi_s0, iw);
  /* #41: @16 = zeros(35x5,34nz) */
  casadi_clear(w16, 34);
  /* #42: @17 = ones(5x1,4nz) */
  casadi_fill(w17, 4, 1.);
  /* #43: {@7, NULL, NULL, NULL, NULL} = vertsplit(@17) */
  w7 = w17[0];
  /* #44: (@16[0] = @7) */
  for (rr=w16+0, ss=(&w7); rr!=w16+1; rr+=1) *rr = *ss++;
  /* #45: @6 = zeros(16x1) */
  casadi_clear(w6, 16);
  /* #46: @18 = @5' */
  for (i=0, rr=w18, cs=w5; i<16; ++i) for (j=0; j<2; ++j) rr[i+j*16] = *cs++;
  /* #47: @3 = zeros(2x1) */
  casadi_clear(w3, 2);
  /* #48: @0 = (@0*@7) */
  w0 *= w7;
  /* #49: @2 = (@2*@0) */
  w2 *= w0;
  /* #50: (@3[0] += @2) */
  for (rr=w3+0, ss=(&w2); rr!=w3+1; rr+=1) *rr += *ss++;
  /* #51: @6 = mac(@18,@3,@6) */
  for (i=0, rr=w6; i<1; ++i) for (j=0; j<16; ++j, ++rr) for (k=0, ss=w18+j, tt=w3+i*2; k<2; ++k) *rr += ss[k*16]**tt++;
  /* #52: @2 = @6[0] */
  for (rr=(&w2), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #53: (@16[1] = @2) */
  for (rr=w16+1, ss=(&w2); rr!=w16+2; rr+=1) *rr = *ss++;
  /* #54: @2 = @6[1] */
  for (rr=(&w2), ss=w6+1; ss!=w6+2; ss+=1) *rr++ = *ss;
  /* #55: (@16[2] = @2) */
  for (rr=w16+2, ss=(&w2); rr!=w16+3; rr+=1) *rr = *ss++;
  /* #56: @2 = @6[2] */
  for (rr=(&w2), ss=w6+2; ss!=w6+3; ss+=1) *rr++ = *ss;
  /* #57: (@16[3] = @2) */
  for (rr=w16+3, ss=(&w2); rr!=w16+4; rr+=1) *rr = *ss++;
  /* #58: @2 = @6[3] */
  for (rr=(&w2), ss=w6+3; ss!=w6+4; ss+=1) *rr++ = *ss;
  /* #59: (@16[4] = @2) */
  for (rr=w16+4, ss=(&w2); rr!=w16+5; rr+=1) *rr = *ss++;
  /* #60: @2 = @6[4] */
  for (rr=(&w2), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #61: (@16[5] = @2) */
  for (rr=w16+5, ss=(&w2); rr!=w16+6; rr+=1) *rr = *ss++;
  /* #62: @2 = @6[5] */
  for (rr=(&w2), ss=w6+5; ss!=w6+6; ss+=1) *rr++ = *ss;
  /* #63: (@16[6] = @2) */
  for (rr=w16+6, ss=(&w2); rr!=w16+7; rr+=1) *rr = *ss++;
  /* #64: @2 = @6[6] */
  for (rr=(&w2), ss=w6+6; ss!=w6+7; ss+=1) *rr++ = *ss;
  /* #65: (@16[7] = @2) */
  for (rr=w16+7, ss=(&w2); rr!=w16+8; rr+=1) *rr = *ss++;
  /* #66: @2 = @6[7] */
  for (rr=(&w2), ss=w6+7; ss!=w6+8; ss+=1) *rr++ = *ss;
  /* #67: (@16[8] = @2) */
  for (rr=w16+8, ss=(&w2); rr!=w16+9; rr+=1) *rr = *ss++;
  /* #68: @2 = @6[8] */
  for (rr=(&w2), ss=w6+8; ss!=w6+9; ss+=1) *rr++ = *ss;
  /* #69: (@16[9] = @2) */
  for (rr=w16+9, ss=(&w2); rr!=w16+10; rr+=1) *rr = *ss++;
  /* #70: @2 = @6[9] */
  for (rr=(&w2), ss=w6+9; ss!=w6+10; ss+=1) *rr++ = *ss;
  /* #71: (@16[10] = @2) */
  for (rr=w16+10, ss=(&w2); rr!=w16+11; rr+=1) *rr = *ss++;
  /* #72: @2 = @6[10] */
  for (rr=(&w2), ss=w6+10; ss!=w6+11; ss+=1) *rr++ = *ss;
  /* #73: (@16[11] = @2) */
  for (rr=w16+11, ss=(&w2); rr!=w16+12; rr+=1) *rr = *ss++;
  /* #74: @2 = @6[11] */
  for (rr=(&w2), ss=w6+11; ss!=w6+12; ss+=1) *rr++ = *ss;
  /* #75: (@16[12] = @2) */
  for (rr=w16+12, ss=(&w2); rr!=w16+13; rr+=1) *rr = *ss++;
  /* #76: @2 = @6[12] */
  for (rr=(&w2), ss=w6+12; ss!=w6+13; ss+=1) *rr++ = *ss;
  /* #77: (@16[13] = @2) */
  for (rr=w16+13, ss=(&w2); rr!=w16+14; rr+=1) *rr = *ss++;
  /* #78: @2 = @6[13] */
  for (rr=(&w2), ss=w6+13; ss!=w6+14; ss+=1) *rr++ = *ss;
  /* #79: (@16[14] = @2) */
  for (rr=w16+14, ss=(&w2); rr!=w16+15; rr+=1) *rr = *ss++;
  /* #80: @2 = @6[14] */
  for (rr=(&w2), ss=w6+14; ss!=w6+15; ss+=1) *rr++ = *ss;
  /* #81: (@16[15] = @2) */
  for (rr=w16+15, ss=(&w2); rr!=w16+16; rr+=1) *rr = *ss++;
  /* #82: @2 = @6[15] */
  for (rr=(&w2), ss=w6+15; ss!=w6+16; ss+=1) *rr++ = *ss;
  /* #83: (@16[16] = @2) */
  for (rr=w16+16, ss=(&w2); rr!=w16+17; rr+=1) *rr = *ss++;
  /* #84: @2 = ones(5x1,1nz) */
  w2 = 1.;
  /* #85: {NULL, @0, NULL, NULL, NULL} = vertsplit(@2) */
  w0 = w2;
  /* #86: (@16[17] = @0) */
  for (rr=w16+17, ss=(&w0); rr!=w16+18; rr+=1) *rr = *ss++;
  /* #87: @6 = zeros(16x1) */
  casadi_clear(w6, 16);
  /* #88: @18 = @5' */
  for (i=0, rr=w18, cs=w5; i<16; ++i) for (j=0; j<2; ++j) rr[i+j*16] = *cs++;
  /* #89: @3 = zeros(2x1) */
  casadi_clear(w3, 2);
  /* #90: @1 = (@1*@0) */
  w1 *= w0;
  /* #91: @8 = (@8*@1) */
  w8 *= w1;
  /* #92: (@3[1] += @8) */
  for (rr=w3+1, ss=(&w8); rr!=w3+2; rr+=1) *rr += *ss++;
  /* #93: @6 = mac(@18,@3,@6) */
  for (i=0, rr=w6; i<1; ++i) for (j=0; j<16; ++j, ++rr) for (k=0, ss=w18+j, tt=w3+i*2; k<2; ++k) *rr += ss[k*16]**tt++;
  /* #94: @8 = @6[0] */
  for (rr=(&w8), ss=w6+0; ss!=w6+1; ss+=1) *rr++ = *ss;
  /* #95: (@16[18] = @8) */
  for (rr=w16+18, ss=(&w8); rr!=w16+19; rr+=1) *rr = *ss++;
  /* #96: @8 = @6[1] */
  for (rr=(&w8), ss=w6+1; ss!=w6+2; ss+=1) *rr++ = *ss;
  /* #97: (@16[19] = @8) */
  for (rr=w16+19, ss=(&w8); rr!=w16+20; rr+=1) *rr = *ss++;
  /* #98: @8 = @6[2] */
  for (rr=(&w8), ss=w6+2; ss!=w6+3; ss+=1) *rr++ = *ss;
  /* #99: (@16[20] = @8) */
  for (rr=w16+20, ss=(&w8); rr!=w16+21; rr+=1) *rr = *ss++;
  /* #100: @8 = @6[3] */
  for (rr=(&w8), ss=w6+3; ss!=w6+4; ss+=1) *rr++ = *ss;
  /* #101: (@16[21] = @8) */
  for (rr=w16+21, ss=(&w8); rr!=w16+22; rr+=1) *rr = *ss++;
  /* #102: @8 = @6[4] */
  for (rr=(&w8), ss=w6+4; ss!=w6+5; ss+=1) *rr++ = *ss;
  /* #103: (@16[22] = @8) */
  for (rr=w16+22, ss=(&w8); rr!=w16+23; rr+=1) *rr = *ss++;
  /* #104: @8 = @6[5] */
  for (rr=(&w8), ss=w6+5; ss!=w6+6; ss+=1) *rr++ = *ss;
  /* #105: (@16[23] = @8) */
  for (rr=w16+23, ss=(&w8); rr!=w16+24; rr+=1) *rr = *ss++;
  /* #106: @8 = @6[6] */
  for (rr=(&w8), ss=w6+6; ss!=w6+7; ss+=1) *rr++ = *ss;
  /* #107: (@16[24] = @8) */
  for (rr=w16+24, ss=(&w8); rr!=w16+25; rr+=1) *rr = *ss++;
  /* #108: @8 = @6[7] */
  for (rr=(&w8), ss=w6+7; ss!=w6+8; ss+=1) *rr++ = *ss;
  /* #109: (@16[25] = @8) */
  for (rr=w16+25, ss=(&w8); rr!=w16+26; rr+=1) *rr = *ss++;
  /* #110: @8 = @6[8] */
  for (rr=(&w8), ss=w6+8; ss!=w6+9; ss+=1) *rr++ = *ss;
  /* #111: (@16[26] = @8) */
  for (rr=w16+26, ss=(&w8); rr!=w16+27; rr+=1) *rr = *ss++;
  /* #112: @8 = @6[9] */
  for (rr=(&w8), ss=w6+9; ss!=w6+10; ss+=1) *rr++ = *ss;
  /* #113: (@16[27] = @8) */
  for (rr=w16+27, ss=(&w8); rr!=w16+28; rr+=1) *rr = *ss++;
  /* #114: @8 = @6[10] */
  for (rr=(&w8), ss=w6+10; ss!=w6+11; ss+=1) *rr++ = *ss;
  /* #115: (@16[28] = @8) */
  for (rr=w16+28, ss=(&w8); rr!=w16+29; rr+=1) *rr = *ss++;
  /* #116: @8 = @6[11] */
  for (rr=(&w8), ss=w6+11; ss!=w6+12; ss+=1) *rr++ = *ss;
  /* #117: (@16[29] = @8) */
  for (rr=w16+29, ss=(&w8); rr!=w16+30; rr+=1) *rr = *ss++;
  /* #118: @8 = @6[12] */
  for (rr=(&w8), ss=w6+12; ss!=w6+13; ss+=1) *rr++ = *ss;
  /* #119: (@16[30] = @8) */
  for (rr=w16+30, ss=(&w8); rr!=w16+31; rr+=1) *rr = *ss++;
  /* #120: @8 = @6[13] */
  for (rr=(&w8), ss=w6+13; ss!=w6+14; ss+=1) *rr++ = *ss;
  /* #121: (@16[31] = @8) */
  for (rr=w16+31, ss=(&w8); rr!=w16+32; rr+=1) *rr = *ss++;
  /* #122: @8 = @6[14] */
  for (rr=(&w8), ss=w6+14; ss!=w6+15; ss+=1) *rr++ = *ss;
  /* #123: (@16[32] = @8) */
  for (rr=w16+32, ss=(&w8); rr!=w16+33; rr+=1) *rr = *ss++;
  /* #124: @8 = @6[15] */
  for (rr=(&w8), ss=w6+15; ss!=w6+16; ss+=1) *rr++ = *ss;
  /* #125: (@16[33] = @8) */
  for (rr=w16+33, ss=(&w8); rr!=w16+34; rr+=1) *rr = *ss++;
  /* #126: @19 = @16' */
  casadi_trans(w16,casadi_s3, w19, casadi_s2, iw);
  /* #127: @20 = horzcat(@15, @19) */
  rr=w20;
  for (i=0, cs=w15; i<3; ++i) *rr++ = *cs++;
  for (i=0, cs=w19; i<34; ++i) *rr++ = *cs++;
  /* #128: @21 = @20' */
  casadi_trans(w20,casadi_s5, w21, casadi_s4, iw);
  /* #129: output[1][0] = @21 */
  casadi_copy(w21, 37, res[1]);
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
    case 0: return casadi_s6;
    case 1: return casadi_s7;
    case 2: return casadi_s8;
    case 3: return casadi_s9;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* LSTM_v21_MHE_cost_y_fun_jac_ut_xt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s10;
    case 1: return casadi_s4;
    case 2: return casadi_s11;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int LSTM_v21_MHE_cost_y_fun_jac_ut_xt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 9;
  if (sz_res) *sz_res = 8;
  if (sz_iw) *sz_iw = 36;
  if (sz_w) *sz_w = 262;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif

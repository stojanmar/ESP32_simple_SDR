#define Nam 256
float AM_FIR_coeffs[256]={
-0.000000e+00,
-2.568094e-10,
4.092468e-09,
-2.057933e-08,
6.442855e-08,
-1.553820e-07,
3.173730e-07,
-5.774684e-07,
9.646075e-07,
-1.508159e-06,
2.236318e-06,
-3.174372e-06,
4.342875e-06,
-5.755746e-06,
7.418364e-06,
-9.325675e-06,
1.146038e-05,
-1.379123e-05,
1.627153e-05,
-1.883788e-05,
2.140913e-05,
-2.388584e-05,
2.614996e-05,
-2.806516e-05,
2.947751e-05,
-3.021683e-05,
3.009854e-05,
-2.892623e-05,
2.649477e-05,
-2.259411e-05,
1.701373e-05,
-9.547633e-06,
0.000000e+00,
1.180874e-05,
-2.603530e-05,
4.280669e-05,
-6.221299e-05,
8.430014e-05,
-1.090627e-04,
1.364367e-04,
-1.662929e-04,
1.984310e-04,
-2.325734e-04,
2.683612e-04,
-3.053505e-04,
3.430102e-04,
-3.807209e-04,
4.177761e-04,
-4.533844e-04,
4.866744e-04,
-5.167007e-04,
5.424525e-04,
-5.628644e-04,
5.768292e-04,
-5.832123e-04,
5.808691e-04,
-5.686637e-04,
5.454891e-04,
-5.102898e-04,
4.620848e-04,
-3.999924e-04,
3.232552e-04,
-2.312655e-04,
1.235911e-04,
0.000000e+00,
-1.395154e-04,
2.947155e-04,
-4.650927e-04,
6.498529e-04,
-8.478992e-04,
1.057820e-03,
-1.277879e-03,
1.506011e-03,
-1.739822e-03,
1.976589e-03,
-2.213276e-03,
2.446541e-03,
-2.672759e-03,
2.888044e-03,
-3.088282e-03,
3.269161e-03,
-3.426214e-03,
3.554861e-03,
-3.650455e-03,
3.708340e-03,
-3.723899e-03,
3.692615e-03,
-3.610129e-03,
3.472302e-03,
-3.275274e-03,
3.015525e-03,
-2.689935e-03,
2.295840e-03,
-1.831085e-03,
1.294077e-03,
-6.838294e-04,
0.000000e+00,
7.570721e-04,
-1.586341e-03,
2.486033e-03,
-3.453637e-03,
4.485899e-03,
-5.578827e-03,
6.727708e-03,
-7.927127e-03,
9.171002e-03,
-1.045262e-02,
1.176470e-02,
-1.309943e-02,
1.444854e-02,
-1.580337e-02,
1.715497e-02,
-1.849414e-02,
1.981156e-02,
-2.109786e-02,
2.234369e-02,
-2.353988e-02,
2.467744e-02,
-2.574775e-02,
2.674257e-02,
-2.765418e-02,
2.847543e-02,
-2.919983e-02,
2.982163e-02,
-3.033586e-02,
3.073838e-02,
-3.102597e-02,
3.119632e-02,
9.686897e-01,
3.118079e-02,
-3.099509e-02,
3.069249e-02,
-3.027548e-02,
2.974744e-02,
-2.911265e-02,
2.837622e-02,
-2.754405e-02,
2.662273e-02,
-2.561951e-02,
2.454219e-02,
-2.339907e-02,
2.219884e-02,
-2.095047e-02,
1.966319e-02,
-1.834630e-02,
1.700916e-02,
-1.566104e-02,
1.431106e-02,
-1.296811e-02,
1.164075e-02,
-1.033713e-02,
9.064951e-03,
-7.831364e-03,
6.642946e-03,
-5.505635e-03,
4.424701e-03,
-3.404708e-03,
2.449501e-03,
-1.562189e-03,
7.451428e-04,
0.000000e+00,
-6.723203e-04,
1.271597e-03,
-1.798280e-03,
2.253453e-03,
-2.638791e-03,
2.956520e-03,
-3.209360e-03,
3.400473e-03,
-3.533408e-03,
3.612039e-03,
-3.640505e-03,
3.623151e-03,
-3.564470e-03,
3.469040e-03,
-3.341471e-03,
3.186351e-03,
-3.008193e-03,
2.811392e-03,
-2.600181e-03,
2.378591e-03,
-2.150421e-03,
1.919207e-03,
-1.688202e-03,
1.460356e-03,
-1.238306e-03,
1.024363e-03,
-8.205154e-04,
6.284258e-04,
-4.494392e-04,
2.845919e-04,
-1.346249e-04,
0.000000e+00,
1.190814e-04,
-2.226572e-04,
3.109819e-04,
-3.845029e-04,
4.438353e-04,
-4.897372e-04,
5.230843e-04,
-5.448458e-04,
5.560599e-04,
-5.578113e-04,
5.512098e-04,
-5.373705e-04,
5.173955e-04,
-4.923579e-04,
4.632875e-04,
-4.311590e-04,
3.968816e-04,
-3.612913e-04,
3.251450e-04,
-2.891165e-04,
2.537940e-04,
-2.196804e-04,
1.871937e-04,
-1.566696e-04,
1.283656e-04,
-1.024649e-04,
7.908270e-05,
-5.827182e-05,
4.002954e-05,
-2.430451e-05,
1.100381e-05,
0.000000e+00,
-8.861933e-06,
1.575795e-05,
-2.087863e-05,
2.442335e-05,
-2.659492e-05,
2.759481e-05,
-2.761902e-05,
2.685446e-05,
-2.547606e-05,
2.364445e-05,
-2.150423e-05,
1.918282e-05,
-1.678985e-05,
1.441705e-05,
-1.213850e-05,
1.001135e-05,
-8.076865e-06,
6.361677e-06,
-4.879328e-06,
3.631935e-06,
-2.611987e-06,
1.804191e-06,
-1.187343e-06,
7.361646e-07,
-4.230782e-07,
2.198744e-07,
-9.925028e-08,
3.618580e-08,
-9.136376e-09,
1.022445e-09,
6.520997e-23
};

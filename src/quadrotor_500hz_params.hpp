#pragma once

#include <tinympc/types.hpp>

tinytype rho_value = 5.0;

tinytype Adyn_data[NSTATES*NSTATES] = {
  1.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000392,	0.0000000,	0.0020000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	1.0000000,	0.0000000,	-0.0000392,	0.0000000,	0.0000000,	0.0000000,	0.0020000,	0.0000000,	-0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	1.0000000,	-0.0000000,	-0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0020000,	-0.0000000,	-0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	1.0000000,	-0.0000000,	-0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0010000,	0.0000000,	-0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	-0.0000000,	0.0010000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	-0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	-0.0000000,	0.0010000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0392400,	0.0000000,	1.0000000,	0.0000000,	0.0000000,	-0.0000000,	0.0000196,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	-0.0392400,	0.0000000,	0.0000000,	0.0000000,	1.0000000,	0.0000000,	-0.0000196,	-0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	-0.0000000,	-0.0000000,	0.0000000,	0.0000000,	0.0000000,	1.0000000,	-0.0000000,	-0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	1.0000000,	0.0000000,	-0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	-0.0000000,	1.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	-0.0000000,	1.0000000	
};

tinytype Bdyn_data[NSTATES*NINPUTS] = {
  -0.0000000,	0.0000000,	0.0000000,	-0.0000000,	
  0.0000000,	0.0000000,	-0.0000000,	-0.0000000,	
  0.0000089,	0.0000089,	0.0000089,	0.0000089,	
  -0.0002754,	-0.0003032,	0.0002757,	0.0003029,	
  -0.0002767,	0.0003043,	0.0002776,	-0.0003051,	
  0.0000197,	-0.0000072,	-0.0000278,	0.0000153,	
  -0.0000036,	0.0000040,	0.0000036,	-0.0000040,	
  0.0000036,	0.0000040,	-0.0000036,	-0.0000040,	
  0.0089182,	0.0089182,	0.0089182,	0.0089182,	
  -0.5507092,	-0.6064681,	0.5513253,	0.6058520,	
  -0.5534140,	0.6085568,	0.5551390,	-0.6102818,	
  0.0394954,	-0.0144473,	-0.0556875,	0.0306394	
};

tinytype Kinf_data[NINPUTS*NSTATES] = {
  -1.1245968,	1.0867975,	1.6877046,	-4.1638247,	-4.3876673,	-3.1999718,	-0.7807755,	0.7495461,	0.6583215,	-0.2970759,	-0.3205151,	-0.6553573,	
  1.0813111,	0.9872427,	1.6877046,	-3.6776283,	4.2203475,	3.2676825,	0.7507846,	0.6740303,	0.6583215,	-0.2525133,	0.3085779,	0.6646935,	
  0.9841783,	-1.0307673,	1.6877046,	3.8457904,	3.5912945,	-3.4290110,	0.6671261,	-0.7041802,	0.6583215,	0.2645074,	0.2390048,	-0.6869080,	
  -0.9408926,	-1.0432730,	1.6877046,	3.9956626,	-3.4239747,	3.3613003,	-0.6371352,	-0.7193961,	0.6583215,	0.2850817,	-0.2270677,	0.6775719	
};

tinytype Pinf_data[NSTATES*NSTATES] = {
  36050.4477947,	-11.2265684,	0.0000000,	44.9389983,	27691.8344230,	525.8935626,	10092.1077810,	-7.9688205,	0.0000000,	3.0984593,	34.7458081,	80.6618342,	
  -11.2265684,	36036.4084160,	0.0000000,	-27633.3174284,	-44.9440850,	-210.0522752,	-7.9692432,	10081.9092734,	0.0000000,	-30.7235827,	-3.0989437,	-32.2216390,	
  0.0000000,	0.0000000,	20478.6290868,	-0.0000000,	-0.0000000,	-0.0000000,	-0.0000000,	0.0000000,	1723.5618402,	-0.0000000,	-0.0000000,	-0.0000000,	
  44.9389983,	-27633.3174284,	-0.0000000,	103306.7134125,	201.7661709,	1075.5062486,	33.3187731,	-18879.6734308,	-0.0000000,	117.1582149,	15.4139870,	181.4381931,	
  27691.8344230,	-44.9440850,	0.0000000,	201.7661709,	103594.8643102,	2691.5948022,	18925.0166462,	-33.3206695,	0.0000000,	15.4134921,	140.0905636,	453.9932712,	
  525.8935626,	-210.0522752,	-0.0000000,	1075.5062486,	2691.5948022,	81956.3504613,	411.6285994,	-164.4435178,	-0.0000000,	91.1698921,	228.1191113,	3752.9481367,	
  10092.1077810,	-7.9692432,	0.0000000,	33.3187731,	18925.0166462,	411.6285994,	5514.0702052,	-5.7565238,	0.0000000,	2.3678678,	24.4128127,	65.0611352,	
  -7.9688205,	10081.9092734,	0.0000000,	-18879.6734308,	-33.3206695,	-164.4435178,	-5.7565238,	5506.4653625,	0.0000000,	-21.1488097,	-2.3680895,	-25.9951257,	
  0.0000000,	0.0000000,	1723.5618402,	-0.0000000,	-0.0000000,	-0.0000000,	0.0000000,	0.0000000,	675.0846978,	-0.0000000,	-0.0000000,	-0.0000000,	
  3.0984593,	-30.7235827,	-0.0000000,	117.1582149,	15.4134921,	91.1698921,	2.3678678,	-21.1488097,	-0.0000000,	11.8408268,	1.4361956,	18.4805698,	
  34.7458081,	-3.0989437,	-0.0000000,	15.4139870,	140.0905636,	228.1191113,	24.4128127,	-2.3680895,	-0.0000000,	1.4361956,	14.1926551,	46.2284337,	
  80.6618342,	-32.2216390,	-0.0000000,	181.4381931,	453.9932712,	3752.9481367,	65.0611352,	-25.9951257,	-0.0000000,	18.4805698,	46.2284337,	762.5436667	
};

tinytype Quu_inv_data[NINPUTS*NINPUTS] = {
  0.0757044,	-0.0000543,	0.0330104,	-0.0001520,	
  -0.0000543,	0.0739098,	-0.0000957,	0.0347486,	
  0.0330104,	-0.0000957,	0.0755032,	0.0000905,	
  -0.0001520,	0.0347486,	0.0000905,	0.0738214	
};

tinytype AmBKt_data[NSTATES*NSTATES] = {
  1.0000000,	-0.0000000,	-0.0000000,	0.0000319,	-0.0012005,	0.0000718,	-0.0000157,	-0.0000004,	-0.0000000,	0.0638941,	-2.4009723,	0.1436732,	
  -0.0000000,	1.0000000,	-0.0000000,	0.0011988,	-0.0000319,	-0.0000270,	-0.0000004,	-0.0000157,	-0.0000000,	2.3975977,	-0.0638157,	-0.0540962,	
  -0.0000000,	-0.0000000,	0.9999398,	-0.0000000,	0.0000000,	-0.0000000,	0.0000000,	-0.0000000,	-0.0602050,	0.0000000,	-0.0000000,	-0.0000000,	
  0.0000000,	-0.0000392,	-0.0000000,	0.9954678,	0.0001186,	0.0001015,	0.0000016,	-0.0391807,	-0.0000000,	-9.0644825,	0.2372587,	0.2030582,	
  0.0000392,	-0.0000000,	-0.0000000,	0.0001188,	0.9954601,	0.0002696,	0.0391806,	-0.0000016,	-0.0000000,	0.2376278,	-9.0797751,	0.5391641,	
  0.0000000,	-0.0000000,	-0.0000000,	0.0000368,	0.0000977,	0.9999398,	0.0000013,	-0.0000005,	-0.0000000,	0.0735410,	0.1954383,	-0.1203479,	
  0.0020000,	-0.0000000,	-0.0000000,	0.0000218,	-0.0008241,	0.0000492,	0.9999892,	-0.0000003,	-0.0000000,	0.0435528,	-1.6481669,	0.0983559,	
  -0.0000000,	0.0020000,	0.0000000,	0.0008228,	-0.0000217,	-0.0000185,	-0.0000003,	0.9999892,	0.0000000,	1.6456397,	-0.0434929,	-0.0370379,	
  0.0000000,	-0.0000000,	0.0019765,	-0.0000000,	0.0000000,	-0.0000000,	0.0000000,	-0.0000000,	0.9765159,	-0.0000000,	-0.0000000,	-0.0000000,	
  0.0000000,	-0.0000000,	-0.0000000,	0.0006824,	0.0000082,	0.0000070,	0.0000001,	-0.0000155,	-0.0000000,	0.3647093,	0.0164046,	0.0140800,	
  0.0000000,	-0.0000000,	-0.0000000,	0.0000082,	0.0006818,	0.0000187,	0.0000155,	-0.0000001,	-0.0000000,	0.0164321,	0.3635790,	0.0373838,	
  0.0000000,	-0.0000000,	-0.0000000,	0.0000052,	0.0000138,	0.0009882,	0.0000002,	-0.0000001,	-0.0000000,	0.0104055,	0.0276515,	0.9764740	
};

tinytype coeff_d2p_data[NSTATES*NINPUTS] = {
  0.0000000,	-0.0000000,	-0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	-0.0000000,	-0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  -0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	-0.0000000,	-0.0000000,	0.0000000,	
  -0.0000000,	0.0000000,	-0.0000000,	0.0000000,	
  0.0000000,	-0.0000000,	-0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	-0.0000000,	-0.0000000,	
  0.0000000,	-0.0000000,	0.0000000,	0.0000000,	
  -0.0000000,	0.0000000,	0.0000000,	-0.0000000,	
  0.0000000,	-0.0000000,	-0.0000000,	0.0000000,	
  -0.0000000,	-0.0000000,	-0.0000000,	0.0000000	
};

tinytype Q_data[NSTATES*NSTATES]= {
  100.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	100.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	100.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	4.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	4.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	400.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	4.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	4.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	4.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	2.0408163,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	2.0408163,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	4.0000000	
};

tinytype Qf_data[NSTATES*NSTATES]= {
  100.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	100.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	100.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	4.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	4.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	400.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	4.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	4.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	4.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	2.0408163,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	2.0408163,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	4.0000000	
};

tinytype R_data[NINPUTS*NINPUTS]= {
  4.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	4.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	4.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	4.0000000	
};

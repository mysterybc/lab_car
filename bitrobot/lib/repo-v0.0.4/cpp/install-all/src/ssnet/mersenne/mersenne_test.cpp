#ifdef BUILD_MERSENNE_TEST
#include "mersenne.h"
#include <cstdio>

std::uint32_t data[100] = {
	2395582193, 
	4192163422,
	2358540478, 
	3154787438,
    2475695097,
    3562712708,
    923106816,
    2792834826,
    3205459683,
    3103856113,
    2236802317,
    3226825633,
    3812791491,
    1425401686,
    3766964285,
    345579324,
    2625510249,
    3005511130,
    1838055281,
    2057880244,
    1360048050,
    3066482394,
    3490513334,
    2255464394,
    2268140653,
    2894286385,
    2598140694,
    3539698717,
    940569181,
    922663331,
    972060469,
    1980934649,
    244910061,
    2849889199,
    3120927626,
    2650415968,
    270524570,
    3426027076,
    3752018081,
    1925592060,
    1140147559,
    793260230,
    120918683,
    2569763803,
    958313339,
    2043474778,
    1834163655,
    3367689383,
    2563413705,
    1183398455,
    1334431025,
    340142212,
    2741792464,
    188214940,
    2467081161,
    3811263567,
    2669875737,
    1282349651,
    314413019,
    2930483296,
    2632268456,
    3734442474,
    2262490497,
    440432617,
    1354586874,
    1807867563,
    2657596511,
    3819576516,
    1939493540,
    3852540151,
    2314460322,
    1077654901,
    3301738264,
    382464060,
    2391063027,
    1745712164,
    323767806,
    373027178,
    3448135040,
    185900987,
    2737251606,
    3567242406,
    2517409215,
    3071591501,
    2827833520,
    1896986803,
    1954988984,
    3837424926,
    397534821,
    2490786580,
    2189909208,
    2884556998,
    298063575,
    1113727933,
    1368254831,
    4176015086,
    1459500594,
    3171036527,
    3487092297,
    4082275776
};


int main() {
	MersenneTwister rand_mt(48295);
	for (int i=0;i<100;++i) {
        std::uint32_t val = rand_mt();
        if (val != data[i]) {
            printf("Error: Found mismatched value i = %d:  %u (computed) != %u (windows record)\n", 
                i, val, data[i]);
            return -1;
        }
	}
    puts("Test passed. Value matched");
	return 0;
}
#endif
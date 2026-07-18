package b2_test

import (
	"fmt"
	"math"
	"sort"
	"strings"
	"testing"

	"github.com/setanarut/b2"
)

var expectedShapeCast string = "hit = false, iters = 3, lambda = 1, distance = 7.040063920164362"

var expected string = `0(00_ground): 0.000 0.000 0.000
0(01_colinearground): 0.000 0.000 0.000
0(02_chainshape): 0.000 0.000 0.785
0(03_squaretiles): 0.000 0.000 0.000
0(04_edgeloopsquare): 0.000 0.000 0.000
0(05_edgelooppoly): -10.000 4.000 0.000
0(06_squarecharacter1): -3.000 7.997 0.000
0(07_squarecharacter2): -5.000 4.997 0.000
0(08_hexagoncharacter): -5.000 7.997 0.000
0(09_circlecharacter1): 3.000 4.997 0.000
0(10_circlecharacter2): -7.000 5.997 0.000
1(00_ground): 0.000 0.000 0.000
1(01_colinearground): 0.000 0.000 0.000
1(02_chainshape): 0.000 0.000 0.785
1(03_squaretiles): 0.000 0.000 0.000
1(04_edgeloopsquare): 0.000 0.000 0.000
1(05_edgelooppoly): -10.000 4.000 0.000
1(06_squarecharacter1): -3.000 7.992 0.000
1(07_squarecharacter2): -5.000 4.992 0.000
1(08_hexagoncharacter): -5.000 7.992 0.000
1(09_circlecharacter1): 3.000 4.992 0.000
1(10_circlecharacter2): -7.000 5.992 0.000
2(00_ground): 0.000 0.000 0.000
2(01_colinearground): 0.000 0.000 0.000
2(02_chainshape): 0.000 0.000 0.785
2(03_squaretiles): 0.000 0.000 0.000
2(04_edgeloopsquare): 0.000 0.000 0.000
2(05_edgelooppoly): -10.000 4.000 0.000
2(06_squarecharacter1): -3.000 7.983 0.000
2(07_squarecharacter2): -5.000 4.983 0.000
2(08_hexagoncharacter): -5.000 7.983 0.000
2(09_circlecharacter1): 3.000 4.983 0.000
2(10_circlecharacter2): -7.000 5.983 0.000
3(00_ground): 0.000 0.000 0.000
3(01_colinearground): 0.000 0.000 0.000
3(02_chainshape): 0.000 0.000 0.785
3(03_squaretiles): 0.000 0.000 0.000
3(04_edgeloopsquare): 0.000 0.000 0.000
3(05_edgelooppoly): -10.000 4.000 0.000
3(06_squarecharacter1): -3.000 7.972 0.000
3(07_squarecharacter2): -5.000 4.972 0.000
3(08_hexagoncharacter): -5.000 7.972 0.000
3(09_circlecharacter1): 3.000 4.972 0.000
3(10_circlecharacter2): -7.000 5.972 0.000
4(00_ground): 0.000 0.000 0.000
4(01_colinearground): 0.000 0.000 0.000
4(02_chainshape): 0.000 0.000 0.785
4(03_squaretiles): 0.000 0.000 0.000
4(04_edgeloopsquare): 0.000 0.000 0.000
4(05_edgelooppoly): -10.000 4.000 0.000
4(06_squarecharacter1): -3.000 7.958 0.000
4(07_squarecharacter2): -5.000 4.958 0.000
4(08_hexagoncharacter): -5.000 7.958 0.000
4(09_circlecharacter1): 3.000 4.958 0.000
4(10_circlecharacter2): -7.000 5.958 0.000
5(00_ground): 0.000 0.000 0.000
5(01_colinearground): 0.000 0.000 0.000
5(02_chainshape): 0.000 0.000 0.785
5(03_squaretiles): 0.000 0.000 0.000
5(04_edgeloopsquare): 0.000 0.000 0.000
5(05_edgelooppoly): -10.000 4.000 0.000
5(06_squarecharacter1): -3.000 7.942 0.000
5(07_squarecharacter2): -5.000 4.942 0.000
5(08_hexagoncharacter): -5.000 7.942 0.000
5(09_circlecharacter1): 3.000 4.942 0.000
5(10_circlecharacter2): -7.000 5.942 0.000
6(00_ground): 0.000 0.000 0.000
6(01_colinearground): 0.000 0.000 0.000
6(02_chainshape): 0.000 0.000 0.785
6(03_squaretiles): 0.000 0.000 0.000
6(04_edgeloopsquare): 0.000 0.000 0.000
6(05_edgelooppoly): -10.000 4.000 0.000
6(06_squarecharacter1): -3.000 7.922 0.000
6(07_squarecharacter2): -5.000 4.922 0.000
6(08_hexagoncharacter): -5.000 7.922 0.000
6(09_circlecharacter1): 3.000 4.922 0.000
6(10_circlecharacter2): -7.000 5.922 0.000
7(00_ground): 0.000 0.000 0.000
7(01_colinearground): 0.000 0.000 0.000
7(02_chainshape): 0.000 0.000 0.785
7(03_squaretiles): 0.000 0.000 0.000
7(04_edgeloopsquare): 0.000 0.000 0.000
7(05_edgelooppoly): -10.000 4.000 0.000
7(06_squarecharacter1): -3.000 7.900 0.000
7(07_squarecharacter2): -5.000 4.900 0.000
7(08_hexagoncharacter): -5.000 7.900 0.000
7(09_circlecharacter1): 3.000 4.900 0.000
7(10_circlecharacter2): -7.000 5.900 0.000
8(00_ground): 0.000 0.000 0.000
8(01_colinearground): 0.000 0.000 0.000
8(02_chainshape): 0.000 0.000 0.785
8(03_squaretiles): 0.000 0.000 0.000
8(04_edgeloopsquare): 0.000 0.000 0.000
8(05_edgelooppoly): -10.000 4.000 0.000
8(06_squarecharacter1): -3.000 7.875 0.000
8(07_squarecharacter2): -5.000 4.875 0.000
8(08_hexagoncharacter): -5.000 7.875 0.000
8(09_circlecharacter1): 3.000 4.875 0.000
8(10_circlecharacter2): -7.000 5.875 0.000
9(00_ground): 0.000 0.000 0.000
9(01_colinearground): 0.000 0.000 0.000
9(02_chainshape): 0.000 0.000 0.785
9(03_squaretiles): 0.000 0.000 0.000
9(04_edgeloopsquare): 0.000 0.000 0.000
9(05_edgelooppoly): -10.000 4.000 0.000
9(06_squarecharacter1): -3.000 7.847 0.000
9(07_squarecharacter2): -5.000 4.847 0.000
9(08_hexagoncharacter): -5.000 7.847 0.000
9(09_circlecharacter1): 3.000 4.847 0.000
9(10_circlecharacter2): -7.000 5.847 0.000
10(00_ground): 0.000 0.000 0.000
10(01_colinearground): 0.000 0.000 0.000
10(02_chainshape): 0.000 0.000 0.785
10(03_squaretiles): 0.000 0.000 0.000
10(04_edgeloopsquare): 0.000 0.000 0.000
10(05_edgelooppoly): -10.000 4.000 0.000
10(06_squarecharacter1): -3.000 7.817 0.000
10(07_squarecharacter2): -5.000 4.817 0.000
10(08_hexagoncharacter): -5.000 7.817 0.000
10(09_circlecharacter1): 3.000 4.817 0.000
10(10_circlecharacter2): -7.000 5.817 0.000
11(00_ground): 0.000 0.000 0.000
11(01_colinearground): 0.000 0.000 0.000
11(02_chainshape): 0.000 0.000 0.785
11(03_squaretiles): 0.000 0.000 0.000
11(04_edgeloopsquare): 0.000 0.000 0.000
11(05_edgelooppoly): -10.000 4.000 0.000
11(06_squarecharacter1): -3.000 7.783 0.000
11(07_squarecharacter2): -5.000 4.783 0.000
11(08_hexagoncharacter): -5.000 7.783 0.000
11(09_circlecharacter1): 3.000 4.783 0.000
11(10_circlecharacter2): -7.000 5.783 0.000
12(00_ground): 0.000 0.000 0.000
12(01_colinearground): 0.000 0.000 0.000
12(02_chainshape): 0.000 0.000 0.785
12(03_squaretiles): 0.000 0.000 0.000
12(04_edgeloopsquare): 0.000 0.000 0.000
12(05_edgelooppoly): -10.000 4.000 0.000
12(06_squarecharacter1): -3.000 7.747 0.000
12(07_squarecharacter2): -5.000 4.747 0.000
12(08_hexagoncharacter): -5.000 7.747 0.000
12(09_circlecharacter1): 3.000 4.747 0.000
12(10_circlecharacter2): -6.990 5.779 -0.043
13(00_ground): 0.000 0.000 0.000
13(01_colinearground): 0.000 0.000 0.000
13(02_chainshape): 0.000 0.000 0.785
13(03_squaretiles): 0.000 0.000 0.000
13(04_edgeloopsquare): 0.000 0.000 0.000
13(05_edgelooppoly): -10.000 4.000 0.000
13(06_squarecharacter1): -3.000 7.708 0.000
13(07_squarecharacter2): -5.000 4.708 0.000
13(08_hexagoncharacter): -5.000 7.708 0.000
13(09_circlecharacter1): 3.000 4.708 0.000
13(10_circlecharacter2): -6.980 5.774 -0.090
14(00_ground): 0.000 0.000 0.000
14(01_colinearground): 0.000 0.000 0.000
14(02_chainshape): 0.000 0.000 0.785
14(03_squaretiles): 0.000 0.000 0.000
14(04_edgeloopsquare): 0.000 0.000 0.000
14(05_edgelooppoly): -10.000 4.000 0.000
14(06_squarecharacter1): -3.000 7.667 0.000
14(07_squarecharacter2): -5.000 4.667 0.000
14(08_hexagoncharacter): -5.000 7.667 0.000
14(09_circlecharacter1): 3.000 4.667 0.000
14(10_circlecharacter2): -6.969 5.769 -0.140
15(00_ground): 0.000 0.000 0.000
15(01_colinearground): 0.000 0.000 0.000
15(02_chainshape): 0.000 0.000 0.785
15(03_squaretiles): 0.000 0.000 0.000
15(04_edgeloopsquare): 0.000 0.000 0.000
15(05_edgelooppoly): -10.000 4.000 0.000
15(06_squarecharacter1): -3.000 7.622 0.000
15(07_squarecharacter2): -5.000 4.622 0.000
15(08_hexagoncharacter): -5.000 7.622 0.000
15(09_circlecharacter1): 3.000 4.622 0.000
15(10_circlecharacter2): -6.957 5.763 -0.193
16(00_ground): 0.000 0.000 0.000
16(01_colinearground): 0.000 0.000 0.000
16(02_chainshape): 0.000 0.000 0.785
16(03_squaretiles): 0.000 0.000 0.000
16(04_edgeloopsquare): 0.000 0.000 0.000
16(05_edgelooppoly): -10.000 4.000 0.000
16(06_squarecharacter1): -3.000 7.575 0.000
16(07_squarecharacter2): -5.000 4.575 0.000
16(08_hexagoncharacter): -5.000 7.575 0.000
16(09_circlecharacter1): 3.000 4.575 0.000
16(10_circlecharacter2): -6.944 5.757 -0.249
17(00_ground): 0.000 0.000 0.000
17(01_colinearground): 0.000 0.000 0.000
17(02_chainshape): 0.000 0.000 0.785
17(03_squaretiles): 0.000 0.000 0.000
17(04_edgeloopsquare): 0.000 0.000 0.000
17(05_edgelooppoly): -10.000 4.000 0.000
17(06_squarecharacter1): -3.000 7.525 0.000
17(07_squarecharacter2): -5.000 4.525 0.000
17(08_hexagoncharacter): -5.000 7.525 0.000
17(09_circlecharacter1): 3.000 4.525 0.000
17(10_circlecharacter2): -6.931 5.750 -0.309
18(00_ground): 0.000 0.000 0.000
18(01_colinearground): 0.000 0.000 0.000
18(02_chainshape): 0.000 0.000 0.785
18(03_squaretiles): 0.000 0.000 0.000
18(04_edgeloopsquare): 0.000 0.000 0.000
18(05_edgelooppoly): -10.000 4.000 0.000
18(06_squarecharacter1): -3.000 7.472 0.000
18(07_squarecharacter2): -5.000 4.472 0.000
18(08_hexagoncharacter): -5.000 7.472 0.000
18(09_circlecharacter1): 3.000 4.504 0.000
18(10_circlecharacter2): -6.917 5.743 -0.372
19(00_ground): 0.000 0.000 0.000
19(01_colinearground): 0.000 0.000 0.000
19(02_chainshape): 0.000 0.000 0.785
19(03_squaretiles): 0.000 0.000 0.000
19(04_edgeloopsquare): 0.000 0.000 0.000
19(05_edgelooppoly): -10.000 4.000 0.000
19(06_squarecharacter1): -3.000 7.417 0.000
19(07_squarecharacter2): -5.000 4.417 0.000
19(08_hexagoncharacter): -5.000 7.417 0.000
19(09_circlecharacter1): 3.000 4.505 0.000
19(10_circlecharacter2): -6.902 5.736 -0.439
20(00_ground): 0.000 0.000 0.000
20(01_colinearground): 0.000 0.000 0.000
20(02_chainshape): 0.000 0.000 0.785
20(03_squaretiles): 0.000 0.000 0.000
20(04_edgeloopsquare): 0.000 0.000 0.000
20(05_edgelooppoly): -10.000 4.000 0.000
20(06_squarecharacter1): -3.000 7.358 0.000
20(07_squarecharacter2): -5.000 4.358 0.000
20(08_hexagoncharacter): -5.000 7.358 0.000
20(09_circlecharacter1): 3.000 4.505 0.000
20(10_circlecharacter2): -6.887 5.728 -0.509
21(00_ground): 0.000 0.000 0.000
21(01_colinearground): 0.000 0.000 0.000
21(02_chainshape): 0.000 0.000 0.785
21(03_squaretiles): 0.000 0.000 0.000
21(04_edgeloopsquare): 0.000 0.000 0.000
21(05_edgelooppoly): -10.000 4.000 0.000
21(06_squarecharacter1): -3.000 7.297 0.000
21(07_squarecharacter2): -5.000 4.297 0.000
21(08_hexagoncharacter): -5.000 7.297 0.000
21(09_circlecharacter1): 3.000 4.505 0.000
21(10_circlecharacter2): -6.871 5.720 -0.582
22(00_ground): 0.000 0.000 0.000
22(01_colinearground): 0.000 0.000 0.000
22(02_chainshape): 0.000 0.000 0.785
22(03_squaretiles): 0.000 0.000 0.000
22(04_edgeloopsquare): 0.000 0.000 0.000
22(05_edgelooppoly): -10.000 4.000 0.000
22(06_squarecharacter1): -3.000 7.233 0.000
22(07_squarecharacter2): -5.000 4.233 0.000
22(08_hexagoncharacter): -5.000 7.233 0.000
22(09_circlecharacter1): 3.000 4.505 0.000
22(10_circlecharacter2): -6.854 5.712 -0.658
23(00_ground): 0.000 0.000 0.000
23(01_colinearground): 0.000 0.000 0.000
23(02_chainshape): 0.000 0.000 0.785
23(03_squaretiles): 0.000 0.000 0.000
23(04_edgeloopsquare): 0.000 0.000 0.000
23(05_edgelooppoly): -10.000 4.000 0.000
23(06_squarecharacter1): -3.000 7.167 0.000
23(07_squarecharacter2): -5.000 4.167 0.000
23(08_hexagoncharacter): -5.000 7.167 0.000
23(09_circlecharacter1): 3.000 4.505 0.000
23(10_circlecharacter2): -6.836 5.703 -0.738
24(00_ground): 0.000 0.000 0.000
24(01_colinearground): 0.000 0.000 0.000
24(02_chainshape): 0.000 0.000 0.785
24(03_squaretiles): 0.000 0.000 0.000
24(04_edgeloopsquare): 0.000 0.000 0.000
24(05_edgelooppoly): -10.000 4.000 0.000
24(06_squarecharacter1): -3.000 7.097 0.000
24(07_squarecharacter2): -5.000 4.097 0.000
24(08_hexagoncharacter): -5.000 7.097 0.000
24(09_circlecharacter1): 3.000 4.505 0.000
24(10_circlecharacter2): -6.818 5.694 -0.821
25(00_ground): 0.000 0.000 0.000
25(01_colinearground): 0.000 0.000 0.000
25(02_chainshape): 0.000 0.000 0.785
25(03_squaretiles): 0.000 0.000 0.000
25(04_edgeloopsquare): 0.000 0.000 0.000
25(05_edgelooppoly): -10.000 4.000 0.000
25(06_squarecharacter1): -3.000 7.025 0.000
25(07_squarecharacter2): -5.000 4.025 0.000
25(08_hexagoncharacter): -5.000 7.025 0.000
25(09_circlecharacter1): 3.000 4.505 0.000
25(10_circlecharacter2): -6.799 5.684 -0.907
26(00_ground): 0.000 0.000 0.000
26(01_colinearground): 0.000 0.000 0.000
26(02_chainshape): 0.000 0.000 0.785
26(03_squaretiles): 0.000 0.000 0.000
26(04_edgeloopsquare): 0.000 0.000 0.000
26(05_edgelooppoly): -10.000 4.000 0.000
26(06_squarecharacter1): -3.000 6.950 0.000
26(07_squarecharacter2): -5.000 3.950 0.000
26(08_hexagoncharacter): -5.000 6.950 0.000
26(09_circlecharacter1): 3.000 4.505 0.000
26(10_circlecharacter2): -6.779 5.674 -0.997
27(00_ground): 0.000 0.000 0.000
27(01_colinearground): 0.000 0.000 0.000
27(02_chainshape): 0.000 0.000 0.785
27(03_squaretiles): 0.000 0.000 0.000
27(04_edgeloopsquare): 0.000 0.000 0.000
27(05_edgelooppoly): -10.000 4.000 0.000
27(06_squarecharacter1): -3.000 6.872 0.000
27(07_squarecharacter2): -5.000 3.771 0.000
27(08_hexagoncharacter): -5.000 6.872 0.000
27(09_circlecharacter1): 3.000 4.505 0.000
27(10_circlecharacter2): -6.758 5.664 -1.090
28(00_ground): 0.000 0.000 0.000
28(01_colinearground): 0.000 0.000 0.000
28(02_chainshape): 0.000 0.000 0.785
28(03_squaretiles): 0.000 0.000 0.000
28(04_edgeloopsquare): 0.000 0.000 0.000
28(05_edgelooppoly): -10.000 4.000 0.000
28(06_squarecharacter1): -3.000 6.792 0.000
28(07_squarecharacter2): -5.000 3.690 0.000
28(08_hexagoncharacter): -5.000 6.792 0.000
28(09_circlecharacter1): 3.000 4.505 0.000
28(10_circlecharacter2): -6.737 5.654 -1.186
29(00_ground): 0.000 0.000 0.000
29(01_colinearground): 0.000 0.000 0.000
29(02_chainshape): 0.000 0.000 0.785
29(03_squaretiles): 0.000 0.000 0.000
29(04_edgeloopsquare): 0.000 0.000 0.000
29(05_edgelooppoly): -10.000 4.000 0.000
29(06_squarecharacter1): -3.000 6.708 0.000
29(07_squarecharacter2): -5.000 3.607 0.000
29(08_hexagoncharacter): -5.000 6.708 0.000
29(09_circlecharacter1): 3.000 4.505 0.000
29(10_circlecharacter2): -6.715 5.642 -1.286
30(00_ground): 0.000 0.000 0.000
30(01_colinearground): 0.000 0.000 0.000
30(02_chainshape): 0.000 0.000 0.785
30(03_squaretiles): 0.000 0.000 0.000
30(04_edgeloopsquare): 0.000 0.000 0.000
30(05_edgelooppoly): -10.000 4.000 0.000
30(06_squarecharacter1): -3.000 6.622 0.000
30(07_squarecharacter2): -5.000 3.521 0.000
30(08_hexagoncharacter): -5.000 6.622 0.000
30(09_circlecharacter1): 3.000 4.505 0.000
30(10_circlecharacter2): -6.692 5.631 -1.389
31(00_ground): 0.000 0.000 0.000
31(01_colinearground): 0.000 0.000 0.000
31(02_chainshape): 0.000 0.000 0.785
31(03_squaretiles): 0.000 0.000 0.000
31(04_edgeloopsquare): 0.000 0.000 0.000
31(05_edgelooppoly): -10.000 4.000 0.000
31(06_squarecharacter1): -3.000 6.533 0.000
31(07_squarecharacter2): -5.000 3.432 0.000
31(08_hexagoncharacter): -5.000 6.533 0.000
31(09_circlecharacter1): 3.000 4.505 0.000
31(10_circlecharacter2): -6.669 5.619 -1.495
32(00_ground): 0.000 0.000 0.000
32(01_colinearground): 0.000 0.000 0.000
32(02_chainshape): 0.000 0.000 0.785
32(03_squaretiles): 0.000 0.000 0.000
32(04_edgeloopsquare): 0.000 0.000 0.000
32(05_edgelooppoly): -10.000 4.000 0.000
32(06_squarecharacter1): -3.000 6.442 0.000
32(07_squarecharacter2): -5.000 3.340 0.000
32(08_hexagoncharacter): -5.000 6.442 0.000
32(09_circlecharacter1): 3.000 4.505 0.000
32(10_circlecharacter2): -6.644 5.607 -1.605
33(00_ground): 0.000 0.000 0.000
33(01_colinearground): 0.000 0.000 0.000
33(02_chainshape): 0.000 0.000 0.785
33(03_squaretiles): 0.000 0.000 0.000
33(04_edgeloopsquare): 0.000 0.000 0.000
33(05_edgelooppoly): -10.000 4.000 0.000
33(06_squarecharacter1): -3.000 6.347 0.000
33(07_squarecharacter2): -5.000 3.246 0.000
33(08_hexagoncharacter): -5.000 6.347 0.000
33(09_circlecharacter1): 3.000 4.505 0.000
33(10_circlecharacter2): -6.619 5.595 -1.718
34(00_ground): 0.000 0.000 0.000
34(01_colinearground): 0.000 0.000 0.000
34(02_chainshape): 0.000 0.000 0.785
34(03_squaretiles): 0.000 0.000 0.000
34(04_edgeloopsquare): 0.000 0.000 0.000
34(05_edgelooppoly): -10.000 4.000 0.000
34(06_squarecharacter1): -3.000 6.250 0.000
34(07_squarecharacter2): -5.000 3.149 0.000
34(08_hexagoncharacter): -5.000 6.250 0.000
34(09_circlecharacter1): 3.000 4.505 0.000
34(10_circlecharacter2): -6.594 5.582 -1.834
35(00_ground): 0.000 0.000 0.000
35(01_colinearground): 0.000 0.000 0.000
35(02_chainshape): 0.000 0.000 0.785
35(03_squaretiles): 0.000 0.000 0.000
35(04_edgeloopsquare): 0.000 0.000 0.000
35(05_edgelooppoly): -10.000 4.000 0.000
35(06_squarecharacter1): -3.000 6.150 0.000
35(07_squarecharacter2): -5.000 3.049 0.000
35(08_hexagoncharacter): -5.000 6.150 0.000
35(09_circlecharacter1): 3.000 4.505 0.000
35(10_circlecharacter2): -6.567 5.569 -1.954
36(00_ground): 0.000 0.000 0.000
36(01_colinearground): 0.000 0.000 0.000
36(02_chainshape): 0.000 0.000 0.785
36(03_squaretiles): 0.000 0.000 0.000
36(04_edgeloopsquare): 0.000 0.000 0.000
36(05_edgelooppoly): -10.000 4.000 0.000
36(06_squarecharacter1): -3.000 6.047 0.000
36(07_squarecharacter2): -5.000 2.946 0.000
36(08_hexagoncharacter): -5.008 6.070 0.000
36(09_circlecharacter1): 3.000 4.505 0.000
36(10_circlecharacter2): -6.540 5.555 -2.076
37(00_ground): 0.000 0.000 0.000
37(01_colinearground): 0.000 0.000 0.000
37(02_chainshape): 0.000 0.000 0.785
37(03_squaretiles): 0.000 0.000 0.000
37(04_edgeloopsquare): 0.000 0.000 0.000
37(05_edgelooppoly): -10.000 4.000 0.000
37(06_squarecharacter1): -3.000 5.942 0.000
37(07_squarecharacter2): -5.000 2.840 0.000
37(08_hexagoncharacter): -5.034 6.058 0.000
37(09_circlecharacter1): 3.000 4.505 0.000
37(10_circlecharacter2): -6.512 5.541 -2.203
38(00_ground): 0.000 0.000 0.000
38(01_colinearground): 0.000 0.000 0.000
38(02_chainshape): 0.000 0.000 0.785
38(03_squaretiles): 0.000 0.000 0.000
38(04_edgeloopsquare): 0.000 0.000 0.000
38(05_edgelooppoly): -10.000 4.000 0.000
38(06_squarecharacter1): -3.000 5.833 0.000
38(07_squarecharacter2): -5.000 2.732 0.000
38(08_hexagoncharacter): -5.060 6.045 0.000
38(09_circlecharacter1): 3.000 4.505 0.000
38(10_circlecharacter2): -6.483 5.527 -2.332
39(00_ground): 0.000 0.000 0.000
39(01_colinearground): 0.000 0.000 0.000
39(02_chainshape): 0.000 0.000 0.785
39(03_squaretiles): 0.000 0.000 0.000
39(04_edgeloopsquare): 0.000 0.000 0.000
39(05_edgelooppoly): -10.000 4.000 0.000
39(06_squarecharacter1): -3.000 5.722 0.000
39(07_squarecharacter2): -5.000 2.621 0.000
39(08_hexagoncharacter): -5.086 6.032 0.000
39(09_circlecharacter1): 3.000 4.505 0.000
39(10_circlecharacter2): -6.454 5.512 -2.465
40(00_ground): 0.000 0.000 0.000
40(01_colinearground): 0.000 0.000 0.000
40(02_chainshape): 0.000 0.000 0.785
40(03_squaretiles): 0.000 0.000 0.000
40(04_edgeloopsquare): 0.000 0.000 0.000
40(05_edgelooppoly): -10.000 4.000 0.000
40(06_squarecharacter1): -3.000 5.608 0.000
40(07_squarecharacter2): -5.000 2.507 0.000
40(08_hexagoncharacter): -5.114 6.018 0.000
40(09_circlecharacter1): 3.000 4.505 0.000
40(10_circlecharacter2): -6.424 5.497 -2.601
41(00_ground): 0.000 0.000 0.000
41(01_colinearground): 0.000 0.000 0.000
41(02_chainshape): 0.000 0.000 0.785
41(03_squaretiles): 0.000 0.000 0.000
41(04_edgeloopsquare): 0.000 0.000 0.000
41(05_edgelooppoly): -10.000 4.000 0.000
41(06_squarecharacter1): -3.000 5.492 0.000
41(07_squarecharacter2): -5.000 2.390 0.000
41(08_hexagoncharacter): -5.142 6.004 0.000
41(09_circlecharacter1): 3.000 4.505 0.000
41(10_circlecharacter2): -6.393 5.481 -2.741
42(00_ground): 0.000 0.000 0.000
42(01_colinearground): 0.000 0.000 0.000
42(02_chainshape): 0.000 0.000 0.785
42(03_squaretiles): 0.000 0.000 0.000
42(04_edgeloopsquare): 0.000 0.000 0.000
42(05_edgelooppoly): -10.000 4.000 0.000
42(06_squarecharacter1): -3.000 5.372 0.000
42(07_squarecharacter2): -5.000 2.271 0.000
42(08_hexagoncharacter): -5.170 5.990 0.000
42(09_circlecharacter1): 3.000 4.505 0.000
42(10_circlecharacter2): -6.361 5.466 -2.884
43(00_ground): 0.000 0.000 0.000
43(01_colinearground): 0.000 0.000 0.000
43(02_chainshape): 0.000 0.000 0.785
43(03_squaretiles): 0.000 0.000 0.000
43(04_edgeloopsquare): 0.000 0.000 0.000
43(05_edgelooppoly): -10.000 4.000 0.000
43(06_squarecharacter1): -3.000 5.250 0.000
43(07_squarecharacter2): -5.000 2.149 0.000
43(08_hexagoncharacter): -5.200 5.975 0.000
43(09_circlecharacter1): 3.000 4.505 0.000
43(10_circlecharacter2): -6.329 5.449 -3.030
44(00_ground): 0.000 0.000 0.000
44(01_colinearground): 0.000 0.000 0.000
44(02_chainshape): 0.000 0.000 0.785
44(03_squaretiles): 0.000 0.000 0.000
44(04_edgeloopsquare): 0.000 0.000 0.000
44(05_edgelooppoly): -10.000 4.000 0.000
44(06_squarecharacter1): -3.000 5.125 0.000
44(07_squarecharacter2): -5.000 2.024 0.000
44(08_hexagoncharacter): -5.230 5.960 0.000
44(09_circlecharacter1): 3.000 4.505 0.000
44(10_circlecharacter2): -6.296 5.433 -3.179
45(00_ground): 0.000 0.000 0.000
45(01_colinearground): 0.000 0.000 0.000
45(02_chainshape): 0.000 0.000 0.785
45(03_squaretiles): 0.000 0.000 0.000
45(04_edgeloopsquare): 0.000 0.000 0.000
45(05_edgelooppoly): -10.000 4.000 0.000
45(06_squarecharacter1): -3.000 4.997 0.000
45(07_squarecharacter2): -5.000 1.896 0.000
45(08_hexagoncharacter): -5.260 5.945 0.000
45(09_circlecharacter1): 3.000 4.505 0.000
45(10_circlecharacter2): -6.262 5.416 -3.332
46(00_ground): 0.000 0.000 0.000
46(01_colinearground): 0.000 0.000 0.000
46(02_chainshape): 0.000 0.000 0.785
46(03_squaretiles): 0.000 0.000 0.000
46(04_edgeloopsquare): 0.000 0.000 0.000
46(05_edgelooppoly): -10.000 4.000 0.000
46(06_squarecharacter1): -3.000 4.867 0.000
46(07_squarecharacter2): -5.000 1.765 0.000
46(08_hexagoncharacter): -5.292 5.929 0.000
46(09_circlecharacter1): 3.000 4.505 0.000
46(10_circlecharacter2): -6.227 5.399 -3.488
47(00_ground): 0.000 0.000 0.000
47(01_colinearground): 0.000 0.000 0.000
47(02_chainshape): 0.000 0.000 0.785
47(03_squaretiles): 0.000 0.000 0.000
47(04_edgeloopsquare): 0.000 0.000 0.000
47(05_edgelooppoly): -10.000 4.000 0.000
47(06_squarecharacter1): -3.000 4.733 0.000
47(07_squarecharacter2): -5.000 1.632 0.000
47(08_hexagoncharacter): -5.324 5.913 0.000
47(09_circlecharacter1): 3.000 4.505 0.000
47(10_circlecharacter2): -6.192 5.381 -3.648
48(00_ground): 0.000 0.000 0.000
48(01_colinearground): 0.000 0.000 0.000
48(02_chainshape): 0.000 0.000 0.785
48(03_squaretiles): 0.000 0.000 0.000
48(04_edgeloopsquare): 0.000 0.000 0.000
48(05_edgelooppoly): -10.000 4.000 0.000
48(06_squarecharacter1): -3.000 4.597 0.000
48(07_squarecharacter2): -5.000 1.496 0.000
48(08_hexagoncharacter): -5.356 5.897 0.000
48(09_circlecharacter1): 3.000 4.505 0.000
48(10_circlecharacter2): -6.156 5.363 -3.811
49(00_ground): 0.000 0.000 0.000
49(01_colinearground): 0.000 0.000 0.000
49(02_chainshape): 0.000 0.000 0.785
49(03_squaretiles): 0.000 0.000 0.000
49(04_edgeloopsquare): 0.000 0.000 0.000
49(05_edgelooppoly): -10.000 4.000 0.000
49(06_squarecharacter1): -3.000 4.458 0.000
49(07_squarecharacter2): -5.000 1.357 0.000
49(08_hexagoncharacter): -5.390 5.880 0.000
49(09_circlecharacter1): 3.000 4.505 0.000
49(10_circlecharacter2): -6.119 5.345 -3.977
50(00_ground): 0.000 0.000 0.000
50(01_colinearground): 0.000 0.000 0.000
50(02_chainshape): 0.000 0.000 0.785
50(03_squaretiles): 0.000 0.000 0.000
50(04_edgeloopsquare): 0.000 0.000 0.000
50(05_edgelooppoly): -10.000 4.000 0.000
50(06_squarecharacter1): -3.000 4.317 0.000
50(07_squarecharacter2): -5.000 1.265 0.000
50(08_hexagoncharacter): -5.424 5.863 0.000
50(09_circlecharacter1): 3.000 4.505 0.000
50(10_circlecharacter2): -6.082 5.326 -4.146
51(00_ground): 0.000 0.000 0.000
51(01_colinearground): 0.000 0.000 0.000
51(02_chainshape): 0.000 0.000 0.785
51(03_squaretiles): 0.000 0.000 0.000
51(04_edgeloopsquare): 0.000 0.000 0.000
51(05_edgelooppoly): -10.000 4.000 0.000
51(06_squarecharacter1): -3.000 4.172 0.000
51(07_squarecharacter2): -5.000 1.265 0.000
51(08_hexagoncharacter): -5.458 5.846 0.000
51(09_circlecharacter1): 3.000 4.505 0.000
51(10_circlecharacter2): -6.043 5.307 -4.319
52(00_ground): 0.000 0.000 0.000
52(01_colinearground): 0.000 0.000 0.000
52(02_chainshape): 0.000 0.000 0.785
52(03_squaretiles): 0.000 0.000 0.000
52(04_edgeloopsquare): 0.000 0.000 0.000
52(05_edgelooppoly): -10.000 4.000 0.000
52(06_squarecharacter1): -3.000 4.025 0.000
52(07_squarecharacter2): -5.000 1.265 0.000
52(08_hexagoncharacter): -5.494 5.828 0.000
52(09_circlecharacter1): 3.000 4.505 0.000
52(10_circlecharacter2): -6.004 5.287 -4.495
53(00_ground): 0.000 0.000 0.000
53(01_colinearground): 0.000 0.000 0.000
53(02_chainshape): 0.000 0.000 0.785
53(03_squaretiles): 0.000 0.000 0.000
53(04_edgeloopsquare): 0.000 0.000 0.000
53(05_edgelooppoly): -10.000 4.000 0.000
53(06_squarecharacter1): -3.000 3.875 0.000
53(07_squarecharacter2): -5.000 1.265 0.000
53(08_hexagoncharacter): -5.530 5.810 0.000
53(09_circlecharacter1): 3.000 4.505 0.000
53(10_circlecharacter2): -5.976 5.301 -4.620
54(00_ground): 0.000 0.000 0.000
54(01_colinearground): 0.000 0.000 0.000
54(02_chainshape): 0.000 0.000 0.785
54(03_squaretiles): 0.000 0.000 0.000
54(04_edgeloopsquare): 0.000 0.000 0.000
54(05_edgelooppoly): -10.000 4.000 0.000
54(06_squarecharacter1): -3.000 3.722 0.000
54(07_squarecharacter2): -5.000 1.265 0.000
54(08_hexagoncharacter): -5.547 5.802 0.000
54(09_circlecharacter1): 3.000 4.505 0.000
54(10_circlecharacter2): -6.017 5.288 -4.653
55(00_ground): 0.000 0.000 0.000
55(01_colinearground): 0.000 0.000 0.000
55(02_chainshape): 0.000 0.000 0.785
55(03_squaretiles): 0.000 0.000 0.000
55(04_edgeloopsquare): 0.000 0.000 0.000
55(05_edgelooppoly): -10.000 4.000 0.000
55(06_squarecharacter1): -3.000 3.567 0.000
55(07_squarecharacter2): -5.000 1.265 0.000
55(08_hexagoncharacter): -5.554 5.804 0.000
55(09_circlecharacter1): 3.000 4.505 0.000
55(10_circlecharacter2): -6.029 5.290 -4.630
56(00_ground): 0.000 0.000 0.000
56(01_colinearground): 0.000 0.000 0.000
56(02_chainshape): 0.000 0.000 0.785
56(03_squaretiles): 0.000 0.000 0.000
56(04_edgeloopsquare): 0.000 0.000 0.000
56(05_edgelooppoly): -10.000 4.000 0.000
56(06_squarecharacter1): -3.000 3.408 0.000
56(07_squarecharacter2): -5.000 1.265 0.000
56(08_hexagoncharacter): -5.555 5.811 0.000
56(09_circlecharacter1): 3.000 4.505 0.000
56(10_circlecharacter2): -6.035 5.293 -4.619
57(00_ground): 0.000 0.000 0.000
57(01_colinearground): 0.000 0.000 0.000
57(02_chainshape): 0.000 0.000 0.785
57(03_squaretiles): 0.000 0.000 0.000
57(04_edgeloopsquare): 0.000 0.000 0.000
57(05_edgelooppoly): -10.000 4.000 0.000
57(06_squarecharacter1): -3.000 3.247 0.000
57(07_squarecharacter2): -5.000 1.265 0.000
57(08_hexagoncharacter): -5.555 5.816 0.000
57(09_circlecharacter1): 3.000 4.505 0.000
57(10_circlecharacter2): -6.038 5.297 -4.612
58(00_ground): 0.000 0.000 0.000
58(01_colinearground): 0.000 0.000 0.000
58(02_chainshape): 0.000 0.000 0.785
58(03_squaretiles): 0.000 0.000 0.000
58(04_edgeloopsquare): 0.000 0.000 0.000
58(05_edgelooppoly): -10.000 4.000 0.000
58(06_squarecharacter1): -3.000 3.083 0.000
58(07_squarecharacter2): -5.000 1.265 0.000
58(08_hexagoncharacter): -5.556 5.818 0.000
58(09_circlecharacter1): 3.000 4.505 0.000
58(10_circlecharacter2): -6.040 5.298 -4.607
59(00_ground): 0.000 0.000 0.000
59(01_colinearground): 0.000 0.000 0.000
59(02_chainshape): 0.000 0.000 0.785
59(03_squaretiles): 0.000 0.000 0.000
59(04_edgeloopsquare): 0.000 0.000 0.000
59(05_edgelooppoly): -10.000 4.000 0.000
59(06_squarecharacter1): -3.000 2.917 0.000
59(07_squarecharacter2): -5.000 1.265 0.000
59(08_hexagoncharacter): -5.556 5.818 0.000
59(09_circlecharacter1): 3.000 4.505 0.000
59(10_circlecharacter2): -6.041 5.299 -4.606
`

func TestCPPComplianceShapeCast(t *testing.T) {
	transformA := b2.MakeTransform()
	transformA.P = b2.Vec2{0.0, 0.25}
	transformA.Q.SetIdentity()

	transformB := b2.MakeTransform()
	transformB.SetIdentity()

	input := b2.MakeShapeCastInput()

	pA := b2.DistanceProxy{}
	pA.M_vertices = append(pA.M_vertices, b2.Vec2{-0.5, 1.0})
	pA.M_vertices = append(pA.M_vertices, b2.Vec2{0.5, 1.0})
	pA.M_vertices = append(pA.M_vertices, b2.Vec2{0.0, 0.0})
	pA.M_count = 3
	pA.M_radius = b2.PolygonRadius

	pB := b2.DistanceProxy{}
	pB.M_vertices = append(pB.M_vertices, b2.Vec2{-0.5, -0.5})
	pB.M_vertices = append(pB.M_vertices, b2.Vec2{0.5, -0.5})
	pB.M_vertices = append(pB.M_vertices, b2.Vec2{0.5, 0.5})
	pB.M_vertices = append(pB.M_vertices, b2.Vec2{-0.5, 0.5})
	pB.M_count = 4
	pB.M_radius = b2.PolygonRadius

	input.ProxyA = pA
	input.ProxyB = pB
	input.TransformA = transformA
	input.TransformB = transformB
	input.TranslationB.Set(8.0, 0.0)

	output := b2.ShapeCastOutput{}

	hit := b2.ShapeCast(&output, &input)

	transform := b2.MakeTransform()
	transform.Q = transformB.Q
	transform.P = b2.Vec2Add(transformB.P, b2.Vec2MulScalar(output.Lambda, input.TranslationB))

	distanceInput := b2.MakeDistanceInput()
	distanceInput.ProxyA = pA
	distanceInput.ProxyB = pB
	distanceInput.TransformA = transformA
	distanceInput.TransformB = transform
	distanceInput.UseRadii = false
	simplexCache := b2.MakeSimplexCache()
	simplexCache.Count = 0
	distanceOutput := b2.MakeDistanceOutput()

	b2.Distance(&distanceOutput, &simplexCache, &distanceInput)

	msg := fmt.Sprintf("hit = %v, iters = %v, lambda = %v, distance = %.15f",
		hit, output.Iterations, output.Lambda, distanceOutput.Distance)

	fmt.Println(msg)

	if msg != expectedShapeCast {
		text := unifiedDiff("Expected", "Current", expectedShapeCast, msg)
		t.Fatalf("NOT Matching c++ reference. Failure: \n%s", text)
	}
}

var expectedShapeCast2 string = "hit = false, iters = 0, lambda = 1, distance = 8.003905296791061"

func TestCPPComplianceShapeCast2(t *testing.T) {
	transformA := b2.MakeTransform()
	transformA.P = b2.Vec2{0.0, 0.25}
	transformA.Q.SetIdentity()

	transformB := b2.MakeTransform()
	transformB.SetIdentity()

	input := b2.MakeShapeCastInput()

	pA := b2.MakeDistanceProxy()
	pA.M_vertices = append(pA.M_vertices, b2.Vec2{})
	pA.M_count = 1
	pA.M_radius = 0.5

	pB := b2.MakeDistanceProxy()
	pB.M_vertices = append(pB.M_vertices, b2.Vec2{})
	pB.M_count = 1
	pB.M_radius = 0.5

	input.ProxyA = pA
	input.ProxyB = pB
	input.TransformA = transformA
	input.TransformB = transformB
	input.TranslationB.Set(8.0, 0.0)

	output := b2.ShapeCastOutput{}

	hit := b2.ShapeCast(&output, &input)

	transform := b2.MakeTransform()
	transform.Q = transformB.Q
	transform.P = b2.Vec2Add(transformB.P, b2.Vec2MulScalar(output.Lambda, input.TranslationB))

	distanceInput := b2.MakeDistanceInput()
	distanceInput.ProxyA = pA
	distanceInput.ProxyB = pB
	distanceInput.TransformA = transformA
	distanceInput.TransformB = transform
	distanceInput.UseRadii = false
	simplexCache := b2.MakeSimplexCache()
	simplexCache.Count = 0
	distanceOutput := b2.MakeDistanceOutput()

	b2.Distance(&distanceOutput, &simplexCache, &distanceInput)

	msg := fmt.Sprintf("hit = %v, iters = %v, lambda = %v, distance = %.15f",
		hit, output.Iterations, output.Lambda, distanceOutput.Distance)

	fmt.Println(msg)

	if msg != expectedShapeCast2 {
		text := unifiedDiff("Expected", "Current", expectedShapeCast2, msg)
		t.Fatalf("NOT Matching c++ reference. Failure: \n%s", text)
	}
}

func TestCPPCompliance(t *testing.T) {

	// Define the gravity vector.
	gravity := b2.Vec2{0.0, -10.0}

	// Construct a world object, which will hold and simulate the rigid bodies.
	world := b2.MakeWorld(gravity)

	characters := make(map[string]*b2.Body)

	// Ground body
	{
		bd := b2.DefaultBodyDef()
		ground := world.CreateBody(&bd)

		shape := b2.MakeEdgeShape()
		shape.SetTwoSided(b2.Vec2{-20.0, 0.0}, b2.Vec2{20.0, 0.0})
		ground.CreateFixture(&shape, 0.0)
		characters["00_ground"] = ground
	}

	// Collinear edges with no adjacency information.
	// This shows the problematic case where a box shape can hit
	// an internal vertex.
	{
		bd := b2.DefaultBodyDef()
		ground := world.CreateBody(&bd)

		shape := b2.MakeEdgeShape()
		shape.SetTwoSided(b2.Vec2{-8.0, 1.0}, b2.Vec2{-6.0, 1.0})
		ground.CreateFixture(&shape, 0.0)
		shape.SetTwoSided(b2.Vec2{-6.0, 1.0}, b2.Vec2{-4.0, 1.0})
		ground.CreateFixture(&shape, 0.0)
		shape.SetTwoSided(b2.Vec2{-4.0, 1.0}, b2.Vec2{-2.0, 1.0})
		ground.CreateFixture(&shape, 0.0)
		characters["01_colinearground"] = ground
	}

	// Chain shape
	{
		bd := b2.DefaultBodyDef()
		bd.Angle = 0.25 * math.Pi
		ground := world.CreateBody(&bd)

		vs := make([]b2.Vec2, 4)
		vs[0].Set(5.0, 7.0)
		vs[1].Set(6.0, 8.0)
		vs[2].Set(7.0, 8.0)
		vs[3].Set(8.0, 7.0)
		shape := b2.MakeChainShape()
		shape.CreateLoop(vs, 4)
		ground.CreateFixture(&shape, 0.0)
		characters["02_chainshape"] = ground
	}

	// Square tiles. This shows that adjacency shapes may
	// have non-smooth  There is no solution
	// to this problem.
	{
		bd := b2.DefaultBodyDef()
		ground := world.CreateBody(&bd)

		shape := b2.MakePolygonShape()
		shape.SetAsBoxFromCenterAndAngle(1.0, 1.0, b2.Vec2{4.0, 3.0}, 0.0)
		ground.CreateFixture(&shape, 0.0)
		shape.SetAsBoxFromCenterAndAngle(1.0, 1.0, b2.Vec2{6.0, 3.0}, 0.0)
		ground.CreateFixture(&shape, 0.0)
		shape.SetAsBoxFromCenterAndAngle(1.0, 1.0, b2.Vec2{8.0, 3.0}, 0.0)
		ground.CreateFixture(&shape, 0.0)
		characters["03_squaretiles"] = ground
	}

	// Square made from an edge loop. Collision should be smooth.
	{
		bd := b2.DefaultBodyDef()
		ground := world.CreateBody(&bd)

		vs := make([]b2.Vec2, 4)
		vs[0].Set(-1.0, 3.0)
		vs[1].Set(1.0, 3.0)
		vs[2].Set(1.0, 5.0)
		vs[3].Set(-1.0, 5.0)
		shape := b2.MakeChainShape()
		shape.CreateLoop(vs, 4)
		ground.CreateFixture(&shape, 0.0)
		characters["04_edgeloopsquare"] = ground
	}

	// Edge loop. Collision should be smooth.
	{
		bd := b2.DefaultBodyDef()
		bd.Position.Set(-10.0, 4.0)
		ground := world.CreateBody(&bd)

		vs := make([]b2.Vec2, 10)
		vs[0].Set(0.0, 0.0)
		vs[1].Set(6.0, 0.0)
		vs[2].Set(6.0, 2.0)
		vs[3].Set(4.0, 1.0)
		vs[4].Set(2.0, 2.0)
		vs[5].Set(0.0, 2.0)
		vs[6].Set(-2.0, 2.0)
		vs[7].Set(-4.0, 3.0)
		vs[8].Set(-6.0, 2.0)
		vs[9].Set(-6.0, 0.0)
		shape := b2.MakeChainShape()
		shape.CreateLoop(vs, 10)
		ground.CreateFixture(&shape, 0.0)
		characters["05_edgelooppoly"] = ground
	}

	// Square character 1
	{
		bd := b2.DefaultBodyDef()
		bd.Position.Set(-3.0, 8.0)
		bd.Type = b2.Dynamic
		bd.FixedRotation = true
		bd.AllowSleep = false

		body := world.CreateBody(&bd)

		shape := b2.MakePolygonShape()
		shape.SetAsBox(0.5, 0.5)

		fd := b2.DefaultFixtureDef()
		fd.Shape = &shape
		fd.Density = 20.0
		body.CreateFixtureFromDef(&fd)
		characters["06_squarecharacter1"] = body
	}

	// Square character 2
	{
		bd := b2.DefaultBodyDef()
		bd.Position.Set(-5.0, 5.0)
		bd.Type = b2.Dynamic
		bd.FixedRotation = true
		bd.AllowSleep = false

		body := world.CreateBody(&bd)

		shape := b2.MakePolygonShape()
		shape.SetAsBox(0.25, 0.25)

		fd := b2.DefaultFixtureDef()
		fd.Shape = &shape
		fd.Density = 20.0
		body.CreateFixtureFromDef(&fd)
		characters["07_squarecharacter2"] = body
	}

	// Hexagon character
	{
		bd := b2.DefaultBodyDef()
		bd.Position.Set(-5.0, 8.0)
		bd.Type = b2.Dynamic
		bd.FixedRotation = true
		bd.AllowSleep = false

		body := world.CreateBody(&bd)

		angle := 0.0
		delta := math.Pi / 3.0
		vertices := make([]b2.Vec2, 6)
		for i := range 6 {
			vertices[i].Set(0.5*math.Cos(angle), 0.5*math.Sin(angle))
			angle += delta
		}

		shape := b2.MakePolygonShape()
		shape.Set(vertices, 6)

		fd := b2.DefaultFixtureDef()
		fd.Shape = &shape
		fd.Density = 20.0
		body.CreateFixtureFromDef(&fd)
		characters["08_hexagoncharacter"] = body
	}

	// Circle character
	{
		bd := b2.DefaultBodyDef()
		bd.Position.Set(3.0, 5.0)
		bd.Type = b2.Dynamic
		bd.FixedRotation = true
		bd.AllowSleep = false

		body := world.CreateBody(&bd)

		shape := b2.MakeCircleShape()
		shape.SetRadius(0.5)

		fd := b2.DefaultFixtureDef()
		fd.Shape = &shape
		fd.Density = 20.0
		body.CreateFixtureFromDef(&fd)
		characters["09_circlecharacter1"] = body
	}

	// Circle character
	{
		bd := b2.DefaultBodyDef()
		bd.Position.Set(-7.0, 6.0)
		bd.Type = b2.Dynamic
		bd.AllowSleep = false

		body := world.CreateBody(&bd)

		shape := b2.MakeCircleShape()
		shape.SetRadius(0.25)

		fd := b2.DefaultFixtureDef()
		fd.Shape = &shape
		fd.Density = 20.0
		fd.Friction = 1.0
		body.CreateFixtureFromDef(&fd)

		characters["10_circlecharacter2"] = body
	}

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 10 iterations. This provides a high quality simulation
	// in most game scenarios.
	timeStep := 1.0 / 60.0
	velocityIterations := 8
	positionIterations := 3

	var output strings.Builder

	characterNames := make([]string, 0)
	for k := range characters {
		characterNames = append(characterNames, k)
	}

	sort.Strings(characterNames)

	// This is our little game loop.
	for i := range 60 {
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		//runtime.Breakpoint()
		world.Step(timeStep, velocityIterations, positionIterations)

		// Now print the position and angle of the body.
		for _, name := range characterNames {
			character := characters[name]
			position := character.Position()
			angle := character.Angle()
			msg := fmt.Sprintf("%v(%s): %4.3f %4.3f %4.3f\n", i, name, position.X, position.Y, angle)
			fmt.Print(msg)
			output.WriteString(msg)
		}
	}

	if output.String() != expected {
		text := unifiedDiff("Expected", "Current", expected, output.String())
		t.Fatalf("NOT Matching c++ reference. Failure: \n%s", text)
	}
}

// unifiedDiff, iki metni satır satır karşılaştırıp basit bir "unified diff"
// formatında (context=0) fark çıktısı üretir. go-difflib'e olan bağımlılığı
// ortadan kaldırmak için sadece standart kütüphane kullanılarak yazılmıştır.
func unifiedDiff(fromFile, toFile, a, b string) string {
	aLines := strings.SplitAfter(a, "\n")
	bLines := strings.SplitAfter(b, "\n")
	// SplitAfter ile oluşan son boş elemanı at (varsa)
	if len(aLines) > 0 && aLines[len(aLines)-1] == "" {
		aLines = aLines[:len(aLines)-1]
	}
	if len(bLines) > 0 && bLines[len(bLines)-1] == "" {
		bLines = bLines[:len(bLines)-1]
	}

	ops := diffOps(aLines, bLines)

	var sb strings.Builder
	fmt.Fprintf(&sb, "--- %s\n", fromFile)
	fmt.Fprintf(&sb, "+++ %s\n", toFile)

	for i := 0; i < len(ops); {
		if ops[i].kind == 'e' {
			i++
			continue
		}
		// Ardışık eşit-olmayan (equal olmayan) op'ları tek bir hunk'ta topla
		start := i
		for i < len(ops) && ops[i].kind != 'e' {
			i++
		}
		group := ops[start:i]

		aStart, aCount := hunkRange(group, 'a')
		bStart, bCount := hunkRange(group, 'b')

		fmt.Fprintf(&sb, "@@ -%s +%s @@\n", formatRange(aStart, aCount), formatRange(bStart, bCount))
		for _, op := range group {
			switch op.kind {
			case 'd':
				sb.WriteString("-")
				sb.WriteString(aLines[op.aIdx])
			case 'i':
				sb.WriteString("+")
				sb.WriteString(bLines[op.bIdx])
			}
		}
	}

	return sb.String()
}

type diffOp struct {
	kind       byte // 'e' = equal, 'd' = delete (a only), 'i' = insert (b only)
	aIdx, bIdx int
}

// diffOps, klasik O(N*M) Longest Common Subsequence tablosuna dayanan
// bir satır bazlı diff algoritması uygular. Test dosyaları için yeterince
// küçük girdiler söz konusu olduğundan performans burada önemli değildir.
func diffOps(a, b []string) []diffOp {
	n, m := len(a), len(b)
	lcs := make([][]int, n+1)
	for i := range lcs {
		lcs[i] = make([]int, m+1)
	}
	for i := n - 1; i >= 0; i-- {
		for j := m - 1; j >= 0; j-- {
			if a[i] == b[j] {
				lcs[i][j] = lcs[i+1][j+1] + 1
			} else if lcs[i+1][j] >= lcs[i][j+1] {
				lcs[i][j] = lcs[i+1][j]
			} else {
				lcs[i][j] = lcs[i][j+1]
			}
		}
	}

	var ops []diffOp
	i, j := 0, 0
	for i < n && j < m {
		switch {
		case a[i] == b[j]:
			ops = append(ops, diffOp{kind: 'e', aIdx: i, bIdx: j})
			i++
			j++
		case lcs[i+1][j] >= lcs[i][j+1]:
			ops = append(ops, diffOp{kind: 'd', aIdx: i})
			i++
		default:
			ops = append(ops, diffOp{kind: 'i', bIdx: j})
			j++
		}
	}
	for ; i < n; i++ {
		ops = append(ops, diffOp{kind: 'd', aIdx: i})
	}
	for ; j < m; j++ {
		ops = append(ops, diffOp{kind: 'i', bIdx: j})
	}
	return ops
}

// hunkRange, verilen op grubu için ilgili tarafın (a veya b) 1-indeksli
// başlangıç satırı ve satır sayısını hesaplar.
func hunkRange(group []diffOp, side byte) (start, count int) {
	first := -1
	for _, op := range group {
		if side == 'a' && op.kind == 'd' {
			if first == -1 {
				first = op.aIdx
			}
			count++
		} else if side == 'b' && op.kind == 'i' {
			if first == -1 {
				first = op.bIdx
			}
			count++
		}
	}
	if first == -1 {
		return 0, 0
	}
	return first + 1, count
}

func formatRange(start, count int) string {
	if count == 1 {
		return fmt.Sprintf("%d", start)
	}
	if count == 0 {
		// context=0 diff'lerde boş taraf için satır numarası bir önceki
		// satırı gösterir (unified diff geleneği).
		if start == 0 {
			return "0,0"
		}
		return fmt.Sprintf("%d,0", start-1)
	}
	return fmt.Sprintf("%d,%d", start, count)
}

/* ekf7_jacobian.c — F7/Q7/磁航向梯度解析雅可比（对应 ../ekf7/jacobian.py）。
 * 全部为逐项标量展开，便于与参考实现逐行核对。 */
#include "ekf7.h"

void ekf7_calc_f7(double dax, double day, double daz,
                  double dvx, double dvy, double dvz,
                  double q0, double q1, double q2, double q3, double f[7][7])
{
    const double n  = dvx * q0 * 2.0;
    const double n2 = dvx * q1 * 2.0;
    const double n3 = dvx * q2 * 2.0;
    const double n4 = dvx * q3 * 2.0;
    const double n5 = dvy * q0 * 2.0;
    const double n6 = dvy * q1 * 2.0;
    const double n7 = dvy * q2 * 2.0;
    const double n8 = dvy * q3 * 2.0;
    const double n9 = dvz * q0 * 2.0;
    const double n10 = dvz * q1 * 2.0;
    const double n11 = dvz * q2 * 2.0;
    const double n12 = dvz * q3 * 2.0;
    const double n13 = dax / 2.0;
    const double n14 = day / 2.0;
    const double n15 = daz / 2.0;
    const double n16 = -n3;
    const double n17 = -n8;
    const double n18 = -n10;
    const double n19 = -n13;
    const double n20 = -n14;
    const double n21 = -n15;
    const double n22 = n2 + n7 + n12;
    const double n23 = n6 + n9 + n16;
    const double n24 = n4 + n5 + n18;
    const double n25 = n + n11 + n17;

    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
            f[i][j] = 0.0;

    f[0][0] = 1.0;  f[0][1] = n19; f[0][2] = n20; f[0][3] = n21;
    f[1][0] = n13;  f[1][1] = 1.0;  f[1][2] = n15; f[1][3] = n20;
    f[2][0] = n14;  f[2][1] = n21;  f[2][2] = 1.0;  f[2][3] = n13;
    f[3][0] = n15;  f[3][1] = n14;  f[3][2] = n19;  f[3][3] = 1.0;
    f[4][0] = n25;  f[4][1] = n22;  f[4][2] = n23;  f[4][3] = -n4 - n5 + n10;
    f[4][4] = 1.0;
    f[5][0] = n24;  f[5][1] = n3 - n6 - n9;  f[5][2] = n22; f[5][3] = n25;
    f[5][5] = 1.0;
    f[6][0] = n23;  f[6][1] = n24;  f[6][2] = -n + n8 - n11; f[6][3] = n22;
    f[6][6] = 1.0;
}

void ekf7_calc_q7(double dax_var, double day_var, double daz_var,
                  double dvx_var, double dvy_var, double dvz_var,
                  double q0, double q1, double q2, double q3, double q[7][7])
{
    const double n = q0 * q0, n2 = q1 * q1, n3 = q2 * q2, n4 = q3 * q3;
    const double n5 = q0 * q1 * 2.0, n6 = q0 * q2 * 2.0, n7 = q0 * q3 * 2.0;
    const double n8 = q1 * q2 * 2.0, n9 = q1 * q3 * 2.0, n10 = q2 * q3 * 2.0;
    const double n11 = dax_var * q0 * q1 / 4.0;
    const double n12 = dax_var * q0 * q2 / 4.0;
    const double n13 = dax_var * q0 * q3 / 4.0;
    const double n14 = dax_var * q1 * q2 / 4.0;
    const double n15 = dax_var * q1 * q3 / 4.0;
    const double n16 = dax_var * q2 * q3 / 4.0;
    const double n17 = day_var * q0 * q1 / 4.0;
    const double n18 = day_var * q0 * q2 / 4.0;
    const double n19 = day_var * q0 * q3 / 4.0;
    const double n20 = day_var * q1 * q2 / 4.0;
    const double n21 = day_var * q1 * q3 / 4.0;
    const double n22 = day_var * q2 * q3 / 4.0;
    const double n23 = daz_var * q0 * q1 / 4.0;
    const double n24 = daz_var * q0 * q2 / 4.0;
    const double n25 = daz_var * q0 * q3 / 4.0;
    const double n26 = daz_var * q1 * q2 / 4.0;
    const double n27 = daz_var * q1 * q3 / 4.0;
    const double n28 = daz_var * q2 * q3 / 4.0;
    const double n29 = -n8, n30 = -n9, n31 = -n10;
    const double n32 = -n2, n33 = -n3, n34 = -n4;
    const double n35 = -n11, n36 = -n12, n37 = -n15, n38 = -n16;
    const double n39 = -n18, n40 = -n19, n41 = -n20, n42 = -n21;
    const double n43 = -n23, n44 = -n25, n45 = -n26, n46 = -n28;
    const double n47 = n5 + n10, n48 = n6 + n9, n49 = n7 + n8;
    const double n50 = n5 + n31, n51 = n6 + n30, n52 = n7 + n29;
    const double n53 = n + n4 + n32 + n33;
    const double n54 = n + n3 + n32 + n34;
    const double n55 = n + n2 + n33 + n34;
    const double n56 = n17 + n38 + n43;
    const double n57 = n24 + n36 + n42;
    const double n58 = n13 + n40 + n45;
    const double n59 = n14 + n41 + n44;
    const double n60 = n27 + n37 + n39;
    const double n61 = n22 + n35 + n46;
    const double n62 = dvx_var * n49 * n51;
    const double n63 = dvy_var * n47 * n52;
    const double n64 = dvz_var * n48 * n50;
    const double n65 = dvx_var * n49 * n55;
    const double n66 = dvy_var * n47 * n54;
    const double n67 = dvz_var * n48 * n53;
    const double n68 = dvx_var * n51 * n55;
    const double n69 = dvy_var * n52 * n54;
    const double n70 = dvz_var * n50 * n53;
    const double n71 = -n62, n72 = -n63, n73 = -n64;
    const double n74 = -n68, n75 = -n69, n76 = -n70;
    const double n77 = n73 + n65 + n75;
    const double n78 = n71 + n66 + n76;
    const double n79 = n72 + n67 + n74;

    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
            q[i][j] = 0.0;

    q[0][0] = dax_var * n2 / 4.0 + day_var * n3 / 4.0 + daz_var * n4 / 4.0;
    q[0][1] = n61;  q[0][2] = n60;  q[0][3] = n59;
    q[1][0] = n61;
    q[1][1] = dax_var * n / 4.0 + day_var * n4 / 4.0 + daz_var * n3 / 4.0;
    q[1][2] = n58;  q[1][3] = n57;
    q[2][0] = n60;  q[2][1] = n58;
    q[2][2] = dax_var * n4 / 4.0 + day_var * n / 4.0 + daz_var * n2 / 4.0;
    q[2][3] = n56;
    q[3][0] = n59;  q[3][1] = n57;  q[3][2] = n56;
    q[3][3] = dax_var * n3 / 4.0 + day_var * n2 / 4.0 + daz_var * n / 4.0;
    q[4][4] = dvx_var * n55 * n55 + dvy_var * n52 * n52 + dvz_var * n48 * n48;
    q[4][5] = n77;  q[4][6] = n79;
    q[5][4] = n77;
    q[5][5] = dvx_var * n49 * n49 + dvy_var * n54 * n54 + dvz_var * n50 * n50;
    q[5][6] = n78;
    q[6][4] = n79;  q[6][5] = n78;
    q[6][6] = dvx_var * n51 * n51 + dvy_var * n47 * n47 + dvz_var * n53 * n53;
}

void ekf7_calc_h_hdgc(double mx, double my, double mz,
                      double q0, double q1, double q2, double q3, double h[7])
{
    const double n = q0 * q0, n2 = q1 * q1, n3 = q2 * q2, n4 = q3 * q3;
    const double n5 = mx * q0 * 2.0, n6 = mx * q1 * 2.0;
    const double n7 = my * q0 * 2.0, n8 = mx * q2 * 2.0;
    const double n9 = my * q1 * 2.0, n10 = mz * q0 * 2.0;
    const double n11 = mx * q3 * 2.0, n12 = my * q2 * 2.0;
    const double n13 = mz * q1 * 2.0, n14 = my * q3 * 2.0;
    const double n15 = mz * q2 * 2.0, n16 = mz * q3 * 2.0;
    const double n17 = q0 * q1 * 2.0, n18 = q0 * q2 * 2.0, n19 = q0 * q3 * 2.0;
    const double n20 = q1 * q2 * 2.0, n21 = q1 * q3 * 2.0, n22 = q2 * q3 * 2.0;
    const double n23 = -n8, n24 = -n13, n25 = -n14;
    const double n26 = -n20, n27 = -n22;
    const double n28 = -n2, n29 = -n3, n30 = -n4;
    const double n31 = n18 + n21, n32 = n19 + n20;
    const double n33 = n6 + n12 + n16;
    const double n34 = n17 + n27;
    const double n35 = n19 + n26;
    const double n36 = mx * n32, n37 = mz * n31;
    const double n38 = n9 + n10 + n23;
    const double n39 = n7 + n11 + n24;
    const double n40 = n5 + n15 + n25;
    const double n41 = n + n3 + n28 + n30;
    const double n42 = n + n2 + n29 + n30;
    const double n43 = my * n35, n44 = mz * n34;
    const double n45 = mx * n42, n46 = my * n41;
    const double n47 = -n43, n48 = -n44;
    const double n49 = n37 + n47 + n45;
    const double n50 = n36 + n48 + n46;
    const double n51 = n49 * n49;
    const double n52 = 1.0 / n50;
    const double n53 = n52 * n52;
    const double n54 = n51 * n53;
    const double n55 = n54 + 1.0;
    const double n56 = 1.0 / n55;

    h[0] = n56 * (n40 * n52 - n39 * n49 * n53);
    h[1] = n56 * (n33 * n52 + n38 * n49 * n53);
    h[2] = n56 * (n38 * n52 - n33 * n49 * n53);
    h[3] = -n56 * (n39 * n52 + n40 * n49 * n53);
    h[4] = 0.0; h[5] = 0.0; h[6] = 0.0;
}

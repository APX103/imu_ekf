# -*- coding: utf-8 -*-
from __future__ import annotations
"""EKF 的解析雅可比：状态转移 F7、过程噪声映射 Q7、磁航向观测梯度 Hdgc。

全部为显式标量展开式，便于逐行核对。
"""

import numpy as np


def calc_f7(dax, day, daz, dvx, dvy, dvz, q0, q1, q2, q3) -> np.ndarray:
    n = dvx * q0 * 2.0
    n2 = dvx * q1 * 2.0
    n3 = dvx * q2 * 2.0
    n4 = dvx * q3 * 2.0
    n5 = dvy * q0 * 2.0
    n6 = dvy * q1 * 2.0
    n7 = dvy * q2 * 2.0
    n8 = dvy * q3 * 2.0
    n9 = dvz * q0 * 2.0
    n10 = dvz * q1 * 2.0
    n11 = dvz * q2 * 2.0
    n12 = dvz * q3 * 2.0
    n13 = dax / 2.0
    n14 = day / 2.0
    n15 = daz / 2.0
    n16 = -n3
    n17 = -n8
    n18 = -n10
    n19 = -n13
    n20 = -n14
    n21 = -n15
    n22 = n2 + n7 + n12
    n23 = n6 + n9 + n16
    n24 = n4 + n5 + n18
    n25 = n + n11 + n17
    return np.array(
        [
            [1.0, n19, n20, n21, 0.0, 0.0, 0.0],
            [n13, 1.0, n15, n20, 0.0, 0.0, 0.0],
            [n14, n21, 1.0, n13, 0.0, 0.0, 0.0],
            [n15, n14, n19, 1.0, 0.0, 0.0, 0.0],
            [n25, n22, n23, -n4 - n5 + n10, 1.0, 0.0, 0.0],
            [n24, n3 - n6 - n9, n22, n25, 0.0, 1.0, 0.0],
            [n23, n24, -n + n8 - n11, n22, 0.0, 0.0, 1.0],
        ]
    )


def calc_q7(dax_var, day_var, daz_var, dvx_var, dvy_var, dvz_var, q0, q1, q2, q3) -> np.ndarray:
    n = q0 * q0
    n2 = q1 * q1
    n3 = q2 * q2
    n4 = q3 * q3
    n5 = q0 * q1 * 2.0
    n6 = q0 * q2 * 2.0
    n7 = q0 * q3 * 2.0
    n8 = q1 * q2 * 2.0
    n9 = q1 * q3 * 2.0
    n10 = q2 * q3 * 2.0
    n11 = dax_var * q0 * q1 / 4.0
    n12 = dax_var * q0 * q2 / 4.0
    n13 = dax_var * q0 * q3 / 4.0
    n14 = dax_var * q1 * q2 / 4.0
    n15 = dax_var * q1 * q3 / 4.0
    n16 = dax_var * q2 * q3 / 4.0
    n17 = day_var * q0 * q1 / 4.0
    n18 = day_var * q0 * q2 / 4.0
    n19 = day_var * q0 * q3 / 4.0
    n20 = day_var * q1 * q2 / 4.0
    n21 = day_var * q1 * q3 / 4.0
    n22 = day_var * q2 * q3 / 4.0
    n23 = daz_var * q0 * q1 / 4.0
    n24 = daz_var * q0 * q2 / 4.0
    n25 = daz_var * q0 * q3 / 4.0
    n26 = daz_var * q1 * q2 / 4.0
    n27 = daz_var * q1 * q3 / 4.0
    n28 = daz_var * q2 * q3 / 4.0
    n29 = -n8
    n30 = -n9
    n31 = -n10
    n32 = -n2
    n33 = -n3
    n34 = -n4
    n35 = -n11
    n36 = -n12
    n37 = -n15
    n38 = -n16
    n39 = -n18
    n40 = -n19
    n41 = -n20
    n42 = -n21
    n43 = -n23
    n44 = -n25
    n45 = -n26
    n46 = -n28
    n47 = n5 + n10
    n48 = n6 + n9
    n49 = n7 + n8
    n50 = n5 + n31
    n51 = n6 + n30
    n52 = n7 + n29
    n53 = n + n4 + n32 + n33
    n54 = n + n3 + n32 + n34
    n55 = n + n2 + n33 + n34
    n56 = n17 + n38 + n43
    n57 = n24 + n36 + n42
    n58 = n13 + n40 + n45
    n59 = n14 + n41 + n44
    n60 = n27 + n37 + n39
    n61 = n22 + n35 + n46
    n62 = dvx_var * n49 * n51
    n63 = dvy_var * n47 * n52
    n64 = dvz_var * n48 * n50
    n65 = dvx_var * n49 * n55
    n66 = dvy_var * n47 * n54
    n67 = dvz_var * n48 * n53
    n68 = dvx_var * n51 * n55
    n69 = dvy_var * n52 * n54
    n70 = dvz_var * n50 * n53
    n71 = -n62
    n72 = -n63
    n73 = -n64
    n74 = -n68
    n75 = -n69
    n76 = -n70
    n77 = n73 + n65 + n75
    n78 = n71 + n66 + n76
    n79 = n72 + n67 + n74
    return np.array(
        [
            [dax_var * n2 / 4.0 + day_var * n3 / 4.0 + daz_var * n4 / 4.0, n61, n60, n59, 0.0, 0.0, 0.0],
            [n61, dax_var * n / 4.0 + day_var * n4 / 4.0 + daz_var * n3 / 4.0, n58, n57, 0.0, 0.0, 0.0],
            [n60, n58, dax_var * n4 / 4.0 + day_var * n / 4.0 + daz_var * n2 / 4.0, n56, 0.0, 0.0, 0.0],
            [n59, n57, n56, dax_var * n3 / 4.0 + day_var * n2 / 4.0 + daz_var * n / 4.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, dvx_var * n55 * n55 + dvy_var * n52 * n52 + dvz_var * n48 * n48, n77, n79],
            [0.0, 0.0, 0.0, 0.0, n77, dvx_var * n49 * n49 + dvy_var * n54 * n54 + dvz_var * n50 * n50, n78],
            [0.0, 0.0, 0.0, 0.0, n79, n78, dvx_var * n51 * n51 + dvy_var * n47 * n47 + dvz_var * n53 * n53],
        ]
    )


def calc_h_hdgc(mx, my, mz, q0, q1, q2, q3) -> np.ndarray:
    """磁航向观测的解析雅可比 d/dq [ atan2((R q · m)_x, (R q · m)_y) ]。"""
    n = q0 * q0
    n2 = q1 * q1
    n3 = q2 * q2
    n4 = q3 * q3
    n5 = mx * q0 * 2.0
    n6 = mx * q1 * 2.0
    n7 = my * q0 * 2.0
    n8 = mx * q2 * 2.0
    n9 = my * q1 * 2.0
    n10 = mz * q0 * 2.0
    n11 = mx * q3 * 2.0
    n12 = my * q2 * 2.0
    n13 = mz * q1 * 2.0
    n14 = my * q3 * 2.0
    n15 = mz * q2 * 2.0
    n16 = mz * q3 * 2.0
    n17 = q0 * q1 * 2.0
    n18 = q0 * q2 * 2.0
    n19 = q0 * q3 * 2.0
    n20 = q1 * q2 * 2.0
    n21 = q1 * q3 * 2.0
    n22 = q2 * q3 * 2.0
    n23 = -n8
    n24 = -n13
    n25 = -n14
    n26 = -n20
    n27 = -n22
    n28 = -n2
    n29 = -n3
    n30 = -n4
    n31 = n18 + n21
    n32 = n19 + n20
    n33 = n6 + n12 + n16
    n34 = n17 + n27
    n35 = n19 + n26
    n36 = mx * n32
    n37 = mz * n31
    n38 = n9 + n10 + n23
    n39 = n7 + n11 + n24
    n40 = n5 + n15 + n25
    n41 = n + n3 + n28 + n30
    n42 = n + n2 + n29 + n30
    n43 = my * n35
    n44 = mz * n34
    n45 = mx * n42
    n46 = my * n41
    n47 = -n43
    n48 = -n44
    n49 = n37 + n47 + n45
    n50 = n36 + n48 + n46
    n51 = n49 * n49
    n52 = 1.0 / n50
    n53 = n52 * n52
    n54 = n51 * n53
    n55 = n54 + 1.0
    n56 = 1.0 / n55
    return np.array(
        [
            n56 * (n40 * n52 - n39 * n49 * n53),
            n56 * (n33 * n52 + n38 * n49 * n53),
            n56 * (n38 * n52 - n33 * n49 * n53),
            -n56 * (n39 * n52 + n40 * n49 * n53),
            0.0,
            0.0,
            0.0,
        ]
    )



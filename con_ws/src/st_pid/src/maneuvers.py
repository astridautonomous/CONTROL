import bezier as bz
import numpy as np

def durak_trajectory(p1, p2, p3, p4, p5, p6, p7, p8, p9):

    # Durak alanına giriş

    nodes_in = np.asfortranarray([
        [p1[0], p2[0], p3[0], p4[0], p5[0]],        # x
        [p1[1], p2[1], p3[1], p4[1], p5[1]],         # y
        ])
    curve1 = bz.Curve(nodes_in, degree=4)

    # Durak alanından çıkış

    nodes_out = np.asfortranarray([
        [p5[0], p6[0], p7[0], p8[0], p9[0]],    # x
        [p5[1], p6[1], p7[1], p8[1], p9[1]],      # y
        ])

    curve2 = bz.Curve(nodes_out, degree=4)

    t = np.linspace(0, 1, 100)
    points = curve1.evaluate_multi(t)
    points2 = curve2.evaluate_multi(t)
    points_combined = np.concatenate((points, points2), axis=1)
    points_transposed = np.transpose(points_combined)
    return points_transposed



def park_trajectory(p1, p2, p3, p4, p5):

    # Park alanına giriş

    nodes = np.asfortranarray([
        [p1[0], p2[0], p3[0], p4[0], p5[0]],        # x
        [p1[1], p2[1], p3[1], p4[1], p5[1]],         # y
        ])

    curve1 = bz.Curve(nodes, degree=4)

    t = np.linspace(0, 1, 100)
    points = curve1.evaluate_multi(t)
    points_transposed = np.transpose(points)
    
    return points_transposed

### Şerit değiştirme fonksiyonu

def serit_degistirme(p1, p2):

    # Şerit değiştirme

    y = np.linspace(p2[1], p1[1], 100)
    x = np.linspace(p2[0], p1[0], 100)
    points = np.column_stack((x, y))
    return points
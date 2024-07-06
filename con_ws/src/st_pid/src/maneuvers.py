import bezier as bz
import numpy as np
import matplotlib.pyplot as plt # For plotting
from matplotlib.patches import Rectangle # For Rectangle patch

def durak_trajectory(baslangic_konum, bitis_konum, durak_bottom_left, durak_bottom_right, durak_top_left, durak_top_right):
    # durak alanı ölçüleri
    durak_orta_x = (durak_bottom_left[0] + durak_bottom_right[0] + durak_top_left[0] + durak_top_right[0]) / 4
    durak_orta_y = (durak_bottom_left[1] + durak_bottom_right[1] + durak_top_left[1] + durak_top_right[1]) / 4

    # Durak alanına giriş

    nodes_in = np.asfortranarray([
        [baslangic_konum[0], baslangic_konum[0], (durak_orta_x + baslangic_konum[0])/2 , durak_orta_x , durak_orta_x],    # x
        [baslangic_konum[1], (baslangic_konum[1] + durak_orta_y) /2 , (baslangic_konum[1] + durak_orta_y)/2 , (baslangic_konum[1] + durak_orta_y)/2 , durak_orta_y],  # y
        ])

    curve1 = bz.Curve(nodes_in, degree=4)

    # Durak alanından çıkış

    nodes_out = np.asfortranarray([
        [durak_orta_x, durak_orta_x, (durak_orta_x + bitis_konum[0])/2 , bitis_konum[0] , bitis_konum[0]],    # x
        [durak_orta_y, (durak_orta_y + bitis_konum[1]) /2 , (durak_orta_y + bitis_konum[1]) /2 ,(durak_orta_y + bitis_konum[1]) /2  , bitis_konum[1]],      # y
        ])

    curve2 = bz.Curve(nodes_out, degree=4)

    t = np.linspace(0, 1, 100)
    points = curve1.evaluate_multi(t)
    points2 = curve2.evaluate_multi(t)
    points_combined = np.concatenate((points, points2), axis=1)
    # Plot the curve
    # plt.plot(points[0], points[1])
    # plt.plot(points2[0], points2[1])
    # durakspot = Rectangle((2, 2), 2.4 , 5.5, fc='pink', alpha=0.5) # durak alanı
    # plt.plot(points_combined[0], points_combined[1], 'k--')
    # plt.gca().add_patch(durakspot)
    # plt.xlabel('x')
    # plt.xlim(0, 10)
    # plt.ylim(0, 10)
    # plt.ylabel('y')
    # plt.title('Bezier Curve')
    # plt.grid(True)
    # plt.show()
    return points_combined

# durak_trajectory([1, 0], [1, 9], [2, 2], [4.4, 2], [2, 7.5], [4.4, 7.5])

def park_trajectory(baslangic_konum, bitis_konum,park_bottom_left, park_bottom_right, park_top_left, park_top_right):
    # park alanı ölçüleri

    park_orta_x = (park_bottom_left[0] + park_bottom_right[0] + park_top_left[0] + park_top_right[0]) / 4
    park_orta_y = (park_bottom_left[1] + park_bottom_right[1] + park_top_left[1] + park_top_right[1]) / 4

    # Park alanına giriş

    nodes_in = np.asfortranarray([
        [baslangic_konum[0], baslangic_konum[0], (park_orta_x + baslangic_konum[0])/2 , park_orta_x , park_orta_x],    # x
        [baslangic_konum[1], (baslangic_konum[1] + park_orta_y) /2 , (baslangic_konum[1] + park_orta_y)/2 , (baslangic_konum[1] + park_orta_y)/2 , park_orta_y],  # y
        ])

    curve1 = bz.Curve(nodes_in, degree=4)

    # Park alanından çıkış

    nodes_out = np.asfortranarray([
        [park_orta_x, park_orta_x, (park_orta_x + bitis_konum[0])/2 , bitis_konum[0] , bitis_konum[0]],    # x
        [park_orta_y, (park_orta_y + bitis_konum[1]) /2 , (park_orta_y + bitis_konum[1]) /2 ,(park_orta_y + bitis_konum[1]) /2  , bitis_konum[1]],      # y
        ])

    curve2 = bz.Curve(nodes_out, degree=4)

    t = np.linspace(0, 1, 100)
    points = curve1.evaluate_multi(t)
    points2 = curve2.evaluate_multi(t)
    points_combined = np.concatenate((points, points2), axis=1)
    
    return points_combined

### Şerit değiştirme fonksiyonu

def serit_degistirme(serit_baslangic,serit_bitis):
    # serit değiştirme ölçüleri
    serit_orta_x = (serit_baslangic[0] + serit_bitis[0]) / 2
    serit_orta_y = (serit_baslangic[1] + serit_bitis[1]) / 2

    # Serit değiştirme alanına giriş

    nodes_in = np.asfortranarray([
        [serit_baslangic[0], serit_baslangic[0], (serit_orta_x + serit_baslangic[0])/2 , serit_orta_x , serit_orta_x],    # x
        [serit_baslangic[1], (serit_baslangic[1] + serit_orta_y) /2 , (serit_baslangic[1] + serit_orta_y)/2 , (serit_baslangic[1] + serit_orta_y)/2 , serit_orta_y],  # y
        ])

    curve1 = bz.Curve(nodes_in, degree=4)

    # Serit değiştirme alanından çıkış

    nodes_out = np.asfortranarray([
        [serit_orta_x, serit_orta_x, (serit_orta_x + serit_bitis[0])/2 , serit_bitis[0] , serit_bitis[0]],    # x
        [serit_orta_y, (serit_orta_y + serit_bitis[1]) /2 , (serit_orta_y + serit_bitis[1]) /2 ,(serit_orta_y + serit_bitis[1]) /2  , serit_bitis[1]],      # y
        ])

    curve2 = bz.Curve(nodes_out, degree=4)

    t = np.linspace(0, 1, 100)
    points = curve1.evaluate_multi(t)
    points2 = curve2.evaluate_multi(t)
    points_combined = np.concatenate((points, points2), axis=1)
    
    return points_combined
from maneuvers import durak_trajectory, park_trajectory, serit_degistirme

kararmetre = 0

refpose = 'A* nav_ws'

# durak koordinatları
durak_baslangic = None
durak_bitis = None
durak_bottom_left = None
durak_bottom_right = None
durak_top_left = None
durak_top_right = None

# park koordinatları
park_baslangic = None
park_bitis = None
park_bottom_left = None
park_bottom_right = None
park_top_left = None
park_top_right = None

# serit koordinatları
serit_baslangic = None
serit_bitis = None

def manevra(kararmetre):

    if kararmetre == 0:
        refpose = park_trajectory(park_baslangic,park_bitis,park_bottom_left,park_bottom_right,park_top_left,park_top_right)

    elif kararmetre == 1:
        refpose = durak_trajectory(durak_baslangic,durak_bitis,durak_bottom_left,durak_bottom_right,durak_top_left,durak_top_right)

    elif kararmetre == 2:
        refpose = serit_degistirme(serit_baslangic,serit_bitis)
        
    else:
        refpose = refpose

    return refpose
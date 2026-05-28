"""Hand-designed face wireframe.

Edit VERTICES to move points (SVG pixel coords, viewBox 960x720).
Edit EDGES to add/remove connections (each entry is a pair of vertex names).
Edge groups are inferred from vertex name prefixes (brow_/eye_/mouth_),
which the FPGA uses to apply per-feature animation later.

Then run build_face_mesh.py to regenerate bishop_mesh_pkg.vhd.
"""

VERTICES = {
    # ---- Silhouette ----
    "crown":     (480,  60),
    "temple_l":  (350, 110),
    "temple_r":  (610, 110),
    "side_l":    (308, 250),
    "side_r":    (652, 250),
    "cheek_l":   (322, 380),
    "cheek_r":   (638, 380),
    "jaw_l":     (375, 545),
    "jaw_r":     (585, 545),
    "chin_l":    (440, 648),
    "chin_r":    (520, 648),

    # ---- Brows ----
    "brow_l_o":  (350, 265),    # outer
    "brow_l_p":  (405, 248),    # peak
    "brow_l_i":  (450, 265),    # inner
    "brow_r_i":  (510, 265),
    "brow_r_p":  (555, 248),
    "brow_r_o":  (610, 265),

    # ---- Eyes (diamond, 4 verts each) ----
    "eye_l_o":   (370, 315),
    "eye_l_t":   (405, 305),
    "eye_l_i":   (445, 315),
    "eye_l_b":   (405, 325),
    "eye_r_i":   (515, 315),
    "eye_r_t":   (555, 305),
    "eye_r_o":   (590, 315),
    "eye_r_b":   (555, 325),

    # ---- Nose ----
    "nose_top":  (480, 285),
    "nose_l":    (455, 460),
    "nose_r":    (505, 460),
    "nose_tip":  (480, 480),

    # ---- Mouth ----
    "mouth_l":   (420, 505),
    "mouth_t":   (480, 498),
    "mouth_r":   (540, 505),
    "mouth_b":   (480, 525),
}

EDGES = [
    # Silhouette (closed loop, clockwise from crown)
    ("crown",    "temple_l"),
    ("temple_l", "side_l"),
    ("side_l",   "cheek_l"),
    ("cheek_l",  "jaw_l"),
    ("jaw_l",    "chin_l"),
    ("chin_l",   "chin_r"),
    ("chin_r",   "jaw_r"),
    ("jaw_r",    "cheek_r"),
    ("cheek_r",  "side_r"),
    ("side_r",   "temple_r"),
    ("temple_r", "crown"),

    # Brows (each as 2 segments)
    ("brow_l_o", "brow_l_p"),
    ("brow_l_p", "brow_l_i"),
    ("brow_r_i", "brow_r_p"),
    ("brow_r_p", "brow_r_o"),

    # Eyes (diamond)
    ("eye_l_o", "eye_l_t"), ("eye_l_t", "eye_l_i"),
    ("eye_l_i", "eye_l_b"), ("eye_l_b", "eye_l_o"),
    ("eye_r_i", "eye_r_t"), ("eye_r_t", "eye_r_o"),
    ("eye_r_o", "eye_r_b"), ("eye_r_b", "eye_r_i"),

    # Nose (bridge + nostril triangle)
    ("nose_top", "nose_l"),
    ("nose_top", "nose_r"),
    ("nose_l",   "nose_tip"),
    ("nose_r",   "nose_tip"),
    ("nose_l",   "nose_r"),

    # Mouth (diamond + horizontal centerline)
    ("mouth_l", "mouth_t"),
    ("mouth_t", "mouth_r"),
    ("mouth_r", "mouth_b"),
    ("mouth_b", "mouth_l"),
    ("mouth_l", "mouth_r"),
]

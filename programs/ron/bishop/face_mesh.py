"""Hand-designed face wireframe.

Edit VERTICES to move points (SVG pixel coords, viewBox 960x720).
Edit EDGES to add/remove connections (each entry is a pair of vertex names).
Edge groups are inferred from vertex name prefixes (brow_/eye_/mouth_),
which the FPGA uses to apply per-feature animation later.

Edit EXPRESSIONS to add or tweak facial poses.  Each expression is a
per-vertex (dx, dy) delta in the same SVG pixel space as VERTICES; +y
is down.  Only brows, eyes, and mouth are morphed — silhouette and
nose stay put so tip-stripping math doesn't change per expression.

Then run build_face_mesh.py to regenerate bishop_mesh_pkg.vhd and the
per-expression preview SVGs.
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

# K4 (registers_in(3)) top-3-bits selects one of these.  Slots 5..7
# fold back to neutral in the FPGA.  Order MATTERS — it's the runtime
# mapping.  Deltas are in SVG pixel space (+y = down).
EXPRESSIONS = {
    "neutral": {},

    "happy": {
        # Mouth: wide upward U.  Corners pull up AND outward (widen, not
        # just curl); top lip lifts and bottom lip drops so the lips part.
        "mouth_l":  (-8, -10), "mouth_r":  (+8, -10),
        "mouth_t":  (0, -6),
        "mouth_b":  (0, +3),
        # Eyes: cheek-raise squint — the lower lid rising is the single
        # biggest "believable smile" tell, so it dominates.  Top lid eases
        # down a touch; outer corner lifts (smile-eye / crow's-feet hint).
        "eye_l_t":  (0, +2),   "eye_r_t":  (0, +2),
        "eye_l_b":  (0, -5),   "eye_r_b":  (0, -5),
        "eye_l_o":  (0, -2),   "eye_r_o":  (0, -2),
        # Nose: nostril sides lift very slightly (nasolabial folds deepen
        # in a real smile, but the wireframe has no fold geometry).
        "nose_l":   (0, -2),   "nose_r":   (0, -2),
        # Brows: stay neutral — a baseline smile doesn't raise the brows
        # (raised brows would read as surprised-happy instead).
    },

    "sad": {
        # Brows — THE signature of sadness: inner corners pull up AND
        # slightly together, peak lifts a touch, outer ends droop, giving
        # the oblique "/\" slope (high at the nose, sloping down to the sides).
        "brow_l_i": (+3, -10), "brow_r_i": (-3, -10),
        "brow_l_p": (0, -2),   "brow_r_p": (0, -2),
        "brow_l_o": (0, +5),   "brow_r_o": (0, +5),
        # Eyes — hooded and heavy: upper lids droop, whole eye sags,
        # outer corners drop (loss of alertness).
        "eye_l_t":  (0, +4),   "eye_r_t":  (0, +4),
        "eye_l_b":  (0, +2),   "eye_r_b":  (0, +2),
        "eye_l_o":  (0, +2),   "eye_r_o":  (0, +2),
        # Mouth — inverted-U frown (mirror of the happy U): corners pull
        # down and slightly in, top lip lifts to deepen the frown, and the
        # lower-lip midpoint nudges up to fake the pout.  Lips stay closed.
        "mouth_l":  (+3, +10), "mouth_r":  (-3, +10),
        "mouth_t":  (0, -2),
        "mouth_b":  (0, -4),
        # Nose stays at rest.
    },

    "angry": {
        # brows: strong downward-inward V; inner drops hard and pulls
        # toward the nose, outer lifts.
        "brow_l_i": (+3, +10), "brow_r_i": (-3, +10),
        "brow_l_p": (0, +3),   "brow_r_p": (0, +3),
        "brow_l_o": (0, -8),   "brow_r_o": (0, -8),
        # eyes: squint (top down, bottom up) + shift inward toward nose.
        "eye_l_t":  (0, +2),   "eye_r_t":  (0, +2),
        "eye_l_b":  (0, -2),   "eye_r_b":  (0, -2),
        "eye_l_i":  (+2, 0),   "eye_r_i":  (-2, 0),
        "eye_l_o":  (+2, 0),   "eye_r_o":  (-2, 0),
        # nose: scrunch — bridge down, base/tip up.
        "nose_top": (0, +5),
        "nose_l":   (0, -3),   "nose_r":   (0, -3),
        "nose_tip": (0, -3),
        # mouth: thin frown.
        "mouth_l":  (0, +2),   "mouth_r":  (0, +2),
        "mouth_t":  (0, +2),   "mouth_b":  (0, -4),
    },

    "surprised": {
        # Brows: the ENTIRE brow lifts high and arches — inner AND outer
        # corners together (unlike sad), peak highest for the arch.
        "brow_l_o": (0, -11), "brow_l_p": (0, -14), "brow_l_i": (0, -11),
        "brow_r_o": (0, -11), "brow_r_p": (0, -14), "brow_r_i": (0, -11),
        # Eyes: widest of the set — upper lids pull way back, lower lids
        # drop (narrowed=happy, drooped=sad, wide=surprised).
        "eye_l_t":  (0, -5),  "eye_r_t":  (0, -5),
        "eye_l_b":  (0, +4),  "eye_r_b":  (0, +4),
        # Mouth: jaw drops into a slack open O — big vertical opening,
        # corners pulled IN (rounded, not stretched wide) and down a touch.
        "mouth_t":  (0, -5),
        "mouth_b":  (0, +12),
        "mouth_l":  (+6, +2), "mouth_r":  (-6, +2),
        # Nose at rest.
    },
}

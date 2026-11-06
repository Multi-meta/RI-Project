"""
generate_aruco.py — Generate ArUco marker PNG textures for boxes and shelf slots.

Per checklist 13.3 & addendum §A7.3:
- DICT_4X4_50
- IDs 1..12: boxes (PKG_1..PKG_12)
- IDs 20..35: shelf slots
- Generates 256x256 PNGs in worlds/textures/aruco/ and worlds_output/textures/aruco/
"""

import os
import cv2
import numpy as np

TEXTURES_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "worlds", "textures", "aruco"))
OUTPUT_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "worlds_output", "textures", "aruco"))


def generate_markers():
    os.makedirs(TEXTURES_DIR, exist_ok=True)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # DICT_4X4_50
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_size = 256
    
    # 1..12 for boxes, 20..35 for shelf slots
    marker_ids = list(range(1, 13)) + list(range(20, 36))
    
    generated = []
    for marker_id in marker_ids:
        # cv2.aruco API
        if hasattr(cv2.aruco, 'generateImageMarker'):
            img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
        else:
            img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size)
            
        fname = f"marker_{marker_id}.png"
        path1 = os.path.join(TEXTURES_DIR, fname)
        path2 = os.path.join(OUTPUT_DIR, fname)
        
        cv2.imwrite(path1, img)
        cv2.imwrite(path2, img)
        generated.append(fname)
        
    print(f"[ARUCO] Generated {len(generated)} markers in {TEXTURES_DIR} and {OUTPUT_DIR}")
    return generated


if __name__ == "__main__":
    generate_markers()

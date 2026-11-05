"""Camera box plate (box + lid + 2 shims) for the Neptune 3 Pro.  python3 print/make_camera_box_plate.py"""
src = open("print/make_stage4_plates.py").read().split("parts = {")[0]
exec(src)
OUT = "print/camera_box"
names = ["camera_box", "camera_lid", "camera_shim_06", "camera_shim_12"]
P = {k: load(f"stl/{k}.stl") for k in names}
spots = {"camera_box": (BED_C - 20, BED_C), "camera_lid": (BED_C + 20, BED_C),
         "camera_shim_06": (BED_C - 14, BED_C - 32), "camera_shim_12": (BED_C + 14, BED_C - 32)}
for k in names:
    write_stl(f"{OUT}/{k}.stl", place(P[k], BED_C, BED_C))
write_3mf(f"{OUT}/plate_camera_box.3mf", [(k, place(P[k], *spots[k])) for k in names])
print("written", OUT)
# Box alone, centred (reprint without the lid)
write_3mf(f"{OUT}/plate_camera_box_only.3mf", [("camera_box", place(P["camera_box"], BED_C, BED_C))])
print("written", f"{OUT}/plate_camera_box_only.3mf")
# Reprint: camera box + camera lid + bay lid, spaced ≥ 20 mm apart for an 8 mm brim
P["bay_lid"] = load("stl/bay_lid.stl")
reprint = {"bay_lid": (70, BED_C), "camera_box": (150, BED_C - 27.5), "camera_lid": (150, BED_C + 27.5)}
write_3mf(f"{OUT}/plate_reprint_box_and_lids.3mf", [(k, place(P[k], *xy)) for k, xy in reprint.items()])
print("written", f"{OUT}/plate_reprint_box_and_lids.3mf")
# Lid for the wide-angle OV2640 module (2026-10-22): lid only, centred
write_3mf(f"{OUT}/plate_camera_lid_wide.3mf", [("camera_lid_wide", place(load("stl/camera_lid.stl"), BED_C, BED_C))])
print("written", f"{OUT}/plate_camera_lid_wide.3mf")
# v5 camera mount (2026-10-22): 25 deg wedge + box (screws at x = +-8) + wide-lens lid, one plate, 15 mm apart
v5 = {"camera_wedge": (BED_C - 40, BED_C), "camera_box": (BED_C, BED_C), "camera_lid": (BED_C + 40, BED_C)}
# + the split lens ring, all three heights (3.5 / 4.0 / 4.5 mm), in a row in front: keep the one that fits snugly
for i, hh in enumerate(("35", "40", "45")):
    for j, half in enumerate(("a", "b")):
        v5[f"lens_ring_{hh}_{half}"] = (BED_C - 50 + i * 50 + j * 22, BED_C - 45)
write_3mf(f"{OUT}/plate_camera_v5_wedge_box_lid.3mf", [(k, place(load(f"stl/{k}.stl"), *xy)) for k, xy in v5.items()])
print("written", f"{OUT}/plate_camera_v5_wedge_box_lid.3mf")

# Stage 1 print files: base and electronics bay

These plates are laid out for a 225 × 225 mm bed (ELEGOO Neptune 3 Pro).

Open each `.3mf` in Cura. The parts are already placed and the right way up, so slice and print.
The loose `.stl` files are the same parts in print orientation, for any other slicer.

| Order | File | Parts | Time (rough) | Infill |
|---|---|---|---|---|
| 1 | `plate1_fit_test.3mf` | fit_test_mg995 | ~15 min | 20 % |
| 2 | `plate2_base.3mf` | base, with the 30° seat for the camera mount | 8–10 h | 30 % (20–40 % is fine) |
| 3 | `plate3_lid_and_pointer.3mf` | bay_lid + pan_pointer | ~2.5 h | 20 % |
| (reprint) | `plate4_lid_only.3mf` | bay_lid with crush ribs | ~1 h | 20 % |

The lid has eight small crush ribs on its plug, 0.15 mm proud of the bay walls, for a snug push fit.
It screws down with 2× M3 × 10 mm pan-head screws into the posts in the bay's back corners. The posts have 2.5 mm pilots that the screw threads cut into.
Turn the screws in gently the first time, and don't overtighten them in plastic.
If it's too tight, scrape or sand the ribs. If it's still loose, make them taller in `cad/highfive_robot.py`.

Print plate 1 first. Drop the MG995 into the window: the screws should bite and the round horn should press snugly into the recess.
If anything is off, measure and change `SERVOS` / `CLR` in `cad/highfive_robot.py` before printing the base.

## Cura settings (on top of the stock Neptune 3 Pro profile, Standard 0.2 mm)

- Material: PLA (or PETG), 0.4 mm nozzle
- Layer height: 0.2 mm
- Wall line count: **3**
- Infill: see the table above
- Support: **off** (nothing needs it)
- Build plate adhesion: **skirt**. If the base corners lift, use a brim of 4 mm or less. The base is 192.7 mm long.

## What to expect

- **base:** the camera seat is tilted 30°, and its walls overhang gently. The wire tunnels are short bridges of 8–10 mm. None of this needs support.
- **pan_pointer:** the horn recess sits on the underside, so the slicer bridges a roughly 22 mm circle over it. Expect the first bridged layer to look a little rough. The horn only has to sit in it.
- **pan_pointer support:** the arrow overhangs the disc rim by 35 mm. The plate includes a small grey rib grid under it, modelled 0.25 mm below the arrow, so keep Cura's own supports **off**. After printing, snap the grid off by hand or with flush cutters. (`stl/pan_pointer_support.stl` is the same grid on its own.)
- **bay_lid:** prints top face down, with the plug rim facing up. The rounded slot at the edge next to the turntable is for the arm's cable bundle in stage 2. It stays empty in stage 1.

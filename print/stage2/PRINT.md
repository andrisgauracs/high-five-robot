# Stage 2 print files: shoulder and upper arm

These plates are laid out for a 225 × 225 mm bed (ELEGOO Neptune 3 Pro).

Clear the Cura build plate first (Edit → Clear Build Plate), then open each `.3mf`.
Same settings as stage 1: PLA or PETG, 0.2 mm layers, 3 walls, supports off, skirt.

| Order | File | Parts | Infill | Notes |
|---|---|---|---|---|
| 1 | `plate1_shoulder_and_spacers.3mf` | shoulder + 3× pivot_spacer | 30 % | Disc down, servo mount standing up |
| 2 | `plate2_upper_arm.3mf` | upper_arm | 25 % | Lying on its side, as in the file. PETG is the safer pick: the fork flexes a little during assembly |
| or | `plate3_shoulder_and_upper_arm.3mf` | shoulder + upper_arm together | 30 % | One long print instead of two |

## What's new in these parts

- **Zip-tie mounts.** Mount A sits on the back of the servo mount, near the top, and holds the shoulder servo's cable right where it leaves the servo. Mount B sits on the turntable's back edge and anchors the cable before its slack loop into the bay lid. Each has a 3.5 × 2 mm tunnel for a 2.5 mm zip tie.
- **Wider pivot gap: 5 mm between the fork and the servo mount.** This gives the horn room to slide onto the servo spline during assembly. Afterwards you fill the gap with a printed C-spacer.
- **Zip-tie slots on the upper arm.** Two pairs of 3.5 × 1.8 mm slots through one side rail, near each end of the open ladder section. From stage 3 on, the elbow (and later wrist) cable lies against the inside of that rail, held by a tie looped through each pair.
- **Shoulder's servo window: 2 mm taller, with a cable dent at the bottom.** The window is 43.9 mm for the 40.7 mm servo and 0.6 mm looser per side. A 5 × 3 mm dent in its bottom edge (the end near the turntable) gives the cable room to pass.
- **Upper arm: cable channel under the elbow servo.** The elbow servo's cable drops from its end into a 6 mm groove in the floor, runs under the servo, and comes out in the open ladder section, then follows the zip-tie slots. The forearm has the same channel for the wrist servo.
- **Upper arm: open-top servo mount.** There's no roof over the elbow servo's pocket, and its window is open at the top, so nothing prints over air there. The elbow servo (stage 3) **drops straight in from above**, cable and all, and screws into the two ends of the flange wall as usual. The forearm has the same design.
- **Horn pockets: 21.5 mm in the shoulder, 21.7 mm in the arm forks.** The flat test suggested 21.0, but 21.0 barely fit printed on the bed and didn't fit at all printed sideways. In the sideways test, both 21.5 and 21.7 fit, and the forks use the roomier 21.7. The shoulder's pocket opens onto the bed, so it has a 0.5 mm chamfered mouth. The arm forks print standing up, so their pockets have a 45° teardrop top instead of a round arch that would sag. Each pocket is 0.6 mm deeper because your horn sits high.
- **Optional check first:** `plate4_horn_test2.3mf` (about 20 min) has pockets printed the same two ways (21.3 / 21.5 / 21.7 opening onto the bed, and 21.5 / 21.7 sideways teardrops). Try the horn in each before printing the big parts.

## Assembling the shoulder joint

1. Put an M3 nut in the hex trap on the inside of the shoulder's idler wall (the mount wall without the servo window).
2. Slide the shoulder servo through the window, cable end up, threading the cable through the dent at the bottom of the window. Screw its flange down.
3. Screw the round horn into the recess in the upper arm's fork plate.
4. Lower the fork over the servo mount, shifted about 4 mm **towards the horn side** (the way the servo shaft points), so the horn clears the spline and the fork's idler plate moves into the 5 mm gap. Then slide it back so the horn presses onto the spline, and drive the horn's centre screw through the access hole. The animation `renders/shoulder_assembly.mp4` shows each step.
5. Slide a pivot spacer into the gap between the fork and the idler wall, around where the bolt goes.
6. Push an **M3 × 20** bolt through the fork plate, the spacer and the wall into the nut. Snug it, but don't clamp it.
7. Once the servo is seated, bend its cable **straight back over the back of the mount**, just above the servo, not up over the top of the wall. Keep it low there: the upper arm's fork passes over this spot when the arm folds up.
   Then run it down the back through zip-tie mount A, to mount B, then in a loose loop of about 8–10 cm through the lid slot to the SHOULDER header.

Pivot bolts for the whole arm: **M3 × 20** for the shoulder and elbow, **M3 × 16** for the wrist. No washers are needed, because the spacers fill the gap.

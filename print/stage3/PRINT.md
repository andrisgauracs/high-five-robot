# Stage 3 print files: forearm

These plates are laid out for a 225 × 225 mm bed (ELEGOO Neptune 3 Pro).

Clear the Cura build plate first (Edit → Clear Build Plate), then open each `.3mf`.
PLA or PETG, 0.2 mm layers, 3 walls, supports off, skirt.

| Order | File | Parts | Infill | Notes |
|---|---|---|---|---|
| 1 | `plate1_fit_test_mg90s.3mf` | fit_test_mg90s (2 pieces) | 20 % | About 15 min. Check the MG90S before printing the forearm |
| 2 | `plate2_forearm.3mf` | forearm | 25 % | Lying on its side, as in the file. PETG is the safer pick for the fork |

## MG90S fit test

- **Frame:** the MG90S body should drop through the window, and its two flange screws should bite.
- **Pad:** the bar horn should sit snugly in the stadium-shaped pocket. This pocket prints open on top, but the hand's pocket (stage 4) prints sideways, so allow a little extra there.
- Measure the MG90S with calipers too: body length and width, flange hole spacing, and the bar horn's length and width. If anything differs from 22, change `SERVOS` in `cad/highfive_robot.py` before printing the forearm.8 × 12.2 mm, 27.8 mm, and 32 × 6 mm.

## Forearm

Same features as the upper arm: a 21.7 mm teardrop horn pocket for the elbow servo's horn, a 5 mm pivot gap for a C-spacer, and an open-top mount that the MG90S drops into from above. It also has a cable channel in the floor under the MG90S and zip-tie slot pairs on one rail.
The elbow pivot uses an **M3 × 20** bolt and an M3 nut, like the shoulder.

## Wrist cable across the elbow (checked in the model)

The forearm's fork can't route the cable through its middle: the solid cross beam closes it, and the elbow servo sits inside the fork. Route it this way instead:
1. From the MG90S, through the floor channel and along the forearm's ladder section, then **out of the bottom** of the ladder section.
2. Under the forearm, past the cross beam, and around the **outside of the fork's idler plate** (the plate with the bolt head), **close to the pivot**. Leave about **3 cm of slack** there: the two ends of that loose piece move up to about 20 mm apart as the elbow bends.
3. Under the upper arm, then up into its ladder section, and on to the shoulder with the elbow servo's cable.

In the model, a 2.4 mm cable on this route touches nothing over the whole elbow range (13° to 123°). The same route run inside the plate collides, which confirms the check works.

# Stage 4 print files: wrist twist and hand

These plates are laid out for a 225 × 225 mm bed (ELEGOO Neptune 3 Pro).

Clear the Cura build plate first (Edit → Clear Build Plate), then open the `.3mf`.
PLA or PETG, 0.2 mm layers, 3 walls, supports off, skirt.

| File | Parts | Infill | Notes |
|---|---|---|---|
| `plate1_hand_and_twist.3mf` | hand + twist bracket + hand mount + 2 C-spacers | 20 % | Everything for stage 4 on one plate |
| `plate1a_hand_only.3mf` / `plate1b_twist_parts_only.3mf` | hand / the three twist parts | hand 15 %, twist parts 30 % | If you'd rather print them separately |

## What's new: the wrist twist (5th joint)

- **The wrist adapter is gone.** The **twist bracket** replaces it. It has the same two fins on the wrist MG90S horn (same M3 × 16 bolt and C-spacer). It also holds the **second MG90S**, lying on its side with its shaft pointing along the hand, towards the fingers.
- **The hand mount** is a small fork that turns on that servo. The horn sits in one plate. An **M3 × 16** idler bolt, a nut and a C-spacer go on the other, like every other joint. The hand screws to its flat base with **4 × M3 × 8**.
- **The hand twists ±60°** about an axis along its length. The axis runs about 35 mm above the back of the hand, so at full twist the palm also swings about 3 cm sideways.
- **The hand is reprinted.** The palm starts 32 mm further out, so it can turn clear of the bracket. The screw seat sits under the twist joint, and the hand prints palm down as before.
- **Print orientation, all as they come in the file:** the bracket prints base down, the hand mount prints plate down with its fork up, and the spacer prints flat. Each part is one piece. Every pocket that prints sideways has extra clearance or a peaked top.

## Checked in the model

- Over the whole wrist range (−38° to 111°) × twist (−60°, −30°, 0, 30°, 60°) and the rest, slap and reach poses, the hand, the mount, the bracket, the twist servo and the screw heads touch nothing. The closest gap is 1.0 mm, between the mount and the bracket.
- **The twist servo drops into its cradle from above,** with the cable bump at the shaft end. Nothing touches anywhere along the way.
- **The twist idler bolt goes in through a 7 mm tunnel in the bracket's spine.** The screwdriver fits, and the head turns inside the tunnel. The bolt passes the nut and stops 1 mm short of the servo. The C-spacer clears the bracket.
- **The twist cable** runs down the cradle and over the top of the spine. It then goes outside the wrist idler fin and joins the wrist cable along the forearm. It clears everything over the full wrist and twist ranges, with a ~3 cm slack loop at the wrist pivot. A route inside the fin collides, which confirms the check catches it.
- **Load:** the shoulder torque with the arm straight out goes from 4.7 to 6.1 kg·cm (MG995 stall ≈ 9). The wrist load goes from 0.45 to 0.86 kg·cm (MG90S stall ≈ 1.8).

## Assembly: build the twist joint on the bench, then mount it

1. **Hand mount → hand:** 4 × M3 × 8 down through the mount's base into the hand's pilots. Turn them in gently, they cut their own thread. Do this first, while the screw heads are still reachable.
2. **Twist servo → bracket:** put an M3 nut in the nut trap on the inner side of the cradle wall. Drop the second MG90S into the cradle from above, shaft towards the fingers, cable at the shaft end, and screw it down. Drill 1.5 mm pilots for the bar horn and screw it into the mount's horn-side plate.
3. **Hand + mount → twist servo:** centre the servo first (TWIST header, GPIO15, 1500 µs). Fit the fork over it with the palm facing straight down, shift it towards the horn side, then slide it back onto the spline. Drive the horn's centre screw through the access hole in the plate.
4. **Twist idler bolt:** push the C-spacer into the gap. Then drive an **M3 × 16** through the tunnel in the spine, through the idler plate, the spacer and the wall into the nut. Snug, not clamped.
5. **Bracket → wrist:** same as the old adapter. Put an M3 nut in the forearm's wrist nut trap and seat the wrist MG90S with its cable in the floor channel. Screw the wrist bar horn into the bracket's horn-side fin. Fit the fins over the servo, slide them onto the spline and drive the centre screw. Then the C-spacer and the **M3 × 16** wrist bolt.
6. **Twist cable:** zip-tie it to the top of the spine, then down outside the wrist idler fin. Leave a ~3 cm loop at the wrist pivot, then run it along the forearm with the wrist cable. It goes to the **TWIST header (GPIO15)**. It will need a servo extension, like the wrist cable.

**Parts for this stage:** 2 × MG90S (wrist + twist), 2 × M3 × 16 + 2 × M3 nuts, 4 × M3 × 8, 2 printed C-spacers (both on the plate).

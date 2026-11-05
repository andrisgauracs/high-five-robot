# Camera mount: XIAO ESP32S3 Sense with a wide-angle lens

The mount looks 60° up: the base's seat gives 30°, and a 30° wedge adds the rest.

`plate_camera_v5_wedge_box_lid.3mf` has every part on one plate. Print it as it lies: PLA, 0.2 mm layers, 3 walls, 20 % infill, **supports off**, skirt.

| Part | Size | Notes |
|---|---|---|
| `camera_wedge` | 28.5 × 34.9 × 24.0 mm | Sits in the seat's lip, thick end at the bottom of the slope |
| `camera_box` | 28.5 × 34.9 × 16.8 mm | Holds the XIAO. The USB-C end is open, and the other end has a notch for the antenna wire |
| `camera_lid` | 28.5 × 34.9 × 9.6 mm | Has a collar that locates the lens module, and an 11.6 mm slot for the lens |
| `lens_ring_35/40/45_a/b` | small | Optional. Two halves press together around the lens neck. Three heights, because the neck length varies; keep the one that fits snugly |
| `camera_shim_06/12` (in `stl/`) | 0.6 and 1.2 mm | Go under the XIAO if the camera rattles |

**Hardware:** 2 × M3 × 8 (or × 10) for the wedge, 4 × M3 × 25 and 4 M3 nuts for the box and lid, plus 2 M3 nuts in the base's seat. A small piece of double-sided foam tape for the lens module.

## Assembly

1. Press an M3 nut into each of the two hex pockets in the camera seat on the front of the base. They're a snug fit.
2. Put the wedge on the seat, thick end down the slope. Fit the 2 short screws through their deep holes into the seat's nuts.
3. Press 4 M3 nuts into the hex pockets on top of the wedge.
4. Snap the antenna plug onto the XIAO's socket, then click the camera board on top. Swap the stock camera for the wide-angle OV2640 module, and seat its ribbon cable fully. A poor ribbon contact shows up as black dots in the picture.
5. Stick the module to the camera board with foam tape, where the old module sat.
6. Put the camera in the box, **flat XIAO underside down**, with the USB-C end and the camera module at the open end. The antenna wire leaves through the notch at the other end.
7. Fit the lid, with the collar around the module. The lens's front ring passes up through the slot and stands about 9 mm above the lid.
8. Fit the 4 M3 × 25 screws through the lid and the box into the wedge's nuts, until the lid sits flat. The heads sit in 1 mm counterbores, so use M3 × 30 if you need more reach.
9. The power lead goes in through the floor slot. See [the wiring guide](../../docs/WIRING.md#camera-power-lead).

The mount was printed and fitted on the real robot, but the lens neck length was estimated from photos, which is why the ring comes in three heights.

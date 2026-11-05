# Bill of materials

**Rough total: about $140 to $230 in parts.** That leaves out the 3D printer, your computer and tools.

Prices are rough USD estimates from November 2026, before tax and shipping. The servos, boards, camera, supply, screws and filament are based on retail listings; the small items are ballpark figures. Prices vary a lot by seller and country, so treat every number as a guide. Several items only come in packs, so you'll have spares.

## Electronics

| Part | Qty | Rough cost | Notes |
|---|---:|---:|---|
| MG995 servo | 3 | $26–37 | Pan, shoulder, elbow. Often sold as a 4-pack, which gives you a spare. Check that each comes with a round horn |
| MG90S micro servo | 2 | $10–14 | Wrist and twist. Get the metal-gear MG90S, not the plastic SG90 |
| ESP32-S3 N16R8 dev board | 1 | $10–15 | Drives the servos. 16 MB flash, 8 MB PSRAM |
| Seeed XIAO ESP32S3 Sense | 1 | $14–20 | The camera board. Get the **Sense** version. Keep its antenna |
| Wide-angle OV2640 module, about 120° | 1 | $6–12 | Replaces the stock camera. Must have the 24-pin ribbon that fits the XIAO |
| 5 V 6 A (30 W) power supply | 1 | $12–23 | Fixed 5.5 × 2.1 mm plug, centre positive. Avoid adapters with swappable tips, and check the output with a multimeter |
| Panel-mount DC jack, 5.5 × 2.1 mm | 1 | $3–5 | Threaded, for the 11 mm hole in the bay. Pre-wired with 18 AWG leads saves soldering |
| 1000 µF 16 V electrolytic capacitor | 1 | $1–2 | Radial, about 10 × 17 mm |
| Perfboard, 50 × 70 mm | 1 | $6–10 for the set | Double-sided with plated holes is easiest. The set includes headers: 2 × female 1 × 22 (count your board's pins first), a male 1 × 40 strip and jumper caps |
| Wire and heat-shrink | 1 set | $5–10 | Bare tinned 20 AWG for the two buses, thinner hookup wire for signals |
| Dupont jumper wires | 1 pack | $5–10 | One 2-wire female lead powers the camera |
| Servo extension leads, 30 cm | 3 | $4–6 | Silicone wire flexes best. The wrist and twist cables run about 45 cm to the bay |

## Hardware

An M3 screw and nut assortment covers most of this, for about $12–25.

| Part | Qty | Used for |
|---|---:|---|
| M3 × 20 bolt | 2 | Shoulder and elbow pivots |
| M3 × 16 bolt | 2 | Wrist pivot and twist idler |
| M3 × 25 screw | 4 | Camera box and lid |
| M3 × 8 screw | 6 | Hand to mount (4), wedge to seat (2) |
| M3 × 10 pan-head screw | 2 | Bay lid |
| M3 hex nut, standard (not nyloc) | 10 | 4 in the joints, 2 in the camera seat, 4 in the wedge |
| M2 × 5 mm nylon standoff | 4 | Lift the perfboard off the bay floor. 5 mm foam tape works too |

| Other | Qty | Rough cost | Notes |
|---|---:|---:|---|
| Small zip ties, 2.5 mm wide | 1 pack | $3–5 | Cable routing |
| Wood screws, #8 × 16–20 mm, pan head, with washers | 4 | $2–3 | Fix the base to the board. No countersunk heads, they can split the foot |
| Wooden board or cutting board, about 18 mm thick | 1 | $6–12 | Weight and stability. Greg slaps hard, so don't skip it |
| Double-sided foam tape | 1 | $0–3 | Holds the lens module |

Servo mounting screws and horn screws come with the servos.

## Filament

| Part | Qty | Rough cost | Notes |
|---|---:|---:|---|
| PLA or PETG, 1.75 mm | 1 kg | $15–20 | The whole robot uses about half a spool. PETG is a safer pick for the arm links, because their forks flex a little during assembly |

## Tools you'll need

Not counted in the total:

- A 3D printer with a bed of at least 225 × 225 mm. The base is 193 mm long. The original was printed on an ELEGOO Neptune 3 Pro.
- Soldering iron, solder and flux
- Multimeter
- Digital calipers. Servo clones vary by about 1 mm, so measure yours before printing the big parts
- Small Phillips screwdrivers, a 2.5 mm hex key, and a 1.5 mm drill bit for horn screw pilot holes
- A computer to run the vision server. It was built and tested on a Mac

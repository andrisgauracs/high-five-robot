"""Grab one upright camera frame (see cam_view.py for the transform).  python3 tools/cam_snap.py out.jpg [camera-ip]"""
import sys, urllib.request
import numpy as np, cv2

CAM = sys.argv[2] if __name__ == "__main__" and len(sys.argv) > 2 else "highfive-cam.local"


def upright(raw_img):
    """Raw sensor frame -> upright, unmirrored (transpose; = ffmpeg transpose=clock,vflip)."""
    return np.ascontiguousarray(raw_img.transpose(1, 0, 2)[::-1, ::-1])


# vflip that gives the correct (unmirrored) upright view. It depends on the sensor: the stock OV3660 needed 0, the
# wide-angle OV2640 fitted 2026-10-22 reads out the other way and needs 1 (checked against the room: the big zigzag of
# fairy lights must be on the LEFT).
CAM_VFLIP = 1


def cam_setup(cam=CAM):
    """Put the camera in the state every tool assumes: VGA (640 x 480 raw -> 480 x 640 upright), no flips.
    VGA is 4:3 like the sensor, so it shows the full view (HVGA 480 x 320 is 3:2 and crops the top and bottom).
    Framesize first: changing it can reset the flip bits. The firmware turns vflip ON at every boot, and vflip on
    gives a MIRRORED upright picture (it caught the first calibration out, 2026-10-20). Correct view: a person's
    raised right hand appears on the left; in this room the fairy lights are on the left."""
    for var, val in (("framesize", 10), ("vflip", CAM_VFLIP), ("hmirror", 0)):
        urllib.request.urlopen(f"http://{cam}/control?var={var}&val={val}", timeout=5).close()


def snap(out=None):
    urllib.request.urlopen(f"http://{CAM}/capture", timeout=8).read()          # drop the stale buffered frame
    raw = urllib.request.urlopen(f"http://{CAM}/capture", timeout=8).read()
    img = upright(cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR))
    if out:
        cv2.imwrite(out, img)
    return img


if __name__ == "__main__":
    snap(sys.argv[1])

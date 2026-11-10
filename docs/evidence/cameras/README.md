# Camera evidence — to be generated in Webots

This folder is filled by `controllers/manip_check` (mode `cameras`):

```bash
MANIP_CHECK_MODE=cameras webots --batch --mode=realtime --stdout --stderr worlds/warehouse_manip_check.wbt
```

It writes, for four robot headings, `<pose>_front.png`, `<pose>_rear.png`,
annotated versions, and `camera_validation.md/.json` comparing each camera's
OpenCV detections with that camera's own Recognition node.

**No images are committed yet**: the branch was developed without Webots/GPU
access and no screenshot was invented. Commit the generated files after the
run on the GPU laptop.

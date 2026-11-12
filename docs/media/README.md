# Media

The GIFs are generated from lithocam's own output; nothing is drawn by hand.

| File | Shows | Produced by |
|---|---|---|
| `ruled_labels.gif` | faces labelled wire, flank or point, with their rulings | `lithocam.ruled` |
| `milling_sim.gif` | the stair tread finished pass by pass in the Z-map simulation (ball R3, 0.2 mm scallop) | `zigzag`, `cut_passes` |
| `wire_cuts.gif` | a window-sill blank squared by planar diamond-wire cuts, with kerf, waste and the volume left for milling | `lithocam.wiresim` |
| `robot_wire_sweep.gif` | the UR10e sweeping the wire over the twisted column along its verified joint path | `lithocam.robot.wire_face_path` |

## Regenerate

```sh
python docs/media/make_gif_data.py build_py/Release build/gif_data   # needs OCP, build123d and the _lithocam module
python docs/media/make_robot_data.py build/gif_data                  # needs tesseract-robotics
python docs/media/render_gifs.py build/gif_data docs/media            # needs matplotlib and Pillow
```

# Media

The GIFs are generated from lithocam's own output; nothing is drawn by hand.

| File | Shows | Produced by |
|---|---|---|
| `ruled_labels.gif` | faces labelled wire, flank or point, with their rulings | `lithocam.ruled` |
| `milling_sim.gif` | the stair tread finished pass by pass in the Z-map simulation (ball R3, 0.2 mm scallop) | `zigzag`, `cut_passes` |
| `wire_cuts.gif` | a window-sill blank squared by planar diamond-wire cuts, with kerf, waste and the volume left for milling | `lithocam.wiresim` |
| `robot_wire_sweep.gif` | the UR10e sweeping the wire over the twisted column along its verified joint path | `lithocam.robot.wire_face_path` |
| `teach_handle.gif` | teaching targets with the move handle in the studio | `python -m lithocam.studio --teach-record DIR` |
| `disc_saw_kr120.gif` | KR120 disc-saw profile cuts with kerf compensation, stock simulated | studio `--record DIR 60 --stock --no-collisions examples/studio/disc_saw_kr120.lcs.json` |
| `disc_comb_ur10e.gif` | UR10e disc-saw comb roughing with a cross set | studio `--record DIR 60 --stock --no-collisions examples/studio/disc_comb_ur10e.lcs.json` |
| `five_axis_stock.gif` | the built-in dome: roughing, raster and 5-axis finishing in the stock simulation | studio `--record DIR 60 --five-axis-demo --stock` |
| `temple_relief_kr120.gif` | the padma relief panel on the KR120, recorded with 8 mm finishing stepovers | studio `--record DIR 70 --stock` on a copy of `temple_relief_kr120` with coarser stepovers |

Studio frames become GIFs with `python docs/media/frames_to_gif.py DIR OUT.gif [width] [ms] [crop l,t,r,b]`.

## Regenerate

```sh
python docs/media/make_gif_data.py build_py/Release build/gif_data   # needs OCP, build123d and the _lithocam module
python docs/media/make_robot_data.py build/gif_data                  # needs tesseract-robotics
python docs/media/render_gifs.py build/gif_data docs/media            # needs matplotlib and Pillow
```

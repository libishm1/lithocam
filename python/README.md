# lithocam (Python)

The pure-Python part of [lithocam](../README.md): stone feature input from STEP.

- `lithocam.features`: recognises a STEP B-rep with [quiddity](https://github.com/pzfreo/quiddity) and maps the records to stone operations and process routes.
- `lithocam.step_canon`: rewrites straight edges that are stored as degree-1 B-splines, as Rhino 8 writes them, into exact LINEs before recognition.

```sh
pip install -e ".[features,test]"
python -m lithocam.features part.step
pytest
```

The C++ core (kerf offset, scallop and stepover) will add Python bindings to this package in a later release.

# Individual photographic piece surfaces

The user supplied `docs/IMG_1183.jpg` (3024×4032). All 32 physical piece faces are
cropped from this native-resolution photograph. Their lettering, green/red colors
and photographic detail are retained; only the wooden background outside each
white-face convex hull is made transparent. Export to 512×512 does not increase
original photographic detail.

```bash
/usr/bin/python3 tools/crop_gazebo_piece_photo.py docs/IMG_1183.jpg
```

- `instances/`: 32 individual RGBA textures, one for each scene body.
- `photo_instances.json`: source SHA-256, crop coordinates, label and instance key.
- `provenance.json` and 14 top-level PNGs: per-class defaults for legacy checks.
- `photo_contact_sheet.jpg`: visual overview of all 32 extracted faces.
- `top.dae`: UV-mapped octagonal top face used by Gazebo.

Each scene piece keeps its texture key while moving or being captured. Class IDs
retain `blue_*` for Cho, while its actual photographed lettering is green.
The previous SVG texture variant is replaced. Recognition still receives only
rendered camera pixels, never scene labels or planned moves.

# OSM map preparation + Pygame viewer

## Install

Use your simulator's Python environment:

```bash
pip install -r requirements-map.txt
```

## Build a map

```bash
python -m map_tools.map_builder --place "Philadelphia, Pennsylvania" --diameter-miles 3 --output maps/philadelphia.json
```

`--place` accepts an address, ZIP code, neighborhood, or city understood by the OSM geocoder. A 3-mile diameter is the default.

## View it

```bash
python map_viewer.py maps/philadelphia.json
```

Viewer controls:
- `B`: toggle buildings
- `R`: toggle roads
- `L`: toggle parks/greenery/water/land-use
- `F`: fit map
- Mouse wheel: zoom
- Middle mouse drag: pan
- `Esc`: quit

## Coordinate contract

The map uses a local projected metric frame centered on the geocoded point:
- X increases east; Y increases north.
- Units are meters.
- The center is `(0, 0)`, so map bounds are negative/positive around the center.
- Z is not encoded in feature geometry; building height is stored as `height_m`.

When placing the simulator's base, drones, or orders on this map, use the same local coordinate frame. The viewer is standalone and does not yet integrate with `Scenario` or `PygameUI`.

## Caveats

- OSM data coverage varies by location. Sidewalks and building heights may be missing.
- Missing building heights use a configurable 10 m estimate; records preserve `height_source`.
- Street centerlines are included separately from mapped highway features.
- The downloader makes live requests to public OSM services; large areas or repeated requests may be throttled. Cache and reuse generated JSON.
- OSM attribution is included in map metadata. Keep attribution visible when redistributing or displaying the map.

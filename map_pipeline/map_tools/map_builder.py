"""Download and preprocess an OpenStreetMap area into a simulator-friendly JSON file.

Install:
    pip install osmnx pygame

Example:
    python -m map_tools.map_builder --place "Richmond, Virginia" --diameter-miles 3 --output maps/richmond.json
    python -m map_tools.map_builder --place "Philadelphia, Pennsylvania" --diameter-miles 2.5 --output maps/philly.json
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import osmnx as ox
from pyproj import Transformer
from shapely.geometry import box, mapping
from shapely.ops import transform


MILES_TO_METERS = 1609.344


def _local_transformer(lat: float, lon: float) -> tuple[str, Transformer]:
    """Return a WGS84 -> local UTM transformer suitable for a city-scale map."""
    zone = int((lon + 180) // 6) + 1
    epsg = (32600 if lat >= 0 else 32700) + zone
    crs = f"EPSG:{epsg}"
    return crs, Transformer.from_crs("EPSG:4326", crs, always_xy=True)


def _xy_geometry(geom, project, origin_x: float, origin_y: float):
    projected = transform(project, geom)
    return transform(lambda x, y, z=None: (x - origin_x, y - origin_y), projected)


def _coords_to_local(geom, project, origin_x: float, origin_y: float) -> dict[str, Any]:
    local = _xy_geometry(geom, project, origin_x, origin_y)
    return mapping(local)


def _feature_type(tags: dict[str, Any]) -> str:
    if tags.get("building") and tags.get("building") != "no":
        return "building"
    if tags.get("highway"):
        return "road"
    if tags.get("leisure") in {"park", "garden", "playground", "nature_reserve"}:
        return "park"
    if tags.get("landuse") in {"grass", "forest", "meadow", "recreation_ground", "village_green"}:
        return "green"
    if tags.get("natural") in {"water", "wood", "wetland", "scrub"}:
        return "water" if tags.get("natural") == "water" else "green"
    if tags.get("waterway") or tags.get("landuse") == "reservoir":
        return "water"
    if tags.get("landuse") in {"residential", "commercial", "retail", "industrial"}:
        return "landuse"
    return "other"


def _height(tags: dict[str, Any]) -> tuple[float, str]:
    raw = tags.get("height")
    if raw is not None:
        try:
            # OSM height values are commonly meters, sometimes suffixed with "m".
            value = float(str(raw).lower().replace("meters", "").replace("meter", "").replace("m", "").strip())
            if 1.0 <= value <= 500:
                return value, "osm_height"
        except (TypeError, ValueError):
            pass
    levels = tags.get("building:levels")
    if levels is not None:
        try:
            value = float(levels) * 3.2
            if 1.0 <= value <= 300:
                return value, "estimated_from_levels"
        except (TypeError, ValueError):
            pass
    return 10.0, "default_estimate"


def _feature_record(row, project, ox0: float, oy0: float) -> dict[str, Any] | None:
    geom = row.geometry
    if geom is None or geom.is_empty:
        return None
    tags = row.to_dict()
    kind = _feature_type(tags)
    if kind == "other":
        return None
    local = _xy_geometry(geom, project, ox0, oy0)
    record = {
        "id": str(row.name),
        "kind": kind,
        "tags": {
            key: str(tags[key]) for key in (
                "building", "highway", "landuse", "leisure", "natural",
                "waterway", "name", "building:levels", "height", "surface"
            ) if key in tags and tags[key] is not None
        },
        "geometry": mapping(local),
    }
    if kind == "building":
        h, source = _height(tags)
        record["height_m"] = h
        record["height_source"] = source
    return record


def build_map(place: str, diameter_miles: float = 3.0, output: str = "map.json",
              network_type: str = "drive") -> dict[str, Any]:
    """Build a square map centered on a geocoded place and write JSON."""
    if diameter_miles <= 0:
        raise ValueError("diameter_miles must be positive")

    ox.settings.use_cache = True
    ox.settings.log_console = True

    gdf = ox.geocode_to_gdf(place)
    if gdf.empty:
        raise ValueError(f"Could not geocode: {place}")
    center = gdf.geometry.iloc[0].representative_point()
    lon, lat = center.x, center.y
    crs, transformer = _local_transformer(lat, lon)
    project = transformer.transform
    center_x, center_y = project(lon, lat)

    radius = diameter_miles * MILES_TO_METERS / 2
    # Convert the square's projected corners back to WGS84 for OSM download.
    inverse = Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
    corners = [inverse.transform(center_x + dx, center_y + dy)
               for dx, dy in [(-radius, -radius), (-radius, radius),
                              (radius, -radius), (radius, radius)]]
    west = min(p[0] for p in corners)
    east = max(p[0] for p in corners)
    south = min(p[1] for p in corners)
    north = max(p[1] for p in corners)
    bbox = (west, south, east, north)

    # OSMnx 2.x bbox order is (west, south, east, north).
    tags = {
        "building": True,
        "highway": True,
        "landuse": True,
        "leisure": True,
        "natural": True,
        "waterway": True,
    }
    features = ox.features_from_bbox(bbox, tags)
    # Retrieve a routable road graph separately. It is exported as line features.
    graph = ox.graph_from_bbox(bbox, network_type=network_type, simplify=True)
    edges = ox.graph_to_gdfs(graph, nodes=False, edges=True)

    records = []
    for idx, row in features.iterrows():
        try:
            rec = _feature_record(row, project, center_x, center_y)
            if rec:
                records.append(rec)
        except Exception:
            continue

    road_records = []
    for idx, row in edges.iterrows():
        geom = row.geometry
        if geom is None or geom.is_empty:
            continue
        road_records.append({
            "id": f"road:{idx}",
            "kind": "road_centerline",
            "tags": {
                "highway": str(row.get("highway", "road")),
                "name": str(row.get("name", "")) if row.get("name") is not None else "",
                "oneway": str(row.get("oneway", "")),
            },
            "geometry": _coords_to_local(geom, project, center_x, center_y),
        })

    # Local origin is the requested center; map coordinates can be negative.
    # Keeping this center-origin frame makes geographic placement straightforward.
    map_data = {
        "schema_version": 1,
        "metadata": {
            "name": place,
            "source": "OpenStreetMap",
            "attribution": "© OpenStreetMap contributors",
            "center_latitude": lat,
            "center_longitude": lon,
            "projected_crs": crs,
            "coordinate_system": "local_projected_meters",
            "origin": "geocoded center",
            "units": "meters",
            "diameter_miles": diameter_miles,
            "bounds": [-radius, -radius, radius, radius],
            "network_type": network_type,
        },
        "features": records,
        "road_network": road_records,
        "delivery_locations": [],
        "flight_corridors": [],
        "no_fly_zones": [],
    }

    out = Path(output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(map_data, indent=2), encoding="utf-8")
    print(f"Saved {len(records)} map features and {len(road_records)} road edges to {out}")
    print("Map coordinates are meters relative to the geocoded center; center is (0, 0).")
    return map_data

def build_map_from_pbf(
    pbf_path: str,
    place: str,
    diameter_miles: float = 3.0,
    output: str = "map.json",
    network_type: str = "drive",
) -> dict[str, Any]:
    """Build a simulator map from a local OSM PBF extract."""
    from pyrosm import OSM

    if diameter_miles <= 0:
        raise ValueError("diameter_miles must be positive")

    pbf = Path(pbf_path)
    if not pbf.is_file():
        raise FileNotFoundError(f"OSM PBF file not found: {pbf}")

    # Geocode the requested center. This requires internet access, but
    # all map features and roads are read from the local PBF.
    ox.settings.use_cache = True
    ox.settings.log_console = True

    gdf = ox.geocode_to_gdf(place)
    if gdf.empty:
        raise ValueError(f"Could not geocode: {place}")

    center = gdf.geometry.iloc[0].representative_point()
    lon, lat = center.x, center.y

    crs, transformer = _local_transformer(lat, lon)
    project = transformer.transform
    center_x, center_y = project(lon, lat)

    radius = diameter_miles * MILES_TO_METERS / 2

    # Calculate the geographic bounding box covering the square map.
    inverse = Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
    corners = [
        inverse.transform(center_x + dx, center_y + dy)
        for dx, dy in [
            (-radius, -radius),
            (-radius, radius),
            (radius, -radius),
            (radius, radius),
        ]
    ]

    west = min(p[0] for p in corners)
    east = max(p[0] for p in corners)
    south = min(p[1] for p in corners)
    north = max(p[1] for p in corners)

    # Pyrosm reads the local PBF and limits processing to the map area.
    osm = OSM(
        str(pbf),
        bounding_box=[west, south, east, north],
    )

    # Clip output to the exact square in projected coordinates.
    clip_bounds = box(
        -radius,
        -radius,
        radius,
        radius,
    )

    def project_and_clip(geom):
        if geom is None or geom.is_empty:
            return None

        local = _xy_geometry(geom, project, center_x, center_y)
        clipped = local.intersection(clip_bounds)

        if clipped.is_empty:
            return None

        return clipped

    # Retrieve feature layers from the local extract.
    layers = [
        ("building", osm.get_buildings),
        ("landuse", osm.get_landuse),
        ("natural", osm.get_natural),
        # ("waterway", osm.get_waterways),
    ]

    records = []
    seen_features = set()

    for layer_name, getter in layers:
        try:
            layer = getter()
        except Exception as exc:
            print(f"Warning: could not read {layer_name} features: {exc}")
            continue

        if layer is None or layer.empty:
            continue

        for idx, row in layer.iterrows():
            geom = row.geometry
            if geom is None or geom.is_empty:
                continue

            tags = row.to_dict()

            # Reuse the existing feature classification and height logic.
            kind = _feature_type(tags)
            if kind == "other":
                continue

            local = project_and_clip(geom)
            if local is None:
                continue

            feature_id = str(row.get("id", idx))
            unique_key = (layer_name, feature_id)

            if unique_key in seen_features:
                continue

            seen_features.add(unique_key)

            record = {
                "id": feature_id,
                "kind": kind,
                "tags": {
                    key: str(tags[key])
                    for key in (
                        "building",
                        "highway",
                        "landuse",
                        "leisure",
                        "natural",
                        "waterway",
                        "name",
                        "building:levels",
                        "height",
                        "surface",
                    )
                    if key in tags and tags[key] is not None
                },
                "geometry": mapping(local),
            }

            if kind == "building":
                height, source = _height(tags)
                record["height_m"] = height
                record["height_source"] = source

            records.append(record)

    # Map the existing CLI network types to Pyrosm network types.
    network_types = {
        "drive": "driving",
        "walk": "walking",
        "bike": "cycling",
        "all": "all",
        "all_public": "all",
    }

    pyrosm_network_type = network_types.get(network_type)
    if pyrosm_network_type is None:
        raise ValueError(f"Unsupported network type: {network_type}")

    # Extract roads from the local PBF.
    try:
        edges = osm.get_network(network_type=pyrosm_network_type)
    except Exception as exc:
        raise RuntimeError(
            f"Failed to extract the {network_type} road network from {pbf}"
        ) from exc

    road_records = []

    if edges is not None and not edges.empty:
        for idx, row in edges.iterrows():
            geom = row.geometry
            if geom is None or geom.is_empty:
                continue

            local = project_and_clip(geom)
            if local is None:
                continue

            road_records.append({
                "id": f"road:{idx}",
                "kind": "road_centerline",
                "tags": {
                    "highway": str(row.get("highway", "road")),
                    "name": (
                        str(row.get("name", ""))
                        if row.get("name") is not None
                        else ""
                    ),
                    "oneway": str(row.get("oneway", "")),
                },
                "geometry": mapping(local),
            })

    map_data = {
        "schema_version": 1,
        "metadata": {
            "name": place,
            "source": "OpenStreetMap",
            "attribution": "© OpenStreetMap contributors",
            "center_latitude": lat,
            "center_longitude": lon,
            "projected_crs": crs,
            "coordinate_system": "local_projected_meters",
            "origin": "geocoded center",
            "units": "meters",
            "diameter_miles": diameter_miles,
            "bounds": [-radius, -radius, radius, radius],
            "network_type": network_type,
            "source_file": pbf.name,
        },
        "features": records,
        "road_network": road_records,
        "delivery_locations": [],
        "flight_corridors": [],
        "no_fly_zones": [],
    }

    out = Path(output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(map_data, indent=2), encoding="utf-8")

    print(
        f"Saved {len(records)} map features and "
        f"{len(road_records)} road edges to {out}"
    )
    print(
        "Map coordinates are meters relative to the geocoded center; "
        "center is (0, 0)."
    )

    return map_data

def main():
    parser = argparse.ArgumentParser(description="Download and prepare an OSM city map.")
    parser.add_argument("--place", required=True, help='Address, neighborhood, city, or ZIP code')
    parser.add_argument("--diameter-miles", type=float, default=3.0)
    parser.add_argument("--output", default="maps/map.json")
    parser.add_argument("--network-type", choices=["drive", "walk", "bike", "all", "all_public"], default="drive")
    parser.add_argument("--source", help="Path to a local OSM PBF file")
    args = parser.parse_args()

    if args.source:
        build_map_from_pbf(args.source, args.place, args.diameter_miles, args.output, args.network_type)
    else:
        build_map(args.place, args.diameter_miles, args.output, args.network_type)


if __name__ == "__main__":
    main()

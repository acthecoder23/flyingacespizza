"""Standalone Pygame viewer for map JSON produced by map_tools.map_builder."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import pygame


COLORS = {
    "building": (190, 184, 170),
    "road": (215, 215, 215),
    "road_centerline": (150, 150, 150),
    "park": (145, 190, 130),
    "green": (170, 205, 150),
    "water": (115, 175, 215),
    "landuse": (225, 215, 190),
}
BACKGROUND = (242, 240, 232)
INK = (65, 65, 65)


class MapViewer:
    def __init__(self, map_path: str, size=(1200, 850)):
        self.data = json.loads(Path(map_path).read_text(encoding="utf-8"))
        self.features = self.data.get("features", [])
        self.roads = self.data.get("road_network", [])
        self.bounds = self.data["metadata"]["bounds"]
        self.width, self.height = size
        pygame.init()
        pygame.display.set_caption(f"Map Viewer — {self.data['metadata'].get('name', 'OSM map')}")
        self.screen = pygame.display.set_mode(size, pygame.RESIZABLE)
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 22)
        self.small_font = pygame.font.SysFont(None, 17)
        self.show_buildings = True
        self.show_roads = True
        self.show_land = True
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.dragging = False
        self.last_mouse = (0, 0)
        self._fit()

    def _fit(self):
        x0, y0, x1, y1 = self.bounds
        self.base_scale = min((self.width - 60) / max(x1 - x0, 1), (self.height - 90) / max(y1 - y0, 1))
        self.zoom = 1.0
        self.pan_x = self.width / 2
        self.pan_y = self.height / 2

    def world_to_screen(self, point):
        x, y = point[:2]
        cx = self.width / 2 + self.pan_x - self.width / 2
        cy = self.height / 2 + self.pan_y - self.height / 2
        scale = self.base_scale * self.zoom
        return (int(cx + x * scale), int(cy - y * scale))

    def _draw_geometry(self, geom, color, width=0, outline=None):
        kind = geom.get("type")
        coords = geom.get("coordinates", [])
        if kind == "Polygon":
            rings = coords
            if not rings:
                return
            points = [self.world_to_screen(p) for p in rings[0]]
            if len(points) >= 3:
                pygame.draw.polygon(self.screen, color, points)
                if outline:
                    pygame.draw.polygon(self.screen, outline, points, 1)
        elif kind == "MultiPolygon":
            for polygon in coords:
                if polygon:
                    points = [self.world_to_screen(p) for p in polygon[0]]
                    if len(points) >= 3:
                        pygame.draw.polygon(self.screen, color, points)
                        if outline:
                            pygame.draw.polygon(self.screen, outline, points, 1)
        elif kind in ("LineString", "LinearRing"):
            points = [self.world_to_screen(p) for p in coords]
            if len(points) >= 2:
                pygame.draw.lines(self.screen, color, False, points, max(width, 1))
        elif kind == "MultiLineString":
            for line in coords:
                points = [self.world_to_screen(p) for p in line]
                if len(points) >= 2:
                    pygame.draw.lines(self.screen, color, False, points, max(width, 1))

    def render(self):
        self.width, self.height = self.screen.get_size()
        self.screen.fill(BACKGROUND)

        # Broad land/water polygons first, then buildings, then road markings.
        if self.show_land:
            for f in self.features:
                if f["kind"] in {"park", "green", "water", "landuse"}:
                    self._draw_geometry(f["geometry"], COLORS.get(f["kind"], COLORS["landuse"]))
        if self.show_roads:
            for f in self.features:
                if f["kind"] == "road":
                    self._draw_geometry(f["geometry"], COLORS["road"], width=2)
            for f in self.roads:
                self._draw_geometry(f["geometry"], COLORS["road_centerline"], width=1)
        if self.show_buildings:
            for f in self.features:
                if f["kind"] == "building":
                    self._draw_geometry(f["geometry"], COLORS["building"], outline=(125, 120, 110))

        title = self.data["metadata"].get("name", "Map")
        attribution = self.data["metadata"].get("attribution", "")
        self.screen.blit(self.font.render(title, True, INK), (12, 10))
        self.screen.blit(self.small_font.render(attribution, True, INK), (12, 34))
        help_text = "B buildings | R roads | L land/water | F fit | wheel zoom | middle-drag pan | Esc quit"
        self.screen.blit(self.small_font.render(help_text, True, INK), (12, self.height - 24))
        pygame.display.flip()

    def run(self):
        running = True
        while running:
            self.clock.tick(60)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_b:
                        self.show_buildings = not self.show_buildings
                    elif event.key == pygame.K_r:
                        self.show_roads = not self.show_roads
                    elif event.key == pygame.K_l:
                        self.show_land = not self.show_land
                    elif event.key == pygame.K_f:
                        self._fit()
                elif event.type == pygame.MOUSEWHEEL:
                    self.zoom = max(0.2, min(15.0, self.zoom * (1.15 ** event.y)))
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 2:
                    self.dragging = True
                    self.last_mouse = event.pos
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 2:
                    self.dragging = False
                elif event.type == pygame.MOUSEMOTION and self.dragging:
                    dx = event.pos[0] - self.last_mouse[0]
                    dy = event.pos[1] - self.last_mouse[1]
                    self.pan_x += dx
                    self.pan_y += dy
                    self.last_mouse = event.pos
                elif event.type == pygame.VIDEORESIZE:
                    self.screen = pygame.display.set_mode(event.size, pygame.RESIZABLE)
            self.render()
        pygame.quit()


def main():
    parser = argparse.ArgumentParser(description="Render a prepared map JSON in Pygame.")
    parser.add_argument("map_json", help="Path to map JSON")
    args = parser.parse_args()
    MapViewer(args.map_json).run()


if __name__ == "__main__":
    main()

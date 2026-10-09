"""Parse-only import helpers. They never create mission artifacts or plan geometry."""

from __future__ import annotations

import math
import os
import tempfile

from dyx3_backend.mission.service import MissionError


def check_dxf_upload(filename: str, data: bytes, max_bytes: int) -> None:
    if os.path.splitext(filename)[1].lower() != ".dxf":
        raise MissionError(415, "unsupported_type", "expected a .dxf file")
    if not data:
        raise MissionError(422, "empty_file", "the uploaded file is empty")
    if len(data) > max_bytes:
        raise MissionError(413, "too_large", f"upload exceeds {max_bytes} bytes")


def parse_dxf_upload(filename: str, data: bytes, max_bytes: int) -> dict:
    check_dxf_upload(filename, data, max_bytes)
    from dyx3_backend.path_engine.parsers.dxf_parser import parse_dxf

    with tempfile.TemporaryDirectory(prefix="dyx3-parse-") as td:
        path = os.path.join(td, "upload.dxf")
        with open(path, "wb") as fh:
            fh.write(data)
        try:
            entities = parse_dxf(path)
        except (ValueError, OSError, ImportError) as exc:
            raise MissionError(422, "parse_failed", f"DXF parse error: {exc}") from exc

    infos = []
    layers = set()
    geo_origin = None
    for ent in entities:
        layers.add(ent.layer)
        if geo_origin is None and ent.geo_origin is not None:
            geo_origin = [float(ent.geo_origin[0]), float(ent.geo_origin[1])]
        length = 0.0
        if ent.entity_type == "LINE":
            start, end = ent.geometry["start"], ent.geometry["end"]
            length = math.hypot(start[0] - end[0], start[1] - end[1])
        elif ent.entity_type == "CIRCLE":
            length = 2 * math.pi * ent.geometry["radius"]
        elif ent.entity_type == "ARC":
            sweep = (ent.geometry["end_angle"] - ent.geometry["start_angle"]) % 360.0
            length = ent.geometry["radius"] * math.radians(sweep)
        infos.append({"entity_type": ent.entity_type, "layer": ent.layer, "color": ent.color,
                      "entity_id": ent.entity_id, "is_mark": ent.is_mark(), "length_m": round(length, 3)})
    return {"filename": os.path.basename(filename), "num_entities": len(entities), "entities": infos,
            "unit_scale": entities[0].unit_scale if entities else 0.01,
            "layer_names": sorted(layers), "is_geographic": geo_origin is not None,
            "geo_origin": geo_origin}

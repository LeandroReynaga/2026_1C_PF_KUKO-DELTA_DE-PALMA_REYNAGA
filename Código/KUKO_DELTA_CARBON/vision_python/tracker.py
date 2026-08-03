from __future__ import annotations

from dataclasses import dataclass
from math import hypot

from config import (
    MAX_MISSED_FRAMES,
    MAX_TRACK_DISTANCE,
)
from detection import Detection


@dataclass
class TrackedObject:
    """Objeto cuyo ID se conserva entre fotogramas."""

    track_id: int

    center: tuple[int, int]
    previous_center: tuple[int, int]

    color: str
    shape: str

    bbox: tuple[int, int, int, int]
    contour: object

    area: float
    circularity: float

    missed_frames: int = 0

    crossed_line: bool = False
    crossing_time: float | None = None


class CentroidTracker:
    """Seguimiento sencillo basado en distancia entre centroides."""

    def __init__(self) -> None:
        self._next_id = 1

        self._tracks: dict[int, TrackedObject] = {}

    def _register(
        self,
        detection: Detection,
    ) -> None:
        track = TrackedObject(
            track_id=self._next_id,
            center=detection.center,
            previous_center=detection.center,
            color=detection.color,
            shape=detection.shape,
            bbox=detection.bbox,
            contour=detection.contour,
            area=detection.area,
            circularity=detection.circularity,
        )

        self._tracks[self._next_id] = track
        self._next_id += 1

    def _remove(self, track_id: int) -> None:
        self._tracks.pop(track_id, None)

    def update(
        self,
        detections: list[Detection],
    ) -> list[TrackedObject]:
        """Relaciona las detecciones nuevas con objetos existentes."""

        # Si no hay detecciones, incrementamos el contador
        # de desaparición para todos los objetos existentes.
        if not detections:
            for track in self._tracks.values():
                track.missed_frames += 1

            tracks_to_remove = [
                track_id
                for track_id, track in self._tracks.items()
                if track.missed_frames > MAX_MISSED_FRAMES
            ]

            for track_id in tracks_to_remove:
                self._remove(track_id)

            return []

        # Si todavía no existen objetos registrados,
        # registramos todas las detecciones.
        if not self._tracks:
            for detection in detections:
                self._register(detection)

            return list(self._tracks.values())

        candidate_pairs: list[
            tuple[float, int, int]
        ] = []

        for track_id, track in self._tracks.items():
            for detection_index, detection in enumerate(
                detections
            ):
                distance = hypot(
                    detection.center[0] - track.center[0],
                    detection.center[1] - track.center[1],
                )

                if distance <= MAX_TRACK_DISTANCE:
                    candidate_pairs.append(
                        (
                            distance,
                            track_id,
                            detection_index,
                        )
                    )

        # Primero relacionamos las parejas más cercanas.
        candidate_pairs.sort(key=lambda item: item[0])

        used_tracks: set[int] = set()
        used_detections: set[int] = set()

        for _, track_id, detection_index in candidate_pairs:
            if track_id in used_tracks:
                continue

            if detection_index in used_detections:
                continue

            track = self._tracks[track_id]
            detection = detections[detection_index]

            track.previous_center = track.center
            track.center = detection.center

            track.color = detection.color
            track.shape = detection.shape

            track.bbox = detection.bbox
            track.contour = detection.contour

            track.area = detection.area
            track.circularity = detection.circularity

            track.missed_frames = 0

            used_tracks.add(track_id)
            used_detections.add(detection_index)

        # Objetos que no encontraron detección.
        for track_id, track in self._tracks.items():
            if track_id not in used_tracks:
                track.missed_frames += 1

        # Detecciones que no encontraron un objeto existente.
        for detection_index, detection in enumerate(detections):
            if detection_index not in used_detections:
                self._register(detection)

        tracks_to_remove = [
            track_id
            for track_id, track in self._tracks.items()
            if track.missed_frames > MAX_MISSED_FRAMES
        ]

        for track_id in tracks_to_remove:
            self._remove(track_id)

        # Solamente devolvemos objetos vistos en este fotograma.
        return [
            track
            for track in self._tracks.values()
            if track.missed_frames == 0
        ]
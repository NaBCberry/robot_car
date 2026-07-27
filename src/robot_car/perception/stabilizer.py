"""Multi-frame confirmation and expiry filtering."""

from collections import defaultdict
from dataclasses import replace
from typing import DefaultDict, List, Optional, Set

from .events import VisionEvent


class EventStabilizer:
    def __init__(self, confirmation_frames: int = 3) -> None:
        self.confirmation_frames = max(1, confirmation_frames)
        self._counts: DefaultDict[str, int] = defaultdict(int)
        self._previous_keys: Set[str] = set()
        self._sources = {}

    def update(self, events: List[VisionEvent], now_ms: int,
               observed_sources: Optional[Set[str]] = None) -> List[VisionEvent]:
        current_keys = {event.key() for event in events if not event.is_expired(now_ms)}
        observed = observed_sources if observed_sources is not None else {event.source for event in events}
        for missing in self._previous_keys - current_keys:
            if self._sources.get(missing) in observed:
                self._counts.pop(missing, None)
                self._sources.pop(missing, None)
        self._previous_keys = {key for key in self._previous_keys
                               if self._sources.get(key) not in observed} | current_keys
        confirmed = []
        for event in events:
            if event.is_expired(now_ms):
                continue
            key = event.key()
            self._counts[key] += 1
            self._sources[key] = event.source
            if self._counts[key] >= self.confirmation_frames:
                confirmed.append(replace(event, confirmed=True))
        return confirmed

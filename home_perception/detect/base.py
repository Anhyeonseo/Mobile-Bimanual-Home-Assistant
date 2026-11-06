"""Shared detector output. Scores are model outputs, not probabilities."""
from dataclasses import dataclass


@dataclass
class Detection:
    label: str
    box_xyxy: list
    detector: str
    score: float | None = None
    prompt_name: str | None = None
    revision: str | None = None

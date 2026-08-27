#!/usr/bin/env python3
"""Candidate-only obstruction classification and PLY archiving for Scene1 v11.

The helpers in this module encode the conservative part of the reviewed
people/bag cleanup without choosing the ROIs or fitting the terrain models.
Callers supply independent surface predictions and graph-connectivity facts;
this module decides which points may be removed, validates that the decision
does not erode measured terrain, and writes a reviewable candidate bundle.

There is deliberately no install, promote, restore, or canonical-path API.
``write_candidate_archive`` reads its source once and publishes a new bundle
directory only after the observed source bytes match the required SHA-256.
The source is never opened writable.  Review and rejected points remain in the
candidate PLY and their classifications remain in lossless NumPy sidecars.
Only ``AUTO_REMOVE`` records are copied, byte-for-byte, to the removed archive.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
from typing import Sequence

import numpy as np


class CandidateMode(IntEnum):
    """How a caller's local connectivity analysis proposed a point."""

    NONE = 0
    GROW = 1
    SEED = 2


class PointDisposition(IntEnum):
    """Final point action.  Only ``AUTO_REMOVE`` changes candidate payload."""

    KEEP = 0
    AUTO_REMOVE = 1
    REVIEW = 2
    REJECTED = 3


class ObstructionReason(IntFlag):
    """Auditable reasons attached to retained or rejected classifications."""

    NONE = 0
    OUTSIDE_TARGET = 1 << 0
    NOT_A_CANDIDATE = 1 << 1
    INVALID_METRIC = 1 << 2
    INSUFFICIENT_MODELS = 1 << 3
    MODEL_SPREAD = 1 << 4
    BELOW_SEED_HEIGHT = 1 << 5
    BELOW_GROW_HEIGHT = 1 << 6
    GROUND_BAND_PROTECTED = 1 << 7
    NOT_CORE_CONNECTED = 1 << 8
    ROI_BOUNDARY_PROTECTED = 1 << 9
    COLLAR_PROTECTED = 1 << 10
    SCANID9_PROTECTED = 1 << 11
    FINE_SEED_MISSING_OR_TOO_FAR = 1 << 12
    FINE_INDEPENDENT_RESIDUAL_FAILED = 1 << 13


@dataclass(frozen=True)
class ObstructionThresholds:
    """Reviewed conservative defaults, in metres."""

    minimum_models: int = 2
    maximum_model_spread_m: float = 0.05
    seed_height_m: float = 0.08
    grow_height_m: float = 0.035
    review_height_m: float = 0.015
    ground_stop_height_m: float = 0.020
    fine_seed_distance_m: float = 0.008

    def __post_init__(self) -> None:
        if not isinstance(self.minimum_models, (int, np.integer)):
            raise ValueError("minimum_models must be an integer")
        if self.minimum_models < 2:
            raise ValueError("minimum_models must be at least two")
        values = (
            self.maximum_model_spread_m,
            self.seed_height_m,
            self.grow_height_m,
            self.review_height_m,
            self.ground_stop_height_m,
            self.fine_seed_distance_m,
        )
        if not all(np.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("obstruction thresholds must be finite and non-negative")
        if self.maximum_model_spread_m == 0.0:
            raise ValueError("maximum_model_spread_m must be positive")
        if self.fine_seed_distance_m == 0.0:
            raise ValueError("fine_seed_distance_m must be positive")
        if not self.seed_height_m >= self.grow_height_m:
            raise ValueError("seed_height_m must be >= grow_height_m")
        if not self.grow_height_m > self.ground_stop_height_m:
            raise ValueError("grow_height_m must be greater than ground stop")
        if not self.ground_stop_height_m >= self.review_height_m:
            raise ValueError("ground stop must be >= review_height_m")


@dataclass(frozen=True)
class ModelConsensusResult:
    """Largest mutually agreeing subset of independent surface predictions."""

    surface_height_m: np.ndarray
    agreeing_model_count: np.ndarray
    finite_model_count: np.ndarray
    model_spread_m: np.ndarray
    has_consensus: np.ndarray


@dataclass(frozen=True)
class SurfaceConsensusResult:
    """Model consensus plus a point's independent vertical residual."""

    surface_height_m: np.ndarray
    residual_m: np.ndarray
    agreeing_model_count: np.ndarray
    finite_model_count: np.ndarray
    model_spread_m: np.ndarray
    has_consensus: np.ndarray
    above_threshold: np.ndarray


@dataclass(frozen=True)
class FineTransferResult:
    """Whether a 5 mm removal may seed a corresponding 1 mm decision."""

    eligible: np.ndarray
    reason_mask: np.ndarray
    consensus: SurfaceConsensusResult


@dataclass(frozen=True)
class ObstructionClassification:
    """Per-point disposition with the metrics needed for review."""

    disposition: np.ndarray
    reason_mask: np.ndarray
    consensus_count: np.ndarray
    finite_model_count: np.ndarray
    model_spread_m: np.ndarray
    residual_m: np.ndarray
    required_residual_m: np.ndarray

    @property
    def auto_remove_mask(self) -> np.ndarray:
        return self.disposition == np.uint8(PointDisposition.AUTO_REMOVE)

    @property
    def retained_mask(self) -> np.ndarray:
        return ~self.auto_remove_mask


def _model_matrix(
    values: np.ndarray | Sequence[Sequence[float]],
) -> np.ndarray:
    models = np.asarray(values, dtype=np.float64)
    if models.ndim != 2 or models.shape[1] < 2:
        raise ValueError("surface predictions must have shape (N, M), M >= 2")
    return models


def _vector(
    values: np.ndarray | Sequence[float] | float | bool | int,
    count: int,
    name: str,
    dtype,
) -> np.ndarray:
    result = np.asarray(values, dtype=dtype)
    if result.ndim == 0:
        result = np.full(count, result.item(), dtype=dtype)
    if result.shape != (count,):
        raise ValueError(f"{name} must be scalar or contain one value per point")
    return result


def evaluate_model_consensus(
    surface_heights_m: np.ndarray | Sequence[Sequence[float]],
    *,
    minimum_models: int = 2,
    maximum_model_spread_m: float = 0.05,
) -> ModelConsensusResult:
    """Find the largest model subset whose height range is within tolerance.

    Models are independent evidence, not samples to average indiscriminately.
    A distant third prediction therefore cannot pull a valid two-of-three
    consensus away from the two agreeing surfaces.  Among equally large
    subsets the tightest one wins, with sorted order providing deterministic
    tie handling.
    """

    models = _model_matrix(surface_heights_m)
    if not isinstance(minimum_models, (int, np.integer)):
        raise ValueError("minimum_models must be an integer")
    if minimum_models < 2 or minimum_models > models.shape[1]:
        raise ValueError("minimum_models must be in [2, model count]")
    if not np.isfinite(maximum_model_spread_m) or maximum_model_spread_m < 0.0:
        raise ValueError("maximum_model_spread_m must be finite and non-negative")

    count, model_count = models.shape
    finite = np.isfinite(models)
    finite_count = np.sum(finite, axis=1).astype(np.uint8)
    ordered = np.sort(np.where(finite, models, np.inf), axis=1)

    best_count = np.zeros(count, dtype=np.uint8)
    best_spread = np.full(count, np.inf, dtype=np.float64)
    best_surface = np.full(count, np.nan, dtype=np.float64)
    smallest_required_spread = np.full(count, np.inf, dtype=np.float64)

    # The number of surface models is deliberately small (three in v11), so
    # looping over model windows keeps the point dimension vectorised.
    for width in range(minimum_models, model_count + 1):
        for start in range(0, model_count - width + 1):
            low = ordered[:, start]
            high = ordered[:, start + width - 1]
            valid = np.isfinite(high)
            spread = high - low
            if width == minimum_models:
                smallest_required_spread = np.minimum(
                    smallest_required_spread,
                    np.where(valid, spread, np.inf),
                )
            allowed = valid & (spread <= maximum_model_spread_m)
            better = allowed & (
                (width > best_count)
                | ((width == best_count) & (spread < best_spread))
            )
            if not np.any(better):
                continue
            window = ordered[:, start : start + width]
            surface = np.median(window, axis=1)
            best_count[better] = np.uint8(width)
            best_spread[better] = spread[better]
            best_surface[better] = surface[better]

    has_consensus = best_count >= minimum_models
    reported_spread = np.where(
        has_consensus,
        best_spread,
        np.where(np.isfinite(smallest_required_spread), smallest_required_spread, np.nan),
    )
    return ModelConsensusResult(
        surface_height_m=best_surface,
        agreeing_model_count=best_count,
        finite_model_count=finite_count,
        model_spread_m=reported_spread,
        has_consensus=has_consensus,
    )


def evaluate_surface_consensus(
    point_z_m: np.ndarray | Sequence[float] | float,
    surface_heights_m: np.ndarray | Sequence[Sequence[float]],
    *,
    residual_threshold_m: float,
    minimum_models: int = 2,
    maximum_model_spread_m: float = 0.05,
) -> SurfaceConsensusResult:
    """Evaluate a point against an independently agreed terrain surface."""

    models = _model_matrix(surface_heights_m)
    point_z = _vector(point_z_m, len(models), "point_z_m", np.float64)
    if not np.isfinite(residual_threshold_m) or residual_threshold_m < 0.0:
        raise ValueError("residual_threshold_m must be finite and non-negative")
    model_result = evaluate_model_consensus(
        models,
        minimum_models=minimum_models,
        maximum_model_spread_m=maximum_model_spread_m,
    )
    residual = point_z - model_result.surface_height_m
    above = (
        model_result.has_consensus
        & np.isfinite(point_z)
        & np.isfinite(residual)
        & (residual >= residual_threshold_m)
    )
    return SurfaceConsensusResult(
        surface_height_m=model_result.surface_height_m,
        residual_m=residual,
        agreeing_model_count=model_result.agreeing_model_count,
        finite_model_count=model_result.finite_model_count,
        model_spread_m=model_result.model_spread_m,
        has_consensus=model_result.has_consensus,
        above_threshold=above,
    )


def evaluate_1mm_transfer(
    point_z_m: np.ndarray | Sequence[float] | float,
    surface_heights_m: np.ndarray | Sequence[Sequence[float]],
    seed_distance_m: np.ndarray | Sequence[float] | float,
    *,
    core_connected: np.ndarray | Sequence[bool] | bool,
    scan_id: np.ndarray | Sequence[float] | float | None = None,
    thresholds: ObstructionThresholds = ObstructionThresholds(),
) -> FineTransferResult:
    """Gate 5 mm -> 1 mm transfer by distance and an independent residual.

    Proximity to a removed coarse point is seed evidence only.  The fine point
    must independently clear the grow-height residual over a two-model surface
    consensus.  This replaces the old unconditional 25 mm dilation.
    """

    models = _model_matrix(surface_heights_m)
    count = len(models)
    distance = _vector(seed_distance_m, count, "seed_distance_m", np.float64)
    connected = _vector(core_connected, count, "core_connected", bool)
    scans = (
        np.zeros(count, dtype=np.float64)
        if scan_id is None
        else _vector(scan_id, count, "scan_id", np.float64)
    )
    consensus = evaluate_surface_consensus(
        point_z_m,
        models,
        residual_threshold_m=thresholds.grow_height_m,
        minimum_models=thresholds.minimum_models,
        maximum_model_spread_m=thresholds.maximum_model_spread_m,
    )
    reason = np.zeros(count, dtype=np.uint32)
    close_seed = np.isfinite(distance) & (
        distance <= thresholds.fine_seed_distance_m
    )
    reason[~close_seed] |= np.uint32(
        ObstructionReason.FINE_SEED_MISSING_OR_TOO_FAR
    )
    reason[~consensus.above_threshold] |= np.uint32(
        ObstructionReason.FINE_INDEPENDENT_RESIDUAL_FAILED
    )
    reason[~connected] |= np.uint32(ObstructionReason.NOT_CORE_CONNECTED)

    scan9 = np.isfinite(scans) & np.isclose(scans, 9.0, atol=1.0e-4)
    scan9_consensus = (
        connected
        & consensus.has_consensus
        & consensus.above_threshold
        & (consensus.agreeing_model_count >= thresholds.minimum_models)
    )
    reason[scan9 & ~scan9_consensus] |= np.uint32(
        ObstructionReason.SCANID9_PROTECTED
    )
    eligible = close_seed & consensus.above_threshold & connected
    eligible &= ~(scan9 & ~scan9_consensus)
    return FineTransferResult(
        eligible=eligible,
        reason_mask=reason,
        consensus=consensus,
    )


def classify_obstruction_points(
    point_z_m: np.ndarray | Sequence[float] | float,
    surface_heights_m: np.ndarray | Sequence[Sequence[float]],
    *,
    core_connected: np.ndarray | Sequence[bool] | bool,
    candidate_mode: np.ndarray | Sequence[int] | int = CandidateMode.SEED,
    in_target: np.ndarray | Sequence[bool] | bool = True,
    touches_roi_boundary: np.ndarray | Sequence[bool] | bool = False,
    collar_point: np.ndarray | Sequence[bool] | bool = False,
    scan_id: np.ndarray | Sequence[float] | float | None = None,
    fine_seed_distance_m: np.ndarray | Sequence[float] | float | None = None,
    thresholds: ObstructionThresholds = ObstructionThresholds(),
) -> ObstructionClassification:
    """Classify local obstruction proposals without mutating point records.

    The caller owns ROI registration, terrain fitting, and graph construction.
    ``core_connected`` must mean the point belongs to a component intersecting
    a reviewed visual core.  A component touching the padded ROI boundary is
    retained for review.  ScanID 9 receives an explicit protection bit unless
    it is core-connected and passes the same two-of-three model consensus.

    Providing ``fine_seed_distance_m`` activates the stricter 1 mm transfer
    contract: at most 8 mm in 3D from a coarse seed plus an independent fine
    residual at or above ``grow_height_m``.
    """

    models = _model_matrix(surface_heights_m)
    count = len(models)
    point_z = _vector(point_z_m, count, "point_z_m", np.float64)
    connected = _vector(core_connected, count, "core_connected", bool)
    modes = _vector(candidate_mode, count, "candidate_mode", np.int16)
    valid_modes = np.isin(
        modes,
        [int(CandidateMode.NONE), int(CandidateMode.GROW), int(CandidateMode.SEED)],
    )
    if not np.all(valid_modes):
        raise ValueError("candidate_mode contains an unknown value")
    target = _vector(in_target, count, "in_target", bool)
    boundary = _vector(
        touches_roi_boundary, count, "touches_roi_boundary", bool
    )
    collar = _vector(collar_point, count, "collar_point", bool)
    scans = (
        np.zeros(count, dtype=np.float64)
        if scan_id is None
        else _vector(scan_id, count, "scan_id", np.float64)
    )

    model_result = evaluate_model_consensus(
        models,
        minimum_models=thresholds.minimum_models,
        maximum_model_spread_m=thresholds.maximum_model_spread_m,
    )
    residual = point_z - model_result.surface_height_m
    required = np.where(
        modes == int(CandidateMode.SEED),
        thresholds.seed_height_m,
        thresholds.grow_height_m,
    )
    active = target & (modes != int(CandidateMode.NONE))
    finite_point = np.isfinite(point_z)
    above_required = (
        model_result.has_consensus
        & finite_point
        & np.isfinite(residual)
        & (residual >= required)
    )
    ground_band = (
        model_result.has_consensus
        & np.isfinite(residual)
        & (residual <= thresholds.ground_stop_height_m)
    )

    reason = np.zeros(count, dtype=np.uint32)
    reason[~target] |= np.uint32(ObstructionReason.OUTSIDE_TARGET)
    reason[target & (modes == int(CandidateMode.NONE))] |= np.uint32(
        ObstructionReason.NOT_A_CANDIDATE
    )
    invalid = ~finite_point | (model_result.finite_model_count == 0)
    reason[active & invalid] |= np.uint32(ObstructionReason.INVALID_METRIC)
    insufficient = model_result.finite_model_count < thresholds.minimum_models
    reason[active & insufficient] |= np.uint32(
        ObstructionReason.INSUFFICIENT_MODELS
    )
    disagreement = (
        (model_result.finite_model_count >= thresholds.minimum_models)
        & ~model_result.has_consensus
    )
    reason[active & disagreement] |= np.uint32(ObstructionReason.MODEL_SPREAD)
    below = active & model_result.has_consensus & ~above_required
    reason[below & (modes == int(CandidateMode.SEED))] |= np.uint32(
        ObstructionReason.BELOW_SEED_HEIGHT
    )
    reason[below & (modes == int(CandidateMode.GROW))] |= np.uint32(
        ObstructionReason.BELOW_GROW_HEIGHT
    )
    reason[active & ground_band] |= np.uint32(
        ObstructionReason.GROUND_BAND_PROTECTED
    )
    reason[active & ~connected] |= np.uint32(
        ObstructionReason.NOT_CORE_CONNECTED
    )
    reason[active & boundary] |= np.uint32(
        ObstructionReason.ROI_BOUNDARY_PROTECTED
    )
    reason[active & collar] |= np.uint32(ObstructionReason.COLLAR_PROTECTED)

    transfer_eligible = np.ones(count, dtype=bool)
    if fine_seed_distance_m is not None:
        transfer = evaluate_1mm_transfer(
            point_z,
            models,
            fine_seed_distance_m,
            core_connected=connected,
            scan_id=scans,
            thresholds=thresholds,
        )
        transfer_eligible = transfer.eligible
        reason[active] |= transfer.reason_mask[active]

    scan9 = np.isfinite(scans) & np.isclose(scans, 9.0, atol=1.0e-4)
    scan9_consensus = (
        connected
        & model_result.has_consensus
        & (model_result.agreeing_model_count >= thresholds.minimum_models)
        & above_required
    )
    scan9_protected = active & scan9 & ~scan9_consensus
    reason[scan9_protected] |= np.uint32(ObstructionReason.SCANID9_PROTECTED)

    auto = (
        active
        & ~invalid
        & above_required
        & connected
        & ~boundary
        & ~collar
        & ~ground_band
        & transfer_eligible
        & ~scan9_protected
    )

    disposition = np.full(count, np.uint8(PointDisposition.KEEP), dtype=np.uint8)
    disposition[auto] = np.uint8(PointDisposition.AUTO_REMOVE)
    # A disagreement can still be visually obstruction-like, but it is never
    # positive evidence.  Keep it in REVIEW when two raw models put the point
    # above the low review threshold; hard ground/collar guards are rejected.
    raw_review_votes = np.sum(
        np.isfinite(models)
        & np.isfinite(point_z[:, None])
        & ((point_z[:, None] - models) >= thresholds.review_height_m),
        axis=1,
    )
    plausible = raw_review_votes >= thresholds.minimum_models
    hard_reject = invalid | ground_band | collar
    review = active & ~auto & plausible & ~hard_reject
    rejected = active & ~auto & ~review
    disposition[review] = np.uint8(PointDisposition.REVIEW)
    disposition[rejected] = np.uint8(PointDisposition.REJECTED)

    return ObstructionClassification(
        disposition=disposition,
        reason_mask=reason,
        consensus_count=model_result.agreeing_model_count,
        finite_model_count=model_result.finite_model_count,
        model_spread_m=model_result.model_spread_m,
        residual_m=residual,
        required_residual_m=required,
    )


@dataclass(frozen=True)
class TerrainPreservationThresholds:
    """Aggregate fail-closed validation thresholds."""

    ground_band_m: float = 0.020
    maximum_collar_removed_fraction: float = 0.001
    maximum_cell_ground_loss_fraction: float = 0.10

    def __post_init__(self) -> None:
        if not np.isfinite(self.ground_band_m) or self.ground_band_m < 0.0:
            raise ValueError("ground_band_m must be finite and non-negative")
        for name, value in (
            ("maximum_collar_removed_fraction", self.maximum_collar_removed_fraction),
            ("maximum_cell_ground_loss_fraction", self.maximum_cell_ground_loss_fraction),
        ):
            if not np.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must lie in [0, 1]")


@dataclass(frozen=True)
class TerrainPreservationResult:
    """Aggregate evidence that automatic removal retained terrain support."""

    passed: bool
    auto_removed_count: int
    invalid_auto_removed_count: int
    ground_band_removed_count: int
    collar_removed_count: int
    collar_point_count: int
    collar_removed_fraction: float
    maximum_cell_ground_loss_fraction: float
    failing_cell_ids: np.ndarray


def validate_terrain_preservation(
    disposition: np.ndarray | Sequence[int],
    consensus_residual_m: np.ndarray | Sequence[float],
    *,
    collar_mask: np.ndarray | Sequence[bool] | bool = False,
    terrain_cell_ids: np.ndarray | Sequence[int] | None = None,
    well_supported_ground_mask: np.ndarray | Sequence[bool] | None = None,
    thresholds: TerrainPreservationThresholds = TerrainPreservationThresholds(),
) -> TerrainPreservationResult:
    """Fail when an auto-removal enters the ground band or erodes a collar.

    Optional ``terrain_cell_ids`` and ``well_supported_ground_mask`` enforce
    the per-cell occupancy contract.  Cell IDs may be any integer labels; only
    points selected by the ground mask contribute to the denominator.
    """

    dispositions = np.asarray(disposition, dtype=np.uint8)
    if dispositions.ndim != 1:
        raise ValueError("disposition must be one-dimensional")
    count = len(dispositions)
    valid_values = np.isin(dispositions, [int(value) for value in PointDisposition])
    if not np.all(valid_values):
        raise ValueError("disposition contains an unknown value")
    residual = _vector(
        consensus_residual_m, count, "consensus_residual_m", np.float64
    )
    collar = _vector(collar_mask, count, "collar_mask", bool)
    removed = dispositions == int(PointDisposition.AUTO_REMOVE)
    invalid_removed = removed & ~np.isfinite(residual)
    ground_removed = removed & np.isfinite(residual) & (
        residual <= thresholds.ground_band_m
    )
    collar_removed = removed & collar
    collar_count = int(np.count_nonzero(collar))
    collar_removed_count = int(np.count_nonzero(collar_removed))
    collar_fraction = (
        collar_removed_count / collar_count if collar_count else 0.0
    )

    maximum_loss = 0.0
    failing_cells = np.empty(0, dtype=np.int64)
    if (terrain_cell_ids is None) != (well_supported_ground_mask is None):
        raise ValueError(
            "terrain_cell_ids and well_supported_ground_mask must be provided together"
        )
    if terrain_cell_ids is not None:
        cells = _vector(terrain_cell_ids, count, "terrain_cell_ids", np.int64)
        ground = _vector(
            well_supported_ground_mask,
            count,
            "well_supported_ground_mask",
            bool,
        )
        selected_cells = cells[ground]
        if len(selected_cells):
            unique, inverse = np.unique(selected_cells, return_inverse=True)
            totals = np.bincount(inverse, minlength=len(unique))
            losses = np.bincount(
                inverse,
                weights=removed[ground].astype(np.float64),
                minlength=len(unique),
            )
            fractions = losses / np.maximum(totals, 1)
            maximum_loss = float(np.max(fractions))
            failing_cells = unique[
                fractions > thresholds.maximum_cell_ground_loss_fraction
            ].astype(np.int64, copy=False)

    passed = (
        not np.any(invalid_removed)
        and not np.any(ground_removed)
        and collar_fraction <= thresholds.maximum_collar_removed_fraction
        and len(failing_cells) == 0
    )
    return TerrainPreservationResult(
        passed=bool(passed),
        auto_removed_count=int(np.count_nonzero(removed)),
        invalid_auto_removed_count=int(np.count_nonzero(invalid_removed)),
        ground_band_removed_count=int(np.count_nonzero(ground_removed)),
        collar_removed_count=collar_removed_count,
        collar_point_count=collar_count,
        collar_removed_fraction=float(collar_fraction),
        maximum_cell_ground_loss_fraction=maximum_loss,
        failing_cell_ids=failing_cells,
    )


@dataclass(frozen=True)
class SparsePointClassifications:
    """Sparse decisions keyed by immutable source-record indices."""

    original_indices: np.ndarray | Sequence[int]
    disposition: np.ndarray | Sequence[int]
    reason_mask: np.ndarray | Sequence[int]
    consensus_count: np.ndarray | Sequence[int] | None = None
    model_spread_m: np.ndarray | Sequence[float] | None = None
    residual_m: np.ndarray | Sequence[float] | None = None

    @classmethod
    def from_classification(
        cls,
        original_indices: np.ndarray | Sequence[int],
        classification: ObstructionClassification,
    ) -> "SparsePointClassifications":
        """Pair an ROI-local pure classification with source record IDs."""

        indices = np.asarray(original_indices, dtype=np.uint64)
        if indices.shape != classification.disposition.shape:
            raise ValueError("original_indices must match the classification")
        return cls(
            original_indices=indices,
            disposition=classification.disposition,
            reason_mask=classification.reason_mask,
            consensus_count=classification.consensus_count,
            model_spread_m=classification.model_spread_m,
            residual_m=classification.residual_m,
        )

    def __post_init__(self) -> None:
        indices = np.asarray(self.original_indices, dtype=np.uint64)
        disposition = np.asarray(self.disposition, dtype=np.uint8)
        reasons = np.asarray(self.reason_mask, dtype=np.uint32)
        if indices.ndim != 1:
            raise ValueError("original_indices must be one-dimensional")
        if disposition.shape != indices.shape or reasons.shape != indices.shape:
            raise ValueError("classification arrays must have matching shapes")
        if not np.all(np.isin(disposition, [int(value) for value in PointDisposition])):
            raise ValueError("disposition contains an unknown value")
        order = np.argsort(indices, kind="stable")
        indices = indices[order]
        disposition = disposition[order]
        reasons = reasons[order]
        if len(indices) > 1 and np.any(indices[1:] == indices[:-1]):
            raise ValueError("original_indices must be unique")
        object.__setattr__(self, "original_indices", indices)
        object.__setattr__(self, "disposition", disposition)
        object.__setattr__(self, "reason_mask", reasons)

        for name, values, dtype in (
            ("consensus_count", self.consensus_count, np.uint8),
            ("model_spread_m", self.model_spread_m, np.float32),
            ("residual_m", self.residual_m, np.float32),
        ):
            if values is None:
                continue
            array = np.asarray(values, dtype=dtype)
            if array.shape != indices.shape:
                raise ValueError(f"{name} must match original_indices")
            object.__setattr__(self, name, array[order])


_PLY_SCALAR_BYTES = {
    "char": 1,
    "int8": 1,
    "uchar": 1,
    "uint8": 1,
    "short": 2,
    "int16": 2,
    "ushort": 2,
    "uint16": 2,
    "int": 4,
    "int32": 4,
    "uint": 4,
    "uint32": 4,
    "float": 4,
    "float32": 4,
    "double": 8,
    "float64": 8,
    "int64": 8,
    "uint64": 8,
}


@dataclass(frozen=True)
class BinaryPlyLayout:
    path: Path
    header: bytes
    vertex_count: int
    record_stride: int

    @property
    def payload_offset(self) -> int:
        return len(self.header)


def inspect_binary_vertex_ply(path: Path | str) -> BinaryPlyLayout:
    """Inspect a fixed-stride, vertex-only binary little-endian PLY."""

    source = Path(path)
    header_parts: list[bytes] = []
    with source.open("rb") as handle:
        for _ in range(200_000):
            line = handle.readline()
            if not line:
                raise ValueError(f"{source}: missing end_header")
            header_parts.append(line)
            if line.rstrip(b"\r\n") == b"end_header":
                break
            if sum(map(len, header_parts)) > 16 * 1024 * 1024:
                raise ValueError(f"{source}: PLY header exceeds 16 MiB")
    header = b"".join(header_parts)
    try:
        lines = header.decode("ascii").splitlines()
    except UnicodeDecodeError as error:
        raise ValueError(f"{source}: PLY header is not ASCII") from error
    if not lines or lines[0].strip() != "ply":
        raise ValueError(f"{source}: not a PLY file")

    format_name = None
    current_element = None
    vertex_count = None
    stride = 0
    nonvertex_count = 0
    for line in lines[1:]:
        fields = line.split()
        if not fields or fields[0] == "comment" or fields[0] == "obj_info":
            continue
        if fields[0] == "format":
            if len(fields) < 2:
                raise ValueError(f"{source}: malformed format line")
            format_name = fields[1]
        elif fields[0] == "element":
            if len(fields) != 3:
                raise ValueError(f"{source}: malformed element line")
            current_element = fields[1]
            element_count = int(fields[2])
            if element_count < 0:
                raise ValueError(f"{source}: negative element count")
            if current_element == "vertex":
                if vertex_count is not None:
                    raise ValueError(f"{source}: duplicate vertex element")
                vertex_count = element_count
            else:
                nonvertex_count += element_count
        elif fields[0] == "property" and current_element == "vertex":
            if len(fields) >= 2 and fields[1] == "list":
                raise ValueError(f"{source}: list-valued vertex properties unsupported")
            if len(fields) != 3 or fields[1] not in _PLY_SCALAR_BYTES:
                raise ValueError(f"{source}: unsupported vertex property: {line}")
            stride += _PLY_SCALAR_BYTES[fields[1]]

    if format_name != "binary_little_endian":
        raise ValueError(f"{source}: only binary_little_endian PLY is supported")
    if vertex_count is None or stride <= 0:
        raise ValueError(f"{source}: missing fixed-stride vertex declaration")
    if nonvertex_count:
        raise ValueError(f"{source}: non-vertex payload elements are unsupported")
    expected_size = len(header) + vertex_count * stride
    if source.stat().st_size != expected_size:
        raise ValueError(
            f"{source}: payload size mismatch; expected {expected_size}, "
            f"found {source.stat().st_size}"
        )
    return BinaryPlyLayout(
        path=source,
        header=header,
        vertex_count=vertex_count,
        record_stride=stride,
    )


_VERTEX_LINE = re.compile(
    rb"^(element[ \t]+vertex[ \t]+)([0-9]+)([ \t]*)(\r?\n)$"
)


def _patched_vertex_header(header: bytes, count: int) -> bytes:
    if count < 0:
        raise ValueError("vertex count must be non-negative")
    output: list[bytes] = []
    patched = False
    for line in header.splitlines(keepends=True):
        match = _VERTEX_LINE.match(line)
        if match is None:
            output.append(line)
            continue
        if patched:
            raise ValueError("duplicate vertex element")
        available = len(match.group(2)) + len(match.group(3))
        digits = str(count).encode("ascii")
        if len(digits) > available:
            raise ValueError("new vertex count does not fit fixed header slot")
        output.append(
            match.group(1)
            + digits
            + b" " * (available - len(digits))
            + match.group(4)
        )
        patched = True
    if not patched:
        raise ValueError("missing vertex element")
    result = b"".join(output)
    if len(result) != len(header):
        raise AssertionError("patched PLY header changed byte length")
    return result


def sha256_path(path: Path | str, chunk_bytes: int = 32 * 1024 * 1024) -> str:
    """Return a streaming SHA-256 without loading a cloud into memory."""

    if chunk_bytes <= 0:
        raise ValueError("chunk_bytes must be positive")
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while True:
            block = handle.read(chunk_bytes)
            if not block:
                return digest.hexdigest()
            digest.update(block)


@dataclass(frozen=True)
class CandidateArchiveResult:
    """Published immutable paths and hashes for an obstruction candidate."""

    bundle_dir: Path
    candidate_path: Path
    removed_records_path: Path
    removed_indices_path: Path
    classifications_path: Path
    manifest_path: Path
    source_sha256: str
    candidate_sha256: str
    removed_records_sha256: str
    source_count: int
    candidate_count: int
    removed_count: int


def _save_npy(path: Path, values: np.ndarray) -> None:
    with path.open("wb") as handle:
        np.save(handle, values, allow_pickle=False)


def _source_identity(path: Path) -> tuple[int, int, int, int]:
    stat = path.stat()
    return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)


def write_candidate_archive(
    source_path: Path | str,
    bundle_dir: Path | str,
    classifications: SparsePointClassifications,
    *,
    expected_source_sha256: str,
    candidate_filename: str | None = None,
    chunk_records: int = 1_000_000,
) -> CandidateArchiveResult:
    """Stream a candidate PLY and exact removal archive into a new bundle.

    ``expected_source_sha256`` is mandatory.  All outputs are staged in a
    private sibling directory and published by one directory rename only after
    the source hash and identity are rechecked.  Existing bundle directories
    are never overwritten.  The candidate contains KEEP, REVIEW, and REJECTED
    source records in original order; only AUTO_REMOVE rows are omitted.
    """

    source = Path(source_path)
    destination = Path(bundle_dir)
    if not re.fullmatch(r"[0-9a-fA-F]{64}", expected_source_sha256):
        raise ValueError("expected_source_sha256 must be a 64-character hex digest")
    expected_hash = expected_source_sha256.lower()
    if not isinstance(chunk_records, (int, np.integer)) or chunk_records <= 0:
        raise ValueError("chunk_records must be a positive integer")
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite candidate bundle {destination}")
    destination_parent = destination.parent
    destination_parent.mkdir(parents=True, exist_ok=True)
    source_resolved = source.resolve(strict=True)
    destination_resolved = destination.resolve(strict=False)
    if destination_resolved == source_resolved:
        raise ValueError("candidate bundle cannot be the source PLY")

    layout = inspect_binary_vertex_ply(source)
    indices = np.asarray(classifications.original_indices, dtype=np.uint64)
    dispositions = np.asarray(classifications.disposition, dtype=np.uint8)
    if len(indices) and int(indices[-1]) >= layout.vertex_count:
        raise ValueError("classification index exceeds source vertex count")
    auto = dispositions == int(PointDisposition.AUTO_REMOVE)
    removed_indices = indices[auto].astype(np.uint64, copy=False)
    removed_count = len(removed_indices)
    candidate_count = layout.vertex_count - removed_count

    candidate_name = (
        f"{source.stem}.candidate.ply"
        if candidate_filename is None
        else candidate_filename
    )
    if Path(candidate_name).name != candidate_name or not candidate_name:
        raise ValueError("candidate_filename must be one file name")
    if candidate_name == source.name:
        raise ValueError("candidate filename must be visibly distinct from source")

    stage = Path(
        tempfile.mkdtemp(
            prefix=f".{destination.name}.staging-",
            dir=destination_parent,
        )
    )
    identity_before = _source_identity(source)
    try:
        candidate_path = stage / candidate_name
        removed_path = stage / f"{source.stem}.removed.ply"
        removed_indices_path = stage / f"{source.stem}.removed_indices.npy"
        classifications_path = stage / f"{source.stem}.classifications.npz"
        header_path = stage / f"{source.stem}.source_header.bin"
        manifest_path = stage / "manifest.json"

        candidate_header = _patched_vertex_header(layout.header, candidate_count)
        removed_header = _patched_vertex_header(layout.header, removed_count)
        source_digest = hashlib.sha256(layout.header)
        candidate_digest = hashlib.sha256(candidate_header)
        removed_digest = hashlib.sha256(removed_header)

        with (
            source.open("rb") as source_handle,
            candidate_path.open("wb") as candidate_handle,
            removed_path.open("wb") as removed_handle,
        ):
            observed_header = source_handle.read(layout.payload_offset)
            if observed_header != layout.header:
                raise RuntimeError("source header changed after inspection")
            candidate_handle.write(candidate_header)
            removed_handle.write(removed_header)

            removed_cursor = 0
            for start in range(0, layout.vertex_count, int(chunk_records)):
                count = min(int(chunk_records), layout.vertex_count - start)
                byte_count = count * layout.record_stride
                payload = source_handle.read(byte_count)
                if len(payload) != byte_count:
                    raise RuntimeError("source payload ended during candidate stream")
                source_digest.update(payload)
                records = np.frombuffer(
                    payload,
                    dtype=np.dtype((np.void, layout.record_stride)),
                    count=count,
                )
                remove_mask = np.zeros(count, dtype=bool)
                end_index = start + count
                next_cursor = int(
                    np.searchsorted(removed_indices, end_index, side="left")
                )
                if next_cursor > removed_cursor:
                    local = removed_indices[removed_cursor:next_cursor].astype(
                        np.int64
                    ) - start
                    remove_mask[local] = True
                kept_payload = records[~remove_mask].tobytes()
                removed_payload = records[remove_mask].tobytes()
                candidate_handle.write(kept_payload)
                removed_handle.write(removed_payload)
                candidate_digest.update(kept_payload)
                removed_digest.update(removed_payload)
                removed_cursor = next_cursor
            if source_handle.read(1):
                raise RuntimeError("unexpected trailing source payload")
            if removed_cursor != removed_count:
                raise AssertionError("not every removal index was streamed")

        observed_hash = source_digest.hexdigest()
        if observed_hash != expected_hash:
            raise RuntimeError(
                "source hash lock failed: "
                f"expected {expected_hash}, observed {observed_hash}"
            )
        if _source_identity(source) != identity_before:
            raise RuntimeError("source identity changed during candidate stream")

        _save_npy(removed_indices_path, removed_indices)
        classification_payload = {
            "original_indices": indices,
            "disposition": dispositions,
            "reason_mask": np.asarray(classifications.reason_mask, dtype=np.uint32),
        }
        for name in ("consensus_count", "model_spread_m", "residual_m"):
            value = getattr(classifications, name)
            if value is not None:
                classification_payload[name] = np.asarray(value)
        with classifications_path.open("wb") as handle:
            np.savez(handle, **classification_payload)
        header_path.write_bytes(layout.header)

        disposition_counts = {
            value.name: int(np.count_nonzero(dispositions == int(value)))
            for value in PointDisposition
        }
        reason_counts = {
            value.name: int(
                np.count_nonzero(
                    np.asarray(classifications.reason_mask, dtype=np.uint32)
                    & np.uint32(value)
                )
            )
            for value in ObstructionReason
            if value != ObstructionReason.NONE
        }
        manifest = {
            "schema": 1,
            "operation": "candidate-only-obstruction-removal",
            "source": {
                "path": str(source_resolved),
                "sha256": observed_hash,
                "points": layout.vertex_count,
                "record_stride": layout.record_stride,
            },
            "candidate": {
                "path": candidate_name,
                "sha256": candidate_digest.hexdigest(),
                "points": candidate_count,
                "retained_dispositions": [
                    PointDisposition.KEEP.name,
                    PointDisposition.REVIEW.name,
                    PointDisposition.REJECTED.name,
                ],
            },
            "removed": {
                "records_path": removed_path.name,
                "records_sha256": removed_digest.hexdigest(),
                "indices_path": removed_indices_path.name,
                "indices_dtype": "uint64",
                "points": removed_count,
                "records_are_exact_source_bytes": True,
            },
            "classifications": {
                "path": classifications_path.name,
                "points": len(indices),
                "disposition_counts": disposition_counts,
                "reason_counts": reason_counts,
                "review_and_rejected_retained": True,
            },
            "source_header": header_path.name,
            "canonical_writes": False,
        }
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        os.replace(stage, destination)
        return CandidateArchiveResult(
            bundle_dir=destination,
            candidate_path=destination / candidate_name,
            removed_records_path=destination / removed_path.name,
            removed_indices_path=destination / removed_indices_path.name,
            classifications_path=destination / classifications_path.name,
            manifest_path=destination / manifest_path.name,
            source_sha256=observed_hash,
            candidate_sha256=candidate_digest.hexdigest(),
            removed_records_sha256=removed_digest.hexdigest(),
            source_count=layout.vertex_count,
            candidate_count=candidate_count,
            removed_count=removed_count,
        )
    except BaseException:
        if stage.exists():
            shutil.rmtree(stage)
        raise


__all__ = [
    "BinaryPlyLayout",
    "CandidateArchiveResult",
    "CandidateMode",
    "FineTransferResult",
    "ModelConsensusResult",
    "ObstructionClassification",
    "ObstructionReason",
    "ObstructionThresholds",
    "PointDisposition",
    "SparsePointClassifications",
    "SurfaceConsensusResult",
    "TerrainPreservationResult",
    "TerrainPreservationThresholds",
    "classify_obstruction_points",
    "evaluate_1mm_transfer",
    "evaluate_model_consensus",
    "evaluate_surface_consensus",
    "inspect_binary_vertex_ply",
    "sha256_path",
    "validate_terrain_preservation",
    "write_candidate_archive",
]

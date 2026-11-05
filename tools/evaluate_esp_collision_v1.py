"""Read-only saved ESP evaluation with explicitly hypothetical opponents."""
import argparse
import csv
import hashlib
import json
import math
from numbers import Integral
from pathlib import Path

import numpy as np

from f1tenth_benchmarks.research.overtaking.playback import PlaybackSession
from f1tenth_benchmarks.research.overtaking.track_data import TrackData
from f1tenth_benchmarks.research.overtaking.prediction_v1 import predict_polyline
from f1tenth_benchmarks.research.overtaking.prediction_collision_adapter_v1 import evaluate_plan_candidates

FIELDS = ['candidate_id', 'native_sampled_valid', 'native_reason', 'prediction_status',
          'prediction_complete', 'collision_assessment', 'evaluation_complete', 'outcome',
          'checked_range_s', 'unchecked_range_s', 'minimum_center_distance_m',
          'minimum_clearance_m', 'first_future_contact_grid_t_s', 'reason']


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_saved_plan(directory, plan_id, track):
    """Validate recorded geometry provenance and an exact, unique source join."""
    if isinstance(plan_id, bool) or not isinstance(plan_id, Integral):
        raise ValueError('invalid_plan_id')
    reader = PlaybackSession(directory, track)
    if reader.metadata.get('map') != 'esp':
        raise ValueError('session_not_explicitly_esp')
    matches = [i for i, row in enumerate(reader.plan_headers) if row['plan_id'] == plan_id]
    if len(matches) != 1:
        raise ValueError('plan_id_missing_or_duplicate')
    plan = reader.record('plans', matches[0])
    state = reader.record('states', plan['source_control_step'])
    if (state['control_step'] != plan['source_control_step'] or
            state['sim_time_s'] != plan['source_sim_time_s'] or state['pose'] != plan['source_pose']):
        raise ValueError('plan_source_state_mismatch')
    if state.get('speed_mps') is None:
        raise ValueError('missing_source_speed')
    for key in ('geometry_id', 'frame_id'):
        expected = reader.metadata[key]
        if plan.get(key, expected) != expected or state.get(key, expected) != expected:
            raise ValueError('source_metadata_mismatch')
    return reader, plan, state


def centerline_reference(track, start_index, distance_m):
    """Unroll a finite CSV-centerline prefix, possibly across one closed seam.

    Do not create repeated complete laps: that makes nearest-start projection
    ambiguous under Prediction V1. No generator or spline optimizer is called.
    """
    points = np.asarray(track.centerline[:, :2], float)
    if np.array_equal(points[-1], points[0]):
        points = points[:-1]
    if isinstance(start_index, bool) or not isinstance(start_index, Integral) or not 0 <= start_index < len(points):
        raise ValueError('invalid_centerline_start_index')
    closed = track.geometry.curve.closed
    if not math.isfinite(distance_m) or distance_m <= 0:
        raise ValueError('invalid_reference_distance')
    chain = np.vstack((points[start_index:], points[:start_index+1])) if closed else points[start_index:]
    lengths = np.linalg.norm(np.diff(chain, axis=0), axis=1)
    total = float(lengths.sum())
    if (closed and distance_m >= total) or (not closed and distance_m > total):
        raise ValueError('reference_exceeds_unambiguous_finite_prefix')
    result = [chain[0].tolist()]
    remaining = distance_m
    for a, b, length in zip(chain[:-1], chain[1:], lengths):
        if length == 0:
            continue
        if remaining <= length:
            result.append((a+(b-a)*remaining/length).tolist())
            break
        result.append(b.tolist())
        remaining -= float(length)
    return result


def evaluate_saved_session(directory, plan_id, scenario, *, track, output_dir):
    """scenario is explicit JSON; output directory must not already exist."""
    source_directory = Path(directory)
    before = {name: sha256(source_directory/name) for name in
              ('metadata.json', 'states.jsonl', 'plans.jsonl', 'summary.json')}
    reader, plan, state = load_saved_plan(directory, plan_id, track)
    if not isinstance(scenario, dict):
        raise ValueError('invalid_scenario')
    for key in ('name', 'session_classification', 'dt_s', 'steps', 'ego_body', 'lead_body', 'opponent'):
        if key not in scenario:
            raise ValueError('missing_scenario_' + key)
    opponent = scenario['opponent']
    if not isinstance(opponent, dict):
        raise ValueError('invalid_opponent')
    for key in ('path_xy', 'start_xy', 'speed_mps', 'frame_id', 'geometry_id', 'start_time_s', 'provenance'):
        if key not in opponent:
            raise ValueError('missing_opponent_' + key)
    if not isinstance(opponent['provenance'], dict):
        raise ValueError('invalid_opponent_provenance')
    if opponent['provenance'].get('kind') not in ('synthetic', 'assumed_reference_path'):
        raise ValueError('opponent_must_be_explicitly_hypothetical')
    prediction = predict_polyline(opponent['path_xy'], opponent['start_xy'], opponent['speed_mps'],
        scenario['dt_s'], scenario['steps'], geometry_id=opponent['geometry_id'], frame_id=opponent['frame_id'])
    result = evaluate_plan_candidates(plan, source_state=state, session_metadata=reader.metadata,
        opponent_prediction=prediction, opponent_start_time_s=opponent['start_time_s'],
        opponent_provenance=opponent['provenance'], ego_body=scenario['ego_body'], lead_body=scenario['lead_body'],
        dt_s=scenario['dt_s'], steps=scenario['steps'])
    payload = dict(schema_version=1, label='ESP geometry + synthetic opponent offline evaluation',
        source=dict(session_directory=str(reader.root.resolve()), classification=scenario['session_classification'],
            file_sha256=before, recorded_program=reader.metadata.get('program'),
            track_provenance=reader.metadata['track'], plan_id=plan_id,
            source_control_step=plan['source_control_step'], source_sim_time_s=plan['source_sim_time_s'],
            source_pose=plan['source_pose'], ego_speed_mps=state['speed_mps'],
            geometry_id=reader.metadata['geometry_id'], frame_id=reader.metadata['frame_id']),
        scenario=scenario, timing=dict(dt_s=scenario['dt_s'], steps=scenario['steps'],
            requested_horizon_s=prediction.get('requested_horizon_s')),
        result=result, inspection_only=True, planning_allowed=False,
        limitations=['Hypothetical opponent; not an observed ESP two-vehicle run.',
                     'Discrete samples only; no continuous or dynamic safety approval.'])
    # Validate finite serialization before creating an output directory.
    encoded = json.dumps(payload, indent=2, allow_nan=False)
    if before != {name: sha256(reader.root/name) for name in before}:
        raise RuntimeError('source_changed_during_read')
    output = Path(output_dir)
    source_root = reader.root.resolve()
    if output.resolve() == source_root or source_root in output.resolve().parents:
        raise ValueError('output_must_be_outside_source_session')
    output.mkdir(parents=True, exist_ok=False)
    (output/'evaluation.json').write_text(encoded, encoding='utf-8')
    with (output/'summary.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for row in result['paths']:
            writer.writerow({k: json.dumps(row.get(k), ensure_ascii=False, allow_nan=False) for k in FIELDS})
    lines = ['# ESP geometry + synthetic opponent offline evaluation', '',
             'Not an observed two-vehicle session; inspection_only=true, planning_allowed=false.', '',
             '| C | Native | Outcome | Complete | Min center [m] | First future contact [s] | Reason |',
             '|---|---|---|---|---|---|---|']
    for row in result['paths']:
        values = [row.get(k) for k in ('candidate_id', 'native_sampled_valid', 'outcome',
                  'evaluation_complete', 'minimum_center_distance_m', 'first_future_contact_grid_t_s', 'reason')]
        lines.append('| ' + ' | '.join(json.dumps(v, ensure_ascii=False).replace('|', '\\|') for v in values) + ' |')
    (output/'summary.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    return payload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-dir', required=True)
    parser.add_argument('--plan-id', type=int, required=True)
    parser.add_argument('--scenario-json', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--geometry-dir', default='maps/frenet/esp/geometry')
    parser.add_argument('--map-yaml', default='maps/esp.yaml')
    args = parser.parse_args()
    try:
        track = TrackData(args.geometry_dir, args.map_yaml)
        scenario = json.loads(Path(args.scenario_json).read_text(encoding='utf-8'))
        payload = evaluate_saved_session(args.session_dir, args.plan_id, scenario, track=track, output_dir=args.output_dir)
    except (ValueError, KeyError, TypeError, OSError, AssertionError) as exc:
        parser.error(str(exc))
    print(json.dumps(dict(batch_status=payload['result']['batch_status'],
                         outcomes=[r['outcome'] for r in payload['result']['paths']]), allow_nan=False))


if __name__ == '__main__':
    main()

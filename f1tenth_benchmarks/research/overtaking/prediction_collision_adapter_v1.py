"""Offline five-candidate inspection adapter; no ranking or control authorization."""
import math
from numbers import Integral, Real

from .prediction_v1 import predict_candidate
from .collision_v1 import evaluate_collision


METHOD = 'offline_prediction_collision_adapter_v1'


def _json_value(value):
    """Return detached JSON primitives; unavailable/invalid diagnostics stay null."""
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    return None


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError('invalid_' + name)
    return float(value)


def _identifier(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('missing_' + name)
    return value


def evaluate_plan_candidates(plan, *, source_state=None, session_metadata=None,
                             opponent_prediction=None, opponent_start_time_s=None,
                             opponent_provenance=None, ego_body=None, lead_body=None,
                             dt_s=None, steps=None):
    """Inspect existing paths in their input order, without regenerating them.

    source_state is the matching states.jsonl snapshot (valid, pose, speed_mps,
    sim_time_s, control_step). session_metadata supplies explicit frame/geometry.
    The external opponent prediction must be from prediction_v1; its mandatory
    provenance has kind and description strings (e.g. assumed_reference_path).
    Origins are assertions in one common clock, never inferred from points.t or
    available_sim_time_s. Different geometry IDs for the opponent are allowed
    only in the same frame; neither road geometry nor transforms are evaluated.
    A malformed five-slot envelope rejects every supplied slot without inventing
    missing candidates. Missing paths yields no rows and a rejected batch.
    """
    paths = plan.get('paths') if isinstance(plan, dict) else None
    paths = paths if isinstance(paths, list) else []
    batch = dict(schema_version=1, method=METHOD, batch_status='rejected', reason=None,
                 plan_id=plan.get('plan_id') if isinstance(plan, dict) else None,
                 expected_candidate_count=5, received_candidate_count=len(paths),
                 source_sim_time_s=None, source_control_step=None, source_pose=None,
                 ego_speed_mps=None, geometry_id=None, frame_id=None,
                 dt_s=None, steps=None, opponent_start_time_s=None,
                 opponent_provenance=opponent_provenance, opponent_prediction=opponent_prediction,
                 ego_body=ego_body, lead_body=lead_body, paths=[],
                 inspection_only=True, planning_allowed=False, continuous_sweep_checked=False,
                 limitations=['Discrete timestamps only; no continuous or dynamic safety claim.',
                              'External opponent trajectory and time origins are caller assertions.',
                              'XY must refer to each vehicle bounding-circle center.'])
    for index, candidate in enumerate(paths):
        candidate = candidate if isinstance(candidate, dict) else {}
        batch['paths'].append(dict(input_index=index, candidate_id=candidate.get('candidate_id'),
            native_sampled_valid=candidate.get('sampled_valid'), native_reason=candidate.get('reason'),
            path_type=candidate.get('path_type', 'state_matched_trajectory'),
            path_type_origin='explicit' if 'path_type' in candidate else 'legacy_default',
            prediction_status='not_evaluated', prediction_reason=None, prediction_complete=False,
            collision_status='not_evaluated', collision_reason=None, collision_assessment='unknown',
            evaluation_complete=False, checked_range_s=None, unchecked_range_s=None,
            current_contact=None, any_contact_in_checked_samples=None, first_contact_grid_t_s=None,
            first_future_contact_grid_t_s=None, minimum_center_distance_m=None,
            minimum_center_distance_t_s=None, minimum_clearance_m=None, minimum_clearance_t_s=None,
            source_sim_time_s=None, geometry_id=None, frame_id=None,
            outcome='rejected', reason=None, prediction=None, collision=None,
            inspection_only=True, planning_allowed=False))
    try:
        if not isinstance(plan, dict) or not isinstance(plan.get('paths'), list):
            raise ValueError('missing_plan_paths')
        ids = [p.get('candidate_id') if isinstance(p, dict) else None for p in paths]
        if len(ids) != 5 or any(isinstance(i, bool) or not isinstance(i, Integral) for i in ids):
            raise ValueError('invalid_five_candidate_envelope')
        if set(ids) != {1, 2, 3, 4, 5}:
            raise ValueError('duplicate_or_unexpected_candidate_ids')
        if plan.get('status') != 'completed':
            raise ValueError('plan_not_completed')
        if not isinstance(source_state, dict) or not isinstance(session_metadata, dict):
            raise ValueError('missing_source_state_or_metadata')
        geometry = _identifier(plan.get('geometry_id'), 'geometry_id')
        frame = _identifier(session_metadata.get('frame_id'), 'frame_id')
        if session_metadata.get('geometry_id') != geometry:
            raise ValueError('session_geometry_mismatch')
        for obj in (plan, source_state, plan.get('result', {})):
            if isinstance(obj, dict):
                if obj.get('frame_id', frame) != frame or obj.get('geometry_id', geometry) != geometry:
                    raise ValueError('source_metadata_mismatch')
        origin = _number(plan.get('source_sim_time_s'), 'source_sim_time_s')
        speed = _number(source_state.get('speed_mps'), 'ego_speed_mps')
        if speed < 0:
            raise ValueError('negative_ego_speed')
        if source_state.get('valid') is not True:
            raise ValueError('invalid_source_state')
        if _number(source_state.get('sim_time_s'), 'state_time') != origin:
            raise ValueError('source_state_time_mismatch')
        control_step = plan.get('source_control_step')
        if isinstance(control_step, bool) or not isinstance(control_step, Integral) or control_step < 0:
            raise ValueError('invalid_source_control_step')
        if type(source_state.get('control_step')) is not int or source_state['control_step'] != control_step:
            raise ValueError('source_control_step_mismatch')
        pose = plan.get('source_pose')
        state_pose = source_state.get('pose')
        if not isinstance(pose, (list, tuple)) or len(pose) != 3 or not isinstance(state_pose, (list, tuple)) or len(state_pose) != 3:
            raise ValueError('invalid_source_pose')
        pose = [_number(v, 'source_pose') for v in pose]
        if pose != [_number(v, 'state_pose') for v in state_pose]:
            raise ValueError('source_pose_mismatch')
        dt = _number(dt_s, 'dt_s')
        if dt <= 0 or isinstance(steps, bool) or not isinstance(steps, Integral) or not 0 <= steps <= 100000:
            raise ValueError('invalid_requested_grid')
        if not isinstance(opponent_provenance, dict):
            raise ValueError('missing_opponent_provenance')
        for field in ('kind', 'description'):
            _identifier(opponent_provenance.get(field), 'opponent_' + field)
        other_origin = _number(opponent_start_time_s, 'opponent_start_time_s')
        if other_origin != origin:
            raise ValueError('prediction_start_time_mismatch')
        batch.update(source_sim_time_s=origin, source_control_step=int(control_step), source_pose=pose,
                     ego_speed_mps=speed, geometry_id=geometry, frame_id=frame,
                     dt_s=dt, steps=int(steps), opponent_start_time_s=other_origin)
        # Public Collision V1 validates the shared prediction and both bodies once.
        probe = evaluate_collision(opponent_prediction, opponent_prediction,
            common_start_time_s=origin, ego_start_time_s=origin, lead_start_time_s=other_origin,
            ego_body=ego_body, lead_body=lead_body)
        if probe['status'] == 'rejected':
            raise ValueError('invalid_shared_inputs:' + str(probe['reason']))
        _identifier(opponent_prediction.get('geometry_id'), 'opponent_geometry_id')
        if opponent_prediction['frame_id'] != frame:
            raise ValueError('opponent_frame_mismatch')
        parameters = opponent_prediction['parameters']
        if parameters['dt_s'] != dt or parameters['steps'] != steps:
            raise ValueError('requested_time_grid_mismatch')
        batch['batch_status'] = 'evaluated'
    except (ValueError, KeyError, TypeError, OverflowError) as exc:
        batch['reason'] = str(exc)
        for row in batch['paths']:
            row.update(reason=str(exc), collision_reason=str(exc),
                       source_sim_time_s=batch['source_sim_time_s'],
                       geometry_id=batch['geometry_id'], frame_id=batch['frame_id'])
        return _json_value(batch)

    context = dict(geometry_id=geometry, frame_id=frame)
    for candidate, row in zip(paths, batch['paths']):
        row.update(source_sim_time_s=origin, geometry_id=geometry, frame_id=frame)
        prediction = predict_candidate(candidate, pose[:2], speed, dt, int(steps), context=context)
        row.update(prediction=prediction, prediction_status=prediction['status'],
                   prediction_reason=prediction.get('reason'),
                   prediction_complete=prediction['prediction_complete'])
        if prediction['status'] == 'rejected':
            row.update(reason=prediction['reason'])
            if prediction['reason'] == 'reference_path_not_predictable_as_ego_trajectory':
                row['outcome'] = 'reference_only'
            continue
        collision = evaluate_collision(prediction, opponent_prediction,
            common_start_time_s=origin, ego_start_time_s=origin, lead_start_time_s=other_origin,
            ego_body=ego_body, lead_body=lead_body)
        row.update(collision=collision, collision_status=collision['status'],
                   collision_reason=collision.get('reason'), collision_assessment=collision['assessment'],
                   outcome='rejected' if collision['status'] == 'rejected' else collision['assessment'],
                   reason=collision.get('reason'))
        for field in ('evaluation_complete', 'checked_range_s', 'unchecked_range_s',
                      'current_contact', 'any_contact_in_checked_samples', 'first_contact_grid_t_s',
                      'first_future_contact_grid_t_s', 'minimum_center_distance_m',
                      'minimum_center_distance_t_s', 'minimum_clearance_m', 'minimum_clearance_t_s'):
            row[field] = collision.get(field)
    return _json_value(batch)

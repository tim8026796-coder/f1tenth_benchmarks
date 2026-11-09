"""Lightweight collision features and comparability; no score, ranking or control."""
import hashlib
import json
import math
from numbers import Integral, Real

METHOD = 'five_candidate_cost_input_v1'
OUTCOMES = {'no_contact_at_samples', 'contact_detected', 'unknown', 'rejected', 'reference_only'}
METRICS = ('minimum_center_distance_m', 'minimum_center_distance_t_s',
           'minimum_clearance_m', 'minimum_clearance_t_s',
           'first_contact_grid_t_s', 'first_future_contact_grid_t_s')
COPY_FIELDS = ('prediction_complete', 'evaluation_complete', 'checked_range_s',
               'unchecked_range_s', 'current_contact') + METRICS


def _json(value):
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [_json(x) for x in value]
    if isinstance(value, dict):
        return {str(k): _json(v) for k, v in value.items()}
    return None


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError('invalid_' + name)
    return float(value)


def _id(value):
    return isinstance(value, Integral) and not isinstance(value, bool)


def _slots(rows, name):
    if not isinstance(rows, list) or len(rows) != 5 or not all(isinstance(r, dict) for r in rows):
        raise ValueError('invalid_' + name + '_slots')
    ids = [r.get('candidate_id') for r in rows]
    if any(not _id(i) for i in ids) or set(ids) != {1, 2, 3, 4, 5}:
        raise ValueError('invalid_' + name + '_ids')
    return {r['candidate_id']: r for r in rows}


def _fingerprint(value):
    # Fingerprints describe supplied artifacts; Adapter did not emit a plan hash.
    data = json.dumps(_json(value), sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(data.encode()).hexdigest()


def _row_checks(row, horizon):
    for flag in ('prediction_complete', 'evaluation_complete'):
        if type(row.get(flag)) is not bool:
            raise ValueError('missing_or_invalid_' + flag)
    if row.get('prediction_status') not in ('complete', 'path_exhausted', 'rejected', 'not_evaluated'):
        raise ValueError('invalid_prediction_status')
    if row.get('collision_status') not in ('complete', 'incomplete', 'rejected', 'not_evaluated'):
        raise ValueError('invalid_collision_status')
    if row['prediction_complete'] != (row['prediction_status'] == 'complete'):
        raise ValueError('inconsistent_prediction_complete')
    if row['evaluation_complete'] and not row['prediction_complete']:
        raise ValueError('complete_collision_requires_complete_prediction')
    if row['evaluation_complete'] != (row['collision_status'] == 'complete'):
        raise ValueError('inconsistent_evaluation_complete')
    checked = row.get('checked_range_s')
    if checked is not None:
        if not isinstance(checked, list) or len(checked) != 2:
            raise ValueError('invalid_checked_range')
        low, high = [_number(v, 'checked_range') for v in checked]
        if low != 0 or not low <= high <= horizon:
            raise ValueError('invalid_checked_range')
        if row['evaluation_complete']:
            if high != horizon or row.get('unchecked_range_s') is not None:
                raise ValueError('inconsistent_complete_range')
        elif row['collision_status'] == 'incomplete':
            unknown = row.get('unchecked_range_s')
            if not isinstance(unknown, dict) or unknown.get('after_t_s') != high or unknown.get('through_t_s') != horizon or high >= horizon:
                raise ValueError('inconsistent_unchecked_range')
    elif row['collision_status'] in ('complete', 'incomplete'):
        raise ValueError('missing_checked_range')
    for field in METRICS:
        if row.get(field) is None:
            continue
        v = _number(row[field], field)
        if v < 0:
            raise ValueError('negative_' + field)
        if checked is None or (field.endswith('_t_s') and not checked[0] <= v <= checked[1]):
            raise ValueError('metric_outside_checked_range')
    if row.get('first_future_contact_grid_t_s') is not None and row['first_future_contact_grid_t_s'] <= 0:
        raise ValueError('future_contact_must_be_positive')
    if row.get('current_contact') not in (None, True, False) or (row.get('current_contact') is not None and type(row['current_contact']) is not bool):
        raise ValueError('invalid_current_contact')
    outcome = row.get('outcome')
    if outcome not in OUTCOMES:
        raise ValueError('invalid_outcome')
    if outcome == 'contact_detected' and row['collision_status'] not in ('complete','incomplete'):
        raise ValueError('contact_requires_checked_samples')
    if outcome == 'unknown' and row['collision_status'] != 'incomplete':
        raise ValueError('unknown_requires_incomplete_evaluation')
    if outcome == 'no_contact_at_samples' and not (row['prediction_complete'] and row['evaluation_complete']):
        raise ValueError('no_contact_requires_complete_horizon')
    if outcome == 'unknown' and row['evaluation_complete']:
        raise ValueError('unknown_cannot_be_complete')
    if outcome in ('rejected', 'reference_only') and row['evaluation_complete']:
        raise ValueError('excluded_path_cannot_be_complete')
    if outcome in ('contact_detected', 'no_contact_at_samples', 'unknown') and row.get('collision_assessment') != outcome:
        raise ValueError('collision_assessment_mismatch')
    if outcome != 'contact_detected' and (row.get('current_contact') is True or row.get('first_contact_grid_t_s') is not None or row.get('first_future_contact_grid_t_s') is not None or row.get('any_contact_in_checked_samples') is True):
        raise ValueError('contradictory_contact_evidence')
    if row.get('current_contact') is True and (outcome != 'contact_detected' or row.get('first_contact_grid_t_s') != 0):
        raise ValueError('inconsistent_current_contact')
    if row.get('any_contact_in_checked_samples') is False and outcome == 'contact_detected':
        raise ValueError('contradictory_contact_evidence')
    center, clearance = row.get('minimum_center_distance_m'), row.get('minimum_clearance_m')
    if center is not None and clearance is not None and clearance > center:
        raise ValueError('clearance_exceeds_center_distance')


def build_cost_inputs(plan, adapter_result, *, source_state=None, session_metadata=None):
    """Join by candidate ID, preserve plan order, and classify available features.

    No Prediction/Collision call or geometry recomputation. Metadata matching is
    not cryptographic proof that Adapter used the same candidate point content.
    Session metadata is required to independently establish geometry/frame;
    source_state, when provided, must be the same snapshot. Missing values stay
    null. Invalid batches retain actual plan slots and report contract errors.
    """
    candidates = plan.get('paths', []) if isinstance(plan, dict) else []
    candidates = candidates if isinstance(candidates, list) else []
    batch = dict(schema_version=1, method=METHOD, contract_valid=False, source_contract_valid=False, contract_errors=[],
        source_binding_level='metadata_join_only', plan_content_binding_verified=False,
        source={}, artifact_fingerprints=dict(plan=_fingerprint(plan), adapter=_fingerprint(adapter_result)),
        paths=[], inspection_only=True, planning_allowed=False,
        limitations=['No cost weights, score, ranking, selection or control.',
                     'No-contact samples do not establish continuous safety.',
                     'Clearance is clipped at zero; grid contact time is not continuous TTC.'])
    for index, candidate in enumerate(candidates):
        c = candidate if isinstance(candidate, dict) else {}
        batch['paths'].append(dict(candidate_id=c.get('candidate_id'), input_index=index,
            path_type=c.get('path_type', 'state_matched_trajectory'),
            path_type_origin='explicit' if 'path_type' in c else 'legacy_default_from_plan',
            native_sampled_valid=c.get('sampled_valid'), native_reason=c.get('reason'),
            outcome='rejected', reported_outcome=None, contract_valid=False, contract_errors=[],
            comparison_status='invalid_contract', exclusion_reason=None,
            has_collision_features=False, eligible_for_general_comparison=False,
            full_horizon_checked=False, contact_observed=None,
            prediction_status=None, collision_status=None,
            source_sim_time_s=None, geometry_id=None, frame_id=None,
            **{k: None for k in COPY_FIELDS}, inspection_only=True, planning_allowed=False))
    try:
        if not isinstance(plan, dict) or not isinstance(adapter_result, dict):
            raise ValueError('invalid_plan_or_adapter')
        _slots(candidates, 'plan')
        paired = _slots(adapter_result.get('paths'), 'adapter')
        if not _id(plan.get('plan_id')) or not _id(adapter_result.get('plan_id')) or adapter_result.get('plan_id') != plan['plan_id']:
            raise ValueError('plan_id_mismatch_or_missing')
        if not isinstance(session_metadata, dict):
            raise ValueError('missing_session_metadata')
        geometry, frame = session_metadata.get('geometry_id'), session_metadata.get('frame_id')
        if not all(isinstance(x, str) and x for x in (geometry, frame)):
            raise ValueError('missing_source_geometry_or_frame')
        if plan.get('geometry_id') != geometry or plan.get('frame_id', frame) != frame:
            raise ValueError('plan_metadata_mismatch')
        origin = _number(plan.get('source_sim_time_s'), 'source_time')
        step = plan.get('source_control_step')
        if not _id(step) or step < 0:
            raise ValueError('missing_source_control_step')
        pose = plan.get('source_pose')
        if not isinstance(pose, list) or len(pose) != 3:
            raise ValueError('missing_source_pose')
        pose = [_number(v, 'source_pose') for v in pose]
        batch['source'] = dict(plan_id=plan['plan_id'], source_control_step=step,
                              source_sim_time_s=origin, source_pose=pose, geometry_id=geometry, frame_id=frame)
        _number(adapter_result.get('source_sim_time_s'), 'adapter_source_time')
        if not _id(adapter_result.get('source_control_step')):
            raise ValueError('invalid_adapter_control_step')
        adapter_pose = adapter_result.get('source_pose')
        if not isinstance(adapter_pose, list) or len(adapter_pose) != 3:
            raise ValueError('missing_adapter_pose')
        for v in adapter_pose:
            _number(v, 'adapter_pose')
        for key, expected in batch['source'].items():
            if adapter_result.get(key) != expected:
                raise ValueError('adapter_source_mismatch:' + key)
        if source_state is not None:
            if not isinstance(source_state, dict) or source_state.get('valid') is not True:
                raise ValueError('invalid_source_state')
            if not _id(source_state.get('control_step')):
                raise ValueError('invalid_state_control_step')
            if source_state.get('geometry_id', geometry) != geometry or source_state.get('frame_id', frame) != frame:
                raise ValueError('state_metadata_mismatch')
            _number(source_state.get('sim_time_s'), 'state_time')
            _number(source_state.get('speed_mps'), 'state_speed')
            for key, expected in [('control_step', step), ('sim_time_s', origin), ('pose', pose)]:
                if source_state.get(key) != expected:
                    raise ValueError('state_source_mismatch:' + key)
            if source_state.get('speed_mps') != adapter_result.get('ego_speed_mps'):
                raise ValueError('state_speed_mismatch')
        speed = _number(adapter_result.get('ego_speed_mps'), 'ego_speed')
        dt = _number(adapter_result.get('dt_s'), 'dt_s'); steps = adapter_result.get('steps')
        if speed < 0 or dt <= 0 or not _id(steps) or not 0 <= steps <= 100000:
            raise ValueError('invalid_grid_or_speed')
        horizon = _number(dt * steps, 'horizon')
        if adapter_result.get('batch_status') != 'evaluated':
            raise ValueError('adapter_batch_rejected:' + str(adapter_result.get('reason')))
        if adapter_result.get('inspection_only') is not True or adapter_result.get('planning_allowed') is not False:
            raise ValueError('invalid_research_flags')
        bodies = {}
        for name in ('ego_body', 'lead_body'):
            body = adapter_result.get(name)
            if not isinstance(body, dict):
                raise ValueError('missing_' + name)
            values = {k: _number(body.get(k), name+'_'+k) for k in ('length_m','width_m','margin_m')}
            if values['length_m'] <= 0 or values['width_m'] <= 0 or values['margin_m'] < 0:
                raise ValueError('invalid_'+name)
            bodies[name] = values
        if _number(adapter_result.get('opponent_start_time_s'), 'opponent_start_time') != origin:
            raise ValueError('opponent_origin_mismatch')
        provenance = adapter_result.get('opponent_provenance')
        if not isinstance(provenance, dict) or any(not isinstance(provenance.get(k),str) or not provenance[k] for k in ('kind','description')):
            raise ValueError('missing_opponent_provenance')
        batch['source'].update(ego_speed_mps=speed, dt_s=dt, steps=steps, horizon_s=horizon,
            ego_body=bodies['ego_body'], lead_body=bodies['lead_body'],
            opponent_start_time_s=adapter_result.get('opponent_start_time_s'),
            opponent_provenance={k:provenance[k] for k in ('kind','description','artifact_id','sha256') if k in provenance})
        batch['contract_valid'] = batch['source_contract_valid'] = True
    except (ValueError, TypeError, KeyError, OverflowError) as exc:
        reason = str(exc); batch['contract_errors'] = [reason]
        for row in batch['paths']:
            row.update(exclusion_reason=reason, contract_errors=[reason])
            if row['native_sampled_valid'] is False:
                row['outcome'] = 'rejected'
            elif row['path_type'] == 'reference_centerline':
                row['outcome'] = 'reference_only'
        return _json(batch)

    for candidate, item in zip(candidates, batch['paths']):
        row = paired[candidate['candidate_id']]
        item.update(reported_outcome=row.get('outcome'),
                    outcome=row.get('outcome') if isinstance(row.get('outcome'), str) and row['outcome'] in OUTCOMES else 'rejected',
                    prediction_status=row.get('prediction_status'), collision_status=row.get('collision_status'),
                    source_sim_time_s=origin, geometry_id=geometry, frame_id=frame)
        for k in COPY_FIELDS:
            item[k] = row.get(k)
        if candidate.get('sampled_valid') is False:
            item['outcome'] = 'rejected'
        elif item['path_type'] == 'reference_centerline':
            item['outcome'] = 'reference_only'
        try:
            if type(candidate.get('sampled_valid')) is not bool:
                raise ValueError('invalid_native_status')
            if row.get('native_sampled_valid') is not candidate['sampled_valid'] or row.get('native_reason') != candidate.get('reason'):
                raise ValueError('native_status_mismatch')
            for key, expected in [('source_sim_time_s', origin), ('geometry_id', geometry), ('frame_id', frame)]:
                if row.get(key) != expected:
                    raise ValueError('candidate_source_mismatch:' + key)
            if 'path_type' in row and row['path_type'] != item['path_type']:
                raise ValueError('path_type_mismatch')
            if 'path_type_origin' in row and row['path_type_origin'] != ('explicit' if 'path_type' in candidate else 'legacy_default'):
                raise ValueError('path_type_origin_mismatch')
            item['path_type_origin'] = row.get('path_type_origin', item['path_type_origin'])
            if row.get('inspection_only') is not True or row.get('planning_allowed') is not False:
                raise ValueError('invalid_candidate_research_flags')
            if candidate.get('geometry_id', geometry) != geometry or candidate.get('frame_id', frame) != frame:
                raise ValueError('candidate_metadata_mismatch')
            if item['path_type'] not in ('state_matched_trajectory','reference_centerline'):
                raise ValueError('unsupported_path_type')
            _row_checks(row, horizon)
            if item['outcome'] != row['outcome']:
                raise ValueError('native_or_reference_promoted')
            item['contract_valid'] = True
            item['contact_observed'] = (True if row['outcome'] == 'contact_detected' else
                False if row['outcome'] == 'no_contact_at_samples' else row.get('any_contact_in_checked_samples'))
            excluded = item['outcome'] in ('rejected', 'reference_only')
            item['full_horizon_checked'] = bool(not excluded and row['prediction_complete'] and row['evaluation_complete'])
            has_numeric = any(row.get(k) is not None for k in METRICS)
            item['has_collision_features'] = bool(not excluded and row.get('checked_range_s') is not None and (has_numeric or item['contact_observed'] is True))
            complete_metrics = all(row.get(k) is not None for k in METRICS[:4])
            complete_metrics = complete_metrics and type(row.get('current_contact')) is bool
            if item['outcome'] == 'contact_detected':
                complete_metrics = complete_metrics and row.get('first_contact_grid_t_s') is not None
            if excluded:
                item.update(comparison_status='not_comparable', exclusion_reason=(
                    'reference_not_ego_trajectory' if item['outcome']=='reference_only' else candidate.get('reason') or row.get('reason') or 'candidate_rejected'))
            elif item['full_horizon_checked'] and complete_metrics:
                item.update(comparison_status='comparable_complete', eligible_for_general_comparison=True)
            else:
                item.update(comparison_status='partial_features' if item['has_collision_features'] else 'insufficient_features',
                    exclusion_reason='incomplete_horizon' if not item['full_horizon_checked'] else 'missing_collision_metrics')
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            item.update(contract_errors=[str(exc)], exclusion_reason=str(exc))
    batch['contract_valid'] = all(r['contract_valid'] for r in batch['paths'])
    batch['contract_errors'] = [f"C{r['candidate_id']}:{error}" for r in batch['paths'] for error in r['contract_errors']]
    return _json(batch)

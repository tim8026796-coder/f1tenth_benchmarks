"""Content binding regression; synthetic/archived data, no new optimization."""
from copy import deepcopy
import json
import math
from pathlib import Path

import pytest

from test_prediction_collision_adapter_v1 import inputs
from test_current_prediction_contract import reference_event, evaluate as reference_evaluate
from f1tenth_benchmarks.research.overtaking.plan_binding_v1 import build_plan_binding, verify_plan_binding
from f1tenth_benchmarks.research.overtaking.prediction_collision_adapter_v1 import evaluate_plan_candidates
from f1tenth_benchmarks.research.overtaking.prediction_v1 import predict_polyline
from f1tenth_benchmarks.research.overtaking.cost_input_v1 import build_cost_inputs


def binding(plan, options):
    return build_plan_binding(plan, session_metadata=options['session_metadata'])


def cost(plan, result, options, **kwargs):
    return build_cost_inputs(plan, result, source_state=options['source_state'],
                             session_metadata=options['session_metadata'], **kwargs)


def evaluated():
    p, o = inputs()
    return p, o, evaluate_plan_candidates(p, **o)


def test_deterministic_detached_and_json_roundtrip():
    p, o = inputs(); before = deepcopy((p, o))
    b = binding(p, o)
    assert b == binding(p, o)
    assert verify_plan_binding(json.loads(json.dumps(p)), json.loads(json.dumps(b)),
                               session_metadata=o['session_metadata']) == b
    assert len(b['plan_sha256']) == 64
    assert b['inspection_only'] is True and b['planning_allowed'] is False
    assert (p, o) == before
    b['source']['source_pose'][0] = 77.
    assert p['source_pose'][0] == 0.


def test_dict_order_does_not_matter():
    def reverse(v):
        if isinstance(v, dict): return {k: reverse(v[k]) for k in reversed(v)}
        if isinstance(v, list): return [reverse(x) for x in v]
        return v
    p, o = inputs()
    assert binding(p, o) == binding(reverse(p), o)


@pytest.mark.parametrize('change', ['x','y','yaw','s','d','t','valid','point_reason','point_order',
                                    'candidate_ids','candidate_order','time','pose','geometry','frame',
                                    'path_type','native','reason','plan_id','step','status'])
def test_semantic_changes_alter_batch_digest(change):
    p, o = inputs(); original = binding(p, o); c = p['paths'][0]; pt = c['points'][1]
    if change in ('x','y','yaw'): pt[change] = math.nextafter(pt[change], math.inf)
    elif change in ('s','d','t'): pt[change] = .25
    elif change == 'valid': pt['valid'] = False
    elif change == 'point_reason': pt['reason'] = 'occupied'
    elif change == 'point_order': c['points'].reverse()
    elif change == 'candidate_ids': p['paths'][0]['candidate_id'],p['paths'][1]['candidate_id'] = 2,1
    elif change == 'candidate_order': p['paths'].reverse()
    elif change == 'time': p['source_sim_time_s'] = math.nextafter(10., math.inf)
    elif change == 'pose': p['source_pose'][0] = .001
    elif change == 'geometry': p['geometry_id'] = o['session_metadata']['geometry_id'] = 'other'
    elif change == 'frame': o['session_metadata']['frame_id'] = 'odom'
    elif change == 'path_type': c['path_type'] = 'reference_centerline'
    elif change == 'native': c['sampled_valid'] = False
    elif change == 'reason': c['reason'] = 'fit_not_accepted'
    elif change == 'plan_id': p['plan_id'] = 8
    elif change == 'step': p['source_control_step'] = 5
    else: p['status'] = 'skipped'
    assert binding(p, o)['plan_sha256'] != original['plan_sha256']


def test_domains_and_candidate_scope():
    p, o = inputs(); b = binding(p, o)
    assert all(c['sha256'] != b['plan_sha256'] for c in b['candidate_sha256'])
    p['paths'].reverse(); reversed_binding = binding(p, o)
    assert b['plan_sha256'] != reversed_binding['plan_sha256']
    assert {c['candidate_id']:c['sha256'] for c in b['candidate_sha256']} == {
        c['candidate_id']:c['sha256'] for c in reversed_binding['candidate_sha256']}
    p['paths'][0]['points'][1]['x'] = 11.
    modified = binding(p, o)
    assert modified['candidate_sha256'][0] != reversed_binding['candidate_sha256'][0]
    assert modified['candidate_sha256'][1:] == reversed_binding['candidate_sha256'][1:]
    bad = deepcopy(b); bad['plan_sha256'] = b['candidate_sha256'][0]['sha256']
    with pytest.raises(ValueError, match='plan_binding_mismatch'):
        verify_plan_binding(p, bad, session_metadata=o['session_metadata'])


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
@pytest.mark.parametrize('where', ['point','pose','time'])
def test_nonfinite_is_rejected_not_normalized(value, where):
    p, o = inputs()
    if where == 'point': p['paths'][0]['points'][1]['x'] = value
    elif where == 'pose': p['source_pose'][0] = value
    else: p['source_sim_time_s'] = value
    with pytest.raises(ValueError): binding(p, o)
    r = evaluate_plan_candidates(p, **o)
    assert r['batch_status'] == 'rejected' and r['plan_binding'] is None
    assert not any(c['evaluation_complete'] for c in r['paths'])
    json.dumps(r, allow_nan=False)


def test_exact_float_encoding_and_numeric_types():
    p, o = inputs(); b = binding(p, o)
    p['paths'][0]['points'][0]['x'] = -0.0
    assert binding(p, o)['plan_sha256'] != b['plan_sha256']
    p['paths'][0]['points'][0]['x'] = 0
    assert binding(p, o)['plan_sha256'] != b['plan_sha256']


@pytest.mark.parametrize('change',['missing_frame','bad_keys','object','missing_id','duplicate_id'])
def test_unavailable_identity_or_unserializable_content(change):
    p, o = inputs()
    if change == 'missing_frame': del o['session_metadata']['frame_id']
    elif change == 'bad_keys': p['paths'][0]['points'][0][3] = 2
    elif change == 'object': p['paths'][0]['points'][0]['x'] = object()
    elif change == 'missing_id': del p['plan_id']
    else: p['paths'][0]['candidate_id'] = 2
    with pytest.raises(ValueError): binding(p, o)


def test_unrelated_diagnostics_and_available_time_are_not_bound():
    p, o = inputs(); b = binding(p, o)
    p.update(wall_time_s=999., available_sim_time_s=1000., result={'diagnostic':'changed'})
    p['paths'][0]['optimizer_status'] = 'diagnostic only'
    assert binding(p, o) == b


def test_adapter_records_actual_input_and_cost_verifies():
    p, o = inputs(); before = deepcopy((p, o))
    r = evaluate_plan_candidates(p, **o)
    assert r['plan_binding'] == binding(p, o) and r['plan_binding_status'] == 'recorded'
    out = cost(p, r, o)
    assert out['binding_status'] == 'verified' and out['plan_content_binding_verified']
    assert out['source_binding_level'] == 'plan_content_sha256'
    assert out['contract_valid'] and all(c['eligible_for_general_comparison'] for c in out['paths'])
    assert (p, o) == before
    r = json.loads(json.dumps(r, allow_nan=False)); p = json.loads(json.dumps(p))
    assert cost(p, r, o)['plan_content_binding_verified']


def test_modified_points_with_identical_metadata_are_rejected():
    p, o, r = evaluated(); old = deepcopy(r)
    p['paths'][1]['points'][1]['x'] = 9.
    out = cost(p, r, o)
    assert r == old
    assert not out['contract_valid'] and not out['plan_content_binding_verified']
    assert out['binding_status'] == 'invalid_binding'
    assert out['contract_errors'] == ['plan_binding_mismatch']
    assert all(not c['eligible_for_general_comparison'] for c in out['paths'])


@pytest.mark.parametrize('change',['schema','version','version_bool','algorithm','canonical','candidate_hash','missing_digest','null'])
def test_no_fallback_from_invalid_binding(change):
    p, o, r = evaluated(); b = r['plan_binding']
    if change == 'schema': b['binding_schema'] = 'other'
    elif change == 'version': b['binding_version'] = 2
    elif change == 'version_bool': b['binding_version'] = True
    elif change == 'algorithm': b['hash_algorithm'] = 'md5'
    elif change == 'canonical': b['canonicalization'] = 'other'
    elif change == 'candidate_hash': b['candidate_sha256'][0]['sha256'] = '0'*64
    elif change == 'missing_digest': del b['plan_sha256']
    else: r['plan_binding'] = None
    out = cost(p, r, o, legacy_policy='allow_metadata_only')
    assert not out['contract_valid'] and out['binding_status'] == 'invalid_binding'
    assert not out['plan_content_binding_verified']


def test_legacy_policy_is_explicit_and_does_not_claim_binding():
    p, o, r = evaluated(); del r['plan_binding']
    blocked = cost(p, r, o)
    assert not blocked['contract_valid'] and blocked['binding_status'] == 'legacy_unbound'
    assert blocked['contract_errors'] == ['legacy_plan_binding_required']
    allowed = cost(p, r, o, legacy_policy='allow_metadata_only')
    assert allowed['contract_valid'] and not allowed['plan_content_binding_verified']
    assert allowed['source_binding_level'] == 'metadata_join_only'
    assert allowed['binding_trust_policy'] == 'allow_metadata_only'
    assert not cost(p, r, o, legacy_policy='guess')['contract_valid']


def test_reference_and_native_rejection_are_bound_not_promoted():
    p, o = inputs(); p['paths'][0].update(sampled_valid=False,reason='fit_not_accepted',points=[])
    r = evaluate_plan_candidates(p, **o); out = cost(p, r, o)
    assert out['plan_content_binding_verified'] and out['paths'][0]['outcome'] == 'rejected'
    assert not out['paths'][0]['eligible_for_general_comparison']
    g, p, state = reference_event(.02); r = reference_evaluate(g, p, state)
    out = build_cost_inputs(p, r, source_state=state, session_metadata=dict(geometry_id=g.geometry_id,frame_id='map'))
    assert out['plan_content_binding_verified'] and out['paths'][2]['outcome'] == 'reference_only'
    assert not out['paths'][2]['eligible_for_general_comparison']


@pytest.mark.parametrize('contact',[False,True])
def test_incomplete_keeps_unknown_or_contact_evidence(contact):
    p, o = inputs(); p['paths'][0]['points'][1]['x'] = .75
    if contact:
        o['opponent_prediction'] = predict_polyline([[.5,0.],[10.,0.]],[.5,0.],0.,.5,8,geometry_id='g',frame_id='map')
    r = evaluate_plan_candidates(p, **o); out = cost(p, r, o); c = out['paths'][0]
    assert out['plan_content_binding_verified'] and c['contract_valid']
    assert c['outcome'] == ('contact_detected' if contact else 'unknown')
    assert c['unchecked_range_s'] == dict(after_t_s=.5,through_t_s=4.)
    assert not c['eligible_for_general_comparison'] and c['has_collision_features']
    if contact: assert c['current_contact'] and c['first_contact_grid_t_s'] == 0.


CASES = ['A_far_centerline','B_stationary_inside_candidate_region','C_crossing_different_arrival',
         'D_incomplete_opponent','E_saved_terminal_rejection']


@pytest.mark.parametrize('case',CASES)
def test_archives_readable_and_new_evaluation_bound_without_rewriting(case):
    import hashlib
    root = Path('docs/research/esp_collision_offline_v1')
    files = list(root.rglob('*')); before = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()}
    summary = json.loads((root/'results'/case/'summary.json').read_text())
    old = dict(summary['result'],paths=summary['paths'])
    source = root/'sources'/('esp_timeout_mutated_test' if case.startswith('E_') else 'esp_short_preview')
    plans = [json.loads(s) for s in (source/'plans.jsonl').read_text().splitlines()]
    states = [json.loads(s) for s in (source/'states.jsonl').read_text().splitlines()]
    p = next(p for p in plans if p['plan_id'] == 1)
    state = next(s for s in states if s['control_step'] == p['source_control_step'])
    metadata = json.loads((source/'metadata.json').read_text())
    legacy = build_cost_inputs(p,old,source_state=state,session_metadata=metadata,legacy_policy='allow_metadata_only')
    assert legacy['binding_status'] == 'legacy_unbound' and not legacy['plan_content_binding_verified']
    scenario = json.loads((root/'scenarios'/(case+'.json')).read_text()); op = scenario['opponent']
    prediction = predict_polyline(op['path_xy'],op['start_xy'],op['speed_mps'],scenario['dt_s'],scenario['steps'],
                                  frame_id=op['frame_id'],geometry_id=op['geometry_id'])
    options = dict(source_state=state,session_metadata=metadata,opponent_prediction=prediction,
        opponent_start_time_s=op['start_time_s'],opponent_provenance=op['provenance'],
        ego_body=scenario['ego_body'],lead_body=scenario['lead_body'],dt_s=scenario['dt_s'],steps=scenario['steps'])
    r = evaluate_plan_candidates(p,**options); out = cost(p,r,options)
    assert r['plan_binding'] == binding(p,options)
    assert out['plan_content_binding_verified']
    assert [c['outcome'] for c in r['paths']] == [c['outcome'] for c in old['paths']]
    assert out['inspection_only'] and not out['planning_allowed']
    if case.startswith('E_'):
        assert r['source_identity_verified'] and r['source_sim_time_s'] == p['source_sim_time_s']
        assert r['batch_status'] == 'rejected' and r['reason'] == 'plan_not_completed'
        assert out['contract_errors'] == ['invalid_source_state']
        assert legacy['contract_errors'] == ['invalid_adapter_source_time']
        assert all(c['outcome'] == 'rejected' and c['minimum_clearance_m'] is None for c in r['paths'])
        assert not any(c['eligible_for_general_comparison'] for c in out['paths'])
    else:
        assert out['contract_valid']
        assert [c['minimum_clearance_m'] for c in r['paths']] == [c['minimum_clearance_m'] for c in old['paths']]
    after = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()}
    assert before == after


def test_invalid_source_not_published_as_verified_identity():
    p, o = inputs(); o['source_state']['sim_time_s'] = 11.
    r = evaluate_plan_candidates(p,**o)
    assert r['batch_status'] == 'rejected' and r['plan_binding'] is None
    assert not r['source_identity_verified'] and r['source_sim_time_s'] is None


@pytest.mark.parametrize('change',['candidate_boolean','source_float'])
def test_binding_record_uses_strict_types(change):
    p, o = inputs(); b = binding(p, o)
    if change == 'candidate_boolean': b['candidate_sha256'][0]['candidate_id'] = True
    else: b['source']['plan_id'] = float(b['source']['plan_id'])
    with pytest.raises(ValueError, match='plan_binding_mismatch'):
        verify_plan_binding(p,b,session_metadata=o['session_metadata'])


def test_frame_must_be_explicit_and_match_metadata():
    p, o = inputs(); p['frame_id'] = 'map'
    assert build_plan_binding(p) == binding(p,o)
    o['session_metadata']['frame_id'] = 'odom'
    with pytest.raises(ValueError, match='binding_metadata_mismatch:frame_id'):
        binding(p,o)

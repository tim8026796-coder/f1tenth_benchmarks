"""Synthetic saved-format contracts only; no optimizer or simulator startup."""
import copy
import csv
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from f1tenth_benchmarks.research.overtaking.session import SessionWriter, make_plan, SessionConfig
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('esp_offline', ROOT/'tools/evaluate_esp_collision_v1.py')
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


@pytest.fixture(scope='module')
def track():
    return tool.TrackData('maps/frenet/esp/geometry', 'maps/esp.yaml')


@pytest.fixture
def saved(tmp_path, track):
    root = tmp_path/'synthetic_session'
    state = dict(control_step=0, sim_time_s=10., pose=[0.,0.,0.], speed_mps=1.,
                 valid=True, frenet=dict(valid=True))
    def synthetic(geometry,x,y,yaw,**kwargs):
        return dict(paths=[dict(candidate_id=i, geometry_id=geometry.geometry_id, frame_id='map',
            sampled_valid=True, reason=None, points=[dict(x=0.,y=0.,yaw=0.,valid=True,t=0.),
                                                  dict(x=10.,y=0.,yaw=0.,valid=True,t=1.)]) for i in range(1,6)])
    plan = make_plan(SimpleNamespace(geometry_id=track.geometry.geometry_id), state, SessionConfig(),0,generator=synthetic)
    metadata = dict(schema_version=1,map='esp',kind='synthetic_contract_fixture',geometry_id=track.geometry.geometry_id,
                    frame_id='map',track=track.provenance,playback_origin_s=10.)
    writer = SessionWriter(root,metadata)
    writer.append('states',state);writer.append('plans',plan)
    writer.finalize(dict(schema_version=1,state_count=1,plan_count=1,end_reason='synthetic',completed_full_laps=0))
    return root


@pytest.fixture
def scenario(track):
    return dict(name='synthetic_parallel', session_classification='synthetic_contract_fixture', dt_s=.5,steps=8,
        ego_body=dict(length_m=.6,width_m=.8,margin_m=0.),lead_body=dict(length_m=.6,width_m=.8,margin_m=0.),
        opponent=dict(path_xy=[[0.,3.],[10.,3.]],start_xy=[0.,3.],speed_mps=1.,frame_id='map',
                      geometry_id=track.geometry.geometry_id,start_time_s=10.,
                      provenance=dict(kind='synthetic',description='Synthetic contract fixture only')))


def change(root,name,fn):
    p=root/name
    if name.endswith('jsonl'):
        data=[json.loads(s) for s in p.read_text().splitlines()];fn(data)
        p.write_text(''.join(json.dumps(x)+'\n' for x in data))
    else:
        data=json.loads(p.read_text());fn(data);p.write_text(json.dumps(data))


def evaluate(saved,scenario,track,tmp_path):
    return tool.evaluate_saved_session(saved,0,scenario,track=track,output_dir=tmp_path/'result')


def test_read_join_hash_output_and_no_source_change(saved,scenario,track,tmp_path):
    before={p.name:p.read_bytes() for p in saved.iterdir()}
    r=evaluate(saved,scenario,track,tmp_path)
    assert before=={p.name:p.read_bytes() for p in saved.iterdir()}
    assert r['source']['source_control_step']==0 and r['source']['source_sim_time_s']==10.
    assert [c['candidate_id'] for c in r['result']['paths']]==[1,2,3,4,5]
    assert all(c['outcome']=='no_contact_at_samples' for c in r['result']['paths'])
    assert r['inspection_only'] and not r['planning_allowed']
    assert json.loads((tmp_path/'result/evaluation.json').read_text())==r
    with (tmp_path/'result/summary.csv').open() as f:rows=list(csv.DictReader(f))
    for a,b in zip(rows,r['result']['paths']):
        assert {k:json.loads(v) for k,v in a.items()}=={k:b[k] for k in tool.FIELDS}
    assert 'no_contact_at_samples' in (tmp_path/'result/summary.md').read_text()
    for name,hash_ in r['source']['file_sha256'].items():assert tool.sha256(saved/name)==hash_


@pytest.mark.parametrize('field,value',[('source_sim_time_s',11.),('source_control_step',1),('source_pose',[1.,0.,0.])])
def test_source_mismatch_rejected(saved,scenario,track,tmp_path,field,value):
    change(saved,'plans.jsonl',lambda rows:rows[0].update({field:value}))
    with pytest.raises((ValueError,IndexError)):evaluate(saved,scenario,track,tmp_path)
    assert not (tmp_path/'result').exists()


@pytest.mark.parametrize('field',['speed_mps','pose','sim_time_s'])
def test_missing_state_field(saved,scenario,track,tmp_path,field):
    change(saved,'states.jsonl',lambda rows:rows[0].pop(field))
    with pytest.raises((ValueError,KeyError)):evaluate(saved,scenario,track,tmp_path)


@pytest.mark.parametrize('field',['path_xy','start_time_s','provenance'])
def test_missing_opponent_field(saved,scenario,track,tmp_path,field):
    del scenario['opponent'][field]
    with pytest.raises(ValueError):evaluate(saved,scenario,track,tmp_path)


def test_rejection_and_order_preserved(saved,scenario,track,tmp_path):
    def edit(rows):
        rows[0]['paths'].reverse()
        rows[0]['paths'][0].update(sampled_valid=False,reason='fit_not_accepted',points=[])
    change(saved,'plans.jsonl',edit)
    r=evaluate(saved,scenario,track,tmp_path)['result']['paths']
    assert [p['candidate_id'] for p in r]==[5,4,3,2,1]
    assert r[0]['outcome']=='rejected' and r[0]['native_reason']=='fit_not_accepted'


@pytest.mark.parametrize('case',['contact','asynchronous','short'])
def test_distinct_scenarios(saved,scenario,track,tmp_path,case):
    op=scenario['opponent']
    if case=='contact':op.update(path_xy=[[3.,0.],[10.,0.]],start_xy=[3.,0.],speed_mps=0.)
    elif case=='asynchronous':op.update(path_xy=[[2.,-5.],[2.,10.]],start_xy=[2.,-5.],speed_mps=1.)
    else:op.update(path_xy=[[0.,3.],[.01,3.]],start_xy=[0.,3.])
    r=evaluate(saved,scenario,track,tmp_path)['result']['paths'][0]
    if case=='contact':assert r['outcome']=='contact_detected' and r['first_future_contact_grid_t_s']==2.
    elif case=='short':assert r['outcome']=='unknown' and not r['evaluation_complete']
    else:assert r['outcome']=='no_contact_at_samples'


@pytest.mark.parametrize('case',['frame','time','grid'])
def test_frame_time_grid_reject(saved,scenario,track,tmp_path,case):
    if case=='frame':scenario['opponent']['frame_id']='odom'
    elif case=='time':scenario['opponent']['start_time_s']=11.
    else:scenario['dt_s']=0.
    r=evaluate(saved,scenario,track,tmp_path)['result']
    assert r['batch_status']=='rejected' and all(c['outcome']=='rejected' for c in r['paths'])


def test_cannot_regenerate(saved,scenario,track,tmp_path,monkeypatch):
    from f1tenth_benchmarks.research.overtaking import session,robust_adaptive_paths
    def forbidden(*args,**kwargs):pytest.fail('No candidate generation allowed')
    monkeypatch.setattr(session,'generate_robust_paths',forbidden)
    monkeypatch.setattr(robust_adaptive_paths,'generate_robust_paths',forbidden)
    assert evaluate(saved,scenario,track,tmp_path)['result']['batch_status']=='evaluated'


def test_refuses_overwrite_or_source_output(saved,scenario,track,tmp_path):
    evaluate(saved,scenario,track,tmp_path)
    with pytest.raises(FileExistsError):evaluate(saved,scenario,track,tmp_path)
    with pytest.raises(ValueError):tool.evaluate_saved_session(saved,0,scenario,track=track,output_dir=saved/'new')


def test_centerline_seam_is_explicit_finite_prefix(track):
    path=tool.centerline_reference(track,len(track.centerline)-3,2.)
    assert len(path)>3
    assert sum(np.linalg.norm(np.diff(path,axis=0),axis=1))==pytest.approx(2.)
    assert not np.array_equal(path[0],path[-1])
    with pytest.raises(ValueError):tool.centerline_reference(track,0,1e6)


def test_cli(saved,scenario,track,tmp_path):
    file=tmp_path/'scenario.json';file.write_text(json.dumps(scenario))
    env=dict(os.environ,PYTHONPATH=str(ROOT),PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1')
    run=subprocess.run([sys.executable,'-B','-m','tools.evaluate_esp_collision_v1',
        '--session-dir',str(saved),'--plan-id','0','--scenario-json',str(file),
        '--output-dir',str(tmp_path/'cli')],cwd=ROOT,env=env,capture_output=True,text=True)
    assert run.returncode==0,run.stderr
    assert (tmp_path/'cli/evaluation.json').exists()


@pytest.mark.parametrize('field',['opponent','provenance'])
def test_invalid_opponent_container_has_clear_reason(saved,scenario,track,tmp_path,field):
    if field=='opponent':scenario['opponent']=None
    else:scenario['opponent']['provenance']=None
    with pytest.raises(ValueError,match='invalid_opponent'):evaluate(saved,scenario,track,tmp_path)

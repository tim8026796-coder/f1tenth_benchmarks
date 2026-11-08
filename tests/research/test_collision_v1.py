import copy
import importlib
import math
import subprocess
import sys
from pathlib import Path
import pytest
from f1tenth_benchmarks.research.overtaking.prediction_v1 import predict_polyline, predict_candidate
from f1tenth_benchmarks.research.overtaking.collision_v1 import evaluate_collision

REPO = Path(__file__).resolve().parents[2]

BODY = dict(length_m=.6, width_m=.8, margin_m=0.)

def pred(y=0., start=0., speed=1., end=30., dt=.5, steps=12):
    return predict_polyline([[start,y],[end,y]], [start,y], speed, dt, steps,
                            geometry_id='synthetic-test',frame_id='map')

def check(e=None,l=None,**kw):
    args=dict(common_start_time_s=100.,ego_start_time_s=100.,lead_start_time_s=100.,
              ego_body=BODY,lead_body=BODY)
    args.update(kw)
    return evaluate_collision(pred() if e is None else e,pred(y=2.) if l is None else l,**args)

def test_hand_catch_up():
    r=check(pred(speed=2.),pred(start=3.))
    assert r['evaluation_complete'] and r['first_contact_grid_t_s']==2.
    assert not r['current_contact'] and r['first_future_contact_grid_t_s']==2.
    assert r['minimum_center_distance_m']==0 and r['minimum_center_distance_t_s']==3.
    assert r['minimum_clearance_m']==0 and r['minimum_clearance_t_s']==2.
    assert [x['center_distance_m'] for x in r['samples']]==[abs(3-i*.5) for i in range(13)]

def test_tangency_and_t0():
    r=check(pred(speed=0),pred(y=1.,speed=0))
    assert r['current_contact'] and r['first_contact_grid_t_s']==0
    assert r['first_future_contact_grid_t_s']==.5
    assert r['minimum_clearance_m']==0

def test_separated():
    r=check()
    assert r['assessment']=='no_contact_at_samples' and not r['any_contact_in_checked_samples']
    assert r['minimum_center_distance_m']==2 and r['minimum_clearance_m']==1
    assert r['inspection_only'] and not r['planning_allowed'] and not r['continuous_sweep_checked']

@pytest.mark.parametrize('offset,contact',[(2.,True),(5.,False)])
def test_crossing(offset,contact):
    e=predict_polyline([[-2.,0.],[20.,0.]],[-2.,0.],1.,.1,60,frame_id='map')
    l=predict_polyline([[0.,-offset],[0.,20.]],[0.,-offset],1.,.1,60,frame_id='map')
    assert check(e,l)['any_contact_in_checked_samples'] is contact

def test_partial_unknown():
    r=check(pred(end=1.),pred(y=2.))
    assert r['status']=='incomplete' and r['assessment']=='unknown'
    assert r['checked_range_s']==[0.,1.] and not r['evaluation_complete']
    assert r['input_sample_counts']==dict(ego=3,lead=13)
    assert r['unchecked_range_s']==dict(after_t_s=1.,through_t_s=6.)

def test_partial_can_report_contact():
    r=check(pred(end=1.),pred(start=.5,speed=0))
    assert r['status']=='incomplete' and r['assessment']=='contact_detected'

def test_zero_horizon():
    r=check(pred(steps=0),pred(y=2.,steps=0))
    assert r['evaluation_complete'] and len(r['samples'])==1
    assert r['first_future_contact_grid_t_s'] is None

@pytest.mark.parametrize('key,value', [('frame_id','odom'),('frame_id',None),('status','rejected'),
    ('samples',[]),('requested_horizon_s',7.),('last_sample_t_s',5.),('available_until_t_s',0.),
    ('prediction_complete',False),('time_origin','absolute')])
def test_bad_prediction(key,value):
    p=pred(y=2.); p[key]=value
    r=check(l=p)
    assert r['status']=='rejected' and r['assessment']=='unknown' and not r['samples']
    assert r['any_contact_in_checked_samples'] is None

@pytest.mark.parametrize('key', ['t_s','x','y','yaw','speed_mps'])
@pytest.mark.parametrize('value',[float('nan'),float('inf')])
def test_nonfinite_samples(key,value):
    p=pred();p['samples'][2][key]=value
    assert check(e=p)['status']=='rejected'

@pytest.mark.parametrize('kind',['reorder','missing_middle','shifted','negative_speed','missing_x'])
def test_bad_samples(kind):
    p=pred()
    if kind=='reorder': p['samples'][1],p['samples'][2]=p['samples'][2],p['samples'][1]
    elif kind=='missing_middle': p['samples'].pop(2)
    elif kind=='shifted':
        for s in p['samples']:s['t_s']+=1
    elif kind=='negative_speed':p['samples'][1]['speed_mps']=-1
    else:del p['samples'][2]['x']
    assert check(e=p)['status']=='rejected'

@pytest.mark.parametrize('kw',[{'lead_start_time_s':101.},{'common_start_time_s':float('nan')},
    {'ego_start_time_s':None},{'lead_start_time_s':True}])
def test_bad_origins(kw):
    assert check(**kw)['status']=='rejected'

@pytest.mark.parametrize('spec',[{},dict(length_m=0,width_m=1,margin_m=0),
    dict(length_m=1,width_m=-1,margin_m=0),dict(length_m=1,width_m=1,margin_m=-.1),
    dict(length_m=float('inf'),width_m=1,margin_m=0)])
def test_bad_dimensions(spec):
    assert check(ego_body=spec)['status']=='rejected'

@pytest.mark.parametrize('p',[pred(dt=.25,steps=24),pred(steps=10)])
def test_different_requested_grid(p):
    assert check(l=p)['reason']=='requested_time_grid_mismatch'

def test_different_geometry_same_frame_allowed():
    p=pred(y=2);p['geometry_id']='different-path'
    assert check(l=p)['evaluation_complete']

def test_radius_margin_explicit():
    r=check(ego_body=dict(length_m=.6,width_m=.8,margin_m=.2))
    assert r['bodies']['ego']['radius_m']==.7
    assert r['minimum_clearance_m']==pytest.approx(.8)

def test_native_candidate_adapter():
    candidate=dict(candidate_id=3,sampled_valid=True,reason=None,geometry_id='synthetic-native-format',
        frame_id='map',points=[dict(t=0.,s=0.,d=0.,x=0.,y=0.,yaw=0.,valid=True,reason=None),
                             dict(t=1.,s=30.,d=0.,x=30.,y=0.,yaw=0.,valid=True,reason=None)])
    p=predict_candidate(candidate,[0.,0.],1.,.5,12)
    r=check(e=p)
    assert r['evaluation_complete'] and r['sources']['ego']['candidate_id']==3
    assert r['sources']['ego']['geometry_id']=='synthetic-native-format'
    assert p['samples'][2]['t_s']==1. and p['samples'][2]['x']==1.
    candidate['sampled_valid']=False
    p=predict_candidate(candidate,[0.,0.],1.,.5,12)
    r=check(e=p)
    assert r['status']=='rejected' and r['sources']['ego']['reason']=='candidate_rejected'

def test_import_no_outputs(tmp_path):
    code='from f1tenth_benchmarks.research.overtaking import collision_v1'
    import os
    env=dict(os.environ,PYTHONPATH=str(REPO),PYTHONDONTWRITEBYTECODE='1')
    subprocess.run([sys.executable,'-B','-c',code],cwd=tmp_path,env=env,check=True)
    assert list(tmp_path.iterdir())==[]

def test_cli_outputs_and_no_overwrite(tmp_path):
    import csv, json, os
    from PIL import Image
    directory=tmp_path/'demo'
    env=dict(os.environ,PYTHONPATH=str(REPO),PYTHONDONTWRITEBYTECODE='1',
             MPLCONFIGDIR='/tmp/collision-v1-mpl',XDG_CACHE_HOME='/tmp/collision-v1-cache')
    command=[sys.executable,'-B','-m','f1tenth_benchmarks.research.overtaking.collision_v1',
             '--output-dir',str(directory)]
    run=subprocess.run(command,cwd=tmp_path,env=env,capture_output=True,text=True)
    assert run.returncode==0,run.stderr
    data=json.loads((directory/'collision.json').read_text())
    assert len(data['results'])==6 and data['results']['path_exhausted']['status']=='incomplete'
    assert len(list(csv.DictReader((directory/'collision.csv').open())))==161
    with Image.open(directory/'collision.png') as im:
        assert im.size==(3600,2000)
        im.verify()
    before={p.name:p.read_bytes() for p in directory.iterdir()}
    assert subprocess.run(command,cwd=tmp_path,env=env,capture_output=True).returncode!=0
    assert before=={p.name:p.read_bytes() for p in directory.iterdir()}

def test_python39_grammar():
    import ast
    path=REPO/'f1tenth_benchmarks/research/overtaking/collision_v1.py'
    ast.parse(path.read_text(),feature_version=(3,9))

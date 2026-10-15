# Three-method candidate path research comparison

Algorithm commit: `97e121b28d4bc1f29c65c9a1d4ca4b142d8049a8`; planners and existing tests are unchanged.
12 cases; each has identical geometry, start, horizon and 0.2 m margin for all three methods. Common reference-s spacing is 0.025 m.
V3 is evaluated from its original Hermite function; V5 is piecewise-linear Frenet resampling of its ORIGINAL generated points, without re-querying width to regenerate d; constrained V5 uses its original serialized spline. Missing native samples remain gaps.
All methods are judged by exact source-consistent Frenet geometry, raw road margins (1e-8 m tolerance), and conservative occupancy segment supercover. Unknown/outside cells are blocked.
Native acceptance and common validity are separate. Accepted + common is their intersection; a passing rejected diagnostic curve is not an accepted plan.
Curvature uses the same discrete three-point Cartesian estimator. Slope jumps use consecutive finite differences in reference s. Gaps split both calculations.
Lateral margins are signed ray clearances, not vehicle footprint distances. Boundary dots are discrete ray hits. No spline-between-samples, vehicle, dynamic or opponent safety certification.
All-generated metrics include rejected diagnostic curves; accepted-common metrics in summary.csv include only the accepted intersection.

|Case|Method|Native|Common|Accepted + common|Min margin, all generated (m)|Max slope jump, all generated|Max curvature, all generated (1/m)|
|---|---|---:|---:|---:|---:|---:|---:|
|esp_40|V3|3|3|3|0.139932|6.77581e-05|1.09195|
|esp_40|V5|5|5|5|0.2|2.41654|52.7604|
|esp_40|constrained_V5|3|3|3|0.183427|0.00877632|1.10794|
|aut_30|V3|0|0|0|0.0751657|0.000138961|1400.74|
|aut_30|V5|0|1|0|0.2|649.394|502.433|
|aut_30|constrained_V5|0|0|0|0.174412|0.00167195|1.29085|
|my_map_5|V3|0|0|0|-0.0871341|0.00198261|1.34545|
|my_map_5|V5|5|5|5|0.2|4.69005|45.1453|
|my_map_5|constrained_V5|2|2|2|0.2|0.00475326|1.16438|
|esp_s30_10|V3|3|3|3|0.138161|0.00113747|0.343567|
|esp_s30_10|V5|5|5|5|0.2|2.33924|44.9136|
|esp_s30_10|constrained_V5|3|3|3|0.502943|0.00087537|0.306095|
|esp_s90_20|V3|3|3|3|0.164707|0.000276019|0.641848|
|esp_s90_20|V5|5|5|5|0.2|2.0675|39.7863|
|esp_s90_20|constrained_V5|4|4|4|0.197058|0.0351733|1.2733|
|aut_s40_10|V3|3|3|3|0.166913|0.00109011|316.472|
|aut_s40_10|V5|1|1|1|0.2|262.342|127.653|
|aut_s40_10|constrained_V5|0|0|0|—|—|—|
|aut_s80_15|V3|4|4|4|0.19944|0.000485444|2.1493|
|aut_s80_15|V5|5|5|5|0.2|1.99471|46.8605|
|aut_s80_15|constrained_V5|5|5|5|0.2|0.00975193|1.28033|
|my_map_s4_3|V3|1|1|1|0.050359|0.00722341|1.36018|
|my_map_s4_3|V5|5|5|5|0.2|2.97554|38.0471|
|my_map_s4_3|constrained_V5|2|2|2|0.2|0.00478394|1.127|
|my_map_s6_5|V3|3|3|3|0.15738|0.00192463|1.2953|
|my_map_s6_5|V5|5|5|5|0.2|5.91577|45.1634|
|my_map_s6_5|constrained_V5|4|4|4|0.2|0.00282919|1.24947|
|narrow_0.6|V3|1|1|1|-0.46545|0.0166641|0.666236|
|narrow_0.6|V5|5|0|0|0.2|62.3068|5.33281|
|narrow_0.6|constrained_V5|3|3|3|0.2|0.0152301|0.608107|
|narrow_1.0|V3|1|1|1|-0.26545|0.0166641|0.666236|
|narrow_1.0|V5|5|1|1|0.2|60.2549|5.96337|
|narrow_1.0|constrained_V5|5|5|5|0.2|0.0276805|1.1046|
|narrow_1.4|V3|3|3|3|-0.06545|0.0166641|0.666236|
|narrow_1.4|V5|5|3|3|0.2|52.424|6.75221|
|narrow_1.4|constrained_V5|5|5|5|0.2|0.0221697|0.886024|

For segment failures, the plotted cross marks the endpoint of the first rejected segment, not a claim that the endpoint is occupied. JSON preserves segment indices and blocked cell coordinates.

## Case provenance and per-candidate rejection reasons

### esp_40

[Three-method PNG](esp_40.png)

Geometry `c53238c43d356997b7035bacb1907958cbc35d77d8f8916ec4691ca9c934de71`, start `{'x': 0.0, 'y': 0.0, 's': 236.83063142271715}`, configuration `{'name': 'esp_40', 'map_name': 'esp', 'source_s': None, 'lookahead_m': 40, 'safety_margin_m': 0.2}`.

- V3: C1: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0
- V5: C1: native=OK, common=OK, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0
- constrained_V5: C1: native=fit_not_accepted, common=insufficient_lateral_margin, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=fit_not_accepted, common=insufficient_lateral_margin, failed segments=0

### aut_30

PNG not included in this branch (numerical results retained below).

Geometry `8e86e7defb3729fbb89f5dd2bd5ead31e3c57428b31d7f0530b4e2efff9e673d`, start `{'x': 1.0549452470830851, 'y': -0.026262044674926693, 's': 1.000000000000007}`, configuration `{'name': 'aut_30', 'map_name': 'aut', 'source_s': 1, 'lookahead_m': 30, 'safety_margin_m': 0.2}`.

- V3: C1: native=singular_frenet, common=not_generated, failed segments=0; C2: native=singular_frenet, common=not_generated, failed segments=0; C3: native=ambiguous_projection, common=not_generated, failed segments=0; C4: native=ambiguous_projection, common=ambiguous_projection, failed segments=0; C5: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0
- V5: C1: native=singular_frenet, common=segment_occupied, failed segments=5; C2: native=ambiguous_projection, common=segment_occupied, failed segments=2; C3: native=ambiguous_projection, common=segment_occupied, failed segments=1; C4: native=ambiguous_projection, common=ambiguous_projection, failed segments=0; C5: native=insufficient_lateral_margin, common=OK, failed segments=0
- constrained_V5: C1: native=fit_not_accepted, common=not_generated, failed segments=0; C2: native=fit_not_accepted, common=not_generated, failed segments=0; C3: native=fit_not_accepted, common=not_generated, failed segments=0; C4: native=fit_not_accepted, common=not_generated, failed segments=0; C5: native=fit_not_accepted, common=insufficient_lateral_margin, failed segments=0

### my_map_5

PNG not included in this branch (numerical results retained below).

Geometry `3257b83623ab4b87ed906687e624e365f4c09339b7e8fd280e4dfb1e2a387085`, start `{'x': 1.8559638677736017, 'y': -2.231503668085311, 's': 1.0000000000002753}`, configuration `{'name': 'my_map_5', 'map_name': 'my_map', 'source_s': 1, 'lookahead_m': 5, 'safety_margin_m': 0.2}`.

- V3: C1: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=24; C2: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C3: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C4: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C5: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0
- V5: C1: native=OK, common=OK, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0
- constrained_V5: C1: native=fit_not_accepted, common=not_generated, failed segments=0; C2: native=fit_not_accepted, common=not_generated, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=fit_not_accepted, common=not_generated, failed segments=0

### esp_s30_10

PNG not included in this branch (numerical results retained below).

Geometry `c53238c43d356997b7035bacb1907958cbc35d77d8f8916ec4691ca9c934de71`, start `{'x': 26.155993907286955, 'y': -5.890712636384478, 's': 29.99999999999997}`, configuration `{'name': 'esp_s30_10', 'map_name': 'esp', 'source_s': 30, 'lookahead_m': 10, 'safety_margin_m': 0.2}`.

- V3: C1: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0
- V5: C1: native=OK, common=OK, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0
- constrained_V5: C1: native=fit_not_accepted, common=not_generated, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=fit_not_accepted, common=not_generated, failed segments=0

### esp_s90_20

PNG not included in this branch (numerical results retained below).

Geometry `c53238c43d356997b7035bacb1907958cbc35d77d8f8916ec4691ca9c934de71`, start `{'x': 26.266952758565495, 'y': -11.895368131510125, 's': 89.99999999999997}`, configuration `{'name': 'esp_s90_20', 'map_name': 'esp', 'source_s': 90, 'lookahead_m': 20, 'safety_margin_m': 0.2}`.

- V3: C1: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0
- V5: C1: native=OK, common=OK, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0
- constrained_V5: C1: native=fit_not_accepted, common=insufficient_lateral_margin, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0

### aut_s40_10

[Three-method PNG](aut_s40_10.png)

Geometry `8e86e7defb3729fbb89f5dd2bd5ead31e3c57428b31d7f0530b4e2efff9e673d`, start `{'x': 7.2215705694352526, 'y': -16.573613754630376, 's': 40.00000000000019}`, configuration `{'name': 'aut_s40_10', 'map_name': 'aut', 'source_s': 40, 'lookahead_m': 10, 'safety_margin_m': 0.2}`.

- V3: C1: native=singular_frenet, common=singular_frenet, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0
- V5: C1: native=singular_frenet, common=segment_occupied, failed segments=2; C2: native=singular_frenet, common=segment_occupied, failed segments=2; C3: native=singular_frenet, common=segment_occupied, failed segments=2; C4: native=singular_frenet, common=singular_frenet, failed segments=0; C5: native=OK, common=OK, failed segments=0
- constrained_V5: C1: native=fit_not_accepted, common=not_generated, failed segments=0; C2: native=fit_not_accepted, common=not_generated, failed segments=0; C3: native=fit_not_accepted, common=not_generated, failed segments=0; C4: native=fit_not_accepted, common=not_generated, failed segments=0; C5: native=fit_not_accepted, common=not_generated, failed segments=0

### aut_s80_15

PNG not included in this branch (numerical results retained below).

Geometry `8e86e7defb3729fbb89f5dd2bd5ead31e3c57428b31d7f0530b4e2efff9e673d`, start `{'x': -4.059317011626425, 'y': -6.37068608017105, 's': 80.0}`, configuration `{'name': 'aut_s80_15', 'map_name': 'aut', 'source_s': 80, 'lookahead_m': 15, 'safety_margin_m': 0.2}`.

- V3: C1: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0
- V5: C1: native=OK, common=OK, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0
- constrained_V5: C1: native=OK, common=OK, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0

### my_map_s4_3

PNG not included in this branch (numerical results retained below).

Geometry `3257b83623ab4b87ed906687e624e365f4c09339b7e8fd280e4dfb1e2a387085`, start `{'x': 1.8603429942045373, 'y': 0.2011446575296826, 's': 4.0}`, configuration `{'name': 'my_map_s4_3', 'map_name': 'my_map', 'source_s': 4, 'lookahead_m': 3, 'safety_margin_m': 0.2}`.

- V3: C1: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C2: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C5: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0
- V5: C1: native=OK, common=OK, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0
- constrained_V5: C1: native=fit_not_accepted, common=not_generated, failed segments=0; C2: native=fit_not_accepted, common=not_generated, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=fit_not_accepted, common=not_generated, failed segments=0

### my_map_s6_5

PNG not included in this branch (numerical results retained below).

Geometry `3257b83623ab4b87ed906687e624e365f4c09339b7e8fd280e4dfb1e2a387085`, start `{'x': -0.1024669679518203, 'y': 0.07517835278010714, 's': 6.0}`, configuration `{'name': 'my_map_s6_5', 'map_name': 'my_map', 'source_s': 6, 'lookahead_m': 5, 'safety_margin_m': 0.2}`.

- V3: C1: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0
- V5: C1: native=OK, common=OK, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0
- constrained_V5: C1: native=fit_not_accepted, common=not_generated, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0

### narrow_0.6

PNG not included in this branch (numerical results retained below).

Geometry `comparison-narrow-d2885d46802a6f90`, start `{'x': 1.5, 'y': 3.0, 's': 0.5000000000000003}`, configuration `{'name': 'narrow_0.6', 'width_m': 0.6, 'xy': [1.5, 3], 'lookahead_m': 4.0, 'safety_margin_m': 0.2}`.

- V3: C1: native=occupied, common=segment_occupied, failed segments=22; C2: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=12; C3: native=OK, common=OK, failed segments=0; C4: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=12; C5: native=occupied, common=segment_occupied, failed segments=22
- V5: C1: native=OK, common=segment_occupied, failed segments=2; C2: native=OK, common=segment_occupied, failed segments=2; C3: native=OK, common=segment_occupied, failed segments=2; C4: native=OK, common=segment_occupied, failed segments=2; C5: native=OK, common=segment_occupied, failed segments=2
- constrained_V5: C1: native=fit_not_accepted, common=not_generated, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=fit_not_accepted, common=not_generated, failed segments=0

### narrow_1.0

[Three-method PNG](narrow_1.0.png)

Geometry `comparison-narrow-5f67a55e068c4a96`, start `{'x': 1.5, 'y': 3.0, 's': 0.5000000000000003}`, configuration `{'name': 'narrow_1.0', 'width_m': 1.0, 'xy': [1.5, 3], 'lookahead_m': 4.0, 'safety_margin_m': 0.2}`.

- V3: C1: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=18; C2: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=0; C5: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=18
- V5: C1: native=OK, common=OK, failed segments=0; C2: native=OK, common=segment_occupied, failed segments=2; C3: native=OK, common=segment_occupied, failed segments=2; C4: native=OK, common=segment_occupied, failed segments=2; C5: native=OK, common=segment_occupied, failed segments=2
- constrained_V5: C1: native=OK, common=OK, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0

### narrow_1.4

PNG not included in this branch (numerical results retained below).

Geometry `comparison-narrow-7cdb02e1c5c50d12`, start `{'x': 1.5, 'y': 3.0, 's': 0.5000000000000003}`, configuration `{'name': 'narrow_1.4', 'width_m': 1.4, 'xy': [1.5, 3], 'lookahead_m': 4.0, 'safety_margin_m': 0.2}`.

- V3: C1: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=5; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=insufficient_lateral_margin, common=insufficient_lateral_margin, failed segments=5
- V5: C1: native=OK, common=OK, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=segment_occupied, failed segments=2; C5: native=OK, common=segment_occupied, failed segments=2
- constrained_V5: C1: native=OK, common=OK, failed segments=0; C2: native=OK, common=OK, failed segments=0; C3: native=OK, common=OK, failed segments=0; C4: native=OK, common=OK, failed segments=0; C5: native=OK, common=OK, failed segments=0

## Findings

- V3 is smooth in Frenet d but a fixed terminal target does not adapt to interior narrowing; lateral-margin and segment failures occur.
- V5 can keep sampled road margins while producing abrupt lateral movement and occupied-cell segment crossings. More accepted candidates do not imply better smoothness.
- Constrained V5 improves narrowing smoothness and segment clearance, but some AUT starts/horizons produce no accepted fit. fit_not_accepted may mean local solver failure or incompatible sampled constraints, not proof of global infeasibility.
- AUT_30 has one V5 diagnostic shape passing common checks while native V5 still rejects all five; the common margin tolerance differs from strict native floating-point margin comparisons. Do not promote rejected shapes to plans.
- Increasing density exposes additional jumps, geometry ambiguity and occupancy crossings; density is still not continuous-curve or swept-body certification.

## Reproduce

```bash
source .venv/bin/activate
OPENBLAS_NUM_THREADS=1 MPLBACKEND=Agg python -B -m f1tenth_benchmarks.research.overtaking.compare_candidate_methods --output-dir /tmp/new-comparison --common-step 0.025
```

The output directory must not already exist. Use --cases with names from the table for a subset. summary.csv has 36 method/case rows; candidates.csv has 180 candidate rows. Per-case JSON includes all common samples and segment failure locations; per-case PNG shows all three methods.

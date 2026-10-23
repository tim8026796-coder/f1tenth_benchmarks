# Test Summary

| Module | Test Cases | Result |
|--------|-----------:|--------|
| Prediction V1 | 53 | Passed |
| Collision V1 | 49 | Passed |
| Total | 102 | 102 Passed |

Source: the existing dedicated verification run on 2026-10-23, Python 3.9.25: `102 passed in 10.99s`.
These are offline synthetic and interface tests, not ESP two-vehicle integration tests or a full-project regression run.
The separately reproduced endpoint floating-point metadata mismatch is not covered by these 102 tests; see README.

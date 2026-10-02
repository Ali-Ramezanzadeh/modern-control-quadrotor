# Verification record

Repository preparation checked the source structure, documentation links, original-file integrity and an independently reconstructed hover model. It did not run MATLAB or Simulink.

## Original file integrity

Thirteen supplied files were copied into the repository: three Live Scripts, three Simulink models, three project reports, three assignment PDFs, and the original presentation link file. Source and destination SHA-256 hashes match. The [original-file manifest](../verification/original-file-manifest.json) records the mappings and hashes.

The RAR containers themselves are not duplicated; their extracted submitted files are included. Original PDFs retain all pages. The embedded figure galleries add extracted images without changing those PDFs.

Plain `.m` exports retain nonempty code-cell text from each `.mlx` file's `matlab/document.xml`; prose is converted to comments and whitespace-only cells are omitted. The per-phase `EXPORT_PROVENANCE.json` records the source hash, exported cell count and each code-cell hash. Saved Live Editor outputs remain in the original `.mlx`; they are not imported into the code export.

## Independent hover-model checks

The [Python check](../verification/check_model.py) reconstructs the hover matrices from the supplied state equations and parameters. Gaussian elimination with Python rational fractions checks exact ranks, avoiding dependence on floating-point rank tolerance.

| Quantity | Phase 1 | Phase 2 | Phase 3 |
| --- | ---: | ---: | ---: |
| Plant order | 12 | 12 | 12 |
| Controllability rank, exact | 12 | 12 | 12 |
| Observability rank, exact | 12 | 12 | 12 |
| `A^4 = 0` | Yes | Yes | Yes |
| Rank of `A^3`, exact | 2 | 2 | 2 |
| Integral-augmented controllability rank, exact | Not used | 16/16 | 16/16 |

The transfer matrices are also reconstructed from `C*A^k*B`. See [saved check results](../verification/hover-check-results.json). Since `A` is nilpotent and has nonzero higher powers, all eigenvalues are zero but the free response need not remain bounded.

Run this optional check from `verification/` with Python 3 and NumPy:

```text
python check_model.py
```

It writes `hover-check-results.json` next to the script. This confirms algebra for the reconstructed model, not correct Simulink wiring, MATLAB execution, controller gains, saved plots, noise experiments or 3D visualization.

## Model inventory

[model-inventory.json](../verification/model-inventory.json) lists blocks and selected saved parameters read from each `.slx` archive. It records diagram files and explicit block comments. Blocks inside a commented parent subsystem can also be inactive; the inventory does not infer complete execution paths or inherited library defaults.

The final model includes a commented-out Kalman Filter block, Rate Transition/Zero-Order Hold blocks and a UAV visualization branch. Their presence is documented separately from validated project results.

## Remaining verification

Successful MATLAB runs, complete dependency resolution, model solver behavior, active manual-switch configurations, quantitative tracking/estimation metrics and agreement with every report figure remain unverified. [The reproducibility guide](REPRODUCIBILITY.md) records specific issues already visible in the sources.

# Third-party material

The upstream projects themselves are not in this repository. The README's setup clones them into `third_party/`. This page lists the committed files that are derived from upstream work, the licence that applies to each, and the copy of that licence in `licenses/`.

| Files | Derived from | Licence |
|---|---|---|
| `configs/ur5e_shared*.yml`, `configs/ur5e_rail_shared*.yml`, `configs/v2/ur5e_rail_v{1,2}.yml`; the sphere sets in `configs/ur5e*_spherized.urdf`, `configs/ur5e*.srdf`, `configs/ur5e*.hh`, `configs/ur5e*_cricket.json` and `configs/v2/ur5e_rail.urdf` | cuRobo's robot configs (NVlabs/curobo, `ur5e.yml`), including the sphere sets of the paper's configs, which were built from that file | NVIDIA License, non-commercial (research or evaluation) use only: `licenses/NVIDIA_cuRobo_LICENSE.txt` |
| `configs/panda_vamp_spheres*.yml`, `configs/ur5_vamp_spheres*.yml`, `configs/ur5_spherized_vel_pi.urdf`, `configs/v2/panda.urdf`, `configs/v2/panda_v{1,2}.yml`, `problems/v2_mbm_panda.pkl` | VAMP (KavrakiLab/vamp): robot resources and MotionBenchMaker problems | Apache 2.0: `licenses/VAMP_Apache-2.0_LICENSE.txt` |
| `problems/v2_mpinets_panda.pkl` | robometrics (`mpinets_set.yaml`) | MIT: `licenses/robometrics_MIT_LICENSE.txt` |
| `configs/ur5e.hh`, `configs/ur5e_rail.hh` (generated code) | Cricket (CoMMALab/cricket) templates | MIT: `licenses/Cricket_MIT_LICENSE.txt` |
| `docs/arxiv_2508.04146_v1.txt` | arXiv 2508.04146 (L. Abuelsamen, H. Rana, H.-W. Lu, W. Tang), text extracted from the PDF | CC BY 4.0 |

The UR5e kinematics in the URDFs come from Universal Robots' description files, by way of cuRobo's assets.

The cuRobo-derived YAML files carry NVIDIA's notice as a header comment. `bench/v2_curobo_configs.py` and `bench/curobo_to_spherized.py` regenerate those files without the header, so re-add it if you regenerate them.

No Universal Robots software is included. The URSim and PolyScope X simulator images mentioned in `docs/V2_2_MOTION_TYPES_PREREGISTRATION.md` are not part of this repository.

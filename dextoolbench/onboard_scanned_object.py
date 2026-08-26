"""Onboard a scanned any2any object into the DexToolBench asset layout.

Steps performed (the HANDOVER_V3 per-object recipe, automated):
1. Copy the scan mesh into assets/urdf/dextoolbench/<category>/<name>/<name>.obj,
   rotating to z-up if the scan is y-up (vertex/normal lines are transformed in
   place so textures/mtl references survive).
2. Write <name>.urdf (visual + collision on the canonical mesh, density field).
3. Emit deployment/<name>_reframe.json describing the applied rotation so the
   perception relay can map FP poses (tracked in the SCAN frame) into the
   canonical sim frame. Identity rotation -> identity reframe.
4. Print the objects.py registration snippet (grasp-box scale convention:
   graspable-region dims in meters; the registry multiplies by 25).

After onboarding + registration, run:
   python dextoolbench/generate_collision_meshes.py --object_name <name>
to produce the _decomposed.urdf used for training.

Usage example:
   python dextoolbench/onboard_scanned_object.py \
     --scene_dir 0818_task2_traj_tracking/assets/scene1 \
     --name yoga_can --category can --up_axis y \
     --grasp_box 0.062 0.121 0.063
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np

REPO = Path(__file__).parent.parent

ROT_Y_UP_TO_Z_UP = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ]
)  # +90 deg about x: scan +y (up) -> sim +z (up)


def transform_obj_file(src: Path, dst: Path, R: np.ndarray) -> None:
    """Rewrite an .obj applying rotation R to v/vn lines, preserving all other
    content (faces, vt, mtllib/usemtl) so textures keep working."""
    out_lines = []
    for line in src.read_text().splitlines():
        parts = line.split()
        if parts and parts[0] in ("v", "vn"):
            vec = R @ np.array([float(parts[1]), float(parts[2]), float(parts[3])])
            extra = parts[4:]  # vertex colors, if present
            out_lines.append(
                f"{parts[0]} {vec[0]:.6f} {vec[1]:.6f} {vec[2]:.6f}"
                + ("" if not extra else " " + " ".join(extra))
            )
        else:
            out_lines.append(line)
    dst.write_text("\n".join(out_lines) + "\n")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--scene_dir", required=True)
    p.add_argument("--name", required=True)
    p.add_argument("--category", required=True)
    p.add_argument("--up_axis", choices=["y", "z"], required=True,
                   help="up axis of the SCAN; y triggers a +90deg-about-x reframe")
    p.add_argument("--grasp_box", type=float, nargs=3, required=True,
                   help="graspable-region dims [m] in the CANONICAL (z-up) frame")
    p.add_argument("--density", type=float, default=450.0,
                   help="base URDF density (overridden per-variant when "
                        "objectDensityRandomization is on)")
    args = p.parse_args()

    scene_dir = Path(args.scene_dir)
    meshes = sorted(scene_dir.glob("*.obj"))
    assert len(meshes) == 1, f"expected exactly one .obj in {scene_dir}: {meshes}"
    src_mesh = meshes[0]

    out_dir = REPO / "assets/urdf/dextoolbench" / args.category / args.name
    out_dir.mkdir(parents=True, exist_ok=True)
    out_mesh = out_dir / f"{args.name}.obj"

    R = ROT_Y_UP_TO_Z_UP if args.up_axis == "y" else np.eye(3)
    transform_obj_file(src_mesh, out_mesh, R)

    # sidecar files (textures/materials); keep original names so mtllib refs work
    for f in scene_dir.iterdir():
        if f.suffix.lower() in (".mtl", ".png", ".jpg", ".jpeg"):
            shutil.copy2(f, out_dir / f.name)

    urdf = f"""<?xml version="1.0"?>
<!-- Onboarded any2any scan ({src_mesh}). Canonical frame: z-up
     ({'rotated +90deg about x from the y-up scan' if args.up_axis == 'y' else 'scan frame kept as-is'}).
     Density: uniform-sampled per training variant (simtoolreal primitive
     convention, LOW range 300-600 kg/m^3); the value below is the base. -->
<robot name="{args.name}">
  <link name="{args.name}">
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{args.name}.obj" scale="1 1 1"/></geometry></visual>
    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{args.name}.obj" scale="1 1 1"/></geometry></collision>
    <inertial><density value="{args.density}"/></inertial>
  </link>
</robot>
"""
    (out_dir / f"{args.name}.urdf").write_text(urdf)

    # Reframe json for the deployment relay: v_canonical = R @ v_scan.
    T = np.eye(4)
    T[:3, :3] = R
    reframe = {
        "T_mesh_canonical": T.tolist(),
        "note": (
            f"v_canonical = R @ v_scan (up_axis={args.up_axis}). Identity if the "
            "scan frame was kept. Hardware side: validate direction with the "
            "overlay QA tool before live use, same as hammer_002_scanned."
        ),
    }
    reframe_path = REPO / "deployment" / f"{args.name}_reframe.json"
    reframe_path.write_text(json.dumps(reframe, indent=1))

    gx, gy, gz = args.grasp_box
    print(f"onboarded {args.name} -> {out_dir}")
    print(f"reframe json -> {reframe_path}")
    print("objects.py snippet:")
    print(f'''    "{args.name}": Object(
        urdf_path=(
            get_repo_root_dir()
            / "assets/urdf/dextoolbench/{args.category}/{args.name}/{args.name}.urdf"
        ),
        scale=rescale_by_factor(({gx}, {gy}, {gz}), factor=25),
        need_vhacd=False,
    ),''')


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Synthetic-cloud tests for detection and grasp staging. No hardware."""
import math
import numpy as np
import pytest

from grasp_pipeline import (Detection, GraspProfile, colour_mask, detect_object,
                            grasp_stages)

RED = (220.0, 40.0, 35.0)
GREY = (120.0, 120.0, 120.0)


def box_cloud(centre, size, yaw, count=4000, seed=0):
    """Points filling a yaw-rotated box, expressed in base_link."""
    rng = np.random.default_rng(seed)
    local = (rng.random((count, 3)) - 0.5) * np.asarray(size)
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return (rotation @ local.T).T + np.asarray(centre)


def coloured(points, colour):
    return np.tile(np.asarray(colour, dtype=float), (len(points), 1))


def top_down_matrix(yaw):
    """Mirror of the gateway helper, so the test pins the shared convention."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[0.0, -s, c], [0.0, c, s], [-1.0, 0.0, 0.0]])


def test_colour_mask_accepts_both_colour_scales():
    profile = GraspProfile(name="t")
    as_255 = np.array([RED, GREY])
    assert colour_mask(as_255, profile).tolist() == [True, False]
    assert colour_mask(as_255 / 255.0, profile).tolist() == [True, False]


def test_colour_mask_handles_a_blue_channel_rule():
    blue = GraspProfile(name="b", channel="blue", min_value=80.0,
                        ratios={"red": 1.30, "green": 1.15}, caps={})
    samples = np.array([(40.0, 90.0, 160.0),    # blue object
                        (220.0, 40.0, 35.0),    # red object
                        (120.0, 120.0, 120.0),  # grey
                        (10.0, 10.0, 60.0)])    # dark shadow, below min_value
    assert colour_mask(samples, blue).tolist() == [True, False, False, False]


def test_colour_rule_validation_is_fail_closed():
    with pytest.raises(ValueError, match="unknown colour channel"):
        GraspProfile(name="t", channel="cyan").validate()
    with pytest.raises(ValueError, match="unknown colour channel"):
        GraspProfile(name="t", ratios={"cyan": 1.2}).validate()
    with pytest.raises(ValueError, match="against itself"):
        GraspProfile(name="t", channel="red", ratios={"red": 1.2}).validate()
    with pytest.raises(ValueError, match="ratios must be positive"):
        GraspProfile(name="t", ratios={"green": 0.0}).validate()


def test_load_profile_reads_the_blue_tape_measure():
    from grasp_pipeline import load_profile
    path = ("/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/"
            "object_profiles/blue_tape_measure.yaml")
    profile, motion_allowed = load_profile(path)
    assert profile.name == "blue_tape_measure"
    assert profile.channel == "blue" and profile.caps == {}
    # Tuned against live frame seq=58758; these ratios isolate a single cluster.
    assert profile.min_value == 60.0
    assert profile.ratios == {"red": 2.00, "green": 1.80}
    # Ruler-verified on 2026-09-10 and released for motion; the profile records
    # the measurement itself under ruler_verification.
    assert motion_allowed is True


def test_detects_a_blue_object_ignoring_a_red_one():
    blue = GraspProfile(name="b", channel="blue", min_value=70.0,
                        ratios={"red": 1.30, "green": 1.12}, caps={})
    target = box_cloud((0.52, 0.05, 0.05), (0.09, 0.06, 0.05), 0.0, seed=21)
    decoy = box_cloud((0.45, -0.15, 0.05), (0.10, 0.08, 0.05), 0.0, seed=22)
    points = np.vstack([target, decoy])
    colours = np.vstack([coloured(target, (40.0, 90.0, 170.0)),
                         coloured(decoy, RED)])
    result = detect_object(points, colours, blue)
    assert result.valid, result.reason
    assert np.allclose(result.centroid_m[:2], (0.52, 0.05), atol=0.006)


def test_detects_centroid_and_extent():
    profile = GraspProfile(name="t")
    size = (0.12, 0.05, 0.06)
    centre = (0.50, 0.10, 0.06)
    points = box_cloud(centre, size, yaw=0.0)
    result = detect_object(points, coloured(points, RED), profile)
    assert result.valid, result.reason
    assert np.allclose(result.centroid_m, centre, atol=0.004)
    assert result.extent_m[0] == pytest.approx(size[0], abs=0.006)
    assert result.extent_m[1] == pytest.approx(size[1], abs=0.006)
    assert result.top_z_m == pytest.approx(centre[2] + size[2] / 2, abs=0.004)


@pytest.mark.parametrize("yaw", [-1.2, -0.6, 0.0, 0.35, 0.9, 1.4])
def test_fingers_close_across_the_narrow_side(yaw):
    """The whole point of the yaw convention, checked end to end."""
    profile = GraspProfile(name="t")
    points = box_cloud((0.52, 0.05, 0.06), (0.14, 0.045, 0.05), yaw, seed=3)
    result = detect_object(points, coloured(points, RED), profile)
    assert result.valid, result.reason

    long_axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    short_axis = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    separation = top_down_matrix(result.yaw_rad) @ np.array([0.0, 1.0, 0.0])

    # The fingers must lie along the short axis, not the long one.
    assert abs(float(np.dot(separation, short_axis))) > 0.99
    assert abs(float(np.dot(separation, long_axis))) < 0.02
    # And the approach must still point straight down.
    approach = top_down_matrix(result.yaw_rad) @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(approach, [0.0, 0.0, -1.0], atol=1e-9)


def test_ignores_out_of_workspace_and_non_red_points():
    profile = GraspProfile(name="t")
    target = box_cloud((0.50, 0.00, 0.06), (0.08, 0.05, 0.05), 0.0, seed=1)
    grey = box_cloud((0.55, 0.20, 0.06), (0.10, 0.10, 0.05), 0.0, seed=2)
    far = box_cloud((1.50, 0.00, 0.06), (0.10, 0.10, 0.05), 0.0, seed=4)
    points = np.vstack([target, grey, far])
    colours = np.vstack([coloured(target, RED), coloured(grey, GREY),
                         coloured(far, RED)])
    result = detect_object(points, colours, profile)
    assert result.valid, result.reason
    assert np.allclose(result.centroid_m, (0.50, 0.00, 0.06), atol=0.005)


def test_picks_the_largest_red_cluster():
    profile = GraspProfile(name="t")
    big = box_cloud((0.50, -0.10, 0.06), (0.10, 0.06, 0.05), 0.0, count=3000, seed=5)
    small = box_cloud((0.60, 0.20, 0.06), (0.04, 0.04, 0.04), 0.0, count=300, seed=6)
    points = np.vstack([big, small])
    result = detect_object(points, coloured(points, RED), profile)
    assert result.valid, result.reason
    assert result.candidate_clusters >= 2
    assert np.allclose(result.centroid_m[:2], (0.50, -0.10), atol=0.006)


def test_rejects_sparse_and_malformed_input():
    profile = GraspProfile(name="t")
    assert not detect_object(np.zeros((0, 3)), np.zeros((0, 3)), profile).valid
    assert not detect_object(np.zeros((5, 2)), np.zeros((5, 3)), profile).valid
    assert not detect_object(np.zeros((5, 3)), np.zeros((4, 3)), profile).valid
    few = box_cloud((0.50, 0.0, 0.06), (0.03, 0.03, 0.03), 0.0, count=10, seed=7)
    assert not detect_object(few, coloured(few, RED), profile).valid


def test_non_finite_points_are_dropped():
    profile = GraspProfile(name="t")
    points = box_cloud((0.50, 0.0, 0.06), (0.08, 0.05, 0.05), 0.0, seed=8)
    points = np.vstack([points, [[np.nan, 0.0, 0.0], [np.inf, 1.0, 1.0]]])
    colours = coloured(points, RED)
    result = detect_object(points, colours, profile)
    assert result.valid, result.reason
    assert np.isfinite(result.centroid_m).all()


def test_grasp_stages_geometry():
    profile = GraspProfile(name="t")
    points = box_cloud((0.50, 0.05, 0.06), (0.12, 0.05, 0.06), 0.0, seed=9)
    detection = detect_object(points, coloured(points, RED), profile)
    stages = grasp_stages(detection, profile)
    assert stages.valid, stages.reason
    # Approach is directly above the grasp, lift is directly above it too.
    assert np.allclose(stages.pre_grasp_m[:2], stages.grasp_m[:2])
    assert np.allclose(stages.lift_m[:2], stages.grasp_m[:2])
    assert stages.pre_grasp_m[2] == pytest.approx(
        stages.grasp_m[2] + profile.approach_height_m)
    assert stages.lift_m[2] == pytest.approx(
        stages.grasp_m[2] + profile.lift_height_m)
    assert stages.grasp_m[2] == pytest.approx(
        detection.top_z_m - profile.grasp_depth_below_top_m, abs=1e-9)
    assert stages.gripper_open_m == pytest.approx(
        detection.width_across_fingers_m + profile.gripper_clearance_m)


def test_grasp_rejects_object_wider_than_the_stroke():
    profile = GraspProfile(name="t")
    points = box_cloud((0.50, 0.0, 0.06), (0.20, 0.16, 0.05), 0.0, seed=10)
    detection = detect_object(points, coloured(points, RED), profile)
    assert detection.valid, detection.reason
    stages = grasp_stages(detection, profile)
    assert not stages.valid
    assert "stroke" in stages.reason


def test_grasp_rejects_when_lift_leaves_the_workspace():
    profile = GraspProfile(name="t", lift_height_m=0.90)
    points = box_cloud((0.50, 0.0, 0.06), (0.08, 0.05, 0.05), 0.0, seed=11)
    detection = detect_object(points, coloured(points, RED), profile)
    stages = grasp_stages(detection, profile)
    assert not stages.valid
    assert "workspace" in stages.reason


def test_grasp_floor_raises_a_low_grasp_and_notes_it():
    profile = GraspProfile(name="t", grasp_depth_below_top_m=0.20)
    points = box_cloud((0.50, 0.0, 0.06), (0.08, 0.05, 0.05), 0.0, seed=12)
    detection = detect_object(points, coloured(points, RED), profile)
    stages = grasp_stages(detection, profile)
    assert stages.valid, stages.reason
    assert stages.grasp_m[2] == pytest.approx(profile.min_grasp_z_m)
    assert stages.notes and "raised" in stages.notes[0]


def test_invalid_detection_never_produces_stages():
    profile = GraspProfile(name="t")
    stages = grasp_stages(Detection(False, "no object"), profile)
    assert not stages.valid and stages.grasp_m is None


def test_profile_validation_is_fail_closed():
    with pytest.raises(ValueError):
        GraspProfile(name="t", x_m=(0.8, 0.3)).validate()
    with pytest.raises(ValueError):
        GraspProfile(name="t", voxel_m=0.0).validate()
    with pytest.raises(ValueError):
        GraspProfile(name="t", gripper_max_width_m=-1.0).validate()


def test_load_profile_reads_the_reviewed_tape_dispenser():
    from grasp_pipeline import load_profile
    path = ("/home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/"
            "object_profiles/red_tape_dispenser.yaml")
    profile, motion_allowed = load_profile(path)
    assert profile.name == "red_tape_dispenser"
    assert profile.channel == "red" and profile.min_value == 130.0
    assert profile.ratios == {"green": 1.45, "blue": 1.45}
    assert profile.x_m == (0.30, 0.80) and profile.z_m == (0.010, 0.60)
    assert profile.approach_height_m == 0.100
    assert profile.min_points == 40 and isinstance(profile.min_points, int)
    # The profile is still awaiting ruler verification.
    assert motion_allowed is False


def test_load_profile_rejects_unknown_keys(tmp_path):
    from grasp_pipeline import load_profile
    bad = tmp_path / "bad.yaml"
    bad.write_text("name: x\ndetection:\n  not_a_field: 1\n")
    with pytest.raises(ValueError, match="unknown detection key"):
        load_profile(str(bad))


def test_load_profile_rejects_unnamed_file(tmp_path):
    from grasp_pipeline import load_profile
    bad = tmp_path / "bad.yaml"
    bad.write_text("detection:\n  red_min: 100\n")
    with pytest.raises(ValueError, match="named object profile"):
        load_profile(str(bad))


def test_detect_objects_finds_every_object_not_just_the_largest():
    """The whole point: two same-coloured objects must both be reported."""
    from grasp_pipeline import detect_objects
    profile = GraspProfile(name="t")
    a = box_cloud((0.45, -0.12, 0.06), (0.10, 0.06, 0.05), 0.0, count=3000, seed=30)
    b = box_cloud((0.62, 0.18, 0.06), (0.08, 0.05, 0.05), 0.0, count=1500, seed=31)
    points = np.vstack([a, b])
    found, reason = detect_objects(points, coloured(points, RED), profile)
    assert reason is None
    assert len(found) == 2, [d.reason for d in found]
    # Largest first.
    assert found[0].point_count >= found[1].point_count
    centres = sorted(tuple(np.round(d.centroid_m[:2], 2)) for d in found)
    assert centres == [(0.45, -0.12), (0.62, 0.18)]
    # Every detection reports the same total cluster count.
    assert all(d.candidate_clusters >= 2 for d in found)


def test_detect_object_still_returns_the_largest_of_several():
    from grasp_pipeline import detect_objects
    profile = GraspProfile(name="t")
    a = box_cloud((0.45, -0.12, 0.06), (0.10, 0.06, 0.05), 0.0, count=3000, seed=32)
    b = box_cloud((0.62, 0.18, 0.06), (0.08, 0.05, 0.05), 0.0, count=800, seed=33)
    points = np.vstack([a, b])
    colours = coloured(points, RED)
    single = detect_object(points, colours, profile)
    first = detect_objects(points, colours, profile)[0][0]
    assert single.valid
    assert np.allclose(single.centroid_m, first.centroid_m)
    assert np.allclose(single.centroid_m[:2], (0.45, -0.12), atol=0.006)


def test_detect_objects_drops_clusters_below_min_points():
    from grasp_pipeline import detect_objects
    profile = GraspProfile(name="t", min_points=200)
    big = box_cloud((0.45, 0.0, 0.06), (0.10, 0.06, 0.05), 0.0, count=3000, seed=34)
    speckle = box_cloud((0.70, 0.25, 0.06), (0.02, 0.02, 0.02), 0.0, count=30, seed=35)
    points = np.vstack([big, speckle])
    found, reason = detect_objects(points, coloured(points, RED), profile)
    assert reason is None
    assert len(found) == 1
    assert np.allclose(found[0].centroid_m[:2], (0.45, 0.0), atol=0.006)


def test_detect_objects_respects_max_objects():
    from grasp_pipeline import detect_objects
    profile = GraspProfile(name="t")
    blocks = [box_cloud((0.40 + 0.08 * i, -0.20 + 0.10 * i, 0.06),
                        (0.04, 0.04, 0.04), 0.0, count=1200, seed=40 + i)
              for i in range(4)]
    points = np.vstack(blocks)
    found, _ = detect_objects(points, coloured(points, RED), profile, max_objects=2)
    assert len(found) == 2
    assert found[0].candidate_clusters >= 4


def test_detect_objects_reports_a_reason_when_nothing_matches():
    from grasp_pipeline import detect_objects
    profile = GraspProfile(name="t")
    grey = box_cloud((0.50, 0.0, 0.06), (0.10, 0.06, 0.05), 0.0, seed=36)
    found, reason = detect_objects(grey, coloured(grey, GREY), profile)
    assert found == []
    assert reason and "in-workspace" in reason


def test_each_detection_yields_its_own_grasp_stages():
    from grasp_pipeline import detect_objects
    profile = GraspProfile(name="t")
    a = box_cloud((0.45, -0.12, 0.06), (0.10, 0.045, 0.05), 0.0, count=3000, seed=37)
    b = box_cloud((0.62, 0.18, 0.06), (0.08, 0.040, 0.05), 0.0, count=1500, seed=38)
    points = np.vstack([a, b])
    found, _ = detect_objects(points, coloured(points, RED), profile)
    assert len(found) == 2
    stages = [grasp_stages(d, profile) for d in found]
    assert all(s.valid for s in stages), [s.reason for s in stages]
    # The two grasps are at different places.
    assert not np.allclose(stages[0].grasp_m, stages[1].grasp_m)

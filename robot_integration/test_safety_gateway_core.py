#!/usr/bin/env python3
"""Fault matrix for the fail-closed motion command gateway core.

Every test asserts that a rejected intent leaves the sink untouched, so no test
can pass by accident through a path that still forwards a bad command.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_integration.safety_gateway_core import (  # noqa: E402
    GatewayConfig,
    GatewayRejection,
    MotionIntent,
    SafetyGatewayCore,
)

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "config", "safety", "mh5_gateway_core_v0.yaml",
)

ORDER = ("joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t")
ZERO = (0.0,) * 6


class Sink:
    def __init__(self):
        self.calls = []

    def __call__(self, positions):
        self.calls.append(tuple(positions))


def make_config():
    return GatewayConfig.load(CONFIG_PATH)


def make_gateway(preconditions_met=True, workspace=True, collision=True, sink=None):
    config = make_config()
    if preconditions_met:
        config.preconditions = {k: True for k in config.preconditions}
    sink = sink if sink is not None else Sink()
    gateway = SafetyGatewayCore(
        config,
        sink=sink,
        workspace_check=(lambda p: workspace) if workspace is not None else None,
        collision_check=(lambda p: collision) if collision is not None else None,
    )
    return gateway, sink


def intent(positions=ZERO, names=ORDER, unit="radian", stamp=100.0, sequence=1):
    return MotionIntent(joint_names=tuple(names), positions_rad=tuple(positions),
                        unit=unit, stamp_s=stamp, sequence=sequence)


def armed_gateway(**kwargs):
    gateway, sink = make_gateway(**kwargs)
    gateway.arm(now_s=100.0, operator_ack=True)
    return gateway, sink


# -- configuration -----------------------------------------------------------

def test_config_loads_six_joints_in_canonical_order():
    config = make_config()
    assert config.canonical_joint_order == ORDER
    assert config.unit == "radian"


def test_config_refuses_persistent_arming():
    config = make_config()
    assert config.arm_persists_across_restart is False


def test_shipped_config_declares_verification_incomplete():
    """The shipped config must not claim the jog verification has been done."""
    config = make_config()
    assert config.preconditions["joint_sign_order_units_verified"] is False


# -- arming ------------------------------------------------------------------

def test_starts_disarmed():
    gateway, _ = make_gateway()
    assert gateway.armed is False


def test_disarmed_gateway_rejects_and_does_not_forward():
    gateway, sink = make_gateway()
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(), now_s=100.0)
    assert exc.value.code == "DISARMED"
    assert sink.calls == []


def test_arming_requires_operator_ack():
    gateway, _ = make_gateway()
    with pytest.raises(GatewayRejection) as exc:
        gateway.arm(now_s=100.0, operator_ack=False)
    assert exc.value.code == "ARM_REQUIRES_OPERATOR_ACK"
    assert gateway.armed is False


def test_arming_blocked_while_preconditions_unmet():
    gateway, _ = make_gateway(preconditions_met=False)
    with pytest.raises(GatewayRejection) as exc:
        gateway.arm(now_s=100.0, operator_ack=True)
    assert exc.value.code == "ARM_PRECONDITION_UNMET"
    assert "joint_sign_order_units_verified" in exc.value.detail
    assert gateway.armed is False


def test_arming_blocked_without_workspace_check():
    gateway, _ = make_gateway(workspace=None)
    with pytest.raises(GatewayRejection) as exc:
        gateway.arm(now_s=100.0, operator_ack=True)
    assert exc.value.code == "ARM_NO_WORKSPACE_CHECK"


def test_arming_blocked_without_collision_check():
    gateway, _ = make_gateway(collision=None)
    with pytest.raises(GatewayRejection) as exc:
        gateway.arm(now_s=100.0, operator_ack=True)
    assert exc.value.code == "ARM_NO_COLLISION_CHECK"


# -- happy path --------------------------------------------------------------

def test_valid_intent_is_forwarded_once():
    gateway, sink = armed_gateway()
    result = gateway.submit(intent(), now_s=100.0)
    assert result == ZERO
    assert sink.calls == [ZERO]


def test_second_valid_intent_within_limits_is_forwarded():
    gateway, sink = armed_gateway()
    gateway.submit(intent(sequence=1, stamp=100.0), now_s=100.0)
    gateway.heartbeat(100.1)
    step = (0.005,) + (0.0,) * 5
    gateway.submit(intent(positions=step, sequence=2, stamp=100.1), now_s=100.1)
    assert len(sink.calls) == 2


# -- name, order, unit, finiteness -------------------------------------------

def test_wrong_unit_rejected():
    gateway, sink = armed_gateway()
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(unit="degree"), now_s=100.0)
    assert exc.value.code == "WRONG_UNIT"
    assert sink.calls == []


def test_reordered_joint_names_rejected():
    gateway, sink = armed_gateway()
    swapped = (ORDER[1], ORDER[0]) + ORDER[2:]
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(names=swapped), now_s=100.0)
    assert exc.value.code == "JOINT_NAME_OR_ORDER"
    assert sink.calls == []


def test_unknown_joint_name_rejected():
    gateway, sink = armed_gateway()
    renamed = ("joint_1_x",) + ORDER[1:]
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(names=renamed), now_s=100.0)
    assert exc.value.code == "JOINT_NAME_OR_ORDER"
    assert sink.calls == []


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_rejected(bad):
    gateway, sink = armed_gateway()
    positions = (bad,) + (0.0,) * 5
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(positions=positions), now_s=100.0)
    assert exc.value.code == "NON_FINITE"
    assert sink.calls == []


# -- limits ------------------------------------------------------------------

def test_position_limit_rejected():
    gateway, sink = armed_gateway()
    positions = (3.5,) + (0.0,) * 5  # joint_1_s upper is about 2.967
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(positions=positions), now_s=100.0)
    assert exc.value.code == "POSITION_LIMIT"
    assert sink.calls == []


def test_step_limit_rejected_and_previous_target_kept():
    gateway, sink = armed_gateway()
    gateway.submit(intent(sequence=1, stamp=100.0), now_s=100.0)
    gateway.heartbeat(100.1)
    big = (0.5,) + (0.0,) * 5
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(positions=big, sequence=2, stamp=100.1), now_s=100.1)
    assert exc.value.code == "STEP_LIMIT"
    assert sink.calls == [ZERO]


def test_velocity_limit_rejected():
    gateway, sink = armed_gateway()
    gateway.submit(intent(sequence=1, stamp=100.0), now_s=100.0)
    gateway.heartbeat(100.03)
    # 0.009 rad in 0.03 s is 0.30 rad/s, above the 0.10 rad/s ceiling, while the
    # step itself stays inside the 0.01 rad step limit and the 0.03 s interval
    # clears the 0.02 s rate limit.
    fast = (0.009,) + (0.0,) * 5
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(positions=fast, sequence=2, stamp=100.03), now_s=100.03)
    assert exc.value.code == "VELOCITY_LIMIT"
    assert sink.calls == [ZERO]


def test_acceleration_limit_rejected():
    gateway, sink = armed_gateway()
    gateway.submit(intent(sequence=1, stamp=100.0), now_s=100.0)
    gateway.heartbeat(100.03)
    # Hold still, so the recorded velocity is zero.
    gateway.submit(intent(positions=ZERO, sequence=2, stamp=100.03), now_s=100.03)
    gateway.heartbeat(100.06)
    # 0.0025 rad in 0.03 s is 0.083 rad/s, under the velocity ceiling, but going
    # from 0 to 0.083 rad/s in 0.03 s is about 2.8 rad/s^2, far above the
    # 0.50 rad/s^2 ceiling.
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(positions=(0.0025,) + (0.0,) * 5, sequence=3, stamp=100.06),
                       now_s=100.06)
    assert exc.value.code == "ACCELERATION_LIMIT"
    assert len(sink.calls) == 2


# -- freshness, ordering, rate ----------------------------------------------

def test_stale_timestamp_rejected():
    gateway, sink = armed_gateway()
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(stamp=99.0), now_s=100.0)
    assert exc.value.code == "STALE_TIMESTAMP"
    assert sink.calls == []


def test_future_timestamp_rejected():
    gateway, sink = armed_gateway()
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(stamp=101.0), now_s=100.0)
    assert exc.value.code == "FUTURE_TIMESTAMP"
    assert sink.calls == []


def test_replayed_sequence_rejected():
    gateway, sink = armed_gateway()
    gateway.submit(intent(sequence=5, stamp=100.0), now_s=100.0)
    gateway.heartbeat(100.1)
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(sequence=5, stamp=100.1), now_s=100.1)
    assert exc.value.code == "STALE_SEQUENCE"
    assert sink.calls == [ZERO]


def test_rate_limit_rejected():
    gateway, sink = armed_gateway()
    gateway.submit(intent(sequence=1, stamp=100.0), now_s=100.0)
    gateway.heartbeat(100.001)
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(sequence=2, stamp=100.001), now_s=100.001)
    assert exc.value.code == "RATE_LIMIT"
    assert sink.calls == [ZERO]


# -- external checks ---------------------------------------------------------

def test_workspace_rejection_blocks_command():
    gateway, sink = armed_gateway(workspace=False)
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(), now_s=100.0)
    assert exc.value.code == "WORKSPACE_REJECTED"
    assert sink.calls == []


def test_collision_rejection_blocks_command():
    gateway, sink = armed_gateway(collision=False)
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(), now_s=100.0)
    assert exc.value.code == "COLLISION_REJECTED"
    assert sink.calls == []


# -- liveness and fault reactions -------------------------------------------

def test_dead_man_expiry_disarms_and_blocks():
    gateway, sink = armed_gateway()
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(stamp=101.0), now_s=101.0)
    assert exc.value.code == "DEAD_MAN_TIMEOUT"
    assert gateway.armed is False
    assert sink.calls == []


def test_watchdog_expiry_disarms_and_does_not_replay():
    gateway, sink = armed_gateway()
    gateway.submit(intent(sequence=1, stamp=100.0), now_s=100.0)
    gateway.heartbeat(101.0)
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(positions=(0.001,) + (0.0,) * 5, sequence=2, stamp=101.0),
                       now_s=101.0)
    assert exc.value.code == "WATCHDOG_TIMEOUT"
    assert gateway.armed is False
    assert sink.calls == [ZERO]


def test_estop_disarms_immediately():
    gateway, sink = armed_gateway()
    gateway.notify_estop()
    assert gateway.armed is False
    with pytest.raises(GatewayRejection) as exc:
        gateway.submit(intent(), now_s=100.0)
    assert exc.value.code == "DISARMED"
    assert sink.calls == []


def test_robot_disable_disarms_immediately():
    gateway, sink = armed_gateway()
    gateway.notify_robot_disabled()
    assert gateway.armed is False
    assert sink.calls == []


def test_disconnect_disarms_immediately():
    gateway, sink = armed_gateway()
    gateway.notify_disconnect()
    assert gateway.armed is False
    assert sink.calls == []


def test_restart_starts_disarmed_even_after_previous_arm():
    gateway, _ = armed_gateway()
    assert gateway.armed is True
    fresh, sink = make_gateway()
    assert fresh.armed is False
    with pytest.raises(GatewayRejection):
        fresh.submit(intent(), now_s=100.0)
    assert sink.calls == []


# -- no transport ------------------------------------------------------------

def test_module_imports_no_transport():
    """Assert against the parsed syntax tree, not the prose in the docstring."""
    import ast
    import robot_integration.safety_gateway_core as module

    tree = ast.parse(open(module.__file__, "r", encoding="utf-8").read())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    banned = {"socket", "rclpy", "rospy", "subprocess", "pymodbus",
              "http", "urllib", "requests", "asyncio", "selectors"}
    assert not (imported & banned), f"gateway core imports transport: {imported & banned}"

    # No literal may name a controller port or address.
    literals = {node.value for node in ast.walk(tree)
                if isinstance(node, ast.Constant)}
    for forbidden in (50240, 50241, 50242, "192.168.0.11"):
        assert forbidden not in literals, f"gateway core must not name {forbidden}"


def test_gateway_without_sink_accepts_but_forwards_nowhere():
    config = make_config()
    config.preconditions = {k: True for k in config.preconditions}
    gateway = SafetyGatewayCore(config, sink=None,
                                workspace_check=lambda p: True,
                                collision_check=lambda p: True)
    gateway.arm(now_s=100.0, operator_ack=True)
    assert gateway.submit(intent(), now_s=100.0) == ZERO

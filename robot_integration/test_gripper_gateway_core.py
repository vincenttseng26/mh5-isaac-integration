from gripper_gateway_core import GripperGatewayCore, GripperStatus


class FakeGripper:
    def __init__(self):
        self.status = GripperStatus(800, False, False)
        self.moves = []
        self.stops = 0

    def move(self, width, force):
        self.moves.append((width, force))

    def stop(self):
        self.stops += 1


def make_core():
    fake = FakeGripper()
    return GripperGatewayCore("secret", lambda: fake.status, fake.move, fake.stop), fake


def arm(core):
    assert core.handle({"cmd": "set_mode", "mode": "COMMAND", "token": "secret"})["ok"]
    reply = core.handle({"cmd": "arm", "token": "secret"})
    assert reply["ok"]
    return reply["nonce"]


def test_defaults_to_read_only_and_reports_status():
    core, _ = make_core()
    reply = core.handle({"cmd": "status"})
    assert reply["ok"] and reply["mode"] == "MIRROR" and not reply["armed"]
    assert reply["gripper"]["width_mm"] == 80.0


def test_move_requires_token_mode_arm_and_nonce():
    core, fake = make_core()
    request = {"cmd": "move", "width_tenth_mm": 420, "force_tenth_n": 60}
    assert core.handle(request)["code"] == "BAD_TOKEN"
    request["token"] = "secret"
    assert core.handle(request)["code"] == "NOT_ARMED"
    nonce = arm(core)
    request["nonce"] = "wrong"
    assert core.handle(request)["code"] == "BAD_NONCE"
    request["nonce"] = nonce
    assert core.handle(request)["ok"]
    assert fake.moves == [(420, 60)]


def test_accepted_move_is_one_shot_and_returns_to_mirror():
    core, fake = make_core()
    nonce = arm(core)
    request = {"cmd": "move", "token": "secret", "nonce": nonce,
               "width_tenth_mm": 420, "force_tenth_n": 60}
    reply = core.handle(request)
    assert reply["ok"] and reply["mode"] == "MIRROR" and not reply["armed"]
    assert core.handle(request)["code"] == "NOT_ARMED"
    assert fake.moves == [(420, 60)]


def test_limits_fail_closed_without_writing():
    for field, value, code in (("width_tenth_mm", 1001, "BAD_WIDTH"),
                               ("width_tenth_mm", 42.0, "BAD_WIDTH"),
                               ("force_tenth_n", 29, "BAD_FORCE"),
                               ("force_tenth_n", True, "BAD_FORCE")):
        core, fake = make_core()
        nonce = arm(core)
        request = {"cmd": "move", "token": "secret", "nonce": nonce,
                   "width_tenth_mm": 420, "force_tenth_n": 60}
        request[field] = value
        assert core.handle(request)["code"] == code
        assert fake.moves == []


def test_busy_gripper_cannot_arm_or_move():
    core, fake = make_core()
    fake.status = GripperStatus(800, True, False)
    core.handle({"cmd": "set_mode", "mode": "COMMAND", "token": "secret"})
    assert core.handle({"cmd": "arm", "token": "secret"})["code"] == "GRIPPER_BUSY"


def test_stop_needs_no_token_and_revokes_authority():
    core, fake = make_core()
    arm(core)
    reply = core.handle({"cmd": "stop"})
    assert reply["ok"] and reply["mode"] == "MIRROR" and not reply["armed"]
    assert fake.stops == 1


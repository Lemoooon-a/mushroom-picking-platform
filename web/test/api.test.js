import assert from "node:assert/strict";
import test from "node:test";

import { ApiError, createRobotApi } from "../src/api.js";
import {
  getControlAvailability,
  shouldRefreshAxis,
  shouldRefreshTcp,
} from "../src/control-state.js";

function response(payload, { ok = true, status = 200 } = {}) {
  return {
    ok,
    status,
    text: async () => JSON.stringify(payload),
  };
}

test("TCP plan and execute use the backend BaseTargetRequest shape", async () => {
  const requests = [];
  const api = createRobotApi("http://robot.test/", async (url, options) => {
    requests.push({ url, options });
    return response({ stages: [] });
  });
  const target = { x_mm: 300, y_mm: 400, z_mm: 120, yaw_deg: 0 };

  await api.planBaseTarget(target);
  await api.executeBaseTarget(target);

  assert.deepEqual(
    requests.map(({ url, options }) => [url, options.method, JSON.parse(options.body)]),
    [
      ["http://robot.test/api/motion/base/plan", "POST", target],
      ["http://robot.test/api/motion/base/execute", "POST", target],
    ],
  );
});

test("relative Jog sends only delta to the selected axis endpoint", async () => {
  let observed;
  const api = createRobotApi("http://robot.test", async (url, options) => {
    observed = { url, options };
    return response({ status: "arrived" });
  });

  await api.moveAxisRelative("shoulder", -10);

  assert.equal(observed.url, "http://robot.test/api/axes/shoulder/move-relative");
  assert.equal(observed.options.method, "POST");
  assert.deepEqual(JSON.parse(observed.options.body), { delta: -10 });
});

test("current TCP and Return use the dedicated Base-motion endpoints", async () => {
  const requests = [];
  const api = createRobotApi("http://robot.test", async (url, options) => {
    requests.push({ url, options });
    return response({ x_mm: 250, y_mm: 200, z_mm: 200, yaw_deg: 0 });
  });

  await api.getCurrentTcpPose();
  await api.returnToStartup();

  assert.deepEqual(
    requests.map(({ url, options }) => [url, options.method]),
    [
      ["http://robot.test/api/motion/base/current", "GET"],
      ["http://robot.test/api/motion/return-to-startup", "POST"],
    ],
  );
});

test("vacuum pump controls use grip and idle suction commands", async () => {
  const requests = [];
  const api = createRobotApi("http://robot.test", async (url, options) => {
    requests.push({ url, options });
    return response({ ok: true });
  });

  await api.setSuction("grip");
  await api.setSuction("idle");

  assert.deepEqual(
    requests.map(({ url, options }) => [url, options.method, JSON.parse(options.body)]),
    [
      ["http://robot.test/api/suction", "POST", { action: "grip" }],
      ["http://robot.test/api/suction", "POST", { action: "idle" }],
    ],
  );
});

test("backend error.message is exposed without a raw response dump", async () => {
  const api = createRobotApi("http://robot.test", async () =>
    response(
      { error: { type: "RobotServiceStateError", message: "move requires READY" } },
      { ok: false, status: 409 },
    ),
  );

  await assert.rejects(api.startup(), (error) => {
    assert.ok(error instanceof ApiError);
    assert.equal(error.status, 409);
    assert.equal(error.type, "RobotServiceStateError");
    assert.equal(error.message, "move requires READY");
    return true;
  });
});

test("STOP remains enabled while another motion request is pending", () => {
  const controls = getControlAvailability(
    {
      startup: false,
      stop: false,
      returnToStartup: false,
      tcpPlan: false,
      tcpExecute: true,
      jog: false,
    },
    "ready",
    "execute",
  );

  assert.equal(controls.tcpExecute, false);
  assert.equal(controls.jog, false);
  assert.equal(controls.stop, true);
});

test("other controls pause during Startup or STOP while STOP stays independent", () => {
  for (const activeRequest of ["startup", "stop"]) {
    const pending = {
      startup: false,
      stop: false,
      returnToStartup: false,
      tcpPlan: false,
      tcpExecute: false,
      jog: false,
      [activeRequest]: true,
    };
    const controls = getControlAvailability(
      pending,
      activeRequest === "startup" ? "created" : "ready",
      "execute",
    );

    assert.equal(controls.startup, false);
    assert.equal(controls.tcpPlan, false);
    assert.equal(controls.tcpExecute, false);
    assert.equal(controls.jog, false);
    assert.equal(controls.stop, activeRequest !== "stop");
  }
});

test("Startup and Return follow Robot Service lifecycle state", () => {
  const idle = {
    startup: false,
    stop: false,
    returnToStartup: false,
    tcpPlan: false,
    tcpExecute: false,
    jog: false,
  };

  assert.equal(getControlAvailability(idle, "created", "execute").startup, true);
  assert.equal(getControlAvailability(idle, "ready", "execute").startup, false);
  assert.equal(
    getControlAvailability(idle, "ready", "execute").returnToStartup,
    true,
  );
  assert.equal(
    getControlAvailability(idle, "ready", "dry-run").returnToStartup,
    false,
  );
  assert.equal(getControlAvailability(idle, "fault", "execute").jog, false);
  assert.equal(getControlAvailability(idle, "ready", "execute").suction, true);
  assert.equal(getControlAvailability(idle, "ready", "dry-run").suction, false);
  assert.equal(
    getControlAvailability({ ...idle, suction: true }, "ready", "execute").tcpExecute,
    false,
  );
});

test("axis refresh is limited to idle READY or DISABLED states", () => {
  const idle = {
    startup: false,
    stop: false,
    returnToStartup: false,
    tcpPlan: false,
    tcpExecute: false,
    jog: false,
  };

  assert.equal(shouldRefreshAxis("ready", idle), true);
  assert.equal(shouldRefreshAxis("disabled", idle), true);
  assert.equal(shouldRefreshAxis("executing", idle), false);
  assert.equal(shouldRefreshAxis("planning", idle), false);
  assert.equal(shouldRefreshAxis("fault", idle), false);
  assert.equal(shouldRefreshAxis("ready", { ...idle, jog: true }), false);
  assert.equal(shouldRefreshAxis("ready", { ...idle, stop: true }), false);
});

test("current TCP refreshes only when READY and no command is pending", () => {
  const idle = {
    startup: false,
    stop: false,
    returnToStartup: false,
    tcpPlan: false,
    tcpExecute: false,
    jog: false,
  };

  assert.equal(shouldRefreshTcp("ready", idle), true);
  assert.equal(shouldRefreshTcp("disabled", idle), false);
  assert.equal(shouldRefreshTcp("executing", idle), false);
  assert.equal(shouldRefreshTcp("ready", { ...idle, jog: true }), false);
  assert.equal(
    shouldRefreshTcp("ready", { ...idle, returnToStartup: true }),
    false,
  );
});

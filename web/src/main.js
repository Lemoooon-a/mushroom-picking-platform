import "./style.css";
import { ApiError, robotApi } from "./api.js";
import {
  getControlAvailability,
  shouldRefreshAxis,
  shouldRefreshTcp,
} from "./control-state.js";

const AXES = [
  { id: "slide", label: "Slide", unit: "mm" },
  { id: "z", label: "Z Axis", unit: "mm" },
  { id: "shoulder", label: "Shoulder", unit: "deg" },
  { id: "elbow", label: "Elbow", unit: "deg" },
  { id: "rotation", label: "Rotation", unit: "deg" },
];

const pending = {
  startup: false,
  stop: false,
  returnToStartup: false,
  tcpPlan: false,
  tcpExecute: false,
  jog: false,
  suction: false,
};

let activeSuctionAction = null;
let suctionCommandVersion = 0;

let selectedAxis = AXES[0];
let latestStatus = null;
let statusRequestPromise = null;
let axisRequestPending = false;
let tcpPoseRequestPending = false;
let tcpPoseInitialized = false;
let lastAxisRefresh = 0;

document.querySelector("#app").innerHTML = `
  <div class="console-shell">
    <header class="topbar">
      <div class="brand-block">
        <div class="brand-mark" aria-hidden="true">
          <span></span><span></span><span></span>
        </div>
        <div>
          <p class="eyebrow">MUSHROOM PICKING PLATFORM</p>
          <h1>Robot Control</h1>
        </div>
      </div>

      <div class="status-cluster" aria-live="polite">
        <div class="status-readout">
          <span class="status-light is-offline" id="status-light"></span>
          <div>
            <span class="readout-label">Robot state</span>
            <strong id="robot-state">CONNECTING</strong>
          </div>
        </div>
        <div class="status-readout mode-readout">
          <span class="readout-label">Mode</span>
          <strong id="robot-mode">—</strong>
        </div>
        <div class="connection-copy" id="connection-copy">API connecting…</div>
      </div>

      <div class="topbar-actions">
        <button class="button button-secondary" id="startup-button" type="button">
          <span class="button-icon" aria-hidden="true">↗</span>
          <span id="startup-label">Startup</span>
        </button>
        <button class="button button-secondary return-button" id="return-button" type="button">
          <span class="button-icon" aria-hidden="true">↩</span>
          Return
        </button>
        <button class="stop-button" id="stop-button" type="button">
          <span class="stop-symbol" aria-hidden="true"></span>
          <span><small>MOTION</small>STOP</span>
        </button>
      </div>
    </header>

    <main class="workspace">
      <section class="card tcp-card" aria-labelledby="tcp-heading">
        <div class="card-heading">
          <div>
            <p class="section-index">01 / CARTESIAN TARGET</p>
            <h2 id="tcp-heading">TCP Absolute Position</h2>
            <p class="card-subtitle">Base Frame · Tool Center Point</p>
          </div>
          <span class="operation-chip is-idle" id="tcp-chip">IDLE</span>
        </div>

        <div class="current-tcp" id="current-tcp">
          <div class="current-tcp-heading">
            <div>
              <span class="axis-overline">CURRENT TCP</span>
              <strong>Base Frame</strong>
            </div>
            <span class="tcp-pose-health" id="tcp-pose-health">Awaiting pose</span>
          </div>
          <div class="current-tcp-grid">
            ${tcpPoseValue("x", "X", "mm")}
            ${tcpPoseValue("y", "Y", "mm")}
            ${tcpPoseValue("z", "Z", "mm")}
            ${tcpPoseValue("yaw", "Yaw", "deg")}
          </div>
        </div>

        <form id="tcp-form" class="tcp-form">
          <div class="coordinate-grid">
            ${coordinateInput("x", "X", "x_mm", "mm")}
            ${coordinateInput("y", "Y", "y_mm", "mm")}
            ${coordinateInput("z", "Z", "z_mm", "mm")}
            ${coordinateInput("yaw", "Yaw", "yaw_deg", "deg")}
          </div>
          <div class="action-row">
            <button class="button button-secondary button-large" id="plan-button" type="submit">
              <span class="button-kicker">CHECK PATH</span>
              Plan
            </button>
            <button class="button button-primary button-large" id="execute-button" type="button">
              <span class="button-kicker">SEND TARGET</span>
              Execute
            </button>
          </div>
        </form>

        <div class="plan-panel is-empty" id="plan-panel">
          <div class="plan-empty">
            <span class="plan-empty-icon" aria-hidden="true">⌁</span>
            <div>
              <strong>No active plan</strong>
              <p>Enter a Base-frame target and run Plan to inspect the result.</p>
            </div>
          </div>
        </div>
      </section>

      <section class="card jog-card" aria-labelledby="jog-heading">
        <div class="card-heading">
          <div>
            <p class="section-index">02 / MANUAL AXIS</p>
            <h2 id="jog-heading">Single Axis Jog</h2>
            <p class="card-subtitle">Relative movement · Service limits apply</p>
          </div>
          <span class="operation-chip is-idle" id="jog-chip">IDLE</span>
        </div>

        <div class="axis-tabs" id="axis-tabs" role="tablist" aria-label="Robot axis">
          ${AXES.map(
            (axis, index) => `
              <button
                class="axis-tab${index === 0 ? " is-selected" : ""}"
                type="button"
                role="tab"
                aria-selected="${index === 0}"
                data-axis="${axis.id}"
              >${axis.label}</button>
            `,
          ).join("")}
        </div>

        <div class="axis-position">
          <div>
            <span class="axis-overline">CURRENT POSITION</span>
            <h3 id="axis-name">Slide</h3>
          </div>
          <div class="position-value-wrap">
            <strong id="axis-position">—</strong>
            <span id="axis-unit">mm</span>
          </div>
          <div class="axis-health" id="axis-health">
            <span></span> Awaiting axis data
          </div>
        </div>

        <div class="jog-presets" aria-label="Preset relative movements">
          <button class="jog-button is-negative" type="button" data-delta="-10">−10</button>
          <button class="jog-button is-negative" type="button" data-delta="-1">−1</button>
          <div class="jog-axis-line" aria-hidden="true"><span></span></div>
          <button class="jog-button is-positive" type="button" data-delta="1">+1</button>
          <button class="jog-button is-positive" type="button" data-delta="10">+10</button>
        </div>

        <form class="custom-jog" id="jog-form">
          <label for="custom-delta">
            <span>Custom delta</span>
            <div class="input-with-unit compact-input">
              <input id="custom-delta" name="delta" type="number" step="any" value="5.0" required />
              <span id="custom-unit">mm</span>
            </div>
          </label>
          <button class="button button-primary" type="submit" id="jog-move-button">
            Move axis
          </button>
        </form>

        <p class="safety-note">
          <span aria-hidden="true">i</span>
          Jog sends only a relative delta. Limits and motion authorization remain in the Robot Service.
        </p>
      </section>

      <section class="card suction-card" aria-labelledby="suction-heading">
        <div class="card-heading suction-heading">
          <div>
            <p class="section-index">03 / END EFFECTOR</p>
            <h2 id="suction-heading">Vacuum Pump</h2>
            <p class="card-subtitle">Direct pump control · STOP always switches the pump off</p>
          </div>
          <span class="operation-chip is-idle" id="suction-chip">UNKNOWN</span>
        </div>
        <div class="suction-controls">
          <p>
            <strong>Air pump command</strong>
            <span>Start suction for gripping, or return the suction output to idle.</span>
          </p>
          <div class="suction-actions">
            <button class="button button-primary" id="pump-on-button" type="button">
              Pump ON
            </button>
            <button class="button button-secondary" id="pump-off-button" type="button">
              Pump OFF
            </button>
          </div>
        </div>
      </section>

      <section class="feedback-bar is-neutral" id="feedback-bar" aria-live="polite">
        <div class="feedback-icon" id="feedback-icon" aria-hidden="true">·</div>
        <div class="feedback-copy">
          <span id="feedback-label">SYSTEM FEEDBACK</span>
          <strong id="feedback-title">Console ready</strong>
          <p id="feedback-message">No motion command is sent when this page loads.</p>
        </div>
        <time id="feedback-time">—</time>
      </section>
    </main>

    <footer class="console-footer">
      <span><i class="footer-dot"></i> LOCAL CONTROL CONSOLE</span>
      <span>HTTP · 1 s status poll</span>
    </footer>
  </div>
`;

const elements = {
  robotState: document.querySelector("#robot-state"),
  robotMode: document.querySelector("#robot-mode"),
  statusLight: document.querySelector("#status-light"),
  connectionCopy: document.querySelector("#connection-copy"),
  startupButton: document.querySelector("#startup-button"),
  startupLabel: document.querySelector("#startup-label"),
  returnButton: document.querySelector("#return-button"),
  stopButton: document.querySelector("#stop-button"),
  tcpForm: document.querySelector("#tcp-form"),
  planButton: document.querySelector("#plan-button"),
  executeButton: document.querySelector("#execute-button"),
  tcpChip: document.querySelector("#tcp-chip"),
  planPanel: document.querySelector("#plan-panel"),
  tcpPoseHealth: document.querySelector("#tcp-pose-health"),
  tcpPoseX: document.querySelector("#current-tcp-x"),
  tcpPoseY: document.querySelector("#current-tcp-y"),
  tcpPoseZ: document.querySelector("#current-tcp-z"),
  tcpPoseYaw: document.querySelector("#current-tcp-yaw"),
  axisTabs: document.querySelector("#axis-tabs"),
  axisName: document.querySelector("#axis-name"),
  axisPosition: document.querySelector("#axis-position"),
  axisUnit: document.querySelector("#axis-unit"),
  axisHealth: document.querySelector("#axis-health"),
  jogForm: document.querySelector("#jog-form"),
  jogMoveButton: document.querySelector("#jog-move-button"),
  customUnit: document.querySelector("#custom-unit"),
  jogChip: document.querySelector("#jog-chip"),
  suctionChip: document.querySelector("#suction-chip"),
  pumpOnButton: document.querySelector("#pump-on-button"),
  pumpOffButton: document.querySelector("#pump-off-button"),
  feedbackBar: document.querySelector("#feedback-bar"),
  feedbackIcon: document.querySelector("#feedback-icon"),
  feedbackLabel: document.querySelector("#feedback-label"),
  feedbackTitle: document.querySelector("#feedback-title"),
  feedbackMessage: document.querySelector("#feedback-message"),
  feedbackTime: document.querySelector("#feedback-time"),
};

elements.startupButton.addEventListener("click", startup);
elements.returnButton.addEventListener("click", returnToStartup);
elements.stopButton.addEventListener("click", stopRobot);
elements.tcpForm.addEventListener("submit", planTarget);
elements.executeButton.addEventListener("click", executeTarget);
elements.axisTabs.addEventListener("click", selectAxis);
elements.jogForm.addEventListener("submit", submitCustomJog);
elements.pumpOnButton.addEventListener("click", () => setSuction("grip"));
elements.pumpOffButton.addEventListener("click", () => setSuction("idle"));
document.querySelectorAll("[data-delta]").forEach((button) => {
  button.addEventListener("click", () => jogAxis(Number(button.dataset.delta)));
});

renderControls();
pollStatus();
window.setInterval(pollStatus, 1000);

function coordinateInput(id, label, name, unit) {
  return `
    <label class="coordinate-field" for="tcp-${id}">
      <span class="coordinate-label">${label}</span>
      <div class="input-with-unit">
        <input
          id="tcp-${id}"
          name="${name}"
          type="number"
          step="any"
          inputmode="decimal"
          placeholder="0.000"
          required
        />
        <span>${unit}</span>
      </div>
    </label>
  `;
}

function tcpPoseValue(id, label, unit) {
  return `
    <div class="current-tcp-value">
      <span>${label}</span>
      <strong id="current-tcp-${id}">—</strong>
      <small>${unit}</small>
    </div>
  `;
}

async function pollStatus({
  forceAxisRefresh = false,
  forceTcpRefresh = false,
} = {}) {
  if (!statusRequestPromise) {
    const request = updateStatus();
    statusRequestPromise = request;
    try {
      await request;
    } finally {
      if (statusRequestPromise === request) statusRequestPromise = null;
    }
  } else {
    await statusRequestPromise;
  }

  if (forceAxisRefresh) {
    await refreshAxisIfAllowed();
  }
  if (forceTcpRefresh) {
    await refreshTcpIfAllowed();
  }
}

async function updateStatus() {
  try {
    latestStatus = await robotApi.getStatus();
    renderStatus(latestStatus);
    const now = Date.now();
    const axisRefreshAllowed = shouldRefreshAxis(latestStatus.state, pending);
    if (axisRefreshAllowed && now - lastAxisRefresh >= 2000) {
      lastAxisRefresh = now;
      void refreshSelectedAxis({ quiet: true });
    } else if (!axisRefreshAllowed && !axisRequestPending) {
      renderDeferredAxisRefresh(latestStatus.state);
    }
    const tcpRefreshAllowed = shouldRefreshTcp(latestStatus.state, pending);
    if (tcpRefreshAllowed && !tcpPoseInitialized) {
      tcpPoseInitialized = true;
      void refreshCurrentTcp({ quiet: true });
    } else if (!tcpRefreshAllowed && !tcpPoseRequestPending) {
      tcpPoseInitialized = false;
      renderDeferredTcpRefresh(latestStatus.state);
    }
  } catch (error) {
    renderStatusError(error);
  }
}

async function refreshAxisIfAllowed() {
  if (shouldRefreshAxis(latestStatus?.state, pending)) {
    lastAxisRefresh = Date.now();
    await refreshSelectedAxis({ quiet: true });
  } else if (!axisRequestPending) {
    renderDeferredAxisRefresh(latestStatus?.state);
  }
}

function renderStatus(status) {
  const state = String(status.state ?? "unknown").toLowerCase();
  const mode = String(status.mode ?? "unknown");
  elements.robotState.textContent = state.toUpperCase();
  elements.robotMode.textContent = mode.toUpperCase();
  elements.statusLight.className = `status-light is-${statusTone(state)}`;
  elements.connectionCopy.textContent = status.fault ? "FAULT REPORTED" : "API ONLINE";
  elements.connectionCopy.classList.toggle("is-error", Boolean(status.fault));
  elements.connectionCopy.classList.add("is-online");
  renderControls();
}

function renderStatusError(error) {
  latestStatus = null;
  elements.robotState.textContent = "OFFLINE";
  elements.robotMode.textContent = "—";
  elements.statusLight.className = "status-light is-offline";
  elements.connectionCopy.textContent = errorMessage(error);
  elements.connectionCopy.className = "connection-copy is-error";
  renderControls();
}

function statusTone(state) {
  if (state === "ready") return "ready";
  if (["starting", "observing", "planning", "executing"].includes(state)) return "busy";
  if (state === "fault") return "fault";
  if (["created", "disabled", "shutdown"].includes(state)) return "standby";
  return "offline";
}

function renderDeferredAxisRefresh(state) {
  const normalizedState = String(state ?? "unknown").toLowerCase();
  if (["ready", "disabled"].includes(normalizedState)) {
    elements.axisHealth.className = "axis-health";
    elements.axisHealth.innerHTML = "<span></span> Axis refresh paused during request";
    return;
  }
  const message = normalizedState === "fault"
    ? "Position refresh unavailable while Robot Service is FAULT"
    : `Position refresh deferred while Service is ${normalizedState.toUpperCase()}`;
  elements.axisHealth.className = `axis-health ${normalizedState === "fault" ? "is-warning" : ""}`;
  elements.axisHealth.innerHTML = "<span></span>";
  elements.axisHealth.append(document.createTextNode(` ${message}`));
}

async function startup() {
  if (pending.startup) return;
  setPending("startup", true);
  setFeedback("sending", "Startup requested", "Waiting for the Robot Service to initialize.");
  try {
    await robotApi.startup();
    setFeedback("success", "Startup complete", "Robot Service accepted the startup request.");
  } catch (error) {
    showRequestError("Startup failed", error);
  } finally {
    setPending("startup", false);
    await pollStatus({ forceAxisRefresh: true, forceTcpRefresh: true });
  }
}

async function returnToStartup() {
  if (pending.returnToStartup) return;
  setPending("returnToStartup", true);
  setFeedback("moving", "Returning to startup", "Executing the configured startup-safe pose.");
  try {
    await robotApi.returnToStartup();
    setFeedback("success", "Return complete", "Robot reached the configured startup-safe pose.");
  } catch (error) {
    showRequestError("Return failed", error);
  } finally {
    setPending("returnToStartup", false);
    await pollStatus({ forceAxisRefresh: true, forceTcpRefresh: true });
  }
}

async function stopRobot() {
  if (pending.stop) return;
  const commandVersion = ++suctionCommandVersion;
  setPending("stop", true);
  setFeedback("moving", "STOP sending", "Requesting a coordinated stop from the Robot Service.");
  try {
    await robotApi.stop();
    if (commandVersion === suctionCommandVersion) {
      setOperationChip(elements.suctionChip, "success", "OFF");
      setFeedback("success", "STOP accepted", "Motion stopped and the vacuum pump was switched off.");
    }
  } catch (error) {
    if (commandVersion === suctionCommandVersion) {
      setOperationChip(elements.suctionChip, "error", "UNKNOWN");
      showRequestError("STOP failed", error);
    }
  } finally {
    setPending("stop", false);
    await pollStatus({ forceAxisRefresh: true, forceTcpRefresh: true });
  }
}

async function setSuction(action) {
  if (pending.suction) return;
  const turningOn = action === "grip";
  const commandVersion = ++suctionCommandVersion;
  activeSuctionAction = action;
  setPending("suction", true);
  setOperationChip(elements.suctionChip, "busy", turningOn ? "STARTING" : "STOPPING");
  setFeedback(
    "sending",
    turningOn ? "Starting vacuum pump" : "Stopping vacuum pump",
    turningOn ? "Sending the suction grip command." : "Returning suction control to idle.",
  );
  try {
    await robotApi.setSuction(action);
    if (commandVersion === suctionCommandVersion) {
      setOperationChip(elements.suctionChip, "success", turningOn ? "ON" : "OFF");
      setFeedback(
        "success",
        turningOn ? "Vacuum pump started" : "Vacuum pump stopped",
        turningOn ? "The grip command was accepted." : "The idle command was accepted.",
      );
    }
  } catch (error) {
    if (commandVersion === suctionCommandVersion) {
      setOperationChip(elements.suctionChip, "error", "ERROR");
      showRequestError(turningOn ? "Pump start failed" : "Pump stop failed", error);
    }
  } finally {
    activeSuctionAction = null;
    setPending("suction", false);
    await pollStatus();
  }
}

async function planTarget(event) {
  event.preventDefault();
  const target = readTcpTarget();
  if (!target || pending.tcpPlan || pending.tcpExecute || pending.jog) return;
  setPending("tcpPlan", true);
  setOperationChip(elements.tcpChip, "busy", "PLANNING");
  setFeedback("sending", "Planning target", formatTarget(target));
  try {
    const result = await robotApi.planBaseTarget(target);
    renderPlan(result);
    setOperationChip(elements.tcpChip, "success", "PLANNED");
    setFeedback("success", "Plan ready", summarizePlan(result));
  } catch (error) {
    setOperationChip(elements.tcpChip, "error", "REJECTED");
    showRequestError("Plan failed", error);
  } finally {
    setPending("tcpPlan", false);
    await pollStatus({ forceAxisRefresh: true });
  }
}

async function executeTarget() {
  const target = readTcpTarget();
  if (!target || pending.tcpPlan || pending.tcpExecute || pending.jog) return;
  setPending("tcpExecute", true);
  setOperationChip(elements.tcpChip, "busy", "MOVING");
  setFeedback("moving", "Executing Base target", formatTarget(target));
  try {
    const result = await robotApi.executeBaseTarget(target);
    if (result?.plan) renderPlan(result.plan);
    setOperationChip(elements.tcpChip, "success", result?.executed === false ? "DRY RUN" : "COMPLETE");
    setFeedback(
      "success",
      result?.executed === false ? "Dry-run complete" : "Motion complete",
      result?.message ?? "Robot Service completed the Base target request.",
    );
  } catch (error) {
    setOperationChip(elements.tcpChip, "error", "ERROR");
    showRequestError("Execute failed", error);
  } finally {
    setPending("tcpExecute", false);
    await pollStatus({ forceAxisRefresh: true, forceTcpRefresh: true });
  }
}

function readTcpTarget() {
  if (!elements.tcpForm.reportValidity()) return null;
  const data = new FormData(elements.tcpForm);
  const target = {
    x_mm: Number(data.get("x_mm")),
    y_mm: Number(data.get("y_mm")),
    z_mm: Number(data.get("z_mm")),
    yaw_deg: Number(data.get("yaw_deg")),
  };
  if (!Object.values(target).every(Number.isFinite)) {
    setFeedback("error", "Invalid target", "All TCP fields must contain finite numbers.");
    return null;
  }
  return target;
}

function selectAxis(event) {
  const button = event.target.closest("[data-axis]");
  if (!button) return;
  selectedAxis = AXES.find((axis) => axis.id === button.dataset.axis) ?? AXES[0];
  elements.axisTabs.querySelectorAll("[data-axis]").forEach((tab) => {
    const selected = tab.dataset.axis === selectedAxis.id;
    tab.classList.toggle("is-selected", selected);
    tab.setAttribute("aria-selected", String(selected));
  });
  elements.axisName.textContent = selectedAxis.label;
  elements.axisUnit.textContent = selectedAxis.unit;
  elements.customUnit.textContent = selectedAxis.unit;
  elements.axisPosition.textContent = "—";
  elements.axisHealth.className = "axis-health";
  elements.axisHealth.innerHTML = "<span></span> Reading axis state";
  if (shouldRefreshAxis(latestStatus?.state, pending)) {
    refreshSelectedAxis();
  } else {
    renderDeferredAxisRefresh(latestStatus?.state);
  }
}

async function refreshSelectedAxis({ quiet = false } = {}) {
  if (axisRequestPending) return;
  const requestedAxis = selectedAxis;
  axisRequestPending = true;
  try {
    const state = await robotApi.getAxisState(requestedAxis.id);
    if (requestedAxis.id !== selectedAxis.id) return;
    const unit = state.position_unit || requestedAxis.unit;
    elements.axisPosition.textContent = formatNumber(state.current_position);
    elements.axisUnit.textContent = unit;
    elements.customUnit.textContent = unit;
    const healthy = state.connected && state.position_valid && !state.faulted;
    const healthText = state.faulted
      ? state.fault_message || `Fault ${state.fault_code ?? "reported"}`
      : !state.connected
        ? "Axis disconnected"
        : !state.position_valid
          ? "Position unavailable"
          : state.busy
            ? "Axis moving"
            : "Axis ready";
    elements.axisHealth.className = `axis-health ${healthy ? "is-healthy" : "is-warning"}`;
    elements.axisHealth.innerHTML = "<span></span>";
    elements.axisHealth.append(document.createTextNode(` ${healthText}`));
  } catch (error) {
    if (requestedAxis.id !== selectedAxis.id) return;
    elements.axisHealth.className = "axis-health is-warning";
    elements.axisHealth.innerHTML = "<span></span>";
    elements.axisHealth.append(document.createTextNode(` ${errorMessage(error)}`));
    if (!quiet) showRequestError(`${requestedAxis.label} state unavailable`, error);
  } finally {
    axisRequestPending = false;
  }
}

async function refreshTcpIfAllowed() {
  if (shouldRefreshTcp(latestStatus?.state, pending)) {
    await refreshCurrentTcp({ quiet: true });
  } else {
    renderDeferredTcpRefresh(latestStatus?.state);
  }
}

async function refreshCurrentTcp({ quiet = false } = {}) {
  if (tcpPoseRequestPending) return;
  tcpPoseInitialized = true;
  tcpPoseRequestPending = true;
  elements.tcpPoseHealth.className = "tcp-pose-health is-reading";
  elements.tcpPoseHealth.textContent = "Reading pose";
  try {
    const pose = await robotApi.getCurrentTcpPose();
    elements.tcpPoseX.textContent = formatNumber(pose.x_mm);
    elements.tcpPoseY.textContent = formatNumber(pose.y_mm);
    elements.tcpPoseZ.textContent = formatNumber(pose.z_mm);
    elements.tcpPoseYaw.textContent = formatNumber(pose.yaw_deg);
    elements.tcpPoseHealth.className = "tcp-pose-health is-healthy";
    elements.tcpPoseHealth.textContent = "Position verified";
  } catch (error) {
    elements.tcpPoseHealth.className = "tcp-pose-health is-warning";
    elements.tcpPoseHealth.textContent = errorMessage(error);
    if (!quiet) showRequestError("Current TCP unavailable", error);
  } finally {
    tcpPoseRequestPending = false;
  }
}

function renderDeferredTcpRefresh(state) {
  const normalizedState = String(state ?? "unknown").toLowerCase();
  elements.tcpPoseHealth.className = "tcp-pose-health";
  elements.tcpPoseHealth.textContent = ["ready", "disabled"].includes(normalizedState)
    ? "Pose refresh paused during request"
    : `Pose refresh deferred · ${normalizedState.toUpperCase()}`;
}

function submitCustomJog(event) {
  event.preventDefault();
  if (!elements.jogForm.reportValidity()) return;
  const delta = Number(new FormData(elements.jogForm).get("delta"));
  if (!Number.isFinite(delta)) {
    setFeedback("error", "Invalid Jog delta", "Delta must be a finite number.");
    return;
  }
  jogAxis(delta);
}

async function jogAxis(delta) {
  if (pending.tcpPlan || pending.tcpExecute || pending.jog) return;
  const axis = selectedAxis;
  setPending("jog", true);
  setOperationChip(elements.jogChip, "busy", "MOVING");
  setFeedback("moving", `Moving ${axis.label}`, `${signed(delta)} ${axis.unit} relative move requested.`);
  try {
    const result = await robotApi.moveAxisRelative(axis.id, delta);
    setOperationChip(elements.jogChip, "success", "COMPLETE");
    setFeedback(
      "success",
      `${axis.label} move complete`,
      result?.message ?? `Relative move ${signed(delta)} ${axis.unit} completed.`,
    );
  } catch (error) {
    setOperationChip(elements.jogChip, "error", "ERROR");
    showRequestError(`${axis.label} move failed`, error);
  } finally {
    setPending("jog", false);
    await pollStatus({ forceAxisRefresh: true, forceTcpRefresh: true });
  }
}

function renderPlan(plan) {
  const stages = Array.isArray(plan?.stages) ? plan.stages : [];
  const finalSolution = stages.at(-1)?.solution;
  const stageNames = stages.map((stage) => String(stage.kind || "stage").toUpperCase());
  const summaryItems = [
    ["Path", stageNames.length ? stageNames.join(" → ") : "Plan accepted"],
    [
      "Workspace",
      plan?.current_workspace_side && plan?.target_workspace_side
        ? `${plan.current_workspace_side} → ${plan.target_workspace_side}`
        : "—",
    ],
    ["Clearance", plan?.requires_side_switch_clearance ? `${formatNumber(plan.clearance_lift_mm)} mm lift` : "Not required"],
    ["Stages", stages.length ? String(stages.length) : "—"],
  ];

  const axisTargets = finalSolution
    ? `
      <div class="target-strip">
        ${[
          ["SLIDE", finalSolution.slide_mm, "mm"],
          ["Z", finalSolution.z_mm, "mm"],
          ["SHOULDER", finalSolution.shoulder_deg, "deg"],
          ["ELBOW", finalSolution.elbow_deg, "deg"],
          ["ROTATION", finalSolution.rotation_deg, "deg"],
        ]
          .map(
            ([label, value, unit]) => `
              <div><span>${label}</span><strong>${escapeHtml(formatNumber(value))}</strong><small>${unit}</small></div>
            `,
          )
          .join("")}
      </div>
    `
    : "";

  elements.planPanel.className = "plan-panel";
  elements.planPanel.innerHTML = `
    <div class="plan-title-row">
      <div><span class="plan-check">✓</span><strong>Plan result</strong></div>
      <span>${stages.length ? `${stages.length} stage${stages.length === 1 ? "" : "s"}` : "Accepted"}</span>
    </div>
    <div class="plan-summary">
      ${summaryItems
        .map(([label, value]) => `<div><span>${label}</span><strong>${escapeHtml(value)}</strong></div>`)
        .join("")}
    </div>
    ${axisTargets}
    <details class="plan-details">
      <summary>Full planning response</summary>
      <pre>${escapeHtml(JSON.stringify(plan, null, 2))}</pre>
    </details>
  `;
}

function summarizePlan(plan) {
  const count = Array.isArray(plan?.stages) ? plan.stages.length : 0;
  return count ? `${count} planning stage${count === 1 ? "" : "s"} returned by the Robot Service.` : "Robot Service accepted the target plan.";
}

function renderControls() {
  const availability = getControlAvailability(
    pending,
    latestStatus?.state,
    latestStatus?.mode,
  );
  elements.startupButton.disabled = !availability.startup;
  elements.returnButton.disabled = !availability.returnToStartup;
  elements.planButton.disabled = !availability.tcpPlan;
  elements.executeButton.disabled = !availability.tcpExecute;
  elements.jogMoveButton.disabled = !availability.jog;
  const suctionAvailable =
    availability.suction && latestStatus?.capabilities?.suction_command !== false;
  elements.pumpOnButton.disabled = !suctionAvailable;
  elements.pumpOffButton.disabled = !suctionAvailable;
  document.querySelectorAll(".jog-button, .axis-tab").forEach((button) => {
    button.disabled = !availability.jog;
  });
  elements.stopButton.disabled = !availability.stop;
  const state = String(latestStatus?.state ?? "").toLowerCase();
  elements.startupLabel.textContent = state === "ready" ? "Started" : "Startup";
  elements.startupButton.classList.toggle("is-loading", pending.startup);
  elements.returnButton.classList.toggle("is-loading", pending.returnToStartup);
  elements.planButton.classList.toggle("is-loading", pending.tcpPlan);
  elements.executeButton.classList.toggle("is-loading", pending.tcpExecute);
  elements.jogMoveButton.classList.toggle("is-loading", pending.jog);
  elements.pumpOnButton.classList.toggle(
    "is-loading",
    pending.suction && activeSuctionAction === "grip",
  );
  elements.pumpOffButton.classList.toggle(
    "is-loading",
    pending.suction && activeSuctionAction === "idle",
  );
  elements.stopButton.classList.toggle("is-loading", pending.stop);
}

function setPending(operation, value) {
  pending[operation] = value;
  renderControls();
}

function setOperationChip(element, tone, text) {
  element.className = `operation-chip is-${tone}`;
  element.textContent = text;
}

function setFeedback(tone, title, message) {
  const icons = { neutral: "·", sending: "↗", moving: "↻", success: "✓", error: "!" };
  const labels = {
    neutral: "SYSTEM FEEDBACK",
    sending: "SENDING",
    moving: "MOVING",
    success: "SUCCESS",
    error: "ERROR",
  };
  elements.feedbackBar.className = `feedback-bar is-${tone}`;
  elements.feedbackIcon.textContent = icons[tone] ?? "·";
  elements.feedbackLabel.textContent = labels[tone] ?? labels.neutral;
  elements.feedbackTitle.textContent = title;
  elements.feedbackMessage.textContent = message;
  elements.feedbackTime.textContent = new Intl.DateTimeFormat(undefined, {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  }).format(new Date());
}

function showRequestError(title, error) {
  const status = error instanceof ApiError && error.status ? `HTTP ${error.status} · ` : "";
  setFeedback("error", title, `${status}${errorMessage(error)}`);
}

function errorMessage(error) {
  return error instanceof Error ? error.message : "Unknown Robot API error.";
}

function formatTarget(target) {
  return `X ${target.x_mm} · Y ${target.y_mm} · Z ${target.z_mm} mm · Yaw ${target.yaw_deg} deg`;
}

function formatNumber(value) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) return "—";
  return Number(value).toLocaleString(undefined, { maximumFractionDigits: 3 });
}

function signed(value) {
  return value > 0 ? `+${value}` : String(value);
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

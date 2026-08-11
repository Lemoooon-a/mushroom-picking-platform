export function getControlAvailability(pending, state, mode) {
  const operationPending =
    pending.startup ||
    pending.stop ||
    pending.returnToStartup ||
    pending.tcpPlan ||
    pending.tcpExecute ||
    pending.jog;
  const normalizedState = String(state ?? "").toLowerCase();
  const normalizedMode = String(mode ?? "").toLowerCase();
  const ready = normalizedState === "ready";
  return {
    startup:
      !operationPending && ["created", "shutdown"].includes(normalizedState),
    returnToStartup:
      !operationPending && ready && normalizedMode === "execute",
    tcpPlan: !operationPending && ready,
    tcpExecute: !operationPending && ready,
    jog: !operationPending && ready,
    stop: !pending.stop,
  };
}

export function shouldRefreshAxis(state, pending) {
  const normalizedState = String(state ?? "").toLowerCase();
  const requestPending =
    pending.startup ||
    pending.stop ||
    pending.returnToStartup ||
    pending.tcpPlan ||
    pending.tcpExecute ||
    pending.jog;
  return ["ready", "disabled"].includes(normalizedState) && !requestPending;
}

export function shouldRefreshTcp(state, pending) {
  const normalizedState = String(state ?? "").toLowerCase();
  const requestPending =
    pending.startup ||
    pending.stop ||
    pending.returnToStartup ||
    pending.tcpPlan ||
    pending.tcpExecute ||
    pending.jog;
  return normalizedState === "ready" && !requestPending;
}

const DEFAULT_API_BASE_URL = "http://127.0.0.1:8000";

export class ApiError extends Error {
  constructor(message, { status = 0, type = "NetworkError" } = {}) {
    super(message);
    this.name = "ApiError";
    this.status = status;
    this.type = type;
  }
}

export function createRobotApi(
  baseUrl = DEFAULT_API_BASE_URL,
  fetchImplementation = globalThis.fetch,
) {
  const normalizedBaseUrl = baseUrl.replace(/\/$/, "");

  async function request(path, { method = "GET", body } = {}) {
    let response;
    try {
      response = await fetchImplementation(`${normalizedBaseUrl}${path}`, {
        method,
        headers: body === undefined ? undefined : { "Content-Type": "application/json" },
        body: body === undefined ? undefined : JSON.stringify(body),
      });
    } catch (error) {
      throw new ApiError(
        error instanceof Error ? error.message : "Unable to reach the Robot API.",
      );
    }

    const payload = await readPayload(response);
    if (!response.ok) {
      throw new ApiError(
        payload?.error?.message ?? payload?.detail ?? `Request failed with HTTP ${response.status}.`,
        {
          status: response.status,
          type: payload?.error?.type ?? "HttpError",
        },
      );
    }
    return payload;
  }

  return {
    getStatus: () => request("/api/status"),
    startup: () => request("/api/startup", { method: "POST" }),
    stop: () => request("/api/stop", { method: "POST" }),
    setSuction: (action) =>
      request("/api/suction", { method: "POST", body: { action } }),
    getAxisState: (axis) => request(`/api/axes/${encodeURIComponent(axis)}`),
    getCurrentTcpPose: () => request("/api/motion/base/current"),
    planBaseTarget: (target) =>
      request("/api/motion/base/plan", { method: "POST", body: target }),
    executeBaseTarget: (target) =>
      request("/api/motion/base/execute", { method: "POST", body: target }),
    returnToStartup: () =>
      request("/api/motion/return-to-startup", { method: "POST" }),
    moveAxisRelative: (axis, delta) =>
      request(`/api/axes/${encodeURIComponent(axis)}/move-relative`, {
        method: "POST",
        body: { delta },
      }),
  };
}

async function readPayload(response) {
  const text = await response.text();
  if (!text) return null;
  try {
    return JSON.parse(text);
  } catch {
    if (!response.ok) return null;
    throw new ApiError("Robot API returned an invalid JSON response.", {
      status: response.status,
      type: "InvalidResponse",
    });
  }
}

const configuredBaseUrl = import.meta.env?.VITE_API_BASE_URL || DEFAULT_API_BASE_URL;

export const robotApi = createRobotApi(configuredBaseUrl);
export { DEFAULT_API_BASE_URL };

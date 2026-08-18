/**
 * Mushroom Robot Service Web API 1.0.0 类型参考。
 *
 * 这是当前接口快照，不是由 OpenAPI 自动生成的客户端。
 * Base 规划响应在 dry-run 与 execute backend 下当前存在两种形态。
 */

export type JsonPrimitive = string | number | boolean | null;
export type JsonValue = JsonPrimitive | JsonValue[] | { [key: string]: JsonValue };

export type RobotMode = "read-only" | "dry-run" | "execute";
export type RobotState =
  | "created"
  | "starting"
  | "ready"
  | "observing"
  | "planning"
  | "executing"
  | "disabled"
  | "fault"
  | "shutdown";

export type AxisName = "slide" | "z" | "shoulder" | "elbow" | "rotation";
export type AxisKind = "linear" | "rotary";
export type ScanPositionIndex = 1 | 2 | 3 | 4;
export type HandEyeCalibrationStatus = "missing" | "provisional" | "validated";
export type MotionCommandStatus =
  | "accepted"
  | "moving"
  | "arrived"
  | "rejected"
  | "aborted"
  | "timeout"
  | "fault"
  | "communication_error";

export type MotionErrorCode =
  | "invalid_request"
  | "unknown_axis"
  | "backend_unavailable"
  | "unsupported_parameter"
  | "unsupported_command"
  | "invalid_state"
  | "position_invalid"
  | "not_homed"
  | "soft_limit"
  | "busy"
  | "timeout"
  | "device_fault"
  | "communication_error"
  | "backend_error";

export interface OkResponse {
  ok: true;
}

export interface ApiErrorBody {
  type: string;
  message: string;
  rejection_reason?: string;
}

export interface ApiErrorResponse {
  error: ApiErrorBody;
}

export interface RobotCapabilities {
  base_frame_motion: boolean;
  tray_workspace_gate: boolean;
  workspace_planning: boolean;
  robot_motion_envelope: boolean;
  joint_holding: boolean;
  suction_command: boolean;
  vision_gateway: string;
  vision_target_observation: boolean;
  hand_eye_calibration: HandEyeCalibrationStatus;
  vision_target_resolution: boolean;
  pick_planning: boolean;
  pick_execution: boolean;
  physical_pick_verification: boolean;
  axis_listing: boolean;
  axis_state_query: boolean;
  axis_absolute_motion: boolean;
  axis_relative_motion: boolean;
}

export interface RobotStatus {
  state: RobotState;
  mode: RobotMode;
  capabilities: RobotCapabilities;
  /** Mode/backend specific diagnostic data; do not use as a stable UI contract. */
  backend_status: JsonValue | null;
  fault: string | null;
}

export interface AxisCapabilities {
  query_state: boolean;
  move_absolute: boolean;
  stop: boolean;
  reference_home: boolean;
  configurable_velocity: boolean;
  configurable_acceleration: boolean;
  arrival_confirmation: boolean;
}

export interface AxisDescriptor {
  name: AxisName;
  display_name: string;
  kind: AxisKind;
  position_unit: "mm" | "deg" | string;
  velocity_unit: "mm/s" | "deg/s" | string;
  acceleration_unit: "mm/s^2" | "deg/s^2" | string;
  minimum_position: number;
  maximum_position: number;
  capabilities: AxisCapabilities;
}

export interface AxisListResponse {
  axes: AxisDescriptor[];
}

export interface AxisState {
  axis: AxisName;
  connected: boolean;
  enabled: boolean | null;
  busy: boolean | null;
  homed: boolean | null;
  position_valid: boolean;
  current_position: number | null;
  position_unit: "mm" | "deg" | string;
  faulted: boolean;
  fault_code: string | number | null;
  fault_message: string | null;
}

export interface BaseTargetRequest {
  x_mm: number;
  y_mm: number;
  z_mm: number;
  yaw_deg?: number | null;
}

export interface AxisAbsoluteMoveRequest {
  position: number;
  velocity?: number | null;
  acceleration?: number | null;
  timeout_s?: number | null;
}

export interface AxisRelativeMoveRequest {
  delta: number;
  velocity?: number | null;
  acceleration?: number | null;
  timeout_s?: number | null;
}

export interface SuctionRequest {
  action: "grip" | "release" | "idle";
}

export interface MotionCommandResult {
  command_id: string;
  axis: AxisName;
  status: MotionCommandStatus;
  accepted: boolean;
  completed: boolean | null;
  target_position: number;
  final_position: number | null;
  position_error: number | null;
  error_code: MotionErrorCode | null;
  message: string;
  stop_method: string | null;
  command_submitted: boolean | null;
}

export interface RigidTransformJson {
  translation_mm: [number, number, number];
  rotation_rpy_deg: [number, number, number];
}

export type WorkspaceStatus = "inside" | "outside";

export type SlideSelectionReason =
  | "keep_current_slide"
  | "workspace_center"
  | "workspace_fallback"
  | "fixed_slide";

export interface FiveAxisSolution {
  slide_mm: number;
  z_mm: number;
  shoulder_deg: number;
  elbow_deg: number;
  rotation_deg: number;
  local_x_mm: number;
  local_y_mm: number;
  workspace_status: WorkspaceStatus;
  slide_selection_reason: SlideSelectionReason;
  elbow_branch: string;
  position_error_xyz_mm: [number, number, number];
  position_residual_mm: number;
  yaw_residual_deg: number;
  score: number;
  limit_margins: Array<[AxisName, number]>;
}

export interface AxisTarget {
  axis: AxisName;
  position: number;
  velocity: number | null;
  acceleration: number | null;
}

export interface MultiAxisTarget {
  targets: AxisTarget[];
}

export interface BaseMoveStage {
  kind: "direct" | "lift" | "transit" | "lower";
  base_T_tool_target: RigidTransformJson;
  solution: FiveAxisSolution;
  multi_axis_target: MultiAxisTarget;
}

/** dry-run backend shape. */
export interface BaseMovePlan {
  current_base_T_tool: RigidTransformJson;
  requested_base_T_tool_target: RigidTransformJson;
  current_local_x_mm: number;
  current_local_y_mm: number;
  current_workspace_status: WorkspaceStatus;
  target_workspace_status: WorkspaceStatus;
  requires_workspace_entry_clearance: boolean;
  clearance_lift_mm: number;
  clearance_base_z_mm: number | null;
  stages: BaseMoveStage[];
}

/** Current execute backend shape. */
export interface DemoStage {
  name: string;
  base_T_tool_target: RigidTransformJson;
  multi_axis_target: MultiAxisTarget;
  solution: FiveAxisSolution | null;
}

export type BasePlanResponse = BaseMovePlan | DemoStage[];

export interface MotionResult {
  executed: boolean;
  plan: BasePlanResponse;
  message: string;
}

export interface CurrentTcpPose {
  x_mm: number;
  y_mm: number;
  z_mm: number;
  yaw_deg: number;
  frame_id: "base" | string;
}

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface CaptureAxisState {
  slide_mm: number;
  z_mm: number;
  shoulder_deg: number;
  elbow_deg: number;
  rotation_deg: number;
}

export interface VisionTargetObservation {
  request_id: string;
  frame_id: string;
  timestamp: number | null;
  position_mm: Vector3;
  orientation: Quaternion | null;
  confidence: number | null;
  target_id: string | null;
  capture_axis_state: CaptureAxisState;
  capture_motion_state: "stationary" | "moving" | "unknown";
}

export interface VisionPlanResponse {
  request_id: string;
  camera: {
    frame_id: string;
    position_mm: Vector3;
    target_compensation_camera_mm: Vector3;
    confidence: number | null;
    timestamp: number | null;
    target_id: string | null;
    orientation: Quaternion | null;
  };
  capture_joint_state: CaptureAxisState;
  base: {
    frame_id: "base";
    raw_position_mm: Vector3;
    target_compensation_base_mm: Vector3;
    position_mm: Vector3;
    tool_camera_source: string;
    tool_camera_validated: boolean;
    transform_status: HandEyeCalibrationStatus;
  };
  planner: {
    succeeded: true;
    five_axis_solution: FiveAxisSolution | null;
    plan: BasePlanResponse;
  };
}

export interface PickPlan {
  observation: VisionTargetObservation;
  overhead_target: BaseTargetRequest;
  contact_target: BaseTargetRequest;
  lift_target: BaseTargetRequest;
  overhead_motion: BasePlanResponse;
  contact_motion: BasePlanResponse;
  lift_motion: BasePlanResponse;
  suction_settle_time_s: number;
}

export type PickOutcome =
  | "planned"
  | "motion_completed"
  | "suction_command_acknowledged"
  | "physical_pick_unverified"
  | "failed";

export interface PickResult {
  outcome: PickOutcome;
  observation: VisionTargetObservation | null;
  plan: PickPlan | null;
  message: string;
}

export interface ScanPositionResult {
  /** Fixed 1-based scan position index. Valid configured positions are 1 through 4. */
  scan_index: ScanPositionIndex;
  detected_count: number;
  picked_count: number;
  /**
   * Manual pick-one returns picked_and_placed_unverified, no_target, or
   * target_rejected:<error type>. Full scan-pick also uses its existing reasons.
   */
  final_reason: string;
}

/**
 * Response from POST /api/scan-positions/{scan_index}/pick-one and
 * POST /api/scan-pick. A picked count does not verify physical pickup because
 * the current system has no vacuum feedback.
 */
export interface ScanAndPickResult {
  result: string;
  visited_scan_positions: ScanPositionResult[];
  total_picked: number;
}

/** Normalize the current two Base planning response shapes for rendering. */
export function planStages(plan: BasePlanResponse): Array<BaseMoveStage | DemoStage> {
  return Array.isArray(plan) ? plan : plan.stages;
}

export function planStageName(stage: BaseMoveStage | DemoStage): string {
  return "kind" in stage ? stage.kind : stage.name;
}

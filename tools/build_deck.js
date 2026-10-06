/* build_deck.js — IntelliBot mid-term progress deck (pptxgenjs). */
const pptxgen = require("pptxgenjs");

const P = new pptxgen();
P.layout = "LAYOUT_WIDE"; // 13.33 x 7.5"
P.author = "Team IntelliBot";
P.title = "IntelliBot — Mid-Term Progress";

// ---------- palette (warehouse / industrial) ----------
const BG_DARK = "1F2937";   // charcoal slate (title + closing)
const BG = "FFFFFF";        // content background
const CARD = "F1F5F9";      // slate tint card
const PRIMARY = "334155";   // slate structural
const ACCENT = "EA580C";    // safety orange (sparing)
const TEXT = "1E293B";
const MUTED = "64748B";
const LIGHT = "E2E8F0";
const FONT = "Segoe UI";

const W = 13.33, H = 7.5, M = 0.5;
const shadow = () => ({ type: "outer", color: "000000", blur: 7, offset: 2, angle: 90, opacity: 0.14 });

// ---------- helpers ----------
function titleBar(s, kicker, title) {
  s.addText(kicker.toUpperCase(), { x: M, y: 0.34, w: 9, h: 0.3, fontSize: 12, fontFace: FONT, bold: true, color: ACCENT, charSpacing: 3, margin: 0 });
  s.addText(title, { x: M, y: 0.62, w: W - 2 * M, h: 0.75, fontSize: 30, fontFace: FONT, bold: true, color: TEXT, margin: 0 });
}
function pageNo(s, n) {
  s.addText(String(n).padStart(2, "0"), { x: W - 0.85, y: H - 0.52, w: 0.5, h: 0.3, fontSize: 11, fontFace: FONT, color: MUTED, align: "right", margin: 0 });
}
function card(s, x, y, w, h, fill = CARD) {
  s.addShape(P.shapes.ROUNDED_RECTANGLE, { x, y, w, h, fill: { color: fill }, rectRadius: 0.07, line: { type: "none" }, shadow: shadow() });
}
function bulletList(s, items, x, y, w, h, opts = {}) {
  const arr = items.map((t, i) => ({
    text: t, options: { bullet: { code: "25B8", indent: 12 }, color: opts.color || TEXT, breakLine: true },
  }));
  s.addText(arr, { x, y, w, h, fontSize: opts.fontSize || 14, fontFace: FONT, paraSpaceAfter: opts.gap ?? 8, margin: 0, valign: "top", align: "left" });
}

// ============================================================ 1 TITLE
{
  const s = P.addSlide();
  s.background = { color: BG_DARK };
  // motif: warehouse route arc — orange path with nodes (bottom-right, clear of cards)
  s.addShape(P.shapes.LINE, { x: 10.0, y: 6.45, w: 2.4, h: 0, line: { color: ACCENT, width: 3.5 } });
  s.addShape(P.shapes.LINE, { x: 12.4, y: 6.45, w: 0, h: -0.3, line: { color: ACCENT, width: 3.5 } });
  s.addShape(P.shapes.OVAL, { x: 9.91, y: 6.36, w: 0.18, h: 0.18, fill: { color: ACCENT } });
  s.addShape(P.shapes.OVAL, { x: 12.25, y: 5.97, w: 0.3, h: 0.3, fill: { color: "67E8F9" } });
  s.addText("P3", { x: 9.55, y: 6.55, w: 0.6, h: 0.28, fontSize: 11, fontFace: FONT, bold: true, color: "93C5FD", margin: 0 });
  s.addText("ZONE B", { x: 11.6, y: 5.5, w: 1.6, h: 0.28, fontSize: 11, fontFace: FONT, bold: true, color: "67E8F9", align: "right", margin: 0 });

  s.addText("ROBOTIC INTELLIGENCE — MID-TERM REVIEW", { x: M, y: 1.15, w: 10, h: 0.35, fontSize: 13, fontFace: FONT, bold: true, color: ACCENT, charSpacing: 4, margin: 0 });
  s.addText("IntelliBot", { x: M, y: 1.55, w: 11, h: 1.15, fontSize: 66, fontFace: FONT, bold: true, color: "FFFFFF", margin: 0 });
  s.addText("Autonomous warehouse pick & delivery — perceive, plan, control in a closed loop", { x: M, y: 2.75, w: 9.5, h: 0.55, fontSize: 20, fontFace: FONT, color: LIGHT, margin: 0 });
  s.addText("Task: pick package P3 (blue)  →  deliver to Zone B (cyan)   ·   Webots + Python, software-only simulation", { x: M, y: 3.4, w: 10.5, h: 0.4, fontSize: 14, fontFace: FONT, color: "94A3B8", margin: 0 });

  const stats = [["14", "Python modules"], ["63", "unit tests passing"], ["100%", "P0 code complete"], ["0", "hard-coded routes"]];
  stats.forEach(([n, l], i) => {
    const x = M + i * 2.6;
    s.addShape(P.shapes.ROUNDED_RECTANGLE, { x, y: 4.6, w: 2.35, h: 1.25, fill: { color: "273449" }, rectRadius: 0.08, line: { type: "none" } });
    s.addText(n, { x, y: 4.72, w: 2.35, h: 0.6, fontSize: 34, fontFace: FONT, bold: true, color: i === 1 ? ACCENT : "FFFFFF", align: "center", margin: 0 });
    s.addText(l, { x, y: 5.36, w: 2.35, h: 0.35, fontSize: 12, fontFace: FONT, color: "94A3B8", align: "center", margin: 0 });
  });
  s.addText("Team of 3 · build status & evidence · October 2026", { x: M, y: 6.75, w: 8, h: 0.35, fontSize: 12, fontFace: FONT, color: MUTED, margin: 0 });
  s.addNotes("Open with the mission one-liner; the four chips frame the talk: everything is built and unit-tested, live simulation runs are the remaining step.");
}

// ============================================================ 2 MISSION
{
  const s = P.addSlide();
  s.background = { color: BG };
  titleBar(s, "The task", "One mission, seven closed-loop capabilities");
  s.addText([
    { text: "\u201CPick package P3 and deliver it to Zone B.\u201D", options: { italic: true, color: TEXT } },
  ], { x: M, y: 1.55, w: 12.3, h: 0.55, fontSize: 22, fontFace: FONT, margin: 0 });
  s.addText("Every step below is sensed, decided and controlled at runtime — a hard-coded route is explicitly not acceptable.", { x: M, y: 2.15, w: 12, h: 0.4, fontSize: 14, fontFace: FONT, color: MUTED, margin: 0 });

  const steps = [
    ["1", "FIND", "camera detects the blue package (HSV)"],
    ["2", "PLAN", "A* over the inflated occupancy grid"],
    ["3", "NAVIGATE", "PID waypoint following"],
    ["4", "AVOID", "LiDAR safety stop, 360° scan"],
    ["5", "PICK", "approach & stand off (attach: final)"],
    ["6", "DELIVER", "re-plan and drive to Zone B"],
    ["7", "REPORT", "time, path length, replans"],
  ];
  const cw = 1.72, gap = 0.12, x0 = M, y0 = 2.85;
  steps.forEach(([n, t, d], i) => {
    const x = x0 + i * (cw + gap);
    s.addShape(P.shapes.CHEVRON, { x, y: y0, w: cw, h: 0.62, fill: { color: i === 6 ? ACCENT : PRIMARY }, line: { type: "none" } });
    s.addText(`${n} · ${t}`, { x: x + 0.08, y: y0 + 0.06, w: cw - 0.25, h: 0.5, fontSize: 12.5, fontFace: FONT, bold: true, color: "FFFFFF", align: "center", valign: "middle", margin: 0 });
    s.addText(d, { x: x + 0.05, y: y0 + 0.78, w: cw - 0.1, h: 1.0, fontSize: 11, fontFace: FONT, color: MUTED, align: "center", valign: "top", margin: 0 });
  });

  card(s, M, 5.0, 12.33, 1.85);
  s.addText("Why this is \u201Crobotic intelligence\u201D", { x: 0.85, y: 5.18, w: 11.5, h: 0.35, fontSize: 15, fontFace: FONT, bold: true, color: ACCENT, margin: 0 });
  bulletList(s, [
    "Package position comes from the robot's own camera + LiDAR estimate — not a coordinate we typed in.",
    "The route is planned by A* on an occupancy grid built from the same config that generates the world.",
    "Motion is closed-loop: every 16 ms the controller re-commands (v, \u03C9) from the latest pose and scan.",
  ], 0.85, 5.55, 11.6, 1.2, { gap: 5 });
  pageNo(s, 2);
  s.addNotes("Walk the chevron left to right; each step maps to a syllabus topic (vision, planning, control, locomotion, manipulation).");
}

// ============================================================ 3 ARCHITECTURE
{
  const s = P.addSlide();
  s.background = { color: BG };
  titleBar(s, "Methodology", "Software architecture — Webots-free core, thin device layer");

  const nodes = [
    ["Sensors", "LiDAR · camera · GPS/IMU · encoders", 0.5, 1.55],
    ["perception.py", "device wrappers BGRA\u2192BGR, ray angles", 2.25, 2.05],
    ["decision_maker.py", "FSM: mission states & goals", 4.5, 2.05],
    ["waypoint_follower.py", "WaypointFollower: PID heading", 6.75, 2.05],
    ["kinematics.py", "(v, \u03C9) \u2192 wheel speeds", 9.0, 2.05],
    ["Motors", "left/right + encoders", 11.25, 1.58],
  ];
  const y0 = 2.0, nh = 1.05;
  nodes.forEach(([t, d, x, w], i) => {
    s.addShape(P.shapes.ROUNDED_RECTANGLE, { x, y: y0, w, h: nh, fill: { color: i === 2 ? ACCENT : PRIMARY }, rectRadius: 0.06, line: { type: "none" }, shadow: shadow() });
    s.addText(t, { x: x + 0.08, y: y0 + 0.12, w: w - 0.16, h: 0.35, fontSize: 13, fontFace: FONT, bold: true, color: "FFFFFF", align: "center", margin: 0 });
    s.addText(d, { x: x + 0.08, y: y0 + 0.47, w: w - 0.16, h: 0.52, fontSize: 9.5, fontFace: FONT, color: LIGHT, align: "center", valign: "top", margin: 0 });
    if (i < nodes.length - 1) {
      const nx = nodes[i + 1][2];
      s.addShape(P.shapes.LINE, { x: x + w, y: y0 + nh / 2, w: nx - x - w, h: 0, line: { color: MUTED, width: 2, endArrowType: "triangle" } });
    }
  });

  // planner + grid row (feedback path)
  s.addShape(P.shapes.ROUNDED_RECTANGLE, { x: 4.5, y: 4.15, w: 2.1, h: 1.05, fill: { color: PRIMARY }, rectRadius: 0.06, line: { type: "none" }, shadow: shadow() });
  s.addText("a_star.py + path_utils", { x: 4.58, y: 4.27, w: 1.94, h: 0.35, fontSize: 12.5, fontFace: FONT, bold: true, color: "FFFFFF", align: "center", margin: 0 });
  s.addText("A* on occupancy grid \u2192 waypoints", { x: 4.58, y: 4.62, w: 1.94, h: 0.5, fontSize: 9.5, fontFace: FONT, color: LIGHT, align: "center", valign: "top", margin: 0 });
  s.addShape(P.shapes.LINE, { x: 5.55, y: y0 + nh, w: 0, h: 4.15 - y0 - nh, line: { color: MUTED, width: 2, endArrowType: "triangle" } });
  s.addText("called by the FSM only when a new goal is set", { x: 6.9, y: 4.5, w: 4.6, h: 0.3, fontSize: 11, fontFace: FONT, italic: true, color: MUTED, margin: 0 });

  card(s, M, 5.65, 12.33, 1.35);
  s.addText("Design rules", { x: 0.85, y: 5.8, w: 3, h: 0.3, fontSize: 14, fontFace: FONT, bold: true, color: ACCENT, margin: 0 });
  s.addText([
    { text: "Pure-Python core (kinematics, PID, grid, A*, detector, FSM) — unit-tested with pytest, no Webots imports.   ", options: { color: TEXT } },
    { text: "config.py is the single source of numbers — the world file is GENERATED from it, so simulator and planner can never disagree.", options: { color: TEXT } },
  ], { x: 0.85, y: 6.12, w: 11.6, h: 0.8, fontSize: 13, fontFace: FONT, margin: 0 });
  pageNo(s, 3);
  s.addNotes("Orange = the FSM that owns the mission. Emphasize: only perception.py and the entry point import Webots; all logic is pytest-verified.");
}

// ============================================================ 4 WHAT WE BUILT
{
  const s = P.addSlide();
  s.background = { color: BG };
  titleBar(s, "Build status", "Everything P0 is coded, modular and unit-tested");

  const cols = [
    ["A — Simulation & Control", ["generate_world.py \u2192 warehouse.wbt", "custom diff-drive robot (LiDAR, camera, GPS, IMU, encoders)", "kinematics.py · pid.py", "waypoint_follower.py — WaypointFollower", "drive_test: sequence / kinematics / odometry"]],
    ["B — Perception", ["perception.py — device wrappers", "detector.py — HSV, red two-range, zone-disjoint", "estimation.py — pixel \u2192 bearing \u2192 range \u2192 world", "LiDAR sector queries + safety stop", "eval_detector.py + frame capture pipeline"]],
    ["C — Planning & Decisions", ["mapping.py — grid, circular inflation", "a_star.py — octile, no corner cutting + Dijkstra", "path_utils.py — LOS simplify, resample", "decision_maker.py — full FSM + mission report", "intellibot_controller.py — integration loop"]],
  ];
  const cw = 3.98, gap = 0.2, y0 = 1.7;
  cols.forEach(([t, items], i) => {
    const x = M + i * (cw + gap);
    card(s, x, y0, cw, 3.6);
    s.addText(t, { x: x + 0.22, y: y0 + 0.18, w: cw - 0.44, h: 0.35, fontSize: 15, fontFace: FONT, bold: true, color: TEXT, margin: 0 });
    bulletList(s, items, x + 0.22, y0 + 0.62, cw - 0.44, 2.85, { fontSize: 12, gap: 7 });
  });

  const chips = [["3,476", "lines of Python"], ["10", "offline + sim tools"], ["7", "architecture docs"], ["1", "generated world file"]];
  chips.forEach(([n, l], i) => {
    const x = M + i * 3.14;
    s.addText(n, { x, y: 5.6, w: 1.6, h: 0.65, fontSize: 36, fontFace: FONT, bold: true, color: i === 0 ? ACCENT : PRIMARY, margin: 0 });
    s.addText(l, { x, y: 6.28, w: 2.9, h: 0.3, fontSize: 12, fontFace: FONT, color: MUTED, margin: 0 });
  });
  pageNo(s, 4);
  s.addNotes("Each column is one future owner. Every module has a 'how to explain it' paragraph in docs/architecture.md.");
}

// ============================================================ 5 WORLD
{
  const s = P.addSlide();
  s.background = { color: BG };
  titleBar(s, "Simulation environment", "An 8 \u00D7 6 m warehouse, generated — never hand-drawn");

  // image right (world_layout.png is 10x7.5 ratio ~1.333)
  const iw = 6.6, ih = iw / (1500 / 1125);
  s.addShape(P.shapes.ROUNDED_RECTANGLE, { x: 6.2, y: 1.55, w: iw + 0.16, h: ih + 0.16, fill: { color: "FFFFFF" }, rectRadius: 0.05, line: { color: LIGHT, width: 1 }, shadow: shadow() });
  s.addImage({ path: "docs/evidence/world_layout.png", x: 6.28, y: 1.63, w: iw, h: ih });

  bulletList(s, [
    "worlds/warehouse.wbt is written by tools/generate_world.py from config.py — the map the planner uses IS the world.",
    "4 shelves, 2 crates, pickup station, packages P1\u2013P3 (pure red/green/blue), Zone A (yellow) / Zone B (cyan).",
    "Flat lighting, shadows OFF \u2014 keeps HSV detection stable.",
    "Packages are physics solids; floor patches are non-colliding.",
    "Constraint verified by test: every aisle \u2265 1.0 m and TWO distinct routes exist to Zone B (needed for the final re-planning demo).",
  ], M, 1.85, 5.4, 4.4, { fontSize: 14, gap: 12 });
  s.addText("Layout rendered from config.py (the same numbers that build the .wbt and the occupancy grid).", { x: M, y: 6.55, w: 5.6, h: 0.55, fontSize: 11, fontFace: FONT, italic: true, color: MUTED, margin: 0 });
  pageNo(s, 5);
  s.addNotes("Point at the robot start and the two corridors on the right side of the plan: center aisle and outer corridor.");
}

// ============================================================ 6 PLANNING VALIDATED
{
  const s = P.addSlide();
  s.background = { color: BG };
  titleBar(s, "Preliminary results", "A* planning on the real warehouse grid — verified optimal");

  const iw = 7.3, ih = iw * (1125 / 1500);
  s.addShape(P.shapes.ROUNDED_RECTANGLE, { x: M, y: 1.6, w: iw + 0.16, h: ih + 0.16, fill: { color: "FFFFFF" }, rectRadius: 0.05, line: { color: LIGHT, width: 1 }, shadow: shadow() });
  s.addImage({ path: "docs/evidence/astar_pickup_to_zoneB.png", x: M + 0.08, y: 1.68, w: iw, h: ih });

  const rx = 8.35, rw = 4.5;
  const stats = [
    ["3.06 m", "simplified path, start \u2192 Zone B"],
    ["1,849", "A* expansions (~14 ms, pure Python)"],
    ["== Dijkstra", "cost matched on 60 random grids"],
    ["2 routes", "path still found with center aisle blocked"],
  ];
  stats.forEach(([n, l], i) => {
    const y = 1.65 + i * 1.28;
    card(s, rx, y, rw, 1.12);
    s.addText(n, { x: rx + 0.25, y: y + 0.12, w: 2.3, h: 0.55, fontSize: 26, fontFace: FONT, bold: true, color: i === 2 ? ACCENT : PRIMARY, margin: 0 });
    s.addText(l, { x: rx + 0.25, y: y + 0.65, w: rw - 0.5, h: 0.4, fontSize: 11.5, fontFace: FONT, color: MUTED, margin: 0 });
  });
  s.addText("Grid: 80 \u00D7 60 cells @ 0.1 m · circular inflation 0.25 m · octile heuristic · corner cutting forbidden.", { x: rx, y: 6.85, w: rw, h: 0.5, fontSize: 11, fontFace: FONT, color: MUTED, margin: 0 });
  pageNo(s, 6);
  s.addNotes("The figure is produced by tools/plot_astar.py — regenerated any time config changes. Orange dots = raw A* cells; blue = simplified waypoints.");
}

// ============================================================ 7 PERCEPTION & CONTROL
{
  const s = P.addSlide();
  s.background = { color: BG };
  titleBar(s, "Methodology", "Perception and control — the closed loop's senses and reflexes");

  const cards = [
    ["Vision: HSV package detector", "Per-color masks in HSV (red = two hue ranges); morphology cleans the mask; largest plausible contour wins. Zone hues (yellow \u2248 30\u00B0, cyan \u2248 90\u00B0) are outside every package window \u2014 confusion is test-asserted impossible."],
    ["Pixel \u2192 world position", "\u03C6 = \u2212atan2(u \u2212 W/2, f\u209A\u2093)  \u2192  d = LiDAR min at \u03C6 (\u00B13\u00B0)  \u2192  x\u1D65 = pose + d\u00B7(cos(\u03B8+\u03C6), sin(\u03B8+\u03C6)). Pinhole fallback d = f\u209A\u2093\u00B7size/bbox when no return."],
    ["Waypoint follower (PID)", "\u03C9 = PID(heading error), clamped \u00B11.5 rad/s;  v = V\u2098\u2090\u2093\u00B7cos(err)\u00B7min(1, d/slowdown); rotate in place beyond 0.6 rad; waypoints reached < 0.10 m."],
    ["LiDAR safety reflex", "Any ray within \u00B130\u00B0 of forward < 0.30 m \u2192 v = 0, status BLOCKED. This flag is the exact hook the final-eval dynamic re-planner will consume."],
  ];
  const cw2 = 6.05, ch = 1.78, gx = 0.23;
  cards.forEach(([t, d], i) => {
    const x = M + (i % 2) * (cw2 + gx), y = 1.65 + Math.floor(i / 2) * (ch + 0.25);
    card(s, x, y, cw2, ch);
    s.addText(t, { x: x + 0.24, y: y + 0.13, w: cw2 - 0.48, h: 0.32, fontSize: 14.5, fontFace: FONT, bold: true, color: ACCENT, margin: 0 });
    s.addText(d, { x: x + 0.24, y: y + 0.48, w: cw2 - 0.48, h: ch - 0.6, fontSize: 11.5, fontFace: FONT, color: TEXT, margin: 0, valign: "top" });
  });

  // FSM strip
  const states = ["IDLE", "FIND_PACKAGE", "NAVIGATE", "PICK", "PLAN_DELIVERY", "DELIVER", "DONE"];
  const sw = 1.62, sg = 0.14, sy = 5.85;
  s.addText("Mission FSM (unit-tested end-to-end with a simulated unicycle):", { x: M, y: 5.42, w: 9, h: 0.3, fontSize: 13, fontFace: FONT, bold: true, color: TEXT, margin: 0 });
  states.forEach((t, i) => {
    const x = M + i * (sw + sg);
    s.addShape(P.shapes.ROUNDED_RECTANGLE, { x, y: sy, w: sw, h: 0.52, fill: { color: i === 3 ? "FDBA74" : CARD }, rectRadius: 0.06, line: { color: i === 3 ? ACCENT : LIGHT, width: 1 } });
    s.addText(t, { x, y: sy, w: sw, h: 0.52, fontSize: 10.5, fontFace: FONT, bold: true, color: TEXT, align: "center", valign: "middle", margin: 0 });
    if (i < states.length - 1) s.addText("\u25B8", { x: x + sw - 0.02, y: sy + 0.1, w: 0.2, h: 0.3, fontSize: 11, color: MUTED, margin: 0 });
  });
  s.addText("Every transition prints \u201C[t=12.3s] STATE: NAVIGATE \u2192 PICK\u201D and lands in the run log.", { x: M, y: 6.55, w: 11, h: 0.35, fontSize: 11.5, fontFace: FONT, italic: true, color: MUTED, margin: 0 });
  pageNo(s, 7);
  s.addNotes("If asked about the detector score: it is a documented heuristic (solidity x size), not a statistical confidence.");
}

// ============================================================ 8 VERIFICATION STATUS
{
  const s = P.addSlide();
  s.background = { color: BG };
  titleBar(s, "Validation & challenges", "What is proven today vs. what the live runs must measure");

  card(s, M, 1.65, 6.05, 4.6, "ECFDF5");
  s.addText("Verified offline — pytest, 63/63 green", { x: 0.75, y: 1.85, w: 5.6, h: 0.35, fontSize: 15, fontFace: FONT, bold: true, color: "047857", margin: 0 });
  bulletList(s, [
    "Kinematics: round-trip, curvature-preserving clamping, odometry on a known circle",
    "A* cost == Dijkstra on 60 random grids; corner-cutting impossible",
    "Real layout: pickup \u2192 Zone B path + two distinct routes",
    "FSM happy path IDLE \u2192 \u2026 \u2192 DONE incl. mission report; ERROR paths",
    "Detector: correct colors/bboxes; zone hues never misfired",
    "Re-plan hooks: blocked route detected \u2192 alternate route planned",
  ], 0.75, 2.3, 5.55, 3.8, { fontSize: 12.5, gap: 9 });

  card(s, 6.8, 1.65, 6.05, 4.6, "FFF7ED");
  s.addText("Pending live Webots runs (exact commands in README)", { x: 7.05, y: 1.85, w: 5.7, h: 0.35, fontSize: 15, fontFace: FONT, bold: true, color: ACCENT, margin: 0 });
  bulletList(s, [
    "Kinematics error % — commanded vs GPS/IMU finite differences (drive_test)",
    "Odometry drift vs GPS on line / square / arc",
    "LiDAR ray-order calibration (left-only obstacle) + range accuracy at 0.5/1/2 m",
    "Detector rates on REAL frames + lighting variants",
    "Cross-track error, success rate over \u226510 runs, screen recording",
  ], 7.05, 2.3, 5.55, 3.0, { fontSize: 12.5, gap: 9 });
  s.addText([
    { text: "Challenge hit & fixed: ", options: { bold: true, color: ACCENT } },
    { text: "the first anti-windup scheme (back-calculation with k\u1D62 = 0.05) produced \u00B140 integral jumps and stalled the heading controller; replaced with conditional integration — follower then completed multi-waypoint paths.", options: { color: TEXT } },
  ], { x: 7.05, y: 5.0, w: 5.6, h: 1.1, fontSize: 11.5, fontFace: FONT, margin: 0 });

  s.addText("No simulation result is claimed without being observed — the build machine has no Webots install, so sim-dependent numbers stay honestly pending.", { x: M, y: 6.55, w: 12.3, h: 0.5, fontSize: 12, fontFace: FONT, italic: true, color: MUTED, margin: 0 });
  pageNo(s, 8);
  s.addNotes("Honesty is the point of this slide: green column is proven by tests, orange column lists exactly what the operator runs produce next.");
}

// ============================================================ 9 TEAM & NEXT STEPS
{
  const s = P.addSlide();
  s.background = { color: BG };
  titleBar(s, "Team & roadmap", "Three owners, one integration — and what comes next");

  const team = [
    ["A — Simulation & Control", "World, robot, kinematics, drive tests, PID + follower tuning", "kinematics · pid · controller · drive_test"],
    ["B — Perception", "LiDAR calibration, camera pipeline, detector evaluation", "perception · detector · estimation · eval tools"],
    ["C — Planning & Decisions", "Grid, A*, FSM, integration, logging, docs", "mapping · a_star · path_utils · decision_maker"],
  ];
  const cw = 3.98, y0 = 1.7;
  team.forEach(([t, d, m], i) => {
    const x = M + i * (cw + 0.2);
    card(s, x, y0, cw, 2.0);
    s.addText(t, { x: x + 0.22, y: y0 + 0.16, w: cw - 0.44, h: 0.35, fontSize: 14.5, fontFace: FONT, bold: true, color: PRIMARY, margin: 0 });
    s.addText(d, { x: x + 0.22, y: y0 + 0.55, w: cw - 0.44, h: 0.8, fontSize: 12, fontFace: FONT, color: TEXT, margin: 0, valign: "top" });
    s.addText(m, { x: x + 0.22, y: y0 + 1.45, w: cw - 0.44, h: 0.45, fontSize: 10.5, fontFace: FONT, italic: true, color: MUTED, margin: 0, valign: "top" });
  });

  s.addText("Road to the final evaluation", { x: M, y: 4.05, w: 8, h: 0.4, fontSize: 17, fontFace: FONT, bold: true, color: TEXT, margin: 0 });
  const road = [
    ["Now", "Run the live validation suite; capture evidence (recordings, metrics)"],
    ["Next", "Tune PID gains on the real robot; \u226510-run success-rate campaign"],
    ["Final", "Dynamic obstacle \u2192 BLOCKED \u2192 live re-plan (hooks already tested)"],
    ["Final", "Real pickup via connector/supervisor attach; collision counter; dashboard"],
  ];
  road.forEach(([tag, d], i) => {
    const y = 4.55 + i * 0.62;
    s.addShape(P.shapes.ROUNDED_RECTANGLE, { x: M, y, w: 1.05, h: 0.44, fill: { color: i < 2 ? ACCENT : PRIMARY }, rectRadius: 0.06, line: { type: "none" } });
    s.addText(tag, { x: M, y, w: 1.05, h: 0.44, fontSize: 11.5, fontFace: FONT, bold: true, color: "FFFFFF", align: "center", valign: "middle", margin: 0 });
    s.addText(d, { x: 1.75, y, w: 11, h: 0.44, fontSize: 13.5, fontFace: FONT, color: TEXT, valign: "middle", margin: 0 });
  });
  pageNo(s, 9);
  s.addNotes("Each member presents their column; docs/qa_prep.md has 29 ready answers for the Q&A.");
}

// ============================================================ 10 CLOSING
{
  const s = P.addSlide();
  s.background = { color: BG_DARK };
  s.addText("STATUS", { x: M, y: 1.3, w: 6, h: 0.35, fontSize: 13, fontFace: FONT, bold: true, color: ACCENT, charSpacing: 4, margin: 0 });
  s.addText("Built. Tested. Ready to drive.", { x: M, y: 1.7, w: 12, h: 1.0, fontSize: 48, fontFace: FONT, bold: true, color: "FFFFFF", margin: 0 });
  const takeaways = [
    "Complete P0 codebase: world generator, custom robot, planner, controller, FSM — 63 unit tests green.",
    "Planning proven optimal against Dijkstra; the warehouse provably offers two routes for the re-planning demo.",
    "Remaining work is measurement, not construction: live Webots runs fill the evidence tables, then the final-eval features plug into tested hooks.",
  ];
  takeawayList(s, takeaways);
  function takeawayList(sl, items) {
    const arr = items.map((t) => ({ text: t, options: { bullet: { code: "25B8", indent: 14 }, color: LIGHT, breakLine: true } }));
    sl.addText(arr, { x: M, y: 3.1, w: 11.5, h: 2.4, fontSize: 17, fontFace: FONT, paraSpaceAfter: 16, margin: 0, valign: "top" });
  }
  s.addText("RI-Project  ·  worlds/warehouse.wbt  ·  python -m pytest tests/  ·  docs/demo_script.md", { x: M, y: 6.6, w: 12, h: 0.4, fontSize: 12, fontFace: FONT, color: MUTED, margin: 0 });
  s.addNotes("Close on: the demo either runs live or we play the pre-recorded run — both show the same FSM console output.");
}

P.writeFile({ fileName: "IntelliBot_MidTerm_Review.pptx" }).then(() => console.log("deck written"));

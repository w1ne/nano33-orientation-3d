import * as THREE from "three";

// ---------- scene ----------
const mount = document.getElementById("scene");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
mount.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0e14);

const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.1, 100);
camera.position.set(4.5, 3.2, 5.5);
camera.lookAt(0, 0, 0);

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const key = new THREE.DirectionalLight(0xffffff, 1.1);
key.position.set(5, 8, 6);
scene.add(key);
const rim = new THREE.DirectionalLight(0x6688ff, 0.5);
rim.position.set(-6, 2, -4);
scene.add(rim);

// reference grid (the "world" floor)
const grid = new THREE.GridHelper(12, 24, 0x2a3550, 0x1a2235);
grid.position.y = -1.6;
scene.add(grid);

// ---------- board model ----------
// A recognizable little PCB so orientation is obvious from any angle.
const board = new THREE.Group();

const pcb = new THREE.Mesh(
  new THREE.BoxGeometry(4.0, 0.12, 1.8),
  new THREE.MeshStandardMaterial({ color: 0x1f7a3d, roughness: 0.6, metalness: 0.1 })
);
board.add(pcb);

// MCU chip (offset so the board is visually asymmetric => rotation is readable)
const chip = new THREE.Mesh(
  new THREE.BoxGeometry(0.9, 0.14, 0.9),
  new THREE.MeshStandardMaterial({ color: 0x111418, roughness: 0.4 })
);
chip.position.set(0.4, 0.13, 0);
board.add(chip);

// USB connector at one end (marks the "front")
const usb = new THREE.Mesh(
  new THREE.BoxGeometry(0.5, 0.28, 0.7),
  new THREE.MeshStandardMaterial({ color: 0xc0c6cc, roughness: 0.3, metalness: 0.8 })
);
usb.position.set(-2.05, 0.05, 0);
board.add(usb);

// pin header rows along both long edges
const pinMat = new THREE.MeshStandardMaterial({ color: 0xd8a93a, metalness: 0.7, roughness: 0.35 });
for (let i = 0; i < 14; i++) {
  for (const z of [-0.78, 0.78]) {
    const pin = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.18, 0.1), pinMat);
    pin.position.set(-1.7 + i * 0.26, -0.09, z);
    board.add(pin);
  }
}

// body-fixed axis arrows (X red, Y green, Z blue) so the board's own frame is visible
function arrow(dir, color) {
  return new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0, 0), 1.6, color, 0.4, 0.22);
}
board.add(arrow(new THREE.Vector3(1, 0, 0), 0xff5566)); // +X
board.add(arrow(new THREE.Vector3(0, 1, 0), 0x55dd66)); // +Y
board.add(arrow(new THREE.Vector3(0, 0, 1), 0x5599ff)); // +Z

// Sensor frame (Z up) -> Three.js frame (Y up): rotate the whole rig -90° about X.
const rig = new THREE.Group();
rig.rotation.x = -Math.PI / 2;
rig.add(board);
scene.add(rig);

// ---------- minimal orbit controls (no extra deps) ----------
let orbit = { theta: 0.7, phi: 1.0, radius: 7.6, drag: false, px: 0, py: 0 };
function applyCamera() {
  const r = orbit.radius, p = orbit.phi, t = orbit.theta;
  camera.position.set(r * Math.sin(p) * Math.sin(t), r * Math.cos(p), r * Math.sin(p) * Math.cos(t));
  camera.lookAt(0, 0, 0);
}
applyCamera();
renderer.domElement.addEventListener("pointerdown", (e) => { orbit.drag = true; orbit.px = e.clientX; orbit.py = e.clientY; });
addEventListener("pointerup", () => { orbit.drag = false; });
addEventListener("pointermove", (e) => {
  if (!orbit.drag) return;
  orbit.theta -= (e.clientX - orbit.px) * 0.01;
  orbit.phi = Math.max(0.15, Math.min(Math.PI - 0.15, orbit.phi - (e.clientY - orbit.py) * 0.01));
  orbit.px = e.clientX; orbit.py = e.clientY;
  applyCamera();
});
renderer.domElement.addEventListener("wheel", (e) => {
  e.preventDefault();
  orbit.radius = Math.max(3, Math.min(20, orbit.radius + e.deltaY * 0.01));
  applyCamera();
}, { passive: false });

// ---------- orientation data ----------
// Madgwick quaternion is (w, x, y, z); Three.js is (x, y, z, w).
const targetQ = new THREE.Quaternion(0, 0, 0, 1);
let yawOffset = new THREE.Quaternion(0, 0, 0, 1); // zero out yaw drift on demand
renderer.domElement.addEventListener("dblclick", () => {
  // capture current yaw and cancel it
  const e = new THREE.Euler().setFromQuaternion(targetQ, "YXZ");
  yawOffset.setFromAxisAngle(new THREE.Vector3(0, 1, 0), -e.y);
});

function setQuaternion(q0, q1, q2, q3) {
  targetQ.set(q1, q2, q3, q0).normalize();
}

// ---------- HUD ----------
const el = (id) => document.getElementById(id);
const sparkCtx = el("spark").getContext("2d");
const SPARK_N = 120;
const hist = new Array(SPARK_N).fill(0);

function updateHud(gyroMag) {
  const e = new THREE.Euler().setFromQuaternion(targetQ, "YXZ");
  const deg = (r) => (r * 180 / Math.PI);
  el("roll").textContent = deg(e.z).toFixed(1) + "°";
  el("pitch").textContent = deg(e.x).toFixed(1) + "°";
  el("yaw").textContent = deg(e.y).toFixed(1) + "°";
  el("rate").textContent = gyroMag.toFixed(1) + " °/s";

  // stability: steady when angular rate is small
  const STEADY = 3, MOVING = 40;          // deg/s thresholds
  const frac = Math.min(1, gyroMag / MOVING);
  const fill = el("barFill");
  fill.style.width = (frac * 100).toFixed(0) + "%";
  let color, label;
  if (gyroMag < STEADY) { color = "#3ddc84"; label = "STEADY"; }
  else if (gyroMag < MOVING) { color = "#e6b400"; label = "MOVING"; }
  else { color = "#d1495b"; label = "FAST"; }
  fill.style.background = color;
  const ss = el("stabState"); ss.textContent = label; ss.style.color = color;

  // sparkline
  hist.push(gyroMag); hist.shift();
  const w = el("spark").width, h = el("spark").height;
  sparkCtx.clearRect(0, 0, w, h);
  sparkCtx.strokeStyle = color; sparkCtx.lineWidth = 1.5; sparkCtx.beginPath();
  const max = Math.max(MOVING, ...hist);
  hist.forEach((v, i) => {
    const x = (i / (SPARK_N - 1)) * w;
    const y = h - (v / max) * (h - 4) - 2;
    i ? sparkCtx.lineTo(x, y) : sparkCtx.moveTo(x, y);
  });
  sparkCtx.stroke();
}

// ---------- websocket ----------
let lastGyro = 0;
function connect() {
  const ws = new WebSocket(`ws://${location.hostname}:8765`);
  ws.onopen = () => { el("dot").classList.add("on"); el("statusText").textContent = "board connected"; };
  ws.onclose = () => { el("dot").classList.remove("on"); el("statusText").textContent = "reconnecting…"; setTimeout(connect, 1500); };
  ws.onmessage = (ev) => {
    const d = JSON.parse(ev.data);
    setQuaternion(d.q[0], d.q[1], d.q[2], d.q[3]);
    lastGyro = d.gyroMag;
  };
}
connect();

// ---------- render loop ----------
const smoothed = new THREE.Quaternion(0, 0, 0, 1);
const corrected = new THREE.Quaternion();
function tick() {
  corrected.copy(yawOffset).multiply(targetQ);
  smoothed.slerp(corrected, 0.35);          // gentle smoothing for silky motion
  board.quaternion.copy(smoothed);
  updateHud(lastGyro);
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}
tick();

addEventListener("resize", () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

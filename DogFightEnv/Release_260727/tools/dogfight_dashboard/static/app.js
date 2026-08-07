import { initTrainingDashboard } from "./training.js";
import { initReplayViewer } from "./replay.js";

const AppState = {
  trainingStarted: false,
  replayStarted: false,
  config: null,
};

const $ = id => document.getElementById(id);

start().catch(error => {
  console.error(error);
  $("app-status").textContent = `Startup failed: ${error.message}`;
});

async function start() {
  bindTabs();
  const params = new URLSearchParams(window.location.search);
  AppState.config = await fetchJson(withQuery("/api/app/config", params));
  bindPathControls();
  loadPathCandidates().catch(error => {
    console.error(error);
    setPathStatus(`Candidates failed: ${error.message}`);
  });
  const initialTab = params.get("tab") || AppState.config.defaultTab || "training";
  await showTab(initialTab === "replay" ? "replay" : "training");
  updateAppStatus();
}

function bindTabs() {
  for (const button of document.querySelectorAll(".tab-button")) {
    button.addEventListener("click", () => {
      showTab(button.dataset.tab).catch(error => {
        console.error(error);
        $("app-status").textContent = `Tab failed: ${error.message}`;
      });
    });
  }
}

function bindPathControls() {
  $("training-logdir-input").value =
    AppState.config.trainingDisplay || AppState.config.trainingLogdir || "";
  $("replay-logdir-input").value =
    AppState.config.replayDisplay || AppState.config.replayLogdir || "";
  $("apply-paths").addEventListener("click", () => {
    applyPathSelection().catch(error => {
      console.error(error);
      setPathStatus(`Path failed: ${error.message}`);
    });
  });
  $("reset-paths").addEventListener("click", resetPathSelection);
  setPathStatus("Ready");
}

async function loadPathCandidates() {
  const paths = await fetchJson("/api/app/paths");
  fillDatalist("training-logdir-options", paths.trainingCandidates || []);
  fillDatalist("replay-logdir-options", paths.replayCandidates || []);
}

function fillDatalist(id, candidates) {
  const datalist = $(id);
  datalist.textContent = "";
  for (const candidate of candidates) {
    const option = document.createElement("option");
    option.value = candidate.path;
    option.label = candidate.absolutePath || candidate.path;
    datalist.appendChild(option);
  }
}

async function applyPathSelection() {
  setPathStatus("Checking...");
  const params = buildPathParams();
  const result = await fetchJson(withQuery("/api/app/validate-paths", params));
  setPathStatus(`Reloading: ${result.replayDisplay}`);
  window.location.href = `${window.location.pathname}?${params.toString()}`;
}

function resetPathSelection() {
  const params = new URLSearchParams();
  params.set("tab", activeTab());
  window.location.href = `${window.location.pathname}?${params.toString()}`;
}

function buildPathParams() {
  const params = new URLSearchParams(window.location.search);
  params.set("tab", activeTab());
  removePathParams(params);
  const trainingLogdir = $("training-logdir-input").value.trim();
  const replayLogdir = $("replay-logdir-input").value.trim();
  if (trainingLogdir) {
    params.set("trainingLogdir", trainingLogdir);
  }
  if (replayLogdir) {
    params.set("replayLogdir", replayLogdir);
  }
  return params;
}

function removePathParams(params) {
  for (const key of [
    "trainingLogdir",
    "training-logdir",
    "training_logdir",
    "replayLogdir",
    "replay-logdir",
    "replay_logdir",
    "logdir",
  ]) {
    params.delete(key);
  }
}

async function showTab(tab) {
  for (const button of document.querySelectorAll(".tab-button")) {
    button.classList.toggle("active", button.dataset.tab === tab);
  }
  $("training-panel").classList.toggle("active", tab === "training");
  $("replay-panel").classList.toggle("active", tab === "replay");
  const params = new URLSearchParams(window.location.search);
  params.set("tab", tab);
  window.history.replaceState(null, "", `?${params.toString()}`);

  if (tab === "training" && !AppState.trainingStarted) {
    initTrainingDashboard({ apiBase: "/api/training" });
    AppState.trainingStarted = true;
  }
  if (tab === "replay" && !AppState.replayStarted) {
    await initReplayViewer({ apiBase: "/api/replay" });
    AppState.replayStarted = true;
  }
  window.dispatchEvent(new Event("resize"));
}

async function fetchJson(path) {
  const response = await fetch(path, { cache: "no-store" });
  const data = await response.json();
  if (!response.ok) {
    throw new Error(data.error || response.statusText);
  }
  return data;
}

function withQuery(path, params) {
  const query = params.toString();
  return query ? `${path}?${query}` : path;
}

function activeTab() {
  return document.querySelector(".tab-button.active")?.dataset.tab || "training";
}

function updateAppStatus() {
  $("app-status").textContent = formatPaths(AppState.config);
}

function setPathStatus(message) {
  $("path-status").textContent = message;
}

function formatPaths(config) {
  const training = config.trainingDisplay || config.trainingLogdir;
  const replay = config.replayDisplay || config.replayLogdir;
  return `Training: ${training} | Replay: ${replay}`;
}

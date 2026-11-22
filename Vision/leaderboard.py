"""
Leaderboard for the maze game: results store, ranking, and a small web server.

The tracker owns the timing. This module keeps the results in a JSON file and
serves two pages:

  /         public board for the portrait monitor (view only)
  /manage   overseer page: delete, restore and rename teams (this computer only)
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

TITLE = "SENSORY DRIFT"
RUN_LIMIT_S = 300.0               # a run ends after five minutes
COUNTDOWN_S = 3.0                 # 3, 2, 1 before the clock starts
ARRIVAL_DWELL_S = 1.0             # the car must stay in the goal ring this long to finish
MAX_TEAM_NAME = 20
DEFAULT_PORT = 8000
BOARD_PATH = Path(__file__).with_name("leaderboard.json")
LOOPBACK = ("127.0.0.1", "::1", "::ffff:127.0.0.1")


def format_time(seconds: float) -> str:
    seconds = max(0, int(seconds))
    return f"{seconds // 60}:{seconds % 60:02d}"


def ordinal(n: int) -> str:
    if 10 <= n % 100 <= 20:
        return f"{n}th"
    return f"{n}{ {1: 'st', 2: 'nd', 3: 'rd'}.get(n % 10, 'th') }"


class Leaderboard:
    """Results on disk. Finishers rank by time, then everyone else by distance to the goal."""

    def __init__(self, path: Path = BOARD_PATH):
        self.path = Path(path)
        self.lock = threading.Lock()
        self.entries: list[dict] = []
        self._load()

    def _load(self) -> None:
        try:
            data = json.loads(self.path.read_text())
            self.entries = list(data.get("entries", []))
        except (OSError, ValueError):
            self.entries = []

    def _save(self) -> None:
        try:
            self.path.write_text(json.dumps({"entries": self.entries}, indent=2) + "\n")
        except OSError as exc:
            print(f"Could not save the leaderboard: {exc}")

    # ---- queries -------------------------------------------------------
    @staticmethod
    def _sort_key(entry: dict):
        if entry["finished"]:
            return (0, entry["time_s"])
        distance = entry.get("distance_mm")
        return (1, distance if distance is not None else float("inf"))

    def ranked(self) -> list[dict]:
        """Active entries, best first, each with its place and display text."""
        with self.lock:
            active = sorted((e for e in self.entries if not e.get("removed")), key=self._sort_key)
            return [dict(e, place=i + 1, result=self.result_text(e)) for i, e in enumerate(active)]

    def removed(self) -> list[dict]:
        with self.lock:
            return [dict(e, result=self.result_text(e)) for e in self.entries if e.get("removed")]

    @staticmethod
    def result_text(entry: dict) -> str:
        if entry["finished"]:
            return format_time(entry["time_s"])
        if entry.get("distance_mm") is None:
            return "no position"
        return f"{entry['distance_mm']:.0f} mm away"

    def name_taken(self, team: str, exclude_id: int | None = None) -> bool:
        wanted = team.strip().casefold()
        with self.lock:
            return any(e["team"].casefold() == wanted and not e.get("removed") and e["id"] != exclude_id
                       for e in self.entries)

    # ---- changes -------------------------------------------------------
    def add(self, team: str, finished: bool, time_s: float, distance_mm: float | None) -> dict:
        with self.lock:
            entry = {
                "id": max((e["id"] for e in self.entries), default=0) + 1,
                "team": team.strip()[:MAX_TEAM_NAME],
                "finished": bool(finished),
                "time_s": round(float(time_s), 1),
                "distance_mm": None if distance_mm is None else round(float(distance_mm), 1),
                "when": time.strftime("%Y-%m-%d %H:%M:%S"),
                "removed": False,
            }
            self.entries.append(entry)
            self._save()
        return entry

    def _find(self, entry_id: int) -> dict | None:
        return next((e for e in self.entries if e["id"] == entry_id), None)

    def delete(self, entry_id: int) -> tuple[bool, str]:
        """Hide a team from the board. The entry stays in the file and can be restored."""
        with self.lock:
            entry = self._find(entry_id)
            if entry is None:
                return False, "No such entry."
            entry["removed"] = True
            self._save()
            return True, f"Removed {entry['team']}."

    def restore(self, entry_id: int) -> tuple[bool, str]:
        with self.lock:
            entry = self._find(entry_id)
            if entry is None:
                return False, "No such entry."
        if self.name_taken(entry["team"], exclude_id=entry_id):
            return False, f"Another team is already called {entry['team']}. Rename one first."
        with self.lock:
            entry["removed"] = False
            self._save()
            return True, f"Restored {entry['team']}."

    def rename(self, entry_id: int, team: str) -> tuple[bool, str]:
        team = team.strip()[:MAX_TEAM_NAME]
        if not team:
            return False, "The name is empty."
        if self.name_taken(team, exclude_id=entry_id):
            return False, f"Another team is already called {team}."
        with self.lock:
            entry = self._find(entry_id)
            if entry is None:
                return False, "No such entry."
            entry["team"] = team
            self._save()
            return True, f"Renamed to {team}."

    def undo_last(self) -> dict | None:
        """Remove the most recent active entry (for a false result). Returns it, or None."""
        with self.lock:
            active = [e for e in self.entries if not e.get("removed")]
            if not active:
                return None
            entry = max(active, key=lambda e: e["id"])
            entry["removed"] = True
            self._save()
            return dict(entry)

    def describe(self, entry_id: int) -> dict:
        """Place and a one-line comparison for a result, for the result card."""
        ranked = self.ranked()
        mine = next((e for e in ranked if e["id"] == entry_id), None)
        if mine is None:
            return {}
        place = mine["place"]
        if mine["finished"]:
            if place == 1:
                detail = "Fastest time" if len(ranked) > 1 else ""
            else:
                ahead = ranked[place - 2]
                gap = mine["time_s"] - ahead["time_s"]
                detail = f"{gap:.0f} seconds behind {ordinal(place - 1)}"
        elif mine.get("distance_mm") is None:
            detail = "The car was not in view"
        else:
            detail = f"{mine['distance_mm']:.0f} mm from the goal"
        return {"place": place, "place_text": f"{ordinal(place)} place", "detail": detail,
                "result": mine["result"], "finished": mine["finished"]}


class LiveState:
    """What the tracker is doing right now, shared with the web server thread."""

    def __init__(self):
        self.lock = threading.Lock()
        self.data = {"phase": "idle", "team": None, "elapsed_s": 0.0, "remaining_s": RUN_LIMIT_S,
                     "distance_mm": None, "closest_mm": None, "result": None, "countdown": 0}

    def set(self, **values) -> None:
        with self.lock:
            self.data.update(values)

    def snapshot(self) -> dict:
        with self.lock:
            return dict(self.data)


def build_state(board: Leaderboard, live: LiveState) -> dict:
    return {"title": TITLE, "limit_s": RUN_LIMIT_S,
            "live": live.snapshot(), "entries": board.ranked()}


# ---------------------------------------------------------------------------
# Pages
# ---------------------------------------------------------------------------
PUBLIC_PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
  :root { --bg:#0d1017; --card:#161b24; --line:#232a36; --text:#e8ebf0; --muted:#8b93a1;
          --gold:#ffd700; --green:#7ee787; --u: min(1vw, 0.5625vh); }
  * { box-sizing: border-box; }
  html, body { height: 100%; }
  body { margin: 0; background: var(--bg); color: var(--text); overflow: hidden;
         font-family: system-ui, -apple-system, "Segoe UI", sans-serif; }
  main { width: calc(var(--u) * 100); height: 100vh; margin: 0 auto; padding: calc(var(--u) * 5);
         display: flex; flex-direction: column; }
  h1 { font-size: calc(var(--u) * 7.2); font-weight: 800; margin: 0 0 calc(var(--u) * 4); letter-spacing: 0.02em; color: #fff; }
  .card { background: var(--card); border: 1px solid var(--line); border-radius: calc(var(--u) * 3.2);
          padding: calc(var(--u) * 4.5); min-height: calc(var(--u) * 54); }
  .card.done { background: #12301c; border: 2px solid #2f8f4e; text-align: center; }
  .card.out { background: #2b1d14; border: 2px solid #9a6a2f; text-align: center; }
  .label { font-size: calc(var(--u) * 2.5); font-weight: 700; letter-spacing: 0.14em; color: var(--muted); }
  .team { font-size: calc(var(--u) * 6.2); font-weight: 800; color: var(--gold); margin: calc(var(--u) * 1.6) 0;
          white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  .done .team, .out .team { color: #fff; }
  .clock { display: flex; align-items: flex-end; gap: calc(var(--u) * 5); }
  .time { font: 700 calc(var(--u) * 16)/1 ui-monospace, Menlo, monospace; color: #fff; }
  .done .time, .out .time { font-size: calc(var(--u) * 17); color: var(--gold); margin: calc(var(--u) * 2) 0; }
  .left small { display: block; font-size: calc(var(--u) * 2.6); color: var(--muted); }
  .left b { font: 700 calc(var(--u) * 4.4)/1.2 ui-monospace, Menlo, monospace; color: #c9ced6; }
  .bar { height: calc(var(--u) * 1.8); background: var(--line); border-radius: 99px; margin: calc(var(--u) * 3) 0; overflow: hidden; }
  .bar i { display: block; height: 100%; background: var(--gold); border-radius: 99px; width: 0; }
  .dist { display: flex; justify-content: space-between; align-items: flex-end; }
  .dist small { display: block; font-size: calc(var(--u) * 2.6); color: var(--muted); }
  .dist b { font: 700 calc(var(--u) * 5.2)/1.2 ui-monospace, Menlo, monospace; color: var(--green); }
  .dist span { font-size: calc(var(--u) * 2.6); color: var(--muted); }
  .count { font: 800 calc(var(--u) * 30)/1 ui-monospace, Menlo, monospace; color: var(--gold); text-align: center;
           margin-top: calc(var(--u) * 2); }
  .place { font-size: calc(var(--u) * 4.2); font-weight: 700; color: #fff; }
  .detail { font-size: calc(var(--u) * 2.8); color: #a9d9b8; margin-top: calc(var(--u) * 1); }
  .out .detail { color: #e0c49a; }
  .idle { font-size: calc(var(--u) * 4.4); font-weight: 700; color: #fff; margin-top: calc(var(--u) * 14); text-align: center; }
  .idle small { display: block; font-size: calc(var(--u) * 2.8); font-weight: 400; color: var(--muted); margin-top: calc(var(--u) * 1.5); }
  h2 { font-size: calc(var(--u) * 2.6); letter-spacing: 0.14em; color: var(--muted); margin: calc(var(--u) * 5) 0 calc(var(--u) * 2); }
  ol { list-style: none; margin: 0; padding: 0; flex: 1; overflow: hidden; }
  li { display: flex; align-items: center; gap: calc(var(--u) * 3); background: var(--card); border: 1px solid var(--line);
       border-radius: calc(var(--u) * 1.8); padding: calc(var(--u) * 1.9) calc(var(--u) * 3.4); margin-bottom: calc(var(--u) * 1.4); }
  li .n { font: 700 calc(var(--u) * 3.6)/1 ui-monospace, Menlo, monospace; color: var(--muted); width: calc(var(--u) * 6); }
  li .t { font-size: calc(var(--u) * 3.6); font-weight: 600; flex: 1; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  li .r { font: 700 calc(var(--u) * 3.6)/1 ui-monospace, Menlo, monospace; color: var(--green); }
  li .r.d { font-size: calc(var(--u) * 2.8); color: #c9ced6; }
  li.hi { background: var(--gold); border-color: var(--gold); }
  li.hi .n, li.hi .t, li.hi .r { color: #1c1c1f; }
  .empty { color: var(--muted); font-size: calc(var(--u) * 3); padding: calc(var(--u) * 3) 0; }
</style></head>
<body><main>
  <h1 id="title"></h1>
  <div id="card" class="card"></div>
  <h2>LEADERBOARD</h2>
  <ol id="list"></ol>
</main>
<script>
const esc = s => String(s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const mmss = s => { s = Math.max(0, Math.floor(s)); return Math.floor(s / 60) + ':' + String(s % 60).padStart(2, '0'); };
function render(st) {
  document.getElementById('title').textContent = st.title;
  const live = st.live, card = document.getElementById('card');
  card.className = 'card';
  if (live.phase === 'running') {
    const d = live.distance_mm == null ? '—' : Math.round(live.distance_mm) + ' mm';
    const c = live.closest_mm == null ? '' : 'closest so far ' + Math.round(live.closest_mm) + ' mm';
    card.innerHTML = `<div class="label">NOW DRIVING</div><div class="team">${esc(live.team)}</div>
      <div class="clock"><div class="time">${mmss(live.elapsed_s)}</div>
      <div class="left"><small>time left</small><b>${mmss(Math.ceil(live.remaining_s))}</b></div></div>
      <div class="bar"><i style="width:${Math.min(100, 100 * live.elapsed_s / st.limit_s)}%"></i></div>
      <div class="dist"><div><small>distance to goal</small><b>${d}</b></div><span>${c}</span></div>`;
  } else if ((live.phase === 'finished' || live.phase === 'timeout') && live.result) {
    const r = live.result, done = live.phase === 'finished';
    card.className = 'card ' + (done ? 'done' : 'out');
    card.innerHTML = `<div class="label" style="color:${done ? 'var(--green)' : '#e0a95a'}">${done ? 'GOAL REACHED' : "TIME'S UP"}</div>
      <div class="team">${esc(live.team)}</div><div class="time">${esc(done ? r.result : r.result.replace(' away', ''))}</div>
      <div class="place">${esc(r.place_text || '')}</div><div class="detail">${esc(r.detail || '')}</div>`;
  } else if (live.phase === 'countdown') {
    card.innerHTML = `<div class="label">STARTING</div><div class="team">${esc(live.team)}</div>
      <div class="count">${live.countdown}</div>`;
  } else if (live.phase === 'ready') {
    card.innerHTML = `<div class="label">UP NEXT</div><div class="team">${esc(live.team)}</div>
      <div class="clock"><div class="time">0:00</div><div class="left"><small>time limit</small><b>${mmss(st.limit_s)}</b></div></div>
      <div class="bar"><i></i></div><div class="dist"><div><small>status</small><b style="color:#c9ced6">Waiting to start</b></div></div>`;
  } else {
    card.innerHTML = `<div class="idle">Waiting for the next team<small>Time limit ${mmss(st.limit_s)}</small></div>`;
  }
  const hi = (live.phase === 'finished' || live.phase === 'timeout') && live.result ? live.result.id : null;
  const list = document.getElementById('list');
  list.innerHTML = st.entries.length ? st.entries.slice(0, 10).map(e =>
    `<li class="${e.id === hi ? 'hi' : ''}"><span class="n">${e.place}</span><span class="t">${esc(e.team)}</span>
     <span class="r ${e.finished ? '' : 'd'}">${esc(e.result)}</span></li>`).join('')
    : '<div class="empty">No runs yet.</div>';
  // show only the rows that fit completely on the screen
  while (list.scrollHeight > list.clientHeight + 1 && list.children.length > 1) list.lastElementChild.remove();
}
const initial = __STATE__;
if (initial) render(initial);
async function tick() {
  try { const r = await fetch('/api/state', {cache: 'no-store'}); if (r.ok) render(await r.json()); } catch (e) {}
}
if (location.protocol.startsWith('http')) { tick(); setInterval(tick, 250); }
</script></body></html>
"""

MANAGE_PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__: manage leaderboard</title>
<style>
  body { margin: 0; padding: 24px; background: #f4f5f7; color: #1f2328; font: 15px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif; }
  main { max-width: 760px; margin: 0 auto; }
  h1 { font-size: 22px; margin: 0 0 4px; } h2 { font-size: 16px; margin: 28px 0 8px; }
  p { margin: 0 0 12px; color: #57606a; }
  table { width: 100%; border-collapse: collapse; background: #fff; border: 1px solid #d8dce1; border-radius: 10px; overflow: hidden; }
  th, td { text-align: left; padding: 9px 12px; border-bottom: 1px solid #eceff2; }
  th { font-size: 13px; color: #57606a; } tr:last-child td { border-bottom: 0; }
  td.num { font-family: ui-monospace, Menlo, monospace; }
  button { font: inherit; padding: 5px 12px; border-radius: 6px; border: 1px solid #c9ced6; background: #fff; cursor: pointer; margin-left: 6px; }
  button.danger { border-color: #d1242f; color: #d1242f; } button:hover { background: #f0f2f5; }
  #msg { min-height: 24px; font-weight: 600; margin: 10px 0; }
  .none { padding: 14px 12px; color: #57606a; background: #fff; border: 1px solid #d8dce1; border-radius: 10px; }
</style></head>
<body><main>
  <h1>Manage the leaderboard</h1>
  <p>Changes show on the public board straight away. This page only opens on the computer running the tracker.</p>
  <div id="msg"></div>
  <h2>On the board</h2><div id="active"></div>
  <h2>Removed</h2><div id="removed"></div>
</main>
<script>
const esc = s => String(s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
async function post(path, body) {
  const r = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
  const j = await r.json().catch(() => ({ok: false, message: 'Request failed.'}));
  const m = document.getElementById('msg'); m.textContent = j.message || ''; m.style.color = j.ok ? '#1a7f37' : '#d1242f';
  load();
}
function del(id, team) { if (confirm('Remove ' + team + ' from the leaderboard?')) post('/api/delete', {id}); }
function ren(id, team) { const n = prompt('New name for ' + team + ':', team); if (n !== null) post('/api/rename', {id, team: n}); }
async function load() {
  const j = await (await fetch('/api/manage', {cache: 'no-store'})).json();
  document.getElementById('active').innerHTML = j.active.length ? '<table><tr><th>Place</th><th>Team</th><th>Result</th><th>When</th><th></th></tr>' +
    j.active.map(e => `<tr><td class="num">${e.place}</td><td>${esc(e.team)}</td><td class="num">${esc(e.result)}</td><td>${esc(e.when)}</td>
      <td style="text-align:right"><button data-a="ren" data-id="${e.id}" data-team="${esc(e.team)}">Rename</button><button class="danger" data-a="del" data-id="${e.id}" data-team="${esc(e.team)}">Delete</button></td></tr>`).join('') + '</table>'
    : '<div class="none">No results yet.</div>';
  document.getElementById('removed').innerHTML = j.removed.length ? '<table><tr><th>Team</th><th>Result</th><th>When</th><th></th></tr>' +
    j.removed.map(e => `<tr><td>${esc(e.team)}</td><td class="num">${esc(e.result)}</td><td>${esc(e.when)}</td>
      <td style="text-align:right"><button data-a="res" data-id="${e.id}">Restore</button></td></tr>`).join('') + '</table>'
    : '<div class="none">Nothing has been removed.</div>';
}
document.addEventListener('click', ev => {
  const b = ev.target.closest('button[data-a]'); if (!b) return;
  const id = Number(b.dataset.id);
  if (b.dataset.a === 'del') del(id, b.dataset.team);
  else if (b.dataset.a === 'ren') ren(id, b.dataset.team);
  else post('/api/restore', {id});
});
load();
</script></body></html>
"""


def public_page(state: dict | None = None) -> str:
    """The public board. With `state` given, the page renders it without needing the server."""
    initial = "null" if state is None else json.dumps(state).replace("</", "<\\/")
    return PUBLIC_PAGE.replace("__TITLE__", TITLE.title()).replace("__STATE__", initial)


# ---------------------------------------------------------------------------
# Web server
# ---------------------------------------------------------------------------
def make_handler(board: Leaderboard, live: LiveState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):       # keep the tracker's terminal quiet
            pass

        def _send(self, status: int, body: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, payload, status: int = 200) -> None:
            self._send(status, json.dumps(payload).encode(), "application/json")

        def _local(self) -> bool:
            return self.client_address[0] in LOOPBACK

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/":
                self._send(200, public_page().encode(), "text/html; charset=utf-8")
            elif path == "/api/state":
                self._json(build_state(board, live))
            elif path in ("/manage", "/api/manage"):
                if not self._local():
                    self._send(403, b"The manage page only opens on the computer running the tracker.",
                               "text/plain; charset=utf-8")
                elif path == "/manage":
                    page = MANAGE_PAGE.replace("__TITLE__", TITLE.title())
                    self._send(200, page.encode(), "text/html; charset=utf-8")
                else:
                    self._json({"active": board.ranked(), "removed": board.removed()})
            else:
                self._send(404, b"Not found", "text/plain; charset=utf-8")

        def do_POST(self):
            if not self._local():
                self._json({"ok": False, "message": "Only allowed on the tracker's computer."}, 403)
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length) or b"{}")
                entry_id = int(body.get("id"))
            except (ValueError, TypeError):
                self._json({"ok": False, "message": "Bad request."}, 400)
                return
            if self.path == "/api/delete":
                ok, message = board.delete(entry_id)
            elif self.path == "/api/restore":
                ok, message = board.restore(entry_id)
            elif self.path == "/api/rename":
                ok, message = board.rename(entry_id, str(body.get("team", "")))
            else:
                self._json({"ok": False, "message": "Not found."}, 404)
                return
            self._json({"ok": ok, "message": message})

    return Handler


def start_server(board: Leaderboard, live: LiveState, port: int = DEFAULT_PORT):
    """Serve the board in a background thread. Returns the server, or None if the port is taken."""
    try:
        server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(board, live))
    except OSError as exc:
        print(f"Leaderboard page not started (port {port}): {exc}")
        return None
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server

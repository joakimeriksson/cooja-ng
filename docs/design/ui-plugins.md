# UI Plugins — contributing a panel to the live web UI

This is the general design for letting a **plugin** draw something in the live
WebSocket UI (`ui/index.html`), added as the v3 increment of the plugin ABI
(`include/sim/csim_plugin.h`). The worked example is the energy estimator
(`src/services/energest_{engine,service}.c`), whose duty-cycle/energy panel is
the UI counterpart to what `packet_sink` is for the headless service ABI.

Both delivery forms publish panels the same way: a dynamic `.so` calls
`api->ui->publish_panel` (the v3 vtable), and a compiled-in built-in calls the
identical-signature kernel function `sim_runtime_ui_publish_panel` directly. The
shared `energest_engine` takes whichever as a sink, so the panel code is written
once.

## The one idea

A plugin **publishes named JSON panels**; the host **transports** them to the
browser over the existing WebSocket; a **single generic renderer** in the page
draws them. The page knows nothing plugin-specific — any plugin that publishes a
panel shows up with zero front-end changes.

```
  observer events  ─▶  plugin  ─▶  ui->publish_panel(sim, "id", json)
                                        │  (stored in sim_runtime: id → latest json)
                                        ▼
  websocket_ui_service, each broadcast tick:
        {"type":"panels","panels":{ "id": <json>, ... }}  ──▶  browser
                                        │
                          renderPluginPanels()  (generic; one box per id)
```

Note the symmetry with how a plugin *consumes* the simulation: structured data
flows **in** as observer events, structured data flows **out** as UI panels.

## The contract (what a plugin author learns)

One call, gated on the v3 capability:

```c
if (api->version >= 3u && api->ui && api->ui->publish_panel)
    /* remember it; call from on_event/destroy with the sim you were handed */
    api->ui->publish_panel(sim, "energest", panel_json);
```

`panel_json` is a small JSON object the generic renderer understands:

```json
{ "title": "Energy (CC2420 @3V)",
  "rows": [ ["mote 0", "duty 100.0%  ~1565 mJ"],
            ["mote 1", "duty  98.8%  ~1043 mJ"] ] }
```

or, for free-form text:

```json
{ "title": "...", "text": "line 1\nline 2" }
```

Rules:
- `id` keys the panel; re-publishing the same `id` **replaces** it (so you push
  a fresh snapshot, you don't diff).
- Publish from `on_event` (or `destroy`). **Do not** rely on `poll()` — the
  service host only polls when a serial/external-command child is active, so a
  plugin's `poll()` won't fire in a normal run. `on_event` always fans out.
- **Throttle.** `on_event` can fire often; rebuild the panel JSON at most every
  ~250 ms of sim time (energest throttles on the event timestamp).
- Keep it small. The panel store holds up to `SIM_RUNTIME_MAX_PANELS` (8)
  panels; each value is a compact object.

## Why it's safe / additive

- **Version-gated**, append-only ABI: `csim_ui_ops` is a new `csim_api` field;
  a plugin built for v3 checks `api->version >= 3` before touching it, and a v1
  plugin (e.g. `packet_sink`) is unaffected. A pre-v3 host leaves `api->ui`
  NULL, and the plugin degrades to its headless behaviour.
- **Byte-identical when idle**: nothing publishes unless a plugin does, and the
  `{"type":"panels"}` frame is only built when a UI client is connected. No
  plugin / headless / no-client runs are unchanged (verified by cross-build
  empty-diff on sky + cc2538).
- **No new transport**: panels ride the existing WebSocket as a small text
  frame, independent of the full-state JSON and the CBOR delta paths — so the
  hot serialization path is untouched.

## Where the pieces live

| Concern | File |
|---|---|
| ABI (`csim_ui_ops`, `api->ui`, version bump to 3) | `include/sim/csim_plugin.h` |
| Panel store + `sim_runtime_ui_publish_panel` / `_panels_json` | `include/sim/sim_runtime.h`, `src/sim/sim_runtime.c` |
| Wiring the sink into the api handed to plugins | `src/sim/sim_plugin.c` (`g_ui_ops`) |
| Broadcasting the `{"type":"panels"}` frame | `src/services/websocket_ui_service.c` (+ the `rt` handle) |
| Generic renderer + overlay | `ui/index.html` (`renderPluginPanels`, `#plugin-panels`) |
| Example producer | `src/services/energest_engine.c` (+ `energest_service.c`) |

`ui/index.html` is served from disk by the UI service, so front-end tweaks need
no rebuild. The renderer is the *only* page change needed for the whole
mechanism — every future plugin reuses it.

## Adding a UI plugin (recipe)

1. In `csim_plugin_init`, record `g_have_ui = (api->version >= 3 && api->ui &&
   api->ui->publish_panel)`.
2. From `on_event`, build a `{title, rows|text}` object (throttled) and call
   `g_api.ui->publish_panel(sim, "<your-id>", json)`.
3. Run with the UI: `./build/test_runner test <cfg> --ui --plugin <your.so>`
   (or config v2 `"plugins": [...]`), open the browser — your panel appears
   over the topology canvas.
4. Verify headlessly without a browser: `CSIM_DUMP_PANELS=1 ./build/test_runner
   ... --plugin <your.so>` prints the published panels to stderr at end-of-run.

## Limitations / future

- **Output only.** Panels are display-only; there is no panel→plugin command
  path yet. A symmetric `{"type":"panel-cmd","id":...}` inbound message routed
  back to the plugin would add interactivity (buttons, toggles).
- **Generic rendering.** The page renders `rows`/`text` generically. A plugin
  that wants bespoke rendering (charts, custom widgets) would need the larger
  "plugin ships a web component served at `/plugins/<id>/ui.js`" model — a
  deliberate future step, not this simple path.
- **One sink.** Panels target the web UI. A headless CSV/file sink for the same
  `publish_panel` data would be a small addition (the data is already
  structured and host-owned).

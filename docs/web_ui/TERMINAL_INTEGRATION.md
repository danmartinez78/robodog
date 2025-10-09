# ShadowHound Advanced Terminal Integration

The advanced terminal component upgrades the ShadowHound mission dashboard with an xterm.js-powered console that supports command history, auto-completion, keyboard shortcuts, and rich, color-coded responses. This guide explains how to connect the component to your backend and customize its behavior.

## Features

- **xterm.js core** with 1000-line scrollback and ANSI color support
- **Command history** via <kbd>↑</kbd>/<kbd>↓</kbd>
- **Auto-completion** with contextual suggestions on <kbd>Tab</kbd>
- **Keyboard shortcuts**: <kbd>Ctrl</kbd>+<kbd>L</kbd> to clear, <kbd>Ctrl</kbd>+<kbd>C</kbd> to interrupt
- **Built-in stubs** for mission diagnostics (`skills`, `status`, `nodes`, `topics`, `map`, `teleop`, `battery`, `uptime`, `stop`, `cancel`, `help`, `clear`)
- **Placeholder mission helpers** (`mission.pause`, `mission.resume`, `log.tail`)
- **WebSocket bridge** to `/ws/terminal` for live command exchange
- **Toast notifications** for auto-complete feedback and hints

## File Layout

| File | Purpose |
| --- | --- |
| `static/terminal.js` | Terminal manager module that wraps xterm.js and manages socket lifecycle |
| `static/terminal.css` | Stylesheet with dark theme, status badge, shortcut legend, and toast styling |
| `dashboard_template.html` | Example integration showing how to mount the terminal on the dashboard |
| `docs/web_ui/terminal_demo.html` | Stand-alone demo that works without a backend by simulating responses |

## Prerequisites

1. Ensure your static assets are served from `/static/` (default Flask/FastAPI behavior).
2. Provide a WebSocket endpoint at `/ws/terminal` that accepts JSON payloads:
   ```json
   { "command": "status" }
   ```
   and emits responses shaped as:
   ```json
   { "type": "info", "message": "Robot status: READY" }
   ```
   The `type` field accepts `info`, `warn`, or `error` and controls terminal coloring.
3. Host the component on pages that allow ES modules (modern browsers only).

## Integration Steps

1. **Include the stylesheet inside `<head>`**:
   ```html
   <link rel="stylesheet" href="/static/terminal.css">
   ```

2. **Add the terminal panel markup**:
   ```html
   <div class="panel terminal-panel">
     <h2>TERMINAL</h2>
     <div class="terminal-toolbar">
       <span id="terminalStatus" class="terminal-status is-disconnected">DISCONNECTED</span>
       <div class="terminal-shortcuts">
         <span><span class="kbd">↑</span>/<span class="kbd">↓</span> History</span>
         <span><span class="kbd">Tab</span> Complete</span>
         <span><span class="kbd">Ctrl</span><span class="kbd">L</span> Clear</span>
         <span><span class="kbd">Ctrl</span><span class="kbd">C</span> Interrupt</span>
       </div>
     </div>
     <div id="terminalMount"></div>
   </div>
   ```

3. **Instantiate the manager with ES module import** (near the end of the `<body>`):
   ```html
   <script type="module">
     import TerminalManager, { DEFAULT_COMMANDS } from '/static/terminal.js';

     const terminalManager = new TerminalManager({
       container: document.getElementById('terminalMount'),
       statusIndicator: document.getElementById('terminalStatus'),
       prompt: 'shadowhound@mission-control:~$ ',
       autocompleteCommands: DEFAULT_COMMANDS,
       builtInCommands: {
         'mission.resume': { type: 'info', message: 'Mission resume acknowledged (placeholder)' },
         'mission.pause': { type: 'info', message: 'Mission pause acknowledged (placeholder)' }
       }
     });

     window.shadowhoundTerminal = terminalManager; // optional global access
   </script>
   ```

4. **Forward status and log messages** (optional):
   ```javascript
   function addTerminalLine(text, type = 'info') {
     if (window.shadowhoundTerminal) {
       window.shadowhoundTerminal.printResponse({ type, message: text });
     }
   }
   ```

## Customization

### Auto-Completion

Pass an `autocompleteCommands` array to the constructor. The helper removes duplicates automatically:
```javascript
new TerminalManager({
  autocompleteCommands: [
    ...DEFAULT_COMMANDS,
    'mission.inspect',
    'mission.abort'
  ]
});
```

### Built-in Responses

Provide additional stub commands or override defaults through `builtInCommands`.
```javascript
new TerminalManager({
  builtInCommands: {
    inspect: { type: 'info', message: 'Inspecting target (placeholder)' },
    teleop: { type: 'warn', message: 'Teleop request queued (placeholder override)' }
  }
});
```

### Command Hook

Attach a callback that intercepts outgoing commands before the WebSocket sends them. Return `true` to mark the command as fully handled.
```javascript
new TerminalManager({
  onCommand: (command, terminal) => {
    if (command === 'ping') {
      terminal.printResponse({ type: 'info', message: 'pong' });
      return true;
    }
    return false;
  }
});
```

### WebSocket Overrides

- `websocketUrl`: custom endpoint path.
- `socketFactory`: supply a mock or wrapped socket (used by the demo).
- `reconnectDelay`: milliseconds before reconnect attempt (default 4000).
- Set `autoConnect: false` to disable automatic WebSocket connection (useful for offline demos).

## Backend Expectations

1. **Inbound messages** are JSON objects with a `command` field. Additional metadata can be included.
2. **Outbound messages** must specify:
   - `type`: `info`, `warn`, or `error`.
   - `message`: string or array of strings.
   Optional fields such as `final: true` can be added later for flow control.
3. For long-running commands emit streaming updates as separate WebSocket messages.

## Testing Checklist

- Verify that pressing <kbd>Tab</kbd> suggests or completes known commands.
- Confirm <kbd>↑</kbd>/<kbd>↓</kbd> cycle through history.
- Trigger `clear` to wipe the display without reloading the page.
- Disconnect the WebSocket to observe the reconnection banner and warning toast.
- Try the standalone `docs/web_ui/terminal_demo.html` file to validate behavior without a backend.

## Troubleshooting

| Symptom | Resolution |
| --- | --- |
| Status badge stuck on **DISCONNECTED** | Ensure `/ws/terminal` is reachable and served over the same protocol as the page (wss/ws). |
| Commands do not echo back | Confirm the backend echoes JSON responses with `type` and `message`. |
| Auto-complete not working | Verify the command prefix is in `autocompleteCommands`. The toast will display available commands for empty input. |
| Copy/paste blocked | Ensure the browser does not intercept clipboard shortcuts; xterm.js supports <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>V</kbd> on Linux. |

---

The terminal module is intentionally backend-agnostic so it can be wired into REST, WebSocket, or even mocked transports. Extend `TerminalManager` through options rather than editing the core file to simplify future upgrades.

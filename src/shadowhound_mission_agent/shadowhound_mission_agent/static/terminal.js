import { Terminal } from 'https://cdn.jsdelivr.net/npm/xterm@5.3.0/+esm';
import { FitAddon } from 'https://cdn.jsdelivr.net/npm/xterm-addon-fit@0.8.0/+esm';

export const DEFAULT_COMMANDS = [
  'skills',
  'status',
  'nodes',
  'topics',
  'map',
  'teleop',
  'help',
  'clear',
  'stop',
  'cancel',
  'battery',
  'uptime'
];

const BUILTIN_RESPONSES = {
  clear: (command, terminal) => {
    terminal.clear();
    return { skipPrompt: true };
  },
  skills: {
    type: 'info',
    message: 'Available skills: nav.goto, nav.rotate, nav.dock, perception.scan, voice.notify'
  },
  battery: {
    type: 'info',
    message: 'Battery level: 87% (estimated 2h 15m remaining)'
  },
  uptime: {
    type: 'info',
    message: 'Mission agent uptime: 01:24:16'
  },
  stop: {
    type: 'warn',
    message: 'Emergency stop command queued (placeholder)'
  },
  cancel: {
    type: 'warn',
    message: 'Cancel request broadcast to active tasks (placeholder)'
  },
  status: {
    type: 'info',
    message: 'Robot status: READY'
  },
  nodes: {
    type: 'info',
    message: 'ROS2 nodes: [mission_agent, nav2_bt_navigator, localization, perception_stack]'
  },
  topics: {
    type: 'info',
    message: 'ROS2 topics: [/cmd_vel, /scan, /tf, /mission/status, /battery_state]'
  },
  map: {
    type: 'info',
    message: 'Current map: office_map'
  },
  teleop: {
    type: 'info',
    message: 'Teleop mode activated (placeholder)'
  },
  help: {
    type: 'info',
    message: 'Available commands: skills, status, nodes, topics, map, teleop, battery, uptime, stop, cancel, help, clear'
  }
};

const COLORS = {
  info: '\u001b[38;5;120m',
  warn: '\u001b[38;5;214m',
  error: '\u001b[38;5;203m',
  prompt: '\u001b[38;5;46m',
  accent: '\u001b[38;5;118m',
  reset: '\u001b[0m'
};

const ICONS = {
  info: '\u2139\ufe0f ',
  warn: '\u26a0\ufe0f ',
  error: '\u274c '
};

function resolveElement(reference) {
  if (!reference) {
    return null;
  }

  if (reference instanceof HTMLElement) {
    return reference;
  }

  if (typeof reference === 'string') {
    return document.getElementById(reference);
  }

  return null;
}

export class TerminalManager {
  constructor(options = {}) {
    this.prompt = options.prompt ?? 'shadowhound@mission-control:~$ ';
    this.container = resolveElement(options.container) ?? resolveElement(options.containerId);

    if (!this.container) {
      throw new Error('TerminalManager requires a container element or containerId');
    }

    this.statusIndicator = resolveElement(options.statusIndicator);
    this.autocompleteCommands = Array.from(
      new Set([...(options.autocompleteCommands ?? DEFAULT_COMMANDS)])
    );

    this.reconnectDelay = options.reconnectDelay ?? 4000;
    this.scrollback = options.scrollback ?? 1000;
    this.websocketUrl = options.websocketUrl ?? this.#inferWebsocketUrl();
    this.socketFactory = options.socketFactory ?? ((url) => (url ? new WebSocket(url) : null));
    this.onCommand = typeof options.onCommand === 'function' ? options.onCommand : null;

    const builtinOverrides = options.builtInCommands ?? {};
    this.builtInCommands = { ...BUILTIN_RESPONSES, ...builtinOverrides };

    this.currentInput = '';
    this.history = [];
    this.historyIndex = 0;
    this.isConnected = false;
    this.awaitingResponse = false;
    this.toastTimeout = null;

    this.fitAddon = new FitAddon();
    this.term = new Terminal({
      cursorBlink: true,
      allowTransparency: true,
      theme: {
        background: '#050b12',
        foreground: '#d4ffe9',
        cursor: '#4df2c2',
        selection: 'rgba(77, 242, 194, 0.25)'
      },
      fontFamily: "'IBM Plex Mono', 'Fira Code', 'Cascadia Code', monospace",
      fontSize: 14,
      scrollback: this.scrollback,
      disableStdin: false,
      convertEol: true
    });

    this.term.loadAddon(this.fitAddon);

    this.#mount();
    this.#initializeTerminal();

    if (options.autoConnect !== false) {
      this.connect();
    } else {
      this.#setStatus('OFFLINE', 'warn');
    }
  }

  connect() {
    if (!this.websocketUrl) {
      this.#setStatus('OFFLINE', 'warn');
      return;
    }

    this.#setStatus('CONNECTING...', 'warn');

    try {
      this.socket = this.socketFactory(this.websocketUrl);
    } catch (error) {
      console.error('[TerminalManager] Failed to open socket:', error);
      this.#setStatus('ERROR', 'error');
      this.#scheduleReconnect();
      return;
    }

    if (!this.socket) {
      this.#setStatus('OFFLINE', 'warn');
      return;
    }

    const onOpen = () => {
      this.isConnected = true;
      this.#setStatus('CONNECTED', 'info');
      this.printSystem('Connected to mission terminal.');
    };

    const onMessage = (event) => {
      this.#handleSocketMessage(event.data);
    };

    const onError = (event) => {
      console.error('[TerminalManager] Socket error:', event);
      this.#setStatus('ERROR', 'error');
    };

    const onClose = () => {
      this.isConnected = false;
      this.#setStatus('RECONNECTING...', 'warn');
      this.printSystem('Connection to terminal closed. Retrying...', 'warn');
      this.#scheduleReconnect();
    };

    if (typeof this.socket.addEventListener === 'function') {
      this.socket.addEventListener('open', onOpen);
      this.socket.addEventListener('message', onMessage);
      this.socket.addEventListener('error', onError);
      this.socket.addEventListener('close', onClose);
    } else {
      this.socket.onopen = onOpen;
      this.socket.onmessage = onMessage;
      this.socket.onerror = onError;
      this.socket.onclose = onClose;
    }
  }

  disconnect() {
    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
  }

  write(text) {
    this.term.write(text);
  }

  printSystem(message, type = 'info') {
    this.#renderMessage({ type, message });
  }

  printResponse(response) {
    this.#renderMessage(response);
  }

  executeCommand(command) {
    if (!command) {
      return;
    }

    const trimmed = command.trim();
    if (!trimmed) {
      return;
    }

    this.history.push(trimmed);
    this.historyIndex = this.history.length;

    if (this.builtInCommands[trimmed]) {
      const response = this.builtInCommands[trimmed];

      if (typeof response === 'function') {
        const result = response(trimmed, this);
        if (result && result.message) {
          this.#renderMessage(result);
        }
        if (!result || !result.skipPrompt) {
          this.#showPrompt();
        }
      } else {
        this.#renderMessage(response);
        this.#showPrompt();
      }

      return;
    }

    let handledByCallback = false;
    if (this.onCommand) {
      handledByCallback = this.onCommand(trimmed, this) === true;
    }

    if (!handledByCallback) {
      this.#sendToSocket(trimmed);
    }

    this.awaitingResponse = true;
  }

  clear() {
    this.term.clear();
    this.term.write(`${COLORS.prompt}${this.prompt}${COLORS.reset}`);
    this.currentInput = '';
    this.historyIndex = this.history.length;
  }

  #mount() {
    this.container.classList.add('terminal-wrapper');
    this.term.open(this.container);
    this.fitAddon.fit();

    window.addEventListener('resize', () => {
      clearTimeout(this.resizeTimer);
      this.resizeTimer = setTimeout(() => this.fitAddon.fit(), 150);
    });
  }

  #initializeTerminal() {
    this.term.write(`${COLORS.accent}ShadowHound Mission Terminal${COLORS.reset}\r\n`);
    this.term.write('Type `help` to list available commands.\r\n');
    this.#showPrompt(false);

    this.term.onData((data) => this.#handleInput(data));

    this.term.attachCustomKeyEventHandler((event) => {
      if (event.ctrlKey && (event.key === 'L' || event.key === 'l')) {
        this.clear();
        return false;
      }

      return true;
    });
  }

  #handleInput(data) {
    if (data === '\u0003') { // Ctrl+C
      this.term.write('^C');
      this.currentInput = '';
      this.#showPrompt();
      return;
    }

    if (data === '\u000c') { // Ctrl+L
      this.clear();
      return;
    }

    if (data === '\u007f') { // Backspace
      if (this.currentInput.length > 0) {
        this.currentInput = this.currentInput.slice(0, -1);
        this.term.write('\b \b');
      }
      return;
    }

    if (data === '\t') {
      this.#handleAutocomplete();
      return;
    }

    if (data === '\u001b[A') { // Arrow Up
      this.#navigateHistory(-1);
      return;
    }

    if (data === '\u001b[B') { // Arrow Down
      this.#navigateHistory(1);
      return;
    }

    if (data === '\r' || data === '\n') {
      this.term.write('\r\n');
      const command = this.currentInput;
      this.currentInput = '';
      this.executeCommand(command);
      return;
    }

    // Ignore other ANSI sequences (e.g., left/right arrows)
    if (data.startsWith('\u001b')) {
      return;
    }

    this.currentInput += data;
    this.term.write(data);
  }

  #handleAutocomplete() {
    const prefix = this.currentInput.trim();
    if (!prefix) {
      this.showToast(`Commands: ${this.autocompleteCommands.join(', ')}`);
      return;
    }

    const matches = this.autocompleteCommands.filter((cmd) => cmd.startsWith(prefix));
    if (matches.length === 1) {
      const completion = matches[0];
      const suffix = completion.slice(prefix.length);
      this.currentInput += suffix;
      this.term.write(suffix);
      this.showToast(`Auto-completed: ${completion}`);
    } else if (matches.length > 1) {
      this.term.write(`\r\n${COLORS.accent}${matches.join('    ')}${COLORS.reset}\r\n`);
      this.term.write(`${COLORS.prompt}${this.prompt}${COLORS.reset}${this.currentInput}`);
    }
  }

  #navigateHistory(direction) {
    if (!this.history.length) {
      return;
    }

    this.historyIndex = Math.min(
      Math.max(this.historyIndex + direction, 0),
      this.history.length
    );

    const historyEntry = this.history[this.historyIndex] ?? '';
    this.#replaceCurrentLine(historyEntry);
  }

  #replaceCurrentLine(value) {
    const sanitized = value ?? '';
    const erase = `\r\u001b[2K`;
    this.term.write(`${erase}${COLORS.prompt}${this.prompt}${COLORS.reset}${sanitized}`);
    this.currentInput = sanitized;
  }

  #handleSocketMessage(raw) {
    if (!raw) {
      return;
    }

    let payload = raw;
    if (typeof raw === 'string') {
      try {
        payload = JSON.parse(raw);
      } catch (error) {
        payload = { type: 'info', message: raw };
      }
    }

    if (payload && typeof payload === 'object' && 'message' in payload) {
      this.#renderMessage(payload);
    } else {
      this.#renderMessage({ type: 'info', message: String(raw) });
    }

    this.awaitingResponse = false;
  }

  #renderMessage({ type = 'info', message = '' }) {
    const color = COLORS[type] ?? COLORS.info;
    const icon = ICONS[type] ?? ICONS.info;
    const normalized = Array.isArray(message) ? message.join('\n') : message;

    const output = `${color}${icon}${normalized}${COLORS.reset}`;
    this.term.write(`\r\n${output}\r\n`);
    this.term.write(`${COLORS.prompt}${this.prompt}${COLORS.reset}${this.currentInput}`);
  }

  #showPrompt(leadingNewline = true) {
    const prefix = leadingNewline ? '\r\n' : '';
    this.term.write(`${prefix}${COLORS.prompt}${this.prompt}${COLORS.reset}`);
    this.currentInput = '';
    this.historyIndex = this.history.length;
  }

  #sendToSocket(command) {
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
      this.printSystem('Command queued locally. Terminal is offline.', 'warn');
      this.#showPrompt();
      return;
    }

    try {
      this.socket.send(JSON.stringify({ command }));
    } catch (error) {
      console.error('[TerminalManager] Failed to send command:', error);
      this.printSystem(`Failed to send command: ${error.message}`, 'error');
    }
  }

  #scheduleReconnect() {
    if (this.reconnectTimer) {
      return;
    }

    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, this.reconnectDelay);
  }

  #inferWebsocketUrl() {
    if (typeof window === 'undefined' || !window.location) {
      return null;
    }

    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    return `${protocol}//${window.location.host}/ws/terminal`;
  }

  #setStatus(label, type = 'info') {
    if (!this.statusIndicator) {
      return;
    }

    this.statusIndicator.textContent = label;
    this.statusIndicator.classList.remove('is-disconnected', 'is-warning');

    if (type === 'warn') {
      this.statusIndicator.classList.add('is-warning');
    } else if (type === 'error') {
      this.statusIndicator.classList.add('is-disconnected');
    } else if (type === 'info') {
      this.statusIndicator.classList.remove('is-disconnected');
    }
  }

  showToast(message) {
    if (!this.toastElement) {
      this.toastElement = document.createElement('div');
      this.toastElement.className = 'terminal-toast';
      this.container.appendChild(this.toastElement);
    }

    this.toastElement.textContent = message;
    this.toastElement.classList.add('is-visible');

    clearTimeout(this.toastTimeout);
    this.toastTimeout = setTimeout(() => {
      this.toastElement.classList.remove('is-visible');
    }, 1500);
  }
}

export default TerminalManager;

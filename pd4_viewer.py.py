#!/usr/bin/env python3
"""
DVL PD4 Viewer – Teledyne/Nortek (TCP or Serial input; TCP/UDP output)

New in this version
-------------------
• **Water‑track (reference layer) velocities** parsed and used when bottom‑track is invalid (0x8000). GUI shows which source is active: BT or WT.
• **Mini table of last ensembles**: A live table logs the last 100 decoded ensembles (time, source, X/Y/Z, altitude, coord). Buttons to Clear.
• **Separate Output Log** pane next to Raw Data.
• **Output protocol selector**: forward parsed JSON over TCP or UDP.
• **Serial input**: choose Serial instead of TCP and set COM port, baud, parity, etc. (Requires `pyserial`).
• **Selectable output fields**: choose exactly which keys to send (e.g., only `xspeed` and `altitude`).
• **2D & 3D speeds** added, plus **meters/minute** versions (m/min) for both 2D and 3D.
• More tolerant checksum (accepts exact-sum or two's-complement).
• 0x8000 velocities shown as “—” (invalid) and not forwarded; if BT invalid and WT valid, WT is used.

Build the .exe
--------------
1) Install deps:  `pip install pyinstaller pyserial`
2) Build:         `pyinstaller --onefile --noconsole --name DVL_PD4_Viewer pd4_viewer.py`

"""
from __future__ import annotations
import socket
import threading
import struct
import json
from datetime import datetime, timezone
import math
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None  # we'll check at runtime when user selects Serial

DVl_ID = 0x7D

COORD_BITS = {0b00: "BEAM", 0b01: "INSTRUMENT", 0b10: "SHIP", 0b11: "EARTH"}

# ------------------------------
# Helpers
# ------------------------------

def checksum_ok(frame: bytes, payload_len: int) -> bool:
    if len(frame) < 6:
        return False
    data = frame[:-2]
    got = struct.unpack_from('<H', frame, len(frame)-2)[0]
    s = sum(data) & 0xFFFF
    return got == s or ((s + got) & 0xFFFF) == 0


def i16_le(b: bytes, off: int) -> int:
    return struct.unpack_from('<h', b, off)[0]


def u16_le(b: bytes, off: int) -> int:
    return struct.unpack_from('<H', b, off)[0]


def parse_pd4_frame(frame: bytes, layout: str = 'nortek') -> dict:
    """Parse PD4-like frame. Two layouts:
    • 'teledyne': payload starts immediately and sys_cfg is 1 byte at payload[0]
    • 'nortek'  : payload starts with a 2-byte word; shift subsequent fields by +2
    Returns dict with xspeed/yspeed/zspeed, altitude, coord, vel_src ('BT' or 'WT').
    """
    if len(frame) < 6 or frame[0] != DVl_ID:
        raise ValueError('Not a PD4/5 frame')

    struct_id = frame[1]
    L = u16_le(frame, 2)
    if len(frame) < 4 + L + 2:
        raise ValueError('Truncated frame')

    payload = frame[4:4+L]
    csum_ok = checksum_ok(frame, L)

    offset = 0 if layout == 'teledyne' else 2

    # Coordinate frame bits from config
    if offset == 0:
        coord_bits = (payload[0] >> 6) & 0b11
    else:
        sys_cfg_word = u16_le(payload, 0)
        coord_bits = (sys_cfg_word >> 14) & 0b11

    coord = COORD_BITS.get(coord_bits, 'UNKNOWN')

    # Bottom‑track velocities (mm/s)
    xv_bt = i16_le(payload, 1 + offset)
    yv_bt = i16_le(payload, 3 + offset)
    zv_bt = i16_le(payload, 5 + offset)

    # Beam ranges (cm)
    b1 = u16_le(payload, 9 + offset)
    b2 = u16_le(payload, 11 + offset)
    b3 = u16_le(payload, 13 + offset)
    b4 = u16_le(payload, 15 + offset)

    # Reference layer (water‑track) velocities (mm/s)
    try:
        xv_wt = i16_le(payload, 18 + offset)
        yv_wt = i16_le(payload, 20 + offset)
        zv_wt = i16_le(payload, 22 + offset)
    except struct.error:
        xv_wt = yv_wt = zv_wt = -32768

    def mmps_to_ms(val: int):
        return None if val == -32768 else val / 1000.0

    xs_bt = mmps_to_ms(xv_bt)
    ys_bt = mmps_to_ms(yv_bt)
    zs_bt = mmps_to_ms(zv_bt)
    xs_wt = mmps_to_ms(xv_wt)
    ys_wt = mmps_to_ms(yv_wt)
    zs_wt = mmps_to_ms(zv_wt)

    # Choose source: prefer BT when valid; else WT
    def choose(a, b):
        return (a, 'BT') if a is not None else ((b, 'WT') if b is not None else (None, None))

    xs, xs_src = choose(xs_bt, xs_wt)
    ys, ys_src = choose(ys_bt, ys_wt)
    zs, zs_src = choose(zs_bt, zs_wt)
    vel_src = 'WT' if 'WT' in {xs_src, ys_src, zs_src} else ('BT' if None not in {xs_src, ys_src, zs_src} else None)

    # Altitude from beams (cm→m), ignore 0 or 0xFFFF
    beams_cm = [v for v in (b1, b2, b3, b4) if v not in (0, 0xFFFF)]
    alt = (sum(beams_cm)/len(beams_cm)/100.0) if beams_cm else None

    return {
        'type': 'PD4' if struct_id == 0 else f'PD{struct_id+4}',
        'checksum_ok': csum_ok,
        'coord': coord,
        'xspeed': xs,
        'yspeed': ys,
        'zspeed': zs,
        'altitude': alt,
        'vel_src': vel_src,
        'raw_len': len(frame),
    }

# ------------------------------
# Framing
# ------------------------------

def find_frames(buf: bytearray) -> list[bytes]:
    frames = []
    i = 0
    while True:
        try:
            start = buf.index(DVl_ID, i)
        except ValueError:
            if i:
                del buf[:i]
            break
        if len(buf) - start < 6:
            if start:
                del buf[:start]
            break
        L = u16_le(buf, start+2)
        need = 4 + L + 2
        if len(buf) - start < need:
            if start:
                del buf[:start]
            break
        frames.append(bytes(buf[start:start+need]))
        i = start + need
    if i:
        del buf[:i]
    return frames

# ------------------------------
# IO threads (TCP/Serial input, TCP/UDP output)
# ------------------------------
class IOThreads:
    def __init__(self, on_raw, on_out, on_parsed, get_input_mode, get_input_params, get_output_params, get_layout, get_output_fields):
        self.on_raw = on_raw
        self.on_out = on_out
        self.on_parsed = on_parsed
        self.get_input_mode = get_input_mode
        self.get_input_params = get_input_params
        self.get_output_params = get_output_params
        self.get_layout = get_layout
        self.get_output_fields = get_output_fields
        self.stop_evt = threading.Event()
        self.rx_thread = None
        self.tcp_tx_sock = None
        self.buf = bytearray()
        self.ser = None

    # ---- RX control
    def start_rx(self):
        self.stop_rx()
        self.stop_evt.clear()
        mode = self.get_input_mode()
        if mode == 'TCP':
            host, port = self.get_input_params()
            self.rx_thread = threading.Thread(target=self._rx_tcp_loop, args=(host, port), daemon=True)
        else:
            if serial is None:
                self.on_raw('[Serial not available: install pyserial]\n')
                return
            port, baud, bytesize, parity, stopbits = self.get_input_params()
            self.rx_thread = threading.Thread(target=self._rx_serial_loop, args=(port, baud, bytesize, parity, stopbits), daemon=True)
        self.rx_thread.start()

    def stop_rx(self):
        self.stop_evt.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
        self.rx_thread = None
        if self.tcp_tx_sock:
            try:
                self.tcp_tx_sock.close()
            except Exception:
                pass
            self.tcp_tx_sock = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    # ---- TCP RX
    def _rx_tcp_loop(self, host, port):
        try:
            with socket.create_connection((host, port), timeout=5) as s:
                s.settimeout(1.0)
                self.on_raw(f"Connected TCP {host}:{port}\n")
                while not self.stop_evt.is_set():
                    try:
                        chunk = s.recv(4096)
                        if not chunk:
                            self.on_raw('[Disconnected]')
                            break
                        self._handle_chunk(chunk)
                    except socket.timeout:
                        continue
        except Exception as e:
            self.on_raw(f"[RX error] {e}\n")

    # ---- Serial RX
    def _rx_serial_loop(self, port, baud, bytesize, parity, stopbits):
        try:
            self.ser = serial.Serial(port=port, baudrate=baud, bytesize=bytesize, parity=parity, stopbits=stopbits, timeout=0.5)
            self.on_raw(f"Connected Serial {port} @ {baud}\n")
            while not self.stop_evt.is_set():
                chunk = self.ser.read(4096)
                if chunk:
                    self._handle_chunk(chunk)
        except Exception as e:
            self.on_raw(f"[Serial RX error] {e}\n")

    # ---- Common chunk handling
    def _handle_chunk(self, chunk: bytes):
        # hex dump with 16 bytes/row
        self.on_raw(chunk.hex(' ', 1) + "\n")
        self.buf.extend(chunk)
        for frame in find_frames(self.buf):
            try:
                parsed = parse_pd4_frame(frame, layout=self.get_layout())
            except Exception as e:
                self.on_raw(f"[Parse error] {e}\n")
                continue
            # Attach timestamp here so GUI & TX share the same value
            parsed['ts'] = datetime.now(timezone.utc).isoformat()
            self.on_parsed(parsed)
            self.forward(parsed)

    # ---- TX (TCP/UDP)
    def ensure_tcp_tx(self, host: str, port: int):
        if self.tcp_tx_sock:
            try:
                self.tcp_tx_sock.send(b'')
                return True
            except Exception:
                try:
                    self.tcp_tx_sock.close()
                except Exception:
                    pass
                self.tcp_tx_sock = None
        try:
            self.tcp_tx_sock = socket.create_connection((host, port), timeout=3)
            return True
        except Exception as e:
            self.on_out(f"[TX connect error] {e}\n")
            self.tcp_tx_sock = None
            return False

    def forward(self, parsed: dict):
        proto, host, port = self.get_output_params()
        if not host or not port or not proto:
            return
        selected = self.get_output_fields()  # set of keys
        # compute derived speeds (m/s and m/min)
        xs = parsed.get('xspeed'); ys = parsed.get('yspeed'); zs = parsed.get('zspeed')
        def hypot2(a, b):
            if a is None or b is None:
                return None
            return math.hypot(a, b)
        def hypot3(a, b, c):
            if a is None or b is None or c is None:
                return None
            return math.sqrt(a*a + b*b + c*c)
        speed2d = hypot2(xs, ys)
        speed3d = hypot3(xs, ys, zs)
        speed2d_mpm = speed2d * 60.0 if speed2d is not None else None
        speed3d_mpm = speed3d * 60.0 if speed3d is not None else None

        msg_full = {
            'ts': parsed.get('ts'),
            'type': parsed.get('type'),
            'coord': parsed.get('coord'),
            'vel_src': parsed.get('vel_src'),
            'xspeed': xs,
            'yspeed': ys,
            'zspeed': zs,
            'altitude': parsed.get('altitude'),
            'speed2d': speed2d,
            'speed3d': speed3d,
            'speed2d_mpm': speed2d_mpm,
            'speed3d_mpm': speed3d_mpm,
        }
        ui_order = ['ts','type','coord','vel_src','xspeed','yspeed','zspeed','altitude','speed2d','speed3d','speed2d_mpm','speed3d_mpm']
        out_list = [msg_full[k] for k in ui_order if k in selected]
        line = (json.dumps(out_list) + "\n").encode('utf-8')
        try:
            if proto == 'TCP':
                if not self.ensure_tcp_tx(host, port):
                    return
                self.tcp_tx_sock.sendall(line)
                self.on_out(line.decode('utf-8'))
            else:  # UDP
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as us:
                    us.sendto(line, (host, port))
                self.on_out(line.decode('utf-8'))
        except Exception as e:
            self.on_out(f"[TX error] {e}\n")

# ------------------------------
# GUI
# ------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("DVL PD4 Viewer")
        self.geometry("1280x860")

        self.layout_var = tk.StringVar(value='nortek')
        self.input_mode = tk.StringVar(value='TCP')   # TCP or Serial
        self.output_proto = tk.StringVar(value='TCP') # TCP or UDP

        # Output field selections
        self.out_fields_vars = {k: tk.BooleanVar(value=True) for k in ['ts','type','coord','vel_src','xspeed','yspeed','zspeed','altitude','speed2d','speed3d','speed2d_mpm','speed3d_mpm']}

        # Mini table store
        self.table_limit = 100

        self.io = IOThreads(
            self.append_raw,
            self.append_out,
            self.update_parsed,
            self.get_input_mode,
            self.get_input_params,
            self.get_output_params,
            self.get_layout,
            self.get_output_fields,
        )
        self._build_widgets()

    # ---- UI
    def _build_widgets(self):
        pad = {'padx': 6, 'pady': 6}

        # Input toolbar
        frm_in = ttk.LabelFrame(self, text='Input')
        frm_in.pack(fill='x', **pad)
        ttk.Label(frm_in, text='Mode').grid(row=0, column=0, sticky='w')
        self.cmb_mode = ttk.Combobox(frm_in, textvariable=self.input_mode, values=['TCP','Serial'], width=8, state='readonly')
        self.cmb_mode.grid(row=0, column=1, sticky='w')
        self.cmb_mode.bind('<<ComboboxSelected>>', lambda e: self._toggle_input_fields())

        # TCP fields
        ttk.Label(frm_in, text='IP').grid(row=0, column=2, sticky='w')
        self.in_ip = ttk.Entry(frm_in, width=18)
        self.in_ip.insert(0, '127.0.0.1')
        self.in_ip.grid(row=0, column=3)
        ttk.Label(frm_in, text='Port').grid(row=0, column=4, sticky='w')
        self.in_port = ttk.Entry(frm_in, width=8)
        self.in_port.insert(0, '2102')
        self.in_port.grid(row=0, column=5)

        # Serial fields
        ttk.Label(frm_in, text='COM').grid(row=0, column=6, sticky='e')
        self.ser_port = ttk.Combobox(frm_in, width=8, values=self._serial_ports())
        self.ser_port.grid(row=0, column=7)
        ttk.Label(frm_in, text='Baud').grid(row=0, column=8, sticky='e')
        self.ser_baud = ttk.Combobox(frm_in, width=8, values=['9600','19200','38400','57600','115200','230400','460800','921600'])
        self.ser_baud.set('115200')
        self.ser_baud.grid(row=0, column=9)
        ttk.Label(frm_in, text='Parity').grid(row=0, column=10, sticky='e')
        self.ser_parity = ttk.Combobox(frm_in, width=4, values=['N','E','O'])
        self.ser_parity.set('N')
        self.ser_parity.grid(row=0, column=11)
        ttk.Label(frm_in, text='Data').grid(row=0, column=12, sticky='e')
        self.ser_bytes = ttk.Combobox(frm_in, width=4, values=['7','8'])
        self.ser_bytes.set('8')
        self.ser_bytes.grid(row=0, column=13)
        ttk.Label(frm_in, text='Stop').grid(row=0, column=14, sticky='e')
        self.ser_stop = ttk.Combobox(frm_in, width=4, values=['1','2'])
        self.ser_stop.set('1')
        self.ser_stop.grid(row=0, column=15)

        # Layout + buttons
        ttk.Label(frm_in, text='Layout').grid(row=0, column=16, sticky='e', padx=(10,4))
        self.layout_combo = ttk.Combobox(frm_in, textvariable=self.layout_var, width=10, values=['teledyne','nortek'], state='readonly')
        self.layout_combo.grid(row=0, column=17)
        ttk.Button(frm_in, text='Connect', command=self.on_connect).grid(row=0, column=18, padx=8)
        ttk.Button(frm_in, text='Disconnect', command=self.on_disconnect).grid(row=0, column=19)

        # Output toolbar
        frm_out = ttk.LabelFrame(self, text='Output (forward parsed JSON)')
        frm_out.pack(fill='x', **pad)
        ttk.Label(frm_out, text='Protocol').grid(row=0, column=0, sticky='w')
        self.cmb_out_proto = ttk.Combobox(frm_out, textvariable=self.output_proto, values=['TCP','UDP'], width=7, state='readonly')
        self.cmb_out_proto.grid(row=0, column=1)
        ttk.Label(frm_out, text='IP').grid(row=0, column=2, sticky='w')
        self.out_ip = ttk.Entry(frm_out, width=18)
        self.out_ip.grid(row=0, column=3)
        ttk.Label(frm_out, text='Port').grid(row=0, column=4, sticky='w')
        self.out_port = ttk.Entry(frm_out, width=8)
        self.out_port.grid(row=0, column=5)

        # Output field checkboxes
        row = 1
        ttk.Label(frm_out, text='Fields to send:').grid(row=row, column=0, sticky='w', pady=(4,0))
        col = 1
        for key in ['ts','type','coord','vel_src','xspeed','yspeed','zspeed','altitude','speed2d','speed3d','speed2d_mpm','speed3d_mpm']:
            cb = ttk.Checkbutton(frm_out, text=key, variable=self.out_fields_vars[key])
            cb.grid(row=row, column=col, sticky='w')
            col += 1
            if col > 8:
                row += 1
                col = 1
        ttk.Button(frm_out, text='All', command=lambda: self._set_all_fields(True)).grid(row=row+1, column=1, sticky='w', pady=(4,0))
        ttk.Button(frm_out, text='None', command=lambda: self._set_all_fields(False)).grid(row=row+1, column=2, sticky='w', pady=(4,0))

        # Interpreted view + Table
        frm_data = ttk.LabelFrame(self, text='View data (interpreted)')
        frm_data.pack(fill='x', **pad)
        labels = [('Coord frame:', 'coord'),
                  ('Velocity source:', 'vel_src'),
                  ('X speed (m/s):', 'xspeed'),
                  ('Y speed (m/s):', 'yspeed'),
                  ('Z speed (m/s):', 'zspeed'),
                  ('2D speed (m/s):', 'speed2d'),
                  ('3D speed (m/s):', 'speed3d'),
                  ('2D speed (m/min):', 'speed2d_mpm'),
                  ('3D speed (m/min):', 'speed3d_mpm'),
                  ('Altitude (m):', 'altitude'),
                  ('Checksum OK:', 'checksum')]
        self.vals = {}
        for r,(txt,key) in enumerate(labels):
            ttk.Label(frm_data, text=txt).grid(row=r, column=0, sticky='w')
            var = tk.StringVar(value='—')
            ttk.Label(frm_data, textvariable=var, font=('Segoe UI', 11, 'bold')).grid(row=r, column=1, sticky='w')
            self.vals[key] = var

        # Mini table
        frm_table = ttk.LabelFrame(self, text='Last ensembles')
        frm_table.pack(fill='both', expand=False, **pad)
        cols = ('ts','src','x','y','z','alt','coord')
        self.table = ttk.Treeview(frm_table, columns=cols, show='headings', height=8)
        for c, w in [('ts',180),('src',60),('x',90),('y',90),('z',90),('alt',90),('coord',100)]:
            self.table.heading(c, text=c.upper())
            self.table.column(c, width=w, anchor='center')
        self.table.pack(fill='x', expand=False)
        btns = ttk.Frame(frm_table)
        btns.pack(fill='x')
        ttk.Button(btns, text='Clear table', command=self._clear_table).pack(side='left', padx=4, pady=4)

        # Logs: Raw and Output side-by-side
        frm_logs = ttk.LabelFrame(self, text='Logs')
        frm_logs.pack(fill='both', expand=True, **pad)
        pan = ttk.Panedwindow(frm_logs, orient=tk.HORIZONTAL)
        pan.pack(fill='both', expand=True)
        frm_raw = ttk.Frame(pan)
        frm_outlog = ttk.Frame(pan)
        pan.add(frm_raw, weight=3)
        pan.add(frm_outlog, weight=2)

        ttk.Label(frm_raw, text='Raw data (hex)').pack(anchor='w')
        self.raw = tk.Text(frm_raw, height=12)
        self.raw.pack(fill='both', expand=True, padx=6, pady=6)

        ttk.Label(frm_outlog, text='Output log').pack(anchor='w')
        self.outlog = tk.Text(frm_outlog, height=12)
        self.outlog.pack(fill='both', expand=True, padx=6, pady=6)

        self._toggle_input_fields()

    # ---- UI helpers
    def _serial_ports(self):
        if serial is None:
            return []
        return [p.device for p in serial.tools.list_ports.comports()]

    def _toggle_input_fields(self):
        mode = self.input_mode.get()
        # TCP widgets
        for w in [self.in_ip, self.in_port]:
            w.configure(state=('normal' if mode=='TCP' else 'disabled'))
        # Serial widgets
        for w in [self.ser_port, self.ser_baud, self.ser_parity, self.ser_bytes, self.ser_stop]:
            w.configure(state=('normal' if mode=='Serial' else 'disabled'))

    def _set_all_fields(self, value: bool):
        for v in self.out_fields_vars.values():
            v.set(value)

    def _clear_table(self):
        for iid in self.table.get_children():
            self.table.delete(iid)

    # ---- IO integration
    def on_connect(self):
        self.append_raw(f"[Connecting via {self.input_mode.get()}]\n")
        self.io.start_rx()

    def on_disconnect(self):
        self.io.stop_rx()
        self.append_raw('[RX stopped]\n')

    def append_raw(self, text: str):
        self.raw.insert('end', text)
        self.raw.see('end')

    def append_out(self, text: str):
        self.outlog.insert('end', text)
        self.outlog.see('end')

    def get_input_mode(self):
        return self.input_mode.get()

    def get_input_params(self):
        if self.input_mode.get() == 'TCP':
            host = self.in_ip.get().strip()
            try:
                port = int(self.in_port.get().strip())
            except ValueError:
                messagebox.showerror('Invalid port', 'Input port must be an integer')
                raise
            return host, port
        # Serial params
        port = self.ser_port.get().strip()
        if not port:
            messagebox.showerror('Serial', 'Select a COM port')
            raise RuntimeError('No COM port selected')
        try:
            baud = int(self.ser_baud.get())
        except ValueError:
            baud = 115200
        bytesize = 7 if self.ser_bytes.get() == '7' else 8
        parity = {'N': 'N', 'E': 'E', 'O': 'O'}[self.ser_parity.get() or 'N']
        stopbits = 2 if self.ser_stop.get() == '2' else 1
        return port, baud, bytesize, parity, stopbits

    def get_output_params(self):
        proto = self.output_proto.get()
        host = self.out_ip.get().strip()
        port_str = self.out_port.get().strip()
        if not host or not port_str:
            return None, None, None
        try:
            port = int(port_str)
        except ValueError:
            return None, None, None
        return proto, host, port

    def get_output_fields(self) -> set[str]:
        return {k for k,v in self.out_fields_vars.items() if v.get()}

    def get_layout(self) -> str:
        return self.layout_var.get()

    def update_parsed(self, d: dict):
        # Update summary labels
        self.vals['coord'].set(d.get('coord','?'))
        self.vals['vel_src'].set(d.get('vel_src') or '—')
        xs = d.get('xspeed'); ys = d.get('yspeed'); zs = d.get('zspeed')
        def fmt(v, nd=3):
            return '—' if v is None else f"{v:.{nd}f}"
        self.vals['xspeed'].set(fmt(xs))
        self.vals['yspeed'].set(fmt(ys))
        self.vals['zspeed'].set(fmt(zs))
        # derived speeds
        speed2d = math.hypot(xs, ys) if (xs is not None and ys is not None) else None
        speed3d = (math.sqrt(xs*xs + ys*ys + zs*zs) if (xs is not None and ys is not None and zs is not None) else None)
        self.vals['speed2d'].set(fmt(speed2d))
        self.vals['speed3d'].set(fmt(speed3d))
        self.vals['speed2d_mpm'].set(fmt(speed2d*60.0 if speed2d is not None else None, nd=1))
        self.vals['speed3d_mpm'].set(fmt(speed3d*60.0 if speed3d is not None else None, nd=1))
        alt = d.get('altitude')
        self.vals['altitude'].set('—' if alt is None else f"{alt:.2f}")
        self.vals['checksum'].set('OK' if d.get('checksum_ok') else 'BAD')

        # Insert into mini table
        ts = d.get('ts') or datetime.now(timezone.utc).isoformat()
        row = (ts,
               d.get('vel_src') or '',
               '—' if xs is None else f"{xs:.3f}",
               '—' if ys is None else f"{ys:.3f}",
               '—' if zs is None else f"{zs:.3f}",
               '—' if alt is None else f"{alt:.2f}",
               d.get('coord') or '')
        self.table.insert('', 'end', values=row)
        kids = self.table.get_children()
        if len(kids) > self.table_limit:
            self.table.delete(kids[0])


if __name__ == '__main__':
    app = App()
    app.mainloop()

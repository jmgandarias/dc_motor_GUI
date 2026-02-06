# Small GUI tool to trigger an ESP32 experiment over a serial port and save the
# resulting stream of measurements to a CSV file.
#
# High-level behavior:
# - Opens a serial port to an ESP32 and optionally pulses DTR to reset the board.
# - Starts a background reader thread that reads raw bytes, splits into text lines
#   and hands complete lines to a handler.
# - The handler posts log lines to a queue for the main (GUI) thread, tracks a
#   READY handshake, parses semicolon-separated data rows while an experiment is
#   "collecting", and reacts to an "END" marker by saving the collected data.
# - The main thread periodically drains the queue and appends lines to the log
#   widget; saving to CSV is performed on the main thread (GUI context).
#
# Threading / synchronization notes:
# - The reader thread is a daemon that performs non-blocking reads and sleeps
#   briefly when no data is available to avoid busy looping.
# - A queue.Queue is used to transfer text/log messages from the reader thread
#   to the GUI thread (Tkinter must only be touched on the main thread).
# - A threading.Lock protects the in-memory list of parsed data rows so both
#   threads may safely access it (reader appends, main thread reads/writes when
#   saving or starting a new experiment).
#
# Serial robustness:
# - Reads are performed into a local byte buffer; lines are extracted at LF
#   boundaries to handle partial reads and different newline conventions.
# - Decoding uses errors='ignore' to be tolerant of transient garbage.
# - Opening the serial port is guarded with exceptions and reports errors via
#   message boxes so the user can correct the configuration.
#
# CSV saving:
# - The CSV writer writes a header then all collected rows; the out directory is
#   created if necessary. Errors are reported to the user.
#
# This file is intended to be run as a standalone GUI application.
#
# Author: Juan M. Gandarias
# web: www.jmgandarias.com
# email: jmgandarias@uma.es

import sys
import time
import threading
import queue
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import csv
from datetime import datetime
import os
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Default serial settings and reader sleep interval
DEFAULT_BAUD = 500000  # Must match Serial.begin() on ESP32
READ_THREAD_SLEEP_S = 0.01  # Sleep between polls in the reader thread
PLOT_UPDATE_MS = 100  # Plot refresh interval
MAX_PLOT_POINTS = 5000  # Limit points for performance


class ESP32StepGUI(tk.Tk):
    """Main GUI application for collecting experiment data from an ESP32.

    Responsibilities:
    - Present a simple UI to select a serial port, set an output CSV file,
      and start/stop experiments.
    - Manage the serial connection lifecycle.
    - Run a background reader thread that turns raw serial bytes into text
      lines and forwards them to the main thread for logging and parsing.
    - Collect parsed measurement rows in-memory (thread-safe) and save them to CSV.
    """

    def __init__(self):
        super().__init__()
        self.title("Serial Port Data Collection (CSV)")
        self.geometry("720x576")

        # Serial and thread state
        # self.ser: an instance of serial.Serial when the port is open, else None
        self.ser = None
        # Background thread that continuously reads from serial
        self.read_thread = None
        # Event to request the reader thread stop; set from the main thread
        self.stop_event = threading.Event()
        # True while we are actively collecting experiment data (between Start and END)
        self.collecting = False
        # Queue used to deliver log/text messages from the reader thread to the UI thread
        self.lines_queue = queue.Queue()

        # In-memory buffer of parsed data rows. Each row is [PWM(int), pos(float), vel(float), time_ms(float)].
        # Access to this list must be guarded by data_lock.
        self.data_rows = []
        self.data_lock = threading.Lock()

        # Build UI and start periodic queue processing on the main loop
        self._build_ui()
        self.after(100, self._process_lines_queue)

        # Tracks whether the device has announced "READY" (used to avoid sending a start
        # byte too early after connecting / resetting the ESP32).
        self.ready_seen = False

    # -------------------- UI --------------------
    def _build_ui(self):
        """Create and lay out all UI widgets."""
        pad = {"padx": 8, "pady": 6}

        # Connection frame
        conn = ttk.LabelFrame(self, text="Connection")
        conn.pack(fill="x", **pad)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, sticky="w", **pad)
        self.port_cmb = ttk.Combobox(
            conn, width=36, state="readonly", values=self._list_ports()
        )
        self.port_cmb.grid(row=0, column=1, sticky="w", **pad)

        self.refresh_btn = ttk.Button(conn, text="Refresh", command=self._refresh_ports)
        self.refresh_btn.grid(row=0, column=2, sticky="w", **pad)

        ttk.Label(conn, text="Baud:").grid(row=1, column=0, sticky="w", **pad)
        self.baud_cmb = ttk.Combobox(
            conn,
            width=12,
            state="readonly",
            values=[
                9600,
                19200,
                38400,
                57600,
                115200,
                230400,
                250000,
                460800,
                500000,
                921600,
                1000000,
                2000000,
            ],
        )
        # If you run your ESP32 at 2,000,000 baud by default, do:
        # self.baud_cmb.set("2000000")
        self.baud_cmb.set(str(DEFAULT_BAUD))
        self.baud_cmb.grid(row=1, column=1, sticky="w", **pad)

        # New: Connect / Disconnect buttons
        self.connect_btn = ttk.Button(conn, text="Connect", command=self._on_connect)
        self.connect_btn.grid(row=0, column=3, sticky="w", **pad)
        self.disconnect_btn = ttk.Button(
            conn, text="Disconnect", command=self._on_disconnect, state="disabled"
        )
        self.disconnect_btn.grid(row=0, column=4, sticky="w", **pad)

        # --- Output frame (add this) ---
        out = ttk.LabelFrame(self, text="Output")
        out.pack(fill="x", **pad)

        ttk.Label(out, text="CSV file:").grid(row=0, column=0, sticky="w", **pad)

        # The actual entry the rest of the code expects:
        self.file_entry = ttk.Entry(out, width=50)
        self.file_entry.grid(row=0, column=1, sticky="we", **pad)
        out.columnconfigure(1, weight=1)

        # Sensible default path: ~/Documents if it exists, else CWD, with timestamped name.
        default_name = datetime.now().strftime("esp32_step_%Y%m%d_%H%M%S.csv")
        docs = os.path.expanduser("~/Documents")
        default_dir = docs if os.path.isdir(docs) else os.getcwd()
        self.file_entry.insert(0, os.path.join(default_dir, default_name))

        # Browse button already wired to _browse_outfile()
        browse_btn = ttk.Button(out, text="Browse…", command=self._browse_outfile)
        browse_btn.grid(row=0, column=2, **pad)

        # Controls frame
        ctrl = ttk.LabelFrame(self, text="Controls")
        ctrl.pack(fill="x", **pad)

        # Start is disabled until connected
        self.start_btn = ttk.Button(
            ctrl, text="Start Experiment", command=self._on_start, state="disabled"
        )
        self.start_btn.grid(row=0, column=0, **pad)

        self.stop_btn = ttk.Button(
            ctrl, text="Stop & Save", command=self._on_stop_and_save, state="disabled"
        )
        self.stop_btn.grid(row=0, column=1, **pad)

        self.exit_btn = ttk.Button(ctrl, text="Exit", command=self._on_exit)
        self.exit_btn.grid(row=0, column=2, **pad)

        # Status text
        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(self, textvariable=self.status_var).pack(anchor="w", **pad)

        # Notebook with Plot and Log tabs
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, **pad)

        # Plot tab
        self.plot_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.plot_tab, text="Live Plot")

        # Log tab
        self.log_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.log_tab, text="Log")

        # Read-only log widget updated from the main thread by draining lines_queue
        self.log = tk.Text(self.log_tab, height=9, wrap="none", state="disabled")
        self.log.pack(fill="both", expand=True)

        # Initialize matplotlib figure in plot tab and schedule periodic updates
        self._init_plot()
        self.after(PLOT_UPDATE_MS, self._update_plot)

    # -------------------- Serial helpers --------------------
    def _list_ports(self):
        """Return a list of human-readable port strings for the combobox."""
        ports = serial.tools.list_ports.comports()
        return [f"{p.device} - {p.description}" for p in ports]

    def _refresh_ports(self):
        """Re-query available serial ports and update the combobox."""
        ports = self._list_ports()
        self.port_cmb["values"] = ports
        if len(ports) == 1:
            self.port_cmb.set(ports[0])
        self._log("Ports refreshed.")

    def _open_serial(self):
        """Open the selected serial port with the chosen baud rate.

        Returns True on success, False otherwise. Shows message boxes for user
        recoverable errors so the user may reconfigure.
        """
        if self.ser and self.ser.is_open:
            return True
        sel = self.port_cmb.get()
        if not sel:
            messagebox.showwarning("Select port", "Please select a serial port first.")
            return False
        port = sel.split(" - ")[0]
        try:
            self.ser = serial.Serial(
                port=port, baudrate=int(self.baud_cmb.get()), timeout=0.05
            )

            # Optional: pulse DTR to reset many ESP32 boards so they start in a known state.
            # This is best-effort; failure to toggle DTR is non-fatal.
            try:
                self.ser.dtr = False
                time.sleep(0.05)
                self.ser.dtr = True
            except Exception:
                pass

            # Clear any old buffered data on the port
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            time.sleep(0.2)  # small settle time to let the device boot/respond
            self._log(f"Opened {port} @ {self.ser.baudrate} baud.")
            return True
        except Exception as e:
            messagebox.showerror("Serial error", f"Could not open port:\n{e}")
            return False

    def _close_serial(self):
        """Close serial port if open (safe to call multiple times)."""
        if self.ser:
            try:
                self.ser.close()
                self._log("Serial port closed.")
            except Exception:
                pass
            self.ser = None

    # -------------------- Actions --------------------
    def _on_start(self):
        """Handler for the Start button: send the single-byte '1' trigger to the ESP32.

        If the device has not yet announced READY, wait a short time before sending
        to avoid racing with the ESP32 boot sequence.
        """
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("Not connected", "Connect first.")
            return
        if not self.ready_seen:
            # Defer slightly to allow the device to finish booting/reset sequence
            self.status_var.set("Waiting for READY…")
            self._log("Device not READY yet; delaying send 1 by 800 ms.")
            self.after(800, self._send_start_trigger)  # send a bit later
        else:
            self._send_start_trigger()

    def _send_start_trigger(self):
        """Clear prior data, flip UI into collecting state, and send the '1' byte."""
        with self.data_lock:
            self.data_rows = []
        self.collecting = True
        self.start_btn["state"] = "disabled"
        self.stop_btn["state"] = "normal"
        self.status_var.set("Experiment running…")
        try:
            # Clear buffers to avoid mixing stale lines with the new run
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            # Protocol: send single ASCII '1' to instruct the ESP32 to start recording
            self.ser.write(b"1")
            self.ser.flush()
            self._log("Sent start trigger: '1'")
        except Exception as e:
            messagebox.showerror("Serial error", f"Failed to write start byte:\n{e}")
            # Attempt a graceful stop without saving when the write fails
            self._on_stop_and_save(force_save=False)

    def _on_stop_and_save(self, force_save=True):
        """Stop collection and, unless force_save==False, save the collected rows to CSV."""
        self.collecting = False
        self.stop_btn["state"] = "disabled"
        self.start_btn["state"] = "normal"
        saved = False
        if force_save:
            saved = self._save_csv()
        self.status_var.set("Saved and ready." if saved else "Ready.")

    def _on_exit(self):
        """Gracefully exit the application: stop thread, close serial and destroy window."""
        # Signal reader thread to stop and wait briefly for it
        self.stop_event.set()
        self.collecting = False
        try:
            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join(timeout=1.0)
        except Exception:
            pass
        self._close_serial()
        self.destroy()

    def _on_connect(self):
        """Open the selected serial port and (if not running) start the reader thread."""
        if not self._open_serial():
            return

        # Start the reader thread if not already running
        if self.read_thread is None or not self.read_thread.is_alive():
            self.stop_event.clear()
            # Daemon thread so it won't prevent process exit
            self.read_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.read_thread.start()

        # Clear any stale data
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass

        self._log(f"Connecting to {self.ser.port} @ {self.ser.baudrate} baud.")
        self.status_var.set("Connected. Ready to start.")
        # Update UI states
        self.connect_btn["state"] = "disabled"
        self.disconnect_btn["state"] = "normal"
        self.start_btn["state"] = "normal"
        self.stop_btn["state"] = "disabled"

    def _on_disconnect(self):
        """Stop the reader thread, close the port, and update the UI."""
        self.collecting = False
        self.stop_event.set()
        try:
            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join(timeout=1.0)
        except Exception:
            pass
        self.read_thread = None

        self._close_serial()

        self.status_var.set("Disconnected.")
        self._log("Disconnected.")
        # Update UI states
        self.connect_btn["state"] = "normal"
        self.disconnect_btn["state"] = "disabled"
        self.start_btn["state"] = "disabled"
        self.stop_btn["state"] = "disabled"

    # -------------------- Reader & parsing --------------------
    def _reader_loop(self):
        """Background thread loop that reads raw bytes from serial and yields complete lines.

        Implementation details:
        - Maintains a small local byte buffer to accumulate partial reads.
        - Extracts lines split by LF (\\n). Carriage returns are stripped when decoding.
        - Posts every decoded, non-empty line to the lines_queue for display.
        - Calls _handle_line() on each line (runs in the reader thread); that method
          is careful to only perform thread-safe actions and to schedule UI work on
          the main thread via self.after(...) when necessary.
        - Sleeps briefly if no data is available (controlled by READ_THREAD_SLEEP_S)
          to avoid pegging the CPU.
        """
        buffer = b""
        while not self.stop_event.is_set():
            try:
                if self.ser and self.ser.is_open:
                    # Read up to 1024 bytes using the serial timeout to avoid blocking forever
                    chunk = self.ser.read(1024)
                    if chunk:
                        buffer += chunk
                        # Process all complete lines available in buffer
                        while b"\n" in buffer:
                            line, buffer = buffer.split(b"\n", 1)
                            # Decode line robustly and strip whitespace
                            line = line.strip().decode(errors="ignore")
                            if line:
                                self._handle_line(line)
                    else:
                        # No data this iteration: sleep briefly to avoid busy-looping
                        time.sleep(READ_THREAD_SLEEP_S)
                else:
                    # Serial not open: sleep a bit and retry
                    time.sleep(0.2)
            except Exception as e:
                # Forward read errors to UI log and continue; don't let the thread die silently.
                self.lines_queue.put(("log", f"[Serial read error] {e}"))
                time.sleep(0.2)

    def _handle_line(self, line: str):
        """Process a single text line from the ESP32.

                Expected data line format: "PWM;pos;vel;vel_filtered;time"
          - PWM: integer (but tolerant to floats like "12.0")
          - pos: float (radians)
                    - vel: float (rad/s)
                    - vel_filtered: float (rad/s)
                    - time: float (milliseconds)

        Special control lines:
          - "READY": device readiness handshake; recorded in self.ready_seen and
                     optionally updates the status label.
          - "END": signals end of the experiment; schedules a save on the main thread.

        This function:
        - Always posts the raw line to the UI log via lines_queue.
        - If collecting==True and the line matches the expected 4-field format,
          parses the row and appends it to data_rows under data_lock.
        - Ignores malformed lines silently.
        """
        # Echo every line in the log (UI thread will display)
        self.lines_queue.put(("log", line))

        if line.strip() == "READY":
            self.ready_seen = True
            # Update status on the main thread immediately
            self.after(0, lambda: self.status_var.set("Connected. READY from device."))
            return

        # If ESP32 signals end of experiment with "END", stop & save on main thread
        if line == "END":
            # Use after() to ensure UI actions run in main thread
            self.after(0, lambda: self._on_stop_and_save(force_save=True))
            return

        # If not currently collecting, ignore parsed data
        if not self.collecting:
            return

        # Parse semicolon-separated fields
        parts = line.split(";")
        if len(parts) != 5:
            return  # ignore malformed lines

        try:
            # Accept integer or float-looking PWM, be tolerant to formatting
            pwm = int(float(parts[0]))
            pos = float(parts[1])
            vel = float(parts[2])
            velf = float(parts[3])
            tms = float(parts[4])  # milliseconds reported by ESP32
        except ValueError:
            return  # ignore parse errors

        # Append row to shared buffer under lock
        with self.data_lock:
            self.data_rows.append([pwm, pos, vel, velf, tms])

    # -------------------- Save CSV --------------------
    def _save_csv(self):
        """Write collected rows to CSV file. Returns True on success.

        Behavior:
        - Copies data_rows under lock to avoid holding the lock while performing IO.
        - If no rows are present, logs and returns False.
        - Ensures the output directory exists.
        - Writes a header then the saved rows using csv.writer.
        - Reports success or failure via the UI log and message boxes.
        """
        with self.data_lock:
            rows = list(self.data_rows)

        if not rows:
            self._log("No data to save.")
            return False

        out_path = self.file_entry.get().strip()
        if not out_path:
            messagebox.showwarning(
                "Missing path", "Please specify an output .csv filename."
            )
            return False
        if not out_path.lower().endswith(".csv"):
            out_path += ".csv"

        out_dir = os.path.dirname(out_path) or "."
        os.makedirs(out_dir, exist_ok=True)

        try:
            # Write header + data rows
            with open(out_path, mode="w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["PWM", "pos_rad", "vel_rad_per_s", "vel_filtered_rad_per_s", "time_ms"])
                writer.writerows(rows)
            self._log(f"Saved {len(rows)} rows to:\n{out_path}")
            return True
        except Exception as e:
            messagebox.showerror("Save error", f"Failed to save CSV:\n{e}")
            return False

    # -------------------- UI helpers --------------------
    def _browse_outfile(self):
        """Open file-save dialog and update file_entry with chosen path."""
        initial = self.file_entry.get()
        if not initial.lower().endswith(".csv"):
            initial += ".csv"
        file = filedialog.asksaveasfilename(
            title="Choose output .csv file",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile=os.path.basename(initial),
            initialdir=(
                os.path.dirname(initial) if os.path.dirname(initial) else os.getcwd()
            ),
        )
        if file:
            self.file_entry.delete(0, tk.END)
            self.file_entry.insert(0, file)

    def _process_lines_queue(self):
        """Periodically called on the main thread to drain the lines queue and append to the log widget.

        This keeps all Tkinter UI updates on the main thread while allowing the
        reader thread to run without interacting with Tk directly.
        """
        try:
            while True:
                kind, payload = self.lines_queue.get_nowait()
                if kind == "log":
                    self._log(payload)
        except queue.Empty:
            pass
        # Re-schedule
        self.after(100, self._process_lines_queue)

    def _log(self, message: str):
        """Append a line to the read-only log text widget (must be called from main thread)."""
        self.log.configure(state="normal")
        self.log.insert("end", message + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    # -------------------- Plot helpers --------------------
    def _init_plot(self):
        # Create matplotlib Figure with two subplots: position and velocity vs time
        # Controls toolbar (pause/resume, series selection)
        toolbar = ttk.Frame(self.plot_tab)
        toolbar.pack(fill="x", padx=8, pady=4)

        self.plot_paused = False
        self.pause_btn = ttk.Button(toolbar, text="Pause Plot", command=self._toggle_plot_pause)
        self.pause_btn.pack(side="left")

        # Series selection checkboxes
        self.show_pos_var = tk.IntVar(value=1)
        self.show_vel_var = tk.IntVar(value=1)
        self.show_velf_var = tk.IntVar(value=1)

        ttk.Checkbutton(toolbar, text="Position", variable=self.show_pos_var, command=self._on_series_toggle).pack(side="left", padx=6)
        ttk.Checkbutton(toolbar, text="Velocity", variable=self.show_vel_var, command=self._on_series_toggle).pack(side="left", padx=6)
        ttk.Checkbutton(toolbar, text="Velocity (filtered)", variable=self.show_velf_var, command=self._on_series_toggle).pack(side="left", padx=6)

        self.fig = Figure(figsize=(6, 4), dpi=100)
        self.ax_pos = self.fig.add_subplot(2, 1, 1)
        self.ax_vel = self.fig.add_subplot(2, 1, 2, sharex=self.ax_pos)

        self.ax_pos.set_ylabel("pos [rad]")
        self.ax_vel.set_ylabel("vel [rad/s]")
        self.ax_vel.set_xlabel("time [s]")
        self.ax_pos.grid(True, linestyle=":", alpha=0.6)
        self.ax_vel.grid(True, linestyle=":", alpha=0.6)

        (self.line_pos,) = self.ax_pos.plot([], [], color="tab:blue", label="pos")
        (self.line_vel,) = self.ax_vel.plot([], [], color="tab:orange", label="vel")
        (self.line_vel_filt,) = self.ax_vel.plot([], [], color="tab:green", label="vel_filt")
        self.ax_pos.legend(loc="upper right")
        self.ax_vel.legend(loc="upper right")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_tab)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Apply initial visibility based on checkboxes
        self._on_series_toggle()

    def _update_plot(self):
        # If paused, skip updating the plot (but keep scheduling)
        if self.plot_paused:
            self.after(PLOT_UPDATE_MS, self._update_plot)
            return

        # Copy data under lock to avoid blocking reader
        with self.data_lock:
            rows = list(self.data_rows)

        if rows:
            # Each row is [PWM, pos, vel, vel_filtered, tms]
            # Limit number of points for performance
            if len(rows) > MAX_PLOT_POINTS:
                rows = rows[-MAX_PLOT_POINTS:]
            t = [r[4] / 1000.0 for r in rows]
            pos = [r[1] for r in rows]
            vel = [r[2] for r in rows]
            velf = [r[3] for r in rows]

            # Update series data
            self.line_pos.set_data(t, pos)
            self.line_vel.set_data(t, vel)
            self.line_vel_filt.set_data(t, velf)

            # Autoscale axes to data
            self.ax_pos.relim()
            self.ax_pos.autoscale_view()
            self.ax_vel.relim()
            self.ax_vel.autoscale_view()

            # Ensure x-limits cover data
            if t:
                self.ax_pos.set_xlim(t[0], t[-1] if t[-1] > t[0] else t[0] + 1e-3)

            self.canvas.draw_idle()

        # Schedule next update
        self.after(PLOT_UPDATE_MS, self._update_plot)

    def _toggle_plot_pause(self):
        self.plot_paused = not self.plot_paused
        self.pause_btn.configure(text="Resume Plot" if self.plot_paused else "Pause Plot")

    def _on_series_toggle(self):
        # Toggle visibility based on checkboxes
        self.line_pos.set_visible(bool(self.show_pos_var.get()))
        self.line_vel.set_visible(bool(self.show_vel_var.get()))
        self.line_vel_filt.set_visible(bool(self.show_velf_var.get()))
        self.canvas.draw_idle()


if __name__ == "__main__":
    app = ESP32StepGUI()
    app.mainloop()

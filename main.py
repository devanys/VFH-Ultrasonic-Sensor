import math, time, threading, collections
import tkinter as tk
from tkinter import ttk
import serial, serial.tools.list_ports
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Wedge, Circle
import matplotlib.patches as mp
import matplotlib.gridspec as gridspec
import matplotlib.lines as mlines

ALPHA   = 10               
N_SEC   = 360 // ALPHA     
A, B    = 1.0, 1.0/200
T_HI    = 0.50
T_LO    = 0.15
L       = 2
MAX_D   = 200.0
FOV     = 30              
HISTORY = 100

BG, BG2 = "#0b0f19", "#131929"
GRID    = "#1e2d40"
ACCENT  = "#00e5ff"
GREEN   = "#00e676"
RED     = "#ff1744"
ORANGE  = "#ff9100"
YELLOW  = "#ffd600"
PURPLE  = "#b388ff"
TEXT    = "#cdd6f4"
MUTED   = "#4a5568"

class VFH:
    def __init__(self):
        self.h      = np.zeros(N_SEC)
        self.hb     = np.zeros(N_SEC)
        self.m      = 0.0
        self.theta  = 0.0
        self.omega  = 0.0
        self.v      = 0.0
        self.blocked= False

    def update(self, d):
        # m_ij = c^2 * (a - b*d)
        self.m = max(0.0, A - B*d) if d < A/B else 0.0

        raw = np.zeros(N_SEC)
        for off in np.arange(-FOV/2, FOV/2 + 1, ALPHA):
            k = int(((0 + off) % 360) / ALPHA) % N_SEC
            w = 1.0 - abs(off) / (FOV/2 + 1)
            raw[k] += self.m * w

        # smooth h'_k
        sm = np.zeros(N_SEC)
        for k in range(N_SEC):
            sm[k] = sum(raw[(k+i) % N_SEC] for i in range(-L, L+1)) / (2*L+1)

        mx = sm.max()
        hn = sm / mx if mx > 0 else sm.copy()
        self.h = hn

        # threshold H^b_k
        for k in range(N_SEC):
            self.hb[k] = 1.0 if hn[k]>T_HI else (0.0 if hn[k]<T_LO else hn[k])

        # steering — nearest free sector to 0°
        free = [k for k in range(N_SEC) if self.hb[k] < T_HI]
        if not free:
            self.theta = self.omega = self.v = 0.0
            self.blocked = True
            return

        angs = [(k*ALPHA + ALPHA/2) % 360 for k in free]
        angs = [a - 360 if a > 180 else a for a in angs]
        all_free = all(self.hb[k] <= T_LO for k in range(N_SEC))
        self.theta   = 0.0 if all_free else min(angs, key=lambda a: abs(a))
        self.omega   = 2.5 * math.radians(self.theta)
        self.v       = max(0.0, 100 * (1 - abs(self.theta) / 180))
        self.blocked = False

class App:
    def __init__(self, root):
        self.root    = root
        self.vfh     = VFH()
        self.ser     = None
        self.running = False
        self.dist    = MAX_D
        self.hist    = collections.deque([MAX_D]*HISTORY, maxlen=HISTORY)

        root.title("VFH — HC-SR04 Ultrasonic Sensor")
        root.configure(bg=BG)
        root.geometry("1150x700")

        self._ui()
        self._plots()
        self._ports()
        self._tick()

    def _ui(self):
        t = tk.Frame(self.root, bg=BG, pady=6)
        t.pack(fill='x', padx=12)
        tk.Label(t, text="VFH", font=("Courier New",18,"bold"), bg=BG, fg=ACCENT).pack(side='left')
        tk.Label(t, text="  Vector Field Histogram  ·  HC-SR04  ·  Arduino",
                 font=("Courier New",9), bg=BG, fg=MUTED).pack(side='left')
        self.lbl_st = tk.Label(t, text="⬤ OFFLINE",
                               font=("Courier New",9,"bold"), bg=BG, fg=RED)
        self.lbl_st.pack(side='right')

        s = tk.Frame(self.root, bg=BG2, pady=7, padx=12)
        s.pack(fill='x', padx=12, pady=(0,6))
        tk.Label(s, text="PORT", font=("Courier New",8), bg=BG2, fg=MUTED).pack(side='left')
        self.pv = tk.StringVar()
        self.cb = ttk.Combobox(s, textvariable=self.pv, width=12, state='readonly')
        self.cb.pack(side='left', padx=6)
        tk.Button(s, text="↺", font=("Courier New",9,"bold"), bg=BG, fg=ACCENT,
                  relief='flat', cursor='hand2', command=self._ports).pack(side='left')
        self.btn = tk.Button(s, text="▶ CONNECT", font=("Courier New",9,"bold"),
                             bg="#6c63ff", fg='white', relief='flat', padx=12,
                             cursor='hand2', command=self._toggle)
        self.btn.pack(side='left', padx=10)
        self.lbl_d = tk.Label(s, text="---.- cm",
                              font=("Courier New",18,"bold"), bg=BG2, fg=ACCENT)
        self.lbl_d.pack(side='left', padx=16)
        self.lbl_i = tk.Label(s, text="", font=("Courier New",9), bg=BG2, fg=TEXT)
        self.lbl_i.pack(side='left')

    def _plots(self):
        self.fig = plt.Figure(figsize=(11, 5.8), facecolor=BG)
        gs = gridspec.GridSpec(1, 3, figure=self.fig,
                               wspace=0.28, left=0.05, right=0.97,
                               top=0.91, bottom=0.10)
        self.ax_fov   = self.fig.add_subplot(gs[0])
        self.ax_pol   = self.fig.add_subplot(gs[1], polar=True)
        self.ax_trend = self.fig.add_subplot(gs[2])

        for ax in [self.ax_fov, self.ax_trend]:
            ax.set_facecolor(BG2)
            for sp in ax.spines.values(): sp.set_edgecolor(GRID)
            ax.tick_params(colors=TEXT, labelsize=8)
        self.ax_pol.set_facecolor(BG2)
        self.ax_pol.tick_params(colors=TEXT, labelsize=7)

        cv = FigureCanvasTkAgg(self.fig, master=self.root)
        cv.get_tk_widget().pack(fill='both', expand=True, padx=12, pady=(0,8))
        self.canvas = cv

    def _ports(self):
        ps = [p.device for p in serial.tools.list_ports.comports()]
        self.cb['values'] = ps
        if ps: self.cb.current(0)

    def _toggle(self):
        if self.running: self._disc()
        else:            self._conn()

    def _conn(self):
        p = self.pv.get()
        if not p: return
        try:
            self.ser = serial.Serial(p, 115200, timeout=1)
            time.sleep(2); self.ser.reset_input_buffer()
            self.running = True
            self.btn.config(text="■ DISCONNECT", bg=RED)
            self.lbl_st.config(text="⬤ ONLINE", fg=GREEN)
            threading.Thread(target=self._read, daemon=True).start()
        except:
            self.lbl_st.config(text="⬤ ERR", fg=RED)

    def _disc(self):
        self.running = False
        if self.ser: self.ser.close(); self.ser = None
        self.btn.config(text="▶ CONNECT", bg="#6c63ff")
        self.lbl_st.config(text="⬤ OFFLINE", fg=RED)

    def _read(self):
        while self.running and self.ser:
            try:
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw or raw in ("READY","ERROR"): continue
                self.dist = min(float(raw), MAX_D)
                self.hist.append(self.dist)
                self.vfh.update(self.dist)
            except: pass

    def _tick(self):
        self._labels()
        self._draw()
        self.root.after(120, self._tick)

    def _labels(self):
        d = self.dist; v = self.vfh
        c = RED if d<30 else (ORANGE if d<80 else GREEN)
        self.lbl_d.config(text=f"{d:6.1f} cm", fg=c)
        st = "BLOCKED" if v.blocked else ("WARNING" if d<50 else "CLEAR")
        sc = RED if v.blocked else (ORANGE if d<50 else GREEN)
        self.lbl_i.config(
            text=f"θ={v.theta:+.1f}°  ω={v.omega:+.2f} rad/s  V={v.v:.1f} cm/s  [{st}]",
            fg=sc)

    def _draw(self):
        v = self.vfh; h = v.h; hb = v.hb

        def col(k):
            return RED if hb[k]>=T_HI else (GREEN if hb[k]<=T_LO else ORANGE)
        colors = [col(k) for k in range(N_SEC)]
        ax = self.ax_fov
        ax.clear(); ax.set_facecolor(BG2)
        ax.set_xlim(-220,220); ax.set_ylim(-25,215); ax.set_aspect('equal')

        for r in [50,100,150,200]:
            ax.add_patch(Circle((0,0),r,color=GRID,fill=False,lw=0.7))
            ax.text(3,r+2,f"{r}cm",color=MUTED,fontsize=6)

        ax.add_patch(Wedge((0,0),MAX_D,90-FOV/2,90+FOV/2,color=ACCENT,alpha=0.07))
        for sgn in [-1,1]:
            a = math.radians(90 + sgn*FOV/2)
            ax.plot([0,MAX_D*math.cos(a)],[0,MAX_D*math.sin(a)],
                    color=ACCENT,lw=0.8,ls='--',alpha=0.45)

        dp = min(self.dist,MAX_D)
        c  = RED if self.dist<30 else (ORANGE if self.dist<80 else GREEN)
        ax.plot([0,0],[0,dp],color=c,lw=2.5,alpha=0.9,zorder=4)
        ax.scatter([0],[dp],s=110,color=c,edgecolors='white',lw=0.8,zorder=5)
        ax.text(7,dp+4,f"{self.dist:.1f} cm",color=c,fontsize=9,fontweight='bold')
        ax.add_patch(plt.Rectangle((-22,-14),44,14,color="#6c63ff",zorder=10))
        ax.text(0,-7,"HC-SR04",color='white',fontsize=6.5,
                ha='center',va='center',fontweight='bold',zorder=11)
        ax.text(-112,185,f"−{FOV//2}°",color=ACCENT,fontsize=7,alpha=0.7)
        ax.text(98, 185,f"+{FOV//2}°",color=ACCENT,fontsize=7,alpha=0.7)

        ax.set_title(f"FOV Sensor View  (±{FOV//2}°  /  {FOV}° total)",
                     color=TEXT, fontsize=9, pad=5)
        ax.set_xlabel("X (cm)", color=TEXT, fontsize=8)
        ax.set_ylabel("Y (cm)", color=TEXT, fontsize=8)
        for sp in ax.spines.values(): sp.set_edgecolor(GRID)

        ax = self.ax_pol
        ax.clear(); ax.set_facecolor(BG2)

        for k in range(N_SEC):
            angle = np.deg2rad(k * ALPHA + ALPHA/2)
            val   = h[k]
            if val < 0.005: continue
            c_bar = col(k)
            ax.plot([angle, angle], [0.0, val],
                    color=c_bar, lw=2.8, alpha=0.85, solid_capstyle='round')

        ring = np.linspace(0, 2*np.pi, 360)
        for r, lc, ls in [(T_HI, RED, '--'), (T_LO, GREEN, '--')]:
            ax.plot(ring, [r]*360, color=lc, lw=0.8, ls=ls, alpha=0.55)

        ax.set_xlabel("Histogram thresholds", color=MUTED, fontsize=7, labelpad=8)

        ax.annotate("", xy=(np.deg2rad(0), 1.05), xytext=(np.deg2rad(0), 0.0),
                    arrowprops=dict(arrowstyle="->,head_width=0.35",
                                    color=YELLOW, lw=1.6))

        if not v.blocked:
            a_s = np.deg2rad(v.theta % 360)
            steer_col = GREEN if v.theta == 0.0 else PURPLE
            ax.annotate("", xy=(a_s, 0.95), xytext=(np.deg2rad(v.theta % 360), 0.0),
                        arrowprops=dict(arrowstyle="->,head_width=0.4",
                                        color=steer_col, lw=2.0))

        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_rlim(0, 1.1)
        ax.set_rticks([0.25, 0.5, 0.75, 1.0])
        ax.set_yticklabels(["0.25","0.5","0.75","1.0"],
                           color=MUTED, fontsize=6)
        ax.set_thetagrids(range(0, 360, 30), fontsize=7)
        ax.tick_params(colors=TEXT, labelsize=7)
        ax.grid(color=GRID, lw=0.5, alpha=0.7)
        ax.spines['polar'].set_color(GRID)

        ax.set_title("Polar Obstacle Density", color=TEXT, fontsize=9, pad=12)

        leg = [
            mlines.Line2D([],[],color=YELLOW, lw=1.6, label="Target direction"),
            mlines.Line2D([],[],color=PURPLE, lw=2.0, label="Steering direction"),
            mlines.Line2D([],[],color=RED,    lw=0.8, ls='--', label="Distance limits"),
            mp.Patch(color=RED,   alpha=0.8, label="Blocked"),
            mp.Patch(color=GREEN, alpha=0.8, label="Free"),
        ]
        ax.legend(handles=leg, loc='lower left', fontsize=6,
                  facecolor=BG, labelcolor=TEXT, framealpha=0.85,
                  bbox_to_anchor=(-0.18, -0.18))

        ax = self.ax_trend
        ax.clear(); ax.set_facecolor(BG2)

        hs = list(self.hist); xs = list(range(len(hs)))
        ax.axhspan(0,  30,  alpha=0.12, color=RED,    zorder=0)
        ax.axhspan(30, 80,  alpha=0.07, color=ORANGE, zorder=0)
        ax.axhspan(80, 200, alpha=0.04, color=GREEN,  zorder=0)
        ax.axhline(30, color=RED,    lw=0.8, ls=':', alpha=0.5, label="30 cm")
        ax.axhline(80, color=ORANGE, lw=0.8, ls=':', alpha=0.5, label="80 cm")

        ax.fill_between(xs, hs, alpha=0.2, color=ACCENT)
        ax.plot(xs, hs, color=ACCENT, lw=1.8, zorder=4)
        if hs:
            dc = RED if hs[-1]<30 else (ORANGE if hs[-1]<80 else GREEN)
            ax.scatter([xs[-1]], [hs[-1]], s=55, color=dc, zorder=5)

        ax.set_xlim(0, HISTORY-1); ax.set_ylim(0, MAX_D+10)
        ax.set_title("Distance History  d_ij", color=TEXT, fontsize=9, pad=5)
        ax.set_xlabel("Sample", color=TEXT, fontsize=8)
        ax.set_ylabel("Distance (cm)", color=TEXT, fontsize=8)
        ax.legend(fontsize=7, facecolor=BG, labelcolor=TEXT, framealpha=0.8)
        ax.grid(color=GRID, lw=0.5, zorder=1)
        for sp in ax.spines.values(): sp.set_edgecolor(GRID)
        ax.yaxis.set_tick_params(labelcolor=TEXT)

        self.canvas.draw_idle()
if __name__ == "__main__":
    root = tk.Tk()
    ttk.Style(root).configure('TCombobox',
        fieldbackground="#1a2235", background="#1a2235",
        foreground=TEXT, arrowcolor=ACCENT)
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app._disc(), root.destroy()))
    root.mainloop()

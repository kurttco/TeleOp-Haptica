#!/usr/bin/env python3
"""
FORCE DISPLAY — 5-Direction (Fx, Fy, Fz) visualization

Shows:
  - XY force direction arrow
  - Per-direction bars: Right, Left, Forward, Back, Bottom
  - Time history of Fx, Fy, Fz, and |F|
  - Contact status

USAGE:
    python3 force_display5.py
"""

import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class ForceDisplay5Dir(Node):
    def __init__(self):
        super().__init__('force_display_5dir')

        self.sub = self.create_subscription(
            WrenchStamped, '/slave/force', self.cb, 10
        )

        n = 500
        self.hist_t = np.zeros(n)
        self.hist_fx = np.zeros(n)
        self.hist_fy = np.zeros(n)
        self.hist_fz = np.zeros(n)
        self.hist_fm = np.zeros(n)

        self.idx = 0
        self.t0 = time.time()

        self.fx = 0.0
        self.fy = 0.0
        self.fz = 0.0
        self.fm = 0.0
        self.peak = 0.0
        self.contact = False
        self.contact_threshold = 2.5

    def cb(self, msg: WrenchStamped):
        self.fx = float(msg.wrench.force.x)
        self.fy = float(msg.wrench.force.y)
        self.fz = float(msg.wrench.force.z)
        self.fm = float(np.sqrt(self.fx**2 + self.fy**2 + self.fz**2))
        self.peak = max(self.peak, self.fm)
        self.contact = self.fm > self.contact_threshold

        i = self.idx % len(self.hist_t)
        self.hist_t[i] = time.time() - self.t0
        self.hist_fx[i] = self.fx
        self.hist_fy[i] = self.fy
        self.hist_fz[i] = self.fz
        self.hist_fm[i] = self.fm
        self.idx += 1


def main():
    rclpy.init()
    node = ForceDisplay5Dir()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig = plt.figure(figsize=(13, 8), facecolor='#0a0a0f')
    fig.suptitle(
        'TE3001B — Sensor de Fuerza 5 Direcciones',
        color='white', fontsize=14, fontweight='bold', y=0.97
    )

    bg = '#0d1117'

    # ===== Top-left: XY direction =====
    ax_arrow = fig.add_subplot(2, 2, 1, facecolor=bg)
    ax_arrow.set_xlim(-22, 22)
    ax_arrow.set_ylim(-22, 22)
    ax_arrow.set_aspect('equal')
    ax_arrow.set_title('Direccion de Fuerza en XY', color='white', fontsize=12)
    ax_arrow.tick_params(colors='#aaa')
    for s in ax_arrow.spines.values():
        s.set_edgecolor('#333')

    theta = np.linspace(0, 2*np.pi, 200)
    ax_arrow.plot(20*np.cos(theta), 20*np.sin(theta), '--', color='#333', lw=1)
    ax_arrow.plot(10*np.cos(theta), 10*np.sin(theta), '--', color='#222', lw=0.7)
    ax_arrow.axhline(0, color='#222', lw=0.7)
    ax_arrow.axvline(0, color='#222', lw=0.7)
    ax_arrow.text(0, 21, 'FWD', color='#666', fontsize=8, ha='center')
    ax_arrow.text(0, -21, 'BACK', color='#666', fontsize=8, ha='center')
    ax_arrow.text(21, 0, 'RIGHT', color='#666', fontsize=8, ha='center')
    ax_arrow.text(-21, 0, 'LEFT', color='#666', fontsize=8, ha='center')

    txt_mag = ax_arrow.text(
        0.5, 0.02, '',
        transform=ax_arrow.transAxes,
        color='white', fontsize=13, fontweight='bold',
        ha='center'
    )
    txt_status = ax_arrow.text(
        0.5, 0.92, '',
        transform=ax_arrow.transAxes,
        color='#00FF00', fontsize=12, fontweight='bold',
        ha='center',
        bbox=dict(boxstyle='round,pad=0.3', fc='#000', alpha=0.7)
    )

    # ===== Top-right: direction bars =====
    ax_bars = fig.add_subplot(2, 2, 2, facecolor=bg)
    ax_bars.set_title('Fuerza por Direccion', color='white', fontsize=12)
    ax_bars.set_ylabel('F [N]', color='#aaa')
    ax_bars.tick_params(colors='#aaa')
    for s in ax_bars.spines.values():
        s.set_edgecolor('#333')

    bar_labels = [
        'Fx+\n(Right)',
        'Fx-\n(Left)',
        'Fy+\n(Fwd)',
        'Fy-\n(Back)',
        'Fz+\n(Bottom)'
    ]

    bars = ax_bars.bar(
        bar_labels,
        [0, 0, 0, 0, 0],
        color=['#00BFFF', '#FF6B6B', '#69FF47', '#FFD700', '#C77DFF']
    )
    ax_bars.set_ylim(0, 20)
    ax_bars.axhline(
        y=node.contact_threshold,
        color='#FF4444',
        lw=1,
        ls='--',
        alpha=0.5
    )

    # ===== Bottom: history =====
    ax_plot = fig.add_subplot(2, 1, 2, facecolor=bg)
    ax_plot.set_title('Historial de Fuerza', color='white', fontsize=12)
    ax_plot.set_xlabel('Tiempo [s]', color='#aaa')
    ax_plot.set_ylabel('Fuerza [N]', color='#aaa')
    ax_plot.tick_params(colors='#aaa')
    ax_plot.grid(True, color='#1e2530', ls='--', alpha=0.5)
    for s in ax_plot.spines.values():
        s.set_edgecolor('#333')

    line_fx, = ax_plot.plot([], [], color='#00BFFF', lw=2, label='Fx')
    line_fy, = ax_plot.plot([], [], color='#69FF47', lw=2, label='Fy')
    line_fz, = ax_plot.plot([], [], color='#C77DFF', lw=2, label='Fz')
    line_fm, = ax_plot.plot([], [], color='white', lw=2.5, label='|F|')

    ax_plot.axhline(
        y=node.contact_threshold,
        color='#FF4444',
        lw=1,
        ls=':',
        alpha=0.6,
        label='Umbral'
    )
    ax_plot.axhline(y=-node.contact_threshold, color='#FF4444', lw=1, ls=':', alpha=0.6)
    ax_plot.axhline(y=0, color='#444', lw=0.6)
    ax_plot.set_ylim(-20, 20)
    ax_plot.legend(
        loc='upper right',
        fontsize=9,
        facecolor='#1a1a2e',
        labelcolor='white'
    )

    plt.tight_layout(rect=[0, 0.02, 1, 0.95])

    def animate(_):
        n = min(node.idx, len(node.hist_t))

        if n == 0:
            txt_mag.set_text('Fx=+0.0  Fy=+0.0  Fz=+0.0  |F|=0.0 N')
            txt_status.set_text('Sin contacto')
            txt_status.set_color('#00FF00')
            return [txt_mag, txt_status] + list(bars) + [line_fx, line_fy, line_fz, line_fm]

        fx = node.fx
        fy = node.fy
        fz = node.fz
        fm = node.fm

        # Redraw XY arrow
        ax_arrow.patches.clear()

        fm_xy = np.sqrt(fx**2 + fy**2)
        if fm_xy > 0.05:
            scale = min(fm_xy, 20.0)
            ux = fx / fm_xy
            uy = fy / fm_xy
            x2 = ux * scale
            y2 = uy * scale
            color = '#FF4444' if node.contact else '#00BFFF'

            ax_arrow.arrow(
                0, 0, x2, y2,
                width=0.6,
                head_width=2.0,
                head_length=2.5,
                length_includes_head=True,
                color=color
            )

        txt_mag.set_text(f'Fx={fx:+.1f}  Fy={fy:+.1f}  Fz={fz:+.1f}  |F|={fm:.1f} N')

        if node.contact:
            txt_status.set_text(f'!! CONTACTO !!  Peak: {node.peak:.1f} N')
            txt_status.set_color('#FF0000')
        else:
            txt_status.set_text('Sin contacto')
            txt_status.set_color('#00FF00')

        # Direction bars
        bar_vals = [
            max(fx, 0.0),
            max(-fx, 0.0),
            max(fy, 0.0),
            max(-fy, 0.0),
            max(fz, 0.0),
        ]

        for bar, val in zip(bars, bar_vals):
            bar.set_height(val)
            if val > 1.0:
                bar.set_color('#FF4444' if node.contact else '#00BFFF')
            else:
                if bar is bars[4]:
                    bar.set_color('#C77DFF')
                else:
                    bar.set_color('#00BFFF')

        # History
        i0 = node.idx % len(node.hist_t)
        idx = np.arange(i0, i0 + n) % len(node.hist_t)

        t = node.hist_t[idx]
        fx_hist = node.hist_fx[idx]
        fy_hist = node.hist_fy[idx]
        fz_hist = node.hist_fz[idx]
        fm_hist = node.hist_fm[idx]

        tw = 10.0
        ct = time.time() - node.t0
        mask = t > ct - tw

        if np.any(mask):
            t_plot = t[mask]
            line_fx.set_data(t_plot, fx_hist[mask])
            line_fy.set_data(t_plot, fy_hist[mask])
            line_fz.set_data(t_plot, fz_hist[mask])
            line_fm.set_data(t_plot, fm_hist[mask])

            ax_plot.set_xlim(max(0, ct - tw), max(tw, ct))
            ax_plot.set_ylim(-20, 20)
        else:
            line_fx.set_data([], [])
            line_fy.set_data([], [])
            line_fz.set_data([], [])
            line_fm.set_data([], [])

        return [txt_mag, txt_status] + list(bars) + [line_fx, line_fy, line_fz, line_fm]

    ani = animation.FuncAnimation(
        fig,
        animate,
        interval=50,
        blit=False,
        cache_frame_data=False
    )

    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
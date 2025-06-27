#!/usr/bin/env python3
import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.tri import Triangulation

if __name__ == "__main__":
    sns.set_theme(style="darkgrid", palette="flare")
    matplotlib.rcParams['mathtext.fontset'] = 'stix'
    matplotlib.rcParams['font.family'] = 'STIXGeneral'
    matplotlib.pyplot.title(r'ABC123 vs $\mathrm{ABC123}^{123}$')

    fixed_lambda_d = 0.4
    plot_overview = True
    plot_lambda_d = True
    plot_lambda_v = True

    if plot_lambda_v:
        b_1 = np.array([0.1, 0.4, 0.1])
        b_2 = np.array([0.4, 0.1, 0.1])
        b_3 = np.array([0.1, 0.1, 0.4])
        v_1 = b_2 - b_1
        v_2 = b_3 - b_1
        n = np.cross(v_1, v_2)
        n /= np.linalg.norm(n)
        u = v_1 / np.linalg.norm(v_1)
        v = np.cross(n, u)
        px, py, pz, qx, qy = [], [], [], [], []
        qz = {'R_g_avg': [],
              'D_avg_avg': [],
              'D_max_avg': [],
              'R_g_max': [],
              'D_avg_max': [],
              'D_max_max': []}


        def projector(p):
            p_c = p - np.array([0.2, 0.2, 0.2])
            u_coord = np.dot(p_c, u)
            v_coord = np.dot(p_c, v)
            return np.array([u_coord, v_coord])

    data_folder = os.path.join(os.path.dirname(__file__), "data")
    files = [f for f in os.listdir(data_folder)
             if f.startswith('u-turn_out_')
             and f.endswith('_post.npz')
             and os.path.isfile(os.path.join(data_folder, f))]

    fig_all, axs_all = plt.subplots(nrows=3, ncols=1, figsize=(8, 10), dpi=200)
    fig_d, axs_d = plt.subplots(nrows=3, ncols=2, figsize=(8, 10), dpi=200)
    for file in files:
        data = np.load(os.path.join(data_folder, file))

        # plot overview
        if plot_overview:
            t = np.arange(0, len(data['R_g'])) * 0.1
            axs_all[0].plot(t, data['R_g'])
            axs_all[1].plot(t, data['D_avg'])
            axs_all[2].plot(t, data['D_max'])

        # get parameters
        params = dict(zip(['lambda_d', 'lambda_heading_virtual', 'lambda_heading_neighbors', 'lambda_contraction'],
                          [float(ps) for ps in file[11:-9].split('_')]))

        # lambda_d scatter plot
        if plot_lambda_d:
            axs_d[0, 0].scatter(params['lambda_d'], data['R_g_avg'])
            axs_d[1, 0].scatter(params['lambda_d'], data['D_avg_avg'])
            axs_d[2, 0].scatter(params['lambda_d'], data['D_max_avg'])
            axs_d[0, 1].scatter(params['lambda_d'], data['R_g_max'])
            axs_d[1, 1].scatter(params['lambda_d'], data['D_avg_max'])
            axs_d[2, 1].scatter(params['lambda_d'], data['D_max_max'])

        # 3d scatter
        if plot_lambda_v:
            if abs(params['lambda_d'] - fixed_lambda_d) <= 1e-6:
                p = np.array([params['lambda_heading_virtual'],
                              params['lambda_heading_neighbors'],
                              params['lambda_contraction']])
                px.append(p[0])
                py.append(p[1])
                pz.append(p[2])
                q = projector(p)
                qx.append(q[0])
                qy.append(q[1])
                qz['R_g_avg'].append(data['R_g_avg'])
                qz['D_avg_avg'].append(data['D_avg_avg'])
                qz['D_max_avg'].append(data['D_max_avg'])
                qz['R_g_max'].append(data['R_g_max'])
                qz['D_avg_max'].append(data['D_avg_max'])
                qz['D_max_max'].append(data['D_max_max'])

        data.close()

    # overall plot cosmetics
    if plot_overview:
        y_label = ['$R_g$', '$D_{avg}$', '$D_{max}$']
        for ax, yl in zip(axs_all, y_label):
            ax.set_yscale('log')
            ax.set_ylabel(yl)
            ax.set_xlabel('time [s]')
        fig_all.suptitle('U-Turn Experiment: All Parameter Sets')
        fig_all.tight_layout()
        fig_all.show()

    # lambda_d plot cosmetics
    if plot_lambda_d:
        axs_d[0, 0].set_ylabel(r'$\text{avg}(R_{g}$)')
        axs_d[1, 0].set_ylabel(r'$\text{avg}(D_\text{avg}$)')
        axs_d[2, 0].set_ylabel(r'$\text{avg}(D_\text{max}$)')
        axs_d[0, 1].set_ylabel(r'$\text{max}(R_{g}$)')
        axs_d[1, 1].set_ylabel(r'$\text{max}(D_\text{avg}$)')
        axs_d[2, 1].set_ylabel(r'$\text{max}(D_\text{max}$)')
        for ax in axs_d.flatten():
            ax.set_xlabel('lambda_d')
        fig_d.suptitle(r'U-Turn Experiment: $\lambda_d$ Evaluation')
        fig_d.tight_layout()
        fig_d.show()

    # lambda plot
    if plot_lambda_v:
        key_list = ['R_g_avg', 'D_avg_avg', 'D_max_avg', 'R_g_max', 'D_avg_max', 'D_max_max', ]
        metric_label_list = [r'$\text{avg}(R_{g}$)', r'$\text{avg}(D_\text{avg}$)', r'$\text{avg}(D_\text{max}$)',
                             r'$\text{max}(R_{g}$)', r'$\text{max}(D_\text{avg}$)', r'$\text{max}(D_\text{max}$)']
        fig_v, axs_v = plt.subplots(nrows=3, ncols=2, figsize=(8, 10), dpi=200, subplot_kw=dict(projection='3d'))
        fig_v_flat, axs_v_flat = plt.subplots(nrows=3, ncols=2, figsize=(8, 10), dpi=200)
        fig_v_proj, axs_v_proj = plt.subplots(nrows=3, ncols=2, figsize=(10, 10), dpi=200)
        size = 40.0

        for ax, key, metric_label in zip(axs_v.flatten(), key_list, metric_label_list):
            pc = ax.scatter(px, py, pz, c=qz[key], cmap='flare')
            fig_v.colorbar(pc, ax=ax, label=metric_label)

            ax.set_facecolor("white")
            ax.view_init(elev=30, azim=30)
            ax.set_xlabel('lambda_heading_virtual')
            ax.set_ylabel('lambda_heading_neighbors')
            ax.set_zlabel('lambda_contraction')

        fig_v.suptitle('U-Turn Experiment: Parameter Evaluation')
        # fig_v.tight_layout()
        fig_v.show()

        for ax, key, metric_label in zip(axs_v_flat.flatten(), key_list, metric_label_list):
            pc = ax.scatter(qx, qy, c=qz[key], s=size, cmap='flare')
            fig_v_flat.colorbar(pc, ax=ax, label=metric_label)

            for x, y, z, u, v in zip(px, py, pz, qx, qy):
                label = f"({x:.1f}; {y:.1f}; {z:.1f})"
                ax.annotate(label, xy=(u, v), xytext=(1e-2, 1e-2),
                            textcoords='offset points', ha='center', va='bottom',
                            fontsize=6)

            ax.set_xticklabels([])
            ax.set_yticklabels([])

        fig_v_flat.suptitle('U-Turn Experiment: Parameter Evaluation')
        fig_v_flat.tight_layout()
        fig_v_flat.show()

        tri = Triangulation(qx, qy)
        for ax, key, metric_label in zip(axs_v_proj.flatten(), key_list, metric_label_list):
            pc = ax.tripcolor(tri, qz[key], cmap='flare', antialiased=True)
            fig_v_proj.colorbar(pc, ax=ax, label=metric_label)

            for x, y, z, u, v in zip(px, py, pz, qx, qy):
                label = f"({x:.1f}; {y:.1f}; {z:.1f})"
                ax.annotate(label, xy=(u, v), xytext=(1e-2, 1e-2),
                            textcoords='offset points', ha='center', va='bottom',
                            fontsize=6)
            ax.set_xticks([])
            ax.set_yticks([])

        fig_v_proj.suptitle('U-Turn Experiment: Parameter Evaluation Projected')
        fig_v_proj.tight_layout()
        fig_v_proj.show()

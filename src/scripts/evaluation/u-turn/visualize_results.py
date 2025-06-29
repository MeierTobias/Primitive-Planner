#!/usr/bin/env python3
import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
from matplotlib.tri import Triangulation

if __name__ == "__main__":
    sns.set_theme(style="darkgrid", palette="flare")
    matplotlib.rcParams['mathtext.fontset'] = 'stix'
    matplotlib.rcParams['font.family'] = 'STIXGeneral'
    matplotlib.pyplot.title(r'ABC123 vs $\mathrm{ABC123}^{123}$')

    fixed_lambda_d = 0.4
    plot_overview = False
    plot_lambda_d = False
    plot_lambda_v = True

    dpi = 200
    grid_size_3x2 = (8, 10)

    data_folder = os.path.join(os.path.dirname(__file__), "data")
    files = [f for f in os.listdir(data_folder)
             if f.startswith('u-turn_out_')
             and f.endswith('_post.npz')
             and os.path.isfile(os.path.join(data_folder, f))]
    print(f"Found {len(files)} experiments.")

    fig_all, axs_all = plt.subplots(nrows=3, ncols=1, figsize=grid_size_3x2, dpi=dpi)

    records = []
    for file in files:
        # load experiment data
        data = np.load(os.path.join(data_folder, file))
        l_d, l_hv, l_hn, l_c = [float(ps) for ps in file[11:-9].split('_')]
        records.append({
            'lambda_d': l_d,
            'lambda_heading_virtual': l_hv,
            'lambda_heading_neighbors': l_hn,
            'lambda_contraction': l_c,
            'R_g_avg': float(data['R_g_avg']),
            'D_avg_avg': float(data['D_avg_avg']),
            'D_max_avg': float(data['D_max_avg']),
            'R_g_max': float(data['R_g_max']),
            'D_avg_max': float(data['D_avg_max']),
            'D_max_max': float(data['D_max_max'])
        })

        # plot overview
        if plot_overview:
            t = np.arange(0, len(data['R_g'])) * 0.1
            axs_all[0].plot(t, data['R_g'])
            axs_all[1].plot(t, data['D_avg'])
            axs_all[2].plot(t, data['D_max'])

        data.close()

    # prepare data index
    df = pd.DataFrame.from_records(records)
    df = df.set_index([
        'lambda_d',
        'lambda_heading_virtual',
        'lambda_heading_neighbors',
        'lambda_contraction'
    ])

    # metric list and lables
    key_list = ['R_g_avg', 'D_avg_avg', 'D_max_avg', 'R_g_max', 'D_avg_max', 'D_max_max', ]
    metric_label_list = [r'$\text{avg}(R_{g}$)', r'$\text{avg}(D_\text{avg}$)', r'$\text{avg}(D_\text{max}$)',
                         r'$\text{max}(R_{g}$)', r'$\text{max}(D_\text{avg}$)', r'$\text{max}(D_\text{max}$)']

    selected_metrics = [0, 5]

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
        fig_d, axs_d = plt.subplots(nrows=3, ncols=2, figsize=grid_size_3x2, dpi=dpi)
        c = np.arange(len(df))
        for ax, key, metric_label in zip(axs_d.T.flatten(), key_list, metric_label_list):
            ax.scatter(df.index.get_level_values('lambda_d'), df[key], c=c, cmap='flare')
            ax.set_xlabel(r'$\lambda_d$')
            ax.set_ylabel(metric_label)
        fig_d.suptitle(r'U-Turn Experiment: $\lambda_d$ Evaluation')
        fig_d.tight_layout()
        fig_d.show()

        fig_d_hist, axs_d_hist = plt.subplots(nrows=3, ncols=2, figsize=grid_size_3x2, dpi=dpi,
                                              subplot_kw=dict(projection='3d'))
        n_bins = 12
        for ax, key, metric_label in zip(axs_d_hist.T.flatten(), key_list, metric_label_list):
            bin_edges = np.logspace(np.log10(df[key].min()), np.log10(df[key].max()), n_bins + 1)
            log_edges = np.log10(bin_edges)
            log_widths = log_edges[1:] - log_edges[:-1]
            log_centers = (log_edges[:-1] + log_edges[1:]) / 2
            lambdas = np.sort(df.index.get_level_values('lambda_d').unique())
            cmap = plt.get_cmap('flare')
            colors = cmap(np.linspace(0, 1, len(lambdas)))

            for l, c in zip(lambdas, colors):
                sub_df = df.xs(l, level='lambda_d')
                hist, bins = np.histogram(sub_df[key], bins=bin_edges, density=True)

                ax.bar(log_centers, hist, zs=l, zdir='y',
                       width=log_widths,
                       alpha=0.9,
                       edgecolor='none',
                       linewidth=0,
                       color=c,
                       )

            step = 2
            tick_pos = log_centers[::step]
            all_labels = [f"{10 ** c:.2g}" for c in log_centers]
            tick_lbls = all_labels[::step]

            ax.set_xticks(tick_pos)
            ax.set_xticklabels(tick_lbls)
            ax.set_xlabel(metric_label)

            ax.set_yticks(lambdas)
            ax.set_ylabel(r'$\lambda_d$')

            ax.set_zlabel("Density")

        fig_d_hist.suptitle(r'U-Turn Experiment: $\lambda_d$ Evaluation')
        fig_d_hist.tight_layout()
        fig_d_hist.show()

    # lambda plot
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
        center = np.array([0.2, 0.2, 0.2])

        P = np.vstack([
            df.index.get_level_values('lambda_heading_virtual'),
            df.index.get_level_values('lambda_heading_neighbors'),
            df.index.get_level_values('lambda_contraction'),
        ]).T
        Pc = P - center
        df['u_coord'] = Pc.dot(u)
        df['v_coord'] = Pc.dot(v)

        fig_v, axs_v = plt.subplots(nrows=3, ncols=2, figsize=grid_size_3x2, dpi=dpi, subplot_kw=dict(projection='3d'))
        fig_v_flat, axs_v_flat = plt.subplots(nrows=3, ncols=2, figsize=grid_size_3x2, dpi=dpi)
        fig_v_proj, axs_v_proj = plt.subplots(nrows=3, ncols=2, figsize=(10, 10), dpi=dpi)
        size = 40.0

        sub_df = df.xs(fixed_lambda_d, level='lambda_d')

        for ax, key, metric_label in zip(axs_v.T.flatten(), key_list, metric_label_list):
            pc = ax.scatter(sub_df.index.get_level_values('lambda_heading_virtual'),
                            sub_df.index.get_level_values('lambda_heading_neighbors'),
                            sub_df.index.get_level_values('lambda_contraction'),
                            c=sub_df[key],
                            cmap='flare')
            fig_v.colorbar(pc, ax=ax, label=metric_label)

            ax.set_facecolor("white")
            ax.view_init(elev=30, azim=30)
            ax.set_xlabel('lambda_heading_virtual')
            ax.set_ylabel('lambda_heading_neighbors')
            ax.set_zlabel('lambda_contraction')

        fig_v.suptitle('U-Turn Experiment: Parameter Evaluation')
        # fig_v.tight_layout()
        fig_v.show()

        for ax, key, metric_label in zip(axs_v_flat.T.flatten(), key_list, metric_label_list):
            pc = ax.scatter(sub_df['u_coord'], sub_df['v_coord'], c=sub_df[key], s=size, cmap='flare')
            fig_v_flat.colorbar(pc, ax=ax, label=metric_label)

            for x, y, z, u, v in zip(sub_df.index.get_level_values('lambda_heading_virtual'),
                                     sub_df.index.get_level_values('lambda_heading_neighbors'),
                                     sub_df.index.get_level_values('lambda_contraction'),
                                     sub_df['u_coord'],
                                     sub_df['v_coord']):
                label = f"({x:.1f}; {y:.1f}; {z:.1f})"
                ax.annotate(label, xy=(u, v), xytext=(1e-2, 1e-2),
                            textcoords='offset points', ha='center', va='bottom',
                            fontsize=6)

            ax.set_xticklabels([])
            ax.set_yticklabels([])

        fig_v_flat.suptitle('U-Turn Experiment: Parameter Evaluation')
        fig_v_flat.tight_layout()
        fig_v_flat.show()

        tri = Triangulation(sub_df['u_coord'], sub_df['v_coord'])
        for ax, key, metric_label in zip(axs_v_proj.T.flatten(), key_list, metric_label_list):
            pc = ax.tripcolor(tri, sub_df[key], cmap='flare', antialiased=True)
            fig_v_proj.colorbar(pc, ax=ax, label=metric_label)

            for x, y, z, u, v in zip(sub_df.index.get_level_values('lambda_heading_virtual'),
                                     sub_df.index.get_level_values('lambda_heading_neighbors'),
                                     sub_df.index.get_level_values('lambda_contraction'),
                                     sub_df['u_coord'],
                                     sub_df['v_coord']):
                label = f"({x:.1f}; {y:.1f}; {z:.1f})"
                ax.annotate(label, xy=(u, v), xytext=(1e-2, 1e-2),
                            textcoords='offset points', ha='center', va='bottom',
                            fontsize=6)
            ax.set_xticks([])
            ax.set_yticks([])

        fig_v_proj.suptitle('U-Turn Experiment: Parameter Evaluation Projected')
        fig_v_proj.tight_layout()
        fig_v_proj.show()

        for i in selected_metrics:
            fig_v_proj_selected, ax_v_proj_selected = plt.subplots(dpi=dpi)

            pc = ax_v_proj_selected.tripcolor(tri, sub_df[key_list[i]], cmap='flare', antialiased=True)
            fig_v_proj_selected.colorbar(pc, ax=ax_v_proj_selected, label=metric_label_list[i])

            ax_v_proj_selected.annotate(r'$\lambda_\text{v} = 0.1$', xy=(-0.12, 0.04), xytext=(1e-2, 1e-2),
                                        textcoords='offset points', ha='center', va='bottom',
                                        fontsize=10, rotation=60)

            ax_v_proj_selected.annotate(r'$\lambda_\text{v} = 0.4$', xy=(0.21, -0.14), xytext=(1e-2, 1e-2),
                                        textcoords='offset points', ha='left', va='center',
                                        fontsize=10, rotation=-30)

            ax_v_proj_selected.annotate(r'$\lambda_\text{nh} = 0.1$', xy=(0.12, 0.04), xytext=(1e-2, 1e-2),
                                        textcoords='offset points', ha='center', va='bottom',
                                        fontsize=10, rotation=-60)

            ax_v_proj_selected.annotate(r'$\lambda_\text{nh} = 0.4$', xy=(-0.21, -0.14), xytext=(1e-2, 1e-2),
                                        textcoords='offset points', ha='right', va='center',
                                        fontsize=10, rotation=30)

            ax_v_proj_selected.annotate(r'$\lambda_\text{c} = 0.1$', xy=(0.0, -0.13), xytext=(1e-2, 1e-2),
                                        textcoords='offset points', ha='center', va='top',
                                        fontsize=10)

            ax_v_proj_selected.annotate(r'$\lambda_\text{c} = 0.4$', xy=(0.0, 0.25), xytext=(1e-2, 1e-2),
                                        textcoords='offset points', ha='center', va='bottom',
                                        fontsize=10)

            ax_v_proj_selected.set_xlim([-0.3, 0.3])
            ax_v_proj_selected.set_ylim([-0.19, 0.3])
            ax_v_proj_selected.set_aspect('equal', 'box')
            
            ax_v_proj_selected.set_xticks([])
            ax_v_proj_selected.set_yticks([])

            fig_v_proj_selected.suptitle('U-Turn Experiment: Parameter Evaluation Projected')
            fig_v_proj_selected.tight_layout()
            fig_v_proj_selected.show()

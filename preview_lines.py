"""
preview_lines.py
Carga un archivo JSON de líneas y lo visualiza con matplotlib.
Uso: python preview_lines.py image_lines.json
"""

import json
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def load_lines(filepath):
    with open(filepath, "r") as f:
        return json.load(f)

def plot_lines(lines, filepath):
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.patch.set_facecolor("#111111")
    ax.set_facecolor("#111111")

    total_points = 0

    for i, line in enumerate(lines):
        if len(line) < 2:
            continue
        xs = [p[0] for p in line]
        ys = [p[1] for p in line]
        total_points += len(line)

        # Color por línea para distinguir trazos
        color = plt.cm.plasma(i / max(len(lines), 1))
        ax.plot(xs, ys, color=color, linewidth=0.8, alpha=0.9)

        # Marca el punto de inicio de cada trazo
        ax.plot(xs[0], ys[0], "o", color="cyan", markersize=2, alpha=0.5)

    # Invertir Y para que coincida con coordenadas de imagen (origen arriba-izquierda)
    ax.invert_yaxis()
    ax.set_aspect("equal")

    ax.tick_params(colors="#888888")
    ax.spines["bottom"].set_color("#444444")
    ax.spines["top"].set_color("#444444")
    ax.spines["left"].set_color("#444444")
    ax.spines["right"].set_color("#444444")

    legend = [
        mpatches.Patch(color="cyan", label=f"Inicio de trazo"),
    ]
    ax.legend(handles=legend, facecolor="#222222", labelcolor="white", fontsize=8)

    ax.set_title(
        f"{filepath}  •  {len(lines)} líneas  •  {total_points} puntos",
        color="white",
        fontsize=11,
        pad=12,
    )
    ax.set_xlabel("X", color="#888888")
    ax.set_ylabel("Y", color="#888888")

    plt.tight_layout()
    plt.show()

    print(f"\n✓ {len(lines)} líneas cargadas")
    print(f"✓ {total_points} puntos en total")

    # Stats básicos
    all_x = [p[0] for line in lines for p in line]
    all_y = [p[1] for line in lines for p in line]
    print(f"✓ Rango X: {min(all_x):.2f} → {max(all_x):.2f}")
    print(f"✓ Rango Y: {min(all_y):.2f} → {max(all_y):.2f}")


if __name__ == "__main__":
    filepath = sys.argv[1] if len(sys.argv) > 1 else "image_lines.json"
    lines = load_lines(filepath)
    plot_lines(lines, filepath)
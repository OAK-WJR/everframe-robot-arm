#!/usr/bin/env python3
"""
Parse nt4_watch.txt files and generate interactive HTML charts using Plotly.
Usage: python3 parse_nt4_to_html.py <file1.txt> <label1> [<file2.txt> <label2> ...] -o <output.html>
"""
import argparse
import json
import re
import sys
from datetime import datetime
from pathlib import Path


def parse_nt4_watch(filepath):
    """
    Parse nt4_watch.txt and return time-series data.
    Each block ends with "----" separator.
    
    IMPORTANT: Deduplicate by RosSeq - keep only the last sample for each RosSeq.
    This prevents drawing multiple lines when nt4_dump samples faster than RosSeq updates.
    """
    keys = ["RosSeq", "RosPosRad", "RosVelRadPerS", "RosAccRadPerS2", "AngleDeg", "OutVoltsCmd"]
    
    # Use dict keyed by RosSeq to deduplicate
    samples_by_seq = {}  # RosSeq -> {key: value, ...}
    samples_no_seq = []  # Samples without RosSeq (shouldn't happen, but just in case)
    
    current = {}
    
    def emit_sample():
        if "RosPosRad" not in current and "AngleDeg" not in current:
            return
        seq = current.get("RosSeq")
        if seq is not None:
            # Overwrite previous sample with same RosSeq (keep last)
            samples_by_seq[seq] = dict(current)
        else:
            samples_no_seq.append(dict(current))
    
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            
            # Block separator
            if line == "----":
                emit_sample()
                current = {}
                continue
            
            # Match Arm/J2/Key=Value or Arm/Key=Value
            m = re.match(r"Arm/(?:J\d+/)?(\w+)=(.*)", line)
            if not m:
                continue
            key, val = m.group(1), m.group(2)
            
            # Parse value
            if val == "nan" or val == "":
                continue
            try:
                v = float(val)
            except ValueError:
                continue
            
            if key in keys:
                current[key] = v
    
    # Emit final block if any
    emit_sample()
    
    # Convert to sorted list by RosSeq
    sorted_seqs = sorted(samples_by_seq.keys())
    data = {k: [] for k in keys}
    for seq in sorted_seqs:
        sample = samples_by_seq[seq]
        for k in keys:
            data[k].append(sample.get(k))
    
    # Append samples without RosSeq at the end
    for sample in samples_no_seq:
        for k in keys:
            data[k].append(sample.get(k))
    
    return data


def generate_html(datasets, labels, output_path):
    """
    Generate an interactive HTML page with Plotly.js charts.
    datasets: list of data dicts
    labels: list of labels (e.g. "0.2 rad/s", "0.5 rad/s")
    """
    html_template = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <!-- Prevent browser caching so 'latest.html' always updates on refresh -->
    <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
    <meta http-equiv="Pragma" content="no-cache">
    <meta http-equiv="Expires" content="0">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>NT4 Data Analysis</title>
    <script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: #0d1117;
            color: #c9d1d9;
            padding: 20px;
        }
        h1 {
            text-align: center;
            margin-bottom: 20px;
            color: #58a6ff;
        }
        .controls {
            display: flex;
            justify-content: center;
            gap: 20px;
            margin-bottom: 20px;
            flex-wrap: wrap;
        }
        .controls label {
            display: flex;
            align-items: center;
            gap: 8px;
            cursor: pointer;
        }
        .controls input[type="checkbox"] {
            width: 18px;
            height: 18px;
            accent-color: #58a6ff;
        }
        .chart-container {
            background: #161b22;
            border-radius: 8px;
            padding: 15px;
            margin-bottom: 20px;
            border: 1px solid #30363d;
        }
        .chart-title {
            font-size: 16px;
            font-weight: 600;
            margin-bottom: 10px;
            color: #f0f6fc;
        }
        .chart {
            width: 100%;
            height: 300px;
        }
        .info {
            text-align: center;
            font-size: 14px;
            color: #8b949e;
            margin-bottom: 20px;
        }
    </style>
</head>
<body>
    <h1>NT4 J2 Data Analysis</h1>
    <p class="info">Use mouse wheel to zoom, drag to pan. Double-click to reset view.</p>
    <p class="info">Generated at: GENERATED_AT</p>
    
    <div class="controls">
        DATASET_CHECKBOXES
    </div>
    
    <div class="chart-container">
        <div class="chart-title">Position: RosPosRad (Setpoint) vs AngleDeg (Feedback)</div>
        <div id="chart-pos" class="chart"></div>
    </div>
    
    <div class="chart-container">
        <div class="chart-title">Velocity: RosVelRadPerS (Setpoint)</div>
        <div id="chart-vel" class="chart"></div>
    </div>
    
    <div class="chart-container">
        <div class="chart-title">Acceleration: RosAccRadPerS2 (Setpoint)</div>
        <div id="chart-acc" class="chart"></div>
    </div>
    
    <div class="chart-container">
        <div class="chart-title">Output Voltage: OutVoltsCmd</div>
        <div id="chart-volts" class="chart"></div>
    </div>
    
    <div class="chart-container">
        <div class="chart-title">Position Error: RosPosRad - AngleDeg*π/180</div>
        <div id="chart-err" class="chart"></div>
    </div>

    <script>
        const datasets = DATASETS_JSON;
        const labels = LABELS_JSON;
        const colors = ['#58a6ff', '#f78166', '#7ee787', '#d2a8ff', '#ff7b72', '#ffa657'];
        
        const layout = {
            paper_bgcolor: '#161b22',
            plot_bgcolor: '#0d1117',
            font: { color: '#c9d1d9' },
            margin: { l: 60, r: 30, t: 30, b: 40 },
            xaxis: {
                title: 'RosSeq (setpoint sequence)',
                gridcolor: '#30363d',
                zerolinecolor: '#30363d'
            },
            yaxis: {
                gridcolor: '#30363d',
                zerolinecolor: '#30363d'
            },
            legend: {
                orientation: 'h',
                y: 1.1
            },
            hovermode: 'x unified'
        };
        
        const config = {
            responsive: true,
            scrollZoom: true,
            displayModeBar: true,
            modeBarButtonsToRemove: ['lasso2d', 'select2d']
        };
        
        function updateCharts() {
            const visibleDatasets = [];
            labels.forEach((label, i) => {
                const cb = document.getElementById('cb-' + i);
                if (cb && cb.checked) {
                    visibleDatasets.push(i);
                }
            });

            function getX(d) {
                // Prefer RosSeq for x-axis to avoid plotting artifacts when watch_hz != publish rate.
                // Fallback to sample index if RosSeq missing.
                const hasSeq = d.RosSeq && d.RosSeq.some(v => v !== null && v !== undefined);
                if (!hasSeq) return d.RosPosRad.map((_, idx) => idx);
                return d.RosSeq.map((v, idx) => (v !== null && v !== undefined) ? v : idx);
            }

            function breakOnNonMonotonic(x, y) {
                // Prevent Plotly from drawing a diagonal line when RosSeq resets/jumps backwards.
                // Insert nulls to break the polyline.
                const xo = [];
                const yo = [];
                let lastX = null;
                for (let i = 0; i < x.length; i++) {
                    const xi = x[i];
                    const yi = y[i];
                    if (xi === null || xi === undefined || yi === null || yi === undefined) {
                        xo.push(null); yo.push(null);
                        lastX = null;
                        continue;
                    }
                    if (lastX !== null && xi < lastX) {
                        xo.push(null); yo.push(null);
                    }
                    xo.push(xi); yo.push(yi);
                    lastX = xi;
                }
                return { x: xo, y: yo };
            }
            
            // Position chart
            const posTraces = [];
            visibleDatasets.forEach(i => {
                const d = datasets[i];
                const x = getX(d);
                const p = breakOnNonMonotonic(x, d.RosPosRad);
                posTraces.push({
                    x: p.x,
                    y: p.y,
                    name: labels[i] + ' RosPosRad',
                    line: { color: colors[i % colors.length], width: 1.5 },
                    type: 'scatter',
                    mode: 'lines',
                    connectgaps: false
                });
                // AngleDeg in radians
                const angleRad = d.AngleDeg.map(v => v !== null ? v * Math.PI / 180 : null);
                const a = breakOnNonMonotonic(x, angleRad);
                posTraces.push({
                    x: a.x,
                    y: a.y,
                    name: labels[i] + ' AngleDeg (rad)',
                    line: { color: colors[i % colors.length], width: 1, dash: 'dot' },
                    type: 'scatter',
                    mode: 'lines',
                    connectgaps: false
                });
            });
            Plotly.react('chart-pos', posTraces, {...layout, yaxis: {...layout.yaxis, title: 'Position (rad)'}}, config);
            
            // Velocity chart
            const velTraces = [];
            visibleDatasets.forEach(i => {
                const d = datasets[i];
                const x = getX(d);
                const v = breakOnNonMonotonic(x, d.RosVelRadPerS);
                velTraces.push({
                    x: v.x,
                    y: v.y,
                    name: labels[i],
                    line: { color: colors[i % colors.length], width: 1.5 },
                    type: 'scatter',
                    mode: 'lines',
                    connectgaps: false
                });
            });
            Plotly.react('chart-vel', velTraces, {...layout, yaxis: {...layout.yaxis, title: 'Velocity (rad/s)'}}, config);
            
            // Acceleration chart
            const accTraces = [];
            visibleDatasets.forEach(i => {
                const d = datasets[i];
                const x = getX(d);
                const a = breakOnNonMonotonic(x, d.RosAccRadPerS2);
                accTraces.push({
                    x: a.x,
                    y: a.y,
                    name: labels[i],
                    line: { color: colors[i % colors.length], width: 1.5 },
                    type: 'scatter',
                    mode: 'lines',
                    connectgaps: false
                });
            });
            Plotly.react('chart-acc', accTraces, {...layout, yaxis: {...layout.yaxis, title: 'Acceleration (rad/s²)'}}, config);
            
            // Voltage chart
            const voltTraces = [];
            visibleDatasets.forEach(i => {
                const d = datasets[i];
                const x = getX(d);
                const vv = breakOnNonMonotonic(x, d.OutVoltsCmd);
                voltTraces.push({
                    x: vv.x,
                    y: vv.y,
                    name: labels[i],
                    line: { color: colors[i % colors.length], width: 1.5 },
                    type: 'scatter',
                    mode: 'lines',
                    connectgaps: false
                });
            });
            Plotly.react('chart-volts', voltTraces, {...layout, yaxis: {...layout.yaxis, title: 'Voltage (V)'}}, config);
            
            // Error chart
            const errTraces = [];
            visibleDatasets.forEach(i => {
                const d = datasets[i];
                const x = getX(d);
                const err = d.RosPosRad.map((pos, idx) => {
                    const angle = d.AngleDeg[idx];
                    if (pos === null || angle === null) return null;
                    return pos - angle * Math.PI / 180;
                });
                const e = breakOnNonMonotonic(x, err);
                errTraces.push({
                    x: e.x,
                    y: e.y,
                    name: labels[i],
                    line: { color: colors[i % colors.length], width: 1.5 },
                    type: 'scatter',
                    mode: 'lines',
                    connectgaps: false
                });
            });
            Plotly.react('chart-err', errTraces, {...layout, yaxis: {...layout.yaxis, title: 'Error (rad)'}}, config);
        }
        
        // Initial render
        updateCharts();
    </script>
</body>
</html>
"""
    
    # Generate checkboxes
    checkboxes = []
    for i, label in enumerate(labels):
        checked = "checked"
        checkboxes.append(f'<label><input type="checkbox" id="cb-{i}" {checked} onchange="updateCharts()"> {label}</label>')
    
    html = html_template.replace("DATASETS_JSON", json.dumps(datasets))
    html = html.replace("LABELS_JSON", json.dumps(labels))
    html = html.replace("DATASET_CHECKBOXES", "\n        ".join(checkboxes))
    html = html.replace("GENERATED_AT", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    
    with open(output_path, "w") as f:
        f.write(html)
    
    print(f"Generated: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Parse NT4 watch files and generate interactive charts")
    parser.add_argument("files", nargs="+", help="Pairs of <file> <label>")
    parser.add_argument("-o", "--output", default="/home/jetson/catkin_ws/arm_data/nt4_analysis.html", help="Output HTML file")
    args = parser.parse_args()
    
    if len(args.files) % 2 != 0:
        print("Error: files must be pairs of <file> <label>")
        sys.exit(1)
    
    datasets = []
    labels = []
    
    for i in range(0, len(args.files), 2):
        filepath = args.files[i]
        label = args.files[i + 1]
        
        print(f"Parsing {filepath} as '{label}'...")
        data = parse_nt4_watch(filepath)
        print(f"  Found {len(data['RosSeq'])} samples")
        
        datasets.append(data)
        labels.append(label)
    
    generate_html(datasets, labels, args.output)


if __name__ == "__main__":
    main()


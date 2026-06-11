#!/usr/bin/env python3
"""
VRP-RPD HTML Gantt Chart Visualization

Contains functions for generating HTML Gantt charts with SVG visualizations.
Includes diagonal cross-agent lines for inter-agent transfers.
"""

import math
from datetime import datetime
from typing import Dict

from .models import VRPRPDInstance
from .decoder import decode_chromosome
from .utils import simulate_solution


def generate_html_gantt(
    result: Dict,
    instance: VRPRPDInstance,
    output_path: str,
    title: str = "BRKGA-GP Solution",
    allow_mixed: bool = True
):
    """Generate HTML Gantt chart from solver result."""
    if result.get('best_chromosome') is None:
        print("No chromosome to visualize")
        return

    chrom = result['best_chromosome']
    makespan = result['makespan']
    solve_time = result.get('solve_time', 0)

    tours = decode_chromosome(chrom, instance, allow_mixed=allow_mixed)
    job_times, agent_tours, agent_completion_times, customer_assignment = simulate_solution(
        tours, instance
    )

    routes_for_html = []
    for a in range(instance.m):
        stops = [{'time': 0.0, 'op': 'D', 'node': instance.depot}]
        tour = agent_tours.get(a, [])

        current_time = 0.0
        current_loc = instance.depot

        for cust_loc, op in tour:
            travel = instance.dist[current_loc][cust_loc]
            arrival = current_time + travel

            if op == 'D':
                stops.append({'time': arrival, 'op': 'D', 'node': cust_loc})
                current_time = arrival + instance.proc[cust_loc]
            else:
                jt = job_times.get(cust_loc, {})
                job_end = jt.get('end', arrival)
                pickup_time = max(arrival, job_end)
                stops.append({'time': pickup_time, 'op': 'P', 'node': cust_loc})
                current_time = pickup_time

            current_loc = cust_loc

        if tour:
            completion = current_time + instance.dist[current_loc][instance.depot]
            stops.append({'time': completion, 'op': 'P', 'node': instance.depot})

        routes_for_html.append({
            'agent': a,
            'stops': stops,
            'customers': [loc for loc, op in tour if op == 'D'],
            'completion': agent_completion_times.get(a, 0)
        })

    html = _create_html_report(
        routes_for_html,
        makespan,
        job_times,
        customer_assignment,
        instance,
        solve_time,
        title
    )

    with open(output_path, 'w') as f:
        f.write(html)
    print(f"HTML Gantt chart saved to: {output_path}")


def _create_html_report(routes, makespan, job_times, customer_assignment, instance, solve_time, title):
    """Create full HTML report with DIAGONAL CROSS-AGENT LINES in SVG Gantt chart"""

    # =========================================================================
    # PATCHED svg_gantt with DIAGONAL lines for cross-agent transfers
    # =========================================================================
    def svg_gantt(routes, width=1000, height=None):
        if not routes:
            return '<text x="50%" y="50%" text-anchor="middle">No route data</text>'

        n_agents = len(routes)
        ms = max((stop['time'] for r in routes for stop in r['stops']), default=1)
        if ms == 0:
            ms = 1

        margin_left = 70
        margin_right = 30
        margin_top = 60
        margin_bottom = 50
        row_height = 90
        radius = 14

        if height is None:
            height = margin_top + margin_bottom + n_agents * row_height

        plot_w = width - margin_left - margin_right

        def x(t):
            return margin_left + t / ms * plot_w

        def y(a):
            return margin_top + a * row_height + row_height / 2

        # Collect dropoff and pickup events per customer
        customer_events = {}
        for agent_data in routes:
            agent = agent_data['agent']
            for stop in agent_data['stops']:
                time_val, op, node = stop['time'], stop['op'], stop['node']
                if node == instance.depot:
                    continue
                if node not in customer_events:
                    customer_events[node] = {}
                customer_events[node][op] = {
                    'agent': agent,
                    'time': time_val,
                    'x': x(time_val),
                    'y': y(agent)
                }

        background_elements = []
        same_agent_lines = []      # Orthogonal bumped lines for same-agent
        cross_agent_lines = []     # DIAGONAL lines for cross-agent
        circle_elements = []
        same_agent_bump_counter = {}

        for node, events in sorted(customer_events.items()):
            if 'P' not in events or 'D' not in events:
                continue

            pick = events['P']
            drop = events['D']
            px, py = pick['x'], pick['y']
            dx, dy = drop['x'], drop['y']
            pick_agent = pick['agent']
            drop_agent = drop['agent']

            if pick_agent != drop_agent:
                # ========== CROSS-AGENT: Draw DIAGONAL line with arrow ==========
                length = math.sqrt((px - dx)**2 + (py - dy)**2)
                if length > 0:
                    # Unit vector from drop to pick
                    ux = (px - dx) / length
                    uy = (py - dy) / length

                    # Start point: edge of dropoff circle
                    start_x = dx + ux * radius
                    start_y = dy + uy * radius

                    # End point: edge of pickup circle
                    end_x = px - ux * radius
                    end_y = py - uy * radius

                    # Draw diagonal line with arrow
                    cross_agent_lines.append(
                        f'<line x1="{start_x:.1f}" y1="{start_y:.1f}" '
                        f'x2="{end_x:.1f}" y2="{end_y:.1f}" '
                        f'stroke="#8b5cf6" stroke-width="2.5" stroke-opacity="0.85" '
                        f'marker-end="url(#arrowhead)"/>'
                    )

                    # Add "X" marker at midpoint showing cross-agent transfer
                    mid_x = (start_x + end_x) / 2
                    mid_y = (start_y + end_y) / 2
                    cross_agent_lines.append(
                        f'<circle cx="{mid_x:.1f}" cy="{mid_y:.1f}" r="8" '
                        f'fill="#8b5cf6" stroke="white" stroke-width="1.5"/>'
                    )
                    cross_agent_lines.append(
                        f'<text x="{mid_x:.1f}" y="{mid_y + 3:.1f}" '
                        f'font-size="8" font-weight="bold" fill="white" '
                        f'text-anchor="middle">X</text>'
                    )
            else:
                # ========== SAME-AGENT: Draw orthogonal bumped line ==========
                agent_key = pick_agent
                bump_index = same_agent_bump_counter.get(agent_key, 0)
                same_agent_bump_counter[agent_key] = bump_index + 1
                bump_offsets = [22, 30, 38, 26, 34, 42]
                bump_y = py - bump_offsets[bump_index % len(bump_offsets)]

                path = (f'M {px:.1f} {py - radius:.1f} '
                        f'L {px:.1f} {bump_y:.1f} '
                        f'L {dx:.1f} {bump_y:.1f} '
                        f'L {dx:.1f} {dy - radius:.1f}')
                same_agent_lines.append(
                    f'<path d="{path}" fill="none" stroke="#9ca3af" '
                    f'stroke-width="1.5" stroke-opacity="0.5"/>'
                )

        # Draw agent backgrounds and circles
        for agent_data in routes:
            agent = agent_data['agent']
            agent_y = y(agent)
            background_elements.append(
                f'<rect x="{margin_left}" y="{agent_y - 22:.1f}" '
                f'width="{plot_w}" height="44" fill="#f3f4f6" rx="4"/>'
            )
            background_elements.append(
                f'<text x="{margin_left-10}" y="{agent_y+5:.1f}" '
                f'text-anchor="end" font-size="13" font-weight="600" '
                f'fill="#374151">Agent {agent}</text>'
            )

            for stop in agent_data['stops']:
                time_val, op, node = stop['time'], stop['op'], stop['node']
                cx, cy = x(time_val), agent_y

                if node == instance.depot:
                    circle_elements.append(
                        f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="6" '
                        f'fill="#9ca3af" stroke="#6b7280" stroke-width="1.5"/>'
                    )
                elif op == 'D':
                    circle_elements.append(
                        f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{radius}" '
                        f'fill="#3b82f6" stroke="#1e40af" stroke-width="2"/>'
                    )
                    circle_elements.append(
                        f'<text x="{cx:.1f}" y="{cy+4:.1f}" font-size="11" '
                        f'font-weight="bold" fill="white" text-anchor="middle">{node}</text>'
                    )
                elif op == 'P':
                    circle_elements.append(
                        f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{radius}" '
                        f'fill="#22c55e" stroke="#15803d" stroke-width="2"/>'
                    )
                    circle_elements.append(
                        f'<text x="{cx:.1f}" y="{cy+4:.1f}" font-size="11" '
                        f'font-weight="bold" fill="white" text-anchor="middle">{node}</text>'
                    )

        # Build SVG with arrowhead marker for cross-agent lines
        svg = f'''<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7"
            refX="9" refY="3.5" orient="auto" markerUnits="strokeWidth">
      <polygon points="0 0, 10 3.5, 0 7" fill="#8b5cf6"/>
    </marker>
  </defs>
  <rect width="100%" height="100%" fill="white"/>
  <text x="{width/2}" y="28" text-anchor="middle" font-size="18" font-weight="bold" fill="#1e293b">Agent Timeline</text>

  <!-- Legend with Cross-Agent indicator -->
  <g transform="translate({width-380}, 8)">
    <circle cx="10" cy="14" r="9" fill="#22c55e" stroke="#15803d" stroke-width="1.5"/>
    <text x="26" y="18" font-size="12" fill="#374151">Pick</text>
    <circle cx="70" cy="14" r="9" fill="#3b82f6" stroke="#1e40af" stroke-width="1.5"/>
    <text x="86" y="18" font-size="12" fill="#374151">Drop</text>
    <circle cx="130" cy="14" r="5" fill="#9ca3af" stroke="#6b7280" stroke-width="1"/>
    <text x="142" y="18" font-size="12" fill="#374151">Depot</text>
    <line x1="185" y1="14" x2="220" y2="14" stroke="#8b5cf6" stroke-width="2.5"/>
    <circle cx="202" cy="14" r="6" fill="#8b5cf6" stroke="white" stroke-width="1"/>
    <text x="202" y="17" font-size="7" font-weight="bold" fill="white" text-anchor="middle">X</text>
    <text x="228" y="18" font-size="12" fill="#374151">Cross-Agent</text>
  </g>

  {''.join(background_elements)}
  {''.join(same_agent_lines)}
  {''.join(cross_agent_lines)}
  {''.join(circle_elements)}

  <line x1="{margin_left}" y1="{height-margin_bottom}" x2="{width-margin_right}" y2="{height-margin_bottom}" stroke="#374151" stroke-width="1"/>
  <text x="{width/2}" y="{height-15}" text-anchor="middle" font-size="12" fill="#6b7280">Time</text>
  <text x="{margin_left}" y="{height-margin_bottom+15}" font-size="10" fill="#6b7280">0</text>
  <text x="{width-margin_right}" y="{height-margin_bottom+15}" text-anchor="end" font-size="10" fill="#6b7280">{ms:.0f}</text>
</svg>'''
        return svg

    # SVG Job chart (unchanged)
    def svg_job_chart(job_times, customer_assignment, makespan, width=1000, height=None):
        if not job_times:
            return '<text x="50%" y="50%" text-anchor="middle">No job data</text>'

        margin_left = 60
        margin_right = 30
        margin_top = 40
        margin_bottom = 40
        row_height = 28

        sorted_customers = sorted(job_times.keys())
        n_customers = len(sorted_customers)

        if height is None:
            height = margin_top + margin_bottom + n_customers * row_height

        plot_w = width - margin_left - margin_right

        def x(t):
            return margin_left + (t / makespan) * plot_w if makespan > 0 else margin_left

        def y(i):
            return margin_top + i * row_height + row_height / 2

        colors = ['#3b82f6', '#22c55e', '#f97316', '#ef4444', '#8b5cf6', '#06b6d4', '#ec4899', '#84cc16']

        elements = []

        for i, c in enumerate(sorted_customers):
            cy = y(i)
            elements.append(f'<rect x="{margin_left}" y="{cy - 12:.1f}" width="{plot_w}" height="24" fill="#f8fafc" rx="2"/>')
            elements.append(f'<text x="{margin_left-10}" y="{cy+4:.1f}" text-anchor="end" font-size="11" fill="#374151">{c}</text>')

        for i, c in enumerate(sorted_customers):
            jt = job_times[c]
            cy = y(i)
            agent = customer_assignment.get(c, 0)
            color = colors[agent % len(colors)]

            start_x = x(jt['start'])
            end_x = x(jt['end'])
            bar_width = max(end_x - start_x, 2)
            elements.append(f'<rect x="{start_x:.1f}" y="{cy-10:.1f}" width="{bar_width:.1f}" height="20" fill="{color}" opacity="0.7" rx="3"/>')

            pickup_time = jt.get('pickup', jt['end'])
            if pickup_time > jt['end']:
                wait_x = x(jt['end'])
                pickup_x = x(pickup_time)
                wait_width = pickup_x - wait_x
                elements.append(f'<rect x="{wait_x:.1f}" y="{cy-10:.1f}" width="{wait_width:.1f}" height="20" fill="#e5e7eb" opacity="0.6" rx="3"/>')

            dropoff_x = x(jt['dropoff'])
            pickup_x = x(pickup_time)
            elements.append(f'<polygon points="{dropoff_x:.1f},{cy-15:.1f} {dropoff_x+8:.1f},{cy:.1f} {dropoff_x:.1f},{cy+15:.1f}" fill="#1e40af"/>')
            elements.append(f'<polygon points="{pickup_x:.1f},{cy-15:.1f} {pickup_x-8:.1f},{cy:.1f} {pickup_x:.1f},{cy+15:.1f}" fill="#15803d"/>')

        makespan_x = x(makespan)
        elements.append(f'<line x1="{makespan_x:.1f}" y1="{margin_top-10}" x2="{makespan_x:.1f}" y2="{height-margin_bottom}" stroke="#ef4444" stroke-width="2" stroke-dasharray="5,3"/>')
        elements.append(f'<text x="{makespan_x+5:.1f}" y="{margin_top}" font-size="10" fill="#ef4444">Makespan: {makespan:.1f}</text>')

        svg = f'''<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">
  <rect width="100%" height="100%" fill="white"/>
  <text x="{width/2}" y="25" text-anchor="middle" font-size="16" font-weight="bold" fill="#1e293b">Job Processing Timeline</text>
  <g>{''.join(elements)}</g>
  <line x1="{margin_left}" y1="{height-margin_bottom}" x2="{width-margin_right}" y2="{height-margin_bottom}" stroke="#374151" stroke-width="1"/>
  <text x="{width/2}" y="{height-10}" text-anchor="middle" font-size="12" fill="#6b7280">Time</text>
  <text x="{margin_left}" y="{height-margin_bottom+15}" font-size="10" fill="#6b7280">0</text>
  <text x="{width-margin_right}" y="{height-margin_bottom+15}" text-anchor="end" font-size="10" fill="#6b7280">{makespan:.0f}</text>
  <text x="15" y="{height/2}" text-anchor="middle" font-size="12" fill="#374151" transform="rotate(-90,15,{height/2})">Customer</text>
</svg>'''
        return svg

    # Agent summary rows
    agent_rows = []
    for a in range(instance.m):
        r = routes[a] if a < len(routes) else {'customers': [], 'completion': 0}
        customers = r.get('customers', [])
        completion = r.get('completion', 0)
        agent_rows.append(f'''
        <tr>
            <td>{a}</td>
            <td>{len(customers)}</td>
            <td>{', '.join(map(str, customers)) if customers else 'None'}</td>
            <td>{completion:.2f}</td>
        </tr>''')

    n_customers = len(job_times)

    html = f'''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{title} - VRP-RPD</title>
    <style>
        * {{ box-sizing: border-box; }}
        body {{
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background: linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%);
            min-height: 100vh;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
        }}
        h1 {{
            color: #1e40af;
            margin-bottom: 5px;
            font-size: 28px;
        }}
        h2 {{
            color: #374151;
            border-bottom: 2px solid #3b82f6;
            padding-bottom: 10px;
            margin-top: 30px;
        }}
        .meta {{
            color: #6b7280;
            margin-bottom: 20px;
            font-size: 14px;
        }}
        .card {{
            background: white;
            border-radius: 12px;
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06);
            padding: 24px;
            margin-bottom: 24px;
        }}
        .stats-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 16px;
            margin-bottom: 20px;
        }}
        .stat-box {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
            border-radius: 10px;
            text-align: center;
        }}
        .stat-box.green {{
            background: linear-gradient(135deg, #11998e 0%, #38ef7d 100%);
        }}
        .stat-box.orange {{
            background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
        }}
        .stat-box.blue {{
            background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);
        }}
        .stat-value {{
            font-size: 32px;
            font-weight: bold;
            margin-bottom: 5px;
        }}
        .stat-label {{
            font-size: 12px;
            text-transform: uppercase;
            opacity: 0.9;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin-top: 15px;
        }}
        th, td {{
            padding: 12px 15px;
            text-align: left;
            border-bottom: 1px solid #e5e7eb;
        }}
        th {{
            background: #f8fafc;
            font-weight: 600;
            color: #374151;
        }}
        tr:hover {{
            background: #f9fafb;
        }}
        .chart-container {{
            overflow-x: auto;
            margin: 20px 0;
        }}
        .chart-container svg {{
            max-width: 100%;
            height: auto;
        }}
        .legend {{
            display: flex;
            gap: 20px;
            flex-wrap: wrap;
            padding: 15px;
            background: #f8fafc;
            border-radius: 8px;
            margin-top: 15px;
            font-size: 13px;
        }}
        .legend-item {{
            display: flex;
            align-items: center;
            gap: 8px;
        }}
        footer {{
            text-align: center;
            color: #6b7280;
            font-size: 12px;
            margin-top: 40px;
            padding-top: 20px;
            border-top: 1px solid #e5e7eb;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>{title}</h1>
        <p class="meta">BRKGA-GP Solver (Diagonal Cross-Agent Lines) | Generated: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}</p>

        <div class="card">
            <div class="stats-grid">
                <div class="stat-box">
                    <div class="stat-value">{makespan:.1f}</div>
                    <div class="stat-label">Makespan</div>
                </div>
                <div class="stat-box green">
                    <div class="stat-value">{n_customers}</div>
                    <div class="stat-label">Customers</div>
                </div>
                <div class="stat-box blue">
                    <div class="stat-value">{instance.m}</div>
                    <div class="stat-label">Agents</div>
                </div>
                <div class="stat-box orange">
                    <div class="stat-value">{solve_time:.1f}s</div>
                    <div class="stat-label">Solve Time</div>
                </div>
            </div>
        </div>

        <div class="card">
            <h2>Agent Timeline</h2>
            <div class="chart-container">
                {svg_gantt(routes)}
            </div>
        </div>

        <div class="card">
            <h2>Job Processing Timeline</h2>
            <div class="chart-container">
                {svg_job_chart(job_times, customer_assignment, makespan)}
            </div>
            <div class="legend">
                <div class="legend-item">
                    <svg width="30" height="16"><polygon points="0,3 8,8 0,13" fill="#1e40af"/></svg>
                    <span>Dropoff</span>
                </div>
                <div class="legend-item">
                    <svg width="30" height="16"><polygon points="30,3 22,8 30,13" fill="#15803d"/></svg>
                    <span>Pickup</span>
                </div>
                <div class="legend-item">
                    <svg width="30" height="16"><rect x="0" y="3" width="30" height="10" fill="#3b82f6" opacity="0.7" rx="2"/></svg>
                    <span>Processing</span>
                </div>
                <div class="legend-item">
                    <svg width="30" height="16"><rect x="0" y="3" width="30" height="10" fill="#e5e7eb" rx="2"/></svg>
                    <span>Wait</span>
                </div>
            </div>
        </div>

        <div class="card">
            <h2>Agent Summary</h2>
            <table>
                <thead>
                    <tr>
                        <th>Agent</th>
                        <th># Customers</th>
                        <th>Customers</th>
                        <th>Completion Time</th>
                    </tr>
                </thead>
                <tbody>
                    {''.join(agent_rows)}
                </tbody>
            </table>
        </div>

        <footer>
            VRP-RPD BRKGA-GP Solver | {instance.m} agents, k={instance.k} | Patched: Diagonal Cross-Agent Lines
        </footer>
    </div>
</body>
</html>'''
    return html

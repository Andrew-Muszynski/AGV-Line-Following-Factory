#!/usr/bin/env python3
"""
Extract makespan data across all experiments and datasets for comparison.
"""

import json
import os
import csv
from pathlib import Path
from collections import defaultdict
import statistics

def extract_makespan_from_file(filepath, experiment_type):
    """Extract makespan from a JSON file based on experiment type."""
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)

        if experiment_type == 'heuristics_only':
            # Heuristics have makespan in best_solution
            makespan = data.get('best_solution', {}).get('makespan')
            solve_time = data.get('all_heuristics', {})[0].get('execution_time_seconds')
            return makespan, solve_time
        else:
            # pure_brkga, warm_start, warm_start_no_gene, cold_start have makespan in solution
            makespan =  data.get('solution', {}).get('makespan')            
            solve_time =  data.get('solution', {}).get('solve_time_seconds')
            return makespan, solve_time

    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return None

def scan_experiment_directory(base_dir, experiment_type):
    """Scan an experiment directory and extract all makespan values."""
    results = []

    exp_path = Path(base_dir) / experiment_type
    if not exp_path.exists():
        print(f"Warning: {exp_path} does not exist")
        return results

    # For cold_start and warm_start, there's an extra directory level (e.g., blocks_only)
    if experiment_type in ['cold_start', 'warm_start']:
        # Get the intermediate directories (e.g., blocks_only)
        intermediate_dirs = [d for d in exp_path.iterdir() if d.is_dir()]
        if not intermediate_dirs:
            print(f"Warning: No subdirectories found in {exp_path}")
            return results

        # Process each intermediate directory
        for intermediate_dir in intermediate_dirs:
            dataset_parent = intermediate_dir
            intermediate_name = intermediate_dir.name
            print(f"  Processing {experiment_type}/{intermediate_name}...")

            # Now iterate through datasets
            for dataset_dir in sorted(dataset_parent.iterdir()):
                if not dataset_dir.is_dir():
                    continue

                dataset_name = dataset_dir.name

                # Iterate through variants
                for variant_dir in sorted(dataset_dir.iterdir()):
                    if not variant_dir.is_dir():
                        continue

                    variant_name = variant_dir.name

                    # Find all JSON files
                    for json_file in sorted(variant_dir.glob("*.json")):
                        makespan, s_time = extract_makespan_from_file(json_file, experiment_type)
                    
                        # if time is not None:
                        #     s_time = time
                        # else:
                        #     time = "n/a"

                        if makespan is not None:
                            # Parse file name to extract seed
                            filename = json_file.stem
                            parts = filename.split('_')

                            # Extract seed if present
                            seed = None
                            if 'seed' in parts:
                                seed_idx = parts.index('seed') + 1
                                if seed_idx < len(parts):
                                    seed = parts[seed_idx]

                            instance_num = variant_name  # Use variant as instance for these experiments

                            results.append({
                                'experiment': experiment_type,
                                'dataset': dataset_name,
                                'variant': variant_name,
                                'instance': instance_num,
                                'seed': seed,
                                'makespan': makespan,
                                'solve_time': s_time,
                                'filename': json_file.name,
                            })
    else:
        # Original logic for pure_brkga, warm_start_no_gene, heuristics_only
        # Iterate through datasets (bays29, berlin52, etc.)
        for dataset_dir in sorted(exp_path.iterdir()):
            if not dataset_dir.is_dir():
                continue

            dataset_name = dataset_dir.name

            # Iterate through variants (1R20, 1R10, 2x, 5x, base)
            for variant_dir in sorted(dataset_dir.iterdir()):
                if not variant_dir.is_dir():
                    continue

                variant_name = variant_dir.name

                # Find all JSON files
                for json_file in sorted(variant_dir.glob("*.json")):
                    makespan, s_time = extract_makespan_from_file(json_file, experiment_type)
                    
                    # if time is not None:
                    #     s_time = time
                    # else:
                    #     time = "n/a"

                    if makespan is not None:
                        # Parse file name to extract instance number and seed
                        filename = json_file.stem

                        # Extract instance number
                        if 'full_jobs_' in filename:
                            parts = filename.split('_')
                            instance_idx = parts.index('jobs') + 1
                            instance_num = parts[instance_idx]

                            # Extract seed if present
                            seed = None
                            if 'seed' in parts:
                                seed_idx = parts.index('seed') + 1
                                if seed_idx < len(parts):
                                    # Remove any trailing numbers (like _1000, _4000)
                                    seed = parts[seed_idx]
                        else:
                            # For files like dataset_heuristics.json
                            instance_num = variant_name
                            seed = None

                        results.append({
                            'experiment': experiment_type,
                            'dataset': dataset_name,
                            'variant': variant_name,
                            'instance': instance_num,
                            'seed': seed,
                            'makespan': makespan,
                            'solve_time': s_time,
                            'filename': json_file.name,
                        })

    return results

def main():
    results_dir = 'results'

    # List of experiment types
    experiments = [
        'pure_brkga',
        'warm_start_no_gene',
        'warm_start',
        'heuristics_only',
        'cold_start'
    ]

    all_results = []

    # Scan each experiment directory
    for experiment in experiments:
        print(f"Processing {experiment}...")
        exp_results = scan_experiment_directory(results_dir, experiment)
        all_results.extend(exp_results)
        print(f"  Found {len(exp_results)} results")

    if not all_results:
        print("No results found!")
        return

    # Save detailed results
    output_file = 'makespan_comparison_detailed.csv'
    with open(output_file, 'w', newline='') as f:
        fieldnames = ['experiment', 'dataset', 'variant', 'instance', 'seed', 'makespan', 'solve_time','filename']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_results)

    print(f"\nDetailed results saved to {output_file}")
    print(f"Total records: {len(all_results)}")

    # Filter out checkpoint files (rows without solve_time) for analysis
    filtered_results = [r for r in all_results if r.get('solve_time') is not None]
    print(f"Records with solve_time (excluding checkpoints): {len(filtered_results)}")
    print(f"Checkpoint records excluded: {len(all_results) - len(filtered_results)}")

    # Create summary statistics grouped by experiment, dataset, and variant
    print("\n" + "="*80)
    print("SUMMARY STATISTICS")
    print("="*80)

    # Group results (using filtered results only)
    grouped = defaultdict(list)
    for result in filtered_results:
        key = (result['experiment'], result['dataset'], result['variant'])
        grouped[key].append(result['makespan'])

    summary_data = []
    for (experiment, dataset, variant), makespans in sorted(grouped.items()):
        summary_data.append({
            'experiment': experiment,
            'dataset': dataset,
            'variant': variant,
            'count': len(makespans),
            'mean': round(statistics.mean(makespans), 2),
            'std': round(statistics.stdev(makespans), 2) if len(makespans) > 1 else 0,
            'min': round(min(makespans), 2),
            'max': round(max(makespans), 2)
        })

    summary_file = 'makespan_comparison_summary.csv'
    with open(summary_file, 'w', newline='') as f:
        fieldnames = ['experiment', 'dataset', 'variant', 'count', 'mean', 'std', 'min', 'max']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_data)

    print(f"Summary statistics saved to {summary_file}")

    # Print summary table
    print(f"\n{'Experiment':<20} {'Dataset':<10} {'Variant':<8} {'Count':>6} {'Mean':>10} {'Std':>10} {'Min':>10} {'Max':>10}")
    print("-" * 90)
    for row in summary_data:
        print(f"{row['experiment']:<20} {row['dataset']:<10} {row['variant']:<8} {row['count']:>6} {row['mean']:>10.2f} {row['std']:>10.2f} {row['min']:>10.2f} {row['max']:>10.2f}")

    # Create a pivot table for easier comparison across experiments
    print("\n" + "="*80)
    print("AVERAGE MAKESPAN BY EXPERIMENT AND DATASET-VARIANT")
    print("="*80)

    # Build pivot structure
    pivot_keys = sorted(set((d, v) for d, v in [(s['dataset'], s['variant']) for s in summary_data]))
    experiments_list = sorted(set(s['experiment'] for s in summary_data))

    pivot_data = {}
    for dataset, variant in pivot_keys:
        pivot_data[(dataset, variant)] = {}
        for exp in experiments_list:
            # Find matching summary
            for s in summary_data:
                if s['dataset'] == dataset and s['variant'] == variant and s['experiment'] == exp:
                    pivot_data[(dataset, variant)][exp] = s['mean']
                    break

    # Write pivot table
    pivot_file = 'makespan_comparison_pivot.csv'
    with open(pivot_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['dataset', 'variant'] + experiments_list)
        for (dataset, variant), exp_values in sorted(pivot_data.items()):
            row = [dataset, variant] + [exp_values.get(exp, '') for exp in experiments_list]
            writer.writerow(row)

    print(f"Pivot table saved to {pivot_file}")

    # Print pivot table
    header = f"{'Dataset':<10} {'Variant':<8}"
    for exp in experiments_list:
        header += f" {exp[:15]:>15}"
    print(header)
    print("-" * (18 + 16 * len(experiments_list)))
    for (dataset, variant), exp_values in sorted(pivot_data.items()):
        row = f"{dataset:<10} {variant:<8}"
        for exp in experiments_list:
            val = exp_values.get(exp, '')
            row += f" {val:>15}" if val == '' else f" {val:>15.2f}"
        print(row)

    # Create improvement analysis over heuristics_only baseline
    print("\n" + "="*80)
    print("PERCENTAGE IMPROVEMENT OVER HEURISTICS_ONLY BASELINE")
    print("="*80)

    improvement_data = []
    for (dataset, variant), exp_values in sorted(pivot_data.items()):
        if 'heuristics_only' in exp_values:
            baseline = exp_values['heuristics_only']

            row_data = {
                'dataset': dataset,
                'variant': variant,
                'heuristics_only': round(baseline, 2)
            }

            for exp in experiments_list:
                if exp != 'heuristics_only' and exp in exp_values:
                    exp_makespan = exp_values[exp]
                    # Percentage improvement: (baseline - current) / baseline * 100
                    # Positive means improvement (lower makespan)
                    improvement = ((baseline - exp_makespan) / baseline) * 100
                    row_data[f'{exp}_makespan'] = round(exp_makespan, 2)
                    row_data[f'{exp}_improvement_%'] = round(improvement, 2)

            improvement_data.append(row_data)

    # Save improvement analysis
    improvement_file = 'makespan_improvement_over_heuristics.csv'
    if improvement_data:
        with open(improvement_file, 'w', newline='') as f:
            fieldnames = ['dataset', 'variant', 'heuristics_only']
            for exp in experiments_list:
                if exp != 'heuristics_only':
                    fieldnames.extend([f'{exp}_makespan', f'{exp}_improvement_%'])
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(improvement_data)

        print(f"Improvement analysis saved to {improvement_file}")

        # Print improvement table
        print(f"\n{'Dataset':<10} {'Variant':<8} {'Heur_MS':>10}", end='')
        for exp in experiments_list:
            if exp != 'heuristics_only':
                print(f" {exp[:12]:>12} {'Improv%':>8}", end='')
        print()
        print("-" * (18 + 21 * (len(experiments_list) - 1)))

        for row in improvement_data:
            print(f"{row['dataset']:<10} {row['variant']:<8} {row['heuristics_only']:>10.2f}", end='')
            for exp in experiments_list:
                if exp != 'heuristics_only':
                    ms_key = f'{exp}_makespan'
                    imp_key = f'{exp}_improvement_%'
                    if ms_key in row and imp_key in row:
                        print(f" {row[ms_key]:>12.2f} {row[imp_key]:>8.2f}", end='')
                    else:
                        print(f" {'':>12} {'':>8}", end='')
            print()

    # Create comparison showing which experiment performs best for each dataset-variant
    print("\n" + "="*80)
    print("BEST EXPERIMENT FOR EACH DATASET-VARIANT (by mean makespan)")
    print("="*80)

    best_exp_data = []
    for (dataset, variant), exp_values in sorted(pivot_data.items()):
        if exp_values:
            best_exp = min(exp_values.items(), key=lambda x: x[1])
            worst_exp = max(exp_values.items(), key=lambda x: x[1])
            improvement = ((worst_exp[1] - best_exp[1]) / worst_exp[1]) * 100 if worst_exp[1] > 0 else 0

            # Calculate improvement over heuristics_only if it exists
            heur_improvement = None
            if 'heuristics_only' in exp_values and best_exp[0] != 'heuristics_only':
                heur_baseline = exp_values['heuristics_only']
                heur_improvement = ((heur_baseline - best_exp[1]) / heur_baseline) * 100

            best_exp_data.append({
                'dataset': dataset,
                'variant': variant,
                'best_experiment': best_exp[0],
                'best_makespan': round(best_exp[1], 2),
                'worst_experiment': worst_exp[0],
                'worst_makespan': round(worst_exp[1], 2),
                'improvement_over_worst_%': round(improvement, 2),
                'improvement_over_heuristics_%': round(heur_improvement, 2) if heur_improvement is not None else None
            })

    best_file = 'makespan_best_experiments.csv'
    with open(best_file, 'w', newline='') as f:
        fieldnames = ['dataset', 'variant', 'best_experiment', 'best_makespan', 'worst_experiment', 'worst_makespan', 'improvement_over_worst_%', 'improvement_over_heuristics_%']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(best_exp_data)

    print(f"Best experiments saved to {best_file}")

    # Print best experiments
    print(f"\n{'Dataset':<10} {'Variant':<8} {'Best Exp':<20} {'Best MS':>10} {'vs Worst':>10} {'vs Heur':>10}")
    print("-" * 80)
    for row in best_exp_data:
        heur_str = f"{row['improvement_over_heuristics_%']:>10.2f}" if row['improvement_over_heuristics_%'] is not None else f"{'N/A':>10}"
        print(f"{row['dataset']:<10} {row['variant']:<8} {row['best_experiment']:<20} {row['best_makespan']:>10.2f} {row['improvement_over_worst_%']:>10.2f} {heur_str}")

    # Overall winner count
    print("\n" + "="*80)
    print("OVERALL PERFORMANCE (How many times each experiment was best)")
    print("="*80)

    winner_counts = defaultdict(int)
    for row in best_exp_data:
        winner_counts[row['best_experiment']] += 1

    for exp in sorted(winner_counts.keys(), key=lambda x: winner_counts[x], reverse=True):
        print(f"{exp:<20} {winner_counts[exp]:>3} times")

    print("\n" + "="*80)
    print("FILES GENERATED:")
    print("  1. makespan_comparison_detailed.csv - All makespan values")
    print("  2. makespan_comparison_summary.csv - Summary statistics")
    print("  3. makespan_comparison_pivot.csv - Pivot table for easy comparison")
    print("  4. makespan_improvement_over_heuristics.csv - % improvement over heuristics baseline")
    print("  5. makespan_best_experiments.csv - Best experiment for each scenario")
    print("="*80)

if __name__ == '__main__':
    main()

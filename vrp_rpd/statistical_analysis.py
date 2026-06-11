#!/usr/bin/env python3
"""
Statistical analysis of VRP-RPD experimental results.
Performs statistical tests comparing different methods across datasets and run types.
"""

import pandas as pd
import numpy as np
from scipy import stats
from scipy.stats import friedmanchisquare, wilcoxon, rankdata
from itertools import combinations
import warnings
warnings.filterwarnings('ignore')

def load_data(csv_file='makespan_comparison_detailed.csv'):
    """Load the detailed makespan comparison data."""
    df = pd.read_csv(csv_file)
    return df

def prepare_data_for_testing(df, variant):
    """
    Prepare data for statistical testing for a specific variant.
    Returns a pivot table with methods as columns and dataset-instance as rows.
    """
    # Filter for the specific variant
    variant_df = df[df['variant'] == variant].copy()

    # For methods with multiple seeds, take the mean
    grouped = variant_df.groupby(['experiment', 'dataset', 'instance'])['makespan'].mean().reset_index()

    # Create a unique identifier for each dataset-instance combination
    grouped['dataset_instance'] = grouped['dataset'] + '_' + grouped['instance'].astype(str)

    # Pivot to get methods as columns
    pivot = grouped.pivot(index='dataset_instance', columns='experiment', values='makespan')

    return pivot

def friedman_test(data_pivot):
    """
    Perform Friedman test to check if there are significant differences among methods.

    Args:
        data_pivot: DataFrame with samples as rows and methods as columns

    Returns:
        statistic, p-value
    """
    # Remove rows with any NaN values
    clean_data = data_pivot.dropna()

    if len(clean_data) < 2:
        return None, None

    # Perform Friedman test
    # Each column is a method, each row is a matched observation (dataset-instance)
    statistic, p_value = friedmanchisquare(*[clean_data[col].values for col in clean_data.columns])

    return statistic, p_value

def wilcoxon_signed_rank_test(data1, data2):
    """
    Perform Wilcoxon signed-rank test for paired samples.

    Args:
        data1, data2: Paired samples to compare

    Returns:
        statistic, p-value, effect_size (rank-biserial correlation)
    """
    # Remove pairs where either value is NaN
    mask = ~(pd.isna(data1) | pd.isna(data2))
    clean_data1 = data1[mask]
    clean_data2 = data2[mask]

    if len(clean_data1) < 5:  # Need at least 5 pairs
        return None, None, None

    # Perform Wilcoxon signed-rank test
    try:
        statistic, p_value = wilcoxon(clean_data1, clean_data2, alternative='two-sided')

        # Calculate effect size (rank-biserial correlation)
        differences = clean_data1 - clean_data2
        n = len(differences)
        r_plus = sum(rankdata(abs(differences))[differences > 0])
        r_minus = sum(rankdata(abs(differences))[differences < 0])
        effect_size = (r_plus - r_minus) / (n * (n + 1) / 2)

        return statistic, p_value, effect_size
    except Exception as e:
        print(f"Error in Wilcoxon test: {e}")
        return None, None, None

def bonferroni_correction(p_values, alpha=0.05):
    """Apply Bonferroni correction for multiple comparisons."""
    n_tests = len(p_values)
    adjusted_alpha = alpha / n_tests
    return adjusted_alpha

def nemenyi_critical_distance(n_methods, n_datasets, alpha=0.05):
    """
    Calculate critical distance for Nemenyi post-hoc test.

    Args:
        n_methods: Number of methods being compared
        n_datasets: Number of datasets
        alpha: Significance level

    Returns:
        critical_distance
    """
    # Critical values for Nemenyi test (from tables)
    # Using q_alpha values for alpha=0.05
    q_alpha_values = {
        2: 1.960, 3: 2.344, 4: 2.569, 5: 2.728, 6: 2.850,
        7: 2.949, 8: 3.031, 9: 3.102, 10: 3.164
    }

    if n_methods not in q_alpha_values:
        # Use approximate formula for larger numbers
        q_alpha = 2.807  # Conservative estimate
    else:
        q_alpha = q_alpha_values[n_methods]

    cd = q_alpha * np.sqrt((n_methods * (n_methods + 1)) / (6 * n_datasets))
    return cd

def rank_methods(data_pivot):
    """
    Rank methods for each dataset (lower makespan = better = lower rank).

    Returns:
        average_ranks: Dictionary of method -> average rank
    """
    # Rank each row (1 = best/lowest makespan)
    ranks = data_pivot.rank(axis=1, method='average')

    # Calculate average rank for each method
    average_ranks = ranks.mean().to_dict()

    return average_ranks

def perform_nemenyi_test(data_pivot, alpha=0.05):
    """
    Perform Nemenyi post-hoc test after Friedman test.

    Returns:
        results: Dictionary with pairwise comparisons
    """
    clean_data = data_pivot.dropna()
    n_datasets = len(clean_data)
    n_methods = len(clean_data.columns)

    # Calculate average ranks
    average_ranks = rank_methods(clean_data)

    # Calculate critical distance
    cd = nemenyi_critical_distance(n_methods, n_datasets, alpha)

    # Pairwise comparisons
    results = []
    methods = list(clean_data.columns)

    for method1, method2 in combinations(methods, 2):
        rank_diff = abs(average_ranks[method1] - average_ranks[method2])
        significant = rank_diff > cd

        # Determine which is better
        if average_ranks[method1] < average_ranks[method2]:
            better = method1
        else:
            better = method2

        results.append({
            'method1': method1,
            'method2': method2,
            'rank1': average_ranks[method1],
            'rank2': average_ranks[method2],
            'rank_difference': rank_diff,
            'critical_distance': cd,
            'significant': significant,
            'better_method': better if significant else 'No significant difference'
        })

    return results, average_ranks

def perform_statistical_analysis(df, output_prefix='statistical_tests'):
    """
    Perform comprehensive statistical analysis.
    """
    variants = ['base', '2x', '5x', '1R10', '1R20']

    all_friedman_results = []
    all_wilcoxon_results = []
    all_nemenyi_results = []
    all_rank_results = []

    print("="*100)
    print("STATISTICAL ANALYSIS OF VRP-RPD EXPERIMENTS")
    print("="*100)

    for variant in variants:
        print(f"\n{'='*100}")
        print(f"VARIANT: {variant}")
        print(f"{'='*100}")

        # Prepare data for this variant
        data_pivot = prepare_data_for_testing(df, variant)

        if data_pivot.empty:
            print(f"No data found for variant {variant}")
            continue

        print(f"\nNumber of dataset-instances: {len(data_pivot)}")
        print(f"Methods compared: {list(data_pivot.columns)}")
        print(f"Missing data per method:")
        for col in data_pivot.columns:
            missing = data_pivot[col].isna().sum()
            print(f"  {col}: {missing}/{len(data_pivot)} missing")

        # 1. Friedman Test
        print(f"\n{'-'*100}")
        print("1. FRIEDMAN TEST (Overall comparison of all methods)")
        print(f"{'-'*100}")

        friedman_stat, friedman_p = friedman_test(data_pivot)

        if friedman_stat is not None:
            print(f"Friedman statistic: {friedman_stat:.4f}")
            print(f"P-value: {friedman_p:.6f}")
            print(f"Significant at α=0.05: {'YES' if friedman_p < 0.05 else 'NO'}")

            all_friedman_results.append({
                'variant': variant,
                'statistic': friedman_stat,
                'p_value': friedman_p,
                'significant': friedman_p < 0.05,
                'n_methods': len(data_pivot.columns),
                'n_datasets': len(data_pivot.dropna())
            })

            if friedman_p < 0.05:
                print("\nConclusion: There are significant differences among the methods.")

                # 2. Nemenyi Post-hoc Test
                print(f"\n{'-'*100}")
                print("2. NEMENYI POST-HOC TEST (Pairwise comparisons with rank differences)")
                print(f"{'-'*100}")

                nemenyi_results, average_ranks = perform_nemenyi_test(data_pivot)

                print(f"\nAverage Ranks (lower is better):")
                sorted_ranks = sorted(average_ranks.items(), key=lambda x: x[1])
                for i, (method, rank) in enumerate(sorted_ranks, 1):
                    print(f"  {i}. {method:<25} Rank: {rank:.3f}")

                # Save ranks
                for method, rank in average_ranks.items():
                    all_rank_results.append({
                        'variant': variant,
                        'method': method,
                        'average_rank': rank
                    })

                print(f"\nPairwise comparisons (Critical Distance: {nemenyi_results[0]['critical_distance']:.3f}):")
                print(f"{'Method 1':<25} {'Method 2':<25} {'Rank Diff':>10} {'Significant':>12} {'Better Method':<25}")
                print("-"*100)

                for result in nemenyi_results:
                    sig_marker = '***' if result['significant'] else 'ns'
                    print(f"{result['method1']:<25} {result['method2']:<25} "
                          f"{result['rank_difference']:>10.3f} {sig_marker:>12} "
                          f"{result['better_method']:<25}")

                    # Save result
                    all_nemenyi_results.append({
                        'variant': variant,
                        'method1': result['method1'],
                        'method2': result['method2'],
                        'rank1': result['rank1'],
                        'rank2': result['rank2'],
                        'rank_difference': result['rank_difference'],
                        'critical_distance': result['critical_distance'],
                        'significant': result['significant'],
                        'better_method': result['better_method']
                    })
            else:
                print("\nConclusion: No significant differences among the methods.")

        # 3. Wilcoxon Signed-Rank Tests (Pairwise)
        print(f"\n{'-'*100}")
        print("3. WILCOXON SIGNED-RANK TESTS (Pairwise comparisons with p-values)")
        print(f"{'-'*100}")

        methods = list(data_pivot.columns)
        n_comparisons = len(list(combinations(methods, 2)))
        bonferroni_alpha = bonferroni_correction([0.05] * n_comparisons)

        print(f"\nPerforming {n_comparisons} pairwise comparisons")
        print(f"Bonferroni-corrected α: {bonferroni_alpha:.6f}")

        print(f"\n{'Method 1':<25} {'Method 2':<25} {'P-value':>12} {'Adj. Sig.':>10} {'Effect Size':>12} {'Better Method':<25}")
        print("-"*120)

        wilcoxon_results = []

        for method1, method2 in combinations(methods, 2):
            stat, p_val, effect_size = wilcoxon_signed_rank_test(
                data_pivot[method1].values,
                data_pivot[method2].values
            )

            if p_val is not None:
                significant_bonf = p_val < bonferroni_alpha
                sig_marker = '***' if significant_bonf else 'ns'

                # Determine which method is better (lower makespan is better)
                mean1 = data_pivot[method1].mean()
                mean2 = data_pivot[method2].mean()
                better = method1 if mean1 < mean2 else method2
                better_str = better if significant_bonf else 'No significant difference'

                print(f"{method1:<25} {method2:<25} {p_val:>12.6f} {sig_marker:>10} "
                      f"{effect_size:>12.3f} {better_str:<25}")

                wilcoxon_results.append({
                    'method1': method1,
                    'method2': method2,
                    'statistic': stat,
                    'p_value': p_val,
                    'effect_size': effect_size,
                    'significant_bonferroni': significant_bonf,
                    'better_method': better_str,
                    'mean1': mean1,
                    'mean2': mean2
                })

        # Save Wilcoxon results
        for result in wilcoxon_results:
            all_wilcoxon_results.append({
                'variant': variant,
                **result
            })

        # Special analysis: Heuristics vs each other method
        print(f"\n{'-'*100}")
        print("4. HEURISTICS vs OTHER METHODS (Special Focus)")
        print(f"{'-'*100}")

        if 'heuristics_only' in data_pivot.columns:
            heur_data = data_pivot['heuristics_only']

            print(f"\n{'Comparison':<50} {'P-value':>12} {'Significant':>12} {'Effect Size':>12} {'Result':<30}")
            print("-"*120)

            for method in methods:
                if method != 'heuristics_only':
                    stat, p_val, effect_size = wilcoxon_signed_rank_test(
                        heur_data.values,
                        data_pivot[method].values
                    )

                    if p_val is not None:
                        significant = p_val < 0.05
                        sig_marker = '***' if significant else 'ns'

                        mean_heur = heur_data.mean()
                        mean_other = data_pivot[method].mean()

                        if significant:
                            if mean_heur < mean_other:
                                result = f"Heuristics better (Δ={mean_other-mean_heur:.2f})"
                            else:
                                result = f"{method} better (Δ={mean_heur-mean_other:.2f})"
                        else:
                            result = "No significant difference"

                        print(f"heuristics_only vs {method:<30} {p_val:>12.6f} {sig_marker:>12} "
                              f"{effect_size:>12.3f} {result:<30}")

    # Save all results to CSV files
    print(f"\n{'='*100}")
    print("SAVING RESULTS TO CSV FILES")
    print(f"{'='*100}")

    if all_friedman_results:
        friedman_df = pd.DataFrame(all_friedman_results)
        friedman_file = f'{output_prefix}_friedman.csv'
        friedman_df.to_csv(friedman_file, index=False)
        print(f"✓ Friedman test results saved to: {friedman_file}")

    if all_nemenyi_results:
        nemenyi_df = pd.DataFrame(all_nemenyi_results)
        nemenyi_file = f'{output_prefix}_nemenyi.csv'
        nemenyi_df.to_csv(nemenyi_file, index=False)
        print(f"✓ Nemenyi post-hoc results saved to: {nemenyi_file}")

    if all_wilcoxon_results:
        wilcoxon_df = pd.DataFrame(all_wilcoxon_results)
        wilcoxon_file = f'{output_prefix}_wilcoxon.csv'
        wilcoxon_df.to_csv(wilcoxon_file, index=False)
        print(f"✓ Wilcoxon signed-rank results saved to: {wilcoxon_file}")

    if all_rank_results:
        rank_df = pd.DataFrame(all_rank_results)
        rank_file = f'{output_prefix}_average_ranks.csv'
        rank_df.to_csv(rank_file, index=False)
        print(f"✓ Average ranks saved to: {rank_file}")

    # Create summary report
    create_summary_report(all_friedman_results, all_nemenyi_results, all_wilcoxon_results,
                         all_rank_results, output_prefix)

    return all_friedman_results, all_nemenyi_results, all_wilcoxon_results, all_rank_results

def create_summary_report(friedman_results, nemenyi_results, wilcoxon_results, rank_results, output_prefix):
    """Create a comprehensive summary report."""

    report_file = f'{output_prefix}_summary_report.txt'

    with open(report_file, 'w') as f:
        f.write("="*100 + "\n")
        f.write("STATISTICAL ANALYSIS SUMMARY REPORT\n")
        f.write("VRP-RPD Experimental Results\n")
        f.write("="*100 + "\n\n")

        # Overall summary
        f.write("OVERVIEW\n")
        f.write("-"*100 + "\n")
        f.write(f"Number of variants analyzed: {len(set(r['variant'] for r in friedman_results))}\n")
        f.write(f"Variants: {', '.join(sorted(set(r['variant'] for r in friedman_results)))}\n\n")

        # Friedman test summary
        f.write("FRIEDMAN TEST RESULTS (Overall comparison)\n")
        f.write("-"*100 + "\n")
        for result in friedman_results:
            f.write(f"\nVariant: {result['variant']}\n")
            f.write(f"  Statistic: {result['statistic']:.4f}\n")
            f.write(f"  P-value: {result['p_value']:.6f}\n")
            f.write(f"  Significant: {'YES' if result['significant'] else 'NO'}\n")
            f.write(f"  Methods compared: {result['n_methods']}\n")
            f.write(f"  Dataset instances: {result['n_datasets']}\n")

        # Method rankings
        f.write("\n\n")
        f.write("METHOD RANKINGS (Average ranks across datasets, lower is better)\n")
        f.write("-"*100 + "\n")

        rank_df = pd.DataFrame(rank_results)
        for variant in sorted(rank_df['variant'].unique()):
            variant_ranks = rank_df[rank_df['variant'] == variant].sort_values('average_rank')
            f.write(f"\nVariant: {variant}\n")
            for i, row in enumerate(variant_ranks.itertuples(), 1):
                f.write(f"  {i}. {row.method:<30} Rank: {row.average_rank:.3f}\n")

        # Significant pairwise differences (Nemenyi)
        f.write("\n\n")
        f.write("SIGNIFICANT PAIRWISE DIFFERENCES (Nemenyi post-hoc test)\n")
        f.write("-"*100 + "\n")

        nemenyi_df = pd.DataFrame(nemenyi_results)
        for variant in sorted(nemenyi_df['variant'].unique()):
            significant = nemenyi_df[(nemenyi_df['variant'] == variant) & (nemenyi_df['significant'])]
            f.write(f"\nVariant: {variant}\n")
            if len(significant) == 0:
                f.write("  No significant pairwise differences found.\n")
            else:
                for row in significant.itertuples():
                    f.write(f"  {row.method1} vs {row.method2}: {row.better_method} is significantly better\n")

        # Key findings for heuristics comparisons
        f.write("\n\n")
        f.write("HEURISTICS vs OTHER METHODS (Key Findings)\n")
        f.write("-"*100 + "\n")

        wilcoxon_df = pd.DataFrame(wilcoxon_results)
        heuristic_comparisons = wilcoxon_df[
            (wilcoxon_df['method1'] == 'heuristics_only') |
            (wilcoxon_df['method2'] == 'heuristics_only')
        ]

        for variant in sorted(heuristic_comparisons['variant'].unique()):
            variant_comps = heuristic_comparisons[heuristic_comparisons['variant'] == variant]
            f.write(f"\nVariant: {variant}\n")

            for row in variant_comps.itertuples():
                other_method = row.method2 if row.method1 == 'heuristics_only' else row.method1
                if row.significant_bonferroni:
                    f.write(f"  {other_method}: {row.better_method} (p={row.p_value:.6f}, effect={row.effect_size:.3f})\n")
                else:
                    f.write(f"  {other_method}: No significant difference (p={row.p_value:.6f})\n")

        f.write("\n\n")
        f.write("="*100 + "\n")
        f.write("END OF REPORT\n")
        f.write("="*100 + "\n")

    print(f"✓ Summary report saved to: {report_file}")

def main():
    """Main execution function."""
    print("Loading data from makespan_comparison_detailed.csv...")

    try:
        df = load_data('makespan_comparison_detailed.csv')
        print(f"Loaded {len(df)} records")

        # Filter out checkpoint files (rows without solve_time) for analysis
        df_filtered = df[df['solve_time'].notna()].copy()
        print(f"Records with solve_time (excluding checkpoints): {len(df_filtered)}")
        print(f"Checkpoint records excluded: {len(df) - len(df_filtered)}")

        print(f"Experiments: {df_filtered['experiment'].unique()}")
        print(f"Datasets: {df_filtered['dataset'].unique()}")
        print(f"Variants: {df_filtered['variant'].unique()}")

        # Perform statistical analysis using filtered data
        perform_statistical_analysis(df_filtered)

        print("\n" + "="*100)
        print("ANALYSIS COMPLETE!")
        print("="*100)
        print("\nGenerated files:")
        print("  1. statistical_tests_friedman.csv - Overall Friedman test results")
        print("  2. statistical_tests_nemenyi.csv - Nemenyi post-hoc pairwise comparisons")
        print("  3. statistical_tests_wilcoxon.csv - Wilcoxon signed-rank pairwise tests")
        print("  4. statistical_tests_average_ranks.csv - Average ranks for each method")
        print("  5. statistical_tests_summary_report.txt - Comprehensive summary report")

    except FileNotFoundError:
        print("Error: makespan_comparison_detailed.csv not found!")
        print("Please run extract_makespan_comparison.py first to generate the data.")
        return
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()

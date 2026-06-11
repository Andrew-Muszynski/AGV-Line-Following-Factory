#!/bin/bash
# Run complete statistical analysis pipeline

echo "=========================================================================="
echo "VRP-RPD Statistical Analysis Pipeline"
echo "=========================================================================="
echo ""

# Step 1: Extract makespan data
echo "Step 1: Extracting makespan data from results..."
echo "--------------------------------------------------------------------------"
python3 extract_makespan_comparison.py
if [ $? -ne 0 ]; then
    echo "Error: Failed to extract makespan data"
    exit 1
fi
echo ""

# Step 2: Run statistical tests
echo "Step 2: Running statistical tests..."
echo "--------------------------------------------------------------------------"
python3 statistical_analysis.py
if [ $? -ne 0 ]; then
    echo "Error: Failed to run statistical analysis"
    exit 1
fi
echo ""

# # Step 3: Generate visualizations
# echo "Step 3: Generating visualizations..."
# echo "--------------------------------------------------------------------------"
# python3 visualize_statistical_results.py
# if [ $? -ne 0 ]; then
#     echo "Error: Failed to generate visualizations"
#     exit 1
# fi
# echo ""

echo "=========================================================================="
echo "ANALYSIS COMPLETE!"
echo "=========================================================================="
echo ""
echo "Generated files:"
echo ""
echo "Data files:"
echo "  - makespan_comparison_detailed.csv"
echo "  - makespan_comparison_summary.csv"
echo "  - makespan_comparison_pivot.csv"
echo "  - makespan_improvement_over_heuristics.csv"
echo "  - makespan_best_experiments.csv"
echo ""
echo "Statistical test results:"
echo "  - statistical_tests_friedman.csv"
echo "  - statistical_tests_nemenyi.csv"
echo "  - statistical_tests_wilcoxon.csv"
echo "  - statistical_tests_average_ranks.csv"
echo "  - statistical_tests_summary_report.txt"
echo ""
echo "Visualizations:"
echo "  - ranks_visualization.png"
echo "  - significance_matrix_*.png (one for each variant)"
echo "  - pvalue_heatmap_*.png (one for each variant)"
echo "  - effect_sizes.png"
echo "  - heuristics_comparison.png"
echo "  - summary_table.png"
echo ""
echo "=========================================================================="

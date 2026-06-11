#!/bin/bash
# Batch run ALNS on all heuristic solutions

# Configuration
ALNS_TIME=300        # 5 minutes per run
ALNS_ITER=20000
PARALLEL=4
OUTPUT_DIR="results/alns_ws"

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Function to run ALNS for a dataset
run_alns_for_dataset() {
    local dataset=$1
    local scenario=$2

    # Find TSP file in datasets directory
    TSP_FILE="datasets/${dataset}/${dataset}.tsp"

    if [ ! -f "$TSP_FILE" ]; then
        echo "ERROR: TSP file not found: $TSP_FILE"
        return 1
    fi

    # Find jobs file in datasets directory
    # For base, 2x, 5x: use job_times.txt
    # For 1R10, 1R20: use job_times_1.txt (first instance as default)
    case $scenario in
        "1R10"|"1R20")
            # Use first instance by default
            JOBS_FILE="datasets/${dataset}/${scenario}/job_times_1.csv"
            ;;
        *)
            JOBS_FILE="datasets/${dataset}/${scenario}/job_times.csv"
            ;;
    esac

    # Check if jobs file exists
    if [ ! -f "$JOBS_FILE" ]; then
        echo "ERROR: Jobs file not found: $JOBS_FILE"
        return 1
    fi

    # Find heuristic results
    HEURISTIC_JSON="results/heuristics_only/${dataset}/${scenario}/${dataset}_heuristics.json"

    if [ ! -f "$HEURISTIC_JSON" ]; then
        echo "WARNING: Heuristic results not found: $HEURISTIC_JSON"
        echo "  Skipping $dataset/$scenario"
        return 1
    fi

    # Create output directory for this dataset/scenario
    DATASET_OUTPUT="$OUTPUT_DIR/${dataset}/${scenario}"
    mkdir -p "$DATASET_OUTPUT"

    OUTPUT_JSON="$DATASET_OUTPUT/${dataset}_alns.json"
    OUTPUT_HTML="$DATASET_OUTPUT/${dataset}_alns.html"
    LOG_FILE="$DATASET_OUTPUT/${dataset}_alns.log"

    echo ""
    echo "=========================================="
    echo "Running ALNS: $dataset ($scenario)"
    echo "=========================================="
    echo "TSP:        $TSP_FILE"
    echo "Jobs:       $JOBS_FILE"
    echo "Warm-start: $HEURISTIC_JSON"
    echo "Output:     $OUTPUT_JSON"
    echo ""

    # Run ALNS with warm-start
    python run_alns.py \
        --tsp "$TSP_FILE" \
        --jobs "$JOBS_FILE" \
        --warmstart "$HEURISTIC_JSON" \
        --output "$OUTPUT_JSON" \
        --html "$OUTPUT_HTML" \
        --alns_time $ALNS_TIME \
        --alns_iter $ALNS_ITER \
        --parallel $PARALLEL \
        --seed 0 \
        2>&1 | tee "$LOG_FILE"

    if [ $? -eq 0 ]; then
        echo "SUCCESS: ALNS completed for $dataset/$scenario"
    else
        echo "ERROR: ALNS failed for $dataset/$scenario"
    fi
}

# Main execution
echo "========================================"
echo "ALNS Batch Runner"
echo "========================================"
echo "ALNS time limit: ${ALNS_TIME}s"
echo "ALNS iterations: $ALNS_ITER"
echo "Parallel workers: $PARALLEL"
echo "Output directory: $OUTPUT_DIR"
echo ""

# All 14 datasets
DATASETS=(
    "berlin52"
    "bays29"
    "gr17"
    "gr21"
    "gr24"
    "gr48"
    "eil51"
    "eil101"
    "kroA100"
    "gr202"
    "gr431"
    "gr666"
    "rat783"
    "dsj1000"
)

# Run base variant for all datasets (SKIPPED - already done)
# echo ""
# echo "========================================"
# echo "WAVE 1: BASE VARIANTS"
# echo "========================================"
# for dataset in "${DATASETS[@]}"; do
#     run_alns_for_dataset "$dataset" "base"
# done

# Run 2x variants for all datasets
echo ""
echo "========================================"
echo "WAVE 1: 2x VARIANTS"
echo "========================================"
for dataset in "${DATASETS[@]}"; do
    run_alns_for_dataset "$dataset" "2x"
done

# Run 5x variants for all datasets
echo ""
echo "========================================"
echo "WAVE 2: 5x VARIANTS"
echo "========================================"
for dataset in "${DATASETS[@]}"; do
    run_alns_for_dataset "$dataset" "5x"
done

# Run 1R10 variants (job_times_1 and job_times_2 only)
echo ""
echo "========================================"
echo "WAVE 3: 1R10 - job_times_1"
echo "========================================"
for dataset in "${DATASETS[@]}"; do
    # For 1R10, need to run each job file separately
    JOBS_FILE="datasets/${dataset}/1R10/job_times_1.csv"
    TSP_FILE="datasets/${dataset}/${dataset}.tsp"
    HEURISTIC_JSON="results/heuristics_only/${dataset}/1R10/job_times_1_heuristics.json"

    if [ ! -f "$JOBS_FILE" ] || [ ! -f "$HEURISTIC_JSON" ]; then
        echo "Skipping ${dataset}/1R10/job_times_1 - missing files"
        continue
    fi

    DATASET_OUTPUT="$OUTPUT_DIR/${dataset}/1R10"
    mkdir -p "$DATASET_OUTPUT"

    OUTPUT_JSON="$DATASET_OUTPUT/job_times_1_alns.json"
    OUTPUT_HTML="$DATASET_OUTPUT/job_times_1_alns.html"
    LOG_FILE="$DATASET_OUTPUT/job_times_1_alns.log"

    echo ""
    echo "=========================================="
    echo "Running ALNS: $dataset (1R10/job_times_1)"
    echo "=========================================="
    echo "TSP:        $TSP_FILE"
    echo "Jobs:       $JOBS_FILE"
    echo "Warm-start: $HEURISTIC_JSON"
    echo "Output:     $OUTPUT_JSON"
    echo ""

    python run_alns.py \
        --tsp "$TSP_FILE" \
        --jobs "$JOBS_FILE" \
        --warmstart "$HEURISTIC_JSON" \
        --output "$OUTPUT_JSON" \
        --html "$OUTPUT_HTML" \
        --alns_time $ALNS_TIME \
        --alns_iter $ALNS_ITER \
        --parallel $PARALLEL \
        --seed 0 \
        2>&1 | tee "$LOG_FILE"
done

echo ""
echo "========================================"
echo "WAVE 4: 1R10 - job_times_2"
echo "========================================"
for dataset in "${DATASETS[@]}"; do
    # For 1R10, need to run each job file separately
    JOBS_FILE="datasets/${dataset}/1R10/job_times_2.csv"
    TSP_FILE="datasets/${dataset}/${dataset}.tsp"
    HEURISTIC_JSON="results/heuristics_only/${dataset}/1R10/job_times_2_heuristics.json"

    if [ ! -f "$JOBS_FILE" ] || [ ! -f "$HEURISTIC_JSON" ]; then
        echo "Skipping ${dataset}/1R10/job_times_2 - missing files"
        continue
    fi

    DATASET_OUTPUT="$OUTPUT_DIR/${dataset}/1R10"
    mkdir -p "$DATASET_OUTPUT"

    OUTPUT_JSON="$DATASET_OUTPUT/job_times_2_alns.json"
    OUTPUT_HTML="$DATASET_OUTPUT/job_times_2_alns.html"
    LOG_FILE="$DATASET_OUTPUT/job_times_2_alns.log"

    echo ""
    echo "=========================================="
    echo "Running ALNS: $dataset (1R10/job_times_2)"
    echo "=========================================="
    echo "TSP:        $TSP_FILE"
    echo "Jobs:       $JOBS_FILE"
    echo "Warm-start: $HEURISTIC_JSON"
    echo "Output:     $OUTPUT_JSON"
    echo ""

    python run_alns.py \
        --tsp "$TSP_FILE" \
        --jobs "$JOBS_FILE" \
        --warmstart "$HEURISTIC_JSON" \
        --output "$OUTPUT_JSON" \
        --html "$OUTPUT_HTML" \
        --alns_time $ALNS_TIME \
        --alns_iter $ALNS_ITER \
        --parallel $PARALLEL \
        --seed 0 \
        2>&1 | tee "$LOG_FILE"
done

# # Run 1R20 variants (job_times_1 and job_times_2 only)
# echo ""
# echo "========================================"
# echo "WAVE 5: 1R20 - job_times_1"
# echo "========================================"
# for dataset in "${DATASETS[@]}"; do
#     # For 1R20, need to run each job file separately
#     JOBS_FILE="datasets/${dataset}/1R20/job_times_1.csv"
#     TSP_FILE="datasets/${dataset}/${dataset}.tsp"
#     HEURISTIC_JSON="results/heuristics_only/${dataset}/1R20/job_times_1_heuristics.json"

#     if [ ! -f "$JOBS_FILE" ] || [ ! -f "$HEURISTIC_JSON" ]; then
#         echo "Skipping ${dataset}/1R20/job_times_1 - missing files"
#         continue
#     fi

#     DATASET_OUTPUT="$OUTPUT_DIR/${dataset}/1R20"
#     mkdir -p "$DATASET_OUTPUT"

#     OUTPUT_JSON="$DATASET_OUTPUT/job_times_1_alns.json"
#     OUTPUT_HTML="$DATASET_OUTPUT/job_times_1_alns.html"
#     LOG_FILE="$DATASET_OUTPUT/job_times_1_alns.log"

#     echo ""
#     echo "=========================================="
#     echo "Running ALNS: $dataset (1R20/job_times_1)"
#     echo "=========================================="
#     echo "TSP:        $TSP_FILE"
#     echo "Jobs:       $JOBS_FILE"
#     echo "Warm-start: $HEURISTIC_JSON"
#     echo "Output:     $OUTPUT_JSON"
#     echo ""

#     python run_alns.py \
#         --tsp "$TSP_FILE" \
#         --jobs "$JOBS_FILE" \
#         --warmstart "$HEURISTIC_JSON" \
#         --output "$OUTPUT_JSON" \
#         --html "$OUTPUT_HTML" \
#         --alns_time $ALNS_TIME \
#         --alns_iter $ALNS_ITER \
#         --parallel $PARALLEL \
#         --seed 0 \
#         2>&1 | tee "$LOG_FILE"
# done

# echo ""
# echo "========================================"
# echo "WAVE 6: 1R20 - job_times_2"
# echo "========================================"
# for dataset in "${DATASETS[@]}"; do
#     # For 1R20, need to run each job file separately
#     JOBS_FILE="datasets/${dataset}/1R20/job_times_2.csv"
#     TSP_FILE="datasets/${dataset}/${dataset}.tsp"
#     HEURISTIC_JSON="results/heuristics_only/${dataset}/1R20/job_times_2_heuristics.json"

#     if [ ! -f "$JOBS_FILE" ] || [ ! -f "$HEURISTIC_JSON" ]; then
#         echo "Skipping ${dataset}/1R20/job_times_2 - missing files"
#         continue
#     fi

#     DATASET_OUTPUT="$OUTPUT_DIR/${dataset}/1R20"
#     mkdir -p "$DATASET_OUTPUT"

#     OUTPUT_JSON="$DATASET_OUTPUT/job_times_2_alns.json"
#     OUTPUT_HTML="$DATASET_OUTPUT/job_times_2_alns.html"
#     LOG_FILE="$DATASET_OUTPUT/job_times_2_alns.log"

#     echo ""
#     echo "=========================================="
#     echo "Running ALNS: $dataset (1R20/job_times_2)"
#     echo "=========================================="
#     echo "TSP:        $TSP_FILE"
#     echo "Jobs:       $JOBS_FILE"
#     echo "Warm-start: $HEURISTIC_JSON"
#     echo "Output:     $OUTPUT_JSON"
#     echo ""

#     python run_alns.py \
#         --tsp "$TSP_FILE" \
#         --jobs "$JOBS_FILE" \
#         --warmstart "$HEURISTIC_JSON" \
#         --output "$OUTPUT_JSON" \
#         --html "$OUTPUT_HTML" \
#         --alns_time $ALNS_TIME \
#         --alns_iter $ALNS_ITER \
#         --parallel $PARALLEL \
#         --seed 0 \
#         2>&1 | tee "$LOG_FILE"
# done

echo ""
echo "========================================"
echo "ALNS Batch Complete"
echo "========================================"
echo "Results saved in: $OUTPUT_DIR"
echo ""

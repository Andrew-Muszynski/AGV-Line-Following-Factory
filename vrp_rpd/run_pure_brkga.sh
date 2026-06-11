#!/bin/bash

# Pure BRKGA Experiment (Experiment 3)
# Runs vanilla BRKGA with NO warm start, NO GP analysis, NO gene injection
# Provides baseline for comparison against GP-enhanced BRKGA
# Runs for 10,000 generations with checkpoints at 1,000, 5,000, and 10,000

# Dataset configuration
DATASETS=(
  gr17 gr21 gr24 gr48 eil51 berlin52 eil101 kroA100 gr202 gr431 gr666 rat783 dsj1000
)

VARIANTS=(base 1R10 1R20 2x 5x)

BASE_DIR=./datasets
OUT_DIR=./results/pure_brkga
SCRIPT=main.py

# BRKGA Configuration
TOTAL_GENS=10000
CHECKPOINT_INTERVAL=1000  # Will save at 1000, 2000, 3000, ..., 10000
SEEDS=(0 1 2)  # Run 3 independent seeds for statistical validity

# GPU Configuration (default values, can be overridden)
DEFAULT_GPUS=1
DEFAULT_POP_GPU=512

GPU_ID=$1
DATASET_INDEX=$2
VARIANT=$3

if [ -z "$DATASET_INDEX" ]; then
    echo "Usage: $0 <gpu_id> <dataset_index> [variant]"
    echo ""
    echo "Arguments:"
    echo "  gpu_id: GPU ID to use (e.g., 0, 1, 2, etc.)"
    echo "  dataset_index: Index of dataset to run"
    echo "  variant: (optional) Specific variant to run"
    echo ""
    echo "Available datasets:"
    for i in "${!DATASETS[@]}"; do
        echo "  $i: ${DATASETS[$i]}"
    done
    echo ""
    echo "Variants: base, 1R10, 1R20, 2x, 5x"
    echo "  If no variant specified, runs ALL variants"
    echo ""
    echo "Example:"
    echo "  $0 0 5  # Run berlin52 (index 5) on GPU 0, all variants"
    echo "  $0 1 0 base  # Run gr17 (index 0) base variant on GPU 1"
    exit 1
fi

DATASET=${DATASETS[$DATASET_INDEX]}
TSP_FILE=${BASE_DIR}/${DATASET}/${DATASET}.tsp

echo "========================================"
echo "PURE BRKGA EXPERIMENT (Baseline)"
echo "========================================"
echo "GPU ID: ${GPU_ID}"
echo "Dataset: ${DATASET}"
echo "Total Generations: ${TOTAL_GENS}"
echo "Checkpoints: Every ${CHECKPOINT_INTERVAL} generations"
echo "GPU Config: ${DEFAULT_GPUS} GPU(s), Population ${DEFAULT_POP_GPU}"
echo ""
echo "Configuration:"
echo "  Warm Start: NO"
echo "  GP Analysis: NO"
echo "  Gene Injection: NO"
echo "  (Pure vanilla BRKGA baseline)"
echo "========================================"
echo ""

# Function to run Pure BRKGA experiment with multiple seeds
run_pure_brkga_experiment() {
    local variant=$1
    local job_file=$2
    local output_name=$3

    # Create output directory
    local run_dir=${OUT_DIR}/${DATASET}/${variant}
    mkdir -p ${run_dir}

    echo ""
    echo "  Running: ${variant}/${output_name} - Pure BRKGA"
    echo "  Output: ${run_dir}"

    # Run across multiple seeds
    for SEED in "${SEEDS[@]}"; do
        echo "    Seed ${SEED}..."

        # Build command with GPU support
        # Pure BRKGA: --Warm no --GP no --GeneInjection no
        local cmd="CUDA_VISIBLE_DEVICES=${GPU_ID} python ${SCRIPT} \
            --tsp ${TSP_FILE} \
            --jobs ${job_file} \
            --gens ${TOTAL_GENS} \
            --checkpoint-interval ${CHECKPOINT_INTERVAL} \
            --Warm no \
            --GP no \
            --GeneInjection no \
            --gpus ${DEFAULT_GPUS} \
            --pop-gpu ${DEFAULT_POP_GPU} \
            --seed ${SEED} \
            --output ${run_dir}/${output_name}_seed${SEED}.json \
            --output_html ${run_dir}/${output_name}_seed${SEED}.html"

        # Redirect output to log file
        cmd="${cmd} > ${run_dir}/${output_name}_seed${SEED}_log.txt 2>&1"

        # Execute
        eval $cmd

        if [ $? -eq 0 ]; then
            echo "      ✓ Seed ${SEED} complete"
        else
            echo "      ✗ Seed ${SEED} failed (check ${run_dir}/${output_name}_seed${SEED}_log.txt)"
        fi
    done
}

# Process variants
for variant in "${VARIANTS[@]}"; do
    # Skip if a specific variant was requested and this isn't it
    if [ -n "$VARIANT" ] && [ "$variant" != "$VARIANT" ]; then
        continue
    fi

    variant_dir=${BASE_DIR}/${DATASET}/${variant}

    # Check if variant exists for this dataset
    if [ ! -d "$variant_dir" ]; then
        echo "Skipping ${variant} (directory not found)"
        continue
    fi

    echo ""
    echo "========================================"
    echo "Variant: ${variant}"
    echo "========================================"

    if [ "$variant" = "1R10" ] || [ "$variant" = "1R20" ]; then
        # Process numbered job files (1-10)
        for i in {1..10}; do
            job_file=${variant_dir}/full_jobs_${i}_proc_times.txt

            if [ -f "$job_file" ]; then
                output_name="full_jobs_${i}"
                run_pure_brkga_experiment "${variant}" "${job_file}" "${output_name}"
            else
                echo "  Skipping full_jobs_${i} (job file not found)"
            fi
        done
    else
        # Process single job file (base, 2x, 5x)
        job_file=${variant_dir}/full_jobs_proc_times.txt

        if [ -f "$job_file" ]; then
            output_name="${DATASET}"
            run_pure_brkga_experiment "${variant}" "${job_file}" "${output_name}"
        else
            echo "  Skipping ${variant} (job file not found)"
        fi
    fi
done

echo ""
echo "========================================"
echo "PURE BRKGA EXPERIMENT COMPLETE"
echo "========================================"
echo "Dataset: ${DATASET}"
echo ""
echo "Results location: ${OUT_DIR}/${DATASET}/"
echo ""
echo "Directory structure:"
echo "  ${OUT_DIR}/${DATASET}/"
echo "    ├── base/"
echo "    │   ├── <dataset>_seed0_1000.json  (checkpoint at 1000 gen)"
echo "    │   ├── <dataset>_seed0_5000.json  (checkpoint at 5000 gen)"
echo "    │   ├── <dataset>_seed0_10000.json (checkpoint at 10000 gen)"
echo "    │   ├── <dataset>_seed0.json       (final result)"
echo "    │   └── (same for seed1, seed2)"
echo "    ├── 1R10/"
echo "    ├── 1R20/"
echo "    ├── 2x/"
echo "    └── 5x/"
echo ""
echo "Compare with:"
echo "  - Exp 1 (Heuristics): ./results/heuristics_only/"
echo "  - Exp 2 (GP+Warm/Cold): ./results/brkga_comparison/"
echo "  - Exp 3 (Pure BRKGA): ./results/pure_brkga/"
echo "========================================"
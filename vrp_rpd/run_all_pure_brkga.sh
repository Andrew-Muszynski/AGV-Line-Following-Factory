#!/bin/bash

# Run Pure BRKGA Experiment across multiple datasets
# Runs vanilla BRKGA with NO warm start, NO GP analysis, NO gene injection
# Provides baseline for comparison against GP-enhanced BRKGA
# Usage: ./run_all_pure_brkga.sh <gpu_id> <variant1> [variant2] [variant3] ...

GPU_ID=$1
shift  # Remove first argument (GPU_ID) so $@ contains only variants

if [ -z "$GPU_ID" ] || [ $# -eq 0 ]; then
    echo "Usage: $0 <gpu_id> <variant1> [variant2] [variant3] ..."
    echo ""
    echo "Arguments:"
    echo "  gpu_id: GPU ID to use (e.g., 0, 1, 2, 3)"
    echo "  variants: One or more variants to run (base, 1R10, 1R20, 2x, 5x)"
    echo ""
    echo "Example:"
    echo "  $0 0 1R10           # Run 1R10 variant on GPU 0 (1 seed)"
    echo "  $0 1 base 2x 5x     # Run base, 2x, 5x variants on GPU 1 (3 seeds each)"
    echo ""
    echo "Note: base, 2x, and 5x variants will run with 3 seeds (0, 1, 2)"
    echo "      Other variants (1R10, 1R20) will run with 1 seed (0)"
    echo ""
    echo "This will run the following datasets:"
    echo "  kroA100 gr202 gr431 gr666 rat783 dsj1000 gr17 gr21 gr24 gr48"
    exit 1
fi

# Store all variants from command line arguments
VARIANTS=("$@")

# Seeds to use for multi-seed variants (base, 2x, 5x)
MULTI_SEEDS=(0)

# Datasets to run (ordered by size)
DATASETS=(eil101 eil51 gr17 gr202 gr21
  gr24 gr431 gr48 gr666 kroA100 rat783
)

#DATASETS=(berlin52 bays29 eil51 eil101)

BASE_DIR=./datasets
OUT_DIR=./results/pure_brkga
SCRIPT=main.py

# BRKGA Configuration
TOTAL_GENS=5000
CHECKPOINT_INTERVAL=1000  # Will save at 1000, 2000, 3000, 4000, 5000

# GPU Configuration (default values, can be overridden)
DEFAULT_GPUS=1
DEFAULT_POP_GPU=512

echo "========================================"
echo "PURE BRKGA - ALL DATASETS"
echo "========================================"
echo "GPU: ${GPU_ID}"
echo "Variants: ${VARIANTS[@]}"
echo "Datasets: ${DATASETS[@]}"
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

# Function to run Pure BRKGA experiment with a specific seed
run_pure_brkga_experiment() {
    local variant=$1
    local job_file=$2
    local output_name=$3
    local run_dir=$4
    local SEED=$5

    # Create output directory
    mkdir -p ${run_dir}

    echo "    Running: ${output_name} - Pure BRKGA (Seed ${SEED})"

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
}

# Function to run pure BRKGA for a single dataset with specific variant and seed
run_pure_brkga_dataset() {
    local DATASET=$1
    local VARIANT=$2
    local SEED=$3

    echo ""
    echo "========================================"
    echo "Dataset: ${DATASET} | Variant: ${VARIANT} | Seed: ${SEED}"
    echo "========================================"

    TSP_FILE=${BASE_DIR}/${DATASET}/${DATASET}.tsp
    variant_dir=${BASE_DIR}/${DATASET}/${VARIANT}

    # Check if variant directory exists
    if [ ! -d "$variant_dir" ]; then
        echo "⚠ Skipping ${DATASET} - variant ${VARIANT} not found"
        return
    fi

    # Handle 1R10/1R20 variants with multiple job files
    if [ "$VARIANT" = "1R10" ] || [ "$VARIANT" = "1R20" ]; then
        for i in {1..10}; do
            job_file=${variant_dir}/job_times_${i}.csv

            if [ -f "$job_file" ]; then
                output_name="job_times_${i}"
                run_dir=${OUT_DIR}/${DATASET}/${VARIANT}
                echo "  Processing: job_times_${i}.csv"
                run_pure_brkga_experiment "${VARIANT}" "${job_file}" "${output_name}" "${run_dir}" "${SEED}"
            else
                echo "  Skipping job_times_${i} (job file not found)"
            fi
        done
    else
        # Handle base, 2x, 5x variants with single job file
        job_file=${variant_dir}/job_times.csv

        if [ -f "$job_file" ]; then
            output_name="${DATASET}"
            run_dir=${OUT_DIR}/${DATASET}/${VARIANT}
            echo "  Processing: job_times.csv"
            run_pure_brkga_experiment "${VARIANT}" "${job_file}" "${output_name}" "${run_dir}" "${SEED}"
        else
            echo "  ⚠ Job file not found: $job_file"
        fi
    fi
}

# Run all combinations based on variant type
for VARIANT in "${VARIANTS[@]}"; do
    # Determine seeds based on variant type
    if [[ "$VARIANT" == "base" || "$VARIANT" == "2x" || "$VARIANT" == "5x" ]]; then
        # Multi-seed variants: run with seeds 0, 1, 2
        SEEDS=("${MULTI_SEEDS[@]}")
        echo "Running variant ${VARIANT} with ${#SEEDS[@]} seeds: ${SEEDS[@]}"
    else
        # Single-seed variants (1R10, 1R20, etc.): run with seed 0 only
        SEEDS=(0)
        echo "Running variant ${VARIANT} with 1 seed: 0"
    fi

    for SEED in "${SEEDS[@]}"; do
        for DATASET in "${DATASETS[@]}"; do
            run_pure_brkga_dataset "$DATASET" "$VARIANT" "$SEED"
        done
    done
done

echo ""
echo "========================================"
echo "ALL PURE BRKGA EXPERIMENTS COMPLETE"
echo "========================================"
echo "GPU: ${GPU_ID}"
echo "Variants: ${VARIANTS[@]}"
echo ""
echo "Results location: ${OUT_DIR}/"
echo ""
echo "Directory structure:"
echo "  ${OUT_DIR}/<dataset>/${VARIANT}/"
echo "    ├── <output>_seed0_1000.json  (checkpoint at 1000 gen)"
echo "    ├── <output>_seed0_2000.json  (checkpoint at 2000 gen)"
echo "    ├── <output>_seed0_3000.json  (checkpoint at 3000 gen)"
echo "    ├── <output>_seed0_4000.json  (checkpoint at 4000 gen)"
echo "    ├── <output>_seed0_5000.json  (checkpoint at 5000 gen)"
echo "    └── <output>_seed0.json       (final result)"
echo ""
echo "Compare with:"
echo "  - Exp 1 (Heuristics): ./results/heuristics_only/"
echo "  - Exp 2 (GP+Warm/Cold): ./results/brkga_comparison/"
echo "  - Exp 3 (Pure BRKGA): ./results/pure_brkga/"
echo "========================================"

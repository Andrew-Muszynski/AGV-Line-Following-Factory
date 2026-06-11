#!/bin/bash

# Run Warm Start Experiment across multiple datasets with different GP analysis configurations
# Usage: ./run_all_warm_start.sh <gpu_id> <variant1> [variant2] [variant3] ...
#
# This script tests different GP analysis configurations:
#   1. blocks_only          - Building blocks only (current default)
#   2. blocks_fft           - Building blocks + FFT frequency analysis
#   3. blocks_similarity    - Building blocks + Similarity clustering
#   4. blocks_fft_similarity - All three methods combined
#   5. fft_only             - FFT frequency analysis only
#   6. similarity_only      - Similarity clustering only
#   7. fft_similarity       - FFT + Similarity (no blocks)
#   8. none                 - No GP analysis (random-like candidates)
#
# Results are saved to: ./results/warm_start/<gp_config>/<dataset>/<variant>/

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
    echo "  gr202 gr431 gr666 rat783 dsj1000 kroA100 gr48 gr17 gr21 gr24"
    exit 1
fi

# Store all variants from command line arguments
VARIANTS=("$@")

# Seeds to use for multi-seed variants (base, 2x, 5x)
MULTI_SEEDS=(0)


# GP Analysis Component Combinations to test
# Format: "name:use_blocks:use_fft:use_similarity"
GP_CONFIGS=(
  "blocks_only:yes:no:no"
)

# Datasets to run (ordered by size)
DATASETS=(gr21 gr24 gr431 gr48 eil101 eil51 kroA100 gr202 dsj1000 gr666 rat783)

#DATASETS=(dsj1000 berlin52)

BASE_DIR=./datasets
HEURISTIC_DIR=./results/heuristics_only
OUT_DIR=./results/warm_start
SCRIPT=main.py

TOTAL_GENS=5000
CHECKPOINT_INTERVAL=1000
DEFAULT_GPUS=1
DEFAULT_POP_GPU=512

echo "========================================"
echo "WARM START - TESTING GP CONFIGS"
echo "========================================"
echo "GPU: ${GPU_ID}"
echo "Variants: ${VARIANTS[@]}"
echo "Datasets: ${DATASETS[@]}"
echo "GP Configs: ${#GP_CONFIGS[@]} combinations"
echo "Config: GP=yes, GeneInjection=yes, WarmStart=yes"
echo "========================================"
echo ""

# Function to run warm start for a single dataset with specific variant, seed, and GP config
run_warm_start_dataset() {
    local DATASET=$1
    local VARIANT=$2
    local SEED=$3
    local GP_CONFIG_NAME=$4
    local USE_BLOCKS=$5
    local USE_FFT=$6
    local USE_SIMILARITY=$7

    echo ""
    echo "========================================"
    echo "Dataset: ${DATASET} | Variant: ${VARIANT} | Seed: ${SEED} | GP: ${GP_CONFIG_NAME}"
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
            heuristic_json=${HEURISTIC_DIR}/${DATASET}/${VARIANT}/job_times_${i}_heuristics.json

            if [ ! -f "$job_file" ]; then
                continue
            fi

            if [ ! -f "$heuristic_json" ]; then
                echo "  ⚠ Heuristic JSON not found: ${heuristic_json}"
                echo "  Skipping job_times_${i}"
                continue
            fi

            output_name="job_times_${i}"
            run_dir=${OUT_DIR}/${GP_CONFIG_NAME}/${DATASET}/${VARIANT}
            mkdir -p ${run_dir}

            echo "  Processing: job_times_${i}.csv (blocks=${USE_BLOCKS} fft=${USE_FFT} sim=${USE_SIMILARITY})"

            CUDA_VISIBLE_DEVICES=${GPU_ID} python ${SCRIPT} \
                --tsp ${TSP_FILE} \
                --jobs ${job_file} \
                --gens ${TOTAL_GENS} \
                --checkpoint-interval ${CHECKPOINT_INTERVAL} \
                --Warm yes \
                --GP yes \
                --GeneInjection yes \
                --use-blocks ${USE_BLOCKS} \
                --use-fft ${USE_FFT} \
                --use-similarity ${USE_SIMILARITY} \
                --gpus ${DEFAULT_GPUS} \
                --pop-gpu ${DEFAULT_POP_GPU} \
                --seed ${SEED} \
                --from-json ${heuristic_json} \
                --output ${run_dir}/${output_name}_seed${SEED}.json \
                --output_html ${run_dir}/${output_name}_seed${SEED}.html \
                > ${run_dir}/${output_name}_seed${SEED}_log.txt 2>&1

            if [ $? -eq 0 ]; then
                echo "    ✓ Complete"
            else
                echo "    ✗ Failed (check ${run_dir}/${output_name}_seed${SEED}_log.txt)"
            fi
        done
    else
        # Handle base, 2x, 5x variants with single job file
        job_file=${variant_dir}/job_times.csv
        heuristic_json=${HEURISTIC_DIR}/${DATASET}/${VARIANT}/${DATASET}_heuristics.json

        if [ ! -f "$job_file" ]; then
            echo "  ⚠ Job file not found: $job_file"
            return
        fi

        if [ ! -f "$heuristic_json" ]; then
            echo "  ⚠ Heuristic JSON not found: ${heuristic_json}"
            return
        fi

        output_name="${DATASET}"
        run_dir=${OUT_DIR}/${GP_CONFIG_NAME}/${DATASET}/${VARIANT}
        mkdir -p ${run_dir}

        echo "  Processing: job_times.csv (blocks=${USE_BLOCKS} fft=${USE_FFT} sim=${USE_SIMILARITY})"

        CUDA_VISIBLE_DEVICES=${GPU_ID} python ${SCRIPT} \
            --tsp ${TSP_FILE} \
            --jobs ${job_file} \
            --gens ${TOTAL_GENS} \
            --checkpoint-interval ${CHECKPOINT_INTERVAL} \
            --Warm yes \
            --GP yes \
            --GeneInjection yes \
            --use-blocks ${USE_BLOCKS} \
            --use-fft ${USE_FFT} \
            --use-similarity ${USE_SIMILARITY} \
            --gpus ${DEFAULT_GPUS} \
            --pop-gpu ${DEFAULT_POP_GPU} \
            --seed ${SEED} \
            --from-json ${heuristic_json} \
            --output ${run_dir}/${output_name}_seed${SEED}.json \
            --output_html ${run_dir}/${output_name}_seed${SEED}.html \
            > ${run_dir}/${output_name}_seed${SEED}_log.txt 2>&1

        if [ $? -eq 0 ]; then
            echo "  ✓ Complete"
        else
            echo "  ✗ Failed (check ${run_dir}/${output_name}_seed${SEED}_log.txt)"
        fi
    fi
}

# Run all combinations: GP configs × variants × seeds × datasets
for GP_CONFIG in "${GP_CONFIGS[@]}"; do
    # Parse GP config: "name:use_blocks:use_fft:use_similarity"
    IFS=':' read -r GP_CONFIG_NAME USE_BLOCKS USE_FFT USE_SIMILARITY <<< "$GP_CONFIG"

    echo ""
    echo "========================================"
    echo "GP Configuration: ${GP_CONFIG_NAME}"
    echo "  Blocks: ${USE_BLOCKS} | FFT: ${USE_FFT} | Similarity: ${USE_SIMILARITY}"
    echo "========================================"

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
                run_warm_start_dataset "$DATASET" "$VARIANT" "$SEED" "$GP_CONFIG_NAME" "$USE_BLOCKS" "$USE_FFT" "$USE_SIMILARITY"
            done
        done
    done
done

echo ""
echo "========================================"
echo "ALL WARM START EXPERIMENTS COMPLETE"
echo "========================================"
echo "GPU: ${GPU_ID}"
echo "Variants: ${VARIANTS[@]}"
echo "GP Configs tested: ${#GP_CONFIGS[@]}"
echo ""
echo "Results location: ${OUT_DIR}/"
echo "  Each GP config has its own subdirectory:"
for GP_CONFIG in "${GP_CONFIGS[@]}"; do
    IFS=':' read -r GP_CONFIG_NAME _ _ _ <<< "$GP_CONFIG"
    echo "    - ${OUT_DIR}/${GP_CONFIG_NAME}/"
done
echo "========================================"

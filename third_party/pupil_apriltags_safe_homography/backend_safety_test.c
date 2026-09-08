#include <assert.h>
#include <stdio.h>

#include "apriltag.h"

extern int quad_update_homographies(struct quad *quad);

int main(void)
{
    const double valid[4][4] = {
        {-1, -1, 10, 10}, {1, -1, 20, 10},
        {1, 1, 20, 20}, {-1, 1, 10, 20},
    };
    const double degenerate[4][4] = {{0}};

    struct quad quad = {0};
    apriltag_reset_rejected_homography_count();
    assert(quad_update_homographies(&quad) == -1);
    assert(quad.H == NULL);
    assert(quad.Hinv == NULL);
    assert(apriltag_get_rejected_homography_count() == 1);

    apriltag_reset_rejected_homography_count();
    for (int i = 0; i < 10000; i++) {
        assert(apriltag_validate_homography_correspondences(&valid[0][0]) == 1);
        assert(apriltag_validate_homography_correspondences(
            &degenerate[0][0]) == 0);
    }
    assert(apriltag_get_rejected_homography_count() == 10000);
    puts("backend safety test: 10000 valid + 10000 degenerate solves passed");
    return 0;
}

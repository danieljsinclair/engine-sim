#include <gtest/gtest.h>

#include "../include/function.h"

#include <stdlib.h>

TEST(FunctionTests, FunctionSanityCheck) {
    Function f;
    f.initialize(16, 1.0);
    f.destroy();
}

TEST(FunctionTests, FunctionTriangleFilterTest) {
    Function f;
    f.initialize(0, 1.0);
    for (int i = 0; i < 10; ++i) {
        f.addSample((double)i, (double)i * 2);
    }

    EXPECT_NEAR(f.sampleTriangle(-1.0), 0.0, 1E-6);
    EXPECT_NEAR(f.sampleTriangle(11.0), 18.0, 1E-6);

    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(f.sampleTriangle((double)i), (double)i * 2, 1E-6);
    }

    f.destroy();
}

TEST(FunctionTests, FunctionClosestTest) {
    Function f;
    f.initialize(0, 1.0);
    f.addSample(0.0, 1.0);
    f.addSample(2.0, 1.0);
    f.addSample(3.0, 1.1);
    f.addSample(1.0, 1.0);
    f.addSample(5.0, 10.0);
    f.addSample(4.0, 9.0);

    EXPECT_EQ(f.closestSample(2.4), 2);
    EXPECT_EQ(f.closestSample(6.0), 5);

    f.destroy();
}

TEST(FunctionTests, FunctionRandomAddTest) {
    Function f;
    f.initialize(0, 1.0);

    for (int i = 0; i < 1000; ++i) {
        f.addSample(rand() % 1000, i);
    }

    EXPECT_TRUE(f.isOrdered());

    f.destroy();
}

// Verifies Gaussian interpolation, including out-of-range queries clamped to
// the boundary samples. Previously disabled due to an out-of-bounds cache
// read in GaussianFilter::evaluate() for large |s|; now fixed.
TEST(FunctionTests, FunctionGaussianTest) {
    Function f;
    f.initialize(0, 1.0);
    f.addSample(0.0, 1.0);
    f.addSample(2.0, 1.0);
    f.addSample(3.0, 5.0);
    f.addSample(1.0, 1.0);
    f.addSample(5.0, 10.0);
    f.addSample(4.0, 9.0);

    // Out-of-range queries must clamp to the boundary samples (this is the
    // behaviour that previously segfaulted via an out-of-bounds cache read).
    EXPECT_NEAR(f.sampleGaussian(100.0), 10.0, 1E-3);
    EXPECT_NEAR(f.sampleGaussian(-100.0), 1.0, 1E-3);

    // In-range interpolation must stay within the data's [yMin, yMax] range.
    const double v_mid = f.sampleGaussian(2.5);
    EXPECT_GE(v_mid, 1.0);
    EXPECT_LE(v_mid, 10.0);

    // The region between x=2 and x=5 is monotonically increasing, so the
    // smoothed value must respect that ordering.
    EXPECT_LT(f.sampleGaussian(2.0), f.sampleGaussian(3.5));
    EXPECT_LT(f.sampleGaussian(3.5), f.sampleGaussian(4.5));

    f.destroy();
}

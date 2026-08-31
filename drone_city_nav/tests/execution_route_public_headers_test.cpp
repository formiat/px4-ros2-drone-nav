#include <gtest/gtest.h>

bool executionRouteModelHeaderIsSelfContained();
bool executionRouteCertificatesHeaderIsSelfContained();
bool executionPlanHeaderIsSelfContained();
bool executionRetentionHeaderIsSelfContained();
bool executionRouteCertificationHeaderIsSelfContained();
bool executionRouteTransitionsHeaderIsSelfContained();
bool executionRouteStoreHeaderIsSelfContained();
bool executionSupervisorHeaderIsSelfContained();
bool executionRouteCompatibilityHeadersCompile();

TEST(ExecutionRoutePublicHeaders, CompileAsIndependentTranslationUnits) {
  EXPECT_TRUE(executionRouteModelHeaderIsSelfContained());
  EXPECT_TRUE(executionRouteCertificatesHeaderIsSelfContained());
  EXPECT_TRUE(executionPlanHeaderIsSelfContained());
  EXPECT_TRUE(executionRetentionHeaderIsSelfContained());
  EXPECT_TRUE(executionRouteCertificationHeaderIsSelfContained());
  EXPECT_TRUE(executionRouteTransitionsHeaderIsSelfContained());
  EXPECT_TRUE(executionRouteStoreHeaderIsSelfContained());
  EXPECT_TRUE(executionSupervisorHeaderIsSelfContained());
  EXPECT_TRUE(executionRouteCompatibilityHeadersCompile());
}

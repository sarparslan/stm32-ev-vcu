#include "test_framework.h"

int g_checks = 0;
int g_failures = 0;

int main(void)
{
  printf("CAN codecs\n");
  run_can_codec_tests();

  printf("Vehicle control\n");
  run_vehicle_control_tests();

  printf("\n%d checks, %d failed\n", g_checks, g_failures);
  return (g_failures == 0) ? 0 : 1;
}

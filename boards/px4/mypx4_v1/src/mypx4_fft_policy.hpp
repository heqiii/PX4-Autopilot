// HE_PX4_v1.0.10 offline carbon-propeller candidate; not flight acceptance.
#pragma once
#include <cstdint>

namespace mypx4
{
constexpr float fft_notch_bandwidth(int32_t airframe, float original_bandwidth)
{
	return (airframe == 4001 || airframe == 4081 || airframe == 4082 || airframe == 4014)
	       ? 20.f : original_bandwidth;
}
} // namespace mypx4

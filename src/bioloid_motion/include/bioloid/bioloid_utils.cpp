/*******************************************************************************
* Copyright 2017 Mathieu Rondonneau
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
// Author: Mathieu Rondonneau
// Modified by: Etevaldo Cardoso
#include <cmath>
#include <bioloid/bioloid_utils.hpp>

joints_t interpolate(joints_t angles_a, joints_t angles_b, double coefa) {
	joints_t::iterator it;
	joints_t new_joints;

	ROS_ASSERT_MSG(angles_a.size() <= angles_b.size(), "%d > %d", angles_a.size(), angles_b.size());

	for (it = angles_a.begin(); it != angles_a.end(); ++it) {
		new_joints[it->first] = it->second * coefa + angles_b[it->first] * (1 - coefa);
    }
	return new_joints;
}

double getDistance(joints_t anglesa, joints_t anglesb) {
	joints_t::iterator it;
	double distance = 0;

	ROS_ASSERT_MSG(anglesa.size() <= anglesb.size(), "%d > %d", anglesa.size(), anglesb.size());

	for (it = anglesa.begin(); it != anglesa.end(); ++it) {
        distance += std::abs(anglesb[it->first] - it->second);
    }

    distance /= anglesb.size();

    return distance;
}


